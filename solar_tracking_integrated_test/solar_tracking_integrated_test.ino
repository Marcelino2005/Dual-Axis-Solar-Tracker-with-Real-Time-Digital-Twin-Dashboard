#define ARDUINO_USB_CDC_ON_BOOT 1

/**
 *
 *
 * Libs: arduino-cli lib install "BH1750" "ESP32Servo" "Adafruit INA3221 Library"
 *
 * cd C:\Users\PC\Desktop\solar_tracker\solar_tracking_integrated_test
 * arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc .
 * arduino-cli upload -p COM6 --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc .
 * arduino-cli monitor -p COM6 -c baudrate=115200 -c dtr=off -c rts=off
 */

#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>
#include <Adafruit_INA3221.h>
#include <cmath>

// =============================================================================
// ???: 4x GY-30 @ 0x23?? Wire ?? SDA/SCL
// =============================================================================
static BH1750 bh(0x23);
static const int kSda[4] = {5, 16, 18, 11};
static const int kScl[4] = {4, 15, 17, 10};
static const char* const kCornerName[4] = {"TL", "TR", "BL", "BR"};
static bool s_bh1750Configured[4] = {false, false, false, false};

// =============================================================================
// ??: pitch GPIO13, yaw GPIO14
// =============================================================================
static constexpr int kPinPitch = 13;
static constexpr int kPinYaw = 14;
static constexpr uint32_t kServoHz = 50;
static constexpr int kPulseMinUs = 500;
static constexpr int kPulseMaxUs = 2500;

static constexpr int kCenterDeg = 90;
static constexpr int kPitchMin = 70;
static constexpr int kPitchMax = 115;
static constexpr int kYawMin = 45;
static constexpr int kYawMax = 135;

static constexpr float kLuxEmaAlpha = 0.25f;
static constexpr float kDeadzoneLux = 30.0f;

// Velocity tracking (sensor logic unchanged; targets integrated on servo tick)
static constexpr float kVelErrorFullScaleLux = 180.0f;
static constexpr float kYawMaxVelDegPerSec = 32.0f;
static constexpr float kYawMinVelDegPerSec = 3.0f;
static constexpr float kPitchMaxVelDegPerSec = 22.0f;
static constexpr float kPitchMinVelDegPerSec = 2.0f;
static constexpr float kCurrentSmoothing = 0.18f;

static constexpr uint32_t kServoTickMs = 20;
static constexpr uint32_t kSensorPeriodMs = 300;

// INA3221 solar input CH2 — separate I2C bus (does not affect tracking)
static constexpr int kInaSda = 1;
static constexpr int kInaScl = 2;
static constexpr uint8_t kIna3221Addr = INA3221_DEFAULT_ADDRESS;
static constexpr float kShuntOhmsCh2 = 0.05f;
/** Hardware CH2 → Adafruit library channel 1 */
static constexpr uint8_t kChannelSolar = 1;
static constexpr uint32_t kInaPeriodMs = 1000;
static constexpr int kInaAvgSamples = 5;

static TwoWire WireIna = TwoWire(1);
static Adafruit_INA3221 ina3221;
static bool g_ina_ok = false;
static float g_inaHistV[kInaAvgSamples];
static float g_inaHistIa[kInaAvgSamples];
static int g_inaHistCount = 0;
static uint32_t g_lastInaMs = 0;

static bool invertYaw = true;
static bool invertPitch = true;

static Servo s_pitch;
static Servo s_yaw;

static float g_currentYaw = kCenterDeg;
static float g_currentPitch = kCenterDeg;
static float g_targetYaw = kCenterDeg;
static float g_targetPitch = kCenterDeg;

static float g_filtLux[4] = {0.f, 0.f, 0.f, 0.f};
static bool g_filtReady[4] = {false, false, false, false};

static float g_lastErrYaw = 0.f;
static float g_lastErrPitch = 0.f;
static float g_yawVelocityDegPerSec = 0.f;
static float g_pitchVelocityDegPerSec = 0.f;

static uint32_t g_lastServoMs = 0;
static uint32_t g_lastMotionIntegrateMs = 0;
static uint32_t g_lastSensorMs = 0;

static constexpr uint32_t kBusSettleMs = 20;
static constexpr uint32_t kBhInitWaitMs = 130;
static constexpr uint32_t kBhReadWaitMs = 40;

enum class SensorFsm : uint8_t {
  Idle,
  BusWait,
  InitWait,
  ReadWait,
};

static SensorFsm g_sensorFsm = SensorFsm::Idle;
static int g_sensorCorner = 0;
static uint32_t g_sensorWaitUntilMs = 0;
static bool g_sensorCycleOk = true;
static bool g_sensorPrintPending = false;

static void emaLux(int index, float raw);

static void inaPushSample(float v_volts, float i_amps) {
  if (g_inaHistCount < kInaAvgSamples) {
    g_inaHistV[g_inaHistCount] = v_volts;
    g_inaHistIa[g_inaHistCount] = i_amps;
    ++g_inaHistCount;
    return;
  }
  for (int i = 1; i < kInaAvgSamples; ++i) {
    g_inaHistV[i - 1] = g_inaHistV[i];
    g_inaHistIa[i - 1] = g_inaHistIa[i];
  }
  g_inaHistV[kInaAvgSamples - 1] = v_volts;
  g_inaHistIa[kInaAvgSamples - 1] = i_amps;
}

static bool inaMeanLast(float& v_mean, float& i_mean_a) {
  if (g_inaHistCount == 0) {
    return false;
  }
  float sv = 0.f;
  float si = 0.f;
  for (int i = 0; i < g_inaHistCount; ++i) {
    sv += g_inaHistV[i];
    si += g_inaHistIa[i];
  }
  const float inv = 1.0f / static_cast<float>(g_inaHistCount);
  v_mean = sv * inv;
  i_mean_a = si * inv;
  return std::isfinite(v_mean) && std::isfinite(i_mean_a);
}

/** Display-only INA read; format matches Python dashboard parser */
static void tickInaSolar(uint32_t now) {
  if (!g_ina_ok) {
    return;
  }
  if (now - g_lastInaMs < kInaPeriodMs) {
    return;
  }
  g_lastInaMs = now;

  const float v_bus = ina3221.getBusVoltage(kChannelSolar);
  const float i_a = ina3221.getCurrentAmps(kChannelSolar);
  if (!std::isfinite(v_bus) || !std::isfinite(i_a)) {
    return;
  }

  inaPushSample(v_bus, i_a);

  float v_avg = NAN;
  float i_avg_a = NAN;
  if (!inaMeanLast(v_avg, i_avg_a)) {
    return;
  }

  const float i_ma = i_avg_a * 1000.0f;
  const float p_mw = v_avg * i_avg_a * 1000.0f;

  Serial.print(F("Solar V="));
  Serial.print(v_avg, 2);
  Serial.print(F(" I="));
  Serial.print(i_ma, 1);
  Serial.print(F("mA P="));
  Serial.print(p_mw, 1);
  Serial.println(F("mW"));
}

// =============================================================================
// Relay + limit switches (motor_relay_limit_test module — runs in parallel)
// IN1/IN2 GPIO35/36; limits GPIO40/41 NC+pullup; FORWARD=both HIGH, REVERSE=both LOW
// =============================================================================
static constexpr int kPinRelayIn1 = 35;
static constexpr int kPinRelayIn2 = 36;
static constexpr int kPinLimitLeft = 40;
static constexpr int kPinLimitRight = 41;

static constexpr uint8_t kRelayOn = LOW;
static constexpr uint8_t kRelayOff = HIGH;
static constexpr int kLimitLevelOpen = LOW;
static constexpr int kLimitLevelPressed = HIGH;
static constexpr uint32_t kLimitDebounceMs = 300;
static constexpr uint32_t kRelayLimitPrintMs = 200;

static constexpr uint8_t kRelayDirFwd = 0;
static constexpr uint8_t kRelayDirRev = 1;

static uint8_t g_relayDir = kRelayDirFwd;
static bool g_relayReversedThisPrint = false;
static int g_limitLeftStable = kLimitLevelOpen;
static int g_limitRightStable = kLimitLevelOpen;
static int g_limitLeftLastRaw = kLimitLevelOpen;
static int g_limitRightLastRaw = kLimitLevelOpen;
static uint32_t g_limitLeftChangeMs = 0;
static uint32_t g_limitRightChangeMs = 0;
static int g_limitLeftPrevStable = kLimitLevelOpen;
static int g_limitRightPrevStable = kLimitLevelOpen;
static uint32_t g_relayLimitLastPrintMs = 0;

static void relayLimitBoth(uint8_t level) {
  digitalWrite(kPinRelayIn1, level);
  digitalWrite(kPinRelayIn2, level);
}

static void relayLimitInit() {
  pinMode(kPinRelayIn1, OUTPUT);
  pinMode(kPinRelayIn2, OUTPUT);
  pinMode(kPinLimitLeft, INPUT_PULLUP);
  pinMode(kPinLimitRight, INPUT_PULLUP);

  const uint32_t t0 = millis();
  g_limitLeftChangeMs = t0;
  g_limitRightChangeMs = t0;
  g_limitLeftLastRaw = digitalRead(kPinLimitLeft);
  g_limitRightLastRaw = digitalRead(kPinLimitRight);
  g_limitLeftStable = g_limitLeftLastRaw;
  g_limitRightStable = g_limitRightLastRaw;
  g_limitLeftPrevStable = g_limitLeftStable;
  g_limitRightPrevStable = g_limitRightStable;
}

static const char* relayLimitDirText(uint8_t dir) {
  return (dir == kRelayDirFwd) ? "FORWARD" : "REVERSE";
}

static const char* relayLimitRawLabel(int level) {
  return (level == kLimitLevelPressed) ? "HIGH (pressed)" : "LOW (open)";
}

static void relayLimitMotorForward() {
  relayLimitBoth(kRelayOff);
  g_relayDir = kRelayDirFwd;
}

static void relayLimitMotorReverse() {
  relayLimitBoth(kRelayOn);
  g_relayDir = kRelayDirRev;
}

static void relayLimitToggleDirection() {
  if (g_relayDir == kRelayDirFwd) {
    relayLimitMotorReverse();
  } else {
    relayLimitMotorForward();
  }
  g_relayReversedThisPrint = true;
  Serial.print(F("[RELAY] Reversed -> "));
  Serial.println(relayLimitDirText(g_relayDir));
}

static void relayLimitPrintInitial() {
  Serial.print(F("Initial LEFT  = "));
  Serial.println(relayLimitRawLabel(g_limitLeftStable));
  Serial.print(F("Initial RIGHT = "));
  Serial.println(relayLimitRawLabel(g_limitRightStable));
  if (g_limitLeftStable == kLimitLevelPressed) {
    Serial.println(F("[WARN] Limit switch is HIGH at boot. Check wiring or switch position."));
    Serial.println(F("       (LEFT / GPIO40)"));
  }
  if (g_limitRightStable == kLimitLevelPressed) {
    Serial.println(F("[WARN] Limit switch is HIGH at boot. Check wiring or switch position."));
    Serial.println(F("       (RIGHT / GPIO41)"));
  }
}

static void relayLimitDebouncePin(int pin, int& stable, int& lastRaw, uint32_t& changeMs) {
  const int raw = digitalRead(pin);
  const uint32_t now = millis();
  if (raw != lastRaw) {
    lastRaw = raw;
    changeMs = now;
  } else if ((now - changeMs) >= kLimitDebounceMs) {
    stable = raw;
  }
}

static bool relayLimitConsumePressEdge(int pin, int& stable, int& lastRaw, uint32_t& changeMs,
                                       int& prevStable) {
  relayLimitDebouncePin(pin, stable, lastRaw, changeMs);
  const bool edge = (prevStable == kLimitLevelOpen && stable == kLimitLevelPressed);
  prevStable = stable;
  return edge;
}

static void relayLimitCheckEdges() {
  bool pressed = false;
  if (relayLimitConsumePressEdge(kPinLimitLeft, g_limitLeftStable, g_limitLeftLastRaw,
                                 g_limitLeftChangeMs, g_limitLeftPrevStable)) {
    pressed = true;
  }
  if (relayLimitConsumePressEdge(kPinLimitRight, g_limitRightStable, g_limitRightLastRaw,
                                 g_limitRightChangeMs, g_limitRightPrevStable)) {
    pressed = true;
  }
  if (pressed) {
    relayLimitToggleDirection();
  }
}

static void relayLimitPrintStatus(uint32_t now) {
  if ((now - g_relayLimitLastPrintMs) < kRelayLimitPrintMs) {
    return;
  }
  g_relayLimitLastPrintMs = now;

  Serial.println(F("--- relay/limit ---"));
  Serial.print(F("relay direction : "));
  Serial.println(relayLimitDirText(g_relayDir));
  Serial.print(F("LEFT RAW        : "));
  Serial.println(relayLimitRawLabel(digitalRead(kPinLimitLeft)));
  Serial.print(F("RIGHT RAW       : "));
  Serial.println(relayLimitRawLabel(digitalRead(kPinLimitRight)));
  Serial.print(F("reversed event  : "));
  Serial.println(g_relayReversedThisPrint ? F("YES") : F("no"));
  g_relayReversedThisPrint = false;
}

static void tickRelayLimit(uint32_t now) {
  relayLimitCheckEdges();
  relayLimitPrintStatus(now);
}

// =============================================================================
// ?????
// =============================================================================
static void busToCorner(int idx) {
  Wire.end();
  Wire.begin(kSda[idx], kScl[idx]);
}

/** Setup only — blocking read with original settle/integration delays */
static bool readBH1750OnBusBlocking(int index, float& luxOut) {
  luxOut = NAN;
  if (index < 0 || index > 3) {
    return false;
  }

  busToCorner(index);
  delay(kBusSettleMs);

  if (!s_bh1750Configured[index]) {
    if (!bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
      return false;
    }
    delay(kBhInitWaitMs);
    s_bh1750Configured[index] = true;
  } else {
    delay(kBhReadWaitMs);
  }

  const float x = bh.readLightLevel();
  if (!std::isfinite(x) || x < 0.0f) {
    luxOut = NAN;
    s_bh1750Configured[index] = false;
    return false;
  }
  luxOut = x;
  return true;
}

static void sensorAdvanceCornerOrFinish(uint32_t now) {
  ++g_sensorCorner;
  if (g_sensorCorner >= 4) {
    g_sensorFsm = SensorFsm::Idle;
    g_lastSensorMs = now;
    g_sensorPrintPending = true;
    return;
  }
  busToCorner(g_sensorCorner);
  g_sensorWaitUntilMs = now + kBusSettleMs;
  g_sensorFsm = SensorFsm::BusWait;
}

static void sensorReadCorner(uint32_t now) {
  const float x = bh.readLightLevel();
  if (!std::isfinite(x) || x < 0.0f) {
    Serial.print(F("SENSOR FAIL: "));
    Serial.println(kCornerName[g_sensorCorner]);
    s_bh1750Configured[g_sensorCorner] = false;
    g_sensorCycleOk = false;
  } else {
    emaLux(g_sensorCorner, x);
  }
  sensorAdvanceCornerOrFinish(now);
}

static void sensorStartCycle(uint32_t now) {
  g_sensorCorner = 0;
  g_sensorCycleOk = true;
  busToCorner(0);
  g_sensorWaitUntilMs = now + kBusSettleMs;
  g_sensorFsm = SensorFsm::BusWait;
}

/** Non-blocking quad scan; servos keep ticking while waiting between corners */
static void tickSensorAsync(uint32_t now) {
  if (g_sensorFsm == SensorFsm::Idle) {
    if (now - g_lastSensorMs >= kSensorPeriodMs) {
      sensorStartCycle(now);
    }
    return;
  }

  if (now < g_sensorWaitUntilMs) {
    return;
  }

  switch (g_sensorFsm) {
  case SensorFsm::BusWait: {
    const int i = g_sensorCorner;
    if (!s_bh1750Configured[i]) {
      if (!bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
        Serial.print(F("SENSOR FAIL: "));
        Serial.println(kCornerName[i]);
        s_bh1750Configured[i] = false;
        g_sensorCycleOk = false;
        sensorAdvanceCornerOrFinish(now);
        break;
      }
      s_bh1750Configured[i] = true;
      g_sensorWaitUntilMs = now + kBhInitWaitMs;
      g_sensorFsm = SensorFsm::InitWait;
    } else {
      g_sensorWaitUntilMs = now + kBhReadWaitMs;
      g_sensorFsm = SensorFsm::ReadWait;
    }
    break;
  }
  case SensorFsm::InitWait:
    sensorReadCorner(now);
    break;
  case SensorFsm::ReadWait:
    sensorReadCorner(now);
    break;
  default:
    g_sensorFsm = SensorFsm::Idle;
    break;
  }
}

static void emaLux(int index, float raw) {
  if (!std::isfinite(raw) || raw < 0.0f) {
    return;
  }
  if (!g_filtReady[index]) {
    g_filtLux[index] = raw;
    g_filtReady[index] = true;
    return;
  }
  g_filtLux[index] = kLuxEmaAlpha * raw + (1.0f - kLuxEmaAlpha) * g_filtLux[index];
}

static bool filteredLuxReady() {
  for (int i = 0; i < 4; i++) {
    if (!g_filtReady[i] || !std::isfinite(g_filtLux[i])) {
      return false;
    }
  }
  return true;
}

// =============================================================================
// ??
// =============================================================================
static float clampPitch(float a) {
  return constrain(a, static_cast<float>(kPitchMin), static_cast<float>(kPitchMax));
}

static float clampYaw(float a) {
  return constrain(a, static_cast<float>(kYawMin), static_cast<float>(kYawMax));
}

static void servosWriteCurrent() {
  s_pitch.write(static_cast<int>(g_currentPitch + 0.5f));
  s_yaw.write(static_cast<int>(g_currentYaw + 0.5f));
}

static void servosToCenter() {
  g_currentPitch = g_targetPitch = kCenterDeg;
  g_currentYaw = g_targetYaw = kCenterDeg;
  g_yawVelocityDegPerSec = 0.f;
  g_pitchVelocityDegPerSec = 0.f;
  servosWriteCurrent();
  Serial.println(F("Servos centered at 90"));
}

static float errorToVelocityDegPerSec(float err, float maxVel, float minVel) {
  const float errAbs = fabsf(err);
  if (errAbs <= kDeadzoneLux) {
    return 0.0f;
  }
  const float excess = errAbs - kDeadzoneLux;
  const float t = constrain(excess / kVelErrorFullScaleLux, 0.0f, 1.0f);
  const float speed = minVel + t * (maxVel - minVel);
  return (err > 0.0f) ? speed : -speed;
}

static void tickServoMotion(uint32_t now) {
  if (g_lastMotionIntegrateMs == 0) {
    g_lastMotionIntegrateMs = now;
  }
  float dt = (now - g_lastMotionIntegrateMs) * 0.001f;
  g_lastMotionIntegrateMs = now;
  if (dt <= 0.0f) {
    return;
  }
  if (dt > 0.08f) {
    dt = kServoTickMs * 0.001f;
  }

  g_targetYaw += g_yawVelocityDegPerSec * dt;
  g_targetPitch += g_pitchVelocityDegPerSec * dt;
  g_targetYaw = clampYaw(g_targetYaw);
  g_targetPitch = clampPitch(g_targetPitch);

  g_currentYaw += (g_targetYaw - g_currentYaw) * kCurrentSmoothing;
  g_currentPitch += (g_targetPitch - g_currentPitch) * kCurrentSmoothing;
  g_currentYaw = clampYaw(g_currentYaw);
  g_currentPitch = clampPitch(g_currentPitch);
  servosWriteCurrent();
}

/**
 * errYaw   = (TR+BR)-(TL+BL)  ????
 * errPitch = (BL+BR)-(TL+TR)  ????
 */
static void updateTrackingTargets() {
  const float tl = g_filtLux[0];
  const float tr = g_filtLux[1];
  const float bl = g_filtLux[2];
  const float br = g_filtLux[3];

  g_lastErrYaw = (tr + br) - (tl + bl);
  g_lastErrPitch = (bl + br) - (tl + tr);

  float yawVel = errorToVelocityDegPerSec(g_lastErrYaw, kYawMaxVelDegPerSec, kYawMinVelDegPerSec);
  float pitchVel =
      errorToVelocityDegPerSec(g_lastErrPitch, kPitchMaxVelDegPerSec, kPitchMinVelDegPerSec);
  if (invertYaw) {
    yawVel = -yawVel;
  }
  if (invertPitch) {
    pitchVel = -pitchVel;
  }
  g_yawVelocityDegPerSec = yawVel;
  g_pitchVelocityDegPerSec = pitchVel;
}

static void printCsvTelemetry(uint32_t ms, bool tracking_ok) {
  const char* action;
  if (!tracking_ok) {
    action = "sensor_fault";
  } else if (fabsf(g_lastErrYaw) > kDeadzoneLux || fabsf(g_lastErrPitch) > kDeadzoneLux) {
    action = "tracking";
  } else {
    action = "hold";
  }

  Serial.print(F("DATA,"));
  Serial.print(ms);
  Serial.print(',');
  Serial.print(g_filtLux[0], 1);
  Serial.print(',');
  Serial.print(g_filtLux[1], 1);
  Serial.print(',');
  Serial.print(g_filtLux[2], 1);
  Serial.print(',');
  Serial.print(g_filtLux[3], 1);
  Serial.print(',');
  Serial.print(g_lastErrYaw, 1);
  Serial.print(',');
  Serial.print(g_lastErrPitch, 1);
  Serial.print(',');
  Serial.print(g_targetYaw, 1);
  Serial.print(',');
  Serial.print(g_currentYaw, 1);
  Serial.print(',');
  Serial.print(g_targetPitch, 1);
  Serial.print(',');
  Serial.print(g_currentPitch, 1);
  Serial.print(',');
  Serial.println(action);
}

static void printDebug(uint32_t ms, bool tracking_ok) {
  Serial.print(F("TL="));
  Serial.print(g_filtLux[0], 1);
  Serial.print(F(" TR="));
  Serial.print(g_filtLux[1], 1);
  Serial.print(F(" BL="));
  Serial.print(g_filtLux[2], 1);
  Serial.print(F(" BR="));
  Serial.println(g_filtLux[3], 1);

  if (!tracking_ok) {
    Serial.println(F("errYaw=skip errPitch=skip"));
    Serial.print(F("targetYaw="));
    Serial.print(g_targetYaw, 1);
    Serial.print(F(" currentYaw="));
    Serial.print(g_currentYaw, 1);
    Serial.print(F(" targetPitch="));
    Serial.print(g_targetPitch, 1);
    Serial.print(F(" currentPitch="));
    Serial.println(g_currentPitch, 1);
    Serial.println(F("action=hold (sensor fault)"));
  } else {
    Serial.print(F("errYaw="));
    Serial.print(g_lastErrYaw, 1);
    Serial.print(F(" errPitch="));
    Serial.println(g_lastErrPitch, 1);

    Serial.print(F("targetYaw="));
    Serial.print(g_targetYaw, 1);
    Serial.print(F(" currentYaw="));
    Serial.print(g_currentYaw, 1);
    Serial.print(F(" targetPitch="));
    Serial.print(g_targetPitch, 1);
    Serial.print(F(" currentPitch="));
    Serial.println(g_currentPitch, 1);

    const bool tracking =
        (fabsf(g_lastErrYaw) > kDeadzoneLux) || (fabsf(g_lastErrPitch) > kDeadzoneLux);
    Serial.println(tracking ? F("action=tracking") : F("action=hold"));
  }

  printCsvTelemetry(ms, tracking_ok);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  s_pitch.setPeriodHertz(kServoHz);
  s_yaw.setPeriodHertz(kServoHz);
  s_pitch.attach(kPinPitch, kPulseMinUs, kPulseMaxUs);
  s_yaw.attach(kPinYaw, kPulseMinUs, kPulseMaxUs);

  Serial.println(F("BOOT baseline: BH1750 x4 + dual servos + INA3221 CH2 + relay/limit"));

  relayLimitInit();
  relayLimitPrintInitial();
  relayLimitMotorForward();

  WireIna.begin(kInaSda, kInaScl);
  g_ina_ok = ina3221.begin(kIna3221Addr, &WireIna);
  if (g_ina_ok) {
    ina3221.setShuntResistance(kChannelSolar, kShuntOhmsCh2);
    Serial.println(F("INA3221 CH2 OK (SDA=GPIO1 SCL=GPIO2)"));
  } else {
    Serial.println(F("INA3221 begin FAIL (check wiring / 0x40)"));
  }

  servosToCenter();

  const uint32_t t0 = millis();
  g_lastServoMs = t0;
  g_lastSensorMs = t0;
  g_lastMotionIntegrateMs = t0;

  Serial.println(F("Sensor init:"));
  for (int i = 0; i < 4; i++) {
    float lx = NAN;
    const bool ok = readBH1750OnBusBlocking(i, lx);
    Serial.print(kCornerName[i]);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
    if (ok) {
      emaLux(i, lx);
    }
  }

  Serial.println(F("--- baseline tracking loop ---"));
}

void loop() {
  const uint32_t now = millis();

  if (now - g_lastServoMs >= kServoTickMs) {
    g_lastServoMs = now;
    tickServoMotion(now);
  }

  tickSensorAsync(now);
  tickInaSolar(now);
  tickRelayLimit(now);

  if (g_sensorPrintPending) {
    g_sensorPrintPending = false;
    const bool tracking_ok = g_sensorCycleOk && filteredLuxReady();
    if (tracking_ok) {
      updateTrackingTargets();
    } else {
      g_yawVelocityDegPerSec = 0.f;
      g_pitchVelocityDegPerSec = 0.f;
    }
    printDebug(now, tracking_ok);
  }
}
