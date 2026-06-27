/**
 * Solar tracker Three.js digital twin (visualization only).
 * GLB loaded once; only yaw_group / panel_group rotations update each frame.
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const GLB_URL = "/assets/assembly_model_junxiangyang_edited.glb";

const YAW_CENTER = 90;
const PITCH_CENTER = 90;
// Yaw visual calibration offset = -90 degrees. (3D model only)
const YAW_VISUAL_OFFSET_DEG = -90;
const YAW_VISUAL_OFFSET_RAD = -Math.PI / 2;
const ROT_LERP = 0.12;
const YAW_FINAL_LOG_MS = 2000;
const POWER_CHART_MAX = 150;
const LUX_CHART_MAX = 150;

/** Blender yaw is Z; glTF/Three.js rig uses Y-up — drive yaw on Y, not Z */
const YAW_AXIS = "y";
const PITCH_AXIS = "x";
/** Match physical tracker direction (invert if motion looks reversed) */
const YAW_SIGN = 1;
const PITCH_SIGN = -1;

const canvas = document.getElementById("canvas3d");
const modelHint = document.getElementById("model-hint");

let modelRoot = null;
let yawGroup = null;
/** REAL yaw visual rotation target — only this node’s rotation.y/z drives visible yaw */
let yawVisualTarget = null;
let panelGroup = null;
let rigOk = false;

let targetYawRad = 0;
let targetPitchRad = 0;
let displayYawRad = 0;
let displayPitchRad = 0;
let rawYawDeg = YAW_CENTER;
let lastYawFinalLogMs = 0;

let latestTelemetry = null;
let powerChart = null;
let luxChart = null;
let luxChartLastCount = 0;
let powerChartLastKey = "";

function degToRadRel(deg, center) {
  return ((Number(deg) - center) * Math.PI) / 180;
}

function findNode(root, name) {
  if (!root) return null;
  let found = root.getObjectByName(name);
  if (found) return found;
  root.traverse((obj) => {
    if (!found && obj.name === name) found = obj;
  });
  return found;
}

function listNodeNames(root, limit = 40) {
  const names = [];
  root.traverse((obj) => {
    if (obj.name) names.push(obj.name);
  });
  return names.slice(0, limit);
}

/** Frame camera from model bounds — avoids near/far clipping the GLB */
function fitCameraToModel(root) {
  const box = new THREE.Box3().setFromObject(root);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 0.001);

  camera.near = Math.max(maxDim * 0.002, 0.01);
  camera.far = Math.max(maxDim * 250, 500);
  camera.updateProjectionMatrix();

  controls.target.copy(center);
  const dist = maxDim * 2.4;
  camera.position.set(
    center.x + dist * 0.85,
    center.y + dist * 0.55,
    center.z + dist * 0.85
  );
  controls.minDistance = maxDim * 0.15;
  controls.maxDistance = maxDim * 20;
  controls.update();
}

function applyModelPose(dynamicYawRad, pitchRad) {
  if (yawVisualTarget) {
    const dynamicYawSigned = YAW_SIGN * dynamicYawRad;
    const yawBase = dynamicYawSigned;
    const now = performance.now();
    const shouldLog = now - lastYawFinalLogMs >= YAW_FINAL_LOG_MS;

    if (shouldLog) {
      lastYawFinalLogMs = now;
      console.log("yawFromSerial", rawYawDeg);
      console.log("targetYawRad", targetYawRad);
    }

    yawVisualTarget.rotation.z = 0;
    // REAL yaw visual rotation target
    // Yaw visual calibration offset = -90 degrees.
    if (YAW_AXIS === "y") {
      yawVisualTarget.rotation.y = yawBase + YAW_VISUAL_OFFSET_RAD;
    } else {
      yawVisualTarget.rotation.z = yawBase + YAW_VISUAL_OFFSET_RAD;
    }

    if (shouldLog) {
      console.log("final object rotation", {
        name: yawVisualTarget.name,
        rotationY: yawVisualTarget.rotation.y,
        rotationZ: yawVisualTarget.rotation.z,
      });
    }
  }

  if (!rigOk) return;
  panelGroup.rotation[PITCH_AXIS] = PITCH_SIGN * pitchRad;
}

// --- Three.js scene ---
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.setClearColor(0x000000, 0);

const scene = new THREE.Scene();
scene.background = null;

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 2000);
camera.position.set(2.8, 1.8, 2.8);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0.6, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1);
key.position.set(4, 8, 6);
scene.add(key);
const fill = new THREE.DirectionalLight(0x58a6ff, 0.35);
fill.position.set(-5, 2, -3);
scene.add(fill);

const grid = new THREE.GridHelper(6, 24, 0x30363d, 0x21262d);
grid.position.y = -0.01;
scene.add(grid);

const chartGridColor = "#30363d";
const chartTextColor = "#8b949e";

function initLuxChart() {
  const canvasEl = document.getElementById("lux-chart");
  if (!canvasEl || typeof Chart === "undefined") {
    console.warn("[chart] lux chart: Chart.js not loaded");
    return;
  }

  luxChart = new Chart(canvasEl, {
    type: "line",
    data: {
      labels: [],
      datasets: [
        {
          label: "TL",
          data: [],
          borderColor: "#e74c3c",
          backgroundColor: "rgba(231, 76, 60, 0.08)",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
        {
          label: "TR",
          data: [],
          borderColor: "#3498db",
          backgroundColor: "rgba(52, 152, 219, 0.08)",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
        {
          label: "BL",
          data: [],
          borderColor: "#2ecc71",
          backgroundColor: "rgba(46, 204, 113, 0.08)",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
        {
          label: "BR",
          data: [],
          borderColor: "#9b59b6",
          backgroundColor: "rgba(155, 89, 182, 0.08)",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: "index", intersect: false },
      plugins: {
        legend: {
          labels: { color: chartTextColor, boxWidth: 12, font: { size: 10 } },
        },
      },
      scales: {
        x: {
          ticks: { color: chartTextColor, maxTicksLimit: 6, font: { size: 9 } },
          grid: { color: chartGridColor },
        },
        y: {
          title: { display: true, text: "lux", color: chartTextColor, font: { size: 10 } },
          ticks: { color: chartTextColor, font: { size: 9 } },
          grid: { color: "rgba(48, 54, 61, 0.5)" },
        },
      },
    },
  });
}

function pushLuxChartSample(d) {
  if (!luxChart) return;
  const n = Number(d.parse_count ?? 0);
  if (n <= luxChartLastCount) return;
  luxChartLastCount = n;

  const tl = Number(d.tl);
  const tr = Number(d.tr);
  const bl = Number(d.bl);
  const br = Number(d.br);
  if (![tl, tr, bl, br].every((v) => Number.isFinite(v))) return;

  const label = `${Math.round(d.ms ?? 0)}`;
  const { labels, datasets } = luxChart.data;
  labels.push(label);
  datasets[0].data.push(tl);
  datasets[1].data.push(tr);
  datasets[2].data.push(bl);
  datasets[3].data.push(br);

  while (labels.length > LUX_CHART_MAX) {
    labels.shift();
    datasets.forEach((ds) => ds.data.shift());
  }

  luxChart.update("none");
}

function initPowerChart() {
  const canvasEl = document.getElementById("power-chart");
  if (!canvasEl || typeof Chart === "undefined") {
    console.warn("[chart] Chart.js not loaded");
    return;
  }

  powerChart = new Chart(canvasEl, {
    type: "line",
    data: {
      labels: [],
      datasets: [
        {
          label: "Voltage (V)",
          data: [],
          borderColor: "#58a6ff",
          backgroundColor: "rgba(88, 166, 255, 0.08)",
          yAxisID: "y",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
        {
          label: "Current (mA)",
          data: [],
          borderColor: "#ffa657",
          backgroundColor: "rgba(255, 166, 87, 0.06)",
          yAxisID: "y1",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
        {
          label: "Power (mW)",
          data: [],
          borderColor: "#3fb950",
          backgroundColor: "rgba(63, 185, 80, 0.06)",
          yAxisID: "y2",
          tension: 0.25,
          pointRadius: 0,
          borderWidth: 1.5,
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: "index", intersect: false },
      plugins: {
        legend: {
          labels: { color: chartTextColor, boxWidth: 12, font: { size: 10 } },
        },
      },
      scales: {
        x: {
          ticks: { color: chartTextColor, maxTicksLimit: 8, font: { size: 9 } },
          grid: { color: chartGridColor },
        },
        y: {
          type: "linear",
          position: "left",
          title: { display: true, text: "V", color: "#58a6ff", font: { size: 10 } },
          ticks: { color: "#58a6ff", font: { size: 9 } },
          grid: { color: "rgba(48, 54, 61, 0.5)" },
        },
        y1: {
          type: "linear",
          position: "right",
          title: { display: true, text: "mA", color: "#ffa657", font: { size: 10 } },
          ticks: { color: "#ffa657", font: { size: 9 } },
          grid: { drawOnChartArea: false },
        },
        y2: {
          type: "linear",
          position: "right",
          offset: true,
          title: { display: true, text: "mW", color: "#3fb950", font: { size: 10 } },
          ticks: { color: "#3fb950", font: { size: 9 } },
          grid: { drawOnChartArea: false },
        },
      },
    },
  });
}

function pushPowerChartSample(d) {
  if (!powerChart) return;

  const hasSolar =
    d.solar_voltage != null && d.solar_current_ma != null && d.solar_power_mw != null;
  if (!hasSolar) return;

  const key = `${d.solar_voltage}|${d.solar_current_ma}|${d.solar_power_mw}`;
  if (key === powerChartLastKey) return;
  powerChartLastKey = key;

  const label = `${Math.round(d.ms ?? 0)}`;
  const { labels, datasets } = powerChart.data;

  labels.push(label);
  datasets[0].data.push(Number(d.solar_voltage));
  datasets[1].data.push(Number(d.solar_current_ma));
  datasets[2].data.push(Number(d.solar_power_mw));

  while (labels.length > POWER_CHART_MAX) {
    labels.shift();
    datasets.forEach((ds) => ds.data.shift());
  }

  powerChart.update("none");
}

function setupCharts() {
  if (typeof Chart === "undefined") {
    console.error("[chart] Chart.js missing — check /frontend/vendor/chart.umd.min.js");
    return;
  }
  try {
    initLuxChart();
    initPowerChart();
  } catch (e) {
    console.error("[chart] init failed", e);
    return;
  }
  window.pushLuxChartSample = pushLuxChartSample;
  window.pushPowerChartSample = pushPowerChartSample;

  requestAnimationFrame(() => {
    requestAnimationFrame(() => {
      if (luxChart) luxChart.resize();
      if (powerChart) powerChart.resize();
      const latest = window.__latestTelemetry;
      if (latest) {
        luxChartLastCount = 0;
        powerChartLastKey = "";
        pushLuxChartSample(latest);
        pushPowerChartSample(latest);
      }
      window.dispatchEvent(new Event("charts-ready"));
    });
  });
}
setupCharts();

function resize() {
  const parent = canvas.parentElement;
  const w = parent.clientWidth;
  const h = parent.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  if (luxChart) luxChart.resize();
  if (powerChart) powerChart.resize();
}

window.addEventListener("resize", resize);
resize();

const loader = new GLTFLoader();
console.log("[3d] Loading GLB:", GLB_URL);

loader.load(
  GLB_URL,
  (gltf) => {
    const root = gltf.scene;
    modelRoot = root;
    scene.add(root);

    yawGroup = findNode(root, "yaw_group");
    yawVisualTarget = yawGroup;
    panelGroup = findNode(root, "panel_group");
    rigOk = Boolean(yawVisualTarget && panelGroup);

    fitCameraToModel(root);

    const names = listNodeNames(root, 60);
    console.log("[3d] GLB loaded. Sample nodes:", names);

    if (rigOk) {
      modelHint.textContent =
        "GLB loaded · yaw→Y (Blender Z) · pitch→X · drag to orbit";
    } else {
      modelHint.textContent =
        `GLB loaded · rig missing (yaw=${yawGroup ? "ok" : "—"}, panel=${
          panelGroup ? "ok" : "—"
        }) · static model`;
      console.warn("[3d] yaw_group or panel_group not found — no joint animation");
    }
  },
  (progress) => {
    if (progress.total) {
      const pct = Math.round((progress.loaded / progress.total) * 100);
      modelHint.textContent = `Loading GLB… ${pct}%`;
    }
  },
  (err) => {
    console.error("[3d] GLB load error:", err);
    modelHint.textContent = `GLB load failed: ${err.message}`;
  }
);

function applyTelemetryFromBridge(d) {
  latestTelemetry = d;
  rawYawDeg = Number(d.current_yaw ?? YAW_CENTER);
  targetYawRad = degToRadRel(rawYawDeg, YAW_CENTER);
  targetPitchRad = degToRadRel(d.current_pitch ?? PITCH_CENTER, PITCH_CENTER);
}

window.addEventListener("solar-telemetry", (ev) => {
  applyTelemetryFromBridge(ev.detail);
});

// --- Render loop (lerp rotations only; GLB not reloaded) ---
function animate() {
  requestAnimationFrame(animate);

  displayYawRad += (targetYawRad - displayYawRad) * ROT_LERP;
  displayPitchRad += (targetPitchRad - displayPitchRad) * ROT_LERP;

  applyModelPose(displayYawRad, displayPitchRad);

  controls.update();
  renderer.render(scene, camera);
}

animate();
