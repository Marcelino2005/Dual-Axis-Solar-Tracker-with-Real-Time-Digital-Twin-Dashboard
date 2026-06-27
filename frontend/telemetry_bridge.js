/**
 * WebSocket + telemetry UI (no ES modules — runs even if Three.js CDN fails).
 * Load before main.js in index.html.
 */
(function () {
  const WS_URL = "ws://127.0.0.1:8765";

  function bindUi() {
    return {
      connStatus: document.getElementById("conn-status"),
      cyaw: document.getElementById("val-cyaw"),
      tyaw: document.getElementById("val-tyaw"),
      cpitch: document.getElementById("val-cpitch"),
      tpitch: document.getElementById("val-tpitch"),
      erryaw: document.getElementById("val-erryaw"),
      errpitch: document.getElementById("val-errpitch"),
      action: document.getElementById("action-badge"),
      solar: document.getElementById("solar-instant"),
      ms: document.getElementById("val-ms"),
      count: document.getElementById("val-count"),
    };
  }

  let ui = bindUi();
  if (!ui.connStatus) {
    console.error("[telemetry] DOM not ready — place telemetry_bridge.js before </body>");
    return;
  }

  function fmt(v, digits) {
    if (v === null || v === undefined || Number.isNaN(Number(v))) return "—";
    return Number(v).toFixed(digits);
  }

  function setStatus(wsOk, serialOk, d) {
    if (!ui.connStatus) return;
    if (!wsOk) {
      ui.connStatus.textContent = "WS: disconnected — start python backend/serial_server.py";
      ui.connStatus.className = "disconnected";
      return;
    }
    const n = Number(d?.parse_count ?? 0);
    const port = d?.serial_port ? String(d.serial_port) : "";
    const portLabel = port ? ` (${port})` : "";
    if (serialOk && n > 0) {
      ui.connStatus.textContent = `WS: ok · Serial: live${portLabel} · samples ${n}`;
      ui.connStatus.className = "connected";
    } else if (serialOk) {
      ui.connStatus.textContent = `WS: ok · Serial: open${portLabel} — waiting for DATA,...`;
      ui.connStatus.className = "disconnected";
    } else {
      ui.connStatus.textContent = "WS: ok · Serial: scanning / not connected";
      ui.connStatus.className = "disconnected";
    }
  }

  function updatePanels(d) {
    if (ui.cyaw) ui.cyaw.textContent = fmt(d.current_yaw);
    if (ui.tyaw) ui.tyaw.textContent = fmt(d.target_yaw);
    if (ui.cpitch) ui.cpitch.textContent = fmt(d.current_pitch);
    if (ui.tpitch) ui.tpitch.textContent = fmt(d.target_pitch);
    if (ui.erryaw) ui.erryaw.textContent = fmt(d.err_yaw);
    if (ui.errpitch) ui.errpitch.textContent = fmt(d.err_pitch);
    if (ui.ms) ui.ms.textContent = fmt(d.ms, 0);
    if (ui.count) ui.count.textContent = String(d.parse_count ?? 0);

    if (ui.action) {
      const action = d.action || "—";
      ui.action.textContent = action;
      ui.action.className = action.replace(/\s+/g, "_");
    }

    if (ui.solar) {
      if (d.solar_voltage != null && d.solar_current_ma != null && d.solar_power_mw != null) {
        ui.solar.textContent = `${fmt(d.solar_voltage, 2)} V · ${fmt(d.solar_current_ma, 1)} mA · ${fmt(
          d.solar_power_mw,
          1
        )} mW`;
      } else {
        ui.solar.textContent = "— V · — mA · — mW（当前固件未输出 INA3221 数据）";
      }
    }

  }

  window.applySolarTelemetry = function (d) {
    window.__latestTelemetry = d;
    updatePanels(d);
    setStatus(true, Boolean(d.connected), d);
    if (typeof window.pushLuxChartSample === "function") {
      window.pushLuxChartSample(d);
    }
    if (typeof window.pushPowerChartSample === "function") {
      window.pushPowerChartSample(d);
    }
    window.dispatchEvent(new CustomEvent("solar-telemetry", { detail: d }));
  };

  function connectWebSocket() {
    setStatus(false, false, null);
    let ws;
    try {
      ws = new WebSocket(WS_URL);
    } catch (e) {
      console.error("[ws] create failed", e);
      setStatus(false, false, null);
      setTimeout(connectWebSocket, 2000);
      return;
    }

    ws.onopen = () => {
      console.log("[ws] connected", WS_URL);
      setStatus(true, false, null);
    };

    ws.onmessage = (ev) => {
      try {
        const d = JSON.parse(ev.data);
        window.applySolarTelemetry(d);
      } catch (e) {
        console.warn("[ws] bad JSON", e);
      }
    };

    ws.onclose = () => {
      console.warn("[ws] closed — retry in 2s");
      setStatus(false, false, null);
      setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = () => {
      console.warn("[ws] error");
      setStatus(false, false, null);
    };
  }

  connectWebSocket();
})();
