# Solar Tracker — Three.js Web Dashboard

Browser-based digital twin (no matplotlib 3D). Firmware and `DATA,...` CSV format unchanged.

## Install

```bash
pip install -r backend/requirements.txt
```

## Run

1. Flash/upload `solar_tracking_integrated_test/solar_tracking_integrated_test.ino`.
2. From project root:

```bash
python backend/serial_server.py
```

3. Open in a browser:

```
http://127.0.0.1:8080/frontend/
```

Layout: **~33%** left = Three.js GLB (orbit/zoom); **~67%** right = angles, lux, V/I/P chart (Chart.js).

- **HTTP** static files: port `8080` (serves `frontend/` and `assets/`)
- **WebSocket** telemetry: `ws://127.0.0.1:8765`
- **Serial** default: `COM6` @ `115200` (edit `SERIAL_PORT` in `backend/serial_server.py`)

## Files

| Path | Role |
|------|------|
| `backend/serial_server.py` | Serial reader + WebSocket + HTTP static server |
| `frontend/index.html` | Dashboard layout |
| `frontend/main.js` | Three.js scene, GLB, lerp animation |
| `frontend/style.css` | Dark engineering UI |
| `assets/assembly_model_junxiangyang_edited.glb` | 3D model (loaded once) |

## 3D rig

- `yaw_group.rotation.z` ← `current_yaw` (relative to 90° center)
- `panel_group.rotation.x` ← `current_pitch` (relative to 90° center)
- Smooth lerp in the render loop (GLB not reloaded per frame)

## Notes

- Solar power panel shows live values only if the firmware prints `Solar V=...` debug lines; otherwise placeholder text is shown.
