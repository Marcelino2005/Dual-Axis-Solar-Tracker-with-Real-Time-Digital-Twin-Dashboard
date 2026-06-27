# solar_tracker

Light tracking on an ESP32-S3; view angles and charts in a browser on your PC.

## How to run

Flash `solar_tracking_integrated_test/solar_tracking_integrated_test.ino`, then:

```
pip install -r backend/requirements.txt
python backend/serial_server.py
```

Open http://127.0.0.1:8080/frontend/ in a browser. COM port and other setup notes are in `WEB_DASHBOARD.md`.





## What’s in this file

**solar_tracking_integrated_test/**  
- `solar_tracking_integrated_test.ino` — firmware on the board. Four BH1750s, two servos, INA3221. Prints `DATA,...` and `Solar V=...` lines over serial.

**backend/**  
- `serial_server.py` — reads serial, pushes data to the page, serves frontend + models on port 8080.  
- `requirements.txt` — pyserial and websockets.

**frontend/**  
- `index.html` — page layout.  
- `main.js` — 3D model and charts.  
- `telemetry_bridge.js` — WebSocket hookup, fills in the numbers.  
- `style.css` — styling.  
- `vendor/` — bundled Three.js and Chart.js (works offline).  
- `chart.js-4.4.1.tgz` — spare download, not needed to run.

**assets/**  
- `assembly_model_junxiangyang_edited.glb` — model the page loads; needs `yaw_group` and `panel_group`.  
- `assembly_model.glb` — copy, not used right now.

**root**  
- `WEB_DASHBOARD.md` — how to start the web UI, ports, etc.  
- `README.md` — this file.
