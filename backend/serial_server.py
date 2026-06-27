#!/usr/bin/env python3
"""
Solar tracker WebSocket bridge — reads ESP32 serial, pushes telemetry to browser.

Does NOT modify firmware. Parses existing DATA,... CSV lines only.

Install:
  pip install pyserial websockets

Run (from project root):
  python backend/serial_server.py

Optional env:
  SERIAL_PORT=COM6   # try this port first (still auto-scans if it fails)

Then open:
  http://127.0.0.1:8080/frontend/

WebSocket:
  ws://127.0.0.1:8765
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

import serial
import websockets
from serial.tools import list_ports
from websockets.server import serve

# --- Configuration ---
PROJECT_ROOT = Path(__file__).resolve().parent.parent
SERIAL_BAUD = 115200
SERIAL_PORT_PREF = os.environ.get("SERIAL_PORT", "").strip()  # e.g. COM6
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8080
WS_HOST = "127.0.0.1"
WS_PORT = 8765
WS_PUSH_HZ = 20.0

PROBE_SECONDS = 2.5
RECONNECT_DELAY_S = 2.0
SCAN_RETRY_DELAY_S = 3.0

# Prefer boards that look like ESP32 / USB-serial adapters
_PORT_HINT_RE = re.compile(
    r"usb|jtag|uart|serial|cp210|ch340|ftdi|silicon|esp|acm|modem",
    re.IGNORECASE,
)

CSV_PREFIX = "DATA,"
CSV_FIELDS = 13
SOLAR_LINE_RE = re.compile(
    r"Solar V=([\d.]+) I=([\d.]+)mA P=([\d.]+)mW",
    re.IGNORECASE,
)

_telemetry_lock = threading.Lock()
_latest: dict[str, Any] = {
    "connected": False,
    "serial_port": "",
    "parse_count": 0,
    "ms": 0.0,
    "tl": 0.0,
    "tr": 0.0,
    "bl": 0.0,
    "br": 0.0,
    "err_yaw": 0.0,
    "err_pitch": 0.0,
    "target_yaw": 90.0,
    "current_yaw": 90.0,
    "target_pitch": 90.0,
    "current_pitch": 90.0,
    "action": "—",
    "solar_voltage": None,
    "solar_current_ma": None,
    "solar_power_mw": None,
}


def parse_data_line(line: str) -> Optional[dict[str, Any]]:
    line = line.strip()
    if not line.startswith(CSV_PREFIX):
        return None
    parts = line.split(",")
    if len(parts) != CSV_FIELDS:
        return None
    try:
        return {
            "ms": float(parts[1]),
            "tl": float(parts[2]),
            "tr": float(parts[3]),
            "bl": float(parts[4]),
            "br": float(parts[5]),
            "err_yaw": float(parts[6]),
            "err_pitch": float(parts[7]),
            "target_yaw": float(parts[8]),
            "current_yaw": float(parts[9]),
            "target_pitch": float(parts[10]),
            "current_pitch": float(parts[11]),
            "action": parts[12],
        }
    except (ValueError, IndexError):
        return None


def _snapshot() -> dict[str, Any]:
    with _telemetry_lock:
        return dict(_latest)


def _update(fields: dict[str, Any]) -> None:
    with _telemetry_lock:
        _latest.update(fields)


def _list_candidate_ports() -> list[str]:
    """Build an ordered list of port names to probe."""
    seen: set[str] = set()
    ordered: list[str] = []

    def add(port: str) -> None:
        if port and port not in seen:
            seen.add(port)
            ordered.append(port)

    if SERIAL_PORT_PREF:
        add(SERIAL_PORT_PREF)

    hinted: list[tuple[str, str]] = []
    other: list[tuple[str, str]] = []
    for info in list_ports.comports():
        desc = f"{info.description or ''} {info.manufacturer or ''}"
        entry = (info.device, desc)
        if _PORT_HINT_RE.search(desc) or _PORT_HINT_RE.search(info.device or ""):
            hinted.append(entry)
        else:
            other.append(entry)

    for device, _ in sorted(hinted, key=lambda x: x[0]):
        add(device)
    for device, _ in sorted(other, key=lambda x: x[0]):
        add(device)

    return ordered


def _port_probe_responds(port: str) -> bool:
    """
    Open port briefly and return True if any non-empty line arrives
    (DATA line, Solar line, or boot/debug text from ESP32).
    """
    ser: Optional[serial.Serial] = None
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=0.15)
        ser.dtr = False
        ser.rts = False
        deadline = time.monotonic() + PROBE_SECONDS
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if parse_data_line(line) is not None or SOLAR_LINE_RE.search(line):
                return True
            # Any printable line counts as a live device
            return True
    except serial.SerialException:
        return False
    finally:
        if ser is not None and ser.is_open:
            ser.close()


def find_responsive_port() -> Optional[str]:
    """Scan ports; return the first that responds within PROBE_SECONDS."""
    candidates = _list_candidate_ports()
    if not candidates:
        print("[serial] failed: no serial ports found on this machine")
        return None

    print(f"[serial] connecting: scanning {len(candidates)} port(s) @ {SERIAL_BAUD}…")
    for port in candidates:
        desc = ""
        for info in list_ports.comports():
            if info.device == port:
                desc = info.description or ""
                break
        label = f"{port} ({desc})" if desc else port
        print(f"[serial] connecting: probing {label}…")
        if _port_probe_responds(port):
            print(f"[serial] connected: probe OK on {port}")
            return port
        print(f"[serial] failed: no response on {port}")

    print("[serial] failed: no responsive port (is ESP32 powered and flashed?)")
    return None


def _process_serial_line(line: str) -> None:
    row = parse_data_line(line)
    if row is not None:
        with _telemetry_lock:
            _latest.update(row)
            _latest["connected"] = True
            _latest["parse_count"] = int(_latest.get("parse_count", 0)) + 1
        return

    solar_match = SOLAR_LINE_RE.search(line)
    if solar_match:
        try:
            _update(
                {
                    "solar_voltage": float(solar_match.group(1)),
                    "solar_current_ma": float(solar_match.group(2)),
                    "solar_power_mw": float(solar_match.group(3)),
                }
            )
        except ValueError:
            pass


def serial_reader_loop() -> None:
    """Background thread: auto-detect port, read lines, reconnect on failure."""
    print(f"[serial] auto-detect enabled (baud {SERIAL_BAUD})")
    if SERIAL_PORT_PREF:
        print(f"[serial] preferred port from env: {SERIAL_PORT_PREF}")

    while True:
        port = find_responsive_port()
        if port is None:
            _update({"connected": False, "serial_port": ""})
            print(f"[serial] reconnecting: retry scan in {SCAN_RETRY_DELAY_S:.0f}s…")
            time.sleep(SCAN_RETRY_DELAY_S)
            continue

        ser: Optional[serial.Serial] = None
        try:
            print(f"[serial] connecting: opening {port} @ {SERIAL_BAUD}…")
            ser = serial.Serial(port, SERIAL_BAUD, timeout=1.0)
            ser.dtr = False
            ser.rts = False
            _update({"connected": True, "serial_port": port})
            print(f"[serial] connected: streaming from {port}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="ignore")
                except Exception:
                    continue
                _process_serial_line(line)

        except serial.SerialException as exc:
            print(f"[serial] failed: {exc}")
        except Exception as exc:
            print(f"[serial] failed (unexpected): {exc}")
        finally:
            if ser is not None and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass
            _update({"connected": False})
            print(f"[serial] disconnected — reconnecting in {RECONNECT_DELAY_S:.0f}s…")
            time.sleep(RECONNECT_DELAY_S)


def run_http_server() -> None:
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, directory=str(PROJECT_ROOT), **kwargs)

        def log_message(self, fmt: str, *args: Any) -> None:
            if args and isinstance(args[0], str) and args[0].endswith((".js", ".css", ".html", ".glb")):
                pass
            else:
                super().log_message(fmt, *args)

    httpd = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), Handler)
    print(f"[http] Serving {PROJECT_ROOT} at http://{HTTP_HOST}:{HTTP_PORT}/")
    print(f"[http] Dashboard: http://{HTTP_HOST}:{HTTP_PORT}/frontend/")
    httpd.serve_forever()


async def ws_client_handler(websocket: websockets.WebSocketServerProtocol) -> None:
    peer = websocket.remote_address
    print(f"[ws] Browser connected from {peer}")
    interval = 1.0 / WS_PUSH_HZ
    try:
        while True:
            payload = json.dumps(_snapshot())
            await websocket.send(payload)
            await asyncio.sleep(interval)
    except websockets.ConnectionClosed:
        pass


async def ws_main() -> None:
    async with serve(ws_client_handler, WS_HOST, WS_PORT):
        print(f"[ws] WebSocket ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()


def main() -> None:
    glb = PROJECT_ROOT / "assets" / "assembly_model_junxiangyang_edited.glb"
    print(f"[3d] GLB path: {glb.resolve()}")
    print(f"[3d] GLB exists: {glb.is_file()}")
    if glb.is_file():
        print(f"[3d] GLB size: {glb.stat().st_size} bytes")

    threading.Thread(target=serial_reader_loop, daemon=True).start()
    threading.Thread(target=run_http_server, daemon=True).start()
    time.sleep(0.3)
    asyncio.run(ws_main())


if __name__ == "__main__":
    main()
