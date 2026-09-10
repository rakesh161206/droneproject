# Copilot instructions for AEGIS SAR

## Project shape

AEGIS SAR is a Python/FastAPI mission gateway paired with a single-page browser dashboard:

- `main.py` is the runtime hub. It serves `aegis-sar-dashboard-2.html`, exposes REST endpoints, streams MJPEG camera frames, accepts camera uploads, runs AI detection, persists mission data, and broadcasts telemetry/events over `/ws/telemetry`.
- `aegis-sar-dashboard-2.html` is a self-contained UI with inline CSS and JavaScript. It talks to the gateway for commands, camera feeds, detections, and WebSocket updates, while retaining simulated map/mission behavior for offline demos.
- `drone_details.py` contains shared enums, telemetry/detection dataclasses, hardware/safety constants, and coordinate math used by the gateway and autopilot.
- `database.py` owns the SQLite schema and persistence API. The database file is `aegis_sar.db`; startup initializes tables and creates a default mission when none exists.
- `ai_detector.py` provides the upload inference path. It uses YOLO only when explicitly given valid weights and otherwise falls back to the NumPy/Pillow heuristic detector.
- `camera/camera.py` captures a local camera or RTSP source and POSTs multipart images to `/upload`.
- `auto_pilot/autopilot.py` is a separate MAVLink/DroneKit process. It can run against ArduPilot SITL or hardware and exposes a plain-text TCP command bridge on port `5760`. `main.py` polls that bridge when available and otherwise runs its built-in waypoint simulation.
- `gps/gps_simulator.py` is a standalone Folium demo that generates `real_gps_map.html`; it is not part of the FastAPI request path.

The deployment described by `render.yaml` runs `uvicorn main:app` and injects `PORT`. Keep the dashboard, gateway, and optional autopilot interfaces compatible rather than duplicating mission state in each layer.

Render uses the committed `render.yaml`: it installs `requirements.txt`, starts `uvicorn main:app --host 0.0.0.0 --port $PORT`, and checks `/healthz`. The service is designed to run without a camera, flight controller, or autopilot process; those integrations are optional and fall back to synthetic frames and local waypoint simulation.

## Install and run

From the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 main.py
```

Open `http://localhost:8000/dashboard`. The gateway also provides `/healthz`, `/api/status`, `/api/missions`, `/api/detections`, `/api/command`, `/upload`, `/video_feed`, and `/ws/telemetry`.

For development with Uvicorn's reloader:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

To run the browser-only simulation without the gateway:

```bash
python3 -m http.server 8000
```

Camera configuration is environment-driven:

```bash
CAMERA_SOURCE="rtsp://192.168.1.1:8554/live" python3 main.py
THERMAL_CAMERA_SOURCE="/path/or/index" python3 main.py
```

The optional autopilot process has additional dependencies that are not in the root requirements file:

```bash
python3 -m pip install dronekit pymavlink dronekit-sitl
dronekit-sitl copter --home=12.9716,77.5946,0,0
python3 auto_pilot/autopilot.py
python3 auto_pilot/trigger_client.py 127.0.0.1 FLY 12.9720,77.5950,20
```

## Validation

There is currently no repository test suite or configured linter. Use targeted syntax checks when changing Python modules:

```bash
python3 -m py_compile main.py
python3 -m py_compile database.py ai_detector.py drone_details.py
python3 -m py_compile camera/camera.py auto_pilot/autopilot.py auto_pilot/trigger_client.py gps/gps_simulator.py
```

For a single changed module, run only its corresponding `py_compile` command. For runtime smoke checks, start `main.py` and request `/healthz` or `/api/status`; do not assume a physical camera or flight controller is available.

## Browser validation with Playwright MCP

When Playwright MCP is available, use it for dashboard smoke checks after frontend or gateway changes:

- Start the gateway and open `/dashboard`.
- Confirm the page loads without console errors and the dashboard connects to `/ws/telemetry`.
- Exercise the mission controls and verify `/api/command` feedback updates the UI.
- Confirm RGB and thermal panels fall back cleanly when no camera is attached.
- Check that detections received through `/api/detections` or `/upload` appear in the dashboard.
- Use `/healthz` as the basic readiness check before browser assertions.

## Cross-cutting conventions

- Keep API payloads JSON-compatible. Convert enums such as `FlightMode` and `IncidentSeverity` to their string values through the existing `to_dict()` methods before returning or broadcasting them.
- Mission state is process-global in `main.py`, while durable history belongs in `database.py`. Persist telemetry periodically and detections/events through the database API instead of writing SQL in route handlers.
- Detection records must remain geo-tagged. Uploads and autopilot reports use the current telemetry position as the fallback, then persist through `db.save_detection()` so the dashboard and mission history see the same record.
- The dashboard expects the existing WebSocket message types (`initial_state`, `telemetry`, `new_detection`, and `command_feedback`) and the REST response shapes. Extend these payloads compatibly; update both `main.py` and the inline dashboard JavaScript when changing a contract.
- Camera endpoints must remain usable without hardware. Preserve the synthetic fallback frames and thermal-style conversion so local demos and hosted deployments do not hang when no camera is attached.
- `CAMERA_SOURCE`, `THERMAL_CAMERA_SOURCE`, `AUTOPILOT_HOST`, `AUTOPILOT_PORT`, and `PORT` are the supported runtime configuration points. Avoid hard-coding deployment-specific addresses or ports.
- The autopilot bridge uses one-line plain-text commands: `FLY <lat>,<lon>,<alt>`, `RTL`, `LAND`, and `STATUS`. Keep the bridge optional: failed TCP queries should leave the local simulation usable.
- Coordinate calculations should reuse the helpers in `drone_details.py` (`calculate_distance_meters`, `calculate_bearing_degrees`, and `offset_coordinate`) rather than introducing competing formulas.
- The SQLite schema is initialized with `CREATE TABLE IF NOT EXISTS`. Preserve existing columns and migration compatibility when changing persistence behavior; use parameterized SQL for values.
- The frontend is intentionally a single HTML file with inline styles/scripts and local Leaflet assets under `vendor/leaflet`. Avoid introducing a build pipeline unless the project is deliberately being migrated.
- External map/search resources currently require network access; local Leaflet files only cover the bundled map library, not tile providers, Nominatim, or fonts.
