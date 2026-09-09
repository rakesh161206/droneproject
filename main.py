"""
AEGIS SAR Mission Gateway & Unified Backend
Problem Statement 26177 (Qualcomm / SIH - Robotics & Drones Theme)

Coordinates:
- Autopilot telemetry & commands (TCP 5760 bridge with SITL / Pixhawk)
- Camera frame uploads (/upload) & real-time AI perception (RGB/Thermal)
- Persistent SQLite mission logging
- Real-time WebSocket telemetry stream to aegis-sar-dashboard-2.html
"""

from __future__ import annotations
import asyncio
import json
import logging
import math
import os
import socket
import time
from contextlib import asynccontextmanager
import platform
from pathlib import Path
from typing import Dict, Any, Iterator, List, Optional, Set

import uvicorn
import cv2
import numpy as np
from fastapi import FastAPI, File, UploadFile, Form, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from drone_details import (
    HARDWARE_SPEC,
    SAFETY_THRESHOLDS,
    TelemetryPacket,
    DetectionRecord,
    FlightMode,
    calculate_distance_meters,
    calculate_bearing_degrees,
    offset_coordinate,
)
from database import db
from ai_detector import ai_detector

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
log = logging.getLogger("aegis_backend")

BASE_DIR = Path(__file__).parent
CAPTURES_DIR = BASE_DIR / "captures"
CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
DASHBOARD_FILE = BASE_DIR / "aegis-sar-dashboard-2.html"

AUTOPILOT_HOST = os.getenv("AUTOPILOT_HOST", "127.0.0.1")
AUTOPILOT_PORT = int(os.getenv("AUTOPILOT_PORT", "5760"))
SERVER_PORT = int(os.getenv("PORT", "8000"))
CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "0")
THERMAL_CAMERA_SOURCE = os.getenv("THERMAL_CAMERA_SOURCE", CAMERA_SOURCE)

# Render and other cloud hosts inject the listening port through PORT.
# Keep this explicit so the app binds correctly in deployment environments.
if "PORT" in os.environ:
    SERVER_PORT = int(os.environ["PORT"])


def get_camera_source(source_name: str = "rgb"):
    """Use the RGB camera by default for the thermal feed and process it as thermal-style imagery."""
    camera_source = THERMAL_CAMERA_SOURCE if source_name.lower() == "thermal" else CAMERA_SOURCE
    if source_name.lower() == "thermal" and (camera_source in (None, "", "1")):
        camera_source = CAMERA_SOURCE
    return int(camera_source) if str(camera_source).isdigit() else camera_source


def convert_to_thermal_style(frame: np.ndarray) -> np.ndarray:
    """Convert an RGB frame into a thermal-style pseudo-color image using the RGB camera as source."""
    if frame is None:
        return frame
    if frame.ndim == 2:
        gray = frame.astype(np.uint8)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray = cv2.equalizeHist(gray)
    thermal = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
    thermal = cv2.cvtColor(thermal, cv2.COLOR_BGR2RGB)
    return thermal


def create_video_capture(camera_source):
    """Create a camera capture using the best available backend for this OS."""
    if isinstance(camera_source, str):
        return cv2.VideoCapture(camera_source)

    if platform.system() == "Windows":
        cap = cv2.VideoCapture(camera_source, cv2.CAP_DSHOW)
        if cap.isOpened():
            return cap
        return cv2.VideoCapture(camera_source)

    return cv2.VideoCapture(camera_source)


def generate_fallback_frame(source_name: str, width: int = 640, height: int = 480):
    """Generate a synthetic frame when no real camera is available."""
    frame = 255 * np.ones((height, width, 3), dtype=np.uint8)

    if source_name == "thermal":
        frame[:] = (30, 35, 40)
        for y in range(height):
            for x in range(width):
                v = (x + y) % 255
                frame[y, x] = (min(255, v + 20), min(255, max(0, 120 - (x // 8))), min(255, max(0, 180 - (y // 10))))

        overlay = np.zeros_like(frame)
        cv2.circle(overlay, (width // 2, height // 2), 120, (60, 120, 255), -1)
        cv2.circle(overlay, (width // 2 - 70, height // 2 + 20), 45, (150, 200, 255), -1)
        cv2.addWeighted(frame, 0.7, overlay, 0.3, 0, frame)
        cv2.putText(frame, "THERMAL IR", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.rectangle(frame, (width // 2 - 17, int(height * 0.47)), (width // 2 + 17, int(height * 0.47) + 44), (0, 165, 255), 2)
        cv2.putText(frame, "HUMAN DETECTED", (18, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
    else:
        frame[:] = (40, 65, 50)
        cv2.rectangle(frame, (80, 120), (560, 380), (130, 185, 110), -1)
        cv2.rectangle(frame, (140, 150), (220, 300), (200, 210, 160), -1)
        cv2.rectangle(frame, (420, 150), (500, 300), (200, 210, 160), -1)
        cv2.circle(frame, (180, 116), 22, (220, 230, 220), -1)
        cv2.circle(frame, (460, 116), 22, (220, 230, 220), -1)
        cv2.rectangle(frame, (250, 170), (390, 290), (120, 160, 220), -1)
        cv2.putText(frame, "RGB CAMERA", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(frame, "LIVE SCENE", (18, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150, 255, 180), 2)

    return frame


def mjpeg_frames(source_name: str = "rgb") -> Iterator[bytes]:
    """Yield JPEG frames for browsers that support multipart MJPEG streams."""
    source_name = (source_name or "rgb").lower()

    # Send a frame before opening hardware so a missing or slow camera never
    # leaves the browser waiting on an empty MJPEG response.
    initial_frame = generate_fallback_frame(source_name)
    ok, encoded = cv2.imencode(".jpg", initial_frame)
    if ok:
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache\r\n\r\n"
            + encoded.tobytes()
            + b"\r\n"
        )

    camera_source = get_camera_source(source_name)
    camera = create_video_capture(camera_source)

    if not camera.isOpened():
        log.warning("Could not open %s camera source %s; falling back to the RGB stream for thermal-style output.", source_name, camera_source)
        camera_source = get_camera_source("rgb")
        camera = create_video_capture(camera_source)

    if not camera.isOpened():
        log.warning("Could not open %s camera source %s; using synthetic fallback stream.", source_name, camera_source)
        try:
            while True:
                frame = generate_fallback_frame(source_name)
                if source_name == "thermal":
                    frame = convert_to_thermal_style(frame)
                ok, encoded = cv2.imencode(".jpg", frame)
                if not ok:
                    continue
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-cache\r\n\r\n"
                    + encoded.tobytes()
                    + b"\r\n"
                )
                time.sleep(0.2)
        except Exception:
            return
        return

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                log.warning("Frame read failed for %s camera; switching to fallback stream.", source_name)
                frame = generate_fallback_frame(source_name)

            if source_name == "thermal":
                frame = convert_to_thermal_style(frame)

            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-cache\r\n\r\n"
                + encoded.tobytes()
                + b"\r\n"
            )
    finally:
        camera.release()

# Active state
current_telemetry = TelemetryPacket(lat=18.5225, lon=73.8567, alt=35.0, ground_speed=0.0, heading=0.0, is_flying=True, flight_mode=FlightMode.LOITER)
target_waypoint: Optional[Dict[str, Any]] = None
home_location: Dict[str, float] = {"lat": 18.5225, "lon": 73.8567, "alt": 0.0}
active_mission_id = "MISSION-SAR-01"
autopilot_connected = False
active_websockets: Set[WebSocket] = set()


# --------------------------------------------------------------------------
# Autopilot TCP Bridge & User-Directed Navigation Simulation Worker
# --------------------------------------------------------------------------

def query_autopilot_tcp(command: str) -> Optional[str]:
    """Send single-line command to autopilot TCP server and return response."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((AUTOPILOT_HOST, AUTOPILOT_PORT))
            s.sendall((command.strip() + "\n").encode())
            reply = s.recv(1024).decode().strip()
            return reply
    except Exception:
        return None


async def telemetry_broadcast_loop():
    """Continuous background loop updating telemetry and broadcasting to WebSockets."""
    global current_telemetry, autopilot_connected, active_mission_id, target_waypoint

    sim_battery = 92.0
    tick_count = 0

    while True:
        try:
            tick_count += 1

            # 1. Try polling real autopilot
            reply = query_autopilot_tcp("STATUS")
            if reply and reply.startswith("OK:"):
                autopilot_connected = True
                parts = reply.replace("OK:", "").strip().split()
                data = {}
                for p in parts:
                    if "=" in p:
                        k, v = p.split("=", 1)
                        data[k] = v.rstrip("%")

                try:
                    current_telemetry.lat = float(data.get("lat", current_telemetry.lat))
                    current_telemetry.lon = float(data.get("lon", current_telemetry.lon))
                    current_telemetry.alt = float(data.get("alt", current_telemetry.alt))
                    current_telemetry.battery_pct = float(data.get("battery", current_telemetry.battery_pct))
                    current_telemetry.is_flying = current_telemetry.alt > 1.5
                    current_telemetry.flight_mode = FlightMode.GUIDED if current_telemetry.is_flying else FlightMode.LOITER
                except Exception as e:
                    log.warning(f"Error parsing autopilot status: {e}")
            else:
                autopilot_connected = False
                
                # 2. User-driven waypoint travel simulation
                if target_waypoint is not None:
                    t_lat = target_waypoint["lat"]
                    t_lon = target_waypoint["lon"]
                    t_alt = target_waypoint.get("alt", 35.0)

                    dist_rem = calculate_distance_meters(current_telemetry.lat, current_telemetry.lon, t_lat, t_lon)

                    if dist_rem > 3.0:
                        bearing = calculate_bearing_degrees(current_telemetry.lat, current_telemetry.lon, t_lat, t_lon)
                        speed_ms = 14.0  # Cruising speed m/s
                        step_m = min(dist_rem, speed_ms * 0.5)
                        next_lat, next_lon = offset_coordinate(current_telemetry.lat, current_telemetry.lon, step_m, bearing)

                        current_telemetry.lat = round(next_lat, 6)
                        current_telemetry.lon = round(next_lon, 6)
                        current_telemetry.heading = round(bearing, 1)
                        current_telemetry.ground_speed = speed_ms
                        
                        # Climb or descend smoothly to target altitude
                        if current_telemetry.alt < t_alt:
                            current_telemetry.alt = round(min(t_alt, current_telemetry.alt + 2.0), 1)
                        elif current_telemetry.alt > t_alt:
                            current_telemetry.alt = round(max(t_alt, current_telemetry.alt - 1.0), 1)

                        current_telemetry.is_flying = True
                        current_telemetry.flight_mode = FlightMode.GUIDED
                        sim_battery = max(10.0, sim_battery - 0.015)
                    else:
                        # Arrived at the user-specified destination
                        current_telemetry.lat = t_lat
                        current_telemetry.lon = t_lon
                        current_telemetry.ground_speed = 0.0
                        current_telemetry.flight_mode = FlightMode.LOITER
                        target_waypoint["status"] = "ARRIVED"
                else:
                    # Drone is hovering / on station at its current location
                    current_telemetry.ground_speed = 0.0
                    if current_telemetry.is_flying:
                        current_telemetry.flight_mode = FlightMode.LOITER
                        sim_battery = max(10.0, sim_battery - 0.003)

                current_telemetry.battery_pct = round(sim_battery, 1)
                current_telemetry.link_latency_ms = 24

            current_telemetry.timestamp = time.time()

            # 3. Log to SQLite periodically
            if tick_count % 5 == 0:
                db.log_telemetry(active_mission_id, current_telemetry.to_dict())

            # 4. Broadcast to WebSockets
            if active_websockets:
                payload = json.dumps({
                    "type": "telemetry",
                    "data": current_telemetry.to_dict(),
                    "target_waypoint": target_waypoint,
                    "autopilot_connected": autopilot_connected,
                    "hardware": {
                        "callsign": HARDWARE_SPEC.callsign,
                        "model": HARDWARE_SPEC.model,
                        "sensors": HARDWARE_SPEC.sensors,
                    }
                })
                dead_sockets = set()
                for ws in list(active_websockets):
                    try:
                        await ws.send_text(payload)
                    except Exception:
                        dead_sockets.add(ws)
                active_websockets.difference_update(dead_sockets)

        except Exception as e:
            log.error(f"Error in telemetry loop: {e}")

        await asyncio.sleep(0.5)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    log.info(f"Starting AEGIS SAR Mission Gateway on port {SERVER_PORT}...")
    db.init_db()
    # Initialize default mission if none exists
    missions = db.get_missions()
    if not missions:
        db.start_mission(active_mission_id, "Operation Red Dawn SAR")
        log.info(f"Initialized mission: {active_mission_id}")
    
    # Start background telemetry loop
    task = asyncio.create_task(telemetry_broadcast_loop())
    yield
    # Shutdown
    task.cancel()
    log.info("AEGIS SAR Gateway stopped.")


app = FastAPI(
    title="AEGIS SAR Mission Control Gateway",
    description="Backend bridge for AI-powered Search and Rescue drone operations.",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve captures statically
app.mount("/captures", StaticFiles(directory=str(CAPTURES_DIR)), name="captures")


# --------------------------------------------------------------------------
# Web & Dashboard Routes
# --------------------------------------------------------------------------

@app.get("/")
@app.get("/healthz")
async def health_check():
    """Simple health endpoint for deployment checks and uptime probes."""
    return {"status": "ok", "service": "aegis-sar-dashboard", "port": SERVER_PORT}


@app.get("/dashboard")
async def serve_dashboard():
    """Serve the AEGIS SAR mission console directly."""
    if DASHBOARD_FILE.exists():
        return FileResponse(DASHBOARD_FILE, media_type="text/html")
    return JSONResponse({"message": "Dashboard HTML not found at root"}, status_code=404)


@app.get("/api/status")
async def get_system_status():
    """Retrieve full system status, hardware specs, and telemetry."""
    return {
        "status": "ONLINE",
        "active_mission_id": active_mission_id,
        "autopilot_connected": autopilot_connected,
        "telemetry": current_telemetry.to_dict(),
        "hardware": {
            "callsign": HARDWARE_SPEC.callsign,
            "model": HARDWARE_SPEC.model,
            "sensors": HARDWARE_SPEC.sensors,
            "max_flight_time_min": HARDWARE_SPEC.max_flight_time_min,
            "battery_capacity_mah": HARDWARE_SPEC.battery_capacity_mah,
        },
        "safety": {
            "min_battery": SAFETY_THRESHOLDS.min_takeoff_battery_pct,
            "critical_rtl": SAFETY_THRESHOLDS.critical_rtl_battery_pct,
            "geofence_m": SAFETY_THRESHOLDS.geofence_radius_m
        }
    }


@app.get("/video_feed")
def video_feed(source: str = "rgb"):
    """Stream the configured RGB or thermal camera live to the dashboard."""
    return StreamingResponse(
        mjpeg_frames(source.lower()),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/detections")
async def get_detections():
    """Retrieve recorded geo-tagged detections."""
    return db.get_detections(active_mission_id)


@app.post("/api/detections")
async def create_detection(payload: Dict[str, Any]):
    """Accept and persist a detection from autopilot or external CV model."""
    record = {
        "mission_id": active_mission_id,
        "timestamp": payload.get("timestamp", time.time()),
        "label": payload.get("label", "Survivor"),
        "confidence": payload.get("confidence", 0.95),
        "severity": payload.get("severity", "HIGH"),
        "lat": payload.get("lat", current_telemetry.lat),
        "lon": payload.get("lon", current_telemetry.lon),
        "alt": payload.get("alt", current_telemetry.alt),
        "image_path": payload.get("image_path"),
        "thermal_temp_c": payload.get("thermal_temp_c", 37.0),
        "bbox": payload.get("bbox"),
        "notes": payload.get("notes", "Reported by autopilot detection hook")
    }
    det_id = db.save_detection(record)
    record["id"] = det_id

    # Broadcast to connected dashboards
    if active_websockets:
        broadcast_msg = json.dumps({
            "type": "new_detection",
            "image_url": record.get("image_path"),
            "source": "autopilot_hook",
            "timestamp": time.strftime("%Y%m%d_%H%M%S"),
            "detections": [record],
            "current_location": {
                "lat": record["lat"],
                "lon": record["lon"],
                "alt": record["alt"]
            }
        })
        for ws in list(active_websockets):
            try:
                await ws.send_text(broadcast_msg)
            except Exception:
                pass

    return {"status": "ok", "detection_id": det_id, "record": record}


@app.get("/api/missions")
async def get_missions():
    """Retrieve mission history."""
    return db.get_missions()


@app.post("/api/command")
async def dispatch_command(payload: Dict[str, Any]):
    """
    Dispatch operational command to the drone.
    Examples:
      {"command": "START"}
      {"command": "PAUSE"}
      {"command": "RTL"}
      {"command": "LAND"}
      {"command": "FLY", "lat": 18.5230, "lon": 73.8570, "alt": 20.0}
    """
    cmd = payload.get("command", "").upper()
    log.info(f"Received operator command: {payload}")
    db.log_event("OPERATOR_COMMAND", f"Command: {cmd}", json.dumps(payload))

    global target_waypoint
    autopilot_msg = ""
    if cmd == "START":
        current_telemetry.is_flying = True
        if current_telemetry.alt < 10.0:
            current_telemetry.alt = 20.0
        current_telemetry.flight_mode = FlightMode.GUIDED if target_waypoint else FlightMode.LOITER
        autopilot_msg = "Mission started - waiting for destination" if not target_waypoint else "Mission active - navigating to destination"
    elif cmd == "PAUSE":
        target_waypoint = None
        current_telemetry.ground_speed = 0.0
        current_telemetry.flight_mode = FlightMode.LOITER
        autopilot_msg = "Mission paused - loitering at current location"
    elif cmd == "RTL":
        current_telemetry.flight_mode = FlightMode.RTL
        target_waypoint = {
            "lat": home_location["lat"],
            "lon": home_location["lon"],
            "alt": 25.0,
            "status": "RETURNING_TO_BASE",
            "start_time": time.time()
        }
        reply = query_autopilot_tcp("RTL")
        autopilot_msg = reply or f"Returning to base ({home_location['lat']}, {home_location['lon']})"
    elif cmd == "LAND":
        target_waypoint = None
        current_telemetry.flight_mode = FlightMode.LAND
        current_telemetry.ground_speed = 0.0
        current_telemetry.alt = 0.0
        current_telemetry.is_flying = False
        reply = query_autopilot_tcp("LAND")
        autopilot_msg = reply or "Landing initiated"
    elif cmd == "FLY":
        lat = float(payload.get("lat", current_telemetry.lat))
        lon = float(payload.get("lon", current_telemetry.lon))
        alt = float(payload.get("alt", 35.0))
        target_waypoint = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "status": "NAVIGATING",
            "start_time": time.time()
        }
        current_telemetry.is_flying = True
        if current_telemetry.alt < 10.0:
            current_telemetry.alt = 20.0
        current_telemetry.flight_mode = FlightMode.GUIDED
        reply = query_autopilot_tcp(f"FLY {lat},{lon},{alt}")
    elif cmd in ("SET_LOCATION", "SET_HOME", "SET_ORIGIN"):
        lat = float(payload.get("lat", current_telemetry.lat))
        lon = float(payload.get("lon", current_telemetry.lon))
        current_telemetry.lat = lat
        current_telemetry.lon = lon
        home_location["lat"] = lat
        home_location["lon"] = lon
        target_waypoint = None
        current_telemetry.ground_speed = 0.0
        autopilot_msg = f"Drone live position set to user location: {lat:.5f}, {lon:.5f}"
    else:
        raise HTTPException(status_code=400, detail=f"Unknown command: {cmd}")

    # Broadcast notification over WebSocket
    if active_websockets:
        event_payload = json.dumps({
            "type": "command_feedback",
            "command": cmd,
            "message": autopilot_msg,
            "telemetry": current_telemetry.to_dict()
        })
        for ws in list(active_websockets):
            try:
                await ws.send_text(event_payload)
            except Exception:
                pass

    return {
        "status": "success",
        "command": cmd,
        "autopilot_response": autopilot_msg,
        "drone_mode": current_telemetry.flight_mode.value
    }


# --------------------------------------------------------------------------
# Camera Upload & AI Detection Pipeline (/upload)
# --------------------------------------------------------------------------

@app.post("/upload")
async def upload_camera_frame(
    image: UploadFile = File(...),
    timestamp: Optional[str] = Form(None),
    source: Optional[str] = Form("drone_cam")
):
    """
    HTTP POST handler for camera.py.
    Saves frame, runs multi-spectral AI detection, geo-tags target,
    persists into SQLite, and broadcasts frame + detections to dashboard.
    """
    ts = timestamp or time.strftime("%Y%m%d_%H%M%S")
    filename = f"capture_{ts}_{image.filename}"
    file_path = CAPTURES_DIR / filename

    # Save uploaded file
    contents = await image.read()
    with open(file_path, "wb") as f:
        f.write(contents)

    log.info(f"Received frame {filename} ({len(contents)} bytes) from {source}")

    # Run AI perception analysis
    detections = ai_detector.analyze_frame(file_path)

    saved_detections = []
    # Geo-tag each detected target with current drone GPS position
    for det in detections:
        record = {
            "mission_id": active_mission_id,
            "timestamp": time.time(),
            "label": det.get("label", "Survivor"),
            "confidence": det.get("confidence", 0.92),
            "severity": det.get("severity", "HIGH"),
            "lat": current_telemetry.lat,
            "lon": current_telemetry.lon,
            "alt": current_telemetry.alt,
            "image_path": f"/captures/{filename}",
            "thermal_temp_c": det.get("thermal_temp_c", 37.0),
            "bbox": det.get("bbox"),
            "notes": det.get("notes", "")
        }
        det_id = db.save_detection(record)
        record["id"] = det_id
        saved_detections.append(record)

    # Broadcast frame and detection results to connected dashboards
    if active_websockets:
        broadcast_msg = json.dumps({
            "type": "new_detection",
            "image_url": f"/captures/{filename}",
            "source": source,
            "timestamp": ts,
            "detections": saved_detections,
            "current_location": {
                "lat": current_telemetry.lat,
                "lon": current_telemetry.lon,
                "alt": current_telemetry.alt
            }
        })
        for ws in list(active_websockets):
            try:
                await ws.send_text(broadcast_msg)
            except Exception:
                pass

    return {
        "status": "ok",
        "saved_as": filename,
        "image_url": f"/captures/{filename}",
        "detections_count": len(saved_detections),
        "detections": saved_detections
    }


# --------------------------------------------------------------------------
# WebSocket Endpoint for Real-Time Console Stream
# --------------------------------------------------------------------------

@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_websockets.add(websocket)
    log.info(f"Dashboard client connected via WebSocket. Total clients: {len(active_websockets)}")

    # Send initial state snapshot immediately
    initial_packet = {
        "type": "initial_state",
        "telemetry": current_telemetry.to_dict(),
        "hardware": {
            "callsign": HARDWARE_SPEC.callsign,
            "model": HARDWARE_SPEC.model,
            "sensors": HARDWARE_SPEC.sensors,
        },
        "autopilot_connected": autopilot_connected,
        "active_mission": active_mission_id,
        "recent_detections": db.get_detections(active_mission_id)
    }
    await websocket.send_text(json.dumps(initial_packet))

    try:
        while True:
            # Listen for client actions or pings
            raw_data = await websocket.receive_text()
            try:
                msg = json.loads(raw_data)
                if msg.get("action") == "COMMAND":
                    cmd = msg.get("command")
                    log.info(f"WebSocket command received: {cmd}")
                    # Handle or relay
                    if cmd in ("START", "PAUSE", "RTL", "LAND"):
                        if cmd == "RTL":
                            query_autopilot_tcp("RTL")
                        elif cmd == "LAND":
                            query_autopilot_tcp("LAND")
            except Exception:
                pass
    except WebSocketDisconnect:
        active_websockets.discard(websocket)
        log.info(f"Dashboard client disconnected. Total clients: {len(active_websockets)}")
    except Exception as e:
        active_websockets.discard(websocket)
        log.warning(f"WebSocket connection closed with error: {e}")


def main():
    uvicorn.run("main:app", host="0.0.0.0", port=SERVER_PORT, reload=False, log_level="info")


if __name__ == "__main__":
    main()
