# SAR Drone Autopilot — SIH Problem Statement 26177

Flight-control module for the AI-powered search-and-rescue drone
(Qualcomm, Robotics & Drones theme). Handles: connecting to the flight
controller, arming/takeoff, autonomous GPS navigation, and a network
trigger so a teammate can command "fly now" and watch live GPS.

This is the **navigation layer**. The RGB/thermal person + hazard
detection model is a separate on-device process — it calls
`autopilot.report_detection(...)` whenever it spots something, so
detections get GPS-tagged for your command-center map.

## Files

| File | Purpose |
|---|---|
| `autopilot.py` | Runs **on the drone's companion computer** (e.g. Raspberry Pi / Jetson connected to the Pixhawk). Connects to the flight controller and listens for commands. |
| `trigger_client.py` | Runs on **your friend's laptop**. Sends the "fly" command and polls live GPS. |

## Option A — Test with a simulator first (recommended before touching real hardware)

You don't need a physical drone to demo this. Install ArduPilot's SITL
(Software In The Loop) simulator, which fully emulates the flight
controller including GPS:

```bash
pip install dronekit pymavlink dronekit-sitl --break-system-packages
dronekit-sitl copter --home=12.9716,77.5946,0,0
```

This starts a simulated copter at Bangalore coordinates and prints the
MAVLink address it's listening on (default `127.0.0.1:14550`, already
set as the default in `autopilot.py`).

In a second terminal:
```bash
python autopilot.py
```

In a third terminal (or your friend's machine, if on the same network):
```bash
python trigger_client.py 127.0.0.1 FLY 12.9720,77.5950,20
```

You'll see the drone arm, climb to 20m, and fly to the new coordinate —
all simulated, with live GPS printouts on the client side.

## Option B — Real hardware

1. Flash **ArduPilot** or **PX4** firmware onto a Pixhawk-class flight
   controller (standard for SIH robotics builds).
2. Connect the flight controller to a companion computer (Raspberry Pi
   4 / Jetson Nano) via USB or UART.
3. In `autopilot.py`, set:
   ```python
   connection_string: str = "/dev/ttyACM0"   # USB
   # or
   connection_string: str = "/dev/ttyUSB0,57600"   # telemetry radio
   ```
4. Run `python autopilot.py` on the companion computer. It will print
   its IP — give that to your friend.
5. Friend runs:
   ```bash
   python trigger_client.py <drone_ip> FLY <lat>,<lon>,<alt>
   ```

**Do this in an open outdoor area with GPS lock, props clear, and
someone ready on a physical RC transmitter to override into manual
mode if anything looks wrong.** The script includes battery and GPS
pre-flight checks, but they're not a substitute for a human safety
observer during any real flight test.

## How this fits the rest of your SIH submission

- **Autonomous Navigation** ✅ — this module (GPS waypoint nav via MAVLink)
- **On-Device AI Inference** — separate process, calls `report_detection()`
- **Multi-Sensor Fusion** — IMU/GPS already fused by the flight controller (EKF); camera fusion happens in your detection module
- **Geo-Tagged Mapping** — every `report_detection()` call is timestamped + GPS-tagged, ready to feed a live map
- **Offline Resilience** — MAVLink + on-device inference need no internet; only the command-center sync needs connectivity, and that can queue and retry

## Next steps you'll probably want

- SLAM/obstacle avoidance for GPS-denied indoor/damaged environments (this script assumes outdoor GPS flight — a good v2 add)
- Swap the plain-text TCP trigger for MQTT if your command-center dashboard already uses it
- Persist detections to SQLite so nothing is lost if connectivity drops mid-mission
