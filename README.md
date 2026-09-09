# AEGIS SAR Dashboard

AEGIS SAR is a browser-based search-and-rescue mission console for monitoring a simulated drone operation. It provides a live operational map, incident zones, telemetry, camera-fusion panels, link health, and mission controls in a single dashboard.

## Features

- Leaflet map with satellite and tactical basemaps
- Simulated drone patrol route and animated drone marker
- Survivor, fire, and flood incident zones with severity levels
- Live zone tour with selectable incident cards
- Simulated telemetry for battery, altitude, speed, coverage, latency, and packet loss
- Live laptop camera feed in the RGB panel, with the thermal panel reserved for future thermal hardware
- Location search using OpenStreetMap Nominatim
- Browser geolocation support
- Mission controls for start, pause, return to base, and abort
- Responsive layout for desktop and smaller screens

## Project Structure

| File | Purpose |
| --- | --- |
| `aegis-sar-dashboard-2.html` | Complete dashboard UI, styling, map integration, and JavaScript simulation |
| `main.py` | Reserved Python entry point; not implemented yet |
| `autopilot.py` | Reserved autopilot module; not implemented yet |
| `camera.py` | Reserved camera module; not implemented yet |
| `drone_details.py` | Reserved drone data module; not implemented yet |
| `gpslocation.py` | Reserved GPS module; not implemented yet |
| `Screenshot 2026-09-04 at 10.31.23 PM.png` | Project screenshot/reference image |

## Run Locally

No build step is required. The gateway uses `fastapi`, `uvicorn`, `opencv-python`, and `requests`.

1. Start the FastAPI gateway so it can read the laptop camera and stream it to the dashboard:

	```bash
	python3 main.py
	```

2. Visit [http://localhost:8000/dashboard](http://localhost:8000/dashboard).

For a future drone or RTSP camera, set `CAMERA_SOURCE` before starting the gateway:

	```bash
	CAMERA_SOURCE="rtsp://192.168.1.1:8554/live" python3 main.py
	```

You can still open the HTML directly or serve the folder with a local web server for the offline simulation:

	```powershell
	python -m http.server 8000
	```

Serving the file locally is recommended because browser geolocation and network requests are more reliable from a local web origin than from a `file://` URL.

## Using the Dashboard

- Use **SATELLITE** and **TACTICAL** to switch map layers.
- Select an incident card or map marker to focus the map on that zone.
- Toggle **LIVE TOUR** to automatically cycle through incident zones.
- Search for a place in the map search box to move the operational area.
- Use the GPS button to center the operation on the browser's current location.
- Use the mission controls to start, pause, return to base, or abort the simulation.

## External Services

The dashboard currently loads these resources over the internet:

- Leaflet 1.9.4 from cdnjs
- Satellite imagery and place labels from ArcGIS
- Tactical map tiles from Carto
- Place search from OpenStreetMap Nominatim
- Fonts from Google Fonts

An internet connection is therefore required for the full map and search experience.

## Current Scope

This is a front-end simulation and demonstration dashboard. Telemetry, patrol movement, incident data, camera feeds, and AI recommendations are generated or displayed locally in the page. The Python files are placeholders for future drone, GPS, camera, and autopilot integrations.

## Future Work

- Connect telemetry to a real drone or simulator
- Replace simulated camera panels with live video streams
- Move mission and incident data into a backend service
- Add authentication, persistent mission history, and operator audit logs
- Implement the Python modules and connect them to the dashboar