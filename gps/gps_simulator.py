import os
import time
import folium
import webbrowser

print("=== GPS Software Simulation & Real Mapping Started ===")

# 1. Initialize a Real Map centered around a geographic location (e.g., Salem, India)
start_lat, start_lon = 11.6643, 78.1460
drone_map = folium.Map(
    location=[start_lat, start_lon], zoom_start=17, tiles="OpenStreetMap"
)

# Lists to store flight coordinates and geo-tagged items
flight_path = []
geo_tags = []

print(
    "\n[SIMULATION] GPS receiver powering on... Acquiring satellite fix..."
)
time.sleep(1)

# Simulating 10 GPS updates (representing the drone flying a search grid)
current_lat = start_lat
current_lon = start_lon

for i in range(1, 11):
  # Simulate drone movement (Updating Latitude and Longitude)
  current_lat += 0.00015
  current_lon += 0.0001
  altitude = 45.0 + (i * 0.5)
  satellites = 14

  # --- HOW GPS SENDS DATA TO THE AUTOPILOT ---
  # In a real setup, a GPS module sends NMEA or binary sentences over UART.
  # In software, we structure this as a clean JSON/Dictionary telemetry packet:
  gps_packet = {
      "fix_type": 3,  # 3D Fix (Valid location lock)
      "lat": round(current_lat, 6),
      "lon": round(current_lon, 6),
      "alt": round(altitude, 2),
      "satellites_visible": satellites,
      "hdop": 0.9,  # Horizontal Dilution of Precision (lower is better)
      "timestamp": time.strftime("%H:%M:%S"),
  }

  print(
      f"[GPS -> Autopilot Packet] Time: {gps_packet['timestamp']} | Lat:"
      f" {gps_packet['lat']} | Lon: {gps_packet['lon']} | Sats:"
      f" {gps_packet['satellites_visible']}"
  )

  # Append coordinate to flight path tracker
  flight_path.append([gps_packet["lat"], gps_packet["lon"]])

  # --- GEO-TAGGING LOGIC ---
  # Simulate the AI detecting a survivor halfway through the flight path (at step 6)
  if i == 6:
    survivor_tag = {
        "object": "Survivor Detected",
        "lat": gps_packet["lat"],
        "lon": gps_packet["lon"],
        "alt": gps_packet["alt"],
        "confidence": 0.95,
        "timestamp": gps_packet["timestamp"],
    }
    geo_tags.append(survivor_tag)
    print(
        "\n >>> [GEO-TAG CREATED] Survivor found at Latitude:"
        f" {survivor_tag['lat']}, Longitude: {survivor_tag['lon']} <<<\n"
    )

  time.sleep(0.4)  # Simulate 2.5Hz GPS refresh rate

# --- PLOTTING ON A REAL GPS MAP ---
# 1. Draw the drone's flight path (blue line)
folium.PolyLine(
    flight_path,
    color="blue",
    weight=4,
    opacity=0.8,
    tooltip="Drone GPS Flight Path",
).add_to(drone_map)

# 2. Add starting point marker
folium.Marker(
    [start_lat, start_lon],
    popup="<b>Drone Launch Point</b>",
    icon=folium.Icon(color="green", icon="play", prefix="fa"),
).add_to(drone_map)

# 3. Add Geo-tagged Survivor markers on the map
for tag in geo_tags:
  popup_content = f"""
    <b>Target Type:</b> {tag['object']}<br>
    <b>Confidence:</b> {tag['confidence'] * 100}%<br>
    <b>Latitude:</b> {tag['lat']}<br>
    <b>Longitude:</b> {tag['lon']}<br>
    <b>Altitude:</b> {tag['alt']}m<br>
    <b>Time:</b> {tag['timestamp']}
    """
  folium.Marker(
      [tag["lat"], tag["lon"]],
      popup=folium.Popup(popup_content, max_width=300),
      icon=folium.Icon(color="red", icon="exclamation-triangle", prefix="fa"),
  ).add_to(drone_map)

# Save the map to an HTML file
map_filename = "real_gps_map.html"
drone_map.save(map_filename)
print(f"\n[SUCCESS] Real GPS Map generated successfully as '{map_filename}'!")
print(
    "Opening map automatically in your browser (or check your VS Code folder)."
)

# Automatically open the HTML map file
webbrowser.open("file://" + os.path.realpath(map_filename))