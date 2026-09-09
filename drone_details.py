"""
AEGIS SAR Drone — Specifications, Data Models & Failsafes
Problem Statement 26177 (Qualcomm / SIH - Robotics & Drones Theme)
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional, Dict, Any, List


class FlightMode(str, Enum):
    MANUAL = "MANUAL"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    RTL = "RTL"
    LAND = "LAND"
    LOITER = "LOITER"
    BRAKE = "BRAKE"


class IncidentType(str, Enum):
    SURVIVOR = "Survivor"
    FIRE = "Fire Hazard"
    FLOOD = "Flood Inundation"
    OBSTACLE = "Debris / Hazard"


class IncidentSeverity(str, Enum):
    CRITICAL = "CRITICAL"
    HIGH = "HIGH"
    MODERATE = "MODERATE"
    LOW = "LOW"


@dataclass
class DroneHardwareSpec:
    """Hardware specification of the SAR Drone."""
    callsign: str = "AEGIS-SAR-01"
    model: str = "Hexacopter SAR Tactical Pro"
    frame_size_mm: int = 680
    max_payload_kg: float = 2.5
    empty_weight_kg: float = 3.2
    max_flight_time_min: int = 35
    max_operating_speed_ms: float = 16.0
    max_altitude_m: float = 120.0
    battery_capacity_mah: int = 16000
    battery_cells: int = 6  # 6S LiPo
    battery_nominal_voltage: float = 22.2
    companion_computer: str = "NVIDIA Jetson Orin Nano / Raspberry Pi 4"
    flight_controller: str = "Pixhawk 6C / ArduPilot Copter 4.4"
    sensors: List[str] = field(default_factory=lambda: [
        "Sony IMX477 4K RGB Camera",
        "FLIR Lepton 3.5 Radiometric Thermal Camera",
        "u-blox NEO-M9N Dual GNSS / RTK",
        "TFmini Plus LiDAR Altimeter",
        "Qualcomm Wi-Fi 6 / 5G Sub-6 Mission Link"
    ])


@dataclass
class SafetyThresholds:
    """Pre-flight and in-flight safety guardrails."""
    min_takeoff_battery_pct: float = 30.0
    warning_battery_pct: float = 25.0
    critical_rtl_battery_pct: float = 20.0
    emergency_land_battery_pct: float = 10.0
    min_gps_satellites: int = 8
    min_gps_fix_type: int = 3  # 3D Fix
    max_hdop: float = 1.5
    geofence_radius_m: float = 3000.0
    max_operating_wind_ms: float = 12.0
    link_loss_timeout_s: float = 10.0


@dataclass
class TelemetryPacket:
    """Real-time drone telemetry state snapshot."""
    timestamp: float = field(default_factory=time.time)
    lat: float = 18.5225
    lon: float = 73.8567
    alt: float = 0.0          # Relative altitude (meters)
    alt_msl: float = 560.0     # Mean Sea Level altitude (meters)
    ground_speed: float = 0.0  # m/s
    vertical_speed: float = 0.0 # m/s
    heading: float = 0.0       # degrees (0-360)
    battery_pct: float = 100.0 # percentage (0-100)
    battery_voltage: float = 25.2
    battery_current_a: float = 0.0
    gps_fix_type: int = 3
    satellites_visible: int = 14
    hdop: float = 0.8
    flight_mode: FlightMode = FlightMode.GUIDED
    is_armed: bool = False
    is_flying: bool = False
    link_latency_ms: int = 32
    packet_loss_pct: float = 0.02
    signal_bars: int = 4
    vision_ai_status: str = "ONLINE"
    mission_time_s: int = 0

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["flight_mode"] = self.flight_mode.value if isinstance(self.flight_mode, FlightMode) else self.flight_mode
        return d


@dataclass
class DetectionRecord:
    """AI Vision detection record tagged with GPS coordinates."""
    id: Optional[int] = None
    mission_id: str = "DEFAULT"
    label: str = "Survivor"
    confidence: float = 0.95
    severity: IncidentSeverity = IncidentSeverity.HIGH
    lat: float = 18.5225
    lon: float = 73.8567
    alt: float = 35.0
    image_path: Optional[str] = None
    thermal_temp_c: Optional[float] = 37.2
    bbox: Optional[List[int]] = None  # [x1, y1, x2, y2]
    timestamp: float = field(default_factory=time.time)
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["severity"] = self.severity.value if isinstance(self.severity, IncidentSeverity) else self.severity
        return d


# --------------------------------------------------------------------------
# Geodetic & Navigation Utilities
# --------------------------------------------------------------------------

def calculate_distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance between two coordinates in meters."""
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def calculate_bearing_degrees(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from coordinate 1 to coordinate 2 in degrees (0-360)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) -
         math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda))
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0


def offset_coordinate(lat: float, lon: float, distance_m: float, bearing_deg: float) -> tuple[float, float]:
    """Calculate target coordinate given a starting point, distance (meters), and bearing (degrees)."""
    R = 6371000.0
    d_div_r = distance_m / R
    bearing_rad = math.radians(bearing_deg)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    target_lat = math.asin(
        math.sin(lat_rad) * math.cos(d_div_r) +
        math.cos(lat_rad) * math.sin(d_div_r) * math.cos(bearing_rad)
    )
    target_lon = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(d_div_r) * math.cos(lat_rad),
        math.cos(d_div_r) - math.sin(lat_rad) * math.sin(target_lat)
    )
    return math.degrees(target_lat), math.degrees(target_lon)


HARDWARE_SPEC = DroneHardwareSpec()
SAFETY_THRESHOLDS = SafetyThresholds()
