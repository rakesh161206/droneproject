"""
AEGIS SAR Mission Persistence Layer
SQLite database for missions, high-rate telemetry, and geo-tagged detections.
"""

from __future__ import annotations
import os
import sqlite3
import time
from typing import List, Dict, Any, Optional
from pathlib import Path

DB_FILE = Path(__file__).parent / "aegis_sar.db"


class Database:
    def __init__(self, db_path: Path = DB_FILE):
        self.db_path = str(db_path)
        self.init_db()

    def get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        return conn

    def init_db(self):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            
            # Missions table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS missions (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                status TEXT NOT NULL,
                start_time REAL NOT NULL,
                end_time REAL,
                total_distance_m REAL DEFAULT 0.0,
                survivors_found INTEGER DEFAULT 0,
                notes TEXT
            );
            """)

            # Telemetry logs table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS telemetry_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT NOT NULL,
                timestamp REAL NOT NULL,
                lat REAL NOT NULL,
                lon REAL NOT NULL,
                alt REAL NOT NULL,
                ground_speed REAL,
                heading REAL,
                battery_pct REAL,
                flight_mode TEXT,
                satellites INTEGER,
                FOREIGN KEY (mission_id) REFERENCES missions(id)
            );
            """)

            # Detections table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS detections (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT NOT NULL,
                timestamp REAL NOT NULL,
                label TEXT NOT NULL,
                confidence REAL NOT NULL,
                severity TEXT NOT NULL,
                lat REAL NOT NULL,
                lon REAL NOT NULL,
                alt REAL NOT NULL,
                image_path TEXT,
                thermal_temp_c REAL,
                bbox_json TEXT,
                notes TEXT,
                FOREIGN KEY (mission_id) REFERENCES missions(id)
            );
            """)

            # System events / audit log
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS audit_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                event_type TEXT NOT NULL,
                message TEXT NOT NULL,
                details TEXT
            );
            """)

            conn.commit()

    def start_mission(self, mission_id: str, name: str) -> Dict[str, Any]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            now = time.time()
            cursor.execute(
                "INSERT INTO missions (id, name, status, start_time) VALUES (?, ?, 'ACTIVE', ?)",
                (mission_id, name, now)
            )
            conn.commit()
            return {"id": mission_id, "name": name, "status": "ACTIVE", "start_time": now}

    def end_mission(self, mission_id: str):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            now = time.time()
            cursor.execute(
                "UPDATE missions SET status = 'COMPLETED', end_time = ? WHERE id = ?",
                (now, mission_id)
            )
            conn.commit()

    def get_missions(self) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM missions ORDER BY start_time DESC")
            return [dict(row) for row in cursor.fetchall()]

    def log_telemetry(self, mission_id: str, t: Dict[str, Any]):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO telemetry_logs 
                (mission_id, timestamp, lat, lon, alt, ground_speed, heading, battery_pct, flight_mode, satellites)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                mission_id,
                t.get("timestamp", time.time()),
                t.get("lat", 0.0),
                t.get("lon", 0.0),
                t.get("alt", 0.0),
                t.get("ground_speed", 0.0),
                t.get("heading", 0.0),
                t.get("battery_pct", 100.0),
                t.get("flight_mode", "GUIDED"),
                t.get("satellites_visible", 14)
            ))
            conn.commit()

    def save_detection(self, det: Dict[str, Any]) -> int:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            import json
            bbox_str = json.dumps(det.get("bbox")) if det.get("bbox") else None
            cursor.execute("""
                INSERT INTO detections
                (mission_id, timestamp, label, confidence, severity, lat, lon, alt, image_path, thermal_temp_c, bbox_json, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                det.get("mission_id", "DEFAULT"),
                det.get("timestamp", time.time()),
                det.get("label", "Survivor"),
                det.get("confidence", 0.95),
                det.get("severity", "HIGH"),
                det.get("lat", 0.0),
                det.get("lon", 0.0),
                det.get("alt", 0.0),
                det.get("image_path"),
                det.get("thermal_temp_c", 37.0),
                bbox_str,
                det.get("notes", "")
            ))
            det_id = cursor.lastrowid
            
            # Increment survivor count in mission
            cursor.execute(
                "UPDATE missions SET survivors_found = survivors_found + 1 WHERE id = ?",
                (det.get("mission_id", "DEFAULT"),)
            )
            conn.commit()
            return det_id

    def get_detections(self, mission_id: Optional[str] = None) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            import json
            if mission_id:
                cursor.execute("SELECT * FROM detections WHERE mission_id = ? ORDER BY timestamp DESC", (mission_id,))
            else:
                cursor.execute("SELECT * FROM detections ORDER BY timestamp DESC LIMIT 50")
            rows = cursor.fetchall()
            results = []
            for r in rows:
                item = dict(r)
                if item.get("bbox_json"):
                    try:
                        item["bbox"] = json.loads(item["bbox_json"])
                    except Exception:
                        item["bbox"] = None
                results.append(item)
            return results

    def log_event(self, event_type: str, message: str, details: str = ""):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(
                "INSERT INTO audit_logs (timestamp, event_type, message, details) VALUES (?, ?, ?, ?)",
                (time.time(), event_type, message, details)
            )
            conn.commit()


db = Database()
