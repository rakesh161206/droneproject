"""
Drone Camera Capture & Dashboard Upload
----------------------------------------
Captures a photo from a connected camera (USB webcam, drone camera exposed
as a video device, or a Raspberry Pi USB camera) and uploads it to a
dashboard server over HTTP.

Requirements:
    pip install opencv-python requests

Usage:
    python capture_and_send.py
    python capture_and_send.py --camera 0 --url http://192.168.1.50:5000/upload
    python capture_and_send.py --interval 10   # capture+send every 10 seconds (loop mode)

Notes for drone use:
- If your camera is accessed over a network stream (RTSP/UDP) instead of a
  local device index, pass that URL as --camera, e.g.:
      --camera "rtsp://192.168.1.1:8554/live"
  OpenCV's VideoCapture supports RTSP/HTTP streams directly.
- If you're on a Raspberry Pi with the official camera module (CSI, not USB),
  OpenCV's VideoCapture usually won't work well -- use `picamera2` instead.
  Ask if you need that version.
"""

import argparse
import platform
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import requests


def create_video_capture(camera_source):
    """Prefer DirectShow on Windows to avoid MSMF grab errors."""
    if isinstance(camera_source, str):
        return cv2.VideoCapture(camera_source)

    if platform.system() == "Windows":
        cap = cv2.VideoCapture(camera_source, cv2.CAP_DSHOW)
        if cap.isOpened():
            return cap
        return cv2.VideoCapture(camera_source)

    return cv2.VideoCapture(camera_source)


def capture_image(camera_source, save_path: Path) -> bool:
    """Capture a single frame from the camera and save it to disk."""
    cap = create_video_capture(camera_source)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera source: {camera_source}")
        return False

    # Give the camera a moment to warm up / adjust exposure
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("[ERROR] Failed to capture frame from camera.")
        return False

    save_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(save_path), frame)
    print(f"[OK] Image captured and saved to {save_path}")
    return True


def send_to_dashboard(image_path: Path, url: str, extra_fields: dict | None = None) -> bool:
    """Upload the captured image to the dashboard via HTTP POST (multipart/form-data)."""
    if not image_path.exists():
        print(f"[ERROR] Image file not found: {image_path}")
        return False

    try:
        with open(image_path, "rb") as f:
            files = {"image": (image_path.name, f, "image/jpeg")}
            data = extra_fields or {}
            response = requests.post(url, files=files, data=data, timeout=10)

        if response.status_code == 200:
            print(f"[OK] Image sent to dashboard successfully ({url})")
            return True
        else:
            print(f"[ERROR] Dashboard responded with status {response.status_code}: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Failed to reach dashboard: {e}")
        return False


def run_once(camera_source, url, out_dir: Path):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    image_path = out_dir / f"capture_{timestamp}.jpg"

    if capture_image(camera_source, image_path):
        send_to_dashboard(
            image_path,
            url,
            extra_fields={"timestamp": timestamp, "source": "drone_cam"},
        )


def main():
    parser = argparse.ArgumentParser(description="Capture drone camera image and send to dashboard.")
    parser.add_argument(
        "--camera",
        default="0",
        help="Camera index (e.g. 0, 1) or stream URL (e.g. rtsp://...). Default: 0",
    )
    parser.add_argument(
        "--url",
        default="http://localhost:8000/upload",
        help="Dashboard upload endpoint. Default: http://localhost:8000/upload",
    )
    parser.add_argument(
        "--out-dir",
        default="./captures",
        help="Local folder to save captured images. Default: ./captures",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0,
        help="If set (seconds), repeatedly capture+send on this interval. Default: run once.",
    )
    args = parser.parse_args()

    # Camera index vs stream URL
    camera_source = int(args.camera) if args.camera.isdigit() else args.camera
    out_dir = Path(args.out_dir)

    if args.interval > 0:
        print(f"Starting loop mode: capturing every {args.interval}s. Press Ctrl+C to stop.")
        try:
            while True:
                run_once(camera_source, args.url, out_dir)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nStopped.")
            sys.exit(0)
    else:
        run_once(camera_source, args.url, out_dir)


if __name__ == "__main__":
    main()

