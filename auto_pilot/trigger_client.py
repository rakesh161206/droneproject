"""
Trigger client — this is what your FRIEND runs.

It sends a "fly" command to the drone over the network and then polls
STATUS every couple seconds so they can watch the live GPS position
change on their own screen.

Usage:
    python trigger_client.py <drone_ip> FLY 12.9716,77.5946,20
    python trigger_client.py <drone_ip> STATUS
    python trigger_client.py <drone_ip> RTL
    python trigger_client.py <drone_ip> LAND

Example for your demo:
    python trigger_client.py 192.168.1.42 FLY 12.9716,77.5946,20
"""

import socket
import sys
import time

PORT = 5760


def send_command(drone_ip: str, command: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(10)
        s.connect((drone_ip, PORT))
        s.sendall((command + "\n").encode())
        return s.recv(1024).decode().strip()


def watch_gps(drone_ip: str, seconds: int = 60, interval: int = 2):
    print(f"Watching live GPS from {drone_ip} for {seconds}s (Ctrl+C to stop)...")
    end = time.time() + seconds
    while time.time() < end:
        try:
            print(send_command(drone_ip, "STATUS"))
        except Exception as e:
            print(f"  (status check failed: {e})")
        time.sleep(interval)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    drone_ip = sys.argv[1]
    command = " ".join(sys.argv[2:])

    reply = send_command(drone_ip, command)
    print("Drone replied:", reply)

    if command.upper().startswith("FLY"):
        watch_gps(drone_ip)
