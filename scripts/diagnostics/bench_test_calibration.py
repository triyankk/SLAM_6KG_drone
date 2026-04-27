#!/usr/bin/env python3
"""Bench test script to verify SLAM calibration sequence without real GPS.
Stops background services, feeds dummy GPS, runs monitor, and restores services on exit.
"""

import argparse
import os
import sys
import time
import subprocess
import threading
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink not installed.")
    sys.exit(1)

def run_cmd(cmd, check=True):
    try:
        subprocess.run(cmd, shell=True, check=check)
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {cmd}\nError: {e}")

def send_fake_gps_stream(master, lat, lon, alt, stop_event):
    """Continuously sends GPS_INPUT to simulate a 3D lock on both GPS 1 and GPS 2."""
    print(f"Starting GPS simulation at {lat}, {lon}...")
    while not stop_event.is_set():
        try:
            now_us = int(time.time() * 1e6)
            # Send to GPS 1 (ID 0) and GPS 2 (ID 1)
            for gps_id in [0, 1]:
                master.mav.gps_input_send(
                    now_us,
                    gps_id, # gps_id
                    0, # ignore_flags
                    0, # time_week_ms
                    0, # time_week
                    3, # fix_type (3 = 3D Fix)
                    int(lat * 1e7),
                    int(lon * 1e7),
                    float(alt),
                    1.0, 1.0, # HDOP, VDOP
                    0, 0, 0, # Velocity N, E, D
                    0.1, 0.1, 0.1, # Accuracies
                    15 # Satellites visible
                )
        except:
            pass
        time.sleep(0.2) # 5Hz

def set_dummy_gps_origin(master, lat, lon, alt):
    print(f"Setting EKF origin to: {lat}, {lon}, {alt}m")
    master.mav.set_gps_global_origin_send(
        1,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt * 1000)
    )
    master.mav.set_home_position_send(
        1,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt * 1000),
        0, 0, 0, [0, 0, 0, 0], 0, 0, 0
    )

def main():
    parser = argparse.ArgumentParser(description="Manual Bench Test for SLAM Calibration")
    parser.add_argument("--lat", type=float, default=37.4123, help="Dummy Latitude")
    parser.add_argument("--lon", type=float, default=-122.0678, help="Dummy Longitude")
    parser.add_argument("--alt", type=float, default=10.0, help="Dummy Altitude (m)")
    parser.add_argument("--mavport", default="/dev/ttyACM0", help="MAVLink port for Cube")
    args = parser.parse_args()

    print("--- STARTING BENCH TEST WITH FAKE GPS FIX ---")
    
    stop_gps = threading.Event()

    # 1. Stop Services
    print("\n[1/4] Stopping background services...")
    run_cmd(f"sudo {REPO_ROOT}/scripts/manage_flight_stack.sh stop")

    # 2. Connect and Start GPS Stream
    print("\n[2/4] Connecting to Cube...")
    try:
        master = mavutil.mavlink_connection(args.mavport, baud=115200)
        master.wait_heartbeat(timeout=5)
        
        # Set Origin once
        set_dummy_gps_origin(master, args.lat, args.lon, args.alt)
        
        # Start continuous Fix stream in background
        gps_thread = threading.Thread(target=send_fake_gps_stream, args=(master, args.lat, args.lon, args.alt, stop_gps))
        gps_thread.daemon = True
        gps_thread.start()
        
    except Exception as e:
        print(f"MAVLink Connection Error: {e}")
        return

    # 3. Run Calibration Monitor in Foreground + Log Listener
    print("\n[3/4] Starting Calibration Monitor (Press Ctrl+C to stop)...")
    print("Listening for MAVLink error logs...")
    print("-" * 50)
    
    cmd_monitor = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "calibration" / "brake_slam_calibration.py"),
        "--config", str(REPO_ROOT / "config" / "autostart.yaml"),
        "--disable-motion"
    ]
    
    # Start monitor process
    proc = subprocess.Popen(cmd_monitor)
    
    try:
        while proc.poll() is None:
            # Check for MAVLink messages (logs)
            msg = master.recv_match(type='STATUSTEXT', blocking=False)
            if msg:
                print(f" [MAVLink Log]: {msg.text}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nBench test interrupted.")
        proc.terminate()
    finally:
        stop_gps.set()

    # 4. Restore Services
    print("\n[4/4] Restoring background services...")
    run_cmd(f"sudo {REPO_ROOT}/scripts/manage_flight_stack.sh start")
    
    print("\n--- BENCH TEST COMPLETE ---")

if __name__ == "__main__":
    main()
