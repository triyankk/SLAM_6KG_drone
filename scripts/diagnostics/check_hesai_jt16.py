#!/usr/bin/env python3
"""Diagnostic script for Hesai JT16 Mini LiDAR.
Checks both Serial (RS485) and Ethernet connectivity.
"""

import argparse
import os
import socket
import sys
import time
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.lidar import LidarReader, find_lidar_port

LOG_FILE = REPO_ROOT / "logs" / "hesai_check.log"

def log(msg):
    print(msg)
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}\n")

def check_network():
    log("--- Network Check ---")
    try:
        # List network interfaces
        result = subprocess.run(["ip", "-br", "addr", "show"], capture_output=True, text=True)
        log("Network interfaces:")
        log(result.stdout)

        # Get default gateway/IP
        result = subprocess.run(["hostname", "-I"], capture_output=True, text=True)
        log(f"Jetson IPs: {result.stdout.strip()}")
    except Exception as e:
        log(f"Error checking network: {e}")

def check_serial():
    log("--- Serial Check ---")
    port = find_lidar_port()
    if port:
        log(f"Detected potential JT16 port: {port}")
        if "jt16_usb" in port:
            log("Found persistent symlink /dev/jt16_usb")
        
        try:
            # Check if port is accessible
            if os.access(port, os.R_OK):
                log(f"Port {port} is readable.")
            else:
                log(f"Port {port} is NOT readable (permission issue?).")
                
            # Try to open and read for a few seconds
            log(f"Attempting to read from {port} at 3000000 baud...")
            reader = LidarReader.open(port, 3000000)
            snap = reader.poll(duration_s=3.0)
            reader.close()
            
            if snap.point_packets > 0:
                log(f"PASS: Received {snap.point_packets} point packets.")
                log(f"Last distance: {snap.median_distance_m:.2f}m")
                return True
            else:
                log("FAIL: Opened port but received 0 point packets. Check power/wiring.")
        except Exception as e:
            log(f"Error opening serial port: {e}")
    else:
        log("FAIL: No JT16 serial port found. Check USB-RS485 adapter.")
    return False

def main():
    parser = argparse.ArgumentParser(description="Hesai JT16 Mini LiDAR Diagnostic")
    parser.add_argument("--ip", help="LiDAR IP to ping (if Ethernet)")
    args = parser.parse_args()

    os.makedirs(REPO_ROOT / "logs", exist_ok=True)
    if LOG_FILE.exists():
        os.remove(LOG_FILE)

    log("Starting Hesai JT16 Mini Diagnostic...")
    
    check_network()
    
    if args.ip:
        log(f"Checking LiDAR IP: {args.ip}")
        try:
            result = subprocess.run(["ping", "-c", "3", "-W", "2", args.ip], capture_output=True, text=True)
            if result.returncode == 0:
                log(f"PASS: LiDAR at {args.ip} is reachable.")
            else:
                log(f"FAIL: LiDAR at {args.ip} is NOT reachable.")
        except Exception as e:
            log(f"Error pinging LiDAR: {e}")

    serial_ok = check_serial()
    
    if serial_ok:
        log("\nSUMMARY: JT16 LiDAR appears healthy on Serial.")
    else:
        log("\nSUMMARY: JT16 LiDAR check FAILED or inconclusive.")

if __name__ == "__main__":
    main()
