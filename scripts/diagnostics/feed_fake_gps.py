#!/usr/bin/env python3

import sys
import time
import math
import argparse
from pathlib import Path

# Add src to path
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from pymavlink import mavutil

from slam_core.fc_config import current_gps_week_time


def gps_input_ignore_flags() -> int:
    return (
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT
    )


def set_param(master, name: str, value: float) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

def main():
    parser = argparse.ArgumentParser(description="Feed a fake MAVLink GPS_INPUT fix to the drone.")
    parser.add_argument(
        "--i-understand-this-spoofs-gps",
        action="store_true",
        help="Required safety acknowledgement. This script can make the FC believe fake GPS is healthy.",
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port for Cube connection")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--gps-id", type=int, default=1, choices=[0, 1], help="GPS_INPUT id: 0=GPS1, 1=GPS2")
    parser.add_argument("--lat", type=float, default=37.7749, help="Latitude (default: SF)")
    parser.add_argument("--lon", type=float, default=-122.4194, help="Longitude")
    parser.add_argument("--alt", type=float, default=10.0, help="Altitude MSL")
    parser.add_argument("--duration", type=float, default=120.0, help="Duration in seconds")
    args = parser.parse_args()

    if not args.i_understand_this_spoofs_gps:
        raise SystemExit(
            "Refusing to run: this diagnostic spoofs GPS_INPUT and can re-enable GPS2. "
            "Re-run with --i-understand-this-spoofs-gps only on a bench with props removed."
        )

    print(f"Connecting to Cube on {args.port} at {args.baud}...")
    master = mavutil.mavlink_connection(args.port, baud=args.baud)
    master.wait_heartbeat()
    print("Heartbeat received!")

    # Parameters to enable GPS spoofing if not already set. Keep VISO off for
    # this diagnostic so prearm checks are not masked by "VisOdom out of memory".
    print("Ensuring FC is configured for MAVLink GPS spoofing...")
    if args.gps_id == 0:
        set_param(master, "GPS_TYPE", 14)
    else:
        set_param(master, "GPS2_TYPE", 14)
        set_param(master, "GPS_AUTO_SWITCH", 1)
    set_param(master, "VISO_TYPE", 0)
    print("If GPS_TYPE/GPS2_TYPE/VISO_TYPE changed, reboot the Cube once, then run this again.")
    
    start_time = time.time()
    duration = max(args.duration, 1.0)
    
    print(f"Feeding fake GPS{args.gps_id + 1} fix at {args.lat}, {args.lon} for {duration:.0f} seconds...")
    
    try:
        while time.time() - start_time < duration:
            now_us = int(time.time() * 1e6)
            gps_week, gps_week_ms = current_gps_week_time()
            
            # Send GPS_INPUT (ID 232)
            master.mav.gps_input_send(
                now_us,
                args.gps_id,
                gps_input_ignore_flags(),
                gps_week_ms, # time_week_ms
                gps_week, # time_week
                3, # fix_type (3D fix)
                int(args.lat * 1e7),
                int(args.lon * 1e7),
                args.alt,
                1.0, # hdop
                1.0, # vdop
                0, 0, 0, # velocities
                1.0, # speed_accuracy
                1.0, # horiz_accuracy
                1.0, # vert_accuracy
                14,  # satellites_visible
            )
            
            # Occasionally send Home Position if needed (usually FC handles this once fix is stable)
            if int(time.time()) % 10 == 0:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                    0, 0, 0, 0, 0, args.lat, args.lon, args.alt
                )

            time.sleep(0.2) # 5Hz GPS rate
            
            elapsed = int(time.time() - start_time)
            if elapsed % 10 == 0:
                print(f"Elapsed: {elapsed}s / {duration}s")
                
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    
    print("Done feeding fake GPS.")

if __name__ == "__main__":
    main()
