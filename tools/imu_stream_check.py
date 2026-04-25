#!/usr/bin/env python3
# Run:
#   python3 imu_stream_check.py --port auto --baud auto

import argparse
import time

from imu_serial_common import (
    DEFAULT_BAUDS,
    Im10aTelemetry,
    choose_port,
    detect_brltty_processes,
    im10a_usb_present,
    open_serial,
    parse_jy901_stream,
    probe_baud,
    safe_read,
    telemetry_orientation,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Health-check the IM10A serial stream and report whether the sensor looks healthy enough for SLAM integration."
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", default="auto")
    parser.add_argument("--baud-candidates", type=int, nargs="*", default=DEFAULT_BAUDS)
    parser.add_argument("--scan-seconds", type=float, default=1.0)
    parser.add_argument("--run-seconds", type=float, default=4.0)
    return parser.parse_args()


def main():
    args = parse_args()
    print(f"IM10A USB present: {'yes' if im10a_usb_present() else 'no'}")
    brltty = detect_brltty_processes()
    if brltty:
        print("Warning: brltty/xbrlapi processes are still running and may steal /dev/ttyUSB0:")
        for line in brltty:
            print(f"- {line}")

    port = choose_port(args.port)
    if args.baud == "auto":
        probe = probe_baud(port, args.baud_candidates, args.scan_seconds)
        baud = probe.baud
        print(
            f"Auto baud result: port={port} baud={probe.baud} frames={probe.valid_frame_count} "
            f"bytes={probe.byte_count} nonzero_ratio={probe.nonzero_ratio:.2f} sample_hex={probe.sample_hex[:80]}"
        )
    else:
        baud = int(args.baud)

    telemetry = Im10aTelemetry()
    packet_buffer = bytearray()
    total_bytes = 0
    started_s = time.time()
    ser = open_serial(port, baud)
    deadline = time.time() + args.run_seconds
    try:
        while time.time() < deadline:
            chunk = safe_read(ser, 4096)
            if not chunk:
                continue
            total_bytes += len(chunk)
            packet_buffer.extend(chunk)
            parse_jy901_stream(packet_buffer, telemetry)
    finally:
        ser.close()

    elapsed = max(time.time() - started_s, 1e-6)
    orientation = telemetry_orientation(telemetry)

    print()
    print("IMU stream report")
    print("=================")
    print(f"port={port}")
    print(f"baud={baud}")
    print(f"bytes={total_bytes}")
    print(f"bytes_per_s={total_bytes / elapsed:.1f}")
    print(f"valid_frames={telemetry.valid_frame_count}")
    print(f"frame_counts={telemetry.frame_counts}")
    print(f"last_frame={telemetry.last_frame_name or 'none'}")
    if telemetry.acc_g is not None:
        print(f"acc_g={telemetry.acc_g}")
    if telemetry.gyro_deg_s is not None:
        print(f"gyro_deg_s={telemetry.gyro_deg_s}")
    if telemetry.angle_deg is not None:
        print(f"angle_deg={telemetry.angle_deg}")
    if telemetry.quaternion is not None:
        print(f"quaternion={telemetry.quaternion}")
    if telemetry.mag_raw is not None:
        print(f"mag_raw={telemetry.mag_raw}")
    if telemetry.pressure_pa is not None:
        print(f"pressure_pa={telemetry.pressure_pa}")
    if telemetry.altitude_m is not None:
        print(f"altitude_m={telemetry.altitude_m:.2f}")
    if orientation is not None:
        print(
            "orientation="
            f"({orientation.roll_deg:+.2f}, {orientation.pitch_deg:+.2f}, {orientation.yaw_deg:+.2f})deg"
            f" source={orientation.source}"
        )

    if telemetry.valid_frame_count == 0:
        print()
        print("Health verdict: no valid IM10A packets were decoded.")
        print("Likely causes:")
        print("- wrong baud")
        print("- IMU output content/firmware configuration changed")
        print("- brltty or another process still touched the CH341 interface")
        return

    if orientation is None:
        print()
        print("Health verdict: IM10A packets are alive, but pose fields were not decoded yet.")
        print("The parser may need additional frame types enabled.")
        return

    print()
    print("Health verdict: IM10A stream is healthy and ready for the next SLAM integration step.")


if __name__ == "__main__":
    main()
