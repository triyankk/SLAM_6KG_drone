#!/usr/bin/env python3
"""Read Hesai JT16/JT26 point packets and publish MAVLink DISTANCE_SENSOR.

Sends a DISTANCE_SENSOR message with the minimum range observed and a
STATUSTEXT warning when the range is below the configured safety distance
(default 2.0 m).

Usage: python3 jt26_to_mavlink.py --jtport auto --mavport /dev/ttyACM1
"""

import argparse
import os
import select
import sys
import time
from pathlib import Path

from jt16_serial_probe import (
    choose_jt16_port,
    open_raw_serial,
    consume_packets,
    extract_point_samples,
    PacketStats,
)

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--jtport', default='auto', help='JT16/JT26 serial port or auto')
    p.add_argument('--baud', type=int, default=3000000)
    p.add_argument('--mavport', default='/dev/ttyACM1', help='MAVLink port to send DISTANCE_SENSOR')
    p.add_argument('--rate', type=float, default=5.0, help='Publish rate (Hz)')
    p.add_argument('--safety-m', type=float, default=2.0, help='Safety distance in meters')
    p.add_argument('--sensor-id', type=int, default=20, help='DISTANCE_SENSOR id')
    return p.parse_args()


def open_jt(port_arg, baud):
    port = choose_jt16_port(port_arg)
    if not os.path.exists(port):
        raise SystemExit(f'{port} does not exist')
    fd = open_raw_serial(port, baud)
    return fd


def main():
    args = parse_args()

    if mavutil is None:
        print('pymavlink not available; please install pymavlink')
        sys.exit(1)

    print('Opening JT port...')
    try:
        fd = open_jt(args.jtport, args.baud)
    except Exception as e:
        print('Failed to open JT port:', e)
        sys.exit(2)

    print('Connecting to MAVLink on', args.mavport)
    master = mavutil.mavlink_connection(args.mavport, baud=115200)
    # wait for heartbeat
    master.wait_heartbeat(timeout=5)

    buffer = bytearray()
    stats = PacketStats()
    last_pub = 0.0
    last_warn = False

    try:
        while True:
            readable, _, _ = select.select([fd], [], [], 1.0 / max(args.rate * 2, 1.0))
            if readable:
                try:
                    chunk = os.read(fd, 8192)
                except BlockingIOError:
                    continue
                if chunk:
                    buffer.extend(chunk)
                    # consume_packets will call update_point_stats; we will parse packets ourselves
                    consume_packets(buffer, stats)

            now = time.time()
            if now - last_pub >= 1.0 / max(args.rate, 1.0):
                # use stats.last_min_distance_m as currently observed min
                dist = float(stats.last_min_distance_m or 0.0)
                if dist <= 0.0:
                    # no valid reading yet
                    last_pub = now
                    continue

                # send DISTANCE_SENSOR (boot_ms, min_cm, max_cm, current_cm, type, id, orientation, covariance)
                boot_ms = int((time.time() - os.getpid()) * 1000) & 0xFFFFFFFF
                current_cm = int(round(dist * 100.0))
                min_cm = 2  # small positive value
                max_cm = 60000
                master.mav.distance_sensor_send(
                    boot_ms,
                    min_cm,
                    max_cm,
                    current_cm,
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    args.sensor_id,
                    0,
                    0,
                )

                # warning STATUSTEXT when too close
                if dist < args.safety_m:
                    if not last_warn:
                        text = f'OBSTACLE WARNING: {dist:.2f} m (<{args.safety_m}m)'
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, text.encode('utf-8'))
                        last_warn = True
                else:
                    if last_warn:
                        text = f'OBSTACLE CLEARED: {dist:.2f} m'
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text.encode('utf-8'))
                        last_warn = False

                last_pub = now

    except KeyboardInterrupt:
        print('Interrupted, exiting')
    finally:
        try:
            os.close(fd)
        except Exception:
            pass


if __name__ == '__main__':
    main()
