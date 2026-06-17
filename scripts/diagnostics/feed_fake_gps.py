#!/usr/bin/env python3
"""Temporarily feed a bench fake GPS fix through MAVLink GPS_INPUT.

This is a diagnostic helper, not a flight feature. The safest way to use it
with the legacy bridge running is the default UDP path:

    python3 scripts/diagnostics/feed_fake_gps.py --i-understand-this-spoofs-gps

The script sends GPS_INPUT to the bridge's QGC uplink UDP port. The bridge then
forwards the MAVLink packets to the Cube while it keeps owning the serial port.
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pymavlink import mavutil

from slam_core.fc_config import current_gps_week_time


DEFAULT_CONNECTION = "udpout:127.0.0.1:14555"


def parse_gps_ids(value: str) -> list[int]:
    value = value.strip().lower()
    if value == "both":
        return [0, 1]
    if value in {"0", "gps1"}:
        return [0]
    if value in {"1", "gps2"}:
        return [1]
    raise argparse.ArgumentTypeError("use 0, 1, gps1, gps2, or both")


def gps_input_ignore_flags() -> int:
    # Do not use GPS_INPUT_IGNORE_FLAG_YAW: older pymavlink builds do not expose
    # it, and this message signature does not include a yaw field anyway.
    return 0


def send_statustext(master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_NOTICE) -> None:
    try:
        master.mav.statustext_send(severity, text[:50].encode("utf-8", errors="ignore"))
    except Exception as exc:
        print(f"STATUSTEXT send failed: {exc}")


def set_param(master, name: str, value: float) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    print(f"PARAM_SET {name}={value}")


def set_origin_and_home(master, lat: float, lon: float, alt_m: float) -> None:
    lat_i = int(round(lat * 1e7))
    lon_i = int(round(lon * 1e7))
    alt_mm = int(round(alt_m * 1000.0))

    try:
        master.mav.set_gps_global_origin_send(
            master.target_system,
            lat_i,
            lon_i,
            alt_mm,
            int(time.time() * 1e6),
        )
    except TypeError:
        master.mav.set_gps_global_origin_send(master.target_system, lat_i, lon_i, alt_mm)

    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        0,
        0,
        0,
        0,
        lat_i,
        lon_i,
        alt_m,
    )


def send_fake_gps_input(
    master,
    gps_ids: Iterable[int],
    lat: float,
    lon: float,
    alt_m: float,
    sats: int,
    hacc_m: float,
    vacc_m: float,
    speed_acc_m_s: float,
) -> None:
    now_s = time.time()
    gps_week, gps_week_ms = current_gps_week_time(now_s)
    for gps_id in gps_ids:
        master.mav.gps_input_send(
            int(now_s * 1e6),
            int(gps_id),
            gps_input_ignore_flags(),
            int(gps_week_ms),
            int(gps_week),
            3,
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            float(alt_m),
            0.8,
            1.2,
            0.0,
            0.0,
            0.0,
            float(speed_acc_m_s),
            float(hacc_m),
            float(vacc_m),
            int(sats),
        )


def listen_statustext(stop_event: threading.Event, udp_port: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", udp_port))
    except OSError as exc:
        print(f"Unable to listen on UDP {udp_port}: {exc}")
        return

    sock.settimeout(0.5)
    parser = mavlink2.MAVLink(None)
    seen_recent: dict[str, float] = {}

    while not stop_event.is_set():
        try:
            payload, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break

        for byte_value in payload:
            try:
                msg = parser.parse_char(bytes([byte_value]))
            except Exception:
                msg = None
            if msg is None or msg.get_type() != "STATUSTEXT":
                continue
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            text = text.rstrip("\x00")
            now_s = time.time()
            if now_s - seen_recent.get(text, 0.0) < 2.0:
                continue
            seen_recent[text] = now_s
            print(f"GCS: {text}")

    sock.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Feed a temporary MAVLink GPS_INPUT fix for bench/prearm diagnostics. "
            "Use props-off only; this can make the FC believe GPS is healthy."
        )
    )
    parser.add_argument(
        "--i-understand-this-spoofs-gps",
        action="store_true",
        help="Required acknowledgement before sending fake GPS.",
    )
    parser.add_argument(
        "--connection",
        default=DEFAULT_CONNECTION,
        help=(
            "MAVLink connection string. Default sends through the running legacy "
            "bridge UDP uplink: udpout:127.0.0.1:14555"
        ),
    )
    parser.add_argument("--serial", help="Shortcut for serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--gps-id",
        type=parse_gps_ids,
        default=[1],
        help="GPS_INPUT id to feed: 0/gps1, 1/gps2, or both. Default: gps2.",
    )
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--target-component", type=int, default=1)
    parser.add_argument("--lat", type=float, default=12.9715987)
    parser.add_argument("--lon", type=float, default=77.5945627)
    parser.add_argument("--alt", type=float, default=900.0, help="MSL altitude in meters")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--sats", type=int, default=15)
    parser.add_argument("--hacc", type=float, default=0.6, help="Horizontal accuracy in meters")
    parser.add_argument("--vacc", type=float, default=1.0, help="Vertical accuracy in meters")
    parser.add_argument("--speed-acc", type=float, default=0.2)
    parser.add_argument("--home-interval", type=float, default=5.0)
    parser.add_argument(
        "--configure-fc",
        action="store_true",
        help="Set selected GPS_TYPE/GPS2_TYPE to MAVLink GPS (14). May need Cube reboot.",
    )
    parser.add_argument(
        "--disable-visodom",
        action="store_true",
        help="Also set VISO_TYPE=0 while configuring. Use only for diagnostics.",
    )
    parser.add_argument("--listen-udp", type=int, default=14550)
    parser.add_argument("--no-listen", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    gps_ids = list(args.gps_id)
    duration_s = max(1.0, float(args.duration))
    period_s = 1.0 / max(1.0, float(args.rate_hz))

    if args.serial:
        connection = args.serial
    else:
        connection = args.connection

    if args.dry_run:
        print("Dry run only; no MAVLink packets will be sent.")
        print(f"connection={connection} gps_ids={gps_ids} duration={duration_s:.1f}s")
        return

    if not args.i_understand_this_spoofs_gps:
        raise SystemExit(
            "Refusing to run. Re-run with --i-understand-this-spoofs-gps on a bench, props removed."
        )

    print("WARNING: feeding fake GPS_INPUT. Props off; do not use this as a flight GPS.")
    print(f"Connecting via {connection}...")
    master = mavutil.mavlink_connection(
        connection,
        baud=args.baud,
        source_system=246,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
    )
    master.target_system = int(args.target_system)
    master.target_component = int(args.target_component)

    if not connection.startswith("udpout:"):
        master.wait_heartbeat(timeout=10)
        print(f"Heartbeat received: sys={master.target_system} comp={master.target_component}")
    else:
        print("UDP-out mode: sending through bridge; heartbeat is not required on this socket.")

    stop_listener = threading.Event()
    listener_thread = None
    if not args.no_listen and args.listen_udp > 0:
        listener_thread = threading.Thread(
            target=listen_statustext,
            args=(stop_listener, args.listen_udp),
            daemon=True,
        )
        listener_thread.start()
        print(f"Listening for GCS STATUSTEXT on UDP {args.listen_udp}...")

    if args.configure_fc:
        if 0 in gps_ids:
            set_param(master, "GPS_TYPE", 14)
        if 1 in gps_ids:
            set_param(master, "GPS2_TYPE", 14)
        set_param(master, "GPS_AUTO_SWITCH", 1)
        if args.disable_visodom:
            set_param(master, "VISO_TYPE", 0)
        print("If GPS_TYPE/GPS2_TYPE changed, reboot Cube once, then run this script again.")

    send_statustext(master, "FAKEGPS diagnostic feed started", mavutil.mavlink.MAV_SEVERITY_WARNING)
    start_s = time.time()
    next_home_s = 0.0
    next_print_s = 0.0
    sent = 0

    try:
        while time.time() - start_s < duration_s:
            now_s = time.time()
            send_fake_gps_input(
                master,
                gps_ids,
                args.lat,
                args.lon,
                args.alt,
                args.sats,
                args.hacc,
                args.vacc,
                args.speed_acc,
            )
            sent += len(gps_ids)

            if now_s >= next_home_s:
                set_origin_and_home(master, args.lat, args.lon, args.alt)
                next_home_s = now_s + max(1.0, args.home_interval)

            if now_s >= next_print_s:
                remaining_s = max(0.0, duration_s - (now_s - start_s))
                print(
                    "FAKEGPS:"
                    f" gps={','.join('GPS' + str(g + 1) for g in gps_ids)}"
                    f" fix=3 sats={args.sats}"
                    f" remaining={remaining_s:.0f}s sent={sent}"
                )
                next_print_s = now_s + 5.0

            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\nInterrupted; stopping fake GPS feed.")
    finally:
        send_statustext(master, "FAKEGPS diagnostic feed stopped", mavutil.mavlink.MAV_SEVERITY_WARNING)
        stop_listener.set()
        if listener_thread is not None:
            listener_thread.join(timeout=1.0)

    print("Done. Fake GPS feed stopped.")


if __name__ == "__main__":
    main()
