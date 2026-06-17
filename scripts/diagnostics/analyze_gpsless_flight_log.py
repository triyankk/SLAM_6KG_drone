#!/usr/bin/env python3
"""Check whether an ArduPilot DataFlash log actually flew GPS-less PosHold.

The field question this answers is deliberately narrow:
did armed POSHOLD happen on the companion GPS2/SLAM lane, or was the aircraft
still flying with the real GPS receiver?
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from pymavlink import mavutil


MODE_NAMES = {
    0: "STABILIZE",
    2: "ALTHOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    9: "LAND",
    16: "POSHOLD",
    17: "BRAKE",
    20: "GUIDED_NOGPS",
    22: "FLOWHOLD",
}

IMPORTANT_PARAMS = {
    "GPS_AUTO_SWITCH",
    "GPS_PRIMARY",
    "GPS1_TYPE",
    "GPS2_TYPE",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC2_POSXY",
    "EK3_SRC2_VELXY",
    "EK3_SRC2_POSZ",
    "EK3_SRC2_VELZ",
    "VISO_TYPE",
    "FLOW_TYPE",
    "RNGFND1_TYPE",
    "SCR_USER1",
    "SCR_USER2",
    "SCR_USER3",
}

COMPANION_KEYWORDS = (
    "SLAM",
    "VIO",
    "OBS",
    "CAL",
    "BRIDGE",
    "NO-GPS",
    "GPS-LESS",
    "GPS LESS",
    "LGC",
)


@dataclass
class GpsStats:
    total: int = 0
    healthy: int = 0
    bad: int = 0
    status_counts: Counter = field(default_factory=Counter)
    sats_min: Optional[int] = None
    sats_max: Optional[int] = None
    valid_latlon: int = 0


@dataclass
class Period:
    start_s: float
    end_s: float
    mode: str
    armed: bool


def msg_time_s(msg) -> Optional[float]:
    if hasattr(msg, "TimeUS"):
        return float(getattr(msg, "TimeUS")) / 1_000_000.0
    if hasattr(msg, "TimeMS"):
        return float(getattr(msg, "TimeMS")) / 1000.0
    return None


def fmt_time(seconds: Optional[float], start_s: Optional[float]) -> str:
    if seconds is None or start_s is None:
        return "??"
    return f"+{seconds - start_s:7.1f}s"


def latlon_valid(lat_raw, lon_raw) -> bool:
    try:
        lat = float(lat_raw)
        lon = float(lon_raw)
    except (TypeError, ValueError):
        return False
    if abs(lat) > 1000.0 or abs(lon) > 1000.0:
        lat /= 1e7
        lon /= 1e7
    return abs(lat) > 1e-7 and abs(lon) > 1e-7 and abs(lat) <= 90.0 and abs(lon) <= 180.0


def gps_healthy(msg) -> bool:
    status = int(getattr(msg, "Status", 0) or 0)
    sats = int(getattr(msg, "NSats", 0) or 0)
    return status >= 3 and sats >= 6 and latlon_valid(getattr(msg, "Lat", 0), getattr(msg, "Lng", 0))


def period_overlaps(period: Period, sample_time_s: float) -> bool:
    return period.start_s <= sample_time_s <= period.end_s


def parse_log(path: Path) -> dict:
    log = mavutil.mavlink_connection(str(path), dialect="ardupilotmega", robust_parsing=True)
    start_s = None
    last_s = None
    armed = False
    mode = "UNKNOWN"
    period_start_s = None
    periods: list[Period] = []
    gps_stats: dict[int, GpsStats] = defaultdict(GpsStats)
    gps_samples: list[tuple[float, int, bool]] = []
    params = {}
    companion_messages = []
    all_relevant_messages = []
    scr_user1_values = Counter()

    while True:
        msg = log.recv_match(blocking=False)
        if msg is None:
            break
        typ = msg.get_type()
        t_s = msg_time_s(msg)
        if t_s is not None:
            if start_s is None:
                start_s = t_s
                period_start_s = t_s
            last_s = t_s

        if typ == "MODE":
            new_mode = MODE_NAMES.get(int(getattr(msg, "Mode", -1)), str(getattr(msg, "Mode", "UNKNOWN")))
            if period_start_s is not None and t_s is not None:
                periods.append(Period(period_start_s, t_s, mode, armed))
                period_start_s = t_s
            mode = new_mode
        elif typ == "ARM":
            new_armed = bool(int(getattr(msg, "ArmState", 0) or 0))
            if period_start_s is not None and t_s is not None:
                periods.append(Period(period_start_s, t_s, mode, armed))
                period_start_s = t_s
            armed = new_armed
        elif typ == "GPS":
            instance = int(getattr(msg, "I", 0) or 0)
            stats = gps_stats[instance]
            stats.total += 1
            status = int(getattr(msg, "Status", 0) or 0)
            sats = int(getattr(msg, "NSats", 0) or 0)
            stats.status_counts[status] += 1
            stats.sats_min = sats if stats.sats_min is None else min(stats.sats_min, sats)
            stats.sats_max = sats if stats.sats_max is None else max(stats.sats_max, sats)
            valid = latlon_valid(getattr(msg, "Lat", 0), getattr(msg, "Lng", 0))
            if valid:
                stats.valid_latlon += 1
            healthy = gps_healthy(msg)
            if healthy:
                stats.healthy += 1
            else:
                stats.bad += 1
            if t_s is not None:
                gps_samples.append((t_s, instance, healthy))
        elif typ == "PARM":
            name = getattr(msg, "Name", "")
            if isinstance(name, bytes):
                name = name.decode("ascii", errors="ignore").rstrip("\x00")
            name = str(name).rstrip("\x00")
            value = float(getattr(msg, "Value", 0.0) or 0.0)
            if name in IMPORTANT_PARAMS:
                params[name] = value
            if name == "SCR_USER1":
                scr_user1_values[int(round(value))] += 1
        elif typ in ("MSG", "STATUSTEXT"):
            text = getattr(msg, "Message", getattr(msg, "text", ""))
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            text = str(text).strip()
            if not text:
                continue
            upper = text.upper()
            if any(keyword in upper for keyword in COMPANION_KEYWORDS):
                companion_messages.append((t_s, text))
            if any(token in upper for token in ("BEEP", "PREARM", "GPS 2", "RANGEFINDER", *COMPANION_KEYWORDS)):
                all_relevant_messages.append((t_s, text))

    if period_start_s is not None and last_s is not None:
        periods.append(Period(period_start_s, last_s, mode, armed))

    armed_poshold = [p for p in periods if p.armed and p.mode == "POSHOLD" and p.end_s - p.start_s >= 1.0]
    armed_poshold_gps = []
    for period in armed_poshold:
        counts = Counter()
        for sample_time_s, instance, healthy in gps_samples:
            if period_overlaps(period, sample_time_s):
                counts[(instance, healthy)] += 1
        armed_poshold_gps.append((period, counts))

    gps_primary = int(round(params.get("GPS_PRIMARY", -1)))
    gps2_healthy = gps_stats.get(1, GpsStats()).healthy > 0
    no_gps_state_seen = any(code in scr_user1_values for code in (53, 72))
    no_gps_message_seen = any("GPS-LESS" in text.upper() or "NO-GPS" in text.upper() for _, text in companion_messages)
    gpsless_evidence = bool(armed_poshold and gps_primary == 1 and gps2_healthy and (no_gps_state_seen or no_gps_message_seen))

    return {
        "path": path,
        "start_s": start_s,
        "last_s": last_s,
        "periods": periods,
        "gps_stats": gps_stats,
        "params": params,
        "companion_messages": companion_messages,
        "relevant_messages": all_relevant_messages,
        "scr_user1_values": scr_user1_values,
        "armed_poshold_gps": armed_poshold_gps,
        "gpsless_evidence": gpsless_evidence,
    }


def print_report(result: dict) -> None:
    start_s = result["start_s"]
    last_s = result["last_s"]
    print(f"log: {result['path']}")
    if start_s is not None and last_s is not None:
        print(f"duration: {last_s - start_s:.1f}s")

    print("\nverdict:")
    if result["gpsless_evidence"]:
        print("  GPS-less PosHold evidence found.")
    else:
        print("  NOT GPS-less in this log.")
        print("  Armed PosHold either used GPS1, GPS2 was unhealthy, or no SLAM no-GPS state was logged.")

    print("\narmed POSHOLD periods:")
    if not result["armed_poshold_gps"]:
        print("  none")
    for period, counts in result["armed_poshold_gps"]:
        gps1_good = counts[(0, True)]
        gps1_bad = counts[(0, False)]
        gps2_good = counts[(1, True)]
        gps2_bad = counts[(1, False)]
        print(
            "  "
            f"{fmt_time(period.start_s, start_s)} to {fmt_time(period.end_s, start_s)} "
            f"dur={period.end_s - period.start_s:.1f}s "
            f"GPS1 good/bad={gps1_good}/{gps1_bad} "
            f"GPS2 good/bad={gps2_good}/{gps2_bad}"
        )

    print("\nGPS health:")
    for instance in sorted(result["gps_stats"]):
        stats = result["gps_stats"][instance]
        print(
            f"  GPS{instance + 1}: samples={stats.total} healthy={stats.healthy} bad={stats.bad} "
            f"status={dict(stats.status_counts)} sats={stats.sats_min}-{stats.sats_max} "
            f"valid_latlon={stats.valid_latlon}"
        )

    print("\nimportant params:")
    for name in sorted(result["params"]):
        print(f"  {name}={result['params'][name]}")

    print("\nSCR_USER1 states:")
    if result["scr_user1_values"]:
        for code, count in sorted(result["scr_user1_values"].items()):
            print(f"  {code}: {count}")
    else:
        print("  none logged")

    print("\ncompanion messages:")
    if not result["companion_messages"]:
        print("  none")
    for t_s, text in result["companion_messages"][:80]:
        print(f"  {fmt_time(t_s, start_s)} {text}")

    print("\nrelevant FC/GCS messages:")
    for t_s, text in result["relevant_messages"][:120]:
        print(f"  {fmt_time(t_s, start_s)} {text}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="ArduPilot DataFlash .bin log")
    args = parser.parse_args()
    print_report(parse_log(args.log))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
