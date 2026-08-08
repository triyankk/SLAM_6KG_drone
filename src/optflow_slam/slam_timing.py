"""Measure the timestamp and delivery contract of recorded SLAM sensors."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np


SCHEMA_VERSION = 1
LIO_MINIMUM_IMU_RATE_HZ = 100.0


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"{path}:{line_number}: invalid JSON: {exc}"
                ) from exc
            if isinstance(payload, dict):
                rows.append(payload)
    return rows


def _get(mapping: dict[str, Any], path: str, default: Any = None) -> Any:
    value: Any = mapping
    for key in path.split("."):
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _finite(values: Iterable[Any]) -> list[float]:
    converted: list[float] = []
    for value in values:
        if value is None:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            converted.append(number)
    return converted


def _distribution(values: Iterable[Any]) -> dict[str, Any]:
    array = np.asarray(_finite(values), dtype=np.float64)
    if not len(array):
        return {"count": 0}
    return {
        "count": int(len(array)),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "mean": float(np.mean(array)),
        "stddev": float(np.std(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def summarize_timestamps(
    timestamps: Iterable[Any],
    *,
    units_per_second: float,
    expected_rate_hz: float | None = None,
) -> dict[str, Any]:
    """Summarize an ordered timestamp sequence without assuming its epoch."""

    values = _finite(timestamps)
    if not values:
        return {
            "samples": 0,
            "intervals": 0,
            "expected_rate_hz": expected_rate_hz,
        }

    raw_intervals_s = [
        (current - previous) / units_per_second
        for previous, current in zip(values, values[1:])
    ]
    intervals_s = [
        interval for interval in raw_intervals_s if interval > 0.0
    ]
    non_monotonic = len(raw_intervals_s) - len(intervals_s)
    duration_s = sum(intervals_s)
    observed_rate_hz = (
        len(intervals_s) / duration_s if duration_s > 0.0 else None
    )
    period_ms = [interval * 1000.0 for interval in intervals_s]
    reference_period_s = None
    if expected_rate_hz is not None and expected_rate_hz > 0.0:
        reference_period_s = 1.0 / expected_rate_hz
    elif intervals_s:
        reference_period_s = float(np.median(intervals_s))

    jitter_rms_ms = None
    estimated_drops = 0
    if reference_period_s is not None and intervals_s:
        residuals_ms = [
            (interval - reference_period_s) * 1000.0
            for interval in intervals_s
        ]
        jitter_rms_ms = float(
            np.sqrt(np.mean(np.square(residuals_ms)))
        )
        if expected_rate_hz is not None:
            estimated_drops = sum(
                max(0, int(round(interval / reference_period_s)) - 1)
                for interval in intervals_s
            )

    rate_error_pct = None
    if observed_rate_hz is not None and expected_rate_hz:
        rate_error_pct = (
            (observed_rate_hz - expected_rate_hz)
            / expected_rate_hz
            * 100.0
        )

    return {
        "samples": len(values),
        "intervals": len(intervals_s),
        "duration_s": duration_s,
        "observed_rate_hz": observed_rate_hz,
        "expected_rate_hz": expected_rate_hz,
        "rate_error_pct": rate_error_pct,
        "period_ms": _distribution(period_ms),
        "jitter_rms_ms": jitter_rms_ms,
        "estimated_drops": estimated_drops,
        "non_monotonic_intervals": non_monotonic,
    }


def _frame_number_gaps(values: Iterable[Any]) -> dict[str, int]:
    numbers = [int(value) for value in _finite(values)]
    gaps = 0
    non_monotonic = 0
    for previous, current in zip(numbers, numbers[1:]):
        difference = current - previous
        if difference <= 0:
            non_monotonic += 1
        elif difference > 1:
            gaps += difference - 1
    return {
        "estimated_missing_frames": gaps,
        "non_monotonic_transitions": non_monotonic,
    }


def _unique_frame_rows(
    rows: list[dict[str, Any]], frame_number_key: str
) -> list[dict[str, Any]]:
    unique: list[dict[str, Any]] = []
    previous: int | None = None
    for row in rows:
        value = row.get(frame_number_key)
        if value is None:
            continue
        frame_number = int(value)
        if previous == frame_number:
            continue
        unique.append(row)
        previous = frame_number
    return unique


def _relative_clock_residual_ms(
    host_values: Iterable[Any],
    sensor_values: Iterable[Any],
    *,
    host_units_per_second: float,
    sensor_units_per_second: float,
) -> dict[str, Any]:
    host = _finite(host_values)
    sensor = _finite(sensor_values)
    count = min(len(host), len(sensor))
    if count < 2:
        return {"count": 0}
    host_zero = host[0] / host_units_per_second
    sensor_zero = sensor[0] / sensor_units_per_second
    residuals_ms = [
        (
            (host[index] / host_units_per_second - host_zero)
            - (sensor[index] / sensor_units_per_second - sensor_zero)
        )
        * 1000.0
        for index in range(count)
    ]
    return _distribution(residuals_ms)


def _timing_rows_from_events(
    session: Path,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in _read_ndjson(session / "sensor_events.ndjson"):
        source = str(event.get("source", ""))
        event_type = str(event.get("type", ""))
        if source == "external_imu":
            rows.append(
                {
                    "source": source,
                    "sample_type": event_type,
                    "host_monotonic_ns": event.get("host_monotonic_ns"),
                    "host_unix_ns": event.get("host_unix_ns"),
                    "source_sequence": event.get("sequence"),
                    "sensor_time_s": event.get("data", {}).get(
                        "sensor_time_s"
                    ),
                }
            )
        elif source == "cube_mavlink" and event_type in {
            "ATTITUDE",
            "HIGHRES_IMU",
            "SCALED_IMU",
            "SYSTEM_TIME",
        }:
            data = event.get("data", {})
            rows.append(
                {
                    "source": source,
                    "sample_type": event_type,
                    "host_monotonic_ns": event.get("host_monotonic_ns"),
                    "host_unix_ns": event.get("host_unix_ns"),
                    "source_sequence": event.get("sequence"),
                    "sensor_time_boot_ms": data.get("time_boot_ms"),
                    "sensor_time_usec": data.get("time_usec"),
                }
            )
    return rows


def _select_imu_stream(
    rows: list[dict[str, Any]],
) -> tuple[str | None, list[dict[str, Any]]]:
    for sample_type in ("gyro_rads", "quaternion_wxyz", "accel_mss"):
        selected = [
            row for row in rows if row.get("sample_type") == sample_type
        ]
        if len(selected) >= 2:
            return sample_type, selected
    return None, []


def _select_cube_stream(
    rows: list[dict[str, Any]],
) -> tuple[str | None, list[dict[str, Any]]]:
    for sample_type in ("HIGHRES_IMU", "SCALED_IMU", "ATTITUDE"):
        selected = [
            row for row in rows if row.get("sample_type") == sample_type
        ]
        if len(selected) >= 2:
            return sample_type, selected
    return None, []


def analyze_slam_timing(
    session_path: Path | str,
    manifest: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Analyze sensor timing and write ``analysis/slam_timing.json``."""

    session = Path(session_path).resolve()
    if manifest is None:
        manifest_path = session / "manifest.json"
        if not manifest_path.exists():
            raise FileNotFoundError(
                f"missing session manifest: {manifest_path}"
            )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    timing_rows = _read_ndjson(session / "sensor_timing.ndjson")
    event_timing_rows = [
        row
        for row in timing_rows
        if row.get("source") in {"external_imu", "cube_mavlink"}
    ]
    if not event_timing_rows:
        event_timing_rows = _timing_rows_from_events(session)

    camera_rows = [
        row
        for row in timing_rows
        if row.get("source") == "realsense_frameset"
    ]
    lidar_rows = [
        row
        for row in timing_rows
        if row.get("source") == "jt16_frame"
    ]
    imu_rows = [
        row
        for row in event_timing_rows
        if row.get("source") == "external_imu"
    ]
    cube_rows = [
        row
        for row in event_timing_rows
        if row.get("source") == "cube_mavlink"
    ]

    camera_rate_hz = _get(manifest, "hardware.depth_camera.fps")
    camera_rate_hz = (
        float(camera_rate_hz) if camera_rate_hz is not None else None
    )
    imu_rate_hz = _get(manifest, "hardware.external_imu.expected_rate_hz")
    imu_rate_hz = float(imu_rate_hz) if imu_rate_hz is not None else None

    depth_rows = _unique_frame_rows(camera_rows, "depth_frame_number")
    color_rows = _unique_frame_rows(camera_rows, "color_frame_number")
    camera_host = [row.get("host_monotonic_ns") for row in camera_rows]
    depth_host = [row.get("host_monotonic_ns") for row in depth_rows]
    depth_sensor = [
        row.get("depth_sensor_timestamp_ms") for row in depth_rows
    ]
    color_sensor = [
        row.get("color_sensor_timestamp_ms") for row in color_rows
    ]
    depth_domains = sorted(
        {
            str(row["depth_timestamp_domain"])
            for row in camera_rows
            if row.get("depth_timestamp_domain") is not None
        }
    )
    color_domains = sorted(
        {
            str(row["color_timestamp_domain"])
            for row in camera_rows
            if row.get("color_timestamp_domain") is not None
        }
    )

    lidar_host = [
        row.get("host_receive_monotonic_ns") for row in lidar_rows
    ]
    lidar_callback = [
        row.get("bridge_callback_monotonic_ns") for row in lidar_rows
    ]
    lidar_point_start = [
        row.get("point_timestamp_min_s") for row in lidar_rows
    ]
    callback_latency_ms = [
        (
            float(row["host_receive_monotonic_ns"])
            - float(row["bridge_callback_monotonic_ns"])
        )
        / 1.0e6
        for row in lidar_rows
        if row.get("host_receive_monotonic_ns") is not None
        and row.get("bridge_callback_monotonic_ns") is not None
    ]

    selected_imu_type, selected_imu_rows = _select_imu_stream(imu_rows)
    imu_host = [
        row.get("host_monotonic_ns") for row in selected_imu_rows
    ]
    imu_summary = summarize_timestamps(
        imu_host,
        units_per_second=1.0e9,
        expected_rate_hz=imu_rate_hz,
    )
    imu_hardware_timestamp_present = any(
        row.get("sensor_time_s") is not None
        or row.get("sensor_time_ms") is not None
        or row.get("sensor_time_usec") is not None
        for row in selected_imu_rows
    )

    selected_cube_type, selected_cube_rows = _select_cube_stream(cube_rows)
    cube_host = [
        row.get("host_monotonic_ns") for row in selected_cube_rows
    ]
    cube_boot = [
        row.get("sensor_time_boot_ms") for row in selected_cube_rows
    ]

    depth_summary = summarize_timestamps(
        depth_sensor,
        units_per_second=1000.0,
        expected_rate_hz=camera_rate_hz,
    )
    lidar_point_summary = summarize_timestamps(
        lidar_point_start,
        units_per_second=1.0,
    )
    timing_verified = bool(
        _get(
            manifest,
            "hardware.calibration.sensor_time_sync_verified",
            _get(
                manifest,
                "calibration_warnings.sensor_time_sync_verified",
                False,
            ),
        )
    )
    imu_to_body_verified = bool(
        _get(
            manifest,
            "hardware.calibration.imu_to_body_extrinsics_verified",
            False,
        )
    )
    lidar_to_body_verified = bool(
        _get(
            manifest,
            "hardware.calibration.lidar_to_body_extrinsics_verified",
            False,
        )
    )

    observed_imu_rate = imu_summary.get("observed_rate_hz")
    gates = {
        "d415_device_timestamps_captured": (
            depth_summary.get("samples", 0) >= 2
        ),
        "jt16_point_timestamps_captured": (
            lidar_point_summary.get("samples", 0) >= 2
        ),
        "im10a_host_timestamps_captured": (
            imu_summary.get("samples", 0) >= 2
        ),
        "im10a_hardware_timestamp_captured": (
            imu_hardware_timestamp_present
        ),
        "im10a_rate_at_least_100_hz": (
            observed_imu_rate is not None
            and observed_imu_rate >= LIO_MINIMUM_IMU_RATE_HZ
        ),
        "imu_to_body_extrinsics_verified": imu_to_body_verified,
        "lidar_to_body_extrinsics_verified": lidar_to_body_verified,
        "sensor_time_sync_verified": timing_verified,
    }
    ready_for_lio_replay = all(
        (
            gates["jt16_point_timestamps_captured"],
            gates["im10a_host_timestamps_captured"],
            gates["im10a_hardware_timestamp_captured"],
            gates["im10a_rate_at_least_100_hz"],
            gates["imu_to_body_extrinsics_verified"],
            gates["lidar_to_body_extrinsics_verified"],
            gates["sensor_time_sync_verified"],
        )
    )
    blocker_messages = {
        "jt16_point_timestamps_captured": (
            "JT16 per-point timing was not captured."
        ),
        "im10a_host_timestamps_captured": (
            "IM10A arrival timing was not captured."
        ),
        "im10a_hardware_timestamp_captured": (
            "IM10A frames do not yet carry sensor time."
        ),
        "im10a_rate_at_least_100_hz": (
            "IM10A is below the 100 Hz minimum candidate gate for LIO."
        ),
        "imu_to_body_extrinsics_verified": (
            "IM10A-to-body extrinsics are not verified."
        ),
        "lidar_to_body_extrinsics_verified": (
            "JT16-to-body extrinsics are not verified."
        ),
        "sensor_time_sync_verified": (
            "Cross-sensor time synchronization is not verified."
        ),
    }
    blockers = [
        message
        for gate, message in blocker_messages.items()
        if not gates[gate]
    ]

    report = {
        "schema_version": SCHEMA_VERSION,
        "generated_utc": datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        ),
        "session_id": manifest.get("session_id"),
        "passive_only": True,
        "clock_model": {
            "host_clock": "Jetson CLOCK_MONOTONIC",
            "host_unix_clock": "Jetson CLOCK_REALTIME",
            "d415_sensor_clock": (
                "device timestamp preserved verbatim; epoch not assumed"
            ),
            "jt16_bridge_clock": "Jetson CLOCK_MONOTONIC at SDK callback",
            "jt16_point_clock": (
                "Hesai SDK point timestamp preserved verbatim; "
                "epoch not assumed"
            ),
            "im10a_clock": (
                "IM10A on-sensor time preserved with Jetson serial arrival"
                if imu_hardware_timestamp_present
                else (
                    "Jetson serial decode arrival only until sensor-time "
                    "output is configured"
                )
            ),
        },
        "d415": {
            "framesets": len(camera_rows),
            "unique_depth_frames": len(depth_rows),
            "unique_color_frames": len(color_rows),
            "repeated_depth_frames": len(camera_rows) - len(depth_rows),
            "repeated_color_frames": len(camera_rows) - len(color_rows),
            "host_arrival": summarize_timestamps(
                camera_host,
                units_per_second=1.0e9,
                expected_rate_hz=camera_rate_hz,
            ),
            "depth_sensor": depth_summary,
            "color_sensor": summarize_timestamps(
                color_sensor,
                units_per_second=1000.0,
                expected_rate_hz=camera_rate_hz,
            ),
            "depth_timestamp_domains": depth_domains,
            "color_timestamp_domains": color_domains,
            "depth_frame_numbers": _frame_number_gaps(
                row.get("depth_frame_number") for row in depth_rows
            ),
            "color_frame_numbers": _frame_number_gaps(
                row.get("color_frame_number") for row in color_rows
            ),
            "host_vs_depth_relative_offset_ms": (
                _relative_clock_residual_ms(
                    depth_host,
                    depth_sensor,
                    host_units_per_second=1.0e9,
                    sensor_units_per_second=1000.0,
                )
            ),
        },
        "jt16": {
            "frames": len(lidar_rows),
            "host_receive": summarize_timestamps(
                lidar_host, units_per_second=1.0e9
            ),
            "bridge_callback": summarize_timestamps(
                lidar_callback, units_per_second=1.0e9
            ),
            "point_start": lidar_point_summary,
            "frame_numbers": _frame_number_gaps(
                row.get("frame_index") for row in lidar_rows
            ),
            "callback_to_receive_ms": _distribution(callback_latency_ms),
            "point_timestamp_span_ms": _distribution(
                (
                    float(row["point_timestamp_span_s"]) * 1000.0
                    for row in lidar_rows
                    if row.get("point_timestamp_span_s") is not None
                )
            ),
        },
        "external_imu": {
            "selected_sample_type": selected_imu_type,
            "configured_rate_hz": imu_rate_hz,
            "event_type_counts": {
                sample_type: sum(
                    row.get("sample_type") == sample_type for row in imu_rows
                )
                for sample_type in sorted(
                    {
                        str(row.get("sample_type"))
                        for row in imu_rows
                        if row.get("sample_type") is not None
                    }
                )
            },
            "host_arrival": imu_summary,
            "hardware_timestamp_present": (
                imu_hardware_timestamp_present
            ),
        },
        "cube_reference": {
            "selected_sample_type": selected_cube_type,
            "host_arrival": summarize_timestamps(
                cube_host, units_per_second=1.0e9
            ),
            "host_vs_boot_relative_offset_ms": (
                _relative_clock_residual_ms(
                    cube_host,
                    cube_boot,
                    host_units_per_second=1.0e9,
                    sensor_units_per_second=1000.0,
                )
            ),
        },
        "gates": {
            **gates,
            "ready_for_lidar_inertial_replay": ready_for_lio_replay,
        },
        "blockers": blockers,
        "interpretation": (
            "A short bench capture validates acquisition plumbing only. "
            "It does not verify dynamic time alignment, noise, extrinsics, "
            "or estimator accuracy."
        ),
    }

    analysis_dir = session / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    output = analysis_dir / "slam_timing.json"
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        report = analyze_slam_timing(args.session)
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(f"SLAM timing analysis failed: {exc}")
        return 2
    output = args.session.resolve() / "analysis" / "slam_timing.json"
    print(f"SLAM timing report: {output}")
    print(
        "Lidar-inertial replay ready: "
        f"{report['gates']['ready_for_lidar_inertial_replay']}"
    )
    for blocker in report["blockers"]:
        print(f"- {blocker}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
