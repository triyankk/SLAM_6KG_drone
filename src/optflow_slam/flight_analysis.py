"""Analyze a passive optFlow_slam flight recording."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
from typing import Any, Iterable

import numpy as np

from .flight_logger import SCHEMA_VERSION, _json_safe
from .slam_timing import analyze_slam_timing


SELECTED_DATAFLASH_TYPES = frozenset(
    (
        "ATT",
        "BARO",
        "BAT",
        "BAT2",
        "CTUN",
        "ERR",
        "EV",
        "IMU",
        "IMU2",
        "IMU3",
        "MAG",
        "MAG2",
        "MAG3",
        "MODE",
        "MOTB",
        "OF",
        "PIDP",
        "PIDR",
        "PIDY",
        "POS",
        "POWR",
        "RATE",
        "RFND",
        "VIBE",
        "XKF1",
        "XKF2",
        "XKF3",
        "XKF4",
        "XKF5",
    )
)


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"{path}:{line_number}: invalid JSON: {exc}"
                ) from exc
            if isinstance(value, dict):
                rows.append(value)
    return rows


def _summarize_sensor_events(path: Path) -> dict[str, Any]:
    counts: dict[str, dict[str, int]] = {}
    rows = 0
    dropped = 0
    first_ns: int | None = None
    last_ns: int | None = None
    if not path.exists():
        return {"rows": 0, "sequence_gaps": 0, "counts": counts}
    with path.open("r", encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            event = json.loads(line)
            rows += 1
            timestamp = int(event.get("host_monotonic_ns", 0))
            first_ns = timestamp if first_ns is None else first_ns
            last_ns = timestamp
            dropped += max(
                int(event.get("dropped_before", 0) or 0),
                int(event.get("observed_gap_before", 0) or 0),
            )
            event_source = str(event.get("source", "unknown"))
            event_type = str(event.get("type", "unknown"))
            source_counts = counts.setdefault(event_source, {})
            source_counts[event_type] = source_counts.get(event_type, 0) + 1
    duration_s = (
        (last_ns - first_ns) / 1.0e9
        if first_ns is not None
        and last_ns is not None
        and last_ns > first_ns
        else 0.0
    )
    return {
        "rows": rows,
        "duration_s": duration_s,
        "average_rate_hz": rows / duration_s if duration_s > 0 else 0.0,
        "sequence_gaps": dropped,
        "counts": counts,
    }


def _finite(values: Iterable[Any]) -> np.ndarray:
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
    return np.asarray(converted, dtype=np.float64)


def _stats(values: Iterable[Any]) -> dict[str, Any]:
    array = _finite(values)
    if not len(array):
        return {"count": 0}
    return {
        "count": int(len(array)),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
        "mean": float(np.mean(array)),
    }


def _rmse(values: Iterable[Any]) -> float | None:
    array = _finite(values)
    if not len(array):
        return None
    return float(np.sqrt(np.mean(array * array)))


def _get(mapping: dict[str, Any], path: str, default: Any = None) -> Any:
    value: Any = mapping
    for key in path.split("."):
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _ply_vertex_count(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("rb") as source:
        for _ in range(100):
            line = source.readline()
            if not line:
                break
            if line.startswith(b"element vertex "):
                return int(line.split()[2])
            if line.strip() == b"end_header":
                break
    return 0


def _estimate_imu_alignment(
    telemetry: list[dict[str, Any]],
) -> dict[str, Any]:
    cube_times: list[float] = []
    cube_axes: list[list[float]] = [[], [], []]
    external_by_time: dict[int, tuple[float, tuple[float, float, float]]] = {}

    for row in telemetry:
        host_s = float(row.get("host_monotonic_ns", 0)) / 1.0e9
        snapshot = row.get("snapshot", {})
        cube = snapshot.get("imu", {})
        cube_age = cube.get("age_ms")
        if cube_age is not None:
            cube_time = host_s - float(cube_age) / 1000.0
            cube_values = (
                cube.get("gyro_x_rads"),
                cube.get("gyro_y_rads"),
                cube.get("gyro_z_rads"),
            )
            if all(value is not None for value in cube_values):
                cube_times.append(cube_time)
                for axis, value in enumerate(cube_values):
                    cube_axes[axis].append(float(value))

        external = snapshot.get("ros_imu", {})
        body = external.get("body_preview", {})
        external_age = external.get("age_ms")
        values = (
            body.get("gyro_x_rads"),
            body.get("gyro_y_rads"),
            body.get("gyro_z_rads"),
        )
        if external_age is None or not all(
            value is not None for value in values
        ):
            continue
        external_time = host_s - float(external_age) / 1000.0
        key = round(external_time * 1000.0)
        external_by_time[key] = (
            external_time,
            tuple(float(value) for value in values),
        )

    if len(cube_times) < 20 or len(external_by_time) < 10:
        return {
            "available": False,
            "reason": "not enough synchronized Cube and external IMU samples",
            "cube_samples": len(cube_times),
            "external_samples": len(external_by_time),
        }

    cube_time_array = np.asarray(cube_times)
    order = np.argsort(cube_time_array)
    cube_time_array = cube_time_array[order]
    cube_arrays = [
        np.asarray(values, dtype=np.float64)[order] for values in cube_axes
    ]
    external_rows = sorted(external_by_time.values())
    external_times = np.asarray([row[0] for row in external_rows])
    external_axes = [
        np.asarray([row[1][axis] for row in external_rows])
        for axis in range(3)
    ]
    axis_names = ("x", "y", "z")
    results: dict[str, Any] = {}
    lags = np.arange(-0.40, 0.401, 0.01)

    for axis_name, cube_values, external_values in zip(
        axis_names, cube_arrays, external_axes
    ):
        best: tuple[float, float, float, int] | None = None
        for lag_s in lags:
            sample_times = external_times + lag_s
            valid = (
                (sample_times >= cube_time_array[0])
                & (sample_times <= cube_time_array[-1])
            )
            if np.count_nonzero(valid) < 10:
                continue
            cube_interpolated = np.interp(
                sample_times[valid], cube_time_array, cube_values
            )
            external_valid = external_values[valid]
            if (
                np.std(cube_interpolated) < 1.0e-6
                or np.std(external_valid) < 1.0e-6
            ):
                continue
            correlation = float(
                np.corrcoef(cube_interpolated, external_valid)[0, 1]
            )
            residual = cube_interpolated - external_valid
            rmse = float(np.sqrt(np.mean(residual * residual)))
            candidate = (
                abs(correlation),
                correlation,
                rmse,
                int(np.count_nonzero(valid)),
            )
            if best is None or candidate[0] > best[0]:
                best = candidate
                best_lag = float(lag_s)
        if best is None:
            results[axis_name] = {"available": False}
            continue
        results[axis_name] = {
            "available": True,
            "lag_s": best_lag,
            "correlation": best[1],
            "rmse_rads": best[2],
            "samples": best[3],
            "lag_convention": (
                "Cube sampled at external sample time plus lag"
            ),
        }

    return {
        "available": any(
            result.get("available") for result in results.values()
        ),
        "cube_samples": len(cube_times),
        "external_samples": len(external_by_time),
        "axes": results,
    }


def extract_dataflash(
    cube_log: Path, session_path: Path
) -> tuple[Path, dict[str, Any]]:
    """Copy a Cube BIN log into the session and extract analysis messages."""

    if not cube_log.is_file():
        raise FileNotFoundError(cube_log)
    cube_dir = session_path / "cube"
    cube_dir.mkdir(parents=True, exist_ok=True)
    destination = cube_dir / cube_log.name
    if cube_log.resolve() != destination.resolve():
        shutil.copy2(cube_log, destination)

    from pymavlink import DFReader

    extracted_path = cube_dir / "dataflash.ndjson"
    counts: dict[str, int] = {}
    first_time_us: int | None = None
    last_time_us: int | None = None
    reader = DFReader.DFReader_binary(str(destination))
    with extracted_path.open("w", encoding="utf-8") as output:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            message_type = message.get_type()
            if message_type not in SELECTED_DATAFLASH_TYPES:
                continue
            payload = message.to_dict()
            time_us_value = payload.get("TimeUS")
            if time_us_value is not None:
                time_us = int(time_us_value)
                first_time_us = (
                    time_us if first_time_us is None else first_time_us
                )
                last_time_us = time_us
            counts[message_type] = counts.get(message_type, 0) + 1
            output.write(
                json.dumps(
                    _json_safe(
                        {
                            "schema_version": SCHEMA_VERSION,
                            "type": message_type,
                            "data": payload,
                        }
                    ),
                    separators=(",", ":"),
                )
            )
            output.write("\n")

    summary = {
        "source": str(destination.relative_to(session_path)),
        "extracted": str(extracted_path.relative_to(session_path)),
        "message_counts": counts,
        "first_time_us": first_time_us,
        "last_time_us": last_time_us,
    }
    (cube_dir / "dataflash_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return extracted_path, summary


def _compare_dataflash_attitude(
    telemetry: list[dict[str, Any]], dataflash: list[dict[str, Any]]
) -> dict[str, Any]:
    logger_rows: list[tuple[float, float, float]] = []
    for row in telemetry:
        attitude = _get(row, "snapshot.attitude", {})
        boot_ms = attitude.get("time_boot_ms")
        if boot_ms is None:
            continue
        logger_rows.append(
            (
                float(boot_ms) / 1000.0,
                math.degrees(float(attitude.get("roll_rad") or 0.0)),
                math.degrees(float(attitude.get("pitch_rad") or 0.0)),
            )
        )

    dataflash_rows: list[tuple[float, float, float]] = []
    for row in dataflash:
        if row.get("type") != "ATT":
            continue
        data = row.get("data", {})
        if not all(key in data for key in ("TimeUS", "Roll", "Pitch")):
            continue
        dataflash_rows.append(
            (
                float(data["TimeUS"]) / 1.0e6,
                float(data["Roll"]),
                float(data["Pitch"]),
            )
        )
    if len(logger_rows) < 10 or len(dataflash_rows) < 10:
        return {
            "available": False,
            "reason": "not enough overlapping ATTITUDE and ATT samples",
        }

    logger = np.asarray(logger_rows)
    dataflash_array = np.asarray(dataflash_rows)
    valid = (
        (logger[:, 0] >= dataflash_array[0, 0])
        & (logger[:, 0] <= dataflash_array[-1, 0])
    )
    if np.count_nonzero(valid) < 10:
        return {
            "available": False,
            "reason": "Cube boot-time ranges do not overlap",
        }
    logger_valid = logger[valid]
    dataflash_roll = np.interp(
        logger_valid[:, 0], dataflash_array[:, 0], dataflash_array[:, 1]
    )
    dataflash_pitch = np.interp(
        logger_valid[:, 0], dataflash_array[:, 0], dataflash_array[:, 2]
    )
    return {
        "available": True,
        "samples": int(len(logger_valid)),
        "roll_rmse_deg": _rmse(logger_valid[:, 1] - dataflash_roll),
        "pitch_rmse_deg": _rmse(logger_valid[:, 2] - dataflash_pitch),
    }


def _write_timeline(
    path: Path,
    telemetry: list[dict[str, Any]],
    shadows: list[dict[str, Any]],
) -> None:
    shadow_by_time = {
        int(row.get("host_monotonic_ns", 0)): row for row in shadows
    }
    fieldnames = (
        "time_s",
        "cube_time_boot_ms",
        "armed",
        "mode",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "flow_x_mps",
        "flow_y_mps",
        "flow_quality",
        "range_m",
        "local_x_m",
        "local_y_m",
        "local_z_down_m",
        "voltage_v",
        "current_a",
        "battery_remaining_pct",
        "vibration_x",
        "vibration_y",
        "vibration_z",
        "shadow_x_m",
        "shadow_y_m",
        "predicted_roll_deg",
        "predicted_pitch_deg",
        "prediction_applicable",
    )
    start_ns = (
        int(telemetry[0].get("host_monotonic_ns", 0)) if telemetry else 0
    )
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row in telemetry:
            timestamp = int(row.get("host_monotonic_ns", 0))
            snapshot = row.get("snapshot", {})
            shadow = shadow_by_time.get(timestamp, {})
            prediction = shadow.get("perfect_slam_stationary_hold", {})
            pose = shadow.get("pose_observation", {})
            attitude = snapshot.get("attitude", {})
            writer.writerow(
                {
                    "time_s": (timestamp - start_ns) / 1.0e9,
                    "cube_time_boot_ms": attitude.get("time_boot_ms"),
                    "armed": _get(snapshot, "vehicle.armed"),
                    "mode": _get(snapshot, "vehicle.mode"),
                    "roll_deg": math.degrees(
                        float(attitude.get("roll_rad") or 0.0)
                    ),
                    "pitch_deg": math.degrees(
                        float(attitude.get("pitch_rad") or 0.0)
                    ),
                    "yaw_deg": math.degrees(
                        float(attitude.get("yaw_rad") or 0.0)
                    ),
                    "flow_x_mps": _get(snapshot, "flow.comp_x_mps"),
                    "flow_y_mps": _get(snapshot, "flow.comp_y_mps"),
                    "flow_quality": _get(snapshot, "flow.quality"),
                    "range_m": _get(snapshot, "range.distance_m"),
                    "local_x_m": _get(snapshot, "local_position.x_m"),
                    "local_y_m": _get(snapshot, "local_position.y_m"),
                    "local_z_down_m": _get(
                        snapshot, "local_position.z_down_m"
                    ),
                    "voltage_v": _get(snapshot, "power.voltage_v"),
                    "current_a": _get(snapshot, "power.current_a"),
                    "battery_remaining_pct": _get(
                        snapshot, "power.remaining_pct"
                    ),
                    "vibration_x": _get(snapshot, "vibration.x_mss"),
                    "vibration_y": _get(snapshot, "vibration.y_mss"),
                    "vibration_z": _get(snapshot, "vibration.z_mss"),
                    "shadow_x_m": pose.get("x_m"),
                    "shadow_y_m": pose.get("y_m"),
                    "predicted_roll_deg": math.degrees(
                        float(prediction.get("predicted_roll_rad") or 0.0)
                    ),
                    "predicted_pitch_deg": math.degrees(
                        float(prediction.get("predicted_pitch_rad") or 0.0)
                    ),
                    "prediction_applicable": prediction.get(
                        "prediction_applicable"
                    ),
                }
            )


def _format_value(
    value: Any, suffix: str = "", digits: int = 3
) -> str:
    if value is None:
        return "not available"
    if isinstance(value, float):
        return f"{value:.{digits}f}{suffix}"
    return f"{value}{suffix}"


def _write_markdown(path: Path, report: dict[str, Any]) -> None:
    coverage = report["coverage"]
    shadow = report["stationary_hold_shadow"]
    power = report["power"]
    vibration = report["vibration"]
    capture = report["capture_3d"]
    imu = report["imu_crosscheck"]
    raw_events = report["raw_sensor_events"]
    slam_timing = report["slam_timing"]
    slam_gates = slam_timing["gates"]
    slam_imu = slam_timing["external_imu"]
    slam_lidar = slam_timing["jt16"]
    slam_camera = slam_timing["d415"]
    lines = [
        "# Flight Analysis",
        "",
        "This is passive shadow analysis. It is not a flight-safety verdict and "
        "none of the predicted values were sent to the Cube.",
        "",
        "## Coverage",
        "",
        f"- Duration: {_format_value(coverage.get('duration_s'), ' s')}",
        f"- Telemetry rows: {coverage.get('telemetry_rows', 0)}",
        f"- Raw sensor events: {coverage.get('sensor_event_rows', 0)}",
        (
            "- Raw event average rate: "
            f"{_format_value(raw_events.get('average_rate_hz'), ' Hz')}"
        ),
        f"- Raw event sequence gaps: {raw_events.get('sequence_gaps', 0)}",
        f"- Shadow rows: {coverage.get('shadow_rows', 0)}",
        f"- Armed samples: {coverage.get('armed_samples', 0)}",
        f"- Modes observed: {', '.join(coverage.get('modes', [])) or 'none'}",
        "",
        "## Hold Shadow",
        "",
        (
            "- Model: exact local pose with a fresh Cube target when available, "
            "otherwise a stationary session-origin target."
        ),
        f"- Applicable samples: {shadow.get('applicable_samples', 0)}",
        (
            "- Roll residual RMSE: "
            f"{_format_value(shadow.get('roll_residual_rmse_deg'), ' deg')}"
        ),
        (
            "- Pitch residual RMSE: "
            f"{_format_value(shadow.get('pitch_residual_rmse_deg'), ' deg')}"
        ),
        (
            "- Peak observed displacement: "
            f"{_format_value(shadow.get('peak_xy_displacement_m'), ' m')}"
        ),
        "",
        "A residual is not automatically an error: pilot input, a moving target, "
        "wind, estimator delay and the Cube controller all change real attitude.",
        "",
        "## Sensor Crosscheck",
        "",
        (
            f"- IMU alignment available: "
            f"{str(bool(imu.get('available'))).lower()}"
        ),
    ]
    for axis, values in imu.get("axes", {}).items():
        if not values.get("available"):
            continue
        lines.append(
            f"- {axis.upper()} gyro: lag "
            f"{_format_value(values.get('lag_s'), ' s')}, correlation "
            f"{_format_value(values.get('correlation'))}, aligned RMSE "
            f"{_format_value(values.get('rmse_rads'), ' rad/s')}"
        )
    lines.extend(
        (
            "",
            "## SLAM Timing Gate",
            "",
            (
                "- Lidar-inertial replay ready: "
                f"{str(bool(slam_gates.get('ready_for_lidar_inertial_replay'))).lower()}"
            ),
            (
                "- IM10A measured rate: "
                f"{_format_value(_get(slam_imu, 'host_arrival.observed_rate_hz'), ' Hz')}"
            ),
            (
                "- IM10A hardware time present: "
                f"{str(bool(slam_imu.get('hardware_timestamp_present'))).lower()}"
            ),
            f"- D415 timed framesets: {slam_camera.get('framesets', 0)}",
            f"- JT16 timed frames: {slam_lidar.get('frames', 0)}",
            (
                "- Remaining blockers: "
                f"{'; '.join(slam_timing.get('blockers', [])) or 'none'}"
            ),
            "",
            "The timing gate is evidence for recorded estimator work, not a "
            "flight-control authorization.",
            "",
            "## Power And Vibration",
            "",
            (
                "- Minimum voltage: "
                f"{_format_value(_get(power, 'voltage_v.min'), ' V')}"
            ),
            (
                "- Maximum current: "
                f"{_format_value(_get(power, 'current_a.max'), ' A')}"
            ),
            (
                "- Vibration maxima X/Y/Z: "
                f"{_format_value(_get(vibration, 'x.max'))}, "
                f"{_format_value(_get(vibration, 'y.max'))}, "
                f"{_format_value(_get(vibration, 'z.max'))}"
            ),
            (
                "- Clipping maxima 0/1/2: "
                f"{vibration.get('clipping_max', [0, 0, 0])}"
            ),
            "",
            "## 3D Capture",
            "",
            (
                "- Merged environment points: "
                f"{capture.get('environment_points', 0)}"
            ),
            (
                "- PLY keyframes: "
                f"{capture.get('pointcloud_frames', 0)}"
            ),
            (
                "- JT16 packet headers: "
                f"{capture.get('lidar_packet_headers', 0)}"
            ),
            (
                "- JT16 raw capture size: "
                f"{_format_value(capture.get('lidar_capture_bytes'), ' bytes', 0)}"
            ),
            (
                "- RealSense bag size: "
                f"{_format_value(capture.get('realsense_bag_bytes'), ' bytes', 0)}"
            ),
            (
                "- RealSense recording rate: "
                f"{_format_value(capture.get('realsense_mb_per_min'), ' MB/min')}"
            ),
            "",
            "The current PLY is a provisional telemetry-registered cloud, not a "
            "loop-closed SLAM map. Rebuild it after camera extrinsics and timing "
            "are calibrated.",
        )
    )
    dataflash = report.get("dataflash")
    if dataflash:
        comparison = report.get("dataflash_attitude_comparison", {})
        lines.extend(
            (
                "",
                "## Cube DataFlash",
                "",
                f"- Source: {dataflash.get('source')}",
                (
                    "- Logger/DataFlash attitude overlap: "
                    f"{str(bool(comparison.get('available'))).lower()}"
                ),
                (
                    "- Roll RMSE: "
                    f"{_format_value(comparison.get('roll_rmse_deg'), ' deg')}"
                ),
                (
                    "- Pitch RMSE: "
                    f"{_format_value(comparison.get('pitch_rmse_deg'), ' deg')}"
                ),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_session(
    session_path: Path | str, cube_log: Path | str | None = None
) -> Path:
    session = Path(session_path).resolve()
    manifest_path = session / "manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing session manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    telemetry = _read_ndjson(session / "telemetry.ndjson")
    shadows = _read_ndjson(session / "shadow_predictions.ndjson")
    raw_sensor_events = _summarize_sensor_events(
        session / "sensor_events.ndjson"
    )
    slam_timing = analyze_slam_timing(session, manifest)

    dataflash_summary: dict[str, Any] | None = None
    if cube_log is not None:
        _, dataflash_summary = extract_dataflash(Path(cube_log), session)
    else:
        summary_path = session / "cube" / "dataflash_summary.json"
        if summary_path.exists():
            dataflash_summary = json.loads(
                summary_path.read_text(encoding="utf-8")
            )
    dataflash_rows = _read_ndjson(session / "cube" / "dataflash.ndjson")

    host_times = _finite(
        row.get("host_monotonic_ns") for row in telemetry
    )
    duration_s = (
        float((host_times[-1] - host_times[0]) / 1.0e9)
        if len(host_times) >= 2
        else 0.0
    )
    armed = [
        bool(_get(row, "snapshot.vehicle.armed", False))
        for row in telemetry
    ]
    modes = sorted(
        {
            str(_get(row, "snapshot.vehicle.mode"))
            for row in telemetry
            if _get(row, "snapshot.vehicle.mode") is not None
        }
    )

    applicable = [
        row
        for row in shadows
        if bool(
            _get(
                row,
                "perfect_slam_stationary_hold.prediction_applicable",
                False,
            )
        )
    ]
    roll_residual_deg = [
        math.degrees(
            float(
                _get(
                    row,
                    "perfect_slam_stationary_hold.roll_residual_rad",
                    0.0,
                )
            )
        )
        for row in applicable
    ]
    pitch_residual_deg = [
        math.degrees(
            float(
                _get(
                    row,
                    "perfect_slam_stationary_hold.pitch_residual_rad",
                    0.0,
                )
            )
        )
        for row in applicable
    ]
    displacements = [
        math.hypot(
            float(_get(row, "pose_observation.x_m", 0.0)),
            float(_get(row, "pose_observation.y_m", 0.0)),
        )
        for row in shadows
    ]

    clipping = [
        [
            int(_get(row, f"snapshot.vibration.clipping_{axis}", 0) or 0)
            for row in telemetry
        ]
        for axis in range(3)
    ]
    environment_path = session / "pointcloud" / "flight_environment.ply"
    lidar_serial_path = session / "lidar" / "jt16_serial.bin"
    lidar_pcap_path = session / "lidar" / "jt16_packets.pcap"
    lidar_path = (
        lidar_serial_path
        if lidar_serial_path.exists()
        else lidar_pcap_path
    )
    bag_path = session / "realsense" / "flight.bag"
    bag_bytes = bag_path.stat().st_size if bag_path.exists() else 0

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "generated_utc": datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        ),
        "session_id": manifest.get("session_id"),
        "passive_only": True,
        "coverage": {
            "duration_s": duration_s,
            "telemetry_rows": len(telemetry),
            "sensor_event_rows": int(
                _get(manifest, "rows.sensor_events", 0) or 0
            ),
            "sensor_timing_rows": int(
                _get(manifest, "rows.sensor_timing", 0) or 0
            ),
            "shadow_rows": len(shadows),
            "armed_samples": sum(armed),
            "modes": modes,
        },
        "flow": {
            "quality": _stats(
                _get(row, "snapshot.flow.quality") for row in telemetry
            ),
            "x_velocity_mps": _stats(
                _get(row, "snapshot.flow.comp_x_mps")
                for row in telemetry
            ),
            "y_velocity_mps": _stats(
                _get(row, "snapshot.flow.comp_y_mps")
                for row in telemetry
            ),
            "fresh_samples": sum(
                (
                    _get(row, "snapshot.flow.age_ms") is not None
                    and float(_get(row, "snapshot.flow.age_ms")) <= 250.0
                )
                for row in telemetry
            ),
        },
        "range_m": _stats(
            _get(row, "snapshot.range.distance_m") for row in telemetry
        ),
        "stationary_hold_shadow": {
            "applicable_samples": len(applicable),
            "roll_residual_rmse_deg": _rmse(roll_residual_deg),
            "pitch_residual_rmse_deg": _rmse(pitch_residual_deg),
            "peak_xy_displacement_m": (
                max(displacements) if displacements else None
            ),
            "final_xy_displacement_m": (
                displacements[-1] if displacements else None
            ),
        },
        "raw_sensor_events": raw_sensor_events,
        "slam_timing": slam_timing,
        "imu_crosscheck": _estimate_imu_alignment(telemetry),
        "power": {
            "voltage_v": _stats(
                _get(row, "snapshot.power.voltage_v")
                for row in telemetry
            ),
            "current_a": _stats(
                _get(row, "snapshot.power.current_a")
                for row in telemetry
            ),
            "remaining_pct": _stats(
                _get(row, "snapshot.power.remaining_pct")
                for row in telemetry
            ),
        },
        "vibration": {
            "x": _stats(
                _get(row, "snapshot.vibration.x_mss")
                for row in telemetry
            ),
            "y": _stats(
                _get(row, "snapshot.vibration.y_mss")
                for row in telemetry
            ),
            "z": _stats(
                _get(row, "snapshot.vibration.z_mss")
                for row in telemetry
            ),
            "clipping_max": [
                max(values) if values else 0 for values in clipping
            ],
        },
        "capture_3d": {
            "environment_cloud": str(
                environment_path.relative_to(session)
            ),
            "environment_points": _ply_vertex_count(environment_path),
            "pointcloud_frames": len(
                tuple((session / "pointcloud" / "frames").glob("*.ply"))
            ),
            "lidar_transport": (
                "serial_rs485"
                if lidar_serial_path.exists()
                else "legacy_udp"
            ),
            "lidar_packet_headers": int(
                _get(
                    manifest,
                    "source_stats.lidar.header_candidates",
                    _get(manifest, "source_stats.lidar.packets", 0),
                )
                or 0
            ),
            "lidar_capture_bytes": (
                lidar_path.stat().st_size if lidar_path.exists() else 0
            ),
            "realsense_bag_bytes": bag_bytes,
            "realsense_mb_per_min": (
                bag_bytes / 1.0e6 * 60.0 / duration_s
                if duration_s > 0 and bag_bytes > 0
                else 0.0
            ),
            "slam_optimized": False,
        },
        "source_stats": manifest.get("source_stats", {}),
    }
    if dataflash_summary is not None:
        report["dataflash"] = dataflash_summary
        report["dataflash_attitude_comparison"] = (
            _compare_dataflash_attitude(telemetry, dataflash_rows)
        )

    analysis_dir = session / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    report_path = analysis_dir / "report.json"
    report_path.write_text(
        json.dumps(_json_safe(report), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_markdown(analysis_dir / "report.md", report)
    _write_timeline(analysis_dir / "timeline.csv", telemetry, shadows)
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--cube-log", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        report = analyze_session(args.session, args.cube_log)
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f"Flight analysis failed: {exc}")
        return 2
    print(f"Flight report: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
