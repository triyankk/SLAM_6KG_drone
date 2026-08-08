"""Score a shadow-mode LIO trajectory without enabling flight-control output."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from .config import ConfigError, ProjectConfig, load_config
from .paths import PROJECT_ROOT


GUIDED_TRANSLATION_TARGETS = {
    "settle": (0.0, 0.0, 0.0),
    "forward_1": (0.5, 0.0, 0.0),
    "center_1": (0.0, 0.0, 0.0),
    "right_1": (0.0, 0.5, 0.0),
    "center_2": (0.0, 0.0, 0.0),
    "final_still": (0.0, 0.0, 0.0),
}
GUIDED_TRANSLATION_MINIMUM_CAPTURE_SAMPLES = 5
GUIDED_TRANSLATION_MAXIMUM_CROSS_AXIS_ERROR_M = 0.15
GUIDED_TRANSLATION_MAXIMUM_RETURN_ERROR_M = 0.15
GUIDED_TRANSLATION_MAXIMUM_VERTICAL_ERROR_M = 0.15


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            # An abrupt power loss can leave a preallocated NDJSON tail full
            # of NUL bytes. It contains no record and is safe to ignore.
            if "\x00" in line and not line.replace("\x00", "").strip():
                continue
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"{path}:{line_number}: invalid JSON: {exc}"
                ) from exc
            if isinstance(row, dict):
                rows.append(row)
    return rows


def _finite_vector(row: dict[str, Any], key: str, length: int) -> np.ndarray | None:
    value = row.get(key)
    if not isinstance(value, list) or len(value) != length:
        return None
    array = np.asarray(value, dtype=np.float64)
    return array if np.all(np.isfinite(array)) else None


def _session_file_matches(
    session: Path,
    value: Any,
    expected_sha256: Any,
) -> bool:
    if not isinstance(value, str) or not isinstance(expected_sha256, str):
        return False
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = session / candidate
    try:
        resolved = candidate.resolve(strict=True)
    except OSError:
        return False
    if resolved.parent != session or not resolved.is_file():
        return False
    return hashlib.sha256(resolved.read_bytes()).hexdigest() == expected_sha256


def _read_session_json(session: Path, value: Any) -> dict[str, Any]:
    if not isinstance(value, str):
        return {}
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = session / candidate
    try:
        resolved = candidate.resolve(strict=True)
    except OSError:
        return {}
    if resolved.parent != session or not resolved.is_file():
        return {}
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _integer(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def trajectory_metrics(
    rows: list[dict[str, Any]],
    *,
    stationary_window_s: float,
) -> dict[str, Any]:
    valid: list[tuple[int, np.ndarray, np.ndarray, np.ndarray]] = []
    invalid_rows = 0
    for row in rows:
        try:
            host_ns = int(row["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            invalid_rows += 1
            continue
        position = _finite_vector(row, "position_m", 3)
        velocity = _finite_vector(row, "linear_velocity_mps", 3)
        quaternion = _finite_vector(row, "quaternion_xyzw", 4)
        if position is None or velocity is None or quaternion is None:
            invalid_rows += 1
            continue
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-9:
            invalid_rows += 1
            continue
        valid.append((host_ns, position, velocity, quaternion / norm))

    if len(valid) < 2:
        return {
            "samples": len(valid),
            "invalid_rows": invalid_rows,
            "duration_s": 0.0,
        }

    timestamps_ns = np.asarray([sample[0] for sample in valid], dtype=np.int64)
    positions = np.stack([sample[1] for sample in valid])
    velocities = np.stack([sample[2] for sample in valid])
    quaternions = np.stack([sample[3] for sample in valid])
    duration_s = float((timestamps_ns[-1] - timestamps_ns[0]) / 1.0e9)
    rate_hz = (len(valid) - 1) / duration_s if duration_s > 0.0 else None
    jumps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    reported_speeds = np.linalg.norm(velocities, axis=1)
    intervals_s = np.diff(timestamps_ns).astype(np.float64) / 1.0e9
    derived_speeds = np.divide(
        jumps,
        intervals_s,
        out=np.full_like(jumps, np.inf),
        where=intervals_s > 0.0,
    )
    path_length_m = float(np.sum(jumps))

    relative_s = (timestamps_ns - timestamps_ns[0]) / 1.0e9
    start_mask = relative_s <= stationary_window_s
    end_mask = relative_s >= max(0.0, duration_s - stationary_window_s)
    start_position = np.median(positions[start_mask], axis=0)
    end_position = np.median(positions[end_mask], axis=0)
    start_radius = float(
        np.max(np.linalg.norm(positions[start_mask] - start_position, axis=1))
    )
    end_radius = float(
        np.max(np.linalg.norm(positions[end_mask] - end_position, axis=1))
    )

    quaternion_dots = np.abs(
        np.sum(quaternions[:-1] * quaternions[1:], axis=1)
    )
    quaternion_dots = np.clip(quaternion_dots, -1.0, 1.0)
    attitude_jumps_deg = np.degrees(2.0 * np.arccos(quaternion_dots))
    non_monotonic = int(np.count_nonzero(np.diff(timestamps_ns) <= 0))
    return {
        "samples": len(valid),
        "invalid_rows": invalid_rows,
        "duration_s": duration_s,
        "odometry_rate_hz": rate_hz,
        "path_length_m": path_length_m,
        "maximum_position_jump_m": float(np.max(jumps)),
        "p95_position_jump_m": float(np.percentile(jumps, 95)),
        "maximum_derived_speed_mps": float(np.max(derived_speeds)),
        "p95_derived_speed_mps": float(np.percentile(derived_speeds, 95)),
        "maximum_reported_speed_mps": float(np.max(reported_speeds)),
        "p95_reported_speed_mps": float(
            np.percentile(reported_speeds, 95)
        ),
        "start_stationary_radius_m": start_radius,
        "end_stationary_radius_m": end_radius,
        "maximum_stationary_drift_m": max(start_radius, end_radius),
        "return_to_start_error_m": float(
            np.linalg.norm(end_position - start_position)
        ),
        "maximum_attitude_jump_deg": float(np.max(attitude_jumps_deg)),
        "non_monotonic_timestamps": non_monotonic,
    }


def cube_reference_metrics(
    odometry_rows: list[dict[str, Any]],
    cube_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    lio: list[tuple[int, np.ndarray]] = []
    for row in odometry_rows:
        position = _finite_vector(row, "position_m", 3)
        if position is not None and row.get("host_monotonic_ns") is not None:
            lio.append((int(row["host_monotonic_ns"]), position))
    cube: list[tuple[int, np.ndarray]] = []
    for row in cube_rows:
        if row.get("type") != "LOCAL_POSITION_NED":
            continue
        data = row.get("data")
        if not isinstance(data, dict):
            continue
        try:
            position = np.asarray(
                (data["x"], data["y"], data["z"]),
                dtype=np.float64,
            )
            host_ns = int(row["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if np.all(np.isfinite(position)):
            cube.append((host_ns, position))
    if len(lio) < 2 or len(cube) < 2:
        return {
            "available": False,
            "paired_samples": 0,
            "note": "Cube local position is a reference, not ground truth",
        }

    cube_non_monotonic = int(
        np.count_nonzero(
            np.diff(np.asarray([sample[0] for sample in cube], dtype=np.int64))
            <= 0
        )
    )
    cube.sort(key=lambda sample: sample[0])
    cube_times = np.asarray([sample[0] for sample in cube], dtype=np.int64)
    cube_positions = np.stack([sample[1] for sample in cube])
    paired_lio: list[np.ndarray] = []
    paired_cube: list[np.ndarray] = []
    pairing_errors_ms: list[float] = []
    for host_ns, position in lio:
        index = int(np.searchsorted(cube_times, host_ns))
        candidates = [
            candidate
            for candidate in (index - 1, index)
            if 0 <= candidate < len(cube_times)
        ]
        nearest = min(
            candidates,
            key=lambda candidate: abs(int(cube_times[candidate]) - host_ns),
        )
        if abs(int(cube_times[nearest]) - host_ns) <= 150_000_000:
            paired_lio.append(position)
            paired_cube.append(cube_positions[nearest])
            pairing_errors_ms.append(
                abs(int(cube_times[nearest]) - host_ns) / 1.0e6
            )
    if len(paired_lio) < 10:
        return {
            "available": False,
            "paired_samples": len(paired_lio),
            "note": "Cube local position is a reference, not ground truth",
        }

    lio_array = np.stack(paired_lio)
    cube_array = np.stack(paired_cube)
    lio_delta = lio_array - lio_array[0]
    cube_delta = cube_array - cube_array[0]
    covariance = lio_delta[:, :2].T @ cube_delta[:, :2]
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = u_matrix @ vt_matrix
    if np.linalg.det(rotation) < 0.0:
        u_matrix[:, -1] *= -1.0
        rotation = u_matrix @ vt_matrix
    aligned_xy = lio_delta[:, :2] @ rotation
    horizontal_error = np.linalg.norm(
        aligned_xy - cube_delta[:, :2],
        axis=1,
    )
    vertical_error = np.abs(lio_delta[:, 2] - cube_delta[:, 2])
    lio_path = float(
        np.sum(np.linalg.norm(np.diff(lio_delta[:, :2], axis=0), axis=1))
    )
    cube_path = float(
        np.sum(np.linalg.norm(np.diff(cube_delta[:, :2], axis=0), axis=1))
    )
    return {
        "available": True,
        "paired_samples": len(paired_lio),
        "cube_non_monotonic_timestamps": cube_non_monotonic,
        "pairing_error_p95_ms": float(
            np.percentile(pairing_errors_ms, 95)
        ),
        "horizontal_aligned_rmse_m": float(
            np.sqrt(np.mean(np.square(horizontal_error)))
        ),
        "horizontal_aligned_p95_m": float(
            np.percentile(horizontal_error, 95)
        ),
        "vertical_rmse_m": float(
            np.sqrt(np.mean(np.square(vertical_error)))
        ),
        "vertical_p95_m": float(np.percentile(vertical_error, 95)),
        "lio_path_length_m": lio_path,
        "cube_path_length_m": cube_path,
        "path_length_ratio": lio_path / cube_path if cube_path > 0.05 else None,
        "note": "Cube local position is a reference, not ground truth",
    }


def _quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x_value, y_value, z_value, w_value = quaternion
    norm = float(np.dot(quaternion, quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("quaternion norm is zero")
    scale = 2.0 / norm
    return np.asarray(
        (
            (
                1.0 - scale * (y_value * y_value + z_value * z_value),
                scale * (x_value * y_value - z_value * w_value),
                scale * (x_value * z_value + y_value * w_value),
            ),
            (
                scale * (x_value * y_value + z_value * w_value),
                1.0 - scale * (x_value * x_value + z_value * z_value),
                scale * (y_value * z_value - x_value * w_value),
            ),
            (
                scale * (x_value * z_value - y_value * w_value),
                scale * (y_value * z_value + x_value * w_value),
                1.0 - scale * (x_value * x_value + y_value * y_value),
            ),
        ),
        dtype=np.float64,
    )


def _euler_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def cube_attitude_metrics(
    odometry_rows: list[dict[str, Any]],
    cube_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    lio: list[tuple[int, np.ndarray]] = []
    for row in odometry_rows:
        quaternion = _finite_vector(row, "quaternion_xyzw", 4)
        try:
            host_ns = int(row["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if quaternion is not None and np.linalg.norm(quaternion) > 1.0e-9:
            lio.append((host_ns, _quaternion_matrix_xyzw(quaternion)))

    cube: list[tuple[int, np.ndarray]] = []
    for row in cube_rows:
        if row.get("type") != "ATTITUDE":
            continue
        data = row.get("data")
        if not isinstance(data, dict):
            continue
        try:
            values = np.asarray(
                (data["roll"], data["pitch"], data["yaw"]),
                dtype=np.float64,
            )
            host_ns = int(row["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if np.all(np.isfinite(values)):
            cube.append((host_ns, _euler_matrix(*values)))

    if len(lio) < 2 or len(cube) < 2:
        return {"available": False, "paired_samples": 0}
    cube.sort(key=lambda sample: sample[0])
    cube_times = np.asarray([sample[0] for sample in cube], dtype=np.int64)
    paired_lio: list[np.ndarray] = []
    paired_cube: list[np.ndarray] = []
    pairing_errors_ms: list[float] = []
    for host_ns, rotation in lio:
        index = int(np.searchsorted(cube_times, host_ns))
        candidates = [
            candidate
            for candidate in (index - 1, index)
            if 0 <= candidate < len(cube)
        ]
        if not candidates:
            continue
        nearest = min(
            candidates,
            key=lambda candidate: abs(int(cube_times[candidate]) - host_ns),
        )
        difference_ns = abs(int(cube_times[nearest]) - host_ns)
        if difference_ns <= 150_000_000:
            paired_lio.append(rotation)
            paired_cube.append(cube[nearest][1])
            pairing_errors_ms.append(difference_ns / 1.0e6)
    if len(paired_lio) < 10:
        return {
            "available": False,
            "paired_samples": len(paired_lio),
        }

    world_alignment = paired_cube[0] @ paired_lio[0].T
    errors_deg: list[float] = []
    for lio_rotation, cube_rotation in zip(paired_lio, paired_cube):
        error_rotation = cube_rotation.T @ world_alignment @ lio_rotation
        cosine = float(
            np.clip((np.trace(error_rotation) - 1.0) * 0.5, -1.0, 1.0)
        )
        errors_deg.append(math.degrees(math.acos(cosine)))
    return {
        "available": True,
        "paired_samples": len(paired_lio),
        "pairing_error_p95_ms": float(
            np.percentile(pairing_errors_ms, 95)
        ),
        "attitude_error_rms_deg": float(
            np.sqrt(np.mean(np.square(errors_deg)))
        ),
        "attitude_error_p95_deg": float(np.percentile(errors_deg, 95)),
        "attitude_error_maximum_deg": float(np.max(errors_deg)),
        "note": "initial world-frame rotation is aligned before comparison",
    }


def sensor_trace_metrics(
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    stamps: list[int] = []
    delivery_offsets_ms: list[float] = []
    invalid_rows = 0
    for row in rows:
        try:
            ros_time_ns = int(row["ros_time_ns"])
            host_unix_ns = int(row["host_unix_ns"])
        except (KeyError, TypeError, ValueError):
            invalid_rows += 1
            continue
        if ros_time_ns <= 0 or host_unix_ns <= 0:
            invalid_rows += 1
            continue
        stamps.append(ros_time_ns)
        delivery_offsets_ms.append((host_unix_ns - ros_time_ns) / 1.0e6)
    if len(stamps) < 2:
        return {
            "samples": len(stamps),
            "invalid_rows": invalid_rows,
            "duration_s": 0.0,
        }
    stamp_array = np.asarray(stamps, dtype=np.int64)
    intervals_ns = np.diff(stamp_array)
    duration_s = float((stamp_array[-1] - stamp_array[0]) / 1.0e9)
    offset_array = np.asarray(delivery_offsets_ms, dtype=np.float64)
    median_offset_ms = float(np.median(offset_array))
    return {
        "samples": len(stamps),
        "invalid_rows": invalid_rows,
        "duration_s": duration_s,
        "rate_hz": (
            (len(stamps) - 1) / duration_s if duration_s > 0.0 else None
        ),
        "non_monotonic_timestamps": int(
            np.count_nonzero(intervals_ns <= 0)
        ),
        "period_p95_ms": float(np.percentile(intervals_ns / 1.0e6, 95)),
        "delivery_offset_median_ms": median_offset_ms,
        "delivery_jitter_p95_ms": float(
            np.percentile(np.abs(offset_array - median_offset_ms), 95)
        ),
    }


def guided_translation_metrics(payload: dict[str, Any]) -> dict[str, Any]:
    if payload.get("guide_kind") != "translation":
        return {"applicable": False, "available": False}
    raw_captures = payload.get("captures")
    if not isinstance(raw_captures, list):
        return {
            "applicable": True,
            "available": False,
            "complete": bool(payload.get("guide_complete")),
            "missing_phases": sorted(GUIDED_TRANSLATION_TARGETS),
        }

    captures: dict[str, dict[str, Any]] = {}
    invalid_phases: list[str] = []
    for raw_capture in raw_captures:
        if not isinstance(raw_capture, dict):
            continue
        phase_id = str(raw_capture.get("phase_id", ""))
        expected = GUIDED_TRANSLATION_TARGETS.get(phase_id)
        if expected is None or phase_id in captures:
            invalid_phases.append(phase_id or "unknown")
            continue
        observed_raw = raw_capture.get("observed_m")
        target_raw = raw_capture.get("target_m")
        try:
            observed = np.asarray(observed_raw, dtype=np.float64)
            target = np.asarray(target_raw, dtype=np.float64)
            samples = int(raw_capture.get("samples", 0))
        except (TypeError, ValueError, OverflowError):
            invalid_phases.append(phase_id)
            continue
        if (
            observed.shape != (3,)
            or target.shape != (3,)
            or not np.all(np.isfinite(observed))
            or not np.all(np.isfinite(target))
            or not np.allclose(target, expected, rtol=0.0, atol=1.0e-9)
            or samples < 0
        ):
            invalid_phases.append(phase_id)
            continue
        captures[phase_id] = {
            "observed_m": observed,
            "target_m": np.asarray(expected, dtype=np.float64),
            "samples": samples,
        }

    missing_phases = sorted(set(GUIDED_TRANSLATION_TARGETS) - captures.keys())
    if missing_phases or invalid_phases:
        return {
            "applicable": True,
            "available": False,
            "complete": bool(payload.get("guide_complete")),
            "missing_phases": missing_phases,
            "invalid_phases": sorted(invalid_phases),
        }

    forward = captures["forward_1"]["observed_m"]
    right = captures["right_1"]["observed_m"]
    return_positions = [
        captures[phase_id]["observed_m"]
        for phase_id in ("center_1", "center_2", "final_still")
    ]
    capture_values = list(captures.values())
    target_errors = [
        float(
            np.linalg.norm(
                capture["observed_m"][:2] - capture["target_m"][:2]
            )
        )
        for capture in capture_values
    ]
    return {
        "applicable": True,
        "available": True,
        "complete": bool(payload.get("guide_complete")),
        "reference": payload.get("reference"),
        "cube_local_position_used_as_ground_truth": payload.get(
            "cube_local_position_used_as_ground_truth"
        ),
        "minimum_capture_samples": min(
            capture["samples"] for capture in capture_values
        ),
        "forward_scale": float(forward[0] / 0.5),
        "right_scale": float(right[1] / 0.5),
        "maximum_cross_axis_error_m": max(
            abs(float(forward[1])),
            abs(float(right[0])),
        ),
        "maximum_return_error_m": max(
            float(np.linalg.norm(position[:2]))
            for position in return_positions
        ),
        "maximum_vertical_error_m": max(
            abs(float(capture["observed_m"][2]))
            for capture in capture_values
        ),
        "maximum_horizontal_target_error_m": max(target_errors),
        "captures": [
            {
                "phase_id": phase_id,
                "observed_m": captures[phase_id]["observed_m"].tolist(),
                "target_m": captures[phase_id]["target_m"].tolist(),
                "samples": captures[phase_id]["samples"],
            }
            for phase_id in GUIDED_TRANSLATION_TARGETS
        ],
    }


def _check(
    checks: list[dict[str, Any]],
    name: str,
    passed: bool,
    *,
    value: Any,
    requirement: str,
) -> None:
    checks.append(
        {
            "name": name,
            "passed": bool(passed),
            "value": value,
            "requirement": requirement,
        }
    )


def validate_lio_session(
    session_path: Path | str,
    config: ProjectConfig,
) -> tuple[Path, dict[str, Any], str]:
    session = Path(session_path).resolve()
    odometry_rows = _read_ndjson(session / "lio_odometry.ndjson")
    diagnostic_rows = _read_ndjson(session / "lio_diagnostics.ndjson")
    cube_rows = _read_ndjson(session / "cube_reference.ndjson")
    imu_rows = _read_ndjson(session / "lio_imu.ndjson")
    lidar_frame_rows = _read_ndjson(session / "lio_lidar_frames.ndjson")
    manifest_path = session / "manifest.json"
    manifest = (
        json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest_path.is_file()
        else {}
    )
    guide_result_required = manifest.get("guide_result_required") is True
    guide_result_intact = (
        not guide_result_required
        or _session_file_matches(
            session,
            manifest.get("guide_result"),
            manifest.get("guide_result_sha256"),
        )
    )
    guide_result = _read_session_json(
        session,
        manifest.get("guide_result"),
    )
    guided_translation = guided_translation_metrics(guide_result)
    validation = config.lidar_inertial_odometry.validation
    trajectory = trajectory_metrics(
        odometry_rows,
        stationary_window_s=validation.stationary_window_s,
    )
    cube_reference = cube_reference_metrics(odometry_rows, cube_rows)
    cube_attitude = cube_attitude_metrics(odometry_rows, cube_rows)
    imu_trace = sensor_trace_metrics(imu_rows)
    lidar_trace = sensor_trace_metrics(lidar_frame_rows)
    latest_diagnostics = (
        diagnostic_rows[-1].get("diagnostics", {})
        if diagnostic_rows
        else {}
    )
    imu_diagnostics = latest_diagnostics.get("imu", {})
    lidar_diagnostics = latest_diagnostics.get("lidar", {})
    imu_clock = imu_diagnostics.get("clock", {})
    lidar_clock = lidar_diagnostics.get("clock", {})
    maximum_resets = max(
        [
            _integer(
                row.get("diagnostics", {})
                .get(sensor, {})
                .get("clock", {})
                .get("resets", 0),
                validation.maximum_clock_resets + 1,
            )
            for row in diagnostic_rows
            for sensor in ("imu", "lidar")
        ]
        or [0]
    )

    cube_link_direction = manifest.get("cube_link_direction")
    poc_ready_tune_only = bool(
        manifest.get("slam_poc")
        and cube_link_direction == "telemetry_plus_ready_tune"
        and manifest.get("navigation_enabled") is False
    )
    flight_shadow_telemetry_only = bool(
        manifest.get("flight_shadow") is True
        and cube_link_direction
        == "telemetry_stream_request_plus_ready_tune"
        and manifest.get("pose_sent_to_cube") is False
        and manifest.get("obstacle_output_to_cube") is False
        and manifest.get("velocity_output_to_cube") is False
        and manifest.get("navigation_enabled") is False
    )
    odometry_transport_only = bool(
        manifest.get("odometry_shadow_to_cube") is True
        and manifest.get("pose_sent_to_cube") is True
        and cube_link_direction == "telemetry_plus_odometry_shadow"
        and manifest.get("cube_external_nav_fusion_enabled") is False
        and manifest.get("navigation_enabled") is False
        and config.lidar_inertial_odometry.odometry_shadow_to_cube_enabled
        and not config.lidar_inertial_odometry.pose_output_to_cube_enabled
        and not config.navigation.external_nav_to_cube_enabled
    )
    checks: list[dict[str, Any]] = []
    _check(
        checks,
        "shadow_contract",
        (
            (
                manifest.get("pose_sent_to_cube") is False
                and (
                    cube_link_direction == "read_only"
                    or poc_ready_tune_only
                    or flight_shadow_telemetry_only
                )
            )
            or odometry_transport_only
        )
        and not config.lidar_inertial_odometry.pose_output_to_cube_enabled,
        value={
            "pose_sent_to_cube": manifest.get("pose_sent_to_cube"),
            "cube_link_direction": cube_link_direction,
            "poc_ready_tune_only": poc_ready_tune_only,
            "flight_shadow_telemetry_only": flight_shadow_telemetry_only,
            "odometry_transport_only": odometry_transport_only,
        },
        requirement=(
            "no active pose/navigation output; guarded ignored ODOMETRY allowed"
        ),
    )
    _check(
        checks,
        "session_provenance",
        (
            manifest.get("kind") == "lio_shadow"
            and manifest.get("status") == "complete"
            and manifest.get("backend")
            == config.lidar_inertial_odometry.backend
            and manifest.get("backend_revision")
            == config.lidar_inertial_odometry.backend_revision
            and _session_file_matches(
                session,
                manifest.get("config_snapshot"),
                manifest.get("config_sha256"),
            )
            and _session_file_matches(
                session,
                manifest.get("resolved_fast_lio_config"),
                manifest.get("resolved_fast_lio_config_sha256"),
            )
            and guide_result_intact
        ),
        value={
            "kind": manifest.get("kind"),
            "status": manifest.get("status"),
            "backend": manifest.get("backend"),
            "backend_revision": manifest.get("backend_revision"),
            "guide_result_required": guide_result_required,
            "guide_result_intact": guide_result_intact,
        },
        requirement=(
            "complete session from the pinned backend with intact configs "
            "and guided evidence"
        ),
    )
    _check(
        checks,
        "duration",
        trajectory.get("duration_s", 0.0) >= validation.minimum_duration_s,
        value=trajectory.get("duration_s"),
        requirement=f">= {validation.minimum_duration_s:.1f} s",
    )
    odometry_rate = trajectory.get("odometry_rate_hz")
    _check(
        checks,
        "odometry_rate",
        odometry_rate is not None
        and odometry_rate >= validation.minimum_odometry_rate_hz,
        value=odometry_rate,
        requirement=f">= {validation.minimum_odometry_rate_hz:.1f} Hz",
    )
    _check(
        checks,
        "finite_continuous_trajectory",
        (
            trajectory.get("invalid_rows", 1) == 0
            and trajectory.get("non_monotonic_timestamps", 1) == 0
        ),
        value={
            "invalid_rows": trajectory.get("invalid_rows"),
            "non_monotonic_timestamps": trajectory.get(
                "non_monotonic_timestamps"
            ),
        },
        requirement="zero invalid rows and zero time regressions",
    )
    _check(
        checks,
        "position_jump",
        trajectory.get("maximum_position_jump_m", math.inf)
        <= validation.maximum_position_jump_m,
        value=trajectory.get("maximum_position_jump_m"),
        requirement=f"<= {validation.maximum_position_jump_m:.2f} m",
    )
    _check(
        checks,
        "derived_speed",
        trajectory.get("maximum_derived_speed_mps", math.inf)
        <= validation.maximum_speed_mps,
        value=trajectory.get("maximum_derived_speed_mps"),
        requirement=f"<= {validation.maximum_speed_mps:.2f} m/s",
    )
    _check(
        checks,
        "attitude_jump",
        trajectory.get("maximum_attitude_jump_deg", math.inf)
        <= validation.maximum_attitude_jump_deg,
        value=trajectory.get("maximum_attitude_jump_deg"),
        requirement=f"<= {validation.maximum_attitude_jump_deg:.1f} deg",
    )
    _check(
        checks,
        "stationary_drift",
        trajectory.get("maximum_stationary_drift_m", math.inf)
        <= validation.maximum_stationary_drift_m,
        value=trajectory.get("maximum_stationary_drift_m"),
        requirement=f"<= {validation.maximum_stationary_drift_m:.2f} m",
    )
    _check(
        checks,
        "return_to_start",
        trajectory.get("return_to_start_error_m", math.inf)
        <= validation.maximum_return_to_start_error_m,
        value=trajectory.get("return_to_start_error_m"),
        requirement=(
            f"<= {validation.maximum_return_to_start_error_m:.2f} m"
        ),
    )
    _check(
        checks,
        "clock_synchronization",
        bool(latest_diagnostics.get("synchronized")),
        value={
            "imu_p95_ms": imu_clock.get("residual_p95_ms"),
            "lidar_p95_ms": lidar_clock.get("residual_p95_ms"),
        },
        requirement="both affine sensor clocks ready",
    )
    _check(
        checks,
        "sensor_health",
        (
            latest_diagnostics.get("mode") == "shadow"
            and latest_diagnostics.get("pose_output_to_cube") is False
            and bool(imu_diagnostics.get("connected"))
            and not imu_diagnostics.get("error")
            and _integer(imu_diagnostics.get("published"), 0) > 0
            and _integer(imu_diagnostics.get("checksum_errors"), 1) == 0
            and _integer(imu_diagnostics.get("payload_errors"), 1) == 0
            and _integer(imu_diagnostics.get("queue_drops"), 1) == 0
            and bool(lidar_diagnostics.get("connected"))
            and not lidar_diagnostics.get("error")
            and _integer(lidar_diagnostics.get("published"), 0) > 0
            and _integer(lidar_diagnostics.get("queue_drops"), 1) == 0
            and _integer(
                lidar_diagnostics.get("non_monotonic_frames"),
                1,
            )
            == 0
            and _integer(
                lidar_diagnostics.get("runtime_time_rejected_frames"),
                1,
            )
            == 0
        ),
        value={
            "imu_connected": imu_diagnostics.get("connected"),
            "imu_error": imu_diagnostics.get("error"),
            "imu_checksum_errors": imu_diagnostics.get("checksum_errors"),
            "imu_payload_errors": imu_diagnostics.get("payload_errors"),
            "imu_queue_drops": imu_diagnostics.get("queue_drops"),
            "lidar_connected": lidar_diagnostics.get("connected"),
            "lidar_error": lidar_diagnostics.get("error"),
            "lidar_queue_drops": lidar_diagnostics.get("queue_drops"),
            "lidar_non_monotonic_frames": lidar_diagnostics.get(
                "non_monotonic_frames"
            ),
            "lidar_time_rejected_frames": lidar_diagnostics.get(
                "time_rejected_frames"
            ),
            "lidar_runtime_time_rejected_frames": lidar_diagnostics.get(
                "runtime_time_rejected_frames"
            ),
            "lidar_frame_span_s": lidar_diagnostics.get("frame_span_s"),
            "lidar_maximum_point_gap_s": lidar_diagnostics.get(
                "maximum_point_gap_s"
            ),
        },
        requirement="healthy read-only bridge with zero corrupt sensor frames",
    )
    _check(
        checks,
        "clock_resets",
        maximum_resets <= validation.maximum_clock_resets,
        value=maximum_resets,
        requirement=f"<= {validation.maximum_clock_resets}",
    )
    imu_rate = imu_diagnostics.get("rate_hz")
    _check(
        checks,
        "imu_rate",
        imu_rate is not None
        and float(imu_rate)
        >= 0.9 * config.lidar_inertial_odometry.required_imu_rate_hz,
        value=imu_rate,
        requirement=(
            ">= 90% of "
            f"{config.lidar_inertial_odometry.required_imu_rate_hz:.0f} Hz"
        ),
    )
    lidar_rate = lidar_diagnostics.get("rate_hz")
    _check(
        checks,
        "lidar_rate",
        lidar_rate is not None and float(lidar_rate) >= 4.0,
        value=lidar_rate,
        requirement=">= 4.0 Hz",
    )
    _check(
        checks,
        "recorded_imu_timing",
        (
            imu_trace.get("invalid_rows", 1) == 0
            and imu_trace.get("non_monotonic_timestamps", 1) == 0
            and imu_trace.get("rate_hz") is not None
            and float(imu_trace["rate_hz"])
            >= 0.9 * config.lidar_inertial_odometry.required_imu_rate_hz
        ),
        value=imu_trace,
        requirement="monotonic recorded IMU trace at >= 90% required rate",
    )
    lidar_layout_valid = bool(lidar_frame_rows) and all(
        _integer(row.get("points"), 0) > 0
        and _integer(row.get("point_step"), 0) == 32
        for row in lidar_frame_rows
    )
    _check(
        checks,
        "recorded_lidar_timing",
        (
            lidar_layout_valid
            and lidar_trace.get("invalid_rows", 1) == 0
            and lidar_trace.get("non_monotonic_timestamps", 1) == 0
            and lidar_trace.get("rate_hz") is not None
            and float(lidar_trace["rate_hz"]) >= 4.0
        ),
        value={**lidar_trace, "point_layout_valid": lidar_layout_valid},
        requirement="monotonic >= 4 Hz JT16 trace with 32-byte point layout",
    )
    if (
        guide_result_required
        and manifest.get("visual_guide") == "translation"
    ):
        minimum_capture_samples = _integer(
            guided_translation.get("minimum_capture_samples"),
            0,
        )
        _check(
            checks,
            "guided_translation_captures",
            (
                guide_result_intact
                and bool(guided_translation.get("available"))
                and bool(guided_translation.get("complete"))
                and minimum_capture_samples
                >= GUIDED_TRANSLATION_MINIMUM_CAPTURE_SAMPLES
                and guided_translation.get(
                    "cube_local_position_used_as_ground_truth"
                )
                is False
            ),
            value={
                "available": guided_translation.get("available"),
                "complete": guided_translation.get("complete"),
                "minimum_capture_samples": minimum_capture_samples,
                "reference": guided_translation.get("reference"),
                "cube_local_position_used_as_ground_truth": (
                    guided_translation.get(
                        "cube_local_position_used_as_ground_truth"
                    )
                ),
            },
            requirement=(
                f">= {GUIDED_TRANSLATION_MINIMUM_CAPTURE_SAMPLES} samples "
                "at every tape-marked phase; Cube position is diagnostic only"
            ),
        )
        forward_scale = guided_translation.get("forward_scale")
        right_scale = guided_translation.get("right_scale")
        _check(
            checks,
            "guided_translation_scale",
            (
                forward_scale is not None
                and right_scale is not None
                and validation.minimum_cube_path_ratio
                <= float(forward_scale)
                <= validation.maximum_cube_path_ratio
                and validation.minimum_cube_path_ratio
                <= float(right_scale)
                <= validation.maximum_cube_path_ratio
            ),
            value={
                "forward": forward_scale,
                "right": right_scale,
            },
            requirement=(
                f"each 0.50 m axis scale is "
                f"{validation.minimum_cube_path_ratio:.2f} to "
                f"{validation.maximum_cube_path_ratio:.2f}"
            ),
        )
        cross_axis_error = guided_translation.get(
            "maximum_cross_axis_error_m"
        )
        _check(
            checks,
            "guided_translation_cross_axis",
            (
                cross_axis_error is not None
                and float(cross_axis_error)
                <= GUIDED_TRANSLATION_MAXIMUM_CROSS_AXIS_ERROR_M
            ),
            value=cross_axis_error,
            requirement=(
                f"<= {GUIDED_TRANSLATION_MAXIMUM_CROSS_AXIS_ERROR_M:.2f} m"
            ),
        )
        return_error = guided_translation.get("maximum_return_error_m")
        _check(
            checks,
            "guided_translation_returns",
            (
                return_error is not None
                and float(return_error)
                <= GUIDED_TRANSLATION_MAXIMUM_RETURN_ERROR_M
            ),
            value=return_error,
            requirement=(
                f"<= {GUIDED_TRANSLATION_MAXIMUM_RETURN_ERROR_M:.2f} m "
                "at both center marks and final stillness"
            ),
        )
        vertical_error = guided_translation.get(
            "maximum_vertical_error_m"
        )
        _check(
            checks,
            "guided_translation_vertical",
            (
                vertical_error is not None
                and float(vertical_error)
                <= GUIDED_TRANSLATION_MAXIMUM_VERTICAL_ERROR_M
            ),
            value=vertical_error,
            requirement=(
                f"<= {GUIDED_TRANSLATION_MAXIMUM_VERTICAL_ERROR_M:.2f} m"
            ),
        )
    cube_samples = _integer(cube_reference.get("paired_samples"), 0)
    cube_path = cube_reference.get("cube_path_length_m")
    cube_horizontal_rmse = cube_reference.get("horizontal_aligned_rmse_m")
    cube_vertical_rmse = cube_reference.get("vertical_rmse_m")
    cube_path_ratio = cube_reference.get("path_length_ratio")
    cube_attitude_p95 = cube_attitude.get("attitude_error_p95_deg")
    _check(
        checks,
        "independent_cube_reference",
        (
            bool(cube_reference.get("available"))
            and cube_samples >= validation.minimum_cube_reference_samples
            and _integer(
                cube_reference.get("cube_non_monotonic_timestamps"),
                1,
            )
            == 0
        ),
        value={
            "available": cube_reference.get("available"),
            "paired_samples": cube_samples,
            "non_monotonic_timestamps": cube_reference.get(
                "cube_non_monotonic_timestamps"
            ),
        },
        requirement=(
            f">= {validation.minimum_cube_reference_samples} synchronized "
            "Cube LOCAL_POSITION_NED samples"
        ),
    )
    _check(
        checks,
        "cube_reference_excitation",
        cube_path is not None
        and float(cube_path) >= validation.minimum_cube_reference_path_m,
        value=cube_path,
        requirement=f">= {validation.minimum_cube_reference_path_m:.2f} m",
    )
    _check(
        checks,
        "cube_horizontal_agreement",
        cube_horizontal_rmse is not None
        and float(cube_horizontal_rmse)
        <= validation.maximum_cube_horizontal_rmse_m,
        value=cube_horizontal_rmse,
        requirement=f"<= {validation.maximum_cube_horizontal_rmse_m:.2f} m RMSE",
    )
    _check(
        checks,
        "cube_vertical_agreement",
        cube_vertical_rmse is not None
        and float(cube_vertical_rmse)
        <= validation.maximum_cube_vertical_rmse_m,
        value=cube_vertical_rmse,
        requirement=f"<= {validation.maximum_cube_vertical_rmse_m:.2f} m RMSE",
    )
    _check(
        checks,
        "cube_path_scale",
        cube_path_ratio is not None
        and validation.minimum_cube_path_ratio
        <= float(cube_path_ratio)
        <= validation.maximum_cube_path_ratio,
        value=cube_path_ratio,
        requirement=(
            f"{validation.minimum_cube_path_ratio:.2f} to "
            f"{validation.maximum_cube_path_ratio:.2f}"
        ),
    )
    _check(
        checks,
        "cube_attitude_agreement",
        (
            bool(cube_attitude.get("available"))
            and _integer(cube_attitude.get("paired_samples"), 0)
            >= validation.minimum_cube_reference_samples
            and cube_attitude_p95 is not None
            and float(cube_attitude_p95)
            <= validation.maximum_cube_attitude_p95_deg
        ),
        value={
            "paired_samples": cube_attitude.get("paired_samples"),
            "p95_error_deg": cube_attitude_p95,
        },
        requirement=(
            f"p95 <= {validation.maximum_cube_attitude_p95_deg:.1f} deg "
            "after initial frame alignment"
        ),
    )
    _check(
        checks,
        "sensor_extrinsics",
        (
            config.external_imu.position_verified
            and config.calibration.imu_to_body_extrinsics_verified
            and config.calibration.lidar_to_body_extrinsics_verified
        ),
        value={
            "imu_position_verified": config.external_imu.position_verified,
            "imu_to_body": (
                config.calibration.imu_to_body_extrinsics_verified
            ),
            "lidar_to_body": (
                config.calibration.lidar_to_body_extrinsics_verified
            ),
        },
        requirement="measured IMU and lidar transforms",
    )
    _check(
        checks,
        "sensor_models",
        (
            config.calibration.imu_noise_profile_verified
            and config.lidar.correction_verified
        ),
        value={
            "imu_noise_profile": (
                config.calibration.imu_noise_profile_verified
            ),
            "jt16_correction": config.lidar.correction_verified,
        },
        requirement="verified IMU noise and JT16 correction models",
    )

    passed = all(check["passed"] for check in checks)
    eligible_for_cube_pose_approval = bool(
        passed and manifest.get("pose_sent_to_cube") is False
    )
    report = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "session": str(session),
        "backend": config.lidar_inertial_odometry.backend,
        "backend_revision": (
            config.lidar_inertial_odometry.backend_revision
        ),
        "result": "pass" if passed else "fail",
        "eligible_for_cube_pose_approval": eligible_for_cube_pose_approval,
        "pose_sent_to_cube": manifest.get("pose_sent_to_cube"),
        "session_provenance": {
            "manifest": str(manifest_path),
            "config_sha256": manifest.get("config_sha256"),
            "resolved_fast_lio_config_sha256": manifest.get(
                "resolved_fast_lio_config_sha256"
            ),
        },
        "checks": checks,
        "trajectory": trajectory,
        "cube_reference": cube_reference,
        "cube_attitude": cube_attitude,
        "guided_translation_reference": guided_translation,
        "sensor_traces": {
            "imu": imu_trace,
            "lidar": lidar_trace,
        },
        "latest_diagnostics": latest_diagnostics,
    }
    analysis_dir = session / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    report_path = analysis_dir / "lio_validation.json"
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    temporary = report_path.with_suffix(".json.tmp")
    temporary.write_bytes(report_bytes)
    temporary.replace(report_path)
    digest = hashlib.sha256(report_bytes).hexdigest()
    report_path.with_suffix(".sha256").write_text(
        f"{digest}  {report_path.name}\n",
        encoding="ascii",
    )
    return report_path, report, digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate a shadow LIO session")
    parser.add_argument("session", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
        report_path, report, digest = validate_lio_session(
            args.session,
            config,
        )
        print(
            json.dumps(
                {
                    "result": report["result"],
                    "report": str(report_path),
                    "sha256": digest,
                    "eligible_for_cube_pose_approval": report[
                        "eligible_for_cube_pose_approval"
                    ],
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if report["result"] == "pass" else 1
    except (ConfigError, OSError, ValueError) as exc:
        print(f"LIO validation error: {exc}")
        return 2
