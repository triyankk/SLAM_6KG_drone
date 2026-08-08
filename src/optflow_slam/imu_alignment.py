"""Estimate IM10A timing and gyro scale from recorded shadow sessions."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from .paths import PROJECT_ROOT


AXES = ("x", "y", "z")
DEFAULT_SEARCH_LIMIT_S = 0.100
DEFAULT_SEARCH_STEP_S = 0.001
MINIMUM_MOVING_RATE_RADS = 0.015
MINIMUM_MATCHED_SAMPLES = 40
MINIMUM_ACCEPTED_CORRELATION = 0.95
MINIMUM_ACCEPTED_GAIN = 0.80
MAXIMUM_ACCEPTED_GAIN = 1.20


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise ValueError(f"missing recording: {path}")
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _correlation_fit(
    measured: np.ndarray,
    reference: np.ndarray,
) -> tuple[float, float, float, float]:
    measured_centered = measured - float(np.mean(measured))
    reference_centered = reference - float(np.mean(reference))
    measured_energy = float(np.dot(measured_centered, measured_centered))
    reference_energy = float(np.dot(reference_centered, reference_centered))
    if measured_energy <= 0.0 or reference_energy <= 0.0:
        return 0.0, 0.0, 0.0, math.inf
    correlation = float(
        np.dot(measured_centered, reference_centered)
        / math.sqrt(measured_energy * reference_energy)
    )
    gain = float(
        np.dot(measured_centered, reference_centered) / measured_energy
    )
    bias = float(np.mean(reference) - gain * np.mean(measured))
    residual = reference - (gain * measured + bias)
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    return correlation, gain, bias, rmse


def cube_rate_alignment(
    imu_time_s: np.ndarray,
    imu_rates_rads: np.ndarray,
    cube_time_s: np.ndarray,
    cube_rates_rads: np.ndarray,
    *,
    search_limit_s: float = DEFAULT_SEARCH_LIMIT_S,
    search_step_s: float = DEFAULT_SEARCH_STEP_S,
) -> dict[str, dict[str, Any] | None]:
    """Correlate body rates; Cube receive time is only a timing cross-check."""

    result: dict[str, dict[str, Any] | None] = {}
    offsets = np.arange(
        -search_limit_s,
        search_limit_s + search_step_s * 0.5,
        search_step_s,
    )
    for axis_index, axis in enumerate(AXES):
        best: tuple[float, float, float, float, float, int] | None = None
        for cube_lag_s in offsets:
            query_time = imu_time_s + cube_lag_s
            in_range = (
                (query_time >= cube_time_s[0])
                & (query_time <= cube_time_s[-1])
            )
            if not np.any(in_range):
                continue
            measured = imu_rates_rads[in_range, axis_index]
            reference = np.interp(
                query_time[in_range],
                cube_time_s,
                cube_rates_rads[:, axis_index],
            )
            moving = (
                (np.abs(measured) >= MINIMUM_MOVING_RATE_RADS)
                | (np.abs(reference) >= MINIMUM_MOVING_RATE_RADS)
            )
            measured = measured[moving]
            reference = reference[moving]
            if measured.size < MINIMUM_MATCHED_SAMPLES:
                continue
            correlation, gain, bias, rmse = _correlation_fit(
                measured,
                reference,
            )
            candidate = (
                correlation,
                -rmse,
                -abs(float(cube_lag_s)),
                float(cube_lag_s),
                gain,
                int(measured.size),
            )
            if best is None or candidate[:3] > best[:3]:
                best = candidate
                best_bias = bias
                best_rmse = rmse
        if best is None:
            result[axis] = None
            continue
        correlation, _, _, cube_lag_s, gain, samples = best
        accepted = (
            correlation >= MINIMUM_ACCEPTED_CORRELATION
            and MINIMUM_ACCEPTED_GAIN <= gain <= MAXIMUM_ACCEPTED_GAIN
        )
        result[axis] = {
            "correlation": correlation,
            "cube_rate_per_imu_rate": gain,
            "bias_rads": best_bias,
            "rmse_rads": best_rmse,
            "matched_samples": samples,
            "cube_receive_time_lag_relative_to_imu_s": cube_lag_s,
            "imu_timestamp_advance_crosscheck_s": -cube_lag_s,
            "accepted": accepted,
        }
    return result


def _quaternion_multiply(
    left: np.ndarray,
    right: np.ndarray,
) -> np.ndarray:
    lx, ly, lz, lw = np.moveaxis(left, -1, 0)
    rx, ry, rz, rw = np.moveaxis(right, -1, 0)
    return np.stack(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        axis=-1,
    )


def _odometry_body_rates(
    timestamps_s: np.ndarray,
    quaternions_xyzw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    quaternions = quaternions_xyzw / np.linalg.norm(
        quaternions_xyzw,
        axis=1,
    )[:, None]
    for index in range(1, len(quaternions)):
        if np.dot(quaternions[index - 1], quaternions[index]) < 0.0:
            quaternions[index] *= -1.0
    conjugates = quaternions[:-1].copy()
    conjugates[:, :3] *= -1.0
    relative = _quaternion_multiply(conjugates, quaternions[1:])
    relative /= np.linalg.norm(relative, axis=1)[:, None]
    relative[relative[:, 3] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(relative[:, :3], axis=1)
    angles = 2.0 * np.arctan2(
        vector_norm,
        np.clip(relative[:, 3], -1.0, 1.0),
    )
    rotation_vectors = np.zeros_like(relative[:, :3])
    nonzero = vector_norm > 1.0e-12
    rotation_vectors[nonzero] = relative[nonzero, :3] * (
        angles[nonzero] / vector_norm[nonzero]
    )[:, None]
    intervals_s = np.diff(timestamps_s)
    return (
        timestamps_s[:-1],
        timestamps_s[1:],
        rotation_vectors / intervals_s[:, None],
        intervals_s,
    )


def _integral_samples(
    timestamps_s: np.ndarray,
    values: np.ndarray,
) -> np.ndarray:
    intervals = np.diff(timestamps_s)
    trapezoids = (values[:-1] + values[1:]) * 0.5 * intervals[:, None]
    return np.vstack((np.zeros(values.shape[1]), np.cumsum(trapezoids, axis=0)))


def _interpolate_columns(
    timestamps_s: np.ndarray,
    values: np.ndarray,
    query_s: np.ndarray,
) -> np.ndarray:
    return np.column_stack(
        [
            np.interp(query_s, timestamps_s, values[:, column])
            for column in range(values.shape[1])
        ]
    )


def odometry_rate_alignment(
    imu_time_s: np.ndarray,
    imu_rates_rads: np.ndarray,
    odometry_time_s: np.ndarray,
    odometry_quaternions_xyzw: np.ndarray,
    *,
    search_limit_s: float = DEFAULT_SEARCH_LIMIT_S,
    search_step_s: float = DEFAULT_SEARCH_STEP_S,
) -> dict[str, dict[str, Any] | None]:
    """Compare lidar-frame orientation increments with integrated IMU rates."""

    starts, ends, reference_rates, intervals = _odometry_body_rates(
        odometry_time_s,
        odometry_quaternions_xyzw,
    )
    regular = (intervals >= 0.15) & (intervals <= 0.30)
    starts = starts[regular]
    ends = ends[regular]
    intervals = intervals[regular]
    reference_rates = reference_rates[regular]
    integral = _integral_samples(imu_time_s, imu_rates_rads)
    offsets = np.arange(
        -search_limit_s,
        search_limit_s + search_step_s * 0.5,
        search_step_s,
    )
    result: dict[str, dict[str, Any] | None] = {}
    for axis_index, axis in enumerate(AXES):
        best: tuple[float, float, float, float, float, int] | None = None
        for offset_s in offsets:
            shifted_starts = starts + offset_s
            shifted_ends = ends + offset_s
            in_range = (
                (shifted_starts >= imu_time_s[0])
                & (shifted_ends <= imu_time_s[-1])
            )
            if not np.any(in_range):
                continue
            integrated = _interpolate_columns(
                imu_time_s,
                integral,
                shifted_ends[in_range],
            ) - _interpolate_columns(
                imu_time_s,
                integral,
                shifted_starts[in_range],
            )
            measured = integrated[:, axis_index] / intervals[in_range]
            reference = reference_rates[in_range, axis_index]
            moving = (
                (np.abs(measured) >= MINIMUM_MOVING_RATE_RADS)
                | (np.abs(reference) >= MINIMUM_MOVING_RATE_RADS)
            )
            measured = measured[moving]
            reference = reference[moving]
            if measured.size < MINIMUM_MATCHED_SAMPLES:
                continue
            correlation, gain, bias, rmse = _correlation_fit(
                measured,
                reference,
            )
            candidate = (
                correlation,
                -rmse,
                -abs(float(offset_s)),
                float(offset_s),
                gain,
                int(measured.size),
            )
            if best is None or candidate[:3] > best[:3]:
                best = candidate
                best_bias = bias
                best_rmse = rmse
        if best is None:
            result[axis] = None
            continue
        correlation, _, _, offset_s, gain, samples = best
        accepted = (
            correlation >= MINIMUM_ACCEPTED_CORRELATION
            and MINIMUM_ACCEPTED_GAIN <= gain <= MAXIMUM_ACCEPTED_GAIN
        )
        result[axis] = {
            "correlation": correlation,
            "odometry_rate_per_imu_rate": gain,
            "bias_rads": best_bias,
            "rmse_rads": best_rmse,
            "matched_intervals": samples,
            "time_offset_lidar_to_imu_candidate_s": offset_s,
            "accepted": accepted,
        }
    return result


def _sensor_arrays(
    session: Path,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    imu_rows = _read_ndjson(session / "lio_imu.ndjson")
    cube_rows = _read_ndjson(session / "cube_reference.ndjson")
    odometry_rows = _read_ndjson(session / "lio_odometry.ndjson")

    imu = [
        (int(row["ros_time_ns"]), *map(float, row["angular_velocity_rads"]))
        for row in imu_rows
        if isinstance(row.get("angular_velocity_rads"), list)
        and len(row["angular_velocity_rads"]) == 3
    ]
    cube = []
    for row in cube_rows:
        if row.get("type") != "ATTITUDE" or not isinstance(row.get("data"), dict):
            continue
        data = row["data"]
        try:
            cube.append(
                (
                    int(row["host_unix_ns"]),
                    float(data["rollspeed"]),
                    float(data["pitchspeed"]),
                    float(data["yawspeed"]),
                )
            )
        except (KeyError, TypeError, ValueError):
            continue
    odometry = [
        (int(row["ros_time_ns"]), *map(float, row["quaternion_xyzw"]))
        for row in odometry_rows
        if isinstance(row.get("quaternion_xyzw"), list)
        and len(row["quaternion_xyzw"]) == 4
    ]
    if len(imu) < 2 or len(cube) < 2 or len(odometry) < 2:
        raise ValueError(f"{session}: incomplete IMU, Cube, or odometry recording")
    imu_ns = np.asarray([row[0] for row in imu], dtype=np.int64)
    imu_rates = np.asarray([row[1:] for row in imu], dtype=np.float64)
    cube_ns = np.asarray([row[0] for row in cube], dtype=np.int64)
    cube_rates = np.asarray([row[1:] for row in cube], dtype=np.float64)
    odometry_ns = np.asarray([row[0] for row in odometry], dtype=np.int64)
    quaternions = np.asarray(
        [row[1:] for row in odometry],
        dtype=np.float64,
    )
    if (
        np.any(np.diff(imu_ns) <= 0)
        or np.any(np.diff(cube_ns) <= 0)
        or np.any(np.diff(odometry_ns) <= 0)
    ):
        raise ValueError(f"{session}: non-monotonic sensor timestamps")

    imu_cube_origin_ns = min(int(imu_ns[0]), int(cube_ns[0]))
    imu_odometry_origin_ns = min(int(imu_ns[0]), int(odometry_ns[0]))
    return (
        (imu_ns - imu_cube_origin_ns).astype(np.float64) / 1.0e9,
        imu_rates,
        (cube_ns - imu_cube_origin_ns).astype(np.float64) / 1.0e9,
        cube_rates,
        (imu_ns - imu_odometry_origin_ns).astype(np.float64) / 1.0e9,
        (odometry_ns - imu_odometry_origin_ns).astype(np.float64) / 1.0e9,
        quaternions,
    )


def _accepted_offsets(
    sessions: list[dict[str, Any]],
    section: str,
    key: str,
) -> list[float]:
    session_medians: list[float] = []
    for session in sessions:
        offsets = [
            float(axis[key])
            for axis in session[section].values()
            if axis is not None and axis["accepted"]
        ]
        if offsets:
            session_medians.append(float(np.median(offsets)))
    return session_medians


def analyze_sessions(session_paths: list[Path]) -> dict[str, Any]:
    sessions: list[dict[str, Any]] = []
    for path in session_paths:
        session = path.resolve()
        (
            imu_cube_time,
            imu_rates,
            cube_time,
            cube_rates,
            imu_odometry_time,
            odometry_time,
            quaternions,
        ) = _sensor_arrays(session)
        manifest_path = session / "manifest.json"
        manifest = (
            json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest_path.is_file()
            else {}
        )
        sessions.append(
            {
                "session": str(session),
                "visual_guide": manifest.get("visual_guide"),
                "status": manifest.get("status"),
                "source_sha256": {
                    name: _sha256(session / name)
                    for name in (
                        "lio_imu.ndjson",
                        "cube_reference.ndjson",
                        "lio_odometry.ndjson",
                    )
                },
                "cube_rate_alignment": cube_rate_alignment(
                    imu_cube_time,
                    imu_rates,
                    cube_time,
                    cube_rates,
                ),
                "lidar_odometry_rate_alignment": odometry_rate_alignment(
                    imu_odometry_time,
                    imu_rates,
                    odometry_time,
                    quaternions,
                ),
            }
        )

    lidar_offsets = _accepted_offsets(
        sessions,
        "lidar_odometry_rate_alignment",
        "time_offset_lidar_to_imu_candidate_s",
    )
    cube_offsets = _accepted_offsets(
        sessions,
        "cube_rate_alignment",
        "imu_timestamp_advance_crosscheck_s",
    )
    lidar_candidate = float(np.median(lidar_offsets)) if lidar_offsets else None
    cube_crosscheck = float(np.median(cube_offsets)) if cube_offsets else None
    agreement = (
        abs(lidar_candidate - cube_crosscheck)
        if lidar_candidate is not None and cube_crosscheck is not None
        else None
    )
    candidate_ready = (
        len(lidar_offsets) >= 2
        and len(cube_offsets) >= 2
        and agreement is not None
        and agreement <= 0.020
    )
    return {
        "schema_version": 1,
        "kind": "im10a_dynamic_alignment",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "result": "candidate_ready" if candidate_ready else "insufficient_evidence",
        "sessions": sessions,
        "summary": {
            "accepted_lidar_session_offsets_s": lidar_offsets,
            "accepted_cube_session_crosschecks_s": cube_offsets,
            "time_offset_lidar_to_imu_candidate_s": lidar_candidate,
            "cube_crosscheck_s": cube_crosscheck,
            "candidate_crosscheck_difference_s": agreement,
            "apply_automatically": False,
            "requires_shadow_ab_validation": True,
            "note": (
                "FAST-LIO subtracts this positive candidate from IMU stamps. "
                "Cube receive-time correlation is a sign cross-check, not an "
                "absolute timing reference."
            ),
        },
    }


def write_report(report: dict[str, Any], output_directory: Path) -> Path:
    output_directory.mkdir(parents=True, exist_ok=False)
    report_path = output_directory / "report.json"
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("ascii")
    report_path.write_bytes(report_bytes)
    digest = hashlib.sha256(report_bytes).hexdigest()
    (output_directory / "report.sha256").write_text(
        f"{digest}  report.json\n",
        encoding="ascii",
    )
    return report_path


def _default_output_directory() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return PROJECT_ROOT / "data" / "calibrations" / "im10a" / "alignment" / stamp


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze dynamic IM10A timing and gyro alignment in shadow logs",
    )
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = analyze_sessions(args.sessions)
        report_path = write_report(report, args.output or _default_output_directory())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"IMU alignment failed: {exc}")
        return 1
    summary = report["summary"]
    print(f"Result: {report['result']}")
    print(
        "Lidar-to-IMU offset candidate: "
        f"{summary['time_offset_lidar_to_imu_candidate_s']} s"
    )
    print(f"Cube timing cross-check: {summary['cube_crosscheck_s']} s")
    print("Applied automatically: no")
    print(f"Report: {report_path}")
    return 0 if report["result"] == "candidate_ready" else 2


if __name__ == "__main__":
    raise SystemExit(main())
