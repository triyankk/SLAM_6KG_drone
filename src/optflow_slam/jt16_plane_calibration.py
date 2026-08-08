"""Capture JT16 wall/floor planes and validate mount and ring correction."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import time
from typing import Any, BinaryIO

import numpy as np

from .config import ConfigError, ProjectConfig, load_config
from .jt16_calibration import CubeCalibrationLink
from .paths import PROJECT_ROOT
from .spatial_stream import lidar_points_to_body_frd


FRAME_HEADER = struct.Struct("<8sIIQQ")
FRAME_MAGIC = b"OFJT16P1"
FRAME_VERSION = 2
MAXIMUM_POINTS = 1_000_000
POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("timestamp", "<f8"),
        ("ring", "<u2"),
        ("intensity", "u1"),
        ("confidence", "u1"),
    ],
    align=False,
)
DEFAULT_DURATION_S = 10.0
DEFAULT_TARGET_DISTANCE_M = 2.5
CALIBRATION_ROOT = PROJECT_ROOT / "data" / "calibrations" / "jt16" / "planes"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _project_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _service_active() -> bool:
    result = subprocess.run(
        ("systemctl", "--user", "is-active", "optflow-flight-logger.service"),
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def _service_action(action: str) -> None:
    result = subprocess.run(
        ("systemctl", "--user", action, "optflow-flight-logger.service"),
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"flight logger {action} failed: {detail}")


def _read_exact(
    process: subprocess.Popen[bytes],
    size: int,
    *,
    deadline_s: float,
) -> bytes:
    if process.stdout is None:
        raise RuntimeError("JT16 bridge stdout is unavailable")
    descriptor = process.stdout.fileno()
    payload = bytearray()
    while len(payload) < size:
        if process.poll() is not None:
            raise RuntimeError(
                f"JT16 bridge exited with {process.returncode}"
            )
        remaining_s = deadline_s - time.monotonic()
        if remaining_s <= 0.0:
            raise TimeoutError("JT16 bridge capture timed out")
        ready, _, _ = select.select(
            (descriptor,),
            (),
            (),
            min(0.25, remaining_s),
        )
        if not ready:
            continue
        chunk = os.read(descriptor, size - len(payload))
        if not chunk:
            raise RuntimeError("JT16 bridge closed its output")
        payload.extend(chunk)
    return bytes(payload)


def capture_points(
    config: ProjectConfig,
    *,
    duration_s: float,
    bridge_log: Path,
) -> dict[str, np.ndarray | int | float]:
    lidar = config.lidar
    bridge = _project_path(lidar.bridge_binary)
    correction = _project_path(lidar.correction_file)
    if not bridge.is_file() or not os.access(bridge, os.X_OK):
        raise OSError(f"JT16 bridge is unavailable: {bridge}")
    if not correction.is_file():
        raise OSError(f"JT16 correction is unavailable: {correction}")
    if not Path(lidar.symlink).exists():
        raise OSError(f"JT16 serial endpoint is unavailable: {lidar.symlink}")

    frames: list[np.ndarray] = []
    frame_indices: list[np.ndarray] = []
    frame_monotonic_ns: list[int] = []
    started_s = time.monotonic()
    deadline_s = started_s + duration_s + 8.0
    with bridge_log.open("wb") as errors:
        process = subprocess.Popen(
            (
                str(bridge),
                "--device",
                lidar.symlink,
                "--baud",
                str(lidar.baud),
                "--correction",
                str(correction),
                "--startup-timeout",
                "5",
            ),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=errors,
            bufsize=0,
        )
        try:
            first_frame_s: float | None = None
            while True:
                header = _read_exact(
                    process,
                    FRAME_HEADER.size,
                    deadline_s=deadline_s,
                )
                magic, version, count, monotonic_ns, frame_index = (
                    FRAME_HEADER.unpack(header)
                )
                if magic != FRAME_MAGIC or version != FRAME_VERSION:
                    raise ValueError("JT16 bridge frame header is invalid")
                if count <= 0 or count > MAXIMUM_POINTS:
                    raise ValueError("JT16 bridge point count is invalid")
                payload = _read_exact(
                    process,
                    count * POINT_DTYPE.itemsize,
                    deadline_s=deadline_s,
                )
                records = np.frombuffer(payload, dtype=POINT_DTYPE).copy()
                frames.append(records)
                frame_indices.append(
                    np.full(count, int(frame_index), dtype=np.uint64)
                )
                frame_monotonic_ns.append(int(monotonic_ns))
                now_s = time.monotonic()
                if first_frame_s is None:
                    first_frame_s = now_s
                if now_s - first_frame_s >= duration_s:
                    break
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

    records = np.concatenate(frames)
    return {
        "points_hesai_xyz_m": np.column_stack(
            (records["x"], records["y"], records["z"])
        ).astype(np.float32),
        "ring": records["ring"].astype(np.uint16),
        "intensity": records["intensity"].astype(np.uint8),
        "confidence": records["confidence"].astype(np.uint8),
        "timestamp_s": records["timestamp"].astype(np.float64),
        "frame_index": np.concatenate(frame_indices),
        "frame_monotonic_ns": np.asarray(
            frame_monotonic_ns,
            dtype=np.uint64,
        ),
        "frames": len(frames),
        "capture_duration_s": time.monotonic() - started_s,
    }


def robust_plane_fit(
    points_m: np.ndarray,
    expected_normal: np.ndarray,
) -> dict[str, Any]:
    points = np.asarray(points_m, dtype=np.float64)
    expected = np.asarray(expected_normal, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100:
        raise ValueError("plane fit requires at least 100 XYZ points")
    if not np.all(np.isfinite(points)):
        raise ValueError("plane fit contains non-finite points")
    expected /= np.linalg.norm(expected)
    projection = points @ expected
    projection_center = float(np.median(projection))
    inliers = np.abs(projection - projection_center) <= 0.20
    if np.count_nonzero(inliers) < 100:
        raise ValueError("plane fit cannot isolate the expected surface")
    normal = expected.copy()
    offset = 0.0
    for _ in range(5):
        selected = points[inliers]
        centroid = np.mean(selected, axis=0)
        _, _, right = np.linalg.svd(
            selected - centroid,
            full_matrices=False,
        )
        normal = right[-1]
        if float(np.dot(normal, expected)) < 0.0:
            normal *= -1.0
        offset = -float(np.dot(normal, centroid))
        residual = points @ normal + offset
        selected_residual = residual[inliers]
        median = float(np.median(selected_residual))
        offset -= median
        residual = points @ normal + offset
        absolute = np.abs(residual)
        median_absolute_deviation = float(
            np.median(
                np.abs(
                    selected_residual - np.median(selected_residual)
                )
            )
        )
        threshold = min(
            0.08,
            max(0.015, 3.5 * 1.4826 * median_absolute_deviation),
        )
        updated = absolute <= threshold
        if np.count_nonzero(updated) < 100:
            break
        if np.array_equal(updated, inliers):
            inliers = updated
            break
        inliers = updated
    residual = points @ normal + offset
    accepted_residual = residual[inliers]
    angle_error_deg = math.degrees(
        math.acos(float(np.clip(np.dot(normal, expected), -1.0, 1.0)))
    )
    return {
        "normal": normal,
        "offset": offset,
        "residual_m": residual,
        "inliers": inliers,
        "metrics": {
            "points": len(points),
            "inliers": int(np.count_nonzero(inliers)),
            "inlier_fraction": float(np.mean(inliers)),
            "normal_body_frd": normal.tolist(),
            "normal_error_deg": angle_error_deg,
            "plane_distance_m": abs(offset),
            "residual_rmse_m": float(
                np.sqrt(np.mean(np.square(accepted_residual)))
            ),
            "residual_p95_m": float(
                np.percentile(np.abs(accepted_residual), 95)
            ),
            "residual_max_m": float(np.max(np.abs(accepted_residual))),
        },
    }


def _ring_metrics(
    rings: np.ndarray,
    residual_m: np.ndarray,
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for ring in range(16):
        values = residual_m[rings == ring]
        if len(values) == 0:
            output.append(
                {
                    "ring": ring,
                    "points": 0,
                    "median_residual_m": None,
                    "residual_p95_m": None,
                }
            )
            continue
        output.append(
            {
                "ring": ring,
                "points": len(values),
                "median_residual_m": float(np.median(values)),
                "residual_p95_m": float(
                    np.percentile(np.abs(values), 95)
                ),
            }
        )
    return output


def analyze_planes(
    points_body_frd_m: np.ndarray,
    rings: np.ndarray,
    *,
    target_distance_m: float,
) -> dict[str, Any]:
    points = np.asarray(points_body_frd_m, dtype=np.float64)
    rings = np.asarray(rings, dtype=np.uint16)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) != len(rings):
        raise ValueError("JT16 points and rings have incompatible shapes")
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    rings = rings[finite]

    bearing_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    wall_vertical_limit_m = max(
        2.0,
        target_distance_m * math.tan(math.radians(45.0)) + 0.50,
    )
    wall_mask = (
        (np.abs(bearing_deg) <= 22.5)
        & (np.abs(points[:, 0] - target_distance_m) <= 0.40)
        & (np.abs(points[:, 2]) <= wall_vertical_limit_m)
    )
    wall_points = points[wall_mask]
    wall_rings = rings[wall_mask]
    wall_fit = robust_plane_fit(wall_points, np.asarray((1.0, 0.0, 0.0)))
    wall_ring_metrics = _ring_metrics(
        wall_rings,
        wall_fit["residual_m"],
    )
    wall_metrics = wall_fit["metrics"]
    wall_metrics["target_distance_m"] = target_distance_m
    wall_metrics["vertical_selection_limit_m"] = wall_vertical_limit_m
    wall_metrics["distance_error_m"] = abs(
        wall_metrics["plane_distance_m"] - target_distance_m
    )
    wall_metrics["rings"] = wall_ring_metrics

    horizontal_range = np.hypot(points[:, 0], points[:, 1])
    floor_search = (
        (horizontal_range >= 0.80)
        & (horizontal_range <= 8.0)
        & (points[:, 2] >= 0.15)
        & (points[:, 2] <= 2.50)
    )
    floor_metrics: dict[str, Any] | None = None
    floor_error: str | None = None
    floor_passed = False
    floor_values = points[floor_search, 2]
    if len(floor_values) < 100:
        floor_error = "not enough downward JT16 points to isolate the floor"
    else:
        try:
            histogram, edges = np.histogram(
                floor_values,
                bins=np.arange(0.15, 2.52, 0.02),
            )
            peak = int(np.argmax(histogram))
            floor_height = float((edges[peak] + edges[peak + 1]) * 0.5)
            floor_mask = floor_search & (
                np.abs(points[:, 2] - floor_height) <= 0.10
            )
            floor_points = points[floor_mask]
            floor_fit = robust_plane_fit(
                floor_points,
                np.asarray((0.0, 0.0, 1.0)),
            )
            floor_metrics = floor_fit["metrics"]
            floor_inliers = floor_points[floor_fit["inliers"]]
            floor_bearing = (
                np.degrees(
                    np.arctan2(floor_inliers[:, 1], floor_inliers[:, 0])
                )
                + 360.0
            ) % 360.0
            occupied_sectors = np.unique(
                np.floor(floor_bearing / 45.0).astype(np.int16)
            )
            floor_radius = np.hypot(
                floor_inliers[:, 0],
                floor_inliers[:, 1],
            )
            floor_metrics["histogram_peak_body_z_m"] = floor_height
            floor_metrics["azimuth_sectors_45deg"] = int(
                len(occupied_sectors)
            )
            floor_metrics["radial_span_m"] = float(
                np.percentile(floor_radius, 95)
                - np.percentile(floor_radius, 5)
            )
            floor_passed = bool(
                floor_metrics["inliers"] >= 1_000
                and floor_metrics["azimuth_sectors_45deg"] >= 6
                and floor_metrics["radial_span_m"] >= 1.0
                and floor_metrics["normal_error_deg"] <= 2.0
                and floor_metrics["residual_p95_m"] <= 0.05
            )
            if not floor_passed:
                floor_error = (
                    "horizontal surface did not satisfy floor coverage and "
                    "planarity gates"
                )
        except (ValueError, np.linalg.LinAlgError) as exc:
            floor_error = str(exc)

    ring_coverage = all(item["points"] >= 100 for item in wall_ring_metrics)
    ring_medians = [
        abs(float(item["median_residual_m"]))
        for item in wall_ring_metrics
        if item["median_residual_m"] is not None
    ]
    correction_passed = bool(
        ring_coverage
        and wall_metrics["residual_p95_m"] <= 0.05
        and max(ring_medians, default=math.inf) <= 0.04
        and wall_metrics["distance_error_m"] <= 0.10
    )
    extrinsic_passed = bool(
        wall_metrics["normal_error_deg"] <= 2.0
        and floor_passed
    )
    return {
        "result": "pass" if correction_passed and extrinsic_passed else "fail",
        "wall": wall_metrics,
        "floor": floor_metrics,
        "floor_error": floor_error,
        "gates": {
            "correction_file_validated_for_unit": correction_passed,
            "lidar_to_body_rotation_verified": extrinsic_passed,
            "all_16_rings_covered": ring_coverage,
        },
        "recommended_config": {
            "correction_verified": correction_passed,
            "lidar_to_body_extrinsics_verified": extrinsic_passed,
            "apply_automatically": False,
        },
    }


def run_calibration(
    config: ProjectConfig,
    config_path: Path,
    *,
    duration_s: float,
    target_distance_m: float,
    output_root: Path,
) -> tuple[Path, dict[str, Any], str]:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session = output_root / f"{stamp}-wall-floor"
    session.mkdir(parents=True, exist_ok=False)
    capture = capture_points(
        config,
        duration_s=duration_s,
        bridge_log=session / "bridge.log",
    )
    raw_path = session / "points.npz"
    np.savez_compressed(
        raw_path,
        points_hesai_xyz_m=capture["points_hesai_xyz_m"],
        ring=capture["ring"],
        intensity=capture["intensity"],
        confidence=capture["confidence"],
        timestamp_s=capture["timestamp_s"],
        frame_index=capture["frame_index"],
        frame_monotonic_ns=capture["frame_monotonic_ns"],
    )
    body_points = lidar_points_to_body_frd(
        capture["points_hesai_xyz_m"],
        config,
    )
    analysis = analyze_planes(
        body_points,
        capture["ring"],
        target_distance_m=target_distance_m,
    )
    correction_path = _project_path(config.lidar.correction_file)
    report = {
        "schema_version": 1,
        "kind": "jt16_wall_floor_calibration",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config_source": str(config_path.resolve()),
        "config_sha256": _sha256(config_path.resolve()),
        "correction_file": str(correction_path),
        "correction_sha256": _sha256(correction_path),
        "raw_samples": raw_path.name,
        "raw_samples_sha256": _sha256(raw_path),
        "frames": int(capture["frames"]),
        "points": len(capture["ring"]),
        "capture_duration_s": float(capture["capture_duration_s"]),
        **analysis,
    }
    report_path = session / "report.json"
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("ascii")
    report_path.write_bytes(report_bytes)
    digest = hashlib.sha256(report_bytes).hexdigest()
    (session / "report.sha256").write_text(
        f"{digest}  report.json\n",
        encoding="ascii",
    )
    return report_path, report, digest


def reanalyze_calibration(
    config: ProjectConfig,
    config_path: Path,
    raw_path: Path,
    *,
    target_distance_m: float,
) -> tuple[Path, dict[str, Any], str]:
    raw_path = raw_path.resolve()
    if not raw_path.is_file():
        raise OSError(f"JT16 raw capture is unavailable: {raw_path}")
    with np.load(raw_path, allow_pickle=False) as capture:
        points_hesai = np.asarray(
            capture["points_hesai_xyz_m"],
            dtype=np.float32,
        )
        rings = np.asarray(capture["ring"], dtype=np.uint16)
        frame_indices = np.asarray(capture["frame_index"], dtype=np.uint64)
    body_points = lidar_points_to_body_frd(points_hesai, config)
    analysis = analyze_planes(
        body_points,
        rings,
        target_distance_m=target_distance_m,
    )
    correction_path = _project_path(config.lidar.correction_file)
    report = {
        "schema_version": 1,
        "kind": "jt16_wall_floor_reanalysis",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config_source": str(config_path.resolve()),
        "config_sha256": _sha256(config_path.resolve()),
        "correction_file": str(correction_path),
        "correction_sha256": _sha256(correction_path),
        "raw_samples": str(raw_path),
        "raw_samples_sha256": _sha256(raw_path),
        "frames": int(len(np.unique(frame_indices))),
        "points": len(rings),
        **analysis,
    }
    report_path = raw_path.parent / "reanalysis.json"
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("ascii")
    report_path.write_bytes(report_bytes)
    digest = hashlib.sha256(report_bytes).hexdigest()
    (raw_path.parent / "reanalysis.sha256").write_text(
        f"{digest}  {report_path.name}\n",
        encoding="ascii",
    )
    return report_path, report, digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument(
        "--distance",
        type=float,
        default=DEFAULT_TARGET_DISTANCE_M,
        help="JT16-center to forward-wall distance in metres",
    )
    parser.add_argument("--output-root", type=Path, default=CALIBRATION_ROOT)
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Reanalyze an existing points.npz without accessing hardware",
    )
    parser.add_argument("--no-beep", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    service_was_active = False
    link: CubeCalibrationLink | None = None
    try:
        if args.duration < 5.0:
            raise ConfigError("JT16 plane capture must be at least 5 seconds")
        if not 1.0 <= args.distance <= 8.0:
            raise ConfigError("wall distance must be from 1.0 to 8.0 metres")
        config_path = args.config.resolve()
        config = load_config(config_path)
        if args.input is not None:
            report_path, report, digest = reanalyze_calibration(
                config,
                config_path,
                args.input,
                target_distance_m=args.distance,
            )
            print(
                json.dumps(
                    {
                        "result": report["result"],
                        "report": str(report_path),
                        "sha256": digest,
                        "gates": report["gates"],
                        "hardware_accessed": False,
                    },
                    indent=2,
                    sort_keys=True,
                )
            )
            return 0 if report["result"] == "pass" else 1
        service_was_active = _service_active()
        if service_was_active:
            _service_action("stop")
        link = CubeCalibrationLink(config.flight_controller)
        link.open()
        link.ensure_disarmed()
        print(
            "JT16_PLANE_CAPTURE_STARTED "
            f"duration_s={args.duration:.0f} distance_m={args.distance:.2f}",
            flush=True,
        )
        report_path, report, digest = run_calibration(
            config,
            config_path,
            duration_s=args.duration,
            target_distance_m=args.distance,
            output_root=args.output_root.resolve(),
        )
        if not args.no_beep:
            link.beep()
        print(
            json.dumps(
                {
                    "result": report["result"],
                    "report": str(report_path),
                    "sha256": digest,
                    "gates": report["gates"],
                    "intervention": "capture complete; the drone may be touched",
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if report["result"] == "pass" else 1
    except KeyboardInterrupt:
        if link is not None and not args.no_beep:
            link.beep()
        print("JT16 plane calibration interrupted")
        return 130
    except (
        ConfigError,
        OSError,
        RuntimeError,
        TimeoutError,
        TypeError,
        ValueError,
    ) as exc:
        if link is not None and not args.no_beep:
            link.beep()
        print(f"JT16 plane calibration error: {exc}")
        return 2
    finally:
        if link is not None:
            link.close()
        if service_was_active:
            try:
                _service_action("start")
            except RuntimeError as exc:
                print(f"Flight logger restore error: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
