"""Bench-check live D415 and JT16 obstacle sectors without Cube output."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from typing import Any

from .config import ConfigError, load_config
from .flight_logger import (
    DEFAULT_CONFIG,
    HesaiLidarRecorder,
    RealSensePointCloudRecorder,
)
from .obstacles import ObstacleFusion, ObstacleScan, UNKNOWN_DISTANCE_CM
from .paths import RUNTIME_DIR


class _BenchSession:
    def __init__(self, root: Path) -> None:
        self.realsense_dir = root / "realsense"
        self.lidar_dir = root / "lidar"
        self.pointcloud_dir = root / "pointcloud"
        self.pointcloud_frames_dir = self.pointcloud_dir / "frames"
        for path in (
            self.realsense_dir,
            self.lidar_dir,
            self.pointcloud_frames_dir,
        ):
            path.mkdir(parents=True, exist_ok=True)
        self.events: list[dict[str, Any]] = []
        self.stats: dict[str, dict[str, Any]] = {}
        self.sensor_timing_sample_count = 0
        self._lock = threading.Lock()

    def event(
        self, source: str, event: str, details: dict[str, Any]
    ) -> None:
        with self._lock:
            self.events.append(
                {"source": source, "event": event, "details": details}
            )

    def set_source_stats(self, source: str, **values: Any) -> None:
        with self._lock:
            self.stats.setdefault(source, {}).update(values)

    def latest_pose(self):
        return None

    def record_sensor_timing(self, _row: dict[str, Any]) -> None:
        with self._lock:
            self.sensor_timing_sample_count += 1


def summarize_scan(
    scan: ObstacleScan,
    *,
    hard_cg_clearance_m: float | None = None,
) -> dict[str, Any]:
    known = []
    for index, distance_cm in enumerate(scan.distances_cm):
        if distance_cm == UNKNOWN_DISTANCE_CM:
            continue
        angle = index * scan.increment_deg
        if angle > 180.0:
            angle -= 360.0
        known.append(
            {
                "angle_deg": angle,
                "distance_m": distance_cm / 100.0,
            }
        )
    summary = {
        "source": scan.source,
        "valid_sector_count": scan.valid_sector_count,
        "nearest_distance_m": scan.nearest_distance_m,
        "known_sectors": known,
    }
    if hard_cg_clearance_m is not None:
        summary["clearance"] = scan.assess_clearance(
            hard_cg_clearance_m
        ).as_dict()
    return summary


def evaluate_target(
    scan: ObstacleScan,
    *,
    expected_distance_m: float,
    angle_deg: float,
    tolerance_m: float,
    sector_window: int = 1,
) -> dict[str, Any]:
    """Compare nearby sectors with a measured CG-to-target distance."""

    if expected_distance_m <= 0.0 or tolerance_m <= 0.0:
        raise ValueError("target distance and tolerance must be positive")
    if not math.isfinite(angle_deg) or sector_window < 0:
        raise ValueError("target angle or sector window is invalid")

    center = (
        round(angle_deg / scan.increment_deg) % len(scan.distances_cm)
    )
    candidates = []
    for offset in range(-sector_window, sector_window + 1):
        index = (center + offset) % len(scan.distances_cm)
        distance_cm = scan.distances_cm[index]
        if distance_cm == UNKNOWN_DISTANCE_CM:
            continue
        measured_angle = index * scan.increment_deg
        if measured_angle > 180.0:
            measured_angle -= 360.0
        candidates.append(
            (distance_cm / 100.0, index, measured_angle)
        )
    if not candidates:
        return {
            "passed": False,
            "status": "no_detection",
            "reference": "aircraft_cg",
            "expected_distance_m": expected_distance_m,
            "target_angle_deg": angle_deg,
            "tolerance_m": tolerance_m,
            "measured_distance_m": None,
            "error_m": None,
            "measured_sector_angle_deg": None,
        }

    measured_distance_m, _index, measured_angle = min(candidates)
    error_m = measured_distance_m - expected_distance_m
    return {
        "passed": abs(error_m) <= tolerance_m,
        "status": (
            "within_tolerance"
            if abs(error_m) <= tolerance_m
            else "outside_tolerance"
        ),
        "reference": "aircraft_cg",
        "expected_distance_m": expected_distance_m,
        "target_angle_deg": angle_deg,
        "tolerance_m": tolerance_m,
        "measured_distance_m": measured_distance_m,
        "error_m": error_m,
        "measured_sector_angle_deg": measured_angle,
    }


def _assert_disarmed(status_path: Path) -> None:
    if not status_path.exists():
        return
    try:
        status = json.loads(status_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    if bool(status.get("vehicle", {}).get("armed")):
        raise ConfigError("obstacle bench check is forbidden while armed")
    if status.get("current_session"):
        raise ConfigError(
            "obstacle bench check is forbidden while flight recording"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--status-file",
        type=Path,
        default=RUNTIME_DIR / "flight_logger_status.json",
    )
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--no-lidar", action="store_true")
    parser.add_argument(
        "--target-distance",
        type=float,
        help="Measured horizontal distance from aircraft CG to a flat target",
    )
    parser.add_argument(
        "--target-angle",
        type=float,
        default=0.0,
        help="Target bearing in body FRD degrees; right is positive",
    )
    parser.add_argument(
        "--target-tolerance",
        type=float,
        default=0.08,
        help="Maximum absolute CG distance error in metres",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.duration <= 0.0:
            raise ConfigError("duration must be positive")
        if args.no_depth and args.no_lidar:
            raise ConfigError("at least one obstacle source is required")
        if args.target_distance is not None and (
            not math.isfinite(args.target_distance)
            or args.target_distance <= 0.0
        ):
            raise ConfigError("target distance must be positive and finite")
        if (
            not math.isfinite(args.target_angle)
            or not math.isfinite(args.target_tolerance)
            or args.target_tolerance <= 0.0
        ):
            raise ConfigError("target angle or tolerance is invalid")
        config = load_config(args.config)
        if (
            args.target_distance is not None
            and args.target_distance
            > config.obstacle_avoidance.max_distance_m
        ):
            raise ConfigError("target distance exceeds configured sensor range")
        _assert_disarmed(args.status_file)
    except (ConfigError, OSError) as exc:
        print(f"Obstacle check configuration error: {exc}")
        return 2

    stop_event = threading.Event()
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    fusion = ObstacleFusion(config.obstacle_avoidance)
    latest: dict[str, ObstacleScan] = {}
    lock = threading.Lock()

    def receive(scan: ObstacleScan) -> None:
        fusion.update(scan)
        with lock:
            latest[scan.source] = scan

    with tempfile.TemporaryDirectory(
        prefix="obstacle-check-", dir=RUNTIME_DIR
    ) as temporary:
        session = _BenchSession(Path(temporary))
        sources: list[threading.Thread] = []
        if not args.no_depth:
            sources.append(
                RealSensePointCloudRecorder(
                    session,
                    stop_event,
                    config,
                    pointcloud_rate_hz=1.0,
                    point_stride=16,
                    voxel_size_m=0.10,
                    record_bag=False,
                    obstacle_sink=receive,
                )
            )
        if not args.no_lidar:
            sources.append(
                HesaiLidarRecorder(
                    session,
                    stop_event,
                    config,
                    obstacle_sink=receive,
                )
            )
        for source in sources:
            source.start()
        stop_event.wait(args.duration)
        sample_ns = time.monotonic_ns()
        fused = fusion.fused(monotonic_ns=sample_ns)
        stop_event.set()
        for source in sources:
            source.join(timeout=10.0)

        with lock:
            source_summaries = {
                name: {
                    **summarize_scan(
                        scan,
                        hard_cg_clearance_m=(
                            config.obstacle_avoidance.hard_cg_clearance_m
                        ),
                    ),
                    "age_ms": max(
                        0, round((sample_ns - scan.monotonic_ns) / 1.0e6)
                    ),
                }
                for name, scan in latest.items()
            }
        expected = {
            name
            for name, enabled in (
                ("depth_camera", not args.no_depth),
                ("lidar", not args.no_lidar),
            )
            if enabled
        }
        maximum_age_ms = round(
            config.obstacle_avoidance.source_stale_timeout_s * 1000
        )
        observed = {
            name
            for name, summary in source_summaries.items()
            if summary["age_ms"] <= maximum_age_ms
        }
        payload = {
            "mode": "bench_shadow_only",
            "mavlink_output_sent": False,
            "duration_s": args.duration,
            "geometry_verified": {
                "airframe": (
                    config.obstacle_avoidance.airframe_geometry_verified
                ),
                "camera_extrinsics": (
                    config.calibration.camera_to_body_extrinsics_verified
                ),
                "lidar_extrinsics": (
                    config.calibration.lidar_to_body_extrinsics_verified
                ),
                "lidar_baud": config.lidar.baud_verified,
                "lidar_correction": config.lidar.correction_verified,
            },
            "sources": source_summaries,
            "fused": (
                None
                if fused is None
                else summarize_scan(
                    fused,
                    hard_cg_clearance_m=(
                        config.obstacle_avoidance.hard_cg_clearance_m
                    ),
                )
            ),
            "source_stats": session.stats,
            "sensor_timing_sample_count": (
                session.sensor_timing_sample_count
            ),
            "events": session.events,
            "missing_sources": sorted(expected - observed),
        }
        target_passed = True
        if args.target_distance is not None:
            source_checks = {
                name: evaluate_target(
                    scan,
                    expected_distance_m=args.target_distance,
                    angle_deg=args.target_angle,
                    tolerance_m=args.target_tolerance,
                )
                for name, scan in latest.items()
                if name in expected
            }
            fused_check = (
                None
                if fused is None
                else evaluate_target(
                    fused,
                    expected_distance_m=args.target_distance,
                    angle_deg=args.target_angle,
                    tolerance_m=args.target_tolerance,
                )
            )
            target_passed = (
                set(source_checks) == expected
                and all(
                    check["passed"] for check in source_checks.values()
                )
                and fused_check is not None
                and bool(fused_check["passed"])
            )
            payload["target_check"] = {
                "reference": "aircraft_cg",
                "distance_metric": "horizontal_xy",
                "sources": source_checks,
                "fused": fused_check,
                "passed": target_passed,
            }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return (
            0
            if not payload["missing_sources"] and target_passed
            else 2
        )


if __name__ == "__main__":
    raise SystemExit(main())
