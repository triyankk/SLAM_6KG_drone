"""Shadow-only local-return supervisor and recorded-session replay."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .config import ProjectConfig, load_config
from .paths import PROJECT_ROOT


@dataclass(frozen=True)
class ReturnSettings:
    maximum_horizontal_speed_mps: float = 0.50
    maximum_horizontal_acceleration_mpss: float = 1.00
    position_gain_s: float = 1.00
    arrival_radius_m: float = 0.12
    breadcrumb_spacing_m: float = 0.10
    waypoint_radius_m: float = 0.08
    visual_stale_timeout_s: float = 0.75
    visual_disagreement_limit_m: float = 0.35
    command_timeout_s: float = 0.50
    maximum_rows: int = 12_000


def settings_from_config(config: ProjectConfig) -> ReturnSettings:
    return ReturnSettings(
        maximum_horizontal_speed_mps=(
            config.navigation.initial_max_horizontal_speed_mps
        ),
        maximum_horizontal_acceleration_mpss=(
            config.obstacle_avoidance.native.acceleration_max_mpss
        ),
        visual_stale_timeout_s=max(
            0.75,
            3.0 / max(1.0, config.depth_camera.fps),
        ),
        command_timeout_s=config.navigation.command_stale_timeout_s,
    )


def add_control_approval_gates(
    report: dict[str, Any],
    config: ProjectConfig,
) -> dict[str, Any]:
    """Attach the gates required before shadow proposals may reach Cube."""
    gates = {
        "shadow_kinematics": report.get("result") == "shadow_pass",
        "lio_validation_approved": (
            config.lidar_inertial_odometry.validation.approved
        ),
        "camera_extrinsics_verified": (
            config.calibration.camera_to_body_extrinsics_verified
        ),
        "lidar_extrinsics_verified": (
            config.calibration.lidar_to_body_extrinsics_verified
        ),
        "imu_noise_profile_verified": (
            config.calibration.imu_noise_profile_verified
        ),
        "sensor_time_sync_verified": (
            config.calibration.sensor_time_sync_verified
        ),
        "pose_covariance_and_reset_contract": False,
        "loop_closure_or_relocalization": False,
        "collision_free_path_verified": False,
        "cube_position_control_proven_for_return": False,
        "pilot_authority_and_failsafe_proven": False,
    }
    report["control_approval_gates"] = gates
    report["control_eligible"] = all(gates.values())
    report["remaining_control_blockers"] = [
        name for name, passed in gates.items() if not passed
    ]
    return report


def _point(value: Any) -> np.ndarray | None:
    try:
        point = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    return point


def _horizontal_distance(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.linalg.norm((first - second)[:2]))


def _percentile(values: Iterable[float], percentile: float) -> float | None:
    finite = np.asarray(
        [value for value in values if math.isfinite(value)],
        dtype=np.float64,
    )
    if not len(finite):
        return None
    return float(np.percentile(finite, percentile))


class LocalReturnShadow:
    """Generate local-return velocity proposals without any transport output."""

    def __init__(self, settings: ReturnSettings | None = None) -> None:
        self.settings = settings or ReturnSettings()
        self.state = "waiting_for_launch"
        self.launch_lio: np.ndarray | None = None
        self.launch_visual: np.ndarray | None = None
        self.breadcrumbs: list[np.ndarray] = []
        self.target_index: int | None = None
        self.current_lio: np.ndarray | None = None
        self.latest_visual: np.ndarray | None = None
        self.latest_visual_timestamp_ns: int | None = None
        self.latest_visual_tracking = False
        self.maximum_excursion_m = 0.0
        self.initial_return_distance_m: float | None = None
        self._previous_timestamp_ns: int | None = None
        self._previous_velocity = np.zeros(3, dtype=np.float64)
        self._rows: deque[dict[str, Any]] = deque(
            maxlen=self.settings.maximum_rows
        )

    def capture_launch(
        self,
        timestamp_ns: int,
        lio_position: Any,
        visual_position: Any | None = None,
    ) -> bool:
        lio = _point(lio_position)
        visual = _point(visual_position)
        if lio is None:
            return False
        self.launch_lio = lio.copy()
        self.launch_visual = None if visual is None else visual.copy()
        self.current_lio = lio.copy()
        self.breadcrumbs = [lio.copy()]
        self.target_index = None
        self.maximum_excursion_m = 0.0
        self.initial_return_distance_m = None
        self._previous_timestamp_ns = int(timestamp_ns)
        self._previous_velocity[:] = 0.0
        self._rows.clear()
        self.state = "recording_outbound"
        return True

    def update_visual(
        self,
        timestamp_ns: int,
        position: Any,
        *,
        tracking: bool,
    ) -> None:
        point = _point(position)
        if point is None:
            return
        self.latest_visual = point.copy()
        self.latest_visual_timestamp_ns = int(timestamp_ns)
        self.latest_visual_tracking = bool(tracking)

    def observe_outbound(self, timestamp_ns: int, lio_position: Any) -> None:
        if self.state != "recording_outbound" or self.launch_lio is None:
            return
        point = _point(lio_position)
        if point is None:
            return
        self.current_lio = point.copy()
        self.maximum_excursion_m = max(
            self.maximum_excursion_m,
            _horizontal_distance(point, self.launch_lio),
        )
        if (
            not self.breadcrumbs
            or _horizontal_distance(point, self.breadcrumbs[-1])
            >= self.settings.breadcrumb_spacing_m
        ):
            self.breadcrumbs.append(point.copy())
        self._previous_timestamp_ns = int(timestamp_ns)

    def begin_return(self, timestamp_ns: int) -> bool:
        if (
            self.state != "recording_outbound"
            or self.launch_lio is None
            or self.current_lio is None
        ):
            return False
        if _horizontal_distance(self.current_lio, self.breadcrumbs[-1]) >= (
            0.5 * self.settings.breadcrumb_spacing_m
        ):
            self.breadcrumbs.append(self.current_lio.copy())
        self.target_index = max(0, len(self.breadcrumbs) - 2)
        self.initial_return_distance_m = _horizontal_distance(
            self.current_lio, self.launch_lio
        )
        self._previous_timestamp_ns = int(timestamp_ns)
        self._previous_velocity[:] = 0.0
        self.state = "returning"
        return True

    def observe_return(
        self,
        timestamp_ns: int,
        lio_position: Any,
    ) -> dict[str, Any] | None:
        if self.state not in {"returning", "arrived"}:
            return None
        point = _point(lio_position)
        if point is None or self.launch_lio is None:
            return self._append_blocked(timestamp_ns, "invalid_lio_pose")
        timestamp_ns = int(timestamp_ns)
        if (
            self._previous_timestamp_ns is not None
            and timestamp_ns <= self._previous_timestamp_ns
        ):
            return self._append_blocked(
                timestamp_ns, "non_monotonic_lio_timestamp"
            )
        previous_timestamp_ns = self._previous_timestamp_ns
        self._previous_timestamp_ns = timestamp_ns
        self.current_lio = point.copy()
        home_distance_m = _horizontal_distance(point, self.launch_lio)
        visual_gap_m, visual_fresh = self._visual_consistency(timestamp_ns)
        blocked_reason = None
        if (
            visual_gap_m is not None
            and visual_fresh
            and visual_gap_m > self.settings.visual_disagreement_limit_m
        ):
            blocked_reason = "visual_lio_disagreement"

        if blocked_reason is None and (
            home_distance_m <= self.settings.arrival_radius_m
            or self.state == "arrived"
        ):
            self.state = "arrived"
            velocity = np.zeros(3, dtype=np.float64)
            target = self.launch_lio
        elif blocked_reason is not None:
            velocity = np.zeros(3, dtype=np.float64)
            target = self._target()
        else:
            target = self._advance_target(point)
            velocity = self._bounded_velocity(
                point,
                target,
                home_distance_m,
                timestamp_ns,
                previous_timestamp_ns,
            )

        quality = self._quality(visual_gap_m, visual_fresh)
        row = self._row(
            timestamp_ns,
            point,
            target,
            velocity,
            home_distance_m,
            visual_gap_m,
            visual_fresh,
            quality,
            blocked_reason,
        )
        self._rows.append(row)
        self._previous_velocity = velocity
        return row

    def _visual_consistency(
        self, timestamp_ns: int
    ) -> tuple[float | None, bool]:
        if (
            self.latest_visual is None
            or self.launch_visual is None
            or self.latest_visual_timestamp_ns is None
        ):
            return None, False
        age_s = abs(timestamp_ns - self.latest_visual_timestamp_ns) / 1.0e9
        fresh = bool(
            self.latest_visual_tracking
            and age_s <= self.settings.visual_stale_timeout_s
        )
        lio_delta = self.current_lio - self.launch_lio
        visual_delta = self.latest_visual - self.launch_visual
        gap_m = float(np.linalg.norm((lio_delta - visual_delta)[:2]))
        return gap_m, fresh

    def _advance_target(self, point: np.ndarray) -> np.ndarray:
        if self.target_index is None:
            self.target_index = max(0, len(self.breadcrumbs) - 2)
        while self.target_index > 0 and _horizontal_distance(
            point, self.breadcrumbs[self.target_index]
        ) <= self.settings.waypoint_radius_m:
            self.target_index -= 1
        return self._target()

    def _target(self) -> np.ndarray:
        if self.launch_lio is None:
            return np.zeros(3, dtype=np.float64)
        if self.target_index is None or not self.breadcrumbs:
            return self.launch_lio
        return self.breadcrumbs[self.target_index]

    def _bounded_velocity(
        self,
        point: np.ndarray,
        target: np.ndarray,
        home_distance_m: float,
        timestamp_ns: int,
        previous_timestamp_ns: int | None,
    ) -> np.ndarray:
        delta = target[:2] - point[:2]
        distance_m = float(np.linalg.norm(delta))
        if distance_m <= 1.0e-9:
            desired = np.zeros(3, dtype=np.float64)
        else:
            stopping_distance_m = max(
                0.0, home_distance_m - self.settings.arrival_radius_m
            )
            stopping_speed = math.sqrt(
                2.0
                * self.settings.maximum_horizontal_acceleration_mpss
                * stopping_distance_m
            )
            speed_mps = min(
                self.settings.maximum_horizontal_speed_mps,
                self.settings.position_gain_s * distance_m,
                stopping_speed,
            )
            desired = np.array(
                [
                    speed_mps * delta[0] / distance_m,
                    speed_mps * delta[1] / distance_m,
                    0.0,
                ],
                dtype=np.float64,
            )
        if previous_timestamp_ns is None:
            return desired
        dt_s = max(0.0, (timestamp_ns - previous_timestamp_ns) / 1.0e9)
        maximum_delta = (
            self.settings.maximum_horizontal_acceleration_mpss * dt_s
        )
        velocity_delta = desired - self._previous_velocity
        delta_speed = float(np.linalg.norm(velocity_delta[:2]))
        if delta_speed > maximum_delta > 0.0:
            desired = self._previous_velocity + (
                velocity_delta * maximum_delta / delta_speed
            )
        return desired

    def _quality(self, visual_gap_m: float | None, visual_fresh: bool) -> int:
        quality = 100.0
        if not visual_fresh or visual_gap_m is None:
            quality -= 25.0
        else:
            quality -= min(
                60.0,
                50.0
                * visual_gap_m
                / self.settings.visual_disagreement_limit_m,
            )
        return max(1, min(100, round(quality)))

    def _append_blocked(
        self, timestamp_ns: int, reason: str
    ) -> dict[str, Any]:
        point = (
            np.zeros(3, dtype=np.float64)
            if self.current_lio is None
            else self.current_lio
        )
        launch = point if self.launch_lio is None else self.launch_lio
        row = self._row(
            int(timestamp_ns),
            point,
            self._target(),
            np.zeros(3, dtype=np.float64),
            _horizontal_distance(point, launch),
            None,
            False,
            1,
            reason,
        )
        self._rows.append(row)
        self._previous_velocity[:] = 0.0
        return row

    def _row(
        self,
        timestamp_ns: int,
        point: np.ndarray,
        target: np.ndarray,
        velocity: np.ndarray,
        home_distance_m: float,
        visual_gap_m: float | None,
        visual_fresh: bool,
        quality: int,
        blocked_reason: str | None,
    ) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "timestamp_ns": timestamp_ns,
            "state": self.state,
            "shadow_only": True,
            "pose_sent_to_cube": False,
            "velocity_sent_to_cube": False,
            "position_local_flu_m": point.tolist(),
            "launch_local_flu_m": (
                None if self.launch_lio is None else self.launch_lio.tolist()
            ),
            "target_local_flu_m": target.tolist(),
            "proposed_velocity_local_flu_mps": velocity.tolist(),
            "proposed_speed_mps": float(np.linalg.norm(velocity[:2])),
            "home_distance_m": home_distance_m,
            "visual_lio_disagreement_m": visual_gap_m,
            "visual_fresh": visual_fresh,
            "pose_quality": quality,
            "blocked_reason": blocked_reason,
            "target_breadcrumb_index": self.target_index,
            "valid_until_ns": timestamp_ns
            + round(self.settings.command_timeout_s * 1.0e9),
        }

    def finish(self) -> None:
        if self.state == "returning" and self.current_lio is not None:
            if (
                self.launch_lio is not None
                and _horizontal_distance(self.current_lio, self.launch_lio)
                <= self.settings.arrival_radius_m
            ):
                self.state = "arrived"
            else:
                self.state = "incomplete"

    def rows(self) -> list[dict[str, Any]]:
        return [dict(row) for row in self._rows]

    def snapshot(self) -> dict[str, Any]:
        latest = self._rows[-1] if self._rows else None
        return {
            "state": self.state,
            "shadow_only": True,
            "pose_sent_to_cube": False,
            "velocity_sent_to_cube": False,
            "launch_captured": self.launch_lio is not None,
            "breadcrumbs": len(self.breadcrumbs),
            "maximum_excursion_m": self.maximum_excursion_m,
            "latest": None if latest is None else dict(latest),
        }

    def report(self) -> dict[str, Any]:
        rows = self.rows()
        valid_rows = [row for row in rows if row["blocked_reason"] is None]
        moving_rows = [row for row in valid_rows if row["proposed_speed_mps"] > 0.0]
        timestamps = [int(row["timestamp_ns"]) for row in rows]
        duration_s = (
            (timestamps[-1] - timestamps[0]) / 1.0e9
            if len(timestamps) >= 2
            else 0.0
        )
        command_rate_hz = (
            (len(timestamps) - 1) / duration_s if duration_s > 0.0 else 0.0
        )
        speeds = [float(row["proposed_speed_mps"]) for row in rows]
        accelerations = []
        motion_alignment_cosines = []
        for previous, current in zip(rows, rows[1:]):
            if current["blocked_reason"] is not None:
                continue
            dt_s = (current["timestamp_ns"] - previous["timestamp_ns"]) / 1.0e9
            if dt_s <= 0.0:
                continue
            previous_velocity = np.asarray(
                previous["proposed_velocity_local_flu_mps"],
                dtype=np.float64,
            )
            current_velocity = np.asarray(
                current["proposed_velocity_local_flu_mps"],
                dtype=np.float64,
            )
            accelerations.append(
                float(np.linalg.norm((current_velocity - previous_velocity)[:2]))
                / dt_s
            )
            previous_speed = float(np.linalg.norm(previous_velocity[:2]))
            previous_position = np.asarray(
                previous["position_local_flu_m"], dtype=np.float64
            )
            current_position = np.asarray(
                current["position_local_flu_m"], dtype=np.float64
            )
            observed_delta = current_position[:2] - previous_position[:2]
            observed_distance = float(np.linalg.norm(observed_delta))
            if previous_speed > 0.01 and observed_distance > 0.005:
                motion_alignment_cosines.append(
                    float(
                        np.dot(previous_velocity[:2], observed_delta)
                        / (previous_speed * observed_distance)
                    )
                )
        final_home_distance_m = (
            float(rows[-1]["home_distance_m"]) if rows else None
        )
        visual_values = [
            float(row["visual_lio_disagreement_m"])
            for row in rows
            if row["visual_lio_disagreement_m"] is not None
            and row["visual_fresh"]
        ]
        visual_p95_m = _percentile(visual_values, 95.0)
        motion_alignment_median = _percentile(
            motion_alignment_cosines, 50.0
        )
        distance_reduction_m = (
            None
            if self.initial_return_distance_m is None
            or final_home_distance_m is None
            else self.initial_return_distance_m - final_home_distance_m
        )
        gates = {
            "launch_captured": self.launch_lio is not None,
            "horizontal_excursion": self.maximum_excursion_m >= 0.20,
            "breadcrumbs_recorded": len(self.breadcrumbs) >= 2,
            "return_commands_generated": len(moving_rows) >= 2,
            "command_rate": command_rate_hz >= 4.0,
            "speed_limit": max(speeds, default=0.0)
            <= self.settings.maximum_horizontal_speed_mps + 1.0e-6,
            "acceleration_limit": max(accelerations, default=0.0)
            <= self.settings.maximum_horizontal_acceleration_mpss + 1.0e-6,
            "visual_consistency": visual_p95_m is not None
            and visual_p95_m <= self.settings.visual_disagreement_limit_m,
            "observed_motion_alignment": (
                len(motion_alignment_cosines) >= 3
                and motion_alignment_median is not None
                and motion_alignment_median >= 0.50
            ),
            "home_distance_reduced": distance_reduction_m is not None
            and distance_reduction_m >= 0.15,
            "arrived": self.state == "arrived"
            and final_home_distance_m is not None
            and final_home_distance_m <= self.settings.arrival_radius_m,
        }
        shadow_pass = all(gates.values())
        return {
            "schema_version": 1,
            "kind": "gps_denied_local_return_shadow",
            "result": "shadow_pass" if shadow_pass else "shadow_fail",
            "state": self.state,
            "shadow_only": True,
            "control_eligible": False,
            "pose_sent_to_cube": False,
            "velocity_sent_to_cube": False,
            "settings": asdict(self.settings),
            "gates": gates,
            "metrics": {
                "breadcrumbs": len(self.breadcrumbs),
                "command_rows": len(rows),
                "moving_command_rows": len(moving_rows),
                "blocked_rows": len(rows) - len(valid_rows),
                "command_rate_hz": command_rate_hz,
                "maximum_excursion_m": self.maximum_excursion_m,
                "initial_return_distance_m": self.initial_return_distance_m,
                "final_home_distance_m": final_home_distance_m,
                "maximum_proposed_speed_mps": max(speeds, default=0.0),
                "maximum_proposed_acceleration_mpss": max(
                    accelerations, default=0.0
                ),
                "motion_alignment_samples": len(
                    motion_alignment_cosines
                ),
                "motion_alignment_cosine_median": (
                    motion_alignment_median
                ),
                "home_distance_reduction_m": distance_reduction_m,
                "visual_lio_disagreement_p95_m": visual_p95_m,
                "visual_lio_disagreement_maximum_m": max(
                    visual_values, default=None
                ),
            },
            "remaining_control_blockers": [
                "pose covariance and reset contract not yet implemented",
                "collision-free path not yet verified against the live map",
                "Cube velocity bridge remains disabled",
                "flight failsafe and authority handoff remain unproven",
            ],
        }


def _read_ndjson(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"invalid JSON in {path}:{line_number}: {exc}"
                ) from exc
            if isinstance(row, dict):
                rows.append(row)
    return rows


def _nearest_visual(
    rows: list[dict[str, Any]],
    timestamps: list[int],
    timestamp_ns: int,
) -> dict[str, Any] | None:
    if not timestamps:
        return None
    index = bisect_left(timestamps, timestamp_ns)
    candidates = [
        candidate
        for candidate in (index - 1, index)
        if 0 <= candidate < len(timestamps)
    ]
    nearest = min(
        candidates,
        key=lambda candidate: abs(timestamps[candidate] - timestamp_ns),
    )
    return rows[nearest]


def replay_session(
    session: Path,
    config: ProjectConfig,
) -> tuple[Path, dict[str, Any], str]:
    session = session.resolve()
    lio_rows = _read_ndjson(session / "lio_odometry.ndjson")
    visual_rows = _read_ndjson(session / "rgbd_odometry.ndjson")
    if len(lio_rows) < 10 or len(visual_rows) < 10:
        raise ValueError("session does not contain enough LIO and RGB-D poses")

    raw_origin = _point(lio_rows[0].get("position_m"))
    if raw_origin is None:
        raise ValueError("first LIO position is invalid")
    lio_samples: list[tuple[int, np.ndarray]] = []
    for row in lio_rows:
        raw = _point(row.get("position_m"))
        timestamp_ns = row.get("host_monotonic_ns")
        if raw is None or not isinstance(timestamp_ns, int):
            continue
        relative = raw - raw_origin
        lio_samples.append(
            (
                timestamp_ns,
                np.array(
                    [relative[0], -relative[1], -relative[2]],
                    dtype=np.float64,
                ),
            )
        )
    visual_samples = [
        row
        for row in visual_rows
        if isinstance(row.get("host_monotonic_ns"), int)
        and _point(row.get("position_local_flu_m")) is not None
    ]
    visual_timestamps = [
        int(row["host_monotonic_ns"]) for row in visual_samples
    ]
    start_ns = lio_samples[0][0]
    launch_end_ns = start_ns + 5_000_000_000
    launch_lio_points = [
        point for timestamp_ns, point in lio_samples if timestamp_ns <= launch_end_ns
    ]
    launch_visual_points = [
        _point(row["position_local_flu_m"])
        for row in visual_samples
        if int(row["host_monotonic_ns"]) <= launch_end_ns
    ]
    launch_visual_points = [
        point for point in launch_visual_points if point is not None
    ]
    launch_lio = np.median(np.stack(launch_lio_points), axis=0)
    launch_visual = np.median(np.stack(launch_visual_points), axis=0)
    horizontal_distances = [
        _horizontal_distance(point, launch_lio) for _, point in lio_samples
    ]
    turn_index = int(np.argmax(horizontal_distances))
    if turn_index < 2 or turn_index >= len(lio_samples) - 2:
        raise ValueError("could not infer an outbound and return trajectory")

    supervisor = LocalReturnShadow(settings_from_config(config))
    launch_timestamp_ns = lio_samples[0][0]
    supervisor.capture_launch(
        launch_timestamp_ns,
        launch_lio,
        launch_visual,
    )
    for timestamp_ns, point in lio_samples[: turn_index + 1]:
        visual = _nearest_visual(
            visual_samples, visual_timestamps, timestamp_ns
        )
        if visual is not None:
            supervisor.update_visual(
                int(visual["host_monotonic_ns"]),
                visual["position_local_flu_m"],
                tracking=bool(visual.get("tracking")),
            )
        supervisor.observe_outbound(timestamp_ns, point)
    supervisor.begin_return(lio_samples[turn_index][0])
    for timestamp_ns, point in lio_samples[turn_index + 1 :]:
        visual = _nearest_visual(
            visual_samples, visual_timestamps, timestamp_ns
        )
        if visual is not None:
            supervisor.update_visual(
                int(visual["host_monotonic_ns"]),
                visual["position_local_flu_m"],
                tracking=bool(visual.get("tracking")),
            )
        supervisor.observe_return(timestamp_ns, point)
    supervisor.finish()
    report = supervisor.report()
    add_control_approval_gates(report, config)
    report["session"] = str(session)
    report["generated_utc"] = datetime.now(timezone.utc).isoformat()
    report["replay_inference"] = {
        "launch_reference": "median first 5 seconds",
        "return_start": "maximum horizontal LIO displacement",
        "return_start_index": turn_index,
        "return_start_timestamp_ns": lio_samples[turn_index][0],
        "not_ground_truth": True,
    }
    analysis_dir = session / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    commands_path = analysis_dir / "rtl_shadow_commands.ndjson"
    commands_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True) + "\n"
            for row in supervisor.rows()
        ),
        encoding="utf-8",
    )
    report["artifacts"] = {
        "commands": str(commands_path),
        "command_rows": len(supervisor.rows()),
    }
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    report_path = analysis_dir / "rtl_shadow_replay.json"
    report_path.write_bytes(report_bytes)
    digest = hashlib.sha256(report_bytes).hexdigest()
    (analysis_dir / "rtl_shadow_replay.sha256").write_text(
        f"{digest}  {report_path.name}\n",
        encoding="utf-8",
    )
    return report_path, report, digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
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
        report_path, report, digest = replay_session(args.session, config)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"RTL shadow replay error: {exc}")
        return 2
    print(
        json.dumps(
            {
                "result": report["result"],
                "control_eligible": report["control_eligible"],
                "report": str(report_path),
                "sha256": digest,
                "metrics": report["metrics"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if report["result"] == "shadow_pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
