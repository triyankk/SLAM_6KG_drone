"""Conservative body-frame obstacle extraction and source fusion."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import numpy as np

from .config import (
    DepthCameraConfig,
    LidarConfig,
    ObstacleAlertConfig,
    ObstacleAvoidanceConfig,
    RotationConfig,
)


UNKNOWN_DISTANCE_CM = 65535


@dataclass(frozen=True)
class ObstacleAlertState:
    """Audible alert state derived only from a fresh CG distance."""

    zone: str
    distance_m: float | None
    beep_rate_hz: float
    avoidance_required: bool


def obstacle_alert_state(
    distance_m: float | None,
    *,
    hard_clearance_m: float,
    full_rate_distance_m: float,
    settings: ObstacleAlertConfig,
) -> ObstacleAlertState:
    """Map distance to the warning, keepout, and escalating alert bands."""

    if (
        distance_m is None
        or not math.isfinite(distance_m)
        or distance_m <= 0.0
        or distance_m > settings.warning_distance_m
    ):
        return ObstacleAlertState("clear", distance_m, 0.0, False)
    if distance_m > hard_clearance_m:
        return ObstacleAlertState(
            "warning",
            distance_m,
            settings.warning_rate_hz,
            False,
        )
    if distance_m >= settings.escalation_distance_m:
        return ObstacleAlertState(
            "keepout",
            distance_m,
            settings.keepout_rate_hz,
            True,
        )

    ramp_span_m = (
        settings.escalation_distance_m - full_rate_distance_m
    )
    ramp_fraction = (
        settings.escalation_distance_m - distance_m
    ) / ramp_span_m
    ramp_fraction = max(0.0, min(1.0, ramp_fraction))
    beep_rate_hz = settings.keepout_rate_hz + ramp_fraction * (
        settings.maximum_rate_hz - settings.keepout_rate_hz
    )
    return ObstacleAlertState(
        "escalating",
        distance_m,
        beep_rate_hz,
        True,
    )


@dataclass(frozen=True)
class ClearanceAssessment:
    """Hard horizontal clearance state referenced to the aircraft CG."""

    status: str
    required_distance_m: float
    nearest_distance_m: float | None
    margin_m: float | None
    violating_sector_indices: tuple[int, ...]
    violating_sector_angles_deg: tuple[float, ...]
    reference: str = "aircraft_cg"
    frame: str = "body_frd"
    distance_metric: str = "horizontal_xy"

    @property
    def breached(self) -> bool:
        return self.status == "breach"

    @property
    def violating_sector_count(self) -> int:
        return len(self.violating_sector_indices)

    def as_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "reference": self.reference,
            "frame": self.frame,
            "distance_metric": self.distance_metric,
            "required_distance_m": self.required_distance_m,
            "nearest_distance_m": self.nearest_distance_m,
            "margin_m": self.margin_m,
            "breached": self.breached,
            "violating_sector_count": self.violating_sector_count,
            "violating_sector_indices": list(
                self.violating_sector_indices
            ),
            "violating_sector_angles_deg": list(
                self.violating_sector_angles_deg
            ),
        }


@dataclass(frozen=True)
class ObstacleScan:
    source: str
    monotonic_ns: int
    distances_cm: tuple[int, ...]
    increment_deg: float
    min_distance_cm: int
    max_distance_cm: int

    def __post_init__(self) -> None:
        expected = round(360.0 / self.increment_deg)
        if len(self.distances_cm) != expected:
            raise ValueError("obstacle scan has the wrong sector count")
        if any(
            value != UNKNOWN_DISTANCE_CM
            and not self.min_distance_cm <= value <= self.max_distance_cm
            for value in self.distances_cm
        ):
            raise ValueError("obstacle distance is outside scan limits")

    @property
    def valid_sector_count(self) -> int:
        return sum(
            value != UNKNOWN_DISTANCE_CM for value in self.distances_cm
        )

    @property
    def nearest_distance_m(self) -> float | None:
        valid = [
            value
            for value in self.distances_cm
            if value != UNKNOWN_DISTANCE_CM
        ]
        return None if not valid else min(valid) / 100.0

    def assess_clearance(
        self, required_distance_m: float
    ) -> ClearanceAssessment:
        """Assess known sectors against a hard CG-referenced boundary."""

        if not math.isfinite(required_distance_m) or required_distance_m <= 0:
            raise ValueError("required clearance must be positive and finite")
        required_cm = math.ceil(required_distance_m * 100.0)
        valid = [
            value
            for value in self.distances_cm
            if value != UNKNOWN_DISTANCE_CM
        ]
        nearest_distance_m = None if not valid else min(valid) / 100.0
        violating_indices = tuple(
            index
            for index, value in enumerate(self.distances_cm)
            if value != UNKNOWN_DISTANCE_CM and value <= required_cm
        )
        violating_angles = []
        for index in violating_indices:
            angle = index * self.increment_deg
            if angle > 180.0:
                angle -= 360.0
            violating_angles.append(angle)

        if nearest_distance_m is None:
            status = "unknown"
            margin_m = None
        else:
            status = "breach" if violating_indices else "clear"
            margin_m = nearest_distance_m - required_distance_m
        return ClearanceAssessment(
            status=status,
            required_distance_m=required_distance_m,
            nearest_distance_m=nearest_distance_m,
            margin_m=margin_m,
            violating_sector_indices=violating_indices,
            violating_sector_angles_deg=tuple(violating_angles),
        )


def _rotation_matrix(rotation: RotationConfig) -> np.ndarray:
    roll = math.radians(rotation.roll_deg)
    pitch = math.radians(rotation.pitch_deg)
    yaw = math.radians(rotation.yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotate_x = np.array(
        ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    )
    rotate_y = np.array(
        ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    )
    rotate_z = np.array(
        ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    )
    return rotate_z @ rotate_y @ rotate_x


class PointObstacleExtractor:
    """Convert body-FRD points into a filtered horizontal proximity scan."""

    def __init__(
        self,
        settings: ObstacleAvoidanceConfig,
        *,
        source: str,
    ) -> None:
        self.settings = settings
        self.source = source
        self._history: deque[np.ndarray] = deque(
            maxlen=settings.temporal_window
        )

    def extract(
        self,
        points_body_frd_m: np.ndarray,
        *,
        monotonic_ns: int | None = None,
    ) -> ObstacleScan:
        points = np.asarray(points_body_frd_m, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("body points must have shape (N, 3)")

        finite = np.isfinite(points).all(axis=1)
        vertical = (
            (points[:, 2] >= self.settings.body_z_min_m)
            & (points[:, 2] <= self.settings.body_z_max_m)
        )
        horizontal_distance = np.hypot(points[:, 0], points[:, 1])
        self_filter_radius_m = max(
            self.settings.min_distance_m,
            self.settings.airframe_radius_m,
        )
        in_range = (
            (horizontal_distance > self_filter_radius_m)
            & (horizontal_distance <= self.settings.max_distance_m)
        )
        accepted = finite & vertical & in_range
        points = points[accepted]
        horizontal_distance = horizontal_distance[accepted]

        sector_count = self.settings.sector_count
        instant = np.full(sector_count, np.nan, dtype=np.float64)
        if len(points):
            bearing_deg = np.degrees(
                np.arctan2(points[:, 1], points[:, 0])
            )
            sector = (
                np.floor(
                    (
                        bearing_deg
                        + self.settings.sector_increment_deg / 2.0
                    )
                    / self.settings.sector_increment_deg
                ).astype(np.int32)
                % sector_count
            )
            for index in np.unique(sector):
                values = horizontal_distance[sector == index]
                if len(values) < self.settings.minimum_points_per_sector:
                    continue
                instant[index] = np.percentile(
                    values, self.settings.depth_percentile
                )

        self._history.append(instant)
        filtered = np.full(sector_count, np.nan, dtype=np.float64)
        history = np.stack(tuple(self._history))
        for index in range(sector_count):
            values = history[:, index]
            valid = values[np.isfinite(values)]
            if len(valid):
                filtered[index] = np.min(valid)

        minimum_cm = round(self.settings.min_distance_m * 100.0)
        maximum_cm = round(self.settings.max_distance_m * 100.0)
        distances_cm = tuple(
            UNKNOWN_DISTANCE_CM
            if not math.isfinite(value)
            else max(minimum_cm, min(maximum_cm, round(value * 100.0)))
            for value in filtered
        )
        return ObstacleScan(
            source=self.source,
            monotonic_ns=(
                time.monotonic_ns()
                if monotonic_ns is None
                else monotonic_ns
            ),
            distances_cm=distances_cm,
            increment_deg=self.settings.sector_increment_deg,
            min_distance_cm=minimum_cm,
            max_distance_cm=maximum_cm,
        )


class DepthObstacleExtractor:
    """Project a D415 depth frame into body FRD and sectorize it."""

    def __init__(
        self,
        settings: ObstacleAvoidanceConfig,
        camera: DepthCameraConfig,
    ) -> None:
        self.settings = settings
        self.camera = camera
        self._point_extractor = PointObstacleExtractor(
            settings, source="depth_camera"
        )
        self._rotation = _rotation_matrix(
            camera.rotation_from_forward_frd
        )
        position = camera.position_from_cg_frd_m
        self._translation = np.array(
            (position.x, position.y, position.z), dtype=np.float64
        )

    def extract(
        self,
        depth_raw: np.ndarray,
        *,
        depth_scale_m: float,
        fx: float,
        fy: float,
        ppx: float,
        ppy: float,
        monotonic_ns: int | None = None,
    ) -> ObstacleScan:
        depth = np.asarray(depth_raw)
        if depth.ndim != 2:
            raise ValueError("depth frame must have shape (height, width)")
        if depth_scale_m <= 0.0 or fx <= 0.0 or fy <= 0.0:
            raise ValueError("depth scale and focal lengths must be positive")

        stride = self.settings.depth_sample_stride
        rows = np.arange(0, depth.shape[0], stride, dtype=np.int32)
        columns = np.arange(0, depth.shape[1], stride, dtype=np.int32)
        uu, vv = np.meshgrid(columns, rows)
        forward_m = depth[vv, uu].astype(np.float64) * depth_scale_m
        valid = (
            np.isfinite(forward_m)
            & (forward_m >= self.settings.min_distance_m)
            & (forward_m <= self.settings.max_distance_m)
        )
        forward_m = forward_m[valid]
        right_m = (
            (uu[valid].astype(np.float64) - ppx) / fx * forward_m
        )
        down_m = (
            (vv[valid].astype(np.float64) - ppy) / fy * forward_m
        )
        forward_frd = np.column_stack((forward_m, right_m, down_m))
        body_frd = forward_frd @ self._rotation.T
        body_frd += self._translation
        return self._point_extractor.extract(
            body_frd, monotonic_ns=monotonic_ns
        )


class LidarObstacleExtractor:
    """Transform Hesai JT16 XYZ points into body FRD and sectorize them."""

    def __init__(
        self,
        settings: ObstacleAvoidanceConfig,
        lidar: LidarConfig,
    ) -> None:
        self._point_extractor = PointObstacleExtractor(
            settings, source="lidar"
        )
        self._rotation = _rotation_matrix(lidar.rotation_to_body_frd)
        position = lidar.position_from_cg_frd_m
        self._translation = np.array(
            (position.x, position.y, position.z), dtype=np.float64
        )

    def extract(
        self,
        points_hesai_xyz_m: np.ndarray,
        *,
        monotonic_ns: int | None = None,
    ) -> ObstacleScan:
        points = np.asarray(points_hesai_xyz_m, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Hesai points must have shape (N, 3)")

        # JT16 uses +Y at zero azimuth, +X right, and +Z up.
        lidar_forward_frd = np.column_stack(
            (points[:, 1], points[:, 0], -points[:, 2])
        )
        body_frd = lidar_forward_frd @ self._rotation.T
        body_frd += self._translation
        return self._point_extractor.extract(
            body_frd, monotonic_ns=monotonic_ns
        )


class ObstacleFusion:
    """Fuse nearest fresh sectors while dropping stale sources."""

    def __init__(self, settings: ObstacleAvoidanceConfig) -> None:
        self.settings = settings
        self._lock = threading.Lock()
        self._scans: dict[str, ObstacleScan] = {}

    def update(self, scan: ObstacleScan) -> None:
        if not math.isclose(
            scan.increment_deg,
            self.settings.sector_increment_deg,
            abs_tol=1.0e-9,
        ):
            raise ValueError("obstacle source sector increment mismatch")
        with self._lock:
            self._scans[scan.source] = scan

    def fused(self, *, monotonic_ns: int | None = None) -> ObstacleScan | None:
        now_ns = (
            time.monotonic_ns()
            if monotonic_ns is None
            else monotonic_ns
        )
        maximum_age_ns = round(
            self.settings.source_stale_timeout_s * 1.0e9
        )
        with self._lock:
            fresh = [
                scan
                for scan in self._scans.values()
                if 0 <= now_ns - scan.monotonic_ns <= maximum_age_ns
            ]
        if not fresh:
            return None

        fused = np.full(
            self.settings.sector_count,
            UNKNOWN_DISTANCE_CM,
            dtype=np.uint16,
        )
        for scan in fresh:
            values = np.asarray(scan.distances_cm, dtype=np.uint16)
            known = values != UNKNOWN_DISTANCE_CM
            replace = known & (
                (fused == UNKNOWN_DISTANCE_CM) | (values < fused)
            )
            fused[replace] = values[replace]
        return ObstacleScan(
            source="+".join(sorted(scan.source for scan in fresh)),
            monotonic_ns=now_ns,
            distances_cm=tuple(int(value) for value in fused),
            increment_deg=self.settings.sector_increment_deg,
            min_distance_cm=round(
                self.settings.min_distance_m * 100.0
            ),
            max_distance_cm=round(
                self.settings.max_distance_m * 100.0
            ),
        )
