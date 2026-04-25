import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .types import PoseSample


def wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0
    return wrapped


def rotate_xy(x_m: float, y_m: float, yaw_deg: float) -> tuple[float, float]:
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        x_m * cos_yaw - y_m * sin_yaw,
        x_m * sin_yaw + y_m * cos_yaw,
    )


def pose_yaw_deg(pose: PoseSample) -> float:
    qw, qx, qy, qz = pose.qw, pose.qx, pose.qy, pose.qz
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def quaternion_from_yaw_deg(yaw_deg: float) -> tuple[float, float, float, float]:
    half_yaw = math.radians(yaw_deg) * 0.5
    return math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)


def quaternion_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def circular_mean_deg(values_deg: list[float]) -> float:
    if not values_deg:
        return 0.0
    sin_sum = sum(math.sin(math.radians(value)) for value in values_deg)
    cos_sum = sum(math.cos(math.radians(value)) for value in values_deg)
    return wrap_angle_deg(math.degrees(math.atan2(sin_sum, cos_sum)))


def circular_std_deg(values_deg: list[float], mean_deg: float) -> float:
    if not values_deg:
        return 999.0
    deltas = [wrap_angle_deg(value - mean_deg) for value in values_deg]
    return math.sqrt(sum(delta * delta for delta in deltas) / len(deltas))


def linear_std(values: list[float]) -> float:
    if not values:
        return 999.0
    mean_value = sum(values) / len(values)
    return math.sqrt(sum((value - mean_value) ** 2 for value in values) / len(values))


@dataclass
class CalibrationProfile:
    valid: bool = False
    calibration_mode: str = "BRAKE"
    sample_count: int = 0
    yaw_offset_deg: float = 0.0
    x_offset_m: float = 0.0
    y_offset_m: float = 0.0
    yaw_std_deg: float = 999.0
    x_std_m: float = 999.0
    y_std_m: float = 999.0
    range_mean_m: float = 0.0
    saved_at_epoch_s: float = 0.0


def load_calibration_profile(path: str | Path) -> CalibrationProfile:
    profile_path = Path(path).expanduser()
    if not profile_path.exists():
        return CalibrationProfile()
    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
    except Exception:
        return CalibrationProfile()
    if not isinstance(payload, dict):
        return CalibrationProfile()
    return CalibrationProfile(
        valid=bool(payload.get("valid", False)),
        calibration_mode=str(payload.get("calibration_mode", "BRAKE")),
        sample_count=int(payload.get("sample_count", 0)),
        yaw_offset_deg=float(payload.get("yaw_offset_deg", 0.0)),
        x_offset_m=float(payload.get("x_offset_m", 0.0)),
        y_offset_m=float(payload.get("y_offset_m", 0.0)),
        yaw_std_deg=float(payload.get("yaw_std_deg", 999.0)),
        x_std_m=float(payload.get("x_std_m", 999.0)),
        y_std_m=float(payload.get("y_std_m", 999.0)),
        range_mean_m=float(payload.get("range_mean_m", 0.0)),
        saved_at_epoch_s=float(payload.get("saved_at_epoch_s", 0.0)),
    )


def save_calibration_profile(path: str | Path, profile: CalibrationProfile) -> None:
    profile_path = Path(path).expanduser()
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    profile_path.write_text(json.dumps(asdict(profile), indent=2, sort_keys=True), encoding="utf-8")


def apply_calibration_profile(pose: PoseSample, profile: CalibrationProfile) -> PoseSample:
    if not profile.valid:
        return pose
    rotated_x_m, rotated_y_m = rotate_xy(pose.x_m, pose.y_m, profile.yaw_offset_deg)
    rotated_vx_m_s, rotated_vy_m_s = rotate_xy(pose.vx_m_s, pose.vy_m_s, profile.yaw_offset_deg)
    corrected_quaternion = quaternion_multiply(
        quaternion_from_yaw_deg(profile.yaw_offset_deg),
        (pose.qw, pose.qx, pose.qy, pose.qz),
    )
    return PoseSample(
        timestamp_us=pose.timestamp_us,
        x_m=rotated_x_m + profile.x_offset_m,
        y_m=rotated_y_m + profile.y_offset_m,
        z_m=pose.z_m,
        qw=corrected_quaternion[0],
        qx=corrected_quaternion[1],
        qy=corrected_quaternion[2],
        qz=corrected_quaternion[3],
        vx_m_s=rotated_vx_m_s,
        vy_m_s=rotated_vy_m_s,
        vz_m_s=pose.vz_m_s,
        roll_rate_rad_s=pose.roll_rate_rad_s,
        pitch_rate_rad_s=pose.pitch_rate_rad_s,
        yaw_rate_rad_s=pose.yaw_rate_rad_s,
        pose_quality=pose.pose_quality,
        tracking_state=pose.tracking_state,
        feature_count=pose.feature_count,
        tracked_feature_count=pose.tracked_feature_count,
        inlier_count=pose.inlier_count,
        source_name=(f"{pose.source_name}+cal" if pose.source_name else "cal"),
    )


@dataclass
class CalibrationAccumulator:
    mode_name: str
    duration_s: float
    min_samples: int
    started_s: float = 0.0
    yaw_offsets_deg: list[float] = field(default_factory=list)
    x_offsets_m: list[float] = field(default_factory=list)
    y_offsets_m: list[float] = field(default_factory=list)
    ranges_m: list[float] = field(default_factory=list)

    def reset(self) -> None:
        self.started_s = 0.0
        self.yaw_offsets_deg.clear()
        self.x_offsets_m.clear()
        self.y_offsets_m.clear()
        self.ranges_m.clear()

    def start(self) -> None:
        self.reset()
        self.started_s = time.monotonic()

    def active(self) -> bool:
        return self.started_s > 0.0

    def elapsed_s(self) -> float:
        if not self.active():
            return 0.0
        return max(0.0, time.monotonic() - self.started_s)

    def collect(
        self,
        reference_x_m: float,
        reference_y_m: float,
        reference_yaw_deg: float,
        range_m: float,
        pose: PoseSample,
    ) -> None:
        pose_yaw = pose_yaw_deg(pose)
        yaw_offset = wrap_angle_deg(reference_yaw_deg - pose_yaw)
        rotated_x_m, rotated_y_m = rotate_xy(pose.x_m, pose.y_m, yaw_offset)
        self.yaw_offsets_deg.append(yaw_offset)
        self.x_offsets_m.append(reference_x_m - rotated_x_m)
        self.y_offsets_m.append(reference_y_m - rotated_y_m)
        if range_m > 0.0:
            self.ranges_m.append(range_m)

    def ready(self) -> bool:
        return self.active() and self.elapsed_s() >= self.duration_s and len(self.yaw_offsets_deg) >= self.min_samples

    def build_profile(self) -> CalibrationProfile:
        yaw_offset_deg = circular_mean_deg(self.yaw_offsets_deg)
        x_offset_m = sum(self.x_offsets_m) / max(len(self.x_offsets_m), 1)
        y_offset_m = sum(self.y_offsets_m) / max(len(self.y_offsets_m), 1)
        range_mean_m = sum(self.ranges_m) / max(len(self.ranges_m), 1) if self.ranges_m else 0.0
        return CalibrationProfile(
            valid=True,
            calibration_mode=self.mode_name,
            sample_count=len(self.yaw_offsets_deg),
            yaw_offset_deg=yaw_offset_deg,
            x_offset_m=x_offset_m,
            y_offset_m=y_offset_m,
            yaw_std_deg=circular_std_deg(self.yaw_offsets_deg, yaw_offset_deg),
            x_std_m=linear_std(self.x_offsets_m),
            y_std_m=linear_std(self.y_offsets_m),
            range_mean_m=range_mean_m,
            saved_at_epoch_s=time.time(),
        )
