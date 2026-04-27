from dataclasses import dataclass


@dataclass
class PoseSample:
    timestamp_us: int
    x_m: float
    y_m: float
    z_m: float
    qw: float
    qx: float
    qy: float
    qz: float
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    vz_m_s: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    pose_quality: int = 100
    tracking_state: str = "ok"
    feature_count: int = 0
    tracked_feature_count: int = 0
    inlier_count: int = 0
    source_name: str = ""


@dataclass
class ImuSample:
    timestamp_us: int
    qw: float
    qx: float
    qy: float
    qz: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    gx_deg_s: float = 0.0
    gy_deg_s: float = 0.0
    gz_deg_s: float = 0.0
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    mx_raw: int = 0
    my_raw: int = 0
    mz_raw: int = 0
    pressure_pa: float = 0.0
    altitude_m: float = 0.0
