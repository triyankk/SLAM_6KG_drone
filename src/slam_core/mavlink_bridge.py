import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from .types import PoseSample


@dataclass
class CubeConnection:
    master: object
    port: str
    baud: int


def discover_cube_ports() -> list[str]:
    candidates: list[str] = []
    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.exists():
        for path in sorted(by_id_dir.iterdir()):
            name = path.name.lower()
            if "cube" in name or "cubepilot" in name or "cubeorange" in name:
                candidates.append(str(path))

    candidates.extend(str(path) for path in sorted(Path("/dev").glob("ttyACM*")))

    deduped: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        deduped.append(candidate)
    return deduped


def expand_cube_ports(ports: Iterable[str]) -> list[str]:
    configured = [str(port) for port in ports]
    discovered = discover_cube_ports()
    expanded: list[str] = []
    seen: set[str] = set()
    for candidate in [*configured, *discovered]:
        if not candidate or candidate.lower() == "auto" or candidate in seen:
            continue
        seen.add(candidate)
        expanded.append(candidate)
    return expanded


def is_autopilot_heartbeat(msg) -> bool:
    autopilot = int(getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    mav_type = int(getattr(msg, "type", 0))
    if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if mav_type in {
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    }:
        return False
    return True


def wait_autopilot_heartbeat(master, timeout_s: float):
    deadline_s = time.time() + max(timeout_s, 0.1)
    while time.time() < deadline_s:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None or not is_autopilot_heartbeat(msg):
            continue
        master.target_system = int(msg.get_srcSystem())
        master.target_component = int(msg.get_srcComponent())
        return msg
    raise TimeoutError("No autopilot heartbeat received")


def quaternion_to_euler(sample: PoseSample) -> tuple[float, float, float]:
    qw, qx, qy, qz = sample.qw, sample.qx, sample.qy, sample.qz

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def connect_to_cube(ports: Iterable[str], baud: int, heartbeat_timeout_s: float = 8.0) -> CubeConnection:
    last_error = None
    candidate_ports = expand_cube_ports(ports)
    for port in candidate_ports:
        master = None
        try:
            master = mavutil.mavlink_connection(port, baud=baud)
            wait_autopilot_heartbeat(master, heartbeat_timeout_s)
            return CubeConnection(master=master, port=port, baud=baud)
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            if master is not None and hasattr(master, "close"):
                try:
                    master.close()
                except Exception:  # noqa: BLE001
                    pass
    raise RuntimeError(f"Failed to connect to Cube on ports {candidate_ports}: {last_error}")


def send_odometry(
    connection: CubeConnection,
    pose: PoseSample,
) -> None:
    mav = connection.master.mav
    # Prefer sending ODOMETRY messages. Use an "external" estimator type when
    # available so the flight controller treats this as external navigation data
    # rather than an onboard visual-inertial estimator (which can trigger
    # VisOdom allocation on the FC and cause out-of-memory issues).
    if hasattr(mav, "odometry_send"):
        # Try to use an external estimator type constant if provided by pymavlink,
        # otherwise fall back to 0.
        est_type = getattr(mavutil.mavlink, "MAV_ESTIMATOR_TYPE_EXTERNAL", None)
        if est_type is None:
            est_type = getattr(mavutil.mavlink, "MAV_ESTIMATOR_TYPE_UNKNOWN", 0)

        mav.odometry_send(
            pose.timestamp_us,
            getattr(mavutil.mavlink, "MAV_FRAME_LOCAL_FRD", 20),
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            pose.x_m,
            pose.y_m,
            pose.z_m,
            [pose.qw, pose.qx, pose.qy, pose.qz],
            pose.vx_m_s,
            pose.vy_m_s,
            pose.vz_m_s,
            pose.roll_rate_rad_s,
            pose.pitch_rate_rad_s,
            pose.yaw_rate_rad_s,
            [float("nan")] * 21,
            [float("nan")] * 21,
            0,
            est_type,
            pose.pose_quality,
        )
        return

    # If ODOMETRY is not available in this pymavlink build, deliberately avoid
    # sending VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE messages. Those
    # messages can cause the flight controller to enable its visual-odometry
    # subsystem which may run out of resources. Instead, skip sending and let
    # higher-level tooling handle compatibility.
    return
