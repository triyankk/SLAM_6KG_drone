import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial
import serial.tools.list_ports

from .types import ImuSample, PoseSample


IM10A_VID = 0x1A86
IM10A_PID = 0x7523
DEFAULT_BAUDS = [9600, 115200, 230400, 460800, 921600, 57600, 38400, 19200, 4800]
FRAME_LEN = 11
SYNC = 0x55


@dataclass
class ExternalImuProbe:
    port: str
    baud: int
    frame_count: int
    byte_count: int
    nonzero_ratio: float
    orientation_ready: bool = False


@dataclass
class ExternalImuHealth:
    usb_present: bool
    port: Optional[str]
    baud: Optional[int]
    stream_healthy: bool
    message: str
    sample: Optional[ImuSample] = None


@dataclass
class _TelemetryState:
    acc_g: Optional[tuple[float, float, float]] = None
    gyro_deg_s: Optional[tuple[float, float, float]] = None
    angle_deg: Optional[tuple[float, float, float]] = None
    quaternion: Optional[tuple[float, float, float, float]] = None
    mag_raw: Optional[tuple[int, int, int]] = None
    pressure_pa: Optional[float] = None
    altitude_m: Optional[float] = None
    frame_count: int = 0


def im10a_usb_present() -> bool:
    for root in Path("/sys/bus/usb/devices").iterdir():
        try:
            vendor = (root / "idVendor").read_text().strip().lower()
            product = (root / "idProduct").read_text().strip().lower()
        except Exception:
            continue
        if vendor == "1a86" and product == "7523":
            return True
    return False


def find_im10a_port() -> Optional[str]:
    if Path("/dev/imu_usb").exists():
        return "/dev/imu_usb"
    for port in serial.tools.list_ports.comports():
        if port.vid == IM10A_VID and port.pid == IM10A_PID:
            return port.device
    return None


def choose_imu_port(port_arg: str) -> str:
    if port_arg != "auto":
        return port_arg
    detected = find_im10a_port()
    if detected:
        return detected
    raise RuntimeError("No IM10A serial port detected")


def safe_read(ser: serial.Serial, size: int) -> bytes:
    try:
        return ser.read(size)
    except serial.SerialException:
        time.sleep(0.02)
        return b""


def _i16(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 2], "little", signed=True)


def _i32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "little", signed=True)


def _u32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "little", signed=False)


def quaternion_to_euler_deg(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def _is_valid_frame(frame: bytes) -> bool:
    return len(frame) == FRAME_LEN and frame[0] == SYNC and ((sum(frame[:10]) & 0xFF) == frame[10])


def extract_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    index = 0
    limit = len(buffer) - FRAME_LEN
    while index <= limit:
        if buffer[index] != SYNC:
            index += 1
            continue
        frame = bytes(buffer[index : index + FRAME_LEN])
        if _is_valid_frame(frame):
            frames.append(frame)
            index += FRAME_LEN
        else:
            index += 1
    if index:
        del buffer[:index]
    return frames


def update_state(frame: bytes, state: _TelemetryState) -> None:
    payload = frame[2:10]
    frame_type = frame[1]
    state.frame_count += 1
    if frame_type == 0x51:
        state.acc_g = tuple(_i16(payload, offset) / 32768.0 * 16.0 for offset in (0, 2, 4))
    elif frame_type == 0x52:
        state.gyro_deg_s = tuple(_i16(payload, offset) / 32768.0 * 2000.0 for offset in (0, 2, 4))
    elif frame_type == 0x53:
        state.angle_deg = tuple(_i16(payload, offset) / 32768.0 * 180.0 for offset in (0, 2, 4))
    elif frame_type == 0x54:
        state.mag_raw = tuple(_i16(payload, offset) for offset in (0, 2, 4))
    elif frame_type == 0x56:
        state.pressure_pa = float(_u32(payload, 0))
        state.altitude_m = _i32(payload, 4) / 100.0
    elif frame_type == 0x59:
        state.quaternion = tuple(_i16(payload, offset) / 32768.0 for offset in (0, 2, 4, 6))


def imu_sample_from_state(state: _TelemetryState) -> Optional[ImuSample]:
    if state.quaternion is not None:
        qw, qx, qy, qz = state.quaternion
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(qw, qx, qy, qz)
    elif state.angle_deg is not None:
        roll_deg, pitch_deg, yaw_deg = state.angle_deg
        half_roll = math.radians(roll_deg) * 0.5
        half_pitch = math.radians(pitch_deg) * 0.5
        half_yaw = math.radians(yaw_deg) * 0.5
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        cy, sy = math.cos(half_yaw), math.sin(half_yaw)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
    else:
        return None

    gx, gy, gz = state.gyro_deg_s or (0.0, 0.0, 0.0)
    ax, ay, az = state.acc_g or (0.0, 0.0, 0.0)
    mx, my, mz = state.mag_raw or (0, 0, 0)
    return ImuSample(
        timestamp_us=int(time.time_ns() // 1000),
        qw=qw,
        qx=qx,
        qy=qy,
        qz=qz,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
        gx_deg_s=gx,
        gy_deg_s=gy,
        gz_deg_s=gz,
        ax_g=ax,
        ay_g=ay,
        az_g=az,
        mx_raw=mx,
        my_raw=my,
        mz_raw=mz,
        pressure_pa=state.pressure_pa or 0.0,
        altitude_m=state.altitude_m or 0.0,
    )


def nonzero_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    return sum(byte != 0 for byte in data) / len(data)


def probe_baud(port: str, baud_candidates: list[int], scan_seconds: float) -> ExternalImuProbe:
    best: Optional[ExternalImuProbe] = None
    for baud in baud_candidates:
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        except Exception:
            continue
        collected = bytearray()
        try:
            time.sleep(0.15)
            ser.reset_input_buffer()
            deadline = time.time() + scan_seconds
            while time.time() < deadline:
                chunk = safe_read(ser, 2048)
                if chunk:
                    collected.extend(chunk)
        finally:
            ser.close()

        state = _TelemetryState()
        parsed_buffer = bytearray(collected)
        parsed_frames = extract_frames(parsed_buffer)
        for frame in parsed_frames:
            update_state(frame, state)
        sample = imu_sample_from_state(state)
        probe = ExternalImuProbe(
            port=port,
            baud=baud,
            frame_count=len(parsed_frames),
            byte_count=len(collected),
            nonzero_ratio=nonzero_ratio(bytes(collected[:160])),
            orientation_ready=sample is not None,
        )
        if best is None:
            best = probe
            continue
        score = (
            int(probe.orientation_ready),
            probe.frame_count,
            probe.nonzero_ratio,
            probe.byte_count,
        )
        best_score = (
            int(best.orientation_ready),
            best.frame_count,
            best.nonzero_ratio,
            best.byte_count,
        )
        if score > best_score:
            best = probe

    if best is None:
        raise RuntimeError(f"Failed to probe IM10A baud on {port}")
    return best


def collect_imu_health(port_arg: str = "auto", baud_arg: str = "auto", scan_seconds: float = 0.8) -> ExternalImuHealth:
    usb_present = im10a_usb_present()
    try:
        port = choose_imu_port(port_arg)
    except Exception as exc:  # noqa: BLE001
        return ExternalImuHealth(
            usb_present=usb_present,
            port=None,
            baud=None,
            stream_healthy=False,
            message=f"IM10A port not ready: {exc}",
        )

    if baud_arg == "auto":
        probe = probe_baud(port, DEFAULT_BAUDS, scan_seconds)
        baud = probe.baud
    else:
        baud = int(baud_arg)

    state = _TelemetryState()
    buffer = bytearray()
    total_bytes = 0
    ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    try:
        deadline = time.time() + scan_seconds
        while time.time() < deadline:
            chunk = safe_read(ser, 2048)
            if not chunk:
                continue
            total_bytes += len(chunk)
            buffer.extend(chunk)
            for frame in extract_frames(buffer):
                update_state(frame, state)
    finally:
        ser.close()

    sample = imu_sample_from_state(state)
    healthy = state.frame_count > 0 and sample is not None
    if healthy:
        message = (
            f"IM10A stream healthy on {port} at {baud} baud: "
            f"frames={state.frame_count} roll={sample.roll_deg:+.2f} pitch={sample.pitch_deg:+.2f} yaw={sample.yaw_deg:+.2f}"
        )
    else:
        message = f"IM10A detected on {port}, but no usable orientation sample was decoded at {baud} baud."

    return ExternalImuHealth(
        usb_present=usb_present,
        port=port,
        baud=baud,
        stream_healthy=healthy,
        message=message,
        sample=sample,
    )


class Im10aReader:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.buffer = bytearray()
        self.state = _TelemetryState()
        self.latest_sample: Optional[ImuSample] = None

    @classmethod
    def open(cls, port_arg: str = "auto", baud_arg: str = "auto", scan_seconds: float = 0.8) -> "Im10aReader":
        port = choose_imu_port(port_arg)
        if baud_arg == "auto":
            baud = probe_baud(port, DEFAULT_BAUDS, scan_seconds).baud
        else:
            baud = int(baud_arg)
        return cls(port=port, baud=baud)

    def poll(self, duration_s: float = 0.05) -> Optional[ImuSample]:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            chunk = safe_read(self.serial, 2048)
            if not chunk:
                continue
            self.buffer.extend(chunk)
            for frame in extract_frames(self.buffer):
                update_state(frame, self.state)
        sample = imu_sample_from_state(self.state)
        if sample is not None:
            self.latest_sample = sample
        return self.latest_sample

    def close(self) -> None:
        self.serial.close()


def apply_imu_sample_to_pose(pose: PoseSample, imu: ImuSample) -> PoseSample:
    return PoseSample(
        timestamp_us=pose.timestamp_us,
        x_m=pose.x_m,
        y_m=pose.y_m,
        z_m=pose.z_m,
        qw=imu.qw,
        qx=imu.qx,
        qy=imu.qy,
        qz=imu.qz,
        vx_m_s=pose.vx_m_s,
        vy_m_s=pose.vy_m_s,
        vz_m_s=pose.vz_m_s,
        roll_rate_rad_s=math.radians(imu.gx_deg_s),
        pitch_rate_rad_s=math.radians(imu.gy_deg_s),
        yaw_rate_rad_s=math.radians(imu.gz_deg_s),
        pose_quality=pose.pose_quality,
        tracking_state=pose.tracking_state,
        feature_count=pose.feature_count,
        tracked_feature_count=pose.tracked_feature_count,
        inlier_count=pose.inlier_count,
        source_name=(f"{pose.source_name}+imu" if pose.source_name else "imu"),
    )
