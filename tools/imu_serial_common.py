#!/usr/bin/env python3

import math
import re
import statistics
import subprocess
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import serial
import serial.tools.list_ports


FLOAT_PATTERN = re.compile(r"[-+]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][-+]?\d+)?")
LABELLED_VALUE_PATTERN = re.compile(
    r"(?P<label>[A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(?P<value>[-+]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][-+]?\d+)?)"
)
DEFAULT_BAUDS = [9600, 115200, 230400, 460800, 921600, 57600, 38400, 19200, 4800]
IM10A_VID = 0x1A86
IM10A_PID = 0x7523
JY901_SYNC = 0x55
JY901_FRAME_LEN = 11
JY901_FRAME_NAMES = {
    0x50: "time",
    0x51: "acc",
    0x52: "gyro",
    0x53: "angle",
    0x54: "mag",
    0x56: "pressure_alt",
    0x57: "lon_lat",
    0x58: "gps_ground",
    0x59: "quaternion",
}


@dataclass
class SerialPortCandidate:
    device: str
    description: str
    hwid: str
    vid: Optional[int]
    pid: Optional[int]
    manufacturer: Optional[str]
    product: Optional[str]
    serial_number: Optional[str]


@dataclass
class BaudProbe:
    baud: int
    byte_count: int
    printable_ratio: float
    line_count: int
    valid_frame_count: int
    nonzero_ratio: float
    sample_text: str
    sample_hex: str


@dataclass
class OrientationEstimate:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    source: str


@dataclass
class Im10aTelemetry:
    valid_frame_count: int = 0
    frame_counts: dict[str, int] = field(default_factory=dict)
    last_frame_name: str = ""
    acc_g: Optional[tuple[float, float, float]] = None
    gyro_deg_s: Optional[tuple[float, float, float]] = None
    angle_deg: Optional[tuple[float, float, float]] = None
    quaternion: Optional[tuple[float, float, float, float]] = None
    mag_raw: Optional[tuple[int, int, int]] = None
    pressure_pa: Optional[int] = None
    altitude_m: Optional[float] = None
    last_update_s: float = 0.0


def list_serial_ports() -> list[SerialPortCandidate]:
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append(
            SerialPortCandidate(
                device=port.device,
                description=port.description or "",
                hwid=port.hwid or "",
                vid=port.vid,
                pid=port.pid,
                manufacturer=port.manufacturer,
                product=port.product,
                serial_number=port.serial_number,
            )
        )
    return ports


def find_im10a_port() -> Optional[str]:
    for port in list_serial_ports():
        if port.vid == IM10A_VID and port.pid == IM10A_PID:
            return port.device
    return None


def choose_port(port_arg: str) -> str:
    if port_arg != "auto":
        return port_arg

    if Path("/dev/imu_usb").exists():
        return "/dev/imu_usb"

    im10a = find_im10a_port()
    if im10a:
        return im10a

    candidates = []
    for port in list_serial_ports():
        if port.device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyTHS", "/dev/ttyS")):
            candidates.append(port.device)
    if not candidates:
        raise SystemExit(
            "No serial ports found. If the IM10A is connected but missing, brltty may still be stealing the CH341 interface."
        )
    return sorted(candidates)[0]


def open_serial(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baudrate=baud, timeout=0.05)


def safe_read(ser: serial.Serial, size: int) -> bytes:
    try:
        return ser.read(size)
    except serial.SerialException:
        time.sleep(0.02)
        return b""


def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    printable = sum(32 <= byte < 127 or byte in (9, 10, 13) for byte in data)
    return printable / len(data)


def nonzero_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    return sum(byte != 0 for byte in data) / len(data)


def _i16(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 2], "little", signed=True)


def _i32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "little", signed=True)


def _u32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "little", signed=False)


def is_valid_jy901_frame(frame: bytes) -> bool:
    return len(frame) == JY901_FRAME_LEN and frame[0] == JY901_SYNC and ((sum(frame[:10]) & 0xFF) == frame[10])


def extract_jy901_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    index = 0
    limit = len(buffer) - JY901_FRAME_LEN
    while index <= limit:
        if buffer[index] != JY901_SYNC:
            index += 1
            continue
        frame = bytes(buffer[index : index + JY901_FRAME_LEN])
        if is_valid_jy901_frame(frame):
            frames.append(frame)
            index += JY901_FRAME_LEN
        else:
            index += 1
    if index:
        del buffer[:index]
    return frames


def parse_jy901_frame(frame: bytes, telemetry: Optional[Im10aTelemetry] = None) -> dict[str, object]:
    if not is_valid_jy901_frame(frame):
        raise ValueError("Invalid JY901 frame")

    payload = frame[2:10]
    packet_id = frame[1]
    packet_name = JY901_FRAME_NAMES.get(packet_id, f"type_0x{packet_id:02x}")
    decoded: dict[str, object] = {"packet_id": packet_id, "packet_name": packet_name}

    if packet_id == 0x51:
        decoded["acc_g"] = tuple(_i16(payload, offset) / 32768.0 * 16.0 for offset in (0, 2, 4))
    elif packet_id == 0x52:
        decoded["gyro_deg_s"] = tuple(_i16(payload, offset) / 32768.0 * 2000.0 for offset in (0, 2, 4))
    elif packet_id == 0x53:
        decoded["angle_deg"] = tuple(_i16(payload, offset) / 32768.0 * 180.0 for offset in (0, 2, 4))
    elif packet_id == 0x54:
        decoded["mag_raw"] = tuple(_i16(payload, offset) for offset in (0, 2, 4))
    elif packet_id == 0x56:
        decoded["pressure_pa"] = _u32(payload, 0)
        decoded["altitude_m"] = _i32(payload, 4) / 100.0
    elif packet_id == 0x59:
        decoded["quaternion"] = tuple(_i16(payload, offset) / 32768.0 for offset in (0, 2, 4, 6))

    if telemetry is not None:
        telemetry.valid_frame_count += 1
        telemetry.last_frame_name = packet_name
        telemetry.frame_counts[packet_name] = telemetry.frame_counts.get(packet_name, 0) + 1
        telemetry.last_update_s = time.time()
        if "acc_g" in decoded:
            telemetry.acc_g = decoded["acc_g"]  # type: ignore[assignment]
        if "gyro_deg_s" in decoded:
            telemetry.gyro_deg_s = decoded["gyro_deg_s"]  # type: ignore[assignment]
        if "angle_deg" in decoded:
            telemetry.angle_deg = decoded["angle_deg"]  # type: ignore[assignment]
        if "mag_raw" in decoded:
            telemetry.mag_raw = decoded["mag_raw"]  # type: ignore[assignment]
        if "pressure_pa" in decoded:
            telemetry.pressure_pa = decoded["pressure_pa"]  # type: ignore[assignment]
        if "altitude_m" in decoded:
            telemetry.altitude_m = decoded["altitude_m"]  # type: ignore[assignment]
        if "quaternion" in decoded:
            telemetry.quaternion = decoded["quaternion"]  # type: ignore[assignment]
    return decoded


def parse_jy901_stream(buffer: bytearray, telemetry: Optional[Im10aTelemetry] = None) -> list[dict[str, object]]:
    frames = extract_jy901_frames(buffer)
    return [parse_jy901_frame(frame, telemetry) for frame in frames]


def telemetry_orientation(telemetry: Im10aTelemetry) -> Optional[OrientationEstimate]:
    if telemetry.quaternion is not None:
        qw, qx, qy, qz = telemetry.quaternion
        roll, pitch, yaw = quaternion_to_euler_deg(qw, qx, qy, qz)
        return OrientationEstimate(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw, source="im10a-quaternion")
    if telemetry.angle_deg is not None:
        roll, pitch, yaw = telemetry.angle_deg
        return OrientationEstimate(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw, source="im10a-angle")
    return None


def probe_baud(port: str, baud_candidates: list[int], scan_seconds: float) -> BaudProbe:
    best: Optional[BaudProbe] = None
    for baud in baud_candidates:
        try:
            ser = open_serial(port, baud)
        except Exception:
            continue
        collected = bytearray()
        start_s = time.time()
        try:
            time.sleep(0.15)
            ser.reset_input_buffer()
            while time.time() - start_s < scan_seconds:
                chunk = safe_read(ser, 2048)
                if chunk:
                    collected.extend(chunk)
        finally:
            ser.close()

        sample_bytes = bytes(collected[:160])
        sample_text = sample_bytes.decode("utf-8", errors="ignore").replace("\r", " ").replace("\n", " ")
        valid_frame_count = len(extract_jy901_frames(bytearray(collected)))
        probe = BaudProbe(
            baud=baud,
            byte_count=len(collected),
            printable_ratio=printable_ratio(sample_bytes),
            line_count=sample_bytes.count(b"\n"),
            valid_frame_count=valid_frame_count,
            nonzero_ratio=nonzero_ratio(sample_bytes),
            sample_text=sample_text[:120],
            sample_hex=sample_bytes[:80].hex(" "),
        )
        if best is None:
            best = probe
            continue
        score = (
            probe.valid_frame_count,
            probe.nonzero_ratio,
            probe.byte_count,
            probe.printable_ratio,
            probe.line_count,
        )
        best_score = (
            best.valid_frame_count,
            best.nonzero_ratio,
            best.byte_count,
            best.printable_ratio,
            best.line_count,
        )
        if score > best_score:
            best = probe

    if best is None:
        fallback = baud_candidates[0] if baud_candidates else 9600
        return BaudProbe(
            baud=fallback,
            byte_count=0,
            printable_ratio=0.0,
            line_count=0,
            valid_frame_count=0,
            nonzero_ratio=0.0,
            sample_text="",
            sample_hex="",
        )
    return best


def parse_numeric_values(line: str) -> list[float]:
    values = []
    for token in FLOAT_PATTERN.findall(line):
        try:
            values.append(float(token))
        except ValueError:
            continue
    return values


def parse_labelled_values(line: str) -> dict[str, float]:
    values = {}
    for match in LABELLED_VALUE_PATTERN.finditer(line):
        label = match.group("label").strip().lower()
        try:
            values[label] = float(match.group("value"))
        except ValueError:
            continue
    return values


def quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def infer_orientation(line: str) -> Optional[OrientationEstimate]:
    labelled = parse_labelled_values(line)
    if labelled:
        aliases = {
            "roll": ["roll", "r", "anglex", "rx"],
            "pitch": ["pitch", "p", "angley", "ry"],
            "yaw": ["yaw", "y", "anglez", "rz", "heading"],
        }
        if all(any(alias in labelled for alias in aliases[axis]) for axis in ("roll", "pitch", "yaw")):
            roll = next(labelled[a] for a in aliases["roll"] if a in labelled)
            pitch = next(labelled[a] for a in aliases["pitch"] if a in labelled)
            yaw = next(labelled[a] for a in aliases["yaw"] if a in labelled)
            return OrientationEstimate(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw, source="labelled-euler")

        quat_aliases = {
            "w": ["qw", "q0", "w"],
            "x": ["qx", "q1", "x"],
            "y": ["qy", "q2", "y"],
            "z": ["qz", "q3", "z"],
        }
        if all(any(alias in labelled for alias in quat_aliases[key]) for key in ("w", "x", "y", "z")):
            qw = next(labelled[a] for a in quat_aliases["w"] if a in labelled)
            qx = next(labelled[a] for a in quat_aliases["x"] if a in labelled)
            qy = next(labelled[a] for a in quat_aliases["y"] if a in labelled)
            qz = next(labelled[a] for a in quat_aliases["z"] if a in labelled)
            roll, pitch, yaw = quaternion_to_euler_deg(qw, qx, qy, qz)
            return OrientationEstimate(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw, source="labelled-quaternion")

    numeric = parse_numeric_values(line)
    if len(numeric) >= 4:
        first_four = numeric[:4]
        norm = math.sqrt(sum(value * value for value in first_four))
        if 0.6 <= norm <= 1.4:
            roll, pitch, yaw = quaternion_to_euler_deg(first_four[0], first_four[1], first_four[2], first_four[3])
            return OrientationEstimate(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw, source="heuristic-quaternion")

    if len(numeric) >= 3:
        return OrientationEstimate(
            roll_deg=numeric[0],
            pitch_deg=numeric[1],
            yaw_deg=numeric[2],
            source="heuristic-euler",
        )
    return None


def detect_brltty_processes() -> list[str]:
    try:
        proc = subprocess.run(
            ["ps", "-ef"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return []

    matches = []
    for line in proc.stdout.splitlines():
        lower = line.lower()
        if "brltty" in lower or "xbrlapi" in lower:
            matches.append(line.strip())
    return matches


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


def summarize_numeric_channels(channel_history: list[deque]) -> list[str]:
    summary = []
    for idx, history in enumerate(channel_history):
        if len(history) < 2:
            continue
        values = list(history)
        summary.append(
            f"ch{idx}: latest={values[-1]:.4f} min={min(values):.4f} "
            f"max={max(values):.4f} std={statistics.pstdev(values):.4f}"
        )
    return summary
