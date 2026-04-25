import os
import select
import statistics
import termios
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

try:
    import serial.tools.list_ports
except Exception:  # noqa: BLE001
    serial = None


POINT_PACKET_LEN = 80
IMU_PACKET_LEN = 34
FAULT_PACKET_LEN = 41
POINT_HEADER = b"\xee\xff"
FAULT_HEADER = b"\xee\xdd"
JT16_VID = 0x067B
JT16_PID = 0x23A3
JT16_SYMLINK = "/dev/jt16_usb"


@dataclass
class PointSample:
    channel: int
    azimuth_deg: float
    distance_m: float
    reflectivity: int


@dataclass
class LidarSnapshot:
    timestamp_s: float = 0.0
    point_packets: int = 0
    imu_packets: int = 0
    fault_packets: int = 0
    unknown_headers: int = 0
    min_distance_m: float = 0.0
    filtered_distance_m: float = 0.0
    median_distance_m: float = 0.0
    max_distance_m: float = 0.0
    min_azimuth_deg: float = 0.0
    sector_distances_m: list[float] = field(default_factory=list)


def find_lidar_port() -> str | None:
    if Path(JT16_SYMLINK).exists():
        return JT16_SYMLINK

    if serial is not None:
        for port in serial.tools.list_ports.comports():
            if port.vid == JT16_VID and port.pid == JT16_PID and port.device.startswith("/dev/ttyUSB"):
                return port.device

    for candidate in sorted(Path("/dev").glob("ttyUSB*")):
        return str(candidate)
    return None


def choose_lidar_port(port_arg: str) -> str:
    if port_arg != "auto":
        return port_arg
    detected = find_lidar_port()
    if detected is None:
        raise RuntimeError("No JT lidar serial port found. Check /dev/jt16_usb or /dev/ttyUSB*.")
    return detected


def _baud_constant(baud: int):
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise RuntimeError(f"termios does not expose {name}; use an adapter/driver that supports {baud}.")
    return getattr(termios, name)


def open_raw_lidar(path: str, baud: int):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = _baud_constant(baud)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def extract_point_samples(packet: bytes) -> list[PointSample]:
    body_offset = 16
    azimuth_raw = int.from_bytes(packet[body_offset : body_offset + 2], "little")
    azimuth_deg = azimuth_raw * 0.01
    samples: list[PointSample] = []
    channel_offset = body_offset + 2
    for channel in range(16):
        offset = channel_offset + channel * 3
        distance_raw = int.from_bytes(packet[offset : offset + 2], "little")
        reflectivity = packet[offset + 2]
        distance_m = distance_raw * 0.004
        samples.append(PointSample(channel, azimuth_deg, distance_m, reflectivity))
    return samples


def consume_packets(
    buffer: bytearray,
    on_point_packet: Callable[[bytes], None],
    on_imu_packet: Callable[[], None],
    on_fault_packet: Callable[[], None],
    on_sync_loss: Callable[[], None],
) -> None:
    while len(buffer) >= IMU_PACKET_LEN:
        if buffer[:2] == POINT_HEADER:
            if len(buffer) < 6:
                return
            data_type = buffer[5]
            if data_type == 0:
                if len(buffer) < POINT_PACKET_LEN:
                    return
                packet = bytes(buffer[:POINT_PACKET_LEN])
                del buffer[:POINT_PACKET_LEN]
                on_point_packet(packet)
                continue
            if data_type == 1:
                if len(buffer) < IMU_PACKET_LEN:
                    return
                del buffer[:IMU_PACKET_LEN]
                on_imu_packet()
                continue

        if buffer[:2] == FAULT_HEADER:
            if len(buffer) < FAULT_PACKET_LEN:
                return
            del buffer[:FAULT_PACKET_LEN]
            on_fault_packet()
            continue

        next_point = buffer.find(POINT_HEADER, 1)
        next_fault = buffer.find(FAULT_HEADER, 1)
        candidates = [idx for idx in (next_point, next_fault) if idx >= 0]
        if not candidates:
            del buffer[:-1]
            on_sync_loss()
            return
        del buffer[: min(candidates)]
        on_sync_loss()


class LidarReader:
    def __init__(
        self,
        port: str,
        baud: int,
        sector_count: int = 72,
        filter_samples: int = 15,
        min_valid_distance_m: float = 0.15,
        max_valid_distance_m: float = 40.0,
    ):
        self.port = choose_lidar_port(port)
        self.baud = baud
        self.sector_count = sector_count
        self.min_valid_distance_m = min_valid_distance_m
        self.max_valid_distance_m = max_valid_distance_m
        self.recent_packet_distances_m = deque(maxlen=max(1, filter_samples))
        self.fd = open_raw_lidar(self.port, self.baud)
        self.buffer = bytearray()
        self.snapshot = LidarSnapshot(
            timestamp_s=time.time(),
            sector_distances_m=[0.0] * self.sector_count,
        )

    @classmethod
    def open(
        cls,
        port: str = "auto",
        baud: int = 3000000,
        sector_count: int = 72,
        filter_samples: int = 15,
        min_valid_distance_m: float = 0.15,
        max_valid_distance_m: float = 40.0,
    ) -> "LidarReader":
        return cls(port, baud, sector_count, filter_samples, min_valid_distance_m, max_valid_distance_m)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def poll(self, duration_s: float = 0.0) -> LidarSnapshot:
        deadline_s = time.time() + max(duration_s, 0.0)
        while True:
            timeout_s = max(0.0, deadline_s - time.time())
            readable, _, _ = select.select([self.fd], [], [], timeout_s)
            if readable:
                try:
                    chunk = os.read(self.fd, 8192)
                except BlockingIOError:
                    chunk = b""
                if chunk:
                    self.buffer.extend(chunk)
                    consume_packets(
                        self.buffer,
                        self._on_point_packet,
                        self._on_imu_packet,
                        self._on_fault_packet,
                        self._on_sync_loss,
                    )
            if time.time() >= deadline_s or not readable:
                return self.snapshot

    def _on_point_packet(self, packet: bytes) -> None:
        samples = [
            sample
            for sample in extract_point_samples(packet)
            if self.min_valid_distance_m <= sample.distance_m <= self.max_valid_distance_m
        ]
        self.snapshot.point_packets += 1
        self.snapshot.timestamp_s = time.time()
        if not samples:
            return

        distances = [sample.distance_m for sample in samples]
        nearest = min(samples, key=lambda sample: sample.distance_m)
        robust_packet_distance_m = sorted(distances)[min(2, len(distances) - 1)]
        self.recent_packet_distances_m.append(robust_packet_distance_m)
        self.snapshot.min_distance_m = nearest.distance_m
        self.snapshot.filtered_distance_m = statistics.median(self.recent_packet_distances_m)
        self.snapshot.median_distance_m = statistics.median(distances)
        self.snapshot.max_distance_m = max(distances)
        self.snapshot.min_azimuth_deg = nearest.azimuth_deg

        for sample in samples:
            sector = int((sample.azimuth_deg % 360.0) / 360.0 * self.sector_count)
            sector = max(0, min(self.sector_count - 1, sector))
            current = self.snapshot.sector_distances_m[sector]
            if current <= 0.0 or sample.distance_m < current:
                self.snapshot.sector_distances_m[sector] = sample.distance_m

    def _on_imu_packet(self) -> None:
        self.snapshot.imu_packets += 1

    def _on_fault_packet(self) -> None:
        self.snapshot.fault_packets += 1

    def _on_sync_loss(self) -> None:
        self.snapshot.unknown_headers += 1
