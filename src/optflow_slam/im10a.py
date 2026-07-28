"""Decode the read-only Hiwonder IM10A serial stream."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


FRAME_START = 0x55
FRAME_LENGTH = 11
TYPE_ACCEL = 0x51
TYPE_GYRO = 0x52
TYPE_ANGLE = 0x53
TYPE_QUATERNION = 0x59
STANDARD_GRAVITY_MSS = 9.80665


@dataclass(frozen=True)
class Im10aMeasurement:
    kind: str
    values: tuple[float, ...]


class Im10aDecoder:
    """Incrementally frame and scale the documented 0x55 packet protocol."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.checksum_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Im10aMeasurement]:
        self.buffer.extend(data)
        measurements: list[Im10aMeasurement] = []
        while len(self.buffer) >= FRAME_LENGTH:
            try:
                start = self.buffer.index(FRAME_START)
            except ValueError:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < FRAME_LENGTH:
                break

            packet = bytes(self.buffer[:FRAME_LENGTH])
            if sum(packet[:10]) & 0xFF != packet[10]:
                self.checksum_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            del self.buffer[:FRAME_LENGTH]
            self.valid_frames += 1
            measurement = decode_packet(packet)
            if measurement is not None:
                measurements.append(measurement)
        return measurements


def _signed_values(packet: bytes) -> tuple[int, int, int, int]:
    return struct.unpack_from("<hhhh", packet, 2)


def decode_packet(packet: bytes) -> Im10aMeasurement | None:
    if len(packet) != FRAME_LENGTH or packet[0] != FRAME_START:
        raise ValueError("IM10A packet must be one complete 0x55 frame")
    if sum(packet[:10]) & 0xFF != packet[10]:
        raise ValueError("IM10A packet checksum failed")

    frame_type = packet[1]
    raw = _signed_values(packet)
    if frame_type == TYPE_ACCEL:
        scale = 16.0 * STANDARD_GRAVITY_MSS / 32768.0
        return Im10aMeasurement(
            "accel_mss", tuple(value * scale for value in raw[:3])
        )
    if frame_type == TYPE_GYRO:
        scale = math.radians(2000.0) / 32768.0
        return Im10aMeasurement(
            "gyro_rads", tuple(value * scale for value in raw[:3])
        )
    if frame_type == TYPE_ANGLE:
        scale = math.pi / 32768.0
        return Im10aMeasurement(
            "euler_rad", tuple(value * scale for value in raw[:3])
        )
    if frame_type == TYPE_QUATERNION:
        quaternion = tuple(value / 32768.0 for value in raw)
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-9:
            return None
        return Im10aMeasurement(
            "quaternion_wxyz",
            tuple(value / norm for value in quaternion),
        )
    return None
