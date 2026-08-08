"""Decode the read-only Hiwonder IM10A serial stream."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import struct


FRAME_START = 0x55
FRAME_LENGTH = 11
TYPE_TIME = 0x50
TYPE_ACCEL = 0x51
TYPE_GYRO = 0x52
TYPE_ANGLE = 0x53
TYPE_QUATERNION = 0x59
STANDARD_GRAVITY_MSS = 9.80665


@dataclass(frozen=True)
class Im10aMeasurement:
    kind: str
    values: tuple[float, ...]
    sensor_time_s: float | None = None
    sensor_time_calendar_valid: bool | None = None


@dataclass(frozen=True)
class Im10aSample:
    """One timestamped raw acceleration and angular-rate sample."""

    sensor_time_s: float
    host_monotonic_ns: int
    accel_mss: tuple[float, float, float]
    gyro_rads: tuple[float, float, float]


class Im10aSampleAssembler:
    """Join the TIME, ACC and GYRO frames emitted for one IM10A sample."""

    def __init__(self) -> None:
        self._sensor_time_s: float | None = None
        self._accel_mss: tuple[float, float, float] | None = None
        self._gyro_rads: tuple[float, float, float] | None = None
        self._latest_host_monotonic_ns = 0
        self.completed_samples = 0
        self.incomplete_samples = 0

    def push(
        self,
        measurement: Im10aMeasurement,
        host_monotonic_ns: int,
    ) -> Im10aSample | None:
        if host_monotonic_ns <= 0:
            raise ValueError("host_monotonic_ns must be positive")

        if measurement.kind == "sensor_time":
            if measurement.sensor_time_s is None:
                raise ValueError("sensor-time frame is missing sensor_time_s")
            if (
                self._sensor_time_s is not None
                and (self._accel_mss is None or self._gyro_rads is None)
            ):
                self.incomplete_samples += 1
            self._sensor_time_s = measurement.sensor_time_s
            self._accel_mss = None
            self._gyro_rads = None
            self._latest_host_monotonic_ns = host_monotonic_ns
            return None

        if self._sensor_time_s is None:
            return None

        self._latest_host_monotonic_ns = max(
            self._latest_host_monotonic_ns,
            host_monotonic_ns,
        )
        if measurement.kind == "accel_mss":
            self._accel_mss = (
                float(measurement.values[0]),
                float(measurement.values[1]),
                float(measurement.values[2]),
            )
        elif measurement.kind == "gyro_rads":
            self._gyro_rads = (
                float(measurement.values[0]),
                float(measurement.values[1]),
                float(measurement.values[2]),
            )
        else:
            return None

        if self._accel_mss is None or self._gyro_rads is None:
            return None

        sample = Im10aSample(
            sensor_time_s=self._sensor_time_s,
            host_monotonic_ns=self._latest_host_monotonic_ns,
            accel_mss=self._accel_mss,
            gyro_rads=self._gyro_rads,
        )
        self.completed_samples += 1
        self._sensor_time_s = None
        self._accel_mss = None
        self._gyro_rads = None
        return sample


class Im10aDecoder:
    """Incrementally frame and scale the documented 0x55 packet protocol."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.checksum_errors = 0
        self.payload_errors = 0
        self.discarded_bytes = 0
        self.frame_type_counts: dict[int, int] = {}

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
            self.frame_type_counts[packet[1]] = (
                self.frame_type_counts.get(packet[1], 0) + 1
            )
            try:
                measurement = decode_packet(packet)
            except ValueError:
                self.payload_errors += 1
                continue
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
    if frame_type == TYPE_TIME:
        raw_year = packet[2]
        year = 2000 + raw_year
        month = packet[3]
        day = packet[4]
        hour = packet[5]
        minute = packet[6]
        second = packet[7]
        millisecond = struct.unpack_from("<H", packet, 8)[0]
        if millisecond > 999:
            raise ValueError("IM10A time packet has an invalid millisecond")
        if not (
            0 <= hour <= 23
            and 0 <= minute <= 59
            and 0 <= second <= 59
        ):
            raise ValueError("IM10A time packet has an invalid time of day")
        if raw_year == 0 and month == 0 and day == 0:
            sensor_time_s = (
                hour * 3600.0
                + minute * 60.0
                + second
                + millisecond / 1000.0
            )
            calendar_valid = False
            displayed_year = 0.0
        else:
            try:
                instant = datetime(
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                    millisecond * 1000,
                    tzinfo=timezone.utc,
                )
            except ValueError as exc:
                raise ValueError(f"IM10A time packet is invalid: {exc}") from exc
            sensor_time_s = instant.timestamp()
            calendar_valid = True
            displayed_year = float(year)
        return Im10aMeasurement(
            "sensor_time",
            (
                displayed_year,
                float(month),
                float(day),
                float(hour),
                float(minute),
                float(second),
                float(millisecond),
            ),
            sensor_time_s=sensor_time_s,
            sensor_time_calendar_valid=calendar_valid,
        )

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
