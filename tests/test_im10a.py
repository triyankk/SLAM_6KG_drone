import math

import pytest

from optflow_slam.im10a import (
    Im10aDecoder,
    Im10aSampleAssembler,
    decode_packet,
)


def frame(frame_type: int, values: tuple[int, int, int, int]) -> bytes:
    payload = bytearray((0x55, frame_type))
    for value in values:
        payload.extend(int(value).to_bytes(2, "little", signed=True))
    payload.append(sum(payload) & 0xFF)
    return bytes(payload)


def test_decoder_handles_split_packets_and_rejects_bad_checksum() -> None:
    decoder = Im10aDecoder()
    good = frame(0x51, (2048, -1024, 16384, 0))
    bad = bytearray(frame(0x52, (1, 2, 3, 0)))
    bad[-1] ^= 0xFF

    assert decoder.feed(b"\x00\x99" + good[:4]) == []
    measurements = decoder.feed(good[4:] + bytes(bad))

    assert len(measurements) == 1
    assert measurements[0].kind == "accel_mss"
    assert measurements[0].values[2] == pytest.approx(8 * 9.80665)
    assert decoder.valid_frames == 1
    assert decoder.checksum_errors == 1


def test_gyro_and_angle_packets_use_si_units() -> None:
    gyro = decode_packet(frame(0x52, (16384, -8192, 0, 0)))
    angle = decode_packet(frame(0x53, (16384, -8192, 4096, 0)))

    assert gyro is not None
    assert gyro.kind == "gyro_rads"
    assert gyro.values == pytest.approx(
        (math.radians(1000), math.radians(-500), 0)
    )
    assert angle is not None
    assert angle.kind == "euler_rad"
    assert angle.values == pytest.approx(
        (math.radians(90), math.radians(-45), math.radians(22.5))
    )


def test_quaternion_is_normalized() -> None:
    measurement = decode_packet(frame(0x59, (30000, 1000, -2000, 5000)))

    assert measurement is not None
    assert measurement.kind == "quaternion_wxyz"
    assert math.sqrt(sum(value**2 for value in measurement.values)) == pytest.approx(
        1.0
    )


def test_time_packet_decodes_calendar_and_sensor_seconds() -> None:
    packet = bytearray((0x55, 0x50, 26, 7, 31, 12, 34, 56))
    packet.extend((789).to_bytes(2, "little"))
    packet.append(sum(packet) & 0xFF)

    measurement = decode_packet(bytes(packet))

    assert measurement is not None
    assert measurement.kind == "sensor_time"
    assert measurement.values == (2026.0, 7.0, 31.0, 12.0, 34.0, 56.0, 789.0)
    assert measurement.sensor_time_s is not None


def test_zero_calendar_decodes_as_relative_sensor_time() -> None:
    relative_time = bytearray((0x55, 0x50, 0, 0, 0, 2, 30, 32))
    relative_time.extend((100).to_bytes(2, "little"))
    relative_time.append(sum(relative_time) & 0xFF)
    decoder = Im10aDecoder()

    measurements = decoder.feed(
        bytes(relative_time) + frame(0x51, (100, 200, 300, 0))
    )

    assert [measurement.kind for measurement in measurements] == [
        "sensor_time",
        "accel_mss"
    ]
    assert measurements[0].sensor_time_s == pytest.approx(9032.1)
    assert measurements[0].sensor_time_calendar_valid is False
    assert decoder.valid_frames == 2
    assert decoder.payload_errors == 0


def test_decoder_counts_invalid_calendar_without_losing_stream() -> None:
    invalid_time = bytearray((0x55, 0x50, 26, 13, 1, 0, 0, 0, 0, 0))
    invalid_time.append(sum(invalid_time) & 0xFF)
    decoder = Im10aDecoder()

    measurements = decoder.feed(
        bytes(invalid_time) + frame(0x51, (100, 200, 300, 0))
    )

    assert [measurement.kind for measurement in measurements] == ["accel_mss"]
    assert decoder.valid_frames == 2
    assert decoder.payload_errors == 1


def test_timestamped_sample_assembler_joins_time_accel_and_gyro() -> None:
    assembler = Im10aSampleAssembler()
    time_packet = bytearray((0x55, 0x50, 26, 7, 31, 12, 34, 56))
    time_packet.extend((10).to_bytes(2, "little"))
    time_packet.append(sum(time_packet) & 0xFF)
    measurements = [
        decode_packet(bytes(time_packet)),
        decode_packet(frame(0x51, (100, 200, 300, 0))),
        decode_packet(frame(0x52, (400, 500, 600, 0))),
    ]

    sample = None
    for index, measurement in enumerate(measurements):
        assert measurement is not None
        sample = assembler.push(measurement, 1_000_000_000 + index)

    assert sample is not None
    assert sample.host_monotonic_ns == 1_000_000_002
    assert sample.accel_mss[0] > 0.0
    assert sample.gyro_rads[2] > 0.0
    assert assembler.completed_samples == 1
