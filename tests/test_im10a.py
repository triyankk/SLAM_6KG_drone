import math

import pytest

from optflow_slam.im10a import Im10aDecoder, decode_packet


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
