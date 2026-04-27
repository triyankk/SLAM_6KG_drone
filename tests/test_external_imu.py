import time
from slam_core.external_imu import FRAME_LEN, SYNC, apply_imu_sample_to_pose, extract_frames, nonzero_ratio
from slam_core.types import ImuSample, PoseSample


def make_frame(frame_type: int, payload_bytes: bytes) -> bytes:
    assert len(payload_bytes) == 8
    head = bytes([SYNC, frame_type])
    frame = head + payload_bytes
    checksum = (sum(frame[:10]) & 0xFF)
    return frame + bytes([checksum])


def test_extract_single_frame():
    payload = bytes([1, 2, 3, 4, 5, 6, 7, 8])
    frame = make_frame(0x51, payload)
    buf = bytearray(frame)
    frames = extract_frames(buf)
    assert len(frames) == 1
    assert frames[0][0] == SYNC
    assert len(frames[0]) == FRAME_LEN


def test_extract_with_noise_and_two_frames():
    payload = bytes([10] * 8)
    f1 = make_frame(0x51, payload)
    f2 = make_frame(0x52, payload)
    buf = bytearray(b"garbage" + f1 + b"x" + f2 + b"tail")
    frames = extract_frames(buf)
    assert len(frames) == 2


def test_nonzero_ratio():
    assert nonzero_ratio(b"\x00\x00") == 0.0
    assert nonzero_ratio(b"\x01\x00") == 0.5
    assert nonzero_ratio(b"") == 0.0


def test_apply_imu_sample_preserves_pose_metadata():
    pose = PoseSample(
        timestamp_us=int(time.time() * 1e6),
        x_m=1.0,
        y_m=2.0,
        z_m=-0.5,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        pose_quality=77,
        tracking_state="ok",
        feature_count=120,
        tracked_feature_count=80,
        inlier_count=50,
        source_name="vio",
    )
    imu = ImuSample(
        timestamp_us=pose.timestamp_us,
        qw=0.9,
        qx=0.1,
        qy=0.0,
        qz=0.0,
        roll_deg=1.0,
        pitch_deg=2.0,
        yaw_deg=3.0,
        gx_deg_s=4.0,
        gy_deg_s=5.0,
        gz_deg_s=6.0,
    )

    fused = apply_imu_sample_to_pose(pose, imu)

    assert fused.x_m == pose.x_m
    assert fused.pose_quality == pose.pose_quality
    assert fused.tracking_state == pose.tracking_state
    assert fused.feature_count == pose.feature_count
    assert fused.tracked_feature_count == pose.tracked_feature_count
    assert fused.inlier_count == pose.inlier_count
    assert fused.source_name == "vio+imu"
