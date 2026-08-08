from __future__ import annotations

import math

import numpy as np
import pytest

from optflow_slam.config import (
    DepthCameraConfig,
    PositionConfig,
    RotationConfig,
)
from optflow_slam.rgbd_odometry import (
    GyroPriorBuffer,
    RgbdOdometryEngine,
    backproject_depth,
    camera_to_body_transform,
    matrix_to_quaternion_xyzw,
    rotation_angle_rad,
)


def camera_config() -> DepthCameraConfig:
    return DepthCameraConfig(
        model="D415",
        backend="realsense",
        mounting="forward",
        serial="test",
        width=640,
        height=480,
        fps=30,
        stream_host="127.0.0.1",
        stream_port=8770,
        jpeg_quality=80,
        position_from_cg_frd_m=PositionConfig(0.19, 0.0, 0.10),
        rotation_from_forward_frd=RotationConfig(0.0, 0.0, 0.0),
    )


def test_nominal_camera_transform_maps_optical_to_body_frd() -> None:
    transform = camera_to_body_transform(camera_config())
    assert transform[:3, :3] @ [0.0, 0.0, 1.0] == pytest.approx(
        [1.0, 0.0, 0.0]
    )
    assert transform[:3, :3] @ [1.0, 0.0, 0.0] == pytest.approx(
        [0.0, 1.0, 0.0]
    )
    assert transform[:3, 3] == pytest.approx([0.19, 0.0, 0.10])


def test_quaternion_conversion_preserves_rotation() -> None:
    rotation = np.array(
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    quaternion = matrix_to_quaternion_xyzw(rotation)
    assert quaternion == pytest.approx(
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
    )


def test_gyro_prior_integrates_camera_rotation() -> None:
    buffer = GyroPriorBuffer()
    start_ns = 1_000_000_000
    end_ns = start_ns + 100_000_000
    for offset_ms in range(0, 101, 5):
        buffer.add(
            start_ns + offset_ms * 1_000_000,
            (0.0, 0.0, 1.0),
        )
    body_from_camera = camera_to_body_transform(camera_config())[:3, :3]
    prior = buffer.rotation_prior(start_ns, end_ns, body_from_camera)
    assert prior is not None
    assert prior.covered_duration_s == pytest.approx(0.1)
    assert rotation_angle_rad(
        prior.transform_current_from_previous[:3, :3]
    ) == pytest.approx(0.1, abs=1.0e-5)


class FakeOdometry:
    def __init__(self, transform: np.ndarray) -> None:
        self.transform = transform
        self.initial_transform: np.ndarray | None = None

    def setMaxTranslation(self, value: float) -> None:
        assert value == 0.5

    def setMaxRotation(self, value: float) -> None:
        assert value == 0.5

    def compute(self, *arguments):
        self.initial_transform = arguments[-1]
        return True, self.transform.copy()


class FakeRgbd:
    Odometry_RIGID_BODY_MOTION = 4

    def __init__(self, odometry: FakeOdometry) -> None:
        self.odometry = odometry

    def RgbdOdometry_create(self, *arguments):
        assert len(arguments) == 8
        return self.odometry


class FakeCv2:
    def __init__(self, transform: np.ndarray) -> None:
        self.odometry = FakeOdometry(transform)
        self.rgbd = FakeRgbd(self.odometry)


def test_rgbd_engine_accumulates_inverse_camera_motion() -> None:
    current_from_previous = np.eye(4)
    current_from_previous[0, 3] = 0.1
    fake_cv2 = FakeCv2(current_from_previous)
    engine = RgbdOdometryEngine(np.eye(3), cv2_module=fake_cv2)
    gray = np.full((12, 16), 100, dtype=np.uint8)
    depth = np.ones((12, 16), dtype=np.float32)

    first = engine.process(gray, depth)
    prior = np.eye(4)
    prior[1, 3] = 0.02
    second = engine.process(
        gray,
        depth,
        initial_transform=prior,
        gyro_prior_samples=7,
    )

    assert first.tracking is False
    assert second.tracking is True
    assert second.transform_world_camera[0, 3] == pytest.approx(-0.1)
    assert second.gyro_prior_used is True
    assert second.gyro_prior_samples == 7
    assert fake_cv2.odometry.initial_transform == pytest.approx(prior)


def test_backproject_depth_keeps_registered_color() -> None:
    depth = np.array(((1.0, 0.0), (2.0, 3.0)), dtype=np.float32)
    color = np.array(
        (((0, 0, 255), (0, 0, 0)), ((0, 255, 0), (255, 0, 0))),
        dtype=np.uint8,
    )
    camera_matrix = np.array(
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    )
    points, colors = backproject_depth(
        depth,
        color,
        camera_matrix,
        stride=1,
        minimum_depth_m=0.5,
        maximum_depth_m=2.5,
    )
    np.testing.assert_allclose(points, [[0.0, 0.0, 1.0], [0.0, 2.0, 2.0]])
    assert colors.tolist() == [[255, 0, 0], [0, 255, 0]]
