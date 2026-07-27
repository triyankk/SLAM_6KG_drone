from argparse import Namespace

import pytest

from optflow_slam.camera_server import CAMERA_PAGE, CameraStore, camera_from_args
from optflow_slam.config import ConfigError, DepthCameraConfig


def configured_camera() -> DepthCameraConfig:
    return DepthCameraConfig(
        model="Intel RealSense D415",
        backend="realsense",
        mounting="forward",
        serial="327322062285",
        width=640,
        height=480,
        fps=30,
        stream_host="0.0.0.0",
        stream_port=8770,
        jpeg_quality=82,
    )


def empty_args(**overrides: object) -> Namespace:
    values = {
        "serial": None,
        "width": None,
        "height": None,
        "fps": None,
        "host": None,
        "port": None,
        "jpeg_quality": None,
    }
    values.update(overrides)
    return Namespace(**values)


def test_camera_store_publishes_frame_and_health_metrics() -> None:
    store = CameraStore("Intel RealSense D415", "327322062285")

    assert store.snapshot()["frame_age_ms"] is None
    store.publish(b"jpeg", 640, 480)

    frame, sequence = store.wait_for_frame(0, timeout=0)
    status = store.snapshot()
    assert frame == b"jpeg"
    assert sequence == 1
    assert status["connected"]
    assert status["width"] == 640
    assert status["height"] == 480
    assert status["frame_age_ms"] is not None


def test_camera_store_does_not_repeat_a_stale_frame() -> None:
    store = CameraStore("Intel RealSense D415", None)
    store.publish(b"jpeg", 640, 480)

    frame, sequence = store.wait_for_frame(1, timeout=0)

    assert frame is None
    assert sequence == 1


def test_camera_overrides_preserve_explicit_values() -> None:
    camera = camera_from_args(
        configured_camera(),
        empty_args(width=1280, port=8870, jpeg_quality=90),
    )

    assert camera.width == 1280
    assert camera.height == 480
    assert camera.stream_port == 8870
    assert camera.jpeg_quality == 90


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"fps": 0}, "width, height, and fps"),
        ({"port": 0}, "stream_port"),
        ({"jpeg_quality": 0}, "jpeg_quality"),
    ],
)
def test_invalid_camera_overrides_are_rejected(
    override: dict[str, int], message: str
) -> None:
    with pytest.raises(ConfigError, match=message):
        camera_from_args(configured_camera(), empty_args(**override))


def test_camera_page_uses_the_live_mjpeg_endpoint() -> None:
    assert b'src="/stream.mjpg"' in CAMERA_PAGE
    assert b"/api/status" in CAMERA_PAGE
