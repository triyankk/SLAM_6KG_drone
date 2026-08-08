import base64
from dataclasses import replace
from pathlib import Path
from threading import Event
import time

import numpy as np

from optflow_slam.config import load_config
from optflow_slam.spatial_stream import (
    SPATIAL_QUANTIZATION_M,
    SpatialFrameFilePublisher,
    SpatialFrameFileSource,
    SpatialFrameStore,
    camera_points_to_body_frd,
    lidar_points_to_body_frd,
    voxel_sample,
)


ROOT = Path(__file__).resolve().parents[1]


def test_camera_points_include_measured_body_translation() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    body = camera_points_to_body_frd(
        np.array(((0.0, 0.0, 2.0),), dtype=np.float32),
        config,
    )

    np.testing.assert_allclose(body, ((2.19, 0.0, 0.10),), atol=1.0e-6)


def test_lidar_points_include_measured_body_translation() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    zero_yaw_config = replace(
        config,
        lidar=replace(
            config.lidar,
            rotation_to_body_frd=replace(
                config.lidar.rotation_to_body_frd,
                yaw_deg=0.0,
            ),
        ),
    )

    body = lidar_points_to_body_frd(
        np.array(((0.0, 2.0, 0.0),), dtype=np.float32),
        zero_yaw_config,
    )

    np.testing.assert_allclose(body, ((2.0, 0.0, -0.10),), atol=1.0e-6)


def test_lidar_points_include_calibrated_body_yaw() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    body = lidar_points_to_body_frd(
        np.array(((0.0, 2.0, 0.0),), dtype=np.float32),
        config,
    )

    np.testing.assert_allclose(
        body,
        ((-2.0, 0.0, -0.10),),
        atol=1.0e-6,
    )


def test_voxel_sample_is_bounded_and_keeps_colors_aligned() -> None:
    points = np.array(
        (
            (0.01, 0.01, 0.01),
            (0.02, 0.02, 0.02),
            (0.20, 0.20, 0.20),
        ),
        dtype=np.float32,
    )
    colors = np.array(
        ((10, 20, 30), (40, 50, 60), (70, 80, 90)),
        dtype=np.uint8,
    )

    sampled_points, sampled_colors = voxel_sample(
        points,
        colors,
        voxel_size_m=0.10,
        max_points=2,
    )

    assert len(sampled_points) == 2
    np.testing.assert_array_equal(
        sampled_colors,
        ((10, 20, 30), (70, 80, 90)),
    )


def test_spatial_store_quantizes_a_loss_detectable_frame() -> None:
    store = SpatialFrameStore(max_events=4)
    points = np.array(
        ((1.23, -0.45, 0.06), (-2.0, 3.0, -4.0)),
        dtype=np.float32,
    )
    colors = np.array(((1, 2, 3), (4, 5, 6)), dtype=np.uint8)

    store.publish_frame(
        "lidar",
        points,
        colors,
        input_points=12,
        frame_rate_hz=3.0,
        frame_monotonic_ns=123,
    )
    events, dropped = store.wait_after(0, timeout=0.0)

    assert dropped == 0
    assert len(events) == 1
    event = events[0]
    assert event["source"] == "lidar"
    assert event["point_count"] == 2
    assert event["input_points"] == 12
    assert event["scale_m"] == SPATIAL_QUANTIZATION_M
    quantized = np.frombuffer(
        base64.b64decode(event["points_b64"]), dtype="<i2"
    ).reshape((-1, 3))
    np.testing.assert_array_equal(
        quantized,
        ((123, -45, 6), (-200, 300, -400)),
    )
    np.testing.assert_array_equal(
        np.frombuffer(
            base64.b64decode(event["colors_b64"]), dtype=np.uint8
        ).reshape((-1, 3)),
        colors,
    )
    status = store.snapshot()["sources"]["lidar"]
    assert status["connected"]
    assert status["display_points"] == 2
    assert status["age_ms"] is not None


def test_runtime_spatial_file_round_trip(tmp_path: Path) -> None:
    publisher = SpatialFrameFilePublisher(tmp_path)
    points = np.array(
        ((1.0, -0.5, 0.2), (2.0, 0.3, -0.4)),
        dtype=np.float32,
    )
    colors = np.array(((20, 30, 40), (80, 90, 100)), dtype=np.uint8)
    publisher.publish_frame(
        "lidar",
        points,
        colors,
        input_points=25,
        frame_rate_hz=5.0,
        frame_monotonic_ns=time.monotonic_ns(),
        detail="test shared frame",
    )
    store = SpatialFrameStore(max_events=8)
    stop_event = Event()
    source = SpatialFrameFileSource(
        store,
        stop_event,
        tmp_path,
        stale_timeout_s=1.0,
    )
    source.start()
    deadline = time.monotonic() + 1.0
    while (
        not store.snapshot()["sources"]["lidar"]["connected"]
        and time.monotonic() < deadline
    ):
        time.sleep(0.01)
    stop_event.set()
    source.join(timeout=1.0)

    status = store.snapshot()["sources"]["lidar"]
    assert status["connected"]
    assert status["display_points"] == 2
    events, _dropped = store.wait_after(0, timeout=0.0)
    frame = next(event for event in events if event["kind"] == "frame")
    assert frame["detail"] == "test shared frame"
    assert frame["input_points"] == 25
