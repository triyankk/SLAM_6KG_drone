import contextlib
from dataclasses import dataclass
from typing import Iterator, Optional

import pyrealsense2 as rs


@dataclass
class RealSenseDeviceInfo:
    name: str
    serial: str
    product_line: str
    has_depth_sensor: bool
    has_color_sensor: bool
    has_motion_sensor: bool


def list_devices() -> list[RealSenseDeviceInfo]:
    context = rs.context()
    devices: list[RealSenseDeviceInfo] = []
    for device in context.devices:
        sensors = list(device.sensors)
        sensor_names = [sensor.get_info(rs.camera_info.name) for sensor in sensors]
        devices.append(
            RealSenseDeviceInfo(
                name=device.get_info(rs.camera_info.name),
                serial=device.get_info(rs.camera_info.serial_number),
                product_line=device.get_info(rs.camera_info.product_line),
                has_depth_sensor=any("Stereo Module" in name or "Depth" in name for name in sensor_names),
                has_color_sensor=any("RGB" in name or "Color" in name for name in sensor_names),
                has_motion_sensor=any("Motion" in name or "Accel" in name or "Gyro" in name for name in sensor_names),
            )
        )
    return devices


@dataclass
class FrameBundle:
    timestamp_ms: float
    depth_frame: Optional[rs.frame]
    infrared_frame: Optional[rs.frame]


@contextlib.contextmanager
def open_depth_pipeline(
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    infrared: bool = True,
) -> Iterator[rs.pipeline]:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    if infrared:
        config.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
    pipeline.start(config)
    try:
        yield pipeline
    finally:
        pipeline.stop()


def wait_for_frame_bundle(pipeline: rs.pipeline, timeout_ms: int = 5000) -> FrameBundle:
    frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
    return FrameBundle(
        timestamp_ms=frames.get_timestamp(),
        depth_frame=frames.get_depth_frame(),
        infrared_frame=frames.get_infrared_frame(1),
    )
