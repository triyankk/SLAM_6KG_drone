import importlib
import os
from dataclasses import dataclass
from typing import List


@dataclass
class ReadinessReport:
    dependencies_ok: bool
    realsense_found: bool
    realsense_has_imu: bool
    lidar_serial_present: bool
    external_imu_usb_present: bool
    external_imu_port_present: bool
    external_imu_stream_healthy: bool
    summary: str
    details: List[str]


def dependency_available(module_name: str) -> bool:
    try:
        importlib.import_module(module_name)
        return True
    except Exception:  # noqa: BLE001
        return False


def jt16_serial_present() -> bool:
    try:
        import serial.tools.list_ports
    except Exception:  # noqa: BLE001
        return any(os.path.exists(path) for path in ("/dev/jt16_usb", "/dev/ttyUSB0", "/dev/ttyUSB1"))

    for port in serial.tools.list_ports.comports():
        if port.vid == 0x067B and port.pid == 0x23A3 and port.device.startswith("/dev/ttyUSB"):
            return True
    return False


def build_readiness_report() -> ReadinessReport:
    deps = {
        "pyrealsense2": dependency_available("pyrealsense2"),
        "pymavlink": dependency_available("pymavlink"),
        "numpy": dependency_available("numpy"),
        "cv2": dependency_available("cv2"),
        "yaml": dependency_available("yaml"),
        "serial": dependency_available("serial"),
    }
    if deps["pyrealsense2"]:
        from .realsense_capture import list_devices

        devices = list_devices()
    else:
        devices = []
    realsense_found = bool(devices)
    realsense_has_imu = any(device.has_motion_sensor for device in devices)
    lidar_serial_present = jt16_serial_present()
    if deps["serial"]:
        from .external_imu import collect_imu_health

        imu_health = collect_imu_health(scan_seconds=0.6)
    else:
        imu_health = None

    details = []
    details.extend([f"dependency {name}: {'ok' if ok else 'missing'}" for name, ok in deps.items()])
    if not devices:
        details.append("realsense: no device detected")
    else:
        for device in devices:
            details.append(
                "realsense: "
                f"{device.name} serial={device.serial} "
                f"product_line={device.product_line} "
                f"depth={'yes' if device.has_depth_sensor else 'no'} "
                f"motion={'yes' if device.has_motion_sensor else 'no'}"
            )
    details.append(f"jt16 serial node present: {'yes' if lidar_serial_present else 'no'}")
    details.append(f"external imu usb present: {'yes' if imu_health and imu_health.usb_present else 'no'}")
    details.append(f"external imu port present: {'yes' if imu_health and bool(imu_health.port) else 'no'}")
    details.append(f"external imu stream healthy: {'yes' if imu_health and imu_health.stream_healthy else 'no'}")
    details.append(
        "external imu detail: "
        f"{imu_health.message if imu_health is not None else 'pyserial missing, so IMU probing was skipped.'}"
    )

    dependencies_ok = all(deps.values())
    if dependencies_ok and realsense_found and (realsense_has_imu or (imu_health and imu_health.stream_healthy)):
        summary = "Good candidate for real VIO/SLAM integration work."
    elif dependencies_ok and realsense_found:
        summary = (
            "Good candidate for bridge and prototype work, but not yet full flight-grade SLAM. "
            "Camera, lidar, and external-nav pieces are present, but backend fusion and calibration still remain."
        )
    else:
        summary = "Not ready yet. Resolve missing dependencies or missing sensors first."

    return ReadinessReport(
        dependencies_ok=dependencies_ok,
        realsense_found=realsense_found,
        realsense_has_imu=realsense_has_imu,
        lidar_serial_present=lidar_serial_present,
        external_imu_usb_present=bool(imu_health and imu_health.usb_present),
        external_imu_port_present=bool(imu_health and imu_health.port),
        external_imu_stream_healthy=bool(imu_health and imu_health.stream_healthy),
        summary=summary,
        details=details,
    )
