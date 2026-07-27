"""Read-only hardware probes and readiness evaluation."""

from __future__ import annotations

from pathlib import Path
import socket
import time

from .config import ProjectConfig
from .models import ProbeResult, Profile, ReadinessReport


REQUIRED_BY_PROFILE = {
    Profile.FC_BENCH: frozenset({"cube_hflow"}),
    Profile.SLAM_BENCH: frozenset(
        {"cube_hflow", "depth_camera", "external_imu", "lidar"}
    ),
    Profile.NAVIGATION: frozenset(
        {
            "cube_hflow",
            "depth_camera",
            "external_imu",
            "lidar",
            "calibration",
            "control_enable",
            "rc_disarm",
        }
    ),
}


def _set_message_interval(master, message_id: int, interval_us: int) -> None:
    from pymavlink import mavutil

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def probe_cube_hflow(config: ProjectConfig) -> ProbeResult:
    fc = config.flight_controller
    endpoint = Path(fc.endpoint)
    if not endpoint.exists():
        return ProbeResult(
            "cube_hflow", False, f"Cube UART is absent: {fc.endpoint}"
        )

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        return ProbeResult("cube_hflow", False, f"pymavlink unavailable: {exc}")

    master = None
    message_ids = (
        mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
    )
    try:
        master = mavutil.mavlink_connection(
            fc.endpoint,
            baud=fc.baud,
            source_system=fc.system_id,
            source_component=fc.companion_component_id,
        )
        deadline = time.monotonic() + fc.heartbeat_timeout_s
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = master.recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.5
            )
            if candidate is None:
                continue
            if (
                candidate.autopilot
                != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue
            heartbeat = candidate
            break
        if heartbeat is None:
            return ProbeResult(
                "cube_hflow", False, "Cube heartbeat timed out on UART"
            )

        master.target_system = heartbeat.get_srcSystem()
        master.target_component = heartbeat.get_srcComponent()
        armed = bool(
            heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if armed:
            return ProbeResult(
                "cube_hflow",
                False,
                "Cube is armed; refusing the active bench probe",
            )

        for message_id in message_ids:
            _set_message_interval(master, message_id, 200_000)

        flow_count = 0
        range_count = 0
        quality = 0
        distance_cm = 0
        deadline = time.monotonic() + fc.sample_window_s
        while time.monotonic() < deadline:
            message = master.recv_match(blocking=True, timeout=0.4)
            if message is None:
                continue
            message_type = message.get_type()
            if message_type == "OPTICAL_FLOW":
                flow_count += 1
                quality = int(message.quality)
            elif (
                message_type == "DISTANCE_SENSOR"
                and int(message.orientation) == 25
            ):
                range_count += 1
                distance_cm = int(message.current_distance)

        available = (
            flow_count > 0
            and range_count > 0
            and quality >= fc.hflow_min_bench_quality
            and distance_cm > 0
        )
        detail = (
            f"Cube sys={master.target_system} comp={master.target_component}; "
            f"flow={flow_count} quality={quality}; "
            f"range={range_count} distance={distance_cm} cm"
        )
        return ProbeResult(
            "cube_hflow",
            available,
            detail,
            {
                "flow_samples": flow_count,
                "flow_quality": quality,
                "range_samples": range_count,
                "distance_cm": distance_cm,
            },
        )
    except (OSError, RuntimeError, ValueError) as exc:
        return ProbeResult("cube_hflow", False, f"Cube probe failed: {exc}")
    finally:
        if master is not None:
            for message_id in message_ids:
                try:
                    _set_message_interval(master, message_id, -1)
                except Exception:
                    pass
            try:
                master.close()
            except Exception:
                pass


def probe_depth_camera(config: ProjectConfig) -> ProbeResult:
    camera = config.depth_camera
    if camera.backend.lower() != "realsense":
        return ProbeResult(
            "depth_camera",
            False,
            f"Unsupported depth-camera backend: {camera.backend}",
        )
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        return ProbeResult(
            "depth_camera", False, f"RealSense SDK unavailable: {exc}"
        )

    try:
        devices = list(rs.context().query_devices())
    except RuntimeError as exc:
        return ProbeResult(
            "depth_camera", False, f"RealSense enumeration failed: {exc}"
        )
    if not devices:
        return ProbeResult(
            "depth_camera", False, "RealSense SDK is installed; no camera detected"
        )

    identities = []
    for device in devices:
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        identities.append(f"{name} serial={serial}")
    return ProbeResult(
        "depth_camera",
        True,
        "; ".join(identities),
        {"device_count": len(devices)},
    )


def probe_external_imu(config: ProjectConfig) -> ProbeResult:
    imu = config.external_imu
    symlink = Path(imu.symlink)
    if symlink.exists():
        return ProbeResult(
            "external_imu",
            True,
            f"{imu.model} serial device present at {imu.symlink}",
        )

    try:
        from serial.tools import list_ports
    except ImportError as exc:
        return ProbeResult(
            "external_imu", False, f"pyserial unavailable: {exc}"
        )

    matches = [
        port.device
        for port in list_ports.comports()
        if port.vid == imu.usb_vid and port.pid == imu.usb_pid
    ]
    if matches:
        return ProbeResult(
            "external_imu",
            True,
            f"{imu.model} detected at {', '.join(matches)}; stable symlink absent",
        )
    return ProbeResult(
        "external_imu",
        False,
        (
            f"{imu.model} not detected "
            f"(USB {imu.usb_vid:04x}:{imu.usb_pid:04x})"
        ),
    )


def probe_lidar(config: ProjectConfig) -> ProbeResult:
    lidar = config.lidar
    packet_count = 0
    source_ips: set[str] = set()
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.settimeout(0.1)
    try:
        listener.bind(("0.0.0.0", lidar.udp_port))
        deadline = time.monotonic() + lidar.packet_probe_s
        while time.monotonic() < deadline:
            try:
                _, source = listener.recvfrom(65535)
            except socket.timeout:
                continue
            packet_count += 1
            source_ips.add(source[0])
    except OSError as exc:
        return ProbeResult(
            "lidar",
            False,
            f"Unable to listen on UDP {lidar.udp_port}: {exc}",
        )
    finally:
        listener.close()

    if packet_count == 0:
        verification = (
            "verified"
            if lidar.network_values_verified
            else "provisional and must be verified"
        )
        return ProbeResult(
            "lidar",
            False,
            (
                f"No {lidar.model} packets on UDP {lidar.udp_port}; "
                f"IP plan {lidar.lidar_ip} -> {lidar.jetson_ip} is {verification}"
            ),
        )
    return ProbeResult(
        "lidar",
        True,
        (
            f"{packet_count} UDP packets from "
            f"{', '.join(sorted(source_ips))}"
        ),
        {"packet_count": packet_count, "source_ips": sorted(source_ips)},
    )


def probe_calibration(config: ProjectConfig) -> ProbeResult:
    calibration = config.calibration
    fields = (
        "camera_intrinsics_verified",
        "camera_to_body_extrinsics_verified",
        "imu_to_body_extrinsics_verified",
        "lidar_to_body_extrinsics_verified",
        "imu_noise_profile_verified",
        "sensor_time_sync_verified",
    )
    missing = [name for name in fields if not getattr(calibration, name)]
    if missing:
        return ProbeResult(
            "calibration",
            False,
            "Unverified: " + ", ".join(missing),
        )
    return ProbeResult("calibration", True, "All calibration gates are marked verified")


def probe_control_enable(config: ProjectConfig) -> ProbeResult:
    nav = config.navigation
    if not nav.autonomous_control_enabled:
        return ProbeResult(
            "control_enable",
            False,
            "Autonomous command output is intentionally disabled",
        )
    if not nav.external_nav_to_cube_enabled:
        return ProbeResult(
            "control_enable",
            True,
            "Bounded setpoint control enabled; Cube ExternalNav remains disabled",
        )
    return ProbeResult(
        "control_enable",
        True,
        "Bounded setpoint control and Cube ExternalNav are enabled",
    )


def probe_rc_disarm(config: ProjectConfig) -> ProbeResult:
    if config.safety.rc_disarm_switch_configured:
        return ProbeResult("rc_disarm", True, "RC disarm switch marked configured")
    return ProbeResult(
        "rc_disarm",
        False,
        "RC disarm switch is not configured; assignment requires operator channel input",
    )


def run_readiness(config: ProjectConfig, profile: Profile) -> ReadinessReport:
    results = (
        probe_cube_hflow(config),
        probe_depth_camera(config),
        probe_external_imu(config),
        probe_lidar(config),
        probe_calibration(config),
        probe_control_enable(config),
        probe_rc_disarm(config),
    )
    return ReadinessReport(
        profile=profile,
        results=results,
        required_names=REQUIRED_BY_PROFILE[profile],
    )
