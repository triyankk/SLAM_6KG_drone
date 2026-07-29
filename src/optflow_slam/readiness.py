"""Read-only hardware probes and readiness evaluation."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import subprocess
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

    identities: list[str] = []
    matching_serial = False
    try:
        for device in devices:
            name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)
            identities.append(f"{name} serial={serial}")
            if camera.serial is None or serial == camera.serial:
                matching_serial = True
    except RuntimeError as exc:
        return ProbeResult(
            "depth_camera",
            False,
            f"RealSense identity query failed: {exc}",
        )
    if not matching_serial:
        return ProbeResult(
            "depth_camera",
            False,
            (
                f"Configured serial {camera.serial} is absent; detected "
                f"{'; '.join(identities)}"
            ),
        )

    pipeline = rs.pipeline()
    stream_config = rs.config()
    if camera.serial:
        stream_config.enable_device(camera.serial)
    stream_config.enable_stream(
        rs.stream.depth,
        camera.width,
        camera.height,
        rs.format.z16,
        camera.fps,
    )
    stream_config.enable_stream(
        rs.stream.color,
        camera.width,
        camera.height,
        rs.format.rgb8,
        camera.fps,
    )
    started = False
    try:
        pipeline.start(stream_config)
        started = True
        paired_frames = 0
        timeouts = 0
        first_frame_s: float | None = None
        last_frame_s: float | None = None
        startup_deadline_s = time.monotonic() + 5.0
        capture_deadline_s: float | None = None
        while True:
            now_s = time.monotonic()
            if first_frame_s is None and now_s >= startup_deadline_s:
                break
            if (
                capture_deadline_s is not None
                and now_s >= capture_deadline_s
            ):
                break
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                timeouts += 1
                continue
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue
            now_s = time.monotonic()
            if first_frame_s is None:
                first_frame_s = now_s
                capture_deadline_s = now_s + 5.0
            last_frame_s = now_s
            paired_frames += 1

        elapsed_s = (
            0.0
            if first_frame_s is None or last_frame_s is None
            else last_frame_s - first_frame_s
        )
        rate_hz = (
            0.0
            if elapsed_s <= 0.0
            else max(0, paired_frames - 1) / elapsed_s
        )
        minimum_rate_hz = 0.8 * camera.fps
        stable = elapsed_s >= 4.0 and rate_hz >= minimum_rate_hz
        if not stable:
            return ProbeResult(
                "depth_camera",
                False,
                (
                    "D415 RGB-depth stream unstable: "
                    f"frames={paired_frames}, rate={rate_hz:.2f} Hz, "
                    f"timeouts={timeouts}"
                ),
                {
                    "paired_frames": paired_frames,
                    "rate_hz": rate_hz,
                    "timeouts": timeouts,
                },
            )
        return ProbeResult(
            "depth_camera",
            True,
            (
                f"{'; '.join(identities)}; synchronized RGB and depth "
                f"{camera.width}x{camera.height} at {rate_hz:.2f} Hz"
            ),
            {
                "device_count": len(devices),
                "serial": camera.serial,
                "rgb_ready": True,
                "depth_ready": True,
                "paired_frames": paired_frames,
                "rate_hz": rate_hz,
                "timeouts": timeouts,
            },
        )
    except RuntimeError as exc:
        return ProbeResult(
            "depth_camera",
            False,
            f"D415 RGB-depth stream failed: {exc}",
        )
    finally:
        if started:
            try:
                pipeline.stop()
            except RuntimeError:
                pass


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
    interface = Path("/sys/class/net") / lidar.ethernet_interface
    if not interface.exists():
        return ProbeResult(
            "lidar",
            False,
            f"JT16 Ethernet interface is absent: {lidar.ethernet_interface}",
        )
    try:
        carrier = (interface / "carrier").read_text(encoding="ascii").strip()
    except OSError as exc:
        return ProbeResult(
            "lidar",
            False,
            f"Cannot read {lidar.ethernet_interface} carrier: {exc}",
        )
    if carrier != "1":
        return ProbeResult(
            "lidar",
            False,
            (
                f"No Ethernet carrier on {lidar.ethernet_interface}; "
                "connect and power the JT16"
            ),
        )

    try:
        address_result = subprocess.run(
            [
                "ip",
                "-j",
                "address",
                "show",
                "dev",
                lidar.ethernet_interface,
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        address_payload = json.loads(address_result.stdout)
    except (
        FileNotFoundError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
    ) as exc:
        return ProbeResult(
            "lidar",
            False,
            f"Cannot inspect JT16 Ethernet address: {exc}",
        )
    addresses = {
        entry["local"]
        for device in address_payload
        for entry in device.get("addr_info", ())
        if entry.get("family") == "inet" and "local" in entry
    }
    if lidar.jetson_ip not in addresses:
        return ProbeResult(
            "lidar",
            False,
            (
                f"{lidar.ethernet_interface} lacks {lidar.jetson_ip}; "
                "run ./optflow lidar-network"
            ),
            {"addresses": sorted(addresses)},
        )

    try:
        route_result = subprocess.run(
            ["ip", "-j", "route", "get", lidar.lidar_ip],
            check=True,
            capture_output=True,
            text=True,
        )
        route_payload = json.loads(route_result.stdout)
        route_interface = route_payload[0].get("dev")
    except (
        FileNotFoundError,
        IndexError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
    ) as exc:
        return ProbeResult(
            "lidar", False, f"Cannot inspect JT16 route: {exc}"
        )
    if route_interface != lidar.ethernet_interface:
        return ProbeResult(
            "lidar",
            False,
            (
                f"Traffic to {lidar.lidar_ip} uses {route_interface}, "
                f"not {lidar.ethernet_interface}"
            ),
        )

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
    expected_source_seen = lidar.lidar_ip in source_ips
    return ProbeResult(
        "lidar",
        expected_source_seen,
        (
            f"{packet_count} UDP packets from "
            f"{', '.join(sorted(source_ips))}; expected {lidar.lidar_ip}"
        ),
        {
            "packet_count": packet_count,
            "source_ips": sorted(source_ips),
            "expected_source_seen": expected_source_seen,
        },
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
