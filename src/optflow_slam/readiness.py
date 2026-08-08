"""Read-only hardware probes and readiness evaluation."""

from __future__ import annotations

import os
from pathlib import Path
import time

from .config import ProjectConfig
from .models import ProbeResult, Profile, ReadinessReport
from .paths import PROJECT_ROOT


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
            source_system=fc.companion_system_id,
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
    endpoint = Path(lidar.symlink)
    if not endpoint.exists():
        adapter_present = False
        for vendor_path in Path("/sys/bus/usb/devices").glob("*/idVendor"):
            try:
                device_path = vendor_path.parent
                vendor = int(
                    vendor_path.read_text(encoding="ascii").strip(), 16
                )
                product = int(
                    (device_path / "idProduct")
                    .read_text(encoding="ascii")
                    .strip(),
                    16,
                )
                serial_path = device_path / "serial"
                serial_number = (
                    serial_path.read_text(encoding="ascii").strip()
                    if serial_path.exists()
                    else ""
                )
            except (OSError, ValueError):
                continue
            if (
                vendor == lidar.usb_vid
                and product == lidar.usb_pid
                and serial_number == lidar.usb_serial
            ):
                adapter_present = True
                break
        if adapter_present:
            detail = (
                f"{lidar.model} USB-RS485 adapter is present but "
                f"{lidar.symlink} is absent; install the PL2303 driver"
            )
        else:
            detail = (
                f"{lidar.model} USB-RS485 adapter is absent; expected "
                f"{lidar.usb_vid:04x}:{lidar.usb_pid:04x} "
                f"serial={lidar.usb_serial}"
            )
        return ProbeResult(
            "lidar",
            False,
            detail,
            {"adapter_present": adapter_present},
        )

    try:
        import serial
    except ImportError as exc:
        return ProbeResult("lidar", False, f"pyserial unavailable: {exc}")

    baud_candidates = (
        (lidar.baud,)
        if lidar.baud_verified
        else (lidar.baud, lidar.legacy_baud)
    )
    observations: list[dict[str, int]] = []
    for baud in baud_candidates:
        payload = bytearray()
        try:
            with serial.Serial(
                lidar.symlink,
                baudrate=baud,
                timeout=0.05,
                exclusive=True,
            ) as connection:
                connection.reset_input_buffer()
                deadline = time.monotonic() + lidar.packet_probe_s
                while time.monotonic() < deadline:
                    chunk = connection.read(65536)
                    if chunk:
                        payload.extend(chunk)
        except (OSError, serial.SerialException, ValueError) as exc:
            return ProbeResult(
                "lidar",
                False,
                f"Unable to read {lidar.symlink} at {baud}: {exc}",
            )
        observations.append(
            {
                "baud": baud,
                "bytes": len(payload),
                "header_candidates": payload.count(b"\xee\xff"),
            }
        )

    best = max(
        observations,
        key=lambda item: (item["header_candidates"], item["bytes"]),
    )
    bridge_path = Path(lidar.bridge_binary)
    if not bridge_path.is_absolute():
        bridge_path = PROJECT_ROOT / bridge_path
    correction_path = Path(lidar.correction_file)
    if not correction_path.is_absolute():
        correction_path = PROJECT_ROOT / correction_path
    bridge_ready = bridge_path.is_file() and os.access(bridge_path, os.X_OK)
    correction_ready = correction_path.is_file()
    stream_ready = (
        best["header_candidates"] >= 5 and best["bytes"] >= 400
    )
    available = stream_ready and bridge_ready and correction_ready
    if available:
        detail = (
            f"{lidar.model} serial stream at {best['baud']} baud; "
            f"bytes={best['bytes']}; "
            f"packet headers={best['header_candidates']}; "
            "Hesai SDK bridge and correction file ready"
        )
    elif stream_ready:
        missing = []
        if not bridge_ready:
            missing.append("SDK bridge")
        if not correction_ready:
            missing.append("correction file")
        detail = (
            f"{lidar.model} serial bytes are present, but "
            f"{', '.join(missing)} is missing"
        )
    else:
        detail = (
            f"No valid-looking {lidar.model} serial stream on "
            f"{lidar.symlink}; observations={observations}"
        )
    return ProbeResult(
        "lidar",
        available,
        detail,
        {
            "selected_baud": best["baud"] if available else None,
            "observations": observations,
            "framing_only": True,
            "bridge_ready": bridge_ready,
            "bridge_path": str(bridge_path),
            "correction_ready": correction_ready,
            "correction_path": str(correction_path),
            "correction_verified": lidar.correction_verified,
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
