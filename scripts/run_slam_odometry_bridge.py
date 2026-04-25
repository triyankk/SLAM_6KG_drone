#!/usr/bin/env python3

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from intellisense_slam.external_imu import Im10aReader, apply_imu_sample_to_pose
from intellisense_slam.bridge_config import SlamBridgeConfig, load_bridge_config
from intellisense_slam.fc_config import (
    BRIDGE_STATE_JETSON_BOOT,
    BRIDGE_STATE_SLAM_STARTED,
    BRIDGE_STATE_SOURCE_SET_ACTIVE,
    BRIDGE_STATE_SOURCE_SWITCH_FAILED,
    BRIDGE_STATE_SOURCE_SWITCH_NO_ACK,
    FlightControllerTelemetry,
    apply_fc_setup,
    configure_telemetry_streams,
    drain_fc_telemetry,
    publish_bridge_state,
    rangefinder_height_valid,
    request_active_source_set,
    send_distance_sensor,
    send_gcs_event,
    send_gps_input_from_pose,
    send_obstacle_distance,
    send_body_velocity_nudge,
    send_companion_heartbeat,
    send_ready_beeps,
    send_statustext,
    set_ekf_source_set,
)
from intellisense_slam.lidar import LidarReader
from intellisense_slam.mavlink_bridge import connect_to_cube, send_odometry
from intellisense_slam.pose_sources import make_pose_source
from intellisense_slam.qgc_bridge import QgcUdpBridge


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send MAVLink ODOMETRY to the Cube from a pose source. "
            "This is the clean starting point for plugging a real SLAM backend into the drone."
        )
    )
    parser.add_argument("--config", default="")
    parser.add_argument("--ports", nargs="+")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--source", choices=["standby", "hover", "circle", "csv", "vio"])
    parser.add_argument("--csv-path", default=None)
    parser.add_argument("--rate-hz", type=float)
    parser.add_argument("--imu", choices=["on", "off"])
    parser.add_argument("--imu-port", default=None)
    parser.add_argument("--imu-baud", default=None)
    parser.add_argument("--imu-scan-seconds", type=float)
    parser.add_argument("--cube-retry-seconds", type=float)
    parser.add_argument("--reconnect-delay-seconds", type=float)
    parser.add_argument("--standby-log-seconds", type=float)
    parser.add_argument("--status-log-seconds", type=float)
    parser.add_argument("--heartbeat-timeout-seconds", type=float)
    parser.add_argument("--connect-in-standby", choices=["on", "off"])
    parser.add_argument("--fc-setup", choices=["on", "off"])
    parser.add_argument("--fc-source-set", type=int)
    parser.add_argument("--fc-switch-after-sends", type=int)
    parser.add_argument("--fc-select-source", choices=["on", "off"])
    return parser.parse_args()


def resolve_config(args) -> SlamBridgeConfig:
    config = load_bridge_config(args.config) if args.config else SlamBridgeConfig()
    if args.ports is not None:
        config.ports = args.ports
    if args.baud is not None:
        config.baud = args.baud
    if args.source is not None:
        config.source = args.source
    if args.csv_path is not None:
        config.csv_path = args.csv_path
    if args.rate_hz is not None:
        config.rate_hz = args.rate_hz
    if args.imu is not None:
        config.imu_enabled = args.imu == "on"
    if args.imu_port is not None:
        config.imu_port = args.imu_port
    if args.imu_baud is not None:
        config.imu_baud = args.imu_baud
    if args.imu_scan_seconds is not None:
        config.imu_scan_seconds = args.imu_scan_seconds
    if args.cube_retry_seconds is not None:
        config.cube_retry_seconds = args.cube_retry_seconds
    if args.reconnect_delay_seconds is not None:
        config.reconnect_delay_seconds = args.reconnect_delay_seconds
    if args.standby_log_seconds is not None:
        config.standby_log_seconds = args.standby_log_seconds
    if args.status_log_seconds is not None:
        config.status_log_seconds = args.status_log_seconds
    if args.heartbeat_timeout_seconds is not None:
        config.heartbeat_timeout_seconds = args.heartbeat_timeout_seconds
    if args.connect_in_standby is not None:
        config.connect_in_standby = args.connect_in_standby == "on"
    if args.fc_setup is not None:
        config.fc_setup.enabled = args.fc_setup == "on"
    if args.fc_source_set is not None:
        config.fc_setup.slam_source_set = args.fc_source_set
    if args.fc_switch_after_sends is not None:
        config.fc_setup.switch_after_sends = args.fc_switch_after_sends
    if args.fc_select_source is not None:
        config.fc_setup.select_source_set_on_stream = args.fc_select_source == "on"
    return config


def sleep_with_floor(seconds: float) -> None:
    time.sleep(max(seconds, 0.2))


def sleep_until_boot_delay(config: SlamBridgeConfig) -> None:
    if config.boot_delay_seconds <= 0:
        return
    try:
        uptime_s = float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
    except Exception:  # noqa: BLE001
        return
    remaining_s = config.boot_delay_seconds - uptime_s
    if remaining_s > 0:
        print(f"Waiting {remaining_s:.1f}s for Jetson boot sensors to settle before SLAM start.")
        time.sleep(remaining_s)


def connect_to_cube_with_retry(config: SlamBridgeConfig):
    while True:
        try:
            return connect_to_cube(
                config.ports,
                config.baud,
                heartbeat_timeout_s=config.heartbeat_timeout_seconds,
            )
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "Cube connection not ready:"
                f" ports={config.ports}"
                f" baud={config.baud}"
                f" error={exc}"
                f" | retrying in {config.cube_retry_seconds:.1f}s"
            )
            sleep_with_floor(config.cube_retry_seconds)


def open_imu_with_retry(config: SlamBridgeConfig):
    if not config.imu_enabled:
        return None

    while True:
        try:
            return Im10aReader.open(config.imu_port, config.imu_baud, config.imu_scan_seconds)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "External IMU not ready:"
                f" port={config.imu_port}"
                f" baud={config.imu_baud}"
                f" error={exc}"
                f" | retrying in {config.reconnect_delay_seconds:.1f}s"
            )
            sleep_with_floor(config.reconnect_delay_seconds)


def open_lidar_with_retry(config: SlamBridgeConfig):
    if not config.obstacle.enabled or not config.obstacle.lidar_enabled:
        return None

    while True:
        try:
            return LidarReader.open(
                config.obstacle.lidar_port,
                config.obstacle.lidar_baud,
                config.obstacle.sector_count,
                config.obstacle.filter_samples,
                config.obstacle.min_valid_distance_m,
                config.obstacle.max_distance_m,
            )
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(
                "JT lidar not ready:"
                f" port={config.obstacle.lidar_port}"
                f" baud={config.obstacle.lidar_baud}"
                f" error={exc}"
                f" | retrying in {config.reconnect_delay_seconds:.1f}s"
            )
            sleep_with_floor(config.reconnect_delay_seconds)


def open_qgc_bridge(config: SlamBridgeConfig):
    if not config.qgc.enabled:
        return None
    return QgcUdpBridge(
        config.qgc.forward_host,
        config.qgc.forward_port,
        config.qgc.bind_host,
        config.qgc.bind_port,
    )


def close_cube_connection(connection) -> None:
    master = getattr(connection, "master", None)
    if master is not None and hasattr(master, "close"):
        try:
            master.close()
        except Exception:  # noqa: BLE001
            pass


def ensure_fc_setup(connection, config: SlamBridgeConfig) -> None:
    if not config.fc_setup.enabled:
        return

    publish_bridge_state(connection.master, BRIDGE_STATE_JETSON_BOOT, 0)
    report = apply_fc_setup(connection.master, config.fc_setup)
    if report.changed:
        changed_summary = ", ".join(f"{item.name}={item.new_value:g}" for item in report.changed)
        print(
            "Flight controller SLAM setup applied:"
            f" source_set={config.fc_setup.slam_source_set}"
            f" changed={changed_summary}"
        )
    else:
        print(
            "Flight controller SLAM setup already matched:"
            f" source_set={config.fc_setup.slam_source_set}"
        )

    if report.reboot_recommended:
        print(
            "Flight controller reboot is recommended because EKF/visual-odometry"
            " parameters changed on this boot."
        )
    send_gcs_event(
        connection.master,
        f"FC prepared for ExternalNav source {config.fc_setup.slam_source_set}, avoidance margin {config.fc_setup.avoid_margin_m:.1f}m",
    )


def format_fc_position(state: FlightControllerTelemetry) -> str:
    if state.local_position is None:
        return "waiting"
    return (
        f"x={float(getattr(state.local_position, 'x', 0.0)):+.2f}"
        f" y={float(getattr(state.local_position, 'y', 0.0)):+.2f}"
        f" z={float(getattr(state.local_position, 'z', 0.0)):+.2f}"
    )


def mode_wants_slam(fc_state: FlightControllerTelemetry, config: SlamBridgeConfig) -> bool:
    target_mode = config.fc_setup.activate_mode.strip().upper()
    if target_mode in {"", "ANY"}:
        return True
    return fc_state.flight_mode.upper() == target_mode


def apply_cube_rangefinder_height(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
):
    if not config.fc_setup.require_rangefinder_height:
        return pose
    if not rangefinder_height_valid(fc_state):
        return pose

    pose.z_m = -float(fc_state.rangefinder_distance_m)
    pose.tracking_state = f"{pose.tracking_state}+rng"
    if pose.source_name:
        pose.source_name = f"{pose.source_name}+rng"
    else:
        pose.source_name = "rng"
    return pose


def slam_poshold_ready(
    pose,
    fc_state: FlightControllerTelemetry,
    config: SlamBridgeConfig,
) -> bool:
    if not mode_wants_slam(fc_state, config):
        return False
    if fc_state.active_source_set != config.fc_setup.slam_source_set:
        return False
    if pose.pose_quality < config.fc_setup.ready_min_quality:
        return False
    if not pose.tracking_state.startswith("ok"):
        return False
    if config.fc_setup.require_rangefinder_height and not rangefinder_height_valid(fc_state):
        return False
    return True


def maybe_start_flowhold_calibration(fc_state: FlightControllerTelemetry, last_mode: str, running_process):
    if fc_state.flight_mode != "FLOWHOLD" or last_mode == "FLOWHOLD" or running_process is not None:
        return running_process

    script = REPO_ROOT / "scripts" / "run_slam_sensor_calibration.py"
    if not script.exists():
        return None
    return subprocess.Popen(  # noqa: S603
        [sys.executable, str(script), "--duration", "12"],
        cwd=str(REPO_ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def compute_lidar_body_nudge(snapshot, config):
    distance_m = snapshot.filtered_distance_m or snapshot.min_distance_m
    if distance_m <= 0.0 or distance_m >= config.trigger_distance_m:
        return None
    angle_rad = math.radians(snapshot.min_azimuth_deg + config.angle_offset_deg)
    strength = min(
        config.max_speed_m_s,
        config.max_speed_m_s * (config.trigger_distance_m - distance_m) / max(config.trigger_distance_m, 0.1),
    )
    return -math.cos(angle_rad) * strength, -math.sin(angle_rad) * strength, distance_m


def run_standby(config: SlamBridgeConfig) -> None:
    print(
        "SLAM bridge standby mode active."
        f" config_source={config.source}"
        f" ports={config.ports}"
        f" imu={'on' if config.imu_enabled else 'off'}"
    )
    print(
        "Standby mode is intentionally safe: the service stays alive, but it does not send "
        "ODOMETRY until a real pose source is configured and the service is restarted."
    )

    if not config.connect_in_standby:
        while True:
            print(
                "Standby heartbeat:"
                f" cube_ports={config.ports}"
                f" imu_port={config.imu_port}"
                f" rate={config.rate_hz:.1f}Hz"
            )
            sleep_with_floor(config.standby_log_seconds)

    while True:
        connection = connect_to_cube_with_retry(config)
        imu_reader = open_imu_with_retry(config)
        lidar_reader = open_lidar_with_retry(config)
        qgc_bridge = open_qgc_bridge(config)
        send_companion_heartbeat(connection.master)
        ensure_fc_setup(connection, config)
        send_gcs_event(connection.master, "standby bridge linked; no ODOMETRY is being sent")
        last_vehicle_heartbeat_s = time.time()
        last_companion_heartbeat_s = 0.0
        last_status_s = 0.0
        print(
            "Standby link bound:"
            f" cube_port={connection.port}"
            f" cube_baud={connection.baud}"
            f" imu={'off' if imu_reader is None else f'{imu_reader.port}@{imu_reader.baud}'}"
            f" lidar={'off' if lidar_reader is None else f'{lidar_reader.port}@{lidar_reader.baud}'}"
            f" qgc={'off' if qgc_bridge is None else f'udpout:{config.qgc.forward_host}:{config.qgc.forward_port}/udpin:{config.qgc.bind_port}'}"
            f" slam_source_set={config.fc_setup.slam_source_set}"
        )

        try:
            while True:
                try:
                    msg = connection.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"Serial read failed on standby link: {exc}") from exc
                now_s = time.time()
                if msg is not None and int(msg.get_srcSystem()) > 0 and int(msg.get_srcSystem()) != 255:
                    last_vehicle_heartbeat_s = now_s
                if now_s - last_companion_heartbeat_s >= 1.0:
                    send_companion_heartbeat(connection.master)
                    last_companion_heartbeat_s = now_s
                if qgc_bridge is not None:
                    qgc_bridge.forward_downlink(msg)
                    qgc_bridge.forward_uplink_to_cube(connection.master)

                imu_summary = "off"
                if imu_reader is not None:
                    imu_sample = imu_reader.poll(duration_s=0.01)
                    if imu_sample is not None:
                        imu_summary = (
                            f"{imu_reader.port} "
                            f"rpy=({imu_sample.roll_deg:+.1f},{imu_sample.pitch_deg:+.1f},{imu_sample.yaw_deg:+.1f})"
                        )
                    else:
                        imu_summary = f"{imu_reader.port} waiting"
                lidar_summary = "off"
                if lidar_reader is not None:
                    lidar = lidar_reader.poll(duration_s=0.01)
                    lidar_summary = (
                        f"{lidar_reader.port} raw={lidar.min_distance_m:.2f}m"
                        f" filt={lidar.filtered_distance_m:.2f}m"
                    )

                if now_s - last_status_s >= max(config.standby_log_seconds, 1.0):
                    print(
                        "Standby link:"
                        f" cube_port={connection.port}"
                        f" cube_baud={connection.baud}"
                        f" hb_age_s={now_s - last_vehicle_heartbeat_s:.1f}"
                        f" imu={imu_summary}"
                        f" lidar={lidar_summary}"
                    )
                    last_status_s = now_s
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(f"Standby link interrupted: {exc} | reconnecting in {config.reconnect_delay_seconds:.1f}s")
            sleep_with_floor(config.reconnect_delay_seconds)
        finally:
            if imu_reader is not None:
                imu_reader.close()
            if lidar_reader is not None:
                lidar_reader.close()
            if qgc_bridge is not None:
                qgc_bridge.close()
            close_cube_connection(connection)


def run_bridge(config: SlamBridgeConfig) -> None:
    source = make_pose_source(config.source, config.csv_path)
    period_s = 1.0 / max(config.rate_hz, 0.1)
    sent_count = 0
    started_s = time.time()
    last_status_s = 0.0

    while True:
        connection = connect_to_cube_with_retry(config)
        imu_reader = open_imu_with_retry(config)
        lidar_reader = open_lidar_with_retry(config)
        qgc_bridge = open_qgc_bridge(config)
        send_companion_heartbeat(connection.master)
        ensure_fc_setup(connection, config)
        send_gcs_event(connection.master, "bridge linked; waiting for SLAM odometry readiness")
        configure_telemetry_streams(connection.master)
        fc_state = FlightControllerTelemetry(active_source_set=request_active_source_set(connection.master))
        last_switch_attempt_s = 0.0
        last_release_attempt_s = 0.0
        last_heartbeat_s = 0.0
        last_obstacle_publish_s = 0.0
        gps_input_ready_announced = False
        gps_input_origin_missing_announced = False
        steering_inside = False
        obstacle_inside = False
        slam_ready_last = False
        last_mode = fc_state.flight_mode
        calibration_process = None
        source_started_published = False
        print(
            "Sending ODOMETRY to Cube:"
            f" port={connection.port}"
            f" baud={connection.baud}"
            f" source={config.source}"
            f" rate={config.rate_hz:.1f}Hz"
            f" imu={'off' if imu_reader is None else f'{imu_reader.port}@{imu_reader.baud}'}"
            f" lidar={'off' if lidar_reader is None else f'{lidar_reader.port}@{lidar_reader.baud}'}"
            f" qgc={'off' if qgc_bridge is None else f'udpout:{config.qgc.forward_host}:{config.qgc.forward_port}/udpin:{config.qgc.bind_port}'}"
            f" slam_source_set={config.fc_setup.slam_source_set}"
        )

        try:
            while True:
                loop_started_s = time.time()
                drain_fc_telemetry(connection.master, fc_state, qgc_bridge)
                if qgc_bridge is not None:
                    qgc_bridge.forward_uplink_to_cube(connection.master)

                if hasattr(source, "set_external_height_m"):
                    source.set_external_height_m(
                        fc_state.rangefinder_distance_m if rangefinder_height_valid(fc_state) else None
                    )
                pose = source.sample()
                if imu_reader is not None:
                    imu_sample = imu_reader.poll(duration_s=min(0.02, period_s))
                    if imu_sample is not None:
                        pose = apply_imu_sample_to_pose(pose, imu_sample)
                pose = apply_cube_rangefinder_height(pose, fc_state, config)

                slam_mode_requested = mode_wants_slam(fc_state, config)
                if not slam_mode_requested and sent_count > 0:
                    sent_count = 0
                    source_started_published = False
                    gps_input_ready_announced = False
                    gps_input_origin_missing_announced = False

                if slam_mode_requested:
                    send_odometry(connection, pose)
                if config.gps_input.enabled and slam_mode_requested:
                    sent_gps_input = send_gps_input_from_pose(connection.master, pose, config.gps_input)
                    if sent_gps_input and not gps_input_ready_announced:
                        send_gcs_event(
                            connection.master,
                            f"VIO mirrored to GPS{config.gps_input.gps_id + 1} GPS_INPUT",
                        )
                        gps_input_ready_announced = True
                    elif not sent_gps_input and not gps_input_origin_missing_announced:
                        send_gcs_event(
                            connection.master,
                            "GPS_INPUT disabled: set gps_input origin lat/lon first",
                            severity=4,
                        )
                        gps_input_origin_missing_announced = True
                if slam_mode_requested:
                    sent_count += 1
                now_s = time.time()
                if now_s - last_heartbeat_s >= 1.0:
                    send_companion_heartbeat(connection.master)
                    last_heartbeat_s = now_s
                if slam_mode_requested and not source_started_published:
                    publish_bridge_state(
                        connection.master,
                        BRIDGE_STATE_SLAM_STARTED,
                        0,
                    )
                    send_gcs_event(connection.master, "SLAM odometry stream started")
                    source_started_published = True

                if lidar_reader is not None and now_s - last_obstacle_publish_s >= 1.0 / max(config.obstacle.publish_rate_hz, 0.1):
                    lidar = lidar_reader.poll(duration_s=0.0)
                    obstacle_distance_m = lidar.filtered_distance_m or lidar.min_distance_m
                    send_distance_sensor(
                        connection.master,
                        obstacle_distance_m,
                        config.obstacle.sensor_id,
                        config.obstacle.max_distance_m,
                    )
                    send_obstacle_distance(
                        connection.master,
                        lidar.sector_distances_m,
                        config.obstacle.max_distance_m,
                    )
                    if 0.0 < obstacle_distance_m < config.obstacle.safety_distance_m and not obstacle_inside:
                        send_gcs_event(
                            connection.master,
                            f"obstacle {obstacle_distance_m:.2f}m; keep >= {config.obstacle.safety_distance_m:.1f}m",
                            severity=4,
                        )
                        obstacle_inside = True
                    elif obstacle_inside and obstacle_distance_m >= config.obstacle.safety_distance_m:
                        send_gcs_event(connection.master, f"obstacle cleared {obstacle_distance_m:.2f}m")
                        obstacle_inside = False
                    last_obstacle_publish_s = now_s

                    if config.lidar_steering.enabled:
                        nudge = compute_lidar_body_nudge(lidar, config.lidar_steering)
                        mode_allowed = fc_state.flight_mode.upper() in config.lidar_steering.allowed_modes
                        if nudge is not None and mode_allowed:
                            vx_m_s, vy_m_s, nudge_distance_m = nudge
                            send_body_velocity_nudge(connection.master, vx_m_s, vy_m_s)
                            if not steering_inside:
                                send_gcs_event(
                                    connection.master,
                                    f"lidar XY nudge active {nudge_distance_m:.2f}m vx={vx_m_s:.2f} vy={vy_m_s:.2f}",
                                    severity=4,
                                )
                                steering_inside = True
                        elif steering_inside and (
                            nudge is None
                            or (lidar.filtered_distance_m or lidar.min_distance_m) >= config.lidar_steering.clear_distance_m
                        ):
                            send_gcs_event(connection.master, "lidar XY nudge clear")
                            steering_inside = False

                if fc_state.flight_mode != last_mode:
                    send_gcs_event(connection.master, f"flight mode {fc_state.flight_mode}")
                    calibration_process = maybe_start_flowhold_calibration(fc_state, last_mode, calibration_process)
                    if calibration_process is not None and fc_state.flight_mode == "FLOWHOLD":
                        send_gcs_event(connection.master, "FLOWHOLD triggered sensor calibration capture")
                    if fc_state.flight_mode == "POSHOLD":
                        send_gcs_event(connection.master, "POSHOLD selected; waiting for SLAM readiness")
                    last_mode = fc_state.flight_mode
                if calibration_process is not None and calibration_process.poll() is not None:
                    send_gcs_event(connection.master, "sensor calibration capture finished")
                    calibration_process = None

                should_switch = (
                    config.fc_setup.enabled
                    and config.fc_setup.select_source_set_on_stream
                    and config.fc_setup.slam_source_set > 0
                    and slam_mode_requested
                    and sent_count >= max(config.fc_setup.switch_after_sends, 1)
                    and fc_state.active_source_set != config.fc_setup.slam_source_set
                )
                if should_switch and time.time() - last_switch_attempt_s >= 2.0:
                    last_switch_attempt_s = time.time()
                    switch_result = set_ekf_source_set(connection.master, config.fc_setup.slam_source_set)
                    if switch_result is True:
                        fc_state.active_source_set = config.fc_setup.slam_source_set
                        print(
                            "EKF source set switched for SLAM:"
                            f" active={config.fc_setup.slam_source_set}"
                            f" after_sends={sent_count}"
                        )
                        send_statustext(
                            connection.master,
                            f"Jetson SLAM source {config.fc_setup.slam_source_set} active",
                        )
                        publish_bridge_state(
                            connection.master,
                            BRIDGE_STATE_SOURCE_SET_ACTIVE,
                            config.fc_setup.slam_source_set,
                        )
                    elif switch_result is False:
                        print(
                            "EKF source set switch was rejected by the flight controller:"
                            f" target={config.fc_setup.slam_source_set}"
                        )
                        publish_bridge_state(
                            connection.master,
                            BRIDGE_STATE_SOURCE_SWITCH_FAILED,
                            0,
                        )
                    else:
                        print(
                            "EKF source set switch was sent but no ACK arrived:"
                            f" target={config.fc_setup.slam_source_set}"
                        )
                        publish_bridge_state(
                            connection.master,
                            BRIDGE_STATE_SOURCE_SWITCH_NO_ACK,
                            0,
                        )

                should_release = (
                    config.fc_setup.enabled
                    and config.fc_setup.select_source_set_on_stream
                    and config.fc_setup.idle_source_set > 0
                    and not slam_mode_requested
                    and fc_state.active_source_set == config.fc_setup.slam_source_set
                )
                if should_release and time.time() - last_release_attempt_s >= 2.0:
                    last_release_attempt_s = time.time()
                    release_result = set_ekf_source_set(connection.master, config.fc_setup.idle_source_set)
                    if release_result is True:
                        fc_state.active_source_set = config.fc_setup.idle_source_set
                        send_gcs_event(
                            connection.master,
                            f"SLAM released; source {config.fc_setup.idle_source_set} restored",
                        )

                slam_ready_now = slam_poshold_ready(pose, fc_state, config)
                if slam_ready_now and not slam_ready_last:
                    send_ready_beeps(connection.master)
                    send_gcs_event(connection.master, "GPS-less PosHold ready")
                slam_ready_last = slam_ready_now

                if now_s - last_status_s >= max(config.status_log_seconds, 1.0):
                    elapsed_s = max(now_s - started_s, 1e-6)
                    extra = ""
                    if fc_state.status_text:
                        extra += f" status={fc_state.status_text}"
                    if fc_state.ekf_flags is not None:
                        extra += f" ekf_flags={fc_state.ekf_flags}"
                    if fc_state.rangefinder_distance_m is not None:
                        extra += f" rng={fc_state.rangefinder_distance_m:.2f}m"
                    print(
                        "SLAM bridge:"
                        f" source={config.source}"
                        f" sent={sent_count}"
                        f" rate={sent_count / elapsed_s:.1f}/s"
                        f" port={connection.port}"
                        f" imu={'off' if imu_reader is None else imu_reader.port}"
                        f" lidar={'off' if lidar_reader is None else f'raw={lidar_reader.snapshot.min_distance_m:.2f}m/filt={lidar_reader.snapshot.filtered_distance_m:.2f}m'}"
                        f" fc_source={fc_state.active_source_set or 'unknown'}"
                        f" mode={fc_state.flight_mode}"
                        f" slam_mode={'on' if slam_mode_requested else 'off'}"
                        f" ready={'yes' if slam_ready_now else 'no'}"
                        f" pose_q={pose.pose_quality}"
                        f" track={pose.tracking_state}"
                        f" fc_pos={format_fc_position(fc_state)}"
                        f"{extra}"
                    )
                    last_status_s = now_s

                remaining_s = period_s - (time.time() - loop_started_s)
                if remaining_s > 0:
                    time.sleep(remaining_s)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            print(f"Bridge loop interrupted: {exc} | reconnecting in {config.reconnect_delay_seconds:.1f}s")
            sleep_with_floor(config.reconnect_delay_seconds)
        finally:
            if imu_reader is not None:
                imu_reader.close()
            if lidar_reader is not None:
                lidar_reader.close()
            if qgc_bridge is not None:
                qgc_bridge.close()
            close_cube_connection(connection)


def main():
    config = resolve_config(parse_args())
    sleep_until_boot_delay(config)
    if config.source == "standby":
        run_standby(config)
        return
    run_bridge(config)


if __name__ == "__main__":
    main()
