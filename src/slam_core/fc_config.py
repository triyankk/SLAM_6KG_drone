"""Flight-controller MAVLink helpers for the Jetson SLAM bridge.

This module is intentionally the one place that knows how to talk to ArduPilot:
parameter writes, telemetry parsing, GPS_INPUT messages, beeps, STATUSTEXT, and
mode/source-set commands all live here. Keep flight-controller side effects
centralized so the rest of the codebase can stay easier to reason about.
"""

import os
import math
import time
from dataclasses import dataclass, field
from typing import Any

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


@dataclass
class FlightControllerSetupConfig:
    """ArduPilot parameter plan used by the bridge at startup.

    The active field-test profile currently uses GPS2 MAVLink input instead of
    ArduPilot Visual Odometry because the target Cube was rejecting VisOdom. In
    that mode `viso_type` stays 0 and `gps2_type` is 14.
    """

    enabled: bool = True
    slam_source_set: int = 3
    idle_source_set: int = 1
    switch_after_sends: int = 30
    select_source_set_on_stream: bool = True
    activate_mode: str = "POSHOLD"
    ready_min_quality: int = 60
    require_rangefinder_height: bool = True
    ahrs_ekf_type: int = 3
    ek2_enable: int = 0
    ek3_enable: int = 1
    ek3_src_options: int = 0
    ek3_ogn_hgt_mask: int = 0
    viso_type: int = 0
    viso_pos_x_m: float = 0.0
    viso_pos_y_m: float = 0.0
    viso_pos_z_m: float = 0.0
    viso_qual_min: int = 0
    posxy_source: int = 6
    velxy_source: int = 6
    posz_source: int = 1
    velz_source: int = 0
    yaw_source: int = 1
    avoid_enable: int = 7
    avoid_margin_m: float = 2.0
    prx1_type: int = 2
    gps2_type: int | None = None
    gps_auto_switch: int | None = None


@dataclass
class ParameterChange:
    name: str
    old_value: float | None
    new_value: float
    changed: bool


@dataclass
class FlightControllerSetupReport:
    changed: list[ParameterChange] = field(default_factory=list)
    unchanged: list[ParameterChange] = field(default_factory=list)
    reboot_recommended: bool = False


@dataclass
class FlightControllerTelemetry:
    """Latest MAVLink state drained from the Cube.

    The bridge keeps this as a simple mutable snapshot. Every control gate reads
    from this object, so fields should be conservative: if a message has not
    arrived yet, leave the value as None instead of inventing a healthy default.
    """

    local_position: Any | None = None
    attitude: Any | None = None
    ekf_flags: int | None = None
    status_text: str = ""
    status_severity: int | None = None
    status_last_update_s: float = 0.0
    active_source_set: int | None = None
    flight_mode: str = "UNKNOWN"
    armed: bool = False
    last_heartbeat_s: float = 0.0
    gps_fix_type: int | None = None
    gps_satellites: int | None = None
    gps_lat: int | None = None
    gps_lon: int | None = None
    gps_alt_mm: int | None = None
    gps_vel_cm_s: int | None = None
    gps_cog_cd: int | None = None
    gps_time_usec: int | None = None
    gps2_fix_type: int | None = None
    gps2_satellites: int | None = None
    gps2_lat: int | None = None
    gps2_lon: int | None = None
    gps2_alt_mm: int | None = None
    gps2_time_usec: int | None = None
    global_lat: int | None = None
    global_lon: int | None = None
    global_alt_mm: int | None = None
    global_relative_alt_mm: int | None = None
    global_vx_cm_s: int | None = None
    global_vy_cm_s: int | None = None
    global_vz_cm_s: int | None = None
    global_hdg_cd: int | None = None
    vfr_alt_m: float | None = None
    vfr_climb_m_s: float | None = None
    vfr_groundspeed_m_s: float | None = None
    vfr_throttle_pct: int | None = None
    baro_pressure_hpa: float | None = None
    baro_alt_m: float | None = None
    rc_channel_count: int = 0
    rc_rssi: int | None = None
    rc_last_update_s: float = 0.0
    rc_channels: dict[int, int] = field(default_factory=dict)
    rangefinder_distance_m: float | None = None
    rangefinder_sensor_id: int | None = None
    rangefinder_last_update_s: float = 0.0
    landed_state: int | None = None
    landed_state_last_update_s: float = 0.0
    battery_remaining_pct: int | None = None
    battery_voltage_v: float | None = None
    battery_last_update_s: float = 0.0


# These SCR_USER* params are a tiny status bus for Lua scripts running on the
# Cube. The beeper script can react to them without parsing the companion log.
BRIDGE_STATE_PARAM = "SCR_USER1"
BRIDGE_SOURCE_SET_PARAM = "SCR_USER2"
BRIDGE_HEARTBEAT_PARAM = "SCR_USER3"
BRIDGE_STATE_IDLE = 0
BRIDGE_STATE_JETSON_BOOT = 10
BRIDGE_STATE_SENSOR_CHECK_PASSED = 12
BRIDGE_STATE_SLAM_STARTED = 40
BRIDGE_STATE_SOURCE_SET_ACTIVE = 50
BRIDGE_STATE_POSHOLD_READY = 54
BRIDGE_STATE_CALIBRATION_WAITING_ARM = 68
BRIDGE_STATE_CALIBRATION_WAITING_TAKEOFF = 69
BRIDGE_STATE_CALIBRATION_ACTIVE = 70
BRIDGE_STATE_CALIBRATION_COMPLETE_RTL = 71
BRIDGE_STATE_SLAM_FLIGHT_ACTIVE = 72
BRIDGE_STATE_SOURCE_SWITCH_FAILED = 82
BRIDGE_STATE_SOURCE_SWITCH_NO_ACK = 83

REBOOT_RECOMMENDED_PARAMS = {
    "AHRS_EKF_TYPE",
    "EK2_ENABLE",
    "EK3_ENABLE",
    "EK3_OGN_HGT_MASK",
    "VISO_TYPE",
    "GPS2_TYPE",
}
OPTIONAL_PARAMS = {
    "EK2_ENABLE",
    "AVOID_ENABLE",
    "AVOID_MARGIN",
    "PRX1_TYPE",
    "GPS2_TYPE",
    "GPS_AUTO_SWITCH",
}

GPS_IDLE_SOURCE_PARAMS = {
    "POSXY": 3.0,  # GPS horizontal position
    "VELXY": 3.0,  # GPS horizontal velocity
    "POSZ": 1.0,  # Barometer altitude
    "VELZ": 0.0,  # No vertical velocity source
    "YAW": 1.0,  # Compass yaw
}

MODE_MAP_CACHE: dict[int, str] = {}
GPS_EPOCH_UNIX_S = 315964800
GPS_UTC_LEAP_SECONDS = int(os.getenv("SLAM_GPS_UTC_LEAP_SECONDS", "18"))
GPS_UNIX_USEC_MIN = 946_684_800_000_000


def _normalize_param_id(param_id: Any) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode("ascii", errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def recv_match_safe(master, *args, **kwargs):
    try:
        return master.recv_match(*args, **kwargs)
    except Exception:
        time.sleep(0.05)
        return None


def request_message_interval(master, message_id: int, frequency_hz: float) -> None:
    if frequency_hz <= 0:
        interval_us = -1
    else:
        interval_us = int(1e6 / max(frequency_hz, 0.1))
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


def send_statustext(master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
    encoded_text = text.encode("utf-8", errors="ignore")
    if len(encoded_text) <= 50:
        master.mav.statustext_send(severity, encoded_text)
        return

    # This pymavlink build exposes the MAVLink1-sized STATUSTEXT API, so split
    # long operator messages instead of silently dropping the useful tail.
    words = text.split()
    chunk = ""
    for word in words:
        candidate = word if not chunk else f"{chunk} {word}"
        if len(candidate.encode("utf-8", errors="ignore")) <= 50:
            chunk = candidate
            continue
        if chunk:
            master.mav.statustext_send(severity, chunk.encode("utf-8", errors="ignore"))
        chunk = word[:50]
    if chunk:
        master.mav.statustext_send(severity, chunk.encode("utf-8", errors="ignore"))


def send_gcs_event(master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
    message = f"SLAM: {text}"
    print(f"GCS[{severity}]: {message}", flush=True)
    send_statustext(master, message, severity=severity)


def send_companion_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_play_tune(master, tune: str, fallback_text: str) -> None:
    # Always send the text first so every audible tune has a matching GCS reason.
    send_gcs_event(master, f"BEEP: {fallback_text}")

    if hasattr(master.mav, "play_tune_send"):
        master.mav.play_tune_send(
            master.target_system,
            master.target_component,
            tune[:30].encode("ascii", errors="ignore"),
            b"",
        )


def send_startup_beeps(master) -> None:
    send_play_tune(master, "MFT200L8AAA", "startup check alive after 30s; monitoring only until FIELD GATE OK")


def send_sensor_check_beep(master) -> None:
    send_play_tune(master, "MFT240L8A", "sensor quick check passed; VIO/IMU basic health only, not full readiness")


def send_ready_beeps(master) -> None:
    send_play_tune(master, "MFT200L16CDEF", "No-GPS POSHOLD gate ready; SLAM/VIO GPS2 feed may be used cautiously")


def send_calibration_active_beeps(master) -> None:
    send_play_tune(master, "MFT180L16GABG", "BRAKE calibration active; hold altitude and keep pilot override ready")


def send_calibration_complete_beeps(master) -> None:
    send_play_tune(master, "MFT160L4CDEF", "calibration successful; SLAM PosHold calibration profile saved")


def send_slam_flight_ping(master) -> None:
    send_play_tune(master, "MFT240L8A", "SLAM flight heartbeat; POSHOLD is using gated SLAM/VIO data")


def send_ground_calibration_warning_beeps(master) -> None:
    send_play_tune(master, "MFT220L16AAA", "BRAKE selected on ground; take off first, then hold near 5m")


def send_calibration_failed_beeps(master) -> None:
    send_play_tune(master, "MFT160L8CBA", "SLAM calibration failed; leave SLAM PosHold disabled")


def send_distance_sensor(master, distance_m: float, sensor_id: int, max_distance_m: float = 40.0) -> None:
    if distance_m <= 0.0:
        return
    current_cm = int(round(max(0.02, min(distance_m, max_distance_m)) * 100.0))
    max_cm = int(round(max(max_distance_m, 0.1) * 100.0))
    master.mav.distance_sensor_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        2,
        max_cm,
        current_cm,
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        sensor_id,
        mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,
        0,
    )


def send_obstacle_distance(master, distances_m: list[float], max_distance_m: float = 40.0) -> None:
    if not hasattr(master.mav, "obstacle_distance_send"):
        return
    distances_cm: list[int] = []
    max_cm = int(round(max(max_distance_m, 0.1) * 100.0))
    for distance_m in distances_m[:72]:
        if distance_m <= 0.0:
            distances_cm.append(65535)
            continue
        distances_cm.append(int(round(max(0.02, min(distance_m, max_distance_m)) * 100.0)))
    while len(distances_cm) < 72:
        distances_cm.append(65535)

    master.mav.obstacle_distance_send(
        int(time.time() * 1e6),
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        distances_cm,
        5,
        2,
        max_cm,
        5.0,
        0.0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
    )


def current_gps_week_time(now_s: float | None = None) -> tuple[int, int]:
    """Return GPS week and week-milliseconds for MAVLink GPS_INPUT.

    ArduPilot can log GPS2 position and satellites while still reporting GPS2
    unhealthy if the GPS_INPUT week/time fields stay at zero. Keep those fields
    populated for the SLAM feed, standby GPS2 mirror, and bench fake-GPS tools.
    """

    unix_s = time.time() if now_s is None else float(now_s)
    gps_s = unix_s - GPS_EPOCH_UNIX_S + GPS_UTC_LEAP_SECONDS
    if gps_s <= 0:
        return 0, 0
    week = int(gps_s // 604800)
    week_ms = int((gps_s - week * 604800) * 1000)
    return week, week_ms


def gps_input_timestamp_from_reference(
    state: FlightControllerTelemetry | None = None,
) -> tuple[int, int, int]:
    """Return GPS_INPUT time fields, preferring the Cube's real GPS clock.

    DataFlash can show GPS2 position/satellites while ArduPilot still reports
    "GPS 2: not healthy" when GPS_INPUT timing is stale. When GPS1 supplies a
    Unix-epoch timestamp, mirror that into the GPS2 feed so GPS1 and GPS2 stay
    time-aligned even if the Jetson clock is still settling after boot.
    """

    gps_time_usec = int(getattr(state, "gps_time_usec", 0) or 0) if state is not None else 0
    if gps_time_usec >= GPS_UNIX_USEC_MIN:
        gps_week, gps_week_ms = current_gps_week_time(gps_time_usec / 1e6)
        return gps_time_usec, gps_week, gps_week_ms

    now_s = time.time()
    gps_week, gps_week_ms = current_gps_week_time(now_s)
    return int(now_s * 1e6), gps_week, gps_week_ms


def send_gps_input_from_pose(
    master,
    pose,
    config,
    reference_state: FlightControllerTelemetry | None = None,
) -> bool:
    """Convert local NED-ish VIO pose into MAVLink GPS_INPUT for GPS2.

    ArduPilot expects GPS_INPUT latitude/longitude, so we anchor local meters to
    an origin learned from the real GPS/EKF reference. Without a valid origin we
    refuse to send, because a zero lat/lon GPS fix is worse than no SLAM fix.
    """

    if abs(config.origin_lat_deg) < 1e-9 and abs(config.origin_lon_deg) < 1e-9:
        return False

    earth_radius_m = 6378137.0
    origin_lat_rad = math.radians(config.origin_lat_deg)
    lat_deg = config.origin_lat_deg + math.degrees(pose.x_m / earth_radius_m)
    lon_deg = config.origin_lon_deg + math.degrees(
        pose.y_m / (earth_radius_m * max(math.cos(origin_lat_rad), 1e-6))
    )
    alt_m = config.origin_alt_m - pose.z_m
    time_usec, gps_week, gps_week_ms = gps_input_timestamp_from_reference(reference_state)
    master.mav.gps_input_send(
        time_usec,
        int(config.gps_id),
        0,
        gps_week_ms,
        gps_week,
        3,
        int(round(lat_deg * 1e7)),
        int(round(lon_deg * 1e7)),
        float(alt_m),
        0.8,
        1.2,
        float(pose.vx_m_s),
        float(pose.vy_m_s),
        float(pose.vz_m_s),
        float(config.speed_accuracy_m_s),
        float(config.horiz_accuracy_m),
        float(config.vert_accuracy_m),
        int(config.satellites_visible),
    )
    return True


def send_gps_input_from_fc_reference(master, state: FlightControllerTelemetry, config) -> bool:
    """Mirror the real GPS into GPS2 while SLAM is not ready.

    This keeps GPS2 from sitting permanently at "bad fix" during normal outdoor
    GPS flight. Once Brake calibration or LOITER observation says SLAM is ready,
    the bridge can switch GPS2 from this standby mirror to SLAM-derived pose.
    """

    if (state.gps_fix_type or 0) < 3:
        return False
    if state.gps_lat in (None, 0) or state.gps_lon in (None, 0) or state.gps_alt_mm is None:
        return False

    vn_m_s = float(state.global_vx_cm_s or 0) / 100.0
    ve_m_s = float(state.global_vy_cm_s or 0) / 100.0
    vd_m_s = float(state.global_vz_cm_s or 0) / 100.0
    time_usec, gps_week, gps_week_ms = gps_input_timestamp_from_reference(state)
    master.mav.gps_input_send(
        time_usec,
        int(config.gps_id),
        0,
        gps_week_ms,
        gps_week,
        int(max(3, state.gps_fix_type or 3)),
        int(state.gps_lat),
        int(state.gps_lon),
        float(state.gps_alt_mm) / 1000.0,
        0.8,
        1.2,
        vn_m_s,
        ve_m_s,
        vd_m_s,
        float(config.speed_accuracy_m_s),
        float(max(config.horiz_accuracy_m, 1.0)),
        float(max(config.vert_accuracy_m, 1.5)),
        int(max(state.gps_satellites or 0, min(config.satellites_visible, 8))),
    )
    return True


def send_fixed_gps_input(master, config) -> None:
    gps_week, gps_week_ms = current_gps_week_time()
    master.mav.gps_input_send(
        int(time.time() * 1e6),
        int(config.gps_id),
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT,
        gps_week_ms,
        gps_week,
        3,
        int(round(config.fixed_lat_deg * 1e7)),
        int(round(config.fixed_lon_deg * 1e7)),
        float(config.fixed_alt_m),
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        float(config.speed_accuracy_m_s),
        float(config.horiz_accuracy_m),
        float(config.vert_accuracy_m),
        int(config.satellites_visible),
    )


def send_body_velocity_nudge(master, vx_m_s: float, vy_m_s: float, vz_m_s: float = 0.0) -> None:
    velocity_only_mask = 1 | 2 | 4 | 64 | 128 | 256 | 1024 | 2048
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        velocity_only_mask,
        0.0,
        0.0,
        0.0,
        float(vx_m_s),
        float(vy_m_s),
        float(vz_m_s),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_body_yaw_rate_nudge(master, yaw_rate_deg_s: float) -> None:
    yaw_rate_only_mask = 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 | 1024
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        yaw_rate_only_mask,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        math.radians(float(yaw_rate_deg_s)),
    )


def request_parameter(master, name: str, timeout_s: float = 2.0) -> float | None:
    for _ in range(3):
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            -1,
        )

        deadline = time.time() + max(timeout_s, 0.0)
        while time.time() <= deadline:
            msg = recv_match_safe(master, blocking=True, timeout=0.5)
            if msg is None or msg.get_type() != "PARAM_VALUE":
                continue
            if _normalize_param_id(getattr(msg, "param_id", "")) != name:
                continue
            return float(getattr(msg, "param_value", 0.0))
    return None


def set_parameter(master, name: str, value: float, timeout_s: float = 3.0) -> bool | None:
    for _ in range(3):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

        deadline = time.time() + max(timeout_s, 0.0)
        while time.time() <= deadline:
            msg = recv_match_safe(master, blocking=True, timeout=0.5)
            if msg is None or msg.get_type() != "PARAM_VALUE":
                continue
            if _normalize_param_id(getattr(msg, "param_id", "")) != name:
                continue
            return abs(float(getattr(msg, "param_value", 0.0)) - float(value)) < 0.01
    return None


def publish_bridge_state(master, state_code: int, source_set_id: int = 0) -> None:
    """Publish bridge state for the FC-side Lua/GCS relay without blocking the stream."""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        BRIDGE_STATE_PARAM.encode("ascii"),
        float(state_code),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        BRIDGE_SOURCE_SET_PARAM.encode("ascii"),
        float(source_set_id),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        BRIDGE_HEARTBEAT_PARAM.encode("ascii"),
        float(int(time.monotonic()) % 1000000),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def set_ekf_source_set(master, source_set_id: int, timeout_s: float = 3.0) -> bool | None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
        0,
        source_set_id,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    deadline = time.time() + max(timeout_s, 0.0)
    while time.time() <= deadline:
        msg = recv_match_safe(master, blocking=True, timeout=0.5)
        if msg is None or msg.get_type() != "COMMAND_ACK":
            continue
        if int(getattr(msg, "command", -1)) != mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
            continue
        return int(getattr(msg, "result", -1)) == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return None


def build_fc_setup_parameters(config: FlightControllerSetupConfig) -> dict[str, float]:
    source_set = int(config.slam_source_set)
    idle_source_set = int(config.idle_source_set)
    params = {
        "AHRS_EKF_TYPE": float(config.ahrs_ekf_type),
        "EK2_ENABLE": float(config.ek2_enable),
        "EK3_ENABLE": float(config.ek3_enable),
        "EK3_SRC_OPTIONS": float(config.ek3_src_options),
        "EK3_OGN_HGT_MASK": float(config.ek3_ogn_hgt_mask),
        "VISO_TYPE": float(config.viso_type),
        "VISO_POS_X": float(config.viso_pos_x_m),
        "VISO_POS_Y": float(config.viso_pos_y_m),
        "VISO_POS_Z": float(config.viso_pos_z_m),
        "VISO_QUAL_MIN": float(config.viso_qual_min),
        "AVOID_ENABLE": float(config.avoid_enable),
        "AVOID_MARGIN": float(config.avoid_margin_m),
        "PRX1_TYPE": float(config.prx1_type),
    }

    # Keep the normal arming/flying lane GPS-safe. If this lane still references
    # ExternalNav while VISO_TYPE is disabled, ArduPilot raises:
    # "PreArm: AHRS: EK3 sources require VisualOdom".
    if idle_source_set > 0:
        params.update(
            {
                f"EK3_SRC{idle_source_set}_{name}": value
                for name, value in GPS_IDLE_SOURCE_PARAMS.items()
            }
        )

    if config.select_source_set_on_stream and config.viso_type > 0:
        params.update(
            {
                f"EK3_SRC{source_set}_POSXY": float(config.posxy_source),
                f"EK3_SRC{source_set}_VELXY": float(config.velxy_source),
                f"EK3_SRC{source_set}_POSZ": float(config.posz_source),
                f"EK3_SRC{source_set}_VELZ": float(config.velz_source),
                f"EK3_SRC{source_set}_YAW": float(config.yaw_source),
            }
        )
    elif source_set > 0:
        # Safe default while VisualOdom is intentionally disabled: scrub stale
        # ExternalNav source-set params left over from earlier experiments.
        params.update(
            {
                f"EK3_SRC{source_set}_{name}": value
                for name, value in GPS_IDLE_SOURCE_PARAMS.items()
            }
        )
    if config.gps2_type is not None:
        params["GPS2_TYPE"] = float(config.gps2_type)
    if config.gps_auto_switch is not None:
        params["GPS_AUTO_SWITCH"] = float(config.gps_auto_switch)
    return params


def apply_fc_setup(master, config: FlightControllerSetupConfig) -> FlightControllerSetupReport:
    report = FlightControllerSetupReport()
    if not config.enabled:
        return report

    desired_params = build_fc_setup_parameters(config)
    for name, value in desired_params.items():
        current = request_parameter(master, name)
        if current is not None and abs(current - float(value)) < 0.01:
            report.unchanged.append(
                ParameterChange(name=name, old_value=current, new_value=float(value), changed=False)
            )
            continue

        applied = set_parameter(master, name, value)
        if applied is None and name in OPTIONAL_PARAMS:
            report.unchanged.append(
                ParameterChange(name=name, old_value=current, new_value=float(value), changed=False)
            )
            continue
        if applied is not True:
            # Some flight controller builds may reject or ignore writes to certain
            # EKF/attitude parameters. Instead of failing the whole setup, record
            # the failure as an unchanged parameter and continue. This keeps the
            # SLAM bring-up robust on heterogeneous FC firmware versions.
            report.unchanged.append(
                ParameterChange(name=name, old_value=current, new_value=float(value), changed=False)
            )
            continue

        report.changed.append(
            ParameterChange(name=name, old_value=current, new_value=float(value), changed=True)
        )
        if name in REBOOT_RECOMMENDED_PARAMS:
            report.reboot_recommended = True

    return report


def configure_telemetry_streams(master) -> None:
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 4.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 4.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GPS2_RAW", 124), 2.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33), 2.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_VFR_HUD", 74), 2.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SCALED_PRESSURE", 29), 1.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 1.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 8.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_RC_CHANNELS", 65), 4.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_EXTENDED_SYS_STATE", 245), 2.0)
    request_message_interval(master, getattr(mavutil.mavlink, "MAVLINK_MSG_ID_BATTERY_STATUS", 147), 1.0)


def is_flight_controller_heartbeat(msg) -> bool:
    """Accept only vehicle/autopilot heartbeats as the Cube state source.

    QGC, companion computers, and other MAVLink nodes can also emit HEARTBEAT.
    Treating those as vehicle heartbeats makes mode and arm state appear to
    flicker, which is dangerous and confusing in the field.
    """

    autopilot = int(getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    mav_type = int(getattr(msg, "type", 0))
    if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if mav_type in {
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    }:
        return False
    return True


def is_from_this_mavlink_node(master, msg) -> bool:
    source_system = int(getattr(master, "source_system", 255) or 255)
    source_component = int(getattr(master, "source_component", 0) or 0)
    try:
        msg_system = int(msg.get_srcSystem())
        msg_component = int(msg.get_srcComponent())
    except Exception:  # noqa: BLE001
        return False
    if msg_system != source_system:
        return False
    return source_component <= 0 or msg_component == source_component


def drain_fc_telemetry(master, state: FlightControllerTelemetry, qgc_bridge=None) -> None:
    while True:
        msg = recv_match_safe(master, blocking=False)
        if msg is None:
            break
        if qgc_bridge is not None:
            qgc_bridge.forward_downlink(msg)

        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            state.local_position = msg
        elif msg_type == "ATTITUDE":
            state.attitude = msg
        elif msg_type == "GPS_RAW_INT":
            state.gps_time_usec = int(getattr(msg, "time_usec", 0))
            state.gps_fix_type = int(getattr(msg, "fix_type", 0))
            state.gps_satellites = int(getattr(msg, "satellites_visible", 0))
            state.gps_lat = int(getattr(msg, "lat", 0))
            state.gps_lon = int(getattr(msg, "lon", 0))
            state.gps_alt_mm = int(getattr(msg, "alt", 0))
            state.gps_vel_cm_s = int(getattr(msg, "vel", 0))
            state.gps_cog_cd = int(getattr(msg, "cog", 0))
        elif msg_type == "GPS2_RAW":
            state.gps2_time_usec = int(getattr(msg, "time_usec", 0))
            state.gps2_fix_type = int(getattr(msg, "fix_type", 0))
            state.gps2_satellites = int(getattr(msg, "satellites_visible", 0))
            state.gps2_lat = int(getattr(msg, "lat", 0))
            state.gps2_lon = int(getattr(msg, "lon", 0))
            state.gps2_alt_mm = int(getattr(msg, "alt", 0))
        elif msg_type == "GLOBAL_POSITION_INT":
            state.global_lat = int(getattr(msg, "lat", 0))
            state.global_lon = int(getattr(msg, "lon", 0))
            state.global_alt_mm = int(getattr(msg, "alt", 0))
            state.global_relative_alt_mm = int(getattr(msg, "relative_alt", 0))
            state.global_vx_cm_s = int(getattr(msg, "vx", 0))
            state.global_vy_cm_s = int(getattr(msg, "vy", 0))
            state.global_vz_cm_s = int(getattr(msg, "vz", 0))
            state.global_hdg_cd = int(getattr(msg, "hdg", 0))
        elif msg_type == "VFR_HUD":
            state.vfr_alt_m = float(getattr(msg, "alt", 0.0))
            state.vfr_climb_m_s = float(getattr(msg, "climb", 0.0))
            state.vfr_groundspeed_m_s = float(getattr(msg, "groundspeed", 0.0))
            state.vfr_throttle_pct = int(getattr(msg, "throttle", 0))
        elif msg_type == "SCALED_PRESSURE":
            state.baro_pressure_hpa = float(getattr(msg, "press_abs", 0.0))
            state.baro_alt_m = state.vfr_alt_m
        elif msg_type == "EKF_STATUS_REPORT":
            state.ekf_flags = int(getattr(msg, "flags", 0))
        elif msg_type == "STATUSTEXT":
            if is_from_this_mavlink_node(master, msg):
                continue
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            state.status_text = str(text).strip()
            state.status_severity = int(getattr(msg, "severity", mavutil.mavlink.MAV_SEVERITY_INFO))
            state.status_last_update_s = time.time()
        elif msg_type == "PARAM_VALUE":
            param_id = _normalize_param_id(getattr(msg, "param_id", ""))
            if param_id == "SCR_USER2":
                state.active_source_set = int(round(float(getattr(msg, "param_value", 0.0))))
        elif msg_type == "NAMED_VALUE_FLOAT":
            name = getattr(msg, "name", "")
            if isinstance(name, bytes):
                name = name.decode("ascii", errors="ignore")
            if str(name).rstrip("\x00") == "SCR_USER2":
                state.active_source_set = int(round(float(getattr(msg, "value", 0.0))))
        elif msg_type == "HEARTBEAT":
            if not is_flight_controller_heartbeat(msg):
                continue
            state.flight_mode = vehicle_mode_name(master, msg)
            state.armed = bool(
                int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            state.last_heartbeat_s = time.time()
        elif msg_type == "RC_CHANNELS":
            state.rc_channel_count = int(getattr(msg, "chancount", 0))
            state.rc_rssi = int(getattr(msg, "rssi", 0))
            state.rc_last_update_s = time.time()
            state.rc_channels = {
                channel: int(getattr(msg, f"chan{channel}_raw", 0))
                for channel in range(1, min(state.rc_channel_count, 18) + 1)
            }
        elif msg_type == "DISTANCE_SENSOR":
            current_distance_cm = int(getattr(msg, "current_distance", 0))
            max_distance_cm = int(getattr(msg, "max_distance", 0))
            distance_m = current_distance_cm / 100.0
            # ArduPilot may forward non-height range/proximity data on
            # DISTANCE_SENSOR. Ignore max-range sentinels and impossible height
            # readings so a side/obstacle sensor cannot poison altitude checks.
            if (
                current_distance_cm > 0
                and (max_distance_cm <= 0 or current_distance_cm < max_distance_cm)
                and 0.05 <= distance_m <= 20.0
            ):
                state.rangefinder_distance_m = distance_m
                state.rangefinder_sensor_id = int(getattr(msg, "id", 0))
                state.rangefinder_last_update_s = time.time()
        elif msg_type == "EXTENDED_SYS_STATE":
            state.landed_state = int(getattr(msg, "landed_state", 0))
            state.landed_state_last_update_s = time.time()
        elif msg_type == "BATTERY_STATUS":
            remaining = int(getattr(msg, "battery_remaining", -1))
            if remaining >= 0:
                state.battery_remaining_pct = remaining
            voltages = getattr(msg, "voltages", [])
            valid_voltages = [int(value) for value in voltages if 0 < int(value) < 65535]
            if valid_voltages:
                state.battery_voltage_v = sum(valid_voltages) / 1000.0
            state.battery_last_update_s = time.time()


def request_active_source_set(master) -> int | None:
    value = request_parameter(master, "SCR_USER2", timeout_s=1.5)
    if value is None:
        return None
    return int(round(value))


def vehicle_mode_name(master, heartbeat_msg) -> str:
    mapping = MODE_MAP_CACHE
    if not mapping:
        try:
            mode_mapping = master.mode_mapping()
        except Exception:  # noqa: BLE001
            mode_mapping = None
        if mode_mapping:
            for name, mode_id in mode_mapping.items():
                mapping[int(mode_id)] = str(name).upper()
    custom_mode = int(getattr(heartbeat_msg, "custom_mode", 0))
    return mapping.get(custom_mode, f"MODE_{custom_mode}")


def rangefinder_height_valid(
    state: FlightControllerTelemetry,
    max_age_s: float = 0.6,
    min_distance_m: float = 0.05,
    max_distance_m: float = 20.0,
) -> bool:
    if state.rangefinder_distance_m is None or state.rangefinder_last_update_s <= 0.0:
        return False
    age_s = time.time() - state.rangefinder_last_update_s
    if age_s > max(max_age_s, 0.05):
        return False
    return min_distance_m <= state.rangefinder_distance_m <= max_distance_m


def mavlink_heartbeat_valid(
    state: FlightControllerTelemetry,
    max_age_s: float = 2.5,
) -> bool:
    return state.last_heartbeat_s > 0.0 and (time.time() - state.last_heartbeat_s) <= max(max_age_s, 0.1)


def rc_link_valid(
    state: FlightControllerTelemetry,
    max_age_s: float = 2.5,
) -> bool:
    if state.rc_last_update_s <= 0.0 or (time.time() - state.rc_last_update_s) > max(max_age_s, 0.1):
        return False
    if state.rc_channel_count <= 0:
        return False
    if state.rc_rssi is not None and state.rc_rssi == 0:
        return False
    return any(value > 0 for value in state.rc_channels.values())


def gps_reference_valid(
    state: FlightControllerTelemetry,
    min_fix_type: int = 3,
    min_satellites: int = 8,
) -> bool:
    if state.gps_fix_type is None or state.gps_satellites is None:
        return False
    return state.gps_fix_type >= min_fix_type and state.gps_satellites >= min_satellites


def recent_status_blocks_slam(
    state: FlightControllerTelemetry,
    max_age_s: float = 8.0,
) -> bool:
    if not state.status_text or state.status_last_update_s <= 0.0:
        return False
    if time.time() - state.status_last_update_s > max(max_age_s, 0.1):
        return False
    text = state.status_text.lower()
    block_tokens = (
        "failsafe",
        "prearm",
        "arm:",
        "arming",
        "ekf",
        "gps glitch",
        "not ready",
        "lane switch",
        "vibration",
        "visodom",
        "visual odom",
        "out of memory",
    )
    return any(token in text for token in block_tokens)


def set_vehicle_mode(master, mode_name: str, timeout_s: float = 8.0) -> bool | None:
    try:
        master.set_mode(mode_name.upper())
    except Exception:
        return None

    deadline = time.time() + max(timeout_s, 0.0)
    while time.time() <= deadline:
        msg = recv_match_safe(master, type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if vehicle_mode_name(master, msg) == mode_name.upper():
            return True
    return None
