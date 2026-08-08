import json
import time
from pathlib import Path
from threading import Event
from types import SimpleNamespace

from optflow_slam.config import load_config
from optflow_slam.obstacles import ObstacleScan, UNKNOWN_DISTANCE_CM
from optflow_slam.visualizer_server import (
    Im10aSource,
    MavlinkSource,
    NavigationTrajectoryStore,
    OBSTACLE_BEEP_TUNE,
    RawEventBus,
    STARTUP_RISING_TUNE,
    TELEMETRY_STREAM_HZ,
    TelemetryStore,
    VisualCueStore,
    _event_safe,
    _restore_message_intervals,
    _set_message_interval,
)

ROOT = Path(__file__).resolve().parents[1]


def test_visual_cue_store_sequences_display_only_instructions() -> None:
    store = VisualCueStore()

    first = store.trigger(
        "MOVE DRONE NOW",
        detail="Translate slowly and keep yaw steady.",
        flash_count=2,
        duration_s=10.0,
    )
    second = store.trigger("HOLD STILL", duration_s=5.0)

    assert first["active"]
    assert first["sequence"] == 1
    assert first["flash_count"] == 2
    assert second["sequence"] == 2
    assert store.snapshot()["message"] == "HOLD STILL"


def test_navigation_trajectory_store_exposes_atomic_runtime_status(
    tmp_path: Path,
) -> None:
    status_path = tmp_path / "slam_navigation_status.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "updated_unix_ns": time.time_ns(),
                "state": "recording_outbound",
                "trajectories": {"lio": [[0.0, 0.0, 0.0]]},
            }
        ),
        encoding="utf-8",
    )

    snapshot = NavigationTrajectoryStore(status_path).snapshot()

    assert snapshot["available"]
    assert snapshot["live"]
    assert snapshot["kind"] == "trajectory"
    assert snapshot["state"] == "recording_outbound"
    assert snapshot["trajectories"]["lio"] == [[0.0, 0.0, 0.0]]


def test_telemetry_snapshot_reports_freshness() -> None:
    store = TelemetryStore("test")
    now = time.monotonic()
    store.update(
        "flow",
        rate_x_rads=0.2,
        rate_y_rads=-0.1,
        quality=123,
        updated_monotonic=now,
    )

    snapshot = store.snapshot()

    assert snapshot["source"] == "test"
    assert snapshot["flow"]["quality"] == 123
    assert snapshot["flow"]["rate_x_rads"] == 0.2
    assert snapshot["flow"]["age_ms"] is not None
    assert snapshot["flow"]["age_ms"] < 100
    assert snapshot["obstacles"]["clearance_reference"] == "aircraft_cg"
    assert snapshot["obstacles"]["clearance_status"] == "unknown"


def test_link_state_is_explicit() -> None:
    store = TelemetryStore("test")
    store.set_link(False, "serial unavailable")

    snapshot = store.snapshot()

    assert not snapshot["link"]["connected"]
    assert snapshot["link"]["detail"] == "serial unavailable"


def test_mavlink_source_sends_only_the_latest_fresh_obstacle_scan() -> None:
    calls = []
    components = []
    master = SimpleNamespace(
        mav=SimpleNamespace(
            obstacle_distance_send=lambda *args, **kwargs: (
                calls.append((args, kwargs)),
                components.append(master.mav.srcComponent),
            )
        )
    )
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_DISTANCE_SENSOR_LASER=0,
            MAV_FRAME_BODY_FRD=12,
            MAV_COMP_ID_OBSTACLE_AVOIDANCE=196,
        )
    )
    source = MavlinkSource(
        TelemetryStore("test"),
        Event(),
        "/dev/null",
        921600,
        obstacle_max_age_s=0.25,
    )
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[0] = 200
    older = ObstacleScan(
        source="depth",
        monotonic_ns=time.monotonic_ns(),
        distances_cm=tuple(distances),
        increment_deg=5.0,
        min_distance_cm=30,
        max_distance_cm=800,
    )
    distances[0] = 150
    latest = ObstacleScan(
        source="depth+lidar",
        monotonic_ns=time.monotonic_ns(),
        distances_cm=tuple(distances),
        increment_deg=5.0,
        min_distance_cm=30,
        max_distance_cm=800,
    )

    source.queue_obstacle_scan(older)
    source.queue_obstacle_scan(latest)

    assert source._send_pending_obstacle(master, mavutil)
    assert not source._send_pending_obstacle(master, mavutil)
    assert len(calls) == 1
    assert components == [196]
    assert not hasattr(master.mav, "srcComponent")
    args, kwargs = calls[0]
    assert args[2][0] == 150
    assert args[3:6] == (5, 30, 800)
    assert kwargs == {
        "increment_f": 5.0,
        "angle_offset": 0.0,
        "frame": 12,
    }


def test_mavlink_source_drops_stale_obstacle_output() -> None:
    calls = []
    master = SimpleNamespace(
        mav=SimpleNamespace(
            obstacle_distance_send=lambda *args, **kwargs: calls.append(
                (args, kwargs)
            )
        )
    )
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_DISTANCE_SENSOR_LASER=0,
            MAV_FRAME_BODY_FRD=12,
            MAV_COMP_ID_OBSTACLE_AVOIDANCE=196,
        )
    )
    source = MavlinkSource(
        TelemetryStore("test"),
        Event(),
        "/dev/null",
        921600,
        obstacle_max_age_s=0.25,
    )
    source.queue_obstacle_scan(
        ObstacleScan(
            source="depth",
            monotonic_ns=time.monotonic_ns() - 300_000_000,
            distances_cm=tuple([UNKNOWN_DISTANCE_CM] * 72),
            increment_deg=5.0,
            min_distance_cm=30,
            max_distance_cm=800,
        )
    )

    assert not source._send_pending_obstacle(master, mavutil)
    assert calls == []


def test_obstacle_beep_requires_armed_vehicle_and_high_rc7() -> None:
    tunes = []
    master = SimpleNamespace(
        mav=SimpleNamespace(
            play_tune_send=lambda *args: tunes.append(args),
        )
    )
    configured = load_config(ROOT / "config" / "system.yaml")
    source = MavlinkSource(
        TelemetryStore("test"),
        Event(),
        "/dev/null",
        921600,
        obstacle_settings=configured.obstacle_avoidance,
    )
    source.target_system = 1
    source.target_component = 1
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[0] = 150
    source.queue_obstacle_scan(
        ObstacleScan(
            source="lidar",
            monotonic_ns=time.monotonic_ns(),
            distances_cm=tuple(distances),
            increment_deg=5.0,
            min_distance_cm=30,
            max_distance_cm=800,
        )
    )

    source._set_rc_toggle_pwm(1800)
    assert not source._maybe_send_obstacle_beep(master)

    source._armed = True
    assert source._maybe_send_obstacle_beep(master)
    assert tunes[-1][2] == OBSTACLE_BEEP_TUNE.encode("ascii")
    assert not source._maybe_send_obstacle_beep(master)

    source._set_rc_toggle_pwm(1000)
    source._last_obstacle_beep_s = float("-inf")
    assert not source._maybe_send_obstacle_beep(master)


def test_startup_rising_tune_is_sent_only_once_per_process() -> None:
    tunes = []
    master = SimpleNamespace(
        mav=SimpleNamespace(
            play_tune_send=lambda *args: tunes.append(args),
        )
    )
    source = MavlinkSource(
        TelemetryStore("test"),
        Event(),
        "/dev/null",
        921600,
        startup_tune_enabled=True,
    )
    source.target_system = 1
    source.target_component = 1

    assert source._maybe_send_startup_tune(master)
    assert not source._maybe_send_startup_tune(master)
    assert len(tunes) == 1
    assert tunes[0][2] == STARTUP_RISING_TUNE.encode("ascii")


def test_imu_snapshot_uses_si_units_and_reports_freshness() -> None:
    store = TelemetryStore("test")
    now = time.monotonic()
    store.update(
        "imu",
        accel_x_mss=0.4,
        accel_y_mss=-0.2,
        accel_z_mss=-9.81,
        gyro_x_rads=0.01,
        gyro_y_rads=-0.02,
        gyro_z_rads=0.03,
        message="HIGHRES_IMU",
        updated_monotonic=now,
    )

    snapshot = store.snapshot()

    assert snapshot["imu"]["accel_z_mss"] == -9.81
    assert snapshot["imu"]["gyro_z_rads"] == 0.03
    assert snapshot["imu"]["message"] == "HIGHRES_IMU"
    assert snapshot["imu"]["age_ms"] is not None
    assert snapshot["imu"]["age_ms"] < 100


def test_message_interval_request_waits_for_cube_ack() -> None:
    class Ack:
        result = 0

    class Mav:
        def __init__(self) -> None:
            self.command = None

        def command_long_send(self, *args) -> None:
            self.command = args

    class Master:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = Mav()
            self.ack_requested = False

        def recv_match(self, **kwargs):
            self.ack_requested = kwargs["type"] == "COMMAND_ACK"
            return Ack()

    master = Master()

    result = _set_message_interval(
        master, message_id=30, interval_us=50_000, wait_for_ack=True
    )

    assert result == 0
    assert master.ack_requested
    assert master.mav.command is not None


def test_message_interval_cleanup_restores_defaults_instead_of_disabling() -> None:
    class Mav:
        def __init__(self) -> None:
            self.commands = []

        def command_long_send(self, *args) -> None:
            self.commands.append(args)

    master = SimpleNamespace(target_system=1, target_component=0, mav=Mav())

    _restore_message_intervals(master, (30, 105))

    assert [command[4] for command in master.mav.commands] == [30, 105]
    assert [command[5] for command in master.mav.commands] == [0, 0]


def test_legacy_optical_flow_message_keeps_pixel_and_compensated_fields() -> None:
    store = TelemetryStore("test")
    source = MavlinkSource(store, Event(), "/dev/null", 921600)
    message = SimpleNamespace(
        get_type=lambda: "OPTICAL_FLOW",
        flow_x=2,
        flow_y=-3,
        flow_comp_m_x=0.25,
        flow_comp_m_y=-0.5,
        quality=137,
    )

    source._handle_message(message, mavutil=None)
    snapshot = store.snapshot()

    assert snapshot["flow"]["delta_x_dpix"] == 2
    assert snapshot["flow"]["delta_y_dpix"] == -3
    assert snapshot["flow"]["rate_x_rads"] == 0.0
    assert snapshot["flow"]["rate_y_rads"] == 0.0
    assert snapshot["flow"]["comp_x_mps"] == 0.25
    assert snapshot["flow"]["comp_y_mps"] == -0.5
    assert snapshot["flow"]["quality"] == 137


def test_extended_optical_flow_message_prefers_float_rate_fields() -> None:
    store = TelemetryStore("test")
    source = MavlinkSource(store, Event(), "/dev/null", 921600)
    message = SimpleNamespace(
        get_type=lambda: "OPTICAL_FLOW",
        flow_x=2,
        flow_y=-3,
        flow_rate_x=0.125,
        flow_rate_y=-0.25,
        flow_comp_m_x=0.25,
        flow_comp_m_y=-0.5,
        quality=201,
    )

    source._handle_message(message, mavutil=None)
    snapshot = store.snapshot()

    assert snapshot["flow"]["rate_x_rads"] == 0.125
    assert snapshot["flow"]["rate_y_rads"] == -0.25
    assert snapshot["flow"]["quality"] == 201


def test_ros_imu_snapshot_has_explicit_unverified_extrinsics() -> None:
    store = TelemetryStore("test")
    snapshot = store.snapshot()

    assert snapshot["ros_imu"]["contract"] == "sensor_msgs/Imu"
    assert snapshot["ros_imu"]["frame_id"] == "im10a_link"
    assert not snapshot["ros_imu"]["extrinsics_verified"]
    assert snapshot["ros_imu"]["age_ms"] is None


def test_ros_imu_snapshot_maps_measured_body_axis_signs() -> None:
    store = TelemetryStore(
        "test",
        imu_axis_signs=(1, -1, -1),
        imu_axis_map_verified=True,
        imu_axis_map_verification="dynamic gyro correlation",
    )
    store.update(
        "ros_imu",
        accel_x_mss=1.0,
        accel_y_mss=2.0,
        accel_z_mss=3.0,
        gyro_x_rads=0.1,
        gyro_y_rads=0.2,
        gyro_z_rads=0.3,
        roll_rad=0.4,
        pitch_rad=0.5,
        yaw_rad=0.6,
    )

    snapshot = store.snapshot()
    ros_imu = snapshot["ros_imu"]

    assert ros_imu["axis_map_label"] == "X/-Y/-Z"
    assert ros_imu["axis_map_verified"]
    assert ros_imu["body_preview"] == {
        "accel_x_mss": 1.0,
        "accel_y_mss": -2.0,
        "accel_z_mss": -3.0,
        "gyro_x_rads": 0.1,
        "gyro_y_rads": -0.2,
        "gyro_z_rads": -0.3,
        "roll_rad": 0.4,
        "pitch_rad": -0.5,
        "yaw_rad": -0.6,
    }
    assert snapshot["visualizer_stream_rate_hz"] == TELEMETRY_STREAM_HZ


def test_im10a_source_is_a_separate_telemetry_thread() -> None:
    source = Im10aSource(
        TelemetryStore("test"), Event(), "/dev/imu_usb", 9600
    )

    assert source.name == "im10a-serial"
    assert source.endpoint == "/dev/imu_usb"
    assert source.baud == 9600


def test_flight_analysis_telemetry_is_exposed_in_snapshot() -> None:
    store = TelemetryStore("test")
    source = MavlinkSource(store, Event(), "/dev/null", 921600)
    source._handle_message(
        SimpleNamespace(
            get_type=lambda: "LOCAL_POSITION_NED",
            x=1.0,
            y=2.0,
            z=-3.0,
            vx=0.1,
            vy=0.2,
            vz=-0.3,
            time_boot_ms=1234,
        ),
        mavutil=None,
    )
    source._handle_message(
        SimpleNamespace(
            get_type=lambda: "VIBRATION",
            vibration_x=4.0,
            vibration_y=5.0,
            vibration_z=6.0,
            clipping_0=1,
            clipping_1=2,
            clipping_2=3,
        ),
        mavutil=None,
    )
    source._handle_message(
        SimpleNamespace(
            get_type=lambda: "SYS_STATUS",
            voltage_battery=23600,
            current_battery=1120,
            battery_remaining=58,
        ),
        mavutil=None,
    )

    snapshot = store.snapshot()

    assert snapshot["local_position"]["x_m"] == 1.0
    assert snapshot["local_position"]["z_down_m"] == -3.0
    assert snapshot["vibration"]["clipping_2"] == 3
    assert snapshot["power"]["voltage_v"] == 23.6
    assert snapshot["power"]["current_a"] == 11.2


def test_raw_event_bus_reports_overwritten_sequences() -> None:
    bus = RawEventBus(max_events=2)
    bus.publish("imu", "accel", {"x": 1})
    bus.publish("imu", "gyro", {"x": 2})
    bus.publish("imu", "accel", {"x": 3})

    events, dropped = bus.wait_after(0, timeout=0.0)

    assert dropped == 1
    assert [event["sequence"] for event in events] == [2, 3]


def test_raw_event_payload_converts_binary_mavlink_fields() -> None:
    assert _event_safe({"data": bytearray((0x01, 0xA2))}) == {
        "data": "01a2"
    }


def test_battery_status_uses_mavlink_power_units() -> None:
    store = TelemetryStore("test")
    source = MavlinkSource(store, Event(), "/dev/null", 921600)
    source._handle_message(
        SimpleNamespace(
            get_type=lambda: "BATTERY_STATUS",
            voltages=[23600] + [65535] * 9,
            current_battery=1120,
            battery_remaining=58,
            current_consumed=250,
            energy_consumed=360,
            time_remaining=120,
            id=0,
        ),
        mavutil=None,
    )

    power = store.snapshot()["power"]

    assert power["voltage_v"] == 23.6
    assert power["current_a"] == 11.2
    assert power["consumed_mah"] == 250
    assert power["consumed_wh"] == 10.0
