import time
from threading import Event
from types import SimpleNamespace

from optflow_slam.visualizer_server import (
    Im10aSource,
    MavlinkSource,
    TelemetryStore,
    _set_message_interval,
)


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


def test_link_state_is_explicit() -> None:
    store = TelemetryStore("test")
    store.set_link(False, "serial unavailable")

    snapshot = store.snapshot()

    assert not snapshot["link"]["connected"]
    assert snapshot["link"]["detail"] == "serial unavailable"


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
    assert not snapshot["ros_imu"]["axis_map_verified"]
    assert snapshot["ros_imu"]["body_axis_map"] == {
        "x": "sensor_y",
        "y": "sensor_x",
        "z": "-sensor_z",
    }
    assert not snapshot["ros_imu"]["orientation_valid"]
    assert snapshot["ros_imu"]["age_ms"] is None


def test_im10a_source_is_a_separate_telemetry_thread() -> None:
    source = Im10aSource(
        TelemetryStore("test"), Event(), "/dev/imu_usb", 9600
    )

    assert source.name == "im10a-serial"
    assert source.endpoint == "/dev/imu_usb"
    assert source.baud == 9600
