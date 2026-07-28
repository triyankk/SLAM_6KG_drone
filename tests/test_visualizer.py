import time
from threading import Event
from types import SimpleNamespace

from optflow_slam.visualizer_server import (
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


def test_legacy_optical_flow_message_uses_base_rate_fields() -> None:
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

    assert snapshot["flow"]["rate_x_rads"] == 2.0
    assert snapshot["flow"]["rate_y_rads"] == -3.0
    assert snapshot["flow"]["comp_x"] == 0.25
    assert snapshot["flow"]["comp_y"] == -0.5
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
