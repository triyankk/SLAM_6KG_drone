from types import SimpleNamespace

from pymavlink import mavutil

from optflow_slam.mavlink_compat import install_pymavlink_instance_guard


def test_instance_guard_recovers_after_uninstanced_message() -> None:
    install_pymavlink_instance_guard(mavutil)
    messages = {
        "BATTERY_STATUS": SimpleNamespace(_instances=None),
    }
    message = SimpleNamespace(
        _instance_field="id",
        _instances=None,
        id=1,
    )

    mavutil.add_message(messages, "BATTERY_STATUS", message)

    assert messages["BATTERY_STATUS"]._instances[1] is message


def test_instance_guard_is_idempotent() -> None:
    first = install_pymavlink_instance_guard(mavutil).add_message
    second = install_pymavlink_instance_guard(mavutil).add_message

    assert first is second
