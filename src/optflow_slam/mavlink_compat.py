"""Compatibility guards for the pinned, released pymavlink runtime."""

from __future__ import annotations

import threading
from typing import Any


_PATCH_LOCK = threading.Lock()


def install_pymavlink_instance_guard(mavutil: Any | None = None) -> Any:
    """Backport the unreleased instance-cache guard after pymavlink 2.4.49."""
    if mavutil is None:
        from pymavlink import mavutil as imported_mavutil

        mavutil = imported_mavutil
    with _PATCH_LOCK:
        if getattr(mavutil, "_optflow_instance_guard", False):
            return mavutil
        original_add_message = mavutil.add_message

        def guarded_add_message(
            messages: dict,
            message_type: str,
            message: Any,
        ):
            instance_field = getattr(message, "_instance_field", None)
            instance_value = (
                getattr(message, instance_field, None)
                if instance_field is not None
                else None
            )
            previous = messages.get(message_type)
            if (
                instance_value is not None
                and previous is not None
                and getattr(previous, "_instances", None) is None
            ):
                messages.pop(message_type, None)
            return original_add_message(messages, message_type, message)

        mavutil.add_message = guarded_add_message
        mavutil._optflow_instance_guard = True
    return mavutil
