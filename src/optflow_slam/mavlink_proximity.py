"""Reliable Cube MAVLink proximity transport for horizontal obstacle scans."""

from __future__ import annotations

import math
from typing import Any

from .obstacles import ObstacleScan, UNKNOWN_DISTANCE_CM


HORIZONTAL_FACE_COUNT = 8
HORIZONTAL_FACE_WIDTH_DEG = 360.0 / HORIZONTAL_FACE_COUNT


def send_horizontal_distance_sensor(
    sender: Any,
    mavlink: Any,
    scan: ObstacleScan,
    orientation: int,
    distance_cm: int,
) -> None:
    """Publish one short horizontal MAVLink DISTANCE_SENSOR packet."""

    if not 0 <= orientation < HORIZONTAL_FACE_COUNT:
        raise ValueError("horizontal orientation must be between 0 and 7")
    if distance_cm == UNKNOWN_DISTANCE_CM:
        raise ValueError("an unknown distance must not be transmitted")
    sender.distance_sensor_send(
        (scan.monotonic_ns // 1_000_000) & 0xFFFFFFFF,
        scan.min_distance_cm,
        scan.max_distance_cm,
        distance_cm,
        mavlink.MAV_DISTANCE_SENSOR_LASER,
        orientation,
        orientation,
        255,
        horizontal_fov=math.radians(HORIZONTAL_FACE_WIDTH_DEG),
        vertical_fov=0.0,
        signal_quality=0,
    )


def horizontal_face_distances_cm(scan: ObstacleScan) -> tuple[int, ...]:
    """Reduce a dense 360-degree scan to conservative 45-degree face minima."""

    faces = [UNKNOWN_DISTANCE_CM] * HORIZONTAL_FACE_COUNT
    half_face_deg = HORIZONTAL_FACE_WIDTH_DEG / 2.0
    for index, distance_cm in enumerate(scan.distances_cm):
        if distance_cm == UNKNOWN_DISTANCE_CM:
            continue
        angle_deg = index * scan.increment_deg
        face = int(
            math.floor(
                (angle_deg + half_face_deg) / HORIZONTAL_FACE_WIDTH_DEG
            )
        ) % HORIZONTAL_FACE_COUNT
        faces[face] = min(faces[face], distance_cm)
    return tuple(faces)


def horizontal_transport_distances_cm(scan: ObstacleScan) -> tuple[int, ...]:
    """Encode a healthy no-return face as clear through sensor range."""

    clear_distance_cm = scan.max_distance_cm + 1
    if clear_distance_cm >= UNKNOWN_DISTANCE_CM:
        raise ValueError("maximum obstacle distance cannot be encoded")
    return tuple(
        clear_distance_cm
        if distance_cm == UNKNOWN_DISTANCE_CM
        else distance_cm
        for distance_cm in horizontal_face_distances_cm(scan)
    )


def send_horizontal_distance_sensors(
    sender: Any,
    mavlink: Any,
    scan: ObstacleScan,
) -> int:
    """Publish valid face minima as short MAVLink DISTANCE_SENSOR packets."""

    packets_sent = 0
    for orientation, distance_cm in enumerate(
        horizontal_transport_distances_cm(scan)
    ):
        send_horizontal_distance_sensor(
            sender,
            mavlink,
            scan,
            orientation,
            distance_cm,
        )
        packets_sent += 1
    return packets_sent
