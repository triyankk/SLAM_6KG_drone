from types import SimpleNamespace

from optflow_slam.mavlink_proximity import (
    horizontal_face_distances_cm,
    horizontal_transport_distances_cm,
    send_horizontal_distance_sensors,
)
from optflow_slam.obstacles import ObstacleScan, UNKNOWN_DISTANCE_CM


def _scan(distances: list[int]) -> ObstacleScan:
    return ObstacleScan(
        source="lidar",
        monotonic_ns=1_234_567_890_000_000,
        distances_cm=tuple(distances),
        increment_deg=5.0,
        min_distance_cm=30,
        max_distance_cm=800,
    )


def test_horizontal_faces_use_conservative_wrapped_minima() -> None:
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[71] = 180
    distances[0] = 200
    distances[4] = 160
    distances[5] = 300
    distances[9] = 250
    distances[13] = 280

    faces = horizontal_face_distances_cm(_scan(distances))

    assert faces[0] == 160
    assert faces[1] == 250
    assert faces[2:] == (UNKNOWN_DISTANCE_CM,) * 6


def test_transport_encodes_no_return_as_clear_through_sensor_range() -> None:
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[0] = 200

    faces = horizontal_transport_distances_cm(_scan(distances))

    assert faces == (200, 801, 801, 801, 801, 801, 801, 801)


def test_sender_publishes_all_faces_and_uses_orientation_ids() -> None:
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[0] = 200
    distances[18] = 350
    calls = []
    sender = SimpleNamespace(
        distance_sensor_send=lambda *args, **kwargs: calls.append(
            (args, kwargs)
        )
    )
    mavlink = SimpleNamespace(MAV_DISTANCE_SENSOR_LASER=0)

    packets = send_horizontal_distance_sensors(
        sender,
        mavlink,
        _scan(distances),
    )

    assert packets == 8
    assert [call[0][6] for call in calls] == list(range(8))
    assert [call[0][3] for call in calls] == [
        200,
        801,
        350,
        801,
        801,
        801,
        801,
        801,
    ]
    assert all(call[1]["signal_quality"] == 0 for call in calls)
