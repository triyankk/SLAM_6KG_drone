from pathlib import Path
import threading
from types import SimpleNamespace

from optflow_slam.config import load_config
from optflow_slam.obstacle_avoidance_service import (
    FACE_PACKET_GAP_NS,
    Jt16ObstacleSource,
    PacedFaceScheduler,
    CubeProximityLink,
    _validate,
)
from optflow_slam.obstacles import ObstacleScan, UNKNOWN_DISTANCE_CM


ROOT = Path(__file__).resolve().parents[1]


def _scan(monotonic_ns: int) -> ObstacleScan:
    distances = [UNKNOWN_DISTANCE_CM] * 72
    distances[0] = 200
    distances[18] = 350
    return ObstacleScan(
        source="lidar",
        monotonic_ns=monotonic_ns,
        distances_cm=tuple(distances),
        increment_deg=5.0,
        min_distance_cm=30,
        max_distance_cm=800,
    )


def test_paced_scheduler_avoids_face_packet_bursts() -> None:
    now_ns = 1_000_000_000
    scheduler = PacedFaceScheduler(rate_hz=10.0, stale_timeout_s=0.45)
    scheduler.queue(_scan(now_ns))

    first = scheduler.next_packet(now_ns)
    assert first is not None and first[1:] == (0, 200)
    assert scheduler.next_packet(now_ns + FACE_PACKET_GAP_NS - 1) is None

    second = scheduler.next_packet(now_ns + FACE_PACKET_GAP_NS)
    assert second is not None and second[1:] == (1, 801)

    packets = [first, second]
    for index in range(2, 8):
        packet = scheduler.next_packet(
            now_ns + index * FACE_PACKET_GAP_NS
        )
        assert packet is not None
        packets.append(packet)
    assert [packet[1] for packet in packets] == list(range(8))
    assert packets[2][2] == 350


def test_paced_scheduler_drops_stale_scan() -> None:
    scheduler = PacedFaceScheduler(rate_hz=10.0, stale_timeout_s=0.45)
    scheduler.queue(_scan(1_000_000_000))

    assert scheduler.next_packet(1_450_000_001) is None


def test_default_config_is_valid_for_oa_only_runtime() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    _validate(config)
    source = Jt16ObstacleSource(config, threading.Event(), lambda _scan: None)
    command = source._command()

    assert "--raw-output" not in command
    assert command[2] == "/dev/jt16_usb"


def test_cube_health_uses_proximity_bit_not_laser_bit() -> None:
    config = load_config(ROOT / "config" / "system.yaml")
    link = CubeProximityLink(config, threading.Event())
    mavlink = SimpleNamespace(
        MAV_SYS_STATUS_SENSOR_PROXIMITY=0x04000000,
    )
    mavutil = SimpleNamespace(mavlink=mavlink)

    def message(bits: int):
        return SimpleNamespace(
            get_type=lambda: "SYS_STATUS",
            onboard_control_sensors_present=bits,
            onboard_control_sensors_enabled=bits,
            onboard_control_sensors_health=bits,
        )

    link._handle_message(message(0x00000100), mavutil)
    assert link.snapshot(1_000_000_000)["proximity_healthy"] is False

    link._handle_message(message(0x04000000), mavutil)
    assert link.snapshot(1_000_000_000)["proximity_healthy"] is True
