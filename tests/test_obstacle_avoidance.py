import pytest

from slam_core.obstacle_avoidance import compute_keepout_velocity, reduce_to_zones


def make_scan(index: int, distance_m: float, sectors: int = 72) -> list[float]:
    scan = [0.0] * sectors
    scan[index] = distance_m
    return scan


def test_front_obstacle_pushes_backward_and_closer_is_faster():
    near = compute_keepout_velocity(make_scan(0, 0.5), keepout_distance_m=1.5, max_speed_m_s=1.0)
    far = compute_keepout_velocity(make_scan(0, 1.4), keepout_distance_m=1.5, max_speed_m_s=1.0)

    assert near.active
    assert far.active
    assert near.vx_m_s < 0.0
    assert abs(near.vy_m_s) < 1e-6
    assert near.speed_m_s > far.speed_m_s
    assert near.speed_m_s == pytest.approx(1.0)


def test_right_obstacle_pushes_left():
    command = compute_keepout_velocity(make_scan(18, 0.5), keepout_distance_m=1.5, max_speed_m_s=1.0)

    assert command.active
    assert abs(command.vx_m_s) < 1e-6
    assert command.vy_m_s < 0.0


def test_reduce_to_zones_tracks_closest_named_zone():
    scan = [0.0] * 72
    scan[0] = 2.0
    scan[1] = 1.2
    scan[18] = 0.8

    zones = reduce_to_zones(scan, zone_count=8)

    assert zones[0] == pytest.approx(1.2)
    assert zones[2] == pytest.approx(0.8)
