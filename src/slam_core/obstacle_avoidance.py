"""Obstacle keepout math shared by LiDAR tools.

The flight controller should remain the primary authority for aircraft
stabilization. This module only turns a 360 degree distance scan into an
operator-visible keepout vector and, when explicitly enabled by a caller, a
small body-frame velocity request.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


ZONE_NAMES = (
    "Front",
    "Front-Right",
    "Right",
    "Rear-Right",
    "Rear",
    "Rear-Left",
    "Left",
    "Front-Left",
)


@dataclass(frozen=True)
class AvoidanceCommand:
    active: bool
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    speed_m_s: float = 0.0
    closest_distance_m: float = 0.0
    closest_angle_deg: float = 0.0
    active_sector_count: int = 0
    reason: str = "clear"


def _positive_distances(distances_m: list[float] | tuple[float, ...]) -> list[float]:
    return [distance_m for distance_m in distances_m if distance_m > 0.0 and math.isfinite(distance_m)]


def reduce_to_zones(
    sector_distances_m: list[float] | tuple[float, ...],
    zone_count: int = 8,
    angle_offset_deg: float = 0.0,
    max_distance_m: float | None = None,
) -> list[float]:
    """Reduce a 360 degree sector scan to minimum distance per named zone.

    Sector angle 0 is body forward and positive angles rotate toward body-right,
    matching ArduPilot's body-frame obstacle convention.
    """

    if zone_count <= 0:
        raise ValueError("zone_count must be positive")
    if not sector_distances_m:
        return [0.0] * zone_count

    zone_width_deg = 360.0 / zone_count
    sector_width_deg = 360.0 / len(sector_distances_m)
    zones = [0.0] * zone_count

    for index, distance_m in enumerate(sector_distances_m):
        if distance_m <= 0.0 or not math.isfinite(distance_m):
            continue
        if max_distance_m is not None and distance_m > max_distance_m:
            continue
        angle_deg = (angle_offset_deg + index * sector_width_deg) % 360.0
        zone_index = int(((angle_deg + zone_width_deg / 2.0) % 360.0) // zone_width_deg)
        current = zones[zone_index]
        if current <= 0.0 or distance_m < current:
            zones[zone_index] = distance_m
    return zones


def compute_keepout_velocity(
    sector_distances_m: list[float] | tuple[float, ...],
    keepout_distance_m: float = 1.5,
    min_valid_distance_m: float = 0.15,
    max_speed_m_s: float = 1.0,
    critical_distance_m: float = 0.5,
    angle_offset_deg: float = 0.0,
    speed_exponent: float = 0.75,
) -> AvoidanceCommand:
    """Compute a body-frame push vector away from close obstacles.

    Returns body-frame velocity where +X is forward and +Y is right. A front
    obstacle therefore produces negative X, and a right obstacle produces
    negative Y. The command magnitude rises as the object approaches the
    critical distance, then clamps at ``max_speed_m_s``.
    """

    if not sector_distances_m:
        return AvoidanceCommand(active=False, reason="no_scan")
    if keepout_distance_m <= min_valid_distance_m:
        return AvoidanceCommand(active=False, reason="invalid_keepout")
    if max_speed_m_s <= 0.0:
        return AvoidanceCommand(active=False, reason="zero_max_speed")

    valid_distances = _positive_distances(sector_distances_m)
    if not valid_distances:
        return AvoidanceCommand(active=False, reason="clear")

    sector_width_deg = 360.0 / len(sector_distances_m)
    closest_distance_m = float("inf")
    closest_angle_deg = 0.0
    vx = 0.0
    vy = 0.0
    active_sector_count = 0
    critical = max(min_valid_distance_m, min(critical_distance_m, keepout_distance_m))
    response_span = max(keepout_distance_m - critical, 0.001)
    exponent = max(speed_exponent, 0.1)

    for index, distance_m in enumerate(sector_distances_m):
        if distance_m <= 0.0 or not math.isfinite(distance_m):
            continue
        if distance_m < closest_distance_m:
            closest_distance_m = distance_m
            closest_angle_deg = (angle_offset_deg + index * sector_width_deg) % 360.0
        if distance_m < min_valid_distance_m or distance_m >= keepout_distance_m:
            continue

        angle_deg = (angle_offset_deg + index * sector_width_deg) % 360.0
        angle_rad = math.radians(angle_deg)
        if distance_m <= critical:
            strength = max_speed_m_s
        else:
            closeness = (keepout_distance_m - distance_m) / response_span
            strength = max_speed_m_s * max(0.0, min(1.0, closeness)) ** exponent

        # Push away from the measured direction.
        vx -= math.cos(angle_rad) * strength
        vy -= math.sin(angle_rad) * strength
        active_sector_count += 1

    speed = math.hypot(vx, vy)
    if speed <= 0.0:
        return AvoidanceCommand(
            active=False,
            closest_distance_m=min(valid_distances),
            closest_angle_deg=closest_angle_deg,
            reason="outside_keepout",
        )
    if speed > max_speed_m_s:
        scale = max_speed_m_s / speed
        vx *= scale
        vy *= scale
        speed = max_speed_m_s

    return AvoidanceCommand(
        active=True,
        vx_m_s=vx,
        vy_m_s=vy,
        speed_m_s=speed,
        closest_distance_m=closest_distance_m,
        closest_angle_deg=closest_angle_deg,
        active_sector_count=active_sector_count,
        reason="keepout",
    )


def velocity_to_tilt_deg(
    vx_m_s: float,
    vy_m_s: float,
    max_speed_m_s: float,
    max_tilt_rad: float,
) -> tuple[float, float]:
    """Return intended pitch/roll degrees for logs and GCS text.

    This is display math only. Actual stabilization remains with ArduPilot.
    """

    if max_speed_m_s <= 0.0 or max_tilt_rad <= 0.0:
        return 0.0, 0.0
    tilt_deg = math.degrees(max_tilt_rad)
    pitch_deg = -(max(-1.0, min(1.0, vx_m_s / max_speed_m_s))) * tilt_deg
    roll_deg = max(-1.0, min(1.0, vy_m_s / max_speed_m_s)) * tilt_deg
    return pitch_deg, roll_deg
