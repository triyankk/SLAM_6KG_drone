"""Point-cloud helpers for passive flight recordings."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class MapPose:
    """Aircraft pose in a local map with Z pointing up."""

    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    source: str


def body_frd_to_local_up_rotation(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> np.ndarray:
    """Return a body-FRD to local X/Y/Z-up rotation matrix."""

    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)

    body_to_ned = np.array(
        (
            (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
            (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
            (-sp, sr * cp, cr * cp),
        ),
        dtype=np.float64,
    )
    ned_to_local_up = np.diag((1.0, 1.0, -1.0))
    return ned_to_local_up @ body_to_ned


def camera_optical_to_local(
    camera_points_m: np.ndarray,
    pose: MapPose,
    *,
    camera_translation_body_frd_m: Iterable[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """Transform RealSense optical XYZ points through an assumed forward mount."""

    points = np.asarray(camera_points_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera_points_m must have shape (N, 3)")

    # RealSense optical: +X right, +Y down, +Z forward.
    body_points = points[:, (2, 0, 1)]
    translation = np.asarray(
        tuple(camera_translation_body_frd_m), dtype=np.float64
    )
    if translation.shape != (3,):
        raise ValueError("camera translation must contain three values")
    body_points += translation

    rotation = body_frd_to_local_up_rotation(
        pose.roll_rad, pose.pitch_rad, pose.yaw_rad
    )
    local_points = body_points @ rotation.T
    local_points += np.array((pose.x_m, pose.y_m, pose.z_m))
    return local_points.astype(np.float32, copy=False)


def write_binary_ply(
    path: Path, points_m: np.ndarray, colors_rgb: np.ndarray | None = None
) -> None:
    """Write an atomic binary little-endian PLY point cloud."""

    points = np.asarray(points_m, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_m must have shape (N, 3)")
    if colors_rgb is None:
        colors = np.full((len(points), 3), 210, dtype=np.uint8)
    else:
        colors = np.asarray(colors_rgb, dtype=np.uint8)
        if colors.shape != (len(points), 3):
            raise ValueError("colors_rgb must have shape (N, 3)")

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    vertices = np.empty(
        len(points),
        dtype=np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("red", "u1"),
                ("green", "u1"),
                ("blue", "u1"),
            ]
        ),
    )
    vertices["x"] = points[:, 0]
    vertices["y"] = points[:, 1]
    vertices["z"] = points[:, 2]
    vertices["red"] = colors[:, 0]
    vertices["green"] = colors[:, 1]
    vertices["blue"] = colors[:, 2]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment optFlow_slam passive flight recorder\n"
        f"element vertex {len(vertices)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    with temporary.open("wb") as output:
        output.write(header)
        output.write(vertices.tobytes())
        output.flush()
    temporary.replace(path)


class VoxelMap:
    """Bounded incremental colored voxel map."""

    def __init__(
        self, voxel_size_m: float = 0.08, max_voxels: int = 750_000
    ) -> None:
        if voxel_size_m <= 0:
            raise ValueError("voxel_size_m must be positive")
        if max_voxels <= 0:
            raise ValueError("max_voxels must be positive")
        self.voxel_size_m = float(voxel_size_m)
        self.max_voxels = int(max_voxels)
        self._voxels: dict[
            tuple[int, int, int], tuple[np.ndarray, np.ndarray, int]
        ] = {}
        self.rejected_new_voxels = 0

    def __len__(self) -> int:
        return len(self._voxels)

    def add(self, points_m: np.ndarray, colors_rgb: np.ndarray) -> None:
        points = np.asarray(points_m, dtype=np.float64)
        colors = np.asarray(colors_rgb, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_m must have shape (N, 3)")
        if colors.shape != (len(points), 3):
            raise ValueError("colors_rgb must have shape (N, 3)")

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        colors = colors[finite]
        if not len(points):
            return

        keys = np.floor(points / self.voxel_size_m).astype(np.int32)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        counts = np.bincount(inverse)
        point_sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        color_sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(point_sums, inverse, points)
        np.add.at(color_sums, inverse, colors)

        for key_array, point_sum, color_sum, count in zip(
            unique_keys, point_sums, color_sums, counts
        ):
            key = tuple(int(value) for value in key_array)
            previous = self._voxels.get(key)
            if previous is None:
                if len(self._voxels) >= self.max_voxels:
                    self.rejected_new_voxels += 1
                    continue
                self._voxels[key] = (
                    point_sum,
                    color_sum,
                    int(count),
                )
                continue
            self._voxels[key] = (
                previous[0] + point_sum,
                previous[1] + color_sum,
                previous[2] + int(count),
            )

    def cloud(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._voxels:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint8),
            )
        points = np.empty((len(self._voxels), 3), dtype=np.float32)
        colors = np.empty((len(self._voxels), 3), dtype=np.uint8)
        for index, (point_sum, color_sum, count) in enumerate(
            self._voxels.values()
        ):
            points[index] = point_sum / count
            colors[index] = np.clip(
                np.rint(color_sum / count), 0, 255
            ).astype(np.uint8)
        return points, colors

    def write(self, path: Path) -> None:
        points, colors = self.cloud()
        write_binary_ply(path, points, colors)
