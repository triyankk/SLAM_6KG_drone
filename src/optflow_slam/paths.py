"""Canonical paths contained by the optFlow_slam project boundary."""

from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config"
DATA_DIR = PROJECT_ROOT / "data"
CALIBRATION_DIR = DATA_DIR / "calibrations"
LOG_DIR = DATA_DIR / "logs"
MAP_DIR = DATA_DIR / "maps"
RECORDING_DIR = DATA_DIR / "recordings"
RUNTIME_DIR = PROJECT_ROOT / "runtime"
ROS_WORKSPACE_DIR = PROJECT_ROOT / "ros_ws"
THIRD_PARTY_DIR = PROJECT_ROOT / "third_party"
VISUALIZER_DIR = PROJECT_ROOT / "visualizer"


def ensure_runtime_directories() -> None:
    """Create project-owned writable directories when a process needs them."""

    for directory in (
        CALIBRATION_DIR,
        LOG_DIR,
        MAP_DIR,
        RECORDING_DIR,
        RUNTIME_DIR,
    ):
        directory.mkdir(parents=True, exist_ok=True)
