#!/usr/bin/env python3
"""Run the project-local forward RGB camera stream."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.camera_server import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
