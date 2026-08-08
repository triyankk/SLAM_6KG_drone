#!/usr/bin/env python3
"""Run the project-owned Cube obstacle-avoidance parameter tool."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.cube_avoidance import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
