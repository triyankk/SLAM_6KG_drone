#!/usr/bin/env python3
"""Analyze sensor clocks in a passive flight session."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.slam_timing import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
