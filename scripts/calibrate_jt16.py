#!/usr/bin/env python3
"""Project-local launcher for guided JT16 calibration."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.jt16_calibration import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
