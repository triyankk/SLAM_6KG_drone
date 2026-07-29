#!/usr/bin/env python3
"""Show the arm-triggered flight logger status."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.flight_service import status_main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(status_main())
