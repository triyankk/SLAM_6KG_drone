#!/usr/bin/env python3
"""Run the optFlow_slam readiness probe without installing the package."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())

