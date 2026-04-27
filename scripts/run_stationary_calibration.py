#!/usr/bin/env python3

import os
import sys
from pathlib import Path


def main() -> None:
    script = Path(__file__).resolve().with_name("stationary_slam_calibrate.py")
    os.execv(sys.executable, [sys.executable, str(script), *sys.argv[1:]])


if __name__ == "__main__":
    main()
