#!/usr/bin/env python3
"""Report the JT16 serial state; Ethernet setup is intentionally retired."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.config import ConfigError, load_config  # noqa: E402
from optflow_slam.paths import CONFIG_DIR  # noqa: E402
from optflow_slam.readiness import probe_lidar  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=CONFIG_DIR / "system.yaml"
    )
    args = parser.parse_args()
    try:
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    result = probe_lidar(config)
    print(
        json.dumps(
            {
                "transport": config.lidar.transport,
                "endpoint": config.lidar.symlink,
                "available": result.available,
                "detail": result.detail,
                "metrics": result.metrics,
                "legacy_ethernet_profile_used": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if result.available else 2


if __name__ == "__main__":
    raise SystemExit(main())
