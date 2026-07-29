#!/usr/bin/env python3
"""Configure the dedicated JT16 Ethernet address and host route."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from optflow_slam.config import ConfigError, load_config  # noqa: E402
from optflow_slam.paths import CONFIG_DIR  # noqa: E402


PROFILE_NAME = "optflow-jt16"


def _run(*arguments: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(
        arguments,
        check=check,
        text=True,
        capture_output=True,
    )


def _carrier(interface: str) -> bool:
    try:
        return (
            Path(f"/sys/class/net/{interface}/carrier")
            .read_text(encoding="ascii")
            .strip()
            == "1"
        )
    except OSError:
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=CONFIG_DIR / "system.yaml"
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    lidar = config.lidar
    interface_path = Path("/sys/class/net") / lidar.ethernet_interface
    if not interface_path.exists():
        print(f"Ethernet interface is absent: {lidar.ethernet_interface}")
        return 2

    address = f"{lidar.jetson_ip}/{lidar.jetson_prefix_length}"
    host_route = f"{lidar.lidar_ip}/32"
    existing = _run(
        "nmcli",
        "-t",
        "-f",
        "NAME",
        "connection",
        "show",
        check=False,
    )
    names = existing.stdout.splitlines()
    if PROFILE_NAME not in names:
        _run(
            "nmcli",
            "connection",
            "add",
            "type",
            "ethernet",
            "ifname",
            lidar.ethernet_interface,
            "con-name",
            PROFILE_NAME,
        )

    _run(
        "nmcli",
        "connection",
        "modify",
        PROFILE_NAME,
        "connection.interface-name",
        lidar.ethernet_interface,
        "connection.autoconnect",
        "yes",
        "connection.autoconnect-priority",
        "100",
        "ipv4.method",
        "manual",
        "ipv4.addresses",
        address,
        "ipv4.gateway",
        "",
        "ipv4.dns",
        "",
        "ipv4.never-default",
        "yes",
        "ipv4.routes",
        host_route,
        "ipv6.method",
        "disabled",
    )

    payload = {
        "profile": PROFILE_NAME,
        "interface": lidar.ethernet_interface,
        "address": address,
        "lidar_host_route": host_route,
        "carrier": _carrier(lidar.ethernet_interface),
    }
    if not payload["carrier"]:
        print(json.dumps(payload, indent=2))
        print("Profile saved; connect and power the JT16 to activate it.")
        return 3

    activated = _run(
        "nmcli",
        "connection",
        "up",
        PROFILE_NAME,
        check=False,
    )
    if activated.returncode != 0:
        print(activated.stderr.strip() or activated.stdout.strip())
        return 2
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
