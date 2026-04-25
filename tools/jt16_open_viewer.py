#!/usr/bin/env python3

import argparse
import os
import platform
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PANDARVIEW_DIR = REPO_ROOT / "hardware" / "jt16_vendor" / "PandarView2"
PANDARVIEW_BINARY = PANDARVIEW_DIR / "bin" / "PandarView"
BUILTIN_VIEWER = REPO_ROOT / "tools" / "jt16_live_view.py"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Open a JT16 viewer. On x86_64 Linux, prefer the bundled PandarView2 if it is "
            "installed. On Jetson/ARM64 or when PandarView is unavailable, fall back to the "
            "repo's built-in JT16 live viewer."
        )
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument(
        "--viewer",
        choices=["auto", "pandar", "builtin"],
        default="auto",
        help="Choose the viewer explicitly. Default: auto.",
    )
    parser.add_argument(
        "--no-sysctl",
        action="store_true",
        help="When launching PandarView, skip the vendor sysctl step and just run the binary.",
    )
    return parser.parse_args()


def pandarview_supported_here() -> tuple[bool, str]:
    if not PANDARVIEW_BINARY.exists():
        return False, "PandarView is not bundled in hardware/jt16_vendor/PandarView2."

    machine = platform.machine().lower()
    if machine not in ("x86_64", "amd64"):
        return False, f"PandarView bundle is x86_64, but this machine is {machine}."

    return True, "PandarView is installed and matches this machine architecture."


def launch_pandarview(args):
    env = os.environ.copy()
    lib_dir = PANDARVIEW_DIR / "lib"
    env["LD_LIBRARY_PATH"] = f"{env.get('LD_LIBRARY_PATH', '')}:{lib_dir}:{lib_dir / 'Qt'}".strip(":")
    env["QT_PLUGIN_PATH"] = f"{env.get('QT_PLUGIN_PATH', '')}:{PANDARVIEW_DIR / 'plugins'}".strip(":")
    env["QML2_IMPORT_PATH"] = f"{env.get('QML2_IMPORT_PATH', '')}:{PANDARVIEW_DIR / 'qml'}".strip(":")

    if args.no_sysctl:
        cmd = [str(PANDARVIEW_BINARY)]
        cwd = str(PANDARVIEW_DIR)
    else:
        cmd = [str(PANDARVIEW_DIR / "PandarView.sh")]
        cwd = str(PANDARVIEW_DIR)

    print(f"Launching PandarView from {PANDARVIEW_DIR}")
    print("Note: PandarView is vendor software and may still require correct JT16 data wiring to show points.")
    return subprocess.call(cmd, cwd=cwd, env=env)


def launch_builtin(args):
    cmd = [
        sys.executable,
        str(BUILTIN_VIEWER),
        "--port",
        args.port,
        "--baud",
        str(args.baud),
    ]
    print(f"Launching built-in JT16 viewer on {args.port} at {args.baud} baud")
    return subprocess.call(cmd, cwd=str(REPO_ROOT / "tools"))


def main():
    args = parse_args()
    pandar_ok, pandar_reason = pandarview_supported_here()

    if args.viewer == "pandar":
        if not pandar_ok:
            raise SystemExit(f"Cannot launch PandarView: {pandar_reason}")
        raise SystemExit(launch_pandarview(args))

    if args.viewer == "builtin":
        raise SystemExit(launch_builtin(args))

    if pandar_ok:
        raise SystemExit(launch_pandarview(args))

    print(f"PandarView unavailable here: {pandar_reason}")
    print("Falling back to the built-in JT16 viewer.")
    raise SystemExit(launch_builtin(args))


if __name__ == "__main__":
    main()
