#!/usr/bin/env python3
"""Pre-flight readiness check for SLAM/VIO autostart.

Checks:
- presence of extrinsics JSON
- calibration CSV has sufficient rows
- external IMU probe (calls existing check_external_imu.py)
- MAVLink heartbeat on configured FC ports

If all checks pass, the script will update `config/autostart.yaml` to
set `source: vio` and `connect_in_standby: false` so the bridge will run
live on boot. Requires write permissions to repo files (no sudo required
for file edit). Restarting systemd service is left to the operator.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path
import time

import yaml

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


def check_file_exists(path: Path) -> bool:
    return path.exists()


def count_csv_rows(path: Path) -> int:
    try:
        with path.open('r', encoding='utf-8') as fh:
            # minus header
            return sum(1 for _ in fh) - 1
    except Exception:
        return 0


def run_imu_probe(scan_seconds: float = 1.0) -> bool:
    script = Path(__file__).parent.parent / 'scripts' / 'check_external_imu.py'
    if not script.exists():
        print('IMU probe script not found:', script)
        return False
    try:
        # allow longer timeout for probe to complete on slower systems
        res = subprocess.run(['python3', str(script), '--port', 'auto', '--scan-seconds', str(scan_seconds)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        print(res.stdout.decode('utf-8', errors='ignore'))
        return res.returncode == 0
    except subprocess.TimeoutExpired:
        print('IMU probe timed out')
        return False
    except Exception as e:
        print('IMU probe error:', e)
        return False


def check_mavlink_heartbeats(ports, timeout_s=5.0) -> bool:
    if not mavutil:
        print('pymavlink not available; cannot check MAVLink heartbeats')
        return False
    success = False
    for p in ports:
        try:
            print('Trying MAVLink on', p)
            m = mavutil.mavlink_connection(p, baud=115200, autoreconnect=False)
            start = time.time()
            while time.time() - start < timeout_s:
                hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if hb is not None:
                    print('Heartbeat from', p, 'sysid=', hb.get_srcSystem(), 'compid=', hb.get_srcComponent())
                    success = True
                    m.close()
                    break
            else:
                print('No heartbeat on', p)
                m.close()
        except Exception as e:
            print('MAVLink connect error on', p, ':', e)
    return success


def update_autostart_config(path: Path) -> bool:
    try:
        with path.open('r', encoding='utf-8') as fh:
            cfg = yaml.safe_load(fh) or {}
    except Exception:
        cfg = {}
    cfg['source'] = 'vio'
    cfg['connect_in_standby'] = False
    try:
        with path.open('w', encoding='utf-8') as fh:
            yaml.safe_dump(cfg, fh)
        print('Updated', path)
        return True
    except Exception as e:
        print('Failed to write', path, e)
        return False


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--config', default='config/autostart.yaml', help='Autostart config to update')
    p.add_argument('--calibration', default='calibration_run.csv', help='Calibration CSV path to check')
    p.add_argument('--extrinsics', default='camera_to_imu_extrinsics.json', help='Extrinsics JSON')
    p.add_argument('--ports', nargs='*', help='MAVLink ports to check (overrides config)')
    return p.parse_args()


def main():
    args = parse_args()
    extr_path = Path(args.extrinsics)
    calib_path = Path(args.calibration)
    autostart_path = Path(args.config)

    ok = True

    print('Checking extrinsics file:', extr_path)
    if not check_file_exists(extr_path):
        print('Missing extrinsics file')
        ok = False

    print('Checking calibration CSV rows:', calib_path)
    rows = count_csv_rows(calib_path)
    print('Rows:', rows)
    if rows < 20:
        print('Insufficient calibration rows (<20)')
        ok = False

    print('Probing external IMU...')
    if not run_imu_probe(1.0):
        print('IMU probe failed')
        ok = False

    # MAVLink ports to check
    ports = args.ports or []
    if not ports:
        # try to read ports from config/default.yaml
        cfgp = Path('config/default.yaml')
        if cfgp.exists():
            try:
                with cfgp.open('r', encoding='utf-8') as fh:
                    cfg = yaml.safe_load(fh) or {}
                    ports = cfg.get('ports', [])
            except Exception:
                ports = []

    if ports:
        print('Checking MAVLink ports:', ports)
        if not check_mavlink_heartbeats(ports, timeout_s=5.0):
            print('MAVLink checks failed')
            ok = False
    else:
        print('No MAVLink ports provided or found; skipping MAVLink check')

    if not ok:
        print('\nPreflight checks failed')
        sys.exit(2)

    # All checks passed: update autostart config
    print('\nAll checks passed — updating autostart to enable live VIO')
    if update_autostart_config(Path(args.config)):
        print('Autostart config updated. To apply live autostart, restart the bridge service or run the installer with sudo:')
        print('  sudo bash install_slam_bridge_autostart.sh --enable-now')
        sys.exit(0)
    else:
        print('Failed to update autostart config')
        sys.exit(3)


if __name__ == '__main__':
    main()
