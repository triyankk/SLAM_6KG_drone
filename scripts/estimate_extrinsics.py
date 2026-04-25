#!/usr/bin/env python3
"""
Estimate camera->IMU rotation from paired VIO pose and IMU RPY samples saved by
`scripts/collect_calibration_data.py`.

This is a conservative starter: it computes the mean relative rotation between
the VIO quaternion and the IMU orientation (converted from roll/pitch/yaw) and
prints a single quaternion estimate `q_imu = q_extr * q_vio` so
`q_extr = q_imu * q_vio^{-1}`.

This is not a full hand-eye solver but a practical first step for aligning
frames prior to a more careful optimization.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz], dtype=float)


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_mul(a, b):
    # Hamilton product a * b
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def normalize(q):
    n = np.linalg.norm(q)
    if n == 0:
        return q
    return q / n


def average_quaternions(quats: np.ndarray):
    # Classic method: form symmetric accumulator and take principal eigenvector
    M = np.zeros((4, 4), dtype=float)
    for q in quats:
        qn = q.reshape(4, 1)
        M += qn @ qn.T
    M /= len(quats)
    eigvals, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, np.argmax(eigvals)]
    return normalize(q_mean)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="infile", required=True, help="CSV from collect_calibration_data.py")
    return p.parse_args()


def main():
    args = parse_args()
    path = Path(args.infile)
    if not path.exists():
        print("Input CSV not found:", path)
        sys.exit(2)

    rel_quats = []
    with path.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                qw = float(row.get("qw", 0.0))
                qx = float(row.get("qx", 0.0))
                qy = float(row.get("qy", 0.0))
                qz = float(row.get("qz", 0.0))
                roll = row.get("roll_deg", "")
                if roll == "":
                    continue
                roll = float(roll)
                pitch = float(row.get("pitch_deg", 0.0))
                yaw = float(row.get("yaw_deg", 0.0))
            except Exception:
                continue

            q_vio = np.array([qw, qx, qy, qz], dtype=float)
            q_imu = rpy_to_quat(roll, pitch, yaw)

            q_rel = quat_mul(q_imu, quat_conjugate(q_vio))
            q_rel = normalize(q_rel)
            rel_quats.append(q_rel)

    if not rel_quats:
        print("No matching rows with IMU RPY found in CSV")
        sys.exit(1)

    rel_quats_np = np.vstack(rel_quats)
    q_mean = average_quaternions(rel_quats_np)
    print("Estimated camera->IMU rotation (mean quaternion w,x,y,z):")
    print(f"{q_mean[0]:+.6f} {q_mean[1]:+.6f} {q_mean[2]:+.6f} {q_mean[3]:+.6f}")


if __name__ == '__main__':
    main()
