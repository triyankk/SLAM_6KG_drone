#!/usr/bin/env python3
"""Visualization script for JT16 Mini LiDAR Avoidance.
Shows 8 obstacle zones, detection/danger boundaries, and avoidance vector.
Now includes a dashboard for Pitch/Roll intentions.
"""

import argparse
import math
import os
import sys
import time
import yaml
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.lidar import LidarReader

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def get_zone_idx(angle_deg):
    angle = (angle_deg + 22.5) % 360
    return int(angle // 45)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/sensors.yaml")
    parser.add_argument("--window-size", type=int, default=800)
    args = parser.parse_args()

    # Handle relative config path if moved
    cfg_path = Path(args.config)
    if not cfg_path.exists():
        cfg_path = REPO_ROOT / "config" / "sensors.yaml"

    config = load_config(cfg_path)
    lc = config['lidar']
    av = config['avoidance']

    print(f"Opening Lidar for visualization: {lc['lidar_port']}...")
    reader = LidarReader.open(
        port=lc['lidar_port'],
        baud=lc['lidar_baud'],
        min_valid_distance_m=lc['min_valid_distance_m'],
        max_valid_distance_m=lc['max_detection_range_m']
    )

    size = args.window_size
    center = size // 2
    # Scale: max_detection_range_m maps to 90% of half-window size
    scale = (size * 0.45) / lc['max_detection_range_m']

    cv2.namedWindow("JT16 Avoidance View", cv2.WINDOW_NORMAL)

    try:
        while True:
            snap = reader.poll(duration_s=0.05)
            canvas = np.zeros((size, size + 300, 3), dtype=np.uint8) # Extra width for dashboard

            # Draw boundaries
            cv2.circle(canvas, (center, center), int(lc['max_detection_range_m'] * scale), (40, 40, 40), 1)
            cv2.circle(canvas, (center, center), int(lc['danger_distance_m'] * scale), (0, 0, 180), 2)
            cv2.circle(canvas, (center, center), int(lc['warning_distance_m'] * scale), (0, 120, 120), 1)

            # Draw zone dividers
            for i in range(8):
                angle_rad = math.radians(i * 45 - 22.5)
                end_x = int(center + math.cos(angle_rad) * size)
                end_y = int(center + math.sin(angle_rad) * size)
                cv2.line(canvas, (center, center), (end_x, end_y), (30, 30, 30), 1)

            # Process zones
            zone_min_dist = [float('inf')] * 8
            sector_size = 360.0 / len(snap.sector_distances_m)
            
            vx_avoid = 0.0
            vy_avoid = 0.0
            danger_m = lc['danger_distance_m']

            for i, dist in enumerate(snap.sector_distances_m):
                if dist <= 0: continue
                angle_deg = i * sector_size
                angle_rad = math.radians(angle_deg)
                
                px = int(center + math.cos(angle_rad) * dist * scale)
                py = int(center + math.sin(angle_rad) * dist * scale)
                
                color = (0, 255, 0) 
                if dist < danger_m: color = (0, 0, 255) 
                elif dist < lc['warning_distance_m']: color = (0, 255, 255) 
                
                cv2.circle(canvas, (px, py), 2, color, -1)

                z_idx = get_zone_idx(angle_deg)
                if dist < zone_min_dist[z_idx]:
                    zone_min_dist[z_idx] = dist
                
                if dist < danger_m:
                    weight = (danger_m - dist) / danger_m
                    vx_avoid -= math.cos(angle_rad) * weight
                    vy_avoid -= math.sin(angle_rad) * weight

            # Avoidance Vector & Dashboard Data
            mag = math.hypot(vx_avoid, vy_avoid)
            p_intent = 0
            r_intent = 0
            if mag > 0:
                vx_draw = (vx_avoid / mag) * 100 
                vy_draw = (vy_avoid / mag) * 100
                cv2.arrowedLine(canvas, (center, center), (int(center + vx_draw), int(center + vy_draw)), (255, 255, 0), 3)
                
                # Intent calculation (same as node)
                max_tilt = math.degrees(av['max_pitch_cmd'])
                res_scale = min(mag, 1.0) * (av['max_velocity_cmd_mps'] / av['max_velocity_cmd_mps'])
                p_intent = -(vx_avoid / mag) * max_tilt * min(mag, 1.0)
                r_intent = (vy_avoid / mag) * max_tilt * min(mag, 1.0)

            # --- Dashboard ---
            dx = size + 20
            # Pitch Bar
            cv2.putText(canvas, "PITCH INTENT", (dx, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.rectangle(canvas, (dx, 50), (dx + 260, 70), (40, 40, 40), -1)
            p_px = int(130 + (p_intent / max_tilt) * 130)
            cv2.rectangle(canvas, (dx + 130, 50), (dx + p_px, 70), (0, 255, 255), -1)
            cv2.putText(canvas, f"{p_intent:+.1f} deg", (dx + 100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Roll Bar
            cv2.putText(canvas, "ROLL INTENT", (dx, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.rectangle(canvas, (dx, 140), (dx + 260, 160), (40, 40, 40), -1)
            r_px = int(130 + (r_intent / max_tilt) * 130)
            cv2.rectangle(canvas, (dx + 130, 140), (dx + r_px, 160), (0, 255, 255), -1)
            cv2.putText(canvas, f"{r_intent:+.1f} deg", (dx + 100, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Status List
            now = time.time()
            stale = (now - snap.timestamp_s) > lc['stale_timeout_sec']
            status_text = "HEALTHY" if not stale else "STALE"
            status_color = (0, 255, 0) if not stale else (0, 0, 255)
            
            cv2.putText(canvas, f"Lidar: {status_text}", (dx, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1)
            cv2.putText(canvas, f"Motion Enabled: {av['enable_avoidance_motion']}", (dx, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(canvas, f"Dry Run: {av['dry_run']}", (dx, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # Zone distances
            zone_names = ["Front", "F-Right", "Right", "R-Right", "Rear", "R-Left", "Left", "F-Left"]
            for i, d in enumerate(zone_min_dist):
                d_val = f"{d:.2f}m" if d != float('inf') else "None"
                color = (200, 200, 200)
                if d < danger_m: color = (0, 0, 255)
                cv2.putText(canvas, f"{zone_names[i]}: {d_val}", (dx + 140, 350 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            cv2.imshow("JT16 Avoidance View", canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        reader.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
