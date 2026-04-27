#!/usr/bin/env python3
"""Hesai JT16 Mini Obstacle Awareness and Avoidance Node.
Filters points to 7m, calculates 8 zones, and resists motion < 2m.
Includes GCS reporting while disarmed and Airborne-only commands.
"""

import argparse
import math
import os
import sys
import time
import yaml
import statistics
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.lidar import LidarReader
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

class AvoidanceNode:
    def __init__(self, config_path):
        self.config = self.load_config(config_path)
        self.lidar = None
        self.master = None
        
        # State tracking
        self.last_warn_time = 0
        self.last_pulse_time = 0
        self.last_beep_time = 0
        self.is_stale = True
        self.last_heartbeat = 0
        self.armed = False
        self.flight_mode = "UNKNOWN"
        self.landed_state = 0 # 1=On Ground, 2=In Air
        self.rc_channels = [1500] * 16
        
        # Tunes (MML format)
        self.TUNE_SINGLE_BEEP = "MFT240L8G"
        self.TUNE_RAPID_BEEPS = "MFT240L16GP16GP16G"
        
        self.zone_names = [
            "Front", "Front-Right", "Right", "Rear-Right", 
            "Rear", "Rear-Left", "Left", "Front-Left"
        ]
        
    def load_config(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def connect_mavlink(self, port, baud=115200):
        if not mavutil: return
        try:
            print(f"Connecting to MAVLink on {port}...")
            self.master = mavutil.mavlink_connection(port, baud=baud)
            print("MAVLink connection initialized.")
        except Exception as e:
            print(f"MAVLink connection failed: {e}")

    def get_zone_idx(self, angle_deg):
        angle = (angle_deg + 22.5) % 360
        return int(angle // 45)

    def update_fc_state(self):
        if not self.master: return
        msg = self.master.recv_match(type=['HEARTBEAT', 'RC_CHANNELS', 'EXTENDED_SYS_STATE'], blocking=False)
        while msg:
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                self.last_heartbeat = time.time()
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.flight_mode = mavutil.mode_string_v10(msg)
            elif mtype == 'RC_CHANNELS':
                for i in range(1, 9):
                    val = getattr(msg, f'chan{i}_raw', 1500)
                    self.rc_channels[i-1] = val
            elif mtype == 'EXTENDED_SYS_STATE':
                self.landed_state = msg.landed_state
            msg = self.master.recv_match(type=['HEARTBEAT', 'RC_CHANNELS', 'EXTENDED_SYS_STATE'], blocking=False)

    def is_safe_to_move(self):
        now = time.time()
        if not self.config['avoidance']['enable_avoidance_motion']: return False
        if self.config['avoidance']['dry_run']: return False
        if now - self.last_heartbeat > 2.0: return False
        if not self.armed: return False
        
        # REQUIRE AIRBORNE for actual commands
        # landed_state 2 is MAV_LANDED_STATE_IN_AIR
        if self.landed_state != 2: return False 
        
        if self.flight_mode not in self.config['avoidance']['allowed_modes']: return False
        if self.is_stale: return False
        return True

    def run(self, mavlink_port=None):
        if mavlink_port:
            self.connect_mavlink(mavlink_port)

        lc = self.config['lidar']
        tick_rate = self.config['avoidance']['tick_rate_hz']
        period = 1.0 / tick_rate
        
        print(f"Avoidance logic active at {tick_rate}Hz. Dry-run: {self.config['avoidance']['dry_run']}")

        while True:
            try:
                if self.lidar is None:
                    print(f"Attempting to open JT16 Lidar on {lc['lidar_port']}...")
                    self.lidar = LidarReader.open(
                        port=lc['lidar_port'],
                        baud=lc['lidar_baud'],
                        min_valid_distance_m=lc['min_valid_distance_m'],
                        max_valid_distance_m=lc['max_detection_range_m']
                    )
                    print("LiDAR connection established.")

                start_time = time.time()
                self.update_fc_state()
                
                snap = self.lidar.poll(duration_s=0.02)
                now = time.time()
                
                if snap.point_packets == 0:
                    if not self.is_stale and (now - snap.timestamp_s > lc['stale_timeout_sec']):
                        self.is_stale = True
                        self.send_statustext("LiDAR data stale. Avoidance disabled.", mavutil.mavlink.MAV_SEVERITY_WARNING)
                    continue
                else:
                    if self.is_stale:
                        self.is_stale = False
                        self.send_statustext("LiDAR data restored.", mavutil.mavlink.MAV_SEVERITY_INFO)

                zone_min_dist = [float('inf')] * 8
                sector_size = 360.0 / len(snap.sector_distances_m)
                
                for i, dist in enumerate(snap.sector_distances_m):
                    if dist <= 0 or dist > lc['max_detection_range_m']: continue
                    angle = i * sector_size
                    z_idx = self.get_zone_idx(angle)
                    if dist < zone_min_dist[z_idx]:
                        zone_min_dist[z_idx] = dist

                zone_min_dist = [d if d != float('inf') else 0 for d in zone_min_dist]
                closest = min([d for d in zone_min_dist if d > 0] or [0])

                vx_avoid = 0.0
                vy_avoid = 0.0
                danger_m = lc['danger_distance_m']
                
                for i, d in enumerate(zone_min_dist):
                    if 0 < d < danger_m:
                        angle_rad = math.radians(i * 45)
                        weight = (danger_m - d) / danger_m
                        vx_avoid -= math.cos(angle_rad) * weight
                        vy_avoid -= math.sin(angle_rad) * weight

                mag = math.hypot(vx_avoid, vy_avoid)
                intended_pitch_deg = 0.0
                intended_roll_deg = 0.0

                if mag > 0:
                    scale = min(mag, 1.0) * self.config['avoidance']['max_velocity_cmd_mps']
                    vx_avoid = (vx_avoid / mag) * scale
                    vy_avoid = (vy_avoid / mag) * scale
                    
                    # Calculate Intended Tilt for display
                    # max_vel (0.4) maps to max_pitch (0.12 rad ~ 6.8 deg)
                    max_tilt_deg = math.degrees(self.config['avoidance']['max_pitch_cmd'])
                    tilt_scale = scale / self.config['avoidance']['max_velocity_cmd_mps']
                    
                    intended_pitch_deg = -(vx_avoid / scale) * max_tilt_deg * tilt_scale
                    intended_roll_deg = (vy_avoid / scale) * max_tilt_deg * tilt_scale

                # Audio & Status reporting (Allowed while disarmed)
                self.handle_feedback(closest, zone_min_dist, intended_pitch_deg, intended_roll_deg)

                # Command logic (Gate: Airborne only)
                if (vx_avoid != 0 or vy_avoid != 0) and self.is_safe_to_move():
                    if now - self.last_pulse_time > (self.config['avoidance']['cooldown_ms'] / 1000.0):
                        print(f"AVOID: Dist={closest:.1f}m | Intended Tilt: P={intended_pitch_deg:+.1f}° R={intended_roll_deg:+.1f}° | Sending Pulse")
                        self.send_velocity_pulse(vx_avoid, vy_avoid)
                        self.last_pulse_time = now
                elif (vx_avoid != 0 or vy_avoid != 0):
                    # Show intention even if blocked by safety gate
                    reason = "DISARMED" if not self.armed else "ON GROUND" if self.landed_state != 2 else "MODE"
                    print(f"INTENT: Dist={closest:.1f}m | Tilt: P={intended_pitch_deg:+.1f}° R={intended_roll_deg:+.1f}° | Blocked: {reason}")

                self.send_obstacle_distance(zone_min_dist)

                elapsed = time.time() - start_time
                time.sleep(max(0, period - elapsed))

            except RuntimeError as e:
                print(f"LiDAR Error: {e}. Retrying in 5s...")
                self.lidar = None
                time.sleep(5.0)
            except Exception as e:
                print(f"Unexpected Error: {e}. Retrying in 5s...")
                time.sleep(5.0)
            except KeyboardInterrupt:
                print("\nShutting down.")
                break

        if self.lidar:
            self.lidar.close()

    def handle_feedback(self, closest, zones, pitch_deg, roll_deg):
        now = time.time()
        danger = self.config['lidar']['danger_distance_m']
        
        # Audio Beeps
        if 0 < closest < 1.2:
            if now - self.last_beep_time > 0.5: # Rapid
                self.send_tune(self.TUNE_RAPID_BEEPS)
                self.last_beep_time = now
        elif 0 < closest < danger:
            if now - self.last_beep_time > 1.5: # Single
                self.send_tune(self.TUNE_SINGLE_BEEP)
                self.last_beep_time = now

        # GCS Warnings (even if disarmed)
        if now - self.last_warn_time > 3.0:
            if 0 < closest < danger:
                z_idx = zones.index(closest)
                name = self.zone_names[z_idx]
                msg = f"LiDAR DANGER: {name} {closest:.1f}m. Intention: P{pitch_deg:.0f} R{roll_deg:.0f}"
                self.send_statustext(msg, mavutil.mavlink.MAV_SEVERITY_CRITICAL)
                self.last_warn_time = now
            elif 0 < closest < self.config['lidar']['warning_distance_m']:
                z_idx = zones.index(closest)
                name = self.zone_names[z_idx]
                self.send_statustext(f"LiDAR Warning: {name} {closest:.1f}m", mavutil.mavlink.MAV_SEVERITY_WARNING)
                self.last_warn_time = now

    def send_tune(self, tune_str):
        if not self.master: return
        try:
            self.master.mav.play_tune_send(1, 1, tune_str.encode('utf-8'))
        except: pass

    def send_statustext(self, text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        if not self.master: return
        print(f"\nGCS: {text}")
        try:
            self.master.mav.statustext_send(severity, text.encode('utf-8'))
        except: pass

    def send_velocity_pulse(self, vx, vy):
        if not self.master: return
        try:
            mask = 0b0000110111000111 
            self.master.mav.set_position_target_local_ned_send(
                0, 1, 1, mavutil.mavlink.MAV_FRAME_BODY_NED, mask,
                0, 0, 0, vx, vy, 0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f"Failed to send velocity: {e}")

    def send_obstacle_distance(self, zones):
        if not self.master: return
        distances_cm = []
        for d in zones:
            val = int(d * 100) if d > 0 else 65535
            for _ in range(9): distances_cm.append(val)
        try:
            self.master.mav.obstacle_distance_send(
                int(time.time() * 1e6),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                distances_cm, 5,
                int(self.config['lidar']['min_valid_distance_m'] * 100),
                int(self.config['lidar']['max_detection_range_m'] * 100),
                0
            )
        except: pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/sensors.yaml")
    parser.add_argument("--mavport", default="/dev/ttyACM1")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    # Handle relative config path if moved
    cfg_path = Path(args.config)
    if not cfg_path.exists():
        cfg_path = REPO_ROOT / "config" / "sensors.yaml"

    node = AvoidanceNode(str(cfg_path))
    if args.enable_motion:
        node.config['avoidance']['enable_avoidance_motion'] = True
        node.config['avoidance']['dry_run'] = False
    if args.dry_run:
        node.config['avoidance']['dry_run'] = True

    node.run(mavlink_port=args.mavport)
