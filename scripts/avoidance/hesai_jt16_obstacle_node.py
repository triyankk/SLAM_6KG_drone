#!/usr/bin/env python3
"""Hesai JT16 Mini obstacle avoidance publisher.

Primary behavior:
- read the top-mounted JT16 scan,
- publish fresh 360 degree obstacle data to ArduPilot,
- let ArduPilot's native avoidance keep the vehicle outside the configured
  margin in pilot-controlled modes.

Optional direct companion motion is intentionally gated. It is useful for
no-prop bench testing and later GUIDED-mode experiments, but normal Loiter/
PosHold stabilization should remain inside the flight controller.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from slam_core.fc_config import send_obstacle_distance  # noqa: E402
from slam_core.lidar import LidarReader  # noqa: E402
from slam_core.obstacle_avoidance import (  # noqa: E402
    ZONE_NAMES,
    AvoidanceCommand,
    compute_keepout_velocity,
    reduce_to_zones,
    velocity_to_tilt_deg,
)

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - field systems should install pymavlink
    mavutil = None


DEFAULT_MAVPORT = "udpout:127.0.0.1:14555"
MODE_MONITOR_ONLY = "monitor_only"
MODE_NATIVE_AVOIDANCE = "native_avoidance"
MODE_RC_TOGGLE = "rc_toggle"
MODE_DIRECT_VELOCITY = "direct_velocity"
VALID_AVOIDANCE_MODES = {MODE_MONITOR_ONLY, MODE_NATIVE_AVOIDANCE, MODE_RC_TOGGLE, MODE_DIRECT_VELOCITY}


class AvoidanceNode:
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self.load_config(config_path)
        self.lidar: LidarReader | None = None
        self.master: Any | None = None
        self.gcs_status_sock: socket.socket | None = None
        self.gcs_status_mav: Any | None = None
        self.rc_downlink_sock: socket.socket | None = None
        self.rc_downlink_mav: Any | None = None

        self.started_s = time.time()
        self.last_warn_time_s = 0.0
        self.last_pulse_time_s = 0.0
        self.last_beep_time_s = 0.0
        self.last_beep_mute_notice_s = 0.0
        self.last_heartbeat_sent_s = 0.0
        self.last_native_publish_s = 0.0
        self.last_active_status_time_s = 0.0
        self.last_param_config_s = 0.0
        self.last_rc_toggle_status_s = 0.0
        self.last_lua_obstacle_status: int | None = None
        self.last_lua_obstacle_status_time_s = 0.0
        self.native_params_sent = False
        self.native_prx_disabled_sent = False
        self.last_param_values_sent: dict[str, float] = {}
        self.rc_toggle_engaged = False
        self.last_rc_update_s = 0.0
        self.last_rc_downlink_notice_s = 0.0
        self.is_stale = True
        self.last_vehicle_heartbeat_s = 0.0
        self.armed = False
        self.flight_mode = "UNKNOWN"
        self.landed_state = 0  # 1=on ground, 2=in air
        self.rc_channels = [1500] * 16
        self.last_block_reason = ""

        self.tune_single_beep = "MFT240L8G"
        self.tune_rapid_beeps = "MFT240L16GP16GP16G"
        self.tune_oa_engaged = "MFT220L16CEG"
        self.tune_oa_disengaged = "MFT180L16GC"

    @staticmethod
    def load_config(path: str) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        data.setdefault("lidar", {})
        data.setdefault("avoidance", {})
        return data

    @property
    def lidar_config(self) -> dict[str, Any]:
        return self.config["lidar"]

    @property
    def avoidance_config(self) -> dict[str, Any]:
        return self.config["avoidance"]

    def avoidance_mode(self) -> str:
        raw_mode = str(self.avoidance_config.get("mode", "")).strip().lower().replace("-", "_")
        if raw_mode in VALID_AVOIDANCE_MODES:
            return raw_mode
        if self.avoidance_config.get("use_ardupilot_native_avoidance", True):
            return MODE_NATIVE_AVOIDANCE
        if self.avoidance_config.get("enable_avoidance_motion", False) and not self.avoidance_config.get("dry_run", True):
            return MODE_DIRECT_VELOCITY
        return MODE_MONITOR_ONLY

    def rc_toggle_config(self) -> dict[str, Any]:
        config = self.avoidance_config.get("rc_toggle", {})
        return config if isinstance(config, dict) else {}

    def rc_toggle_enabled(self) -> bool:
        toggle = self.rc_toggle_config()
        return self.avoidance_mode() == MODE_RC_TOGGLE or bool(toggle.get("enable", False))

    def rc_toggle_pwm(self) -> int:
        channel = int(self.rc_toggle_config().get("channel", 7))
        if channel < 1 or channel > len(self.rc_channels):
            return 0
        return int(self.rc_channels[channel - 1])

    def native_avoidance_enabled(self) -> bool:
        if self.rc_toggle_enabled():
            return self.rc_toggle_engaged
        return self.avoidance_mode() == MODE_NATIVE_AVOIDANCE

    def proximity_publishing_enabled(self) -> bool:
        if self.rc_toggle_enabled():
            toggle = self.rc_toggle_config()
            return self.rc_toggle_engaged or bool(toggle.get("publish_proximity_while_disengaged", True))
        return self.avoidance_mode() == MODE_NATIVE_AVOIDANCE

    def current_tick_rate_hz(self) -> float:
        av = self.avoidance_config
        if self.rc_toggle_enabled():
            key = "engaged_tick_rate_hz" if self.rc_toggle_engaged else "detect_tick_rate_hz"
            if key in av:
                return max(float(av.get(key, av.get("tick_rate_hz", 10))), 1.0)
        return max(float(av.get("tick_rate_hz", 10)), 1.0)

    def current_native_publish_rate_hz(self) -> float:
        av = self.avoidance_config
        if self.rc_toggle_enabled():
            key = (
                "native_engaged_publish_rate_hz"
                if self.rc_toggle_engaged
                else "native_detect_publish_rate_hz"
            )
            if key in av:
                return max(float(av.get(key, av.get("native_publish_rate_hz", 10))), 0.5)
        return max(float(av.get("native_publish_rate_hz", av.get("tick_rate_hz", 10))), 0.5)

    def direct_motion_enabled(self) -> bool:
        return self.avoidance_mode() == MODE_DIRECT_VELOCITY or bool(
            self.avoidance_config.get("enable_avoidance_motion", False)
        )

    def gcs_status_targets(self) -> list[tuple[str, int]]:
        raw_targets = self.avoidance_config.get(
            "gcs_status_targets",
            ["127.0.0.1:14550", "255.255.255.255:14550", "127.0.0.1:14551", "255.255.255.255:14551"],
        )
        targets: list[tuple[str, int]] = []
        for value in raw_targets:
            text = str(value)
            if ":" not in text:
                continue
            host, port_text = text.rsplit(":", 1)
            try:
                port = int(port_text)
            except ValueError:
                continue
            if host and 0 < port <= 65535:
                targets.append((host, port))
        return targets

    def ensure_gcs_status_sender(self) -> None:
        if self.gcs_status_sock is not None or mavutil is None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.gcs_status_sock = sock
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 190
        mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        self.gcs_status_mav = mav

    def ensure_rc_downlink_listener(self) -> None:
        if self.rc_downlink_sock is not None or mavutil is None:
            return
        port = int(self.avoidance_config.get("rc_downlink_udp_port", 14551))
        if port <= 0:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
            sock.setblocking(False)
            self.rc_downlink_sock = sock
            self.rc_downlink_mav = mavutil.mavlink.MAVLink(None)
            print(f"Listening for FC RC downlink on UDP {port}.")
        except OSError as exc:
            print(f"Unable to listen for RC downlink on UDP {port}: {exc}")
            self.rc_downlink_sock = None
            self.rc_downlink_mav = None

    def connect_mavlink(self, port: str, baud: int = 115200) -> None:
        if mavutil is None:
            print("pymavlink is not installed; running LiDAR processing without MAVLink output.")
            return
        if port.startswith("/dev/ttyACM") or port.startswith("/dev/serial/"):
            print(
                "WARNING: direct Cube USB MAVLink selected for LiDAR node. "
                "Stop the legacy bridge and any direct GCS USB session first, "
                "or use udpout:127.0.0.1:14555."
            )
        try:
            print(f"Connecting obstacle node MAVLink on {port}...")
            self.master = mavutil.mavlink_connection(port, baud=baud, source_system=190)
            self.send_companion_heartbeat(force=True)
            self.request_rc_channels_stream()
            print("MAVLink connection initialized.")
        except Exception as exc:  # noqa: BLE001
            print(f"MAVLink connection failed: {exc}")
            self.master = None

    def request_rc_channels_stream(self) -> None:
        if self.master is None or mavutil is None:
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system or 1,
                self.master.target_component or 1,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                200000,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"Unable to request RC_CHANNELS stream: {exc}")

    def send_companion_heartbeat(self, force: bool = False) -> None:
        if self.master is None or mavutil is None:
            return
        now_s = time.time()
        if not force and now_s - self.last_heartbeat_sent_s < 1.0:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            self.last_heartbeat_sent_s = now_s
        except Exception:
            return

    def is_vehicle_heartbeat(self, msg: Any) -> bool:
        if mavutil is None:
            return False
        autopilot = getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
        vehicle_type = getattr(msg, "type", None)
        if vehicle_type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return False
        return autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID

    def process_fc_message(self, msg: Any) -> None:
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            if not self.is_vehicle_heartbeat(msg):
                return
            self.last_vehicle_heartbeat_s = time.time()
            self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.flight_mode = mavutil.mode_string_v10(msg)
        elif mtype == "RC_CHANNELS":
            previous_update_s = self.last_rc_update_s
            for channel in range(1, 9):
                self.rc_channels[channel - 1] = getattr(msg, f"chan{channel}_raw", 1500)
            self.last_rc_update_s = time.time()
            if previous_update_s <= 0.0 and self.rc_toggle_enabled():
                channel = int(self.rc_toggle_config().get("channel", 7))
                self.send_statustext(
                    f"OA RC{channel} telemetry live: pwm={self.rc_toggle_pwm()}",
                    self.severity_notice(),
                )
        elif mtype == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state

    def drain_rc_downlink_listener(self) -> None:
        if self.rc_downlink_sock is None or self.rc_downlink_mav is None:
            return
        while True:
            try:
                payload, _addr = self.rc_downlink_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"RC downlink listener error: {exc}")
                self.rc_downlink_sock = None
                self.rc_downlink_mav = None
                break
            for byte_value in payload:
                try:
                    msg = self.rc_downlink_mav.parse_char(bytes([byte_value]))
                except Exception:
                    msg = None
                if msg is not None:
                    self.process_fc_message(msg)

    def update_fc_state(self) -> None:
        if mavutil is None:
            return

        if self.master is not None:
            self.send_companion_heartbeat()
            wanted = ["HEARTBEAT", "RC_CHANNELS", "EXTENDED_SYS_STATE"]
            msg = self.master.recv_match(type=wanted, blocking=False)
            while msg is not None:
                self.process_fc_message(msg)
                msg = self.master.recv_match(type=wanted, blocking=False)
        self.drain_rc_downlink_listener()

    def update_rc_toggle_state(self) -> None:
        if not self.rc_toggle_enabled():
            return
        toggle = self.rc_toggle_config()
        pwm = self.rc_toggle_pwm()
        engage_pwm = int(toggle.get("engage_pwm", 1700))
        disengage_pwm = int(toggle.get("disengage_pwm", 1300))
        previous = self.rc_toggle_engaged

        if pwm >= engage_pwm:
            self.rc_toggle_engaged = True
        elif 800 <= pwm <= disengage_pwm:
            self.rc_toggle_engaged = False

        if self.rc_toggle_engaged != previous:
            self.last_param_config_s = 0.0
            self.native_prx_disabled_sent = False
            self.last_active_status_time_s = 0.0
            state_text = (
                "OA ENGAGED by RC7: FC native avoidance active."
                if self.rc_toggle_engaged
                else "OA DETECT ONLY by RC7: avoidance disabled."
            )
            self.send_statustext(state_text, self.severity_warning())
            self.publish_lua_obstacle_status(10 if self.rc_toggle_engaged else 12, force=True)
            if self.rc_toggle_engaged and toggle.get("beep_on_engage", True):
                tune = str(toggle.get("engage_tune", self.tune_oa_engaged))
                self.send_mode_tune(tune)
            elif not self.rc_toggle_engaged and toggle.get("beep_on_disengage", False):
                tune = str(toggle.get("disengage_tune", self.tune_oa_disengaged))
                self.send_mode_tune(tune)

    def rc_toggle_status_text(self) -> str:
        toggle = self.rc_toggle_config()
        channel = int(toggle.get("channel", 7))
        if self.last_rc_update_s <= 0.0:
            return f"OA DETECT ONLY: RC{channel}=? waiting RC"
        pwm = self.rc_toggle_pwm()
        if self.rc_toggle_engaged:
            return f"OA ENGAGED: RC{channel}={pwm} avoid active"
        return f"OA DETECT ONLY: RC{channel}={pwm} scan active"

    def send_periodic_rc_toggle_status(self) -> None:
        if not self.rc_toggle_enabled():
            return
        interval_s = max(float(self.rc_toggle_config().get("status_interval_sec", 10.0)), 1.0)
        now_s = time.time()
        if now_s - self.last_rc_toggle_status_s < interval_s:
            return
        self.last_rc_toggle_status_s = now_s
        if self.last_rc_update_s <= 0.0:
            self.send_statustext("OA RC7 toggle waiting for RC_CHANNELS.", self.severity_warning())
            return
        self.send_statustext(self.rc_toggle_status_text(), self.severity_notice())

    def set_fc_param_real32(self, name: str, value: float) -> None:
        if self.master is None or mavutil is None:
            return
        try:
            self.master.mav.param_set_send(
                self.master.target_system or 1,
                self.master.target_component or 1,
                name.encode("ascii", errors="ignore")[:16],
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"Param set failed for {name}: {exc}")

    def desired_ardupilot_params(self) -> dict[str, float]:
        av = self.avoidance_config
        if self.proximity_publishing_enabled():
            params = {
                "PRX1_TYPE": float(av.get("prx1_type", 2)),
                "AVOID_ENABLE": float(av.get("avoid_enable", 7) if self.native_avoidance_enabled() else 0),
                "AVOID_MARGIN": float(av.get("avoid_margin_m", av.get("keepout_distance_m", 1.5))),
                "AVOID_BACKUP_SPD": float(av.get("avoid_backup_speed_mps", 0.8)),
                "AVOID_ACCEL_MAX": float(av.get("avoid_accel_max_mpss", 1.0)),
            }
            optional_dist_max = av.get("avoid_dist_max_m")
            if optional_dist_max is not None:
                params["AVOID_DIST_MAX"] = float(optional_dist_max)
            return params

        if av.get("disable_ardupilot_prx_when_native_disabled", False):
            params = {"PRX1_TYPE": 0.0}
            if av.get("disable_ardupilot_avoidance_when_native_disabled", False):
                params["AVOID_ENABLE"] = 0.0
            return params
        return {}

    def maybe_disable_ardupilot_prx(self) -> None:
        av = self.avoidance_config
        if self.native_prx_disabled_sent or not av.get("disable_ardupilot_prx_when_native_disabled", False):
            return
        now_s = time.time()
        if now_s - self.last_param_config_s < 5.0:
            return
        self.last_param_config_s = now_s
        if self.armed:
            if av.get("disable_ardupilot_avoidance_when_native_disabled", False):
                self.set_fc_param_real32("AVOID_ENABLE", 0.0)
                self.last_param_values_sent["AVOID_ENABLE"] = 0.0
                self.send_statustext(
                    "OA DETECT ONLY: AVOID_ENABLE=0, PRX kept while armed.",
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                )
            else:
                self.send_statustext(
                    "LiDAR PRX disable skipped while armed.",
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                )
            self.native_prx_disabled_sent = True
            return
        self.set_fc_param_real32("PRX1_TYPE", 0.0)
        self.last_param_values_sent["PRX1_TYPE"] = 0.0
        if av.get("disable_ardupilot_avoidance_when_native_disabled", False):
            self.set_fc_param_real32("AVOID_ENABLE", 0.0)
            self.last_param_values_sent["AVOID_ENABLE"] = 0.0
        self.native_prx_disabled_sent = True
        self.send_statustext(
            "LiDAR native PRX/avoidance disabled; monitoring only.",
            mavutil.mavlink.MAV_SEVERITY_NOTICE,
        )

    def maybe_configure_ardupilot_avoidance(self) -> None:
        if self.master is None or mavutil is None:
            return
        if not self.avoidance_config.get("configure_ardupilot_avoidance", True):
            return

        param_values = self.desired_ardupilot_params()
        if not param_values:
            return

        changed = {
            name: value
            for name, value in param_values.items()
            if self.last_param_values_sent.get(name) != value
        }
        if not changed:
            return
        now_s = time.time()
        if now_s - self.last_param_config_s < 5.0:
            return
        self.last_param_config_s = now_s

        if self.armed:
            changed = {name: value for name, value in changed.items() if name == "AVOID_ENABLE"}
            if not changed:
                return

        for name, value in changed.items():
            self.set_fc_param_real32(name, value)
            self.last_param_values_sent[name] = value

        self.native_params_sent = True
        if "AVOID_ENABLE" in changed:
            if changed["AVOID_ENABLE"] > 0:
                self.send_statustext("OA ENGAGED: AVOID_ENABLE=7.", self.severity_notice())
            else:
                self.send_statustext("OA DETECT ONLY: AVOID_ENABLE=0.", self.severity_notice())
        elif "PRX1_TYPE" in changed:
            self.send_statustext("LiDAR PRX configured for detection stream.", self.severity_notice())

    def motion_block_reason(self) -> str:
        av = self.avoidance_config
        now_s = time.time()
        if not self.direct_motion_enabled():
            return "motion disabled"
        if av.get("dry_run", True):
            return "dry-run"
        if self.master is None:
            return "no mavlink"
        if now_s - self.last_vehicle_heartbeat_s > 2.0:
            return "no heartbeat"
        if not self.armed:
            return "disarmed"
        if self.landed_state != 2:
            return "not airborne"
        allowed_modes = av.get("direct_allowed_modes") or av.get("allowed_modes") or ["GUIDED"]
        if self.flight_mode not in allowed_modes:
            return f"mode {self.flight_mode}"
        if self.is_stale:
            return "stale lidar"
        return ""

    def is_safe_to_move(self) -> bool:
        reason = self.motion_block_reason()
        if reason:
            self.last_block_reason = reason
            return False
        self.last_block_reason = ""
        return True

    def run(self, mavlink_port: str | None = None) -> None:
        if mavlink_port:
            self.connect_mavlink(mavlink_port)
        self.ensure_gcs_status_sender()
        self.ensure_rc_downlink_listener()

        lc = self.lidar_config
        av = self.avoidance_config
        tick_rate = self.current_tick_rate_hz()
        keepout_m = float(av.get("keepout_distance_m", lc.get("danger_distance_m", 1.5)))
        mode = self.avoidance_mode()
        proximity_enabled = self.proximity_publishing_enabled()
        motion_mode = "enabled" if av.get("enable_avoidance_motion", False) and not av.get("dry_run", True) else "dry-run/off"

        print(
            f"LiDAR avoidance active at {tick_rate:.1f}Hz: keepout={keepout_m:.1f}m "
            f"mode={mode} proximity={proximity_enabled} motion={motion_mode}"
        )
        if self.rc_toggle_enabled():
            channel = int(self.rc_toggle_config().get("channel", 7))
            self.send_statustext(
                f"LiDAR OA RC{channel} toggle ready: low detect, high avoid.",
                self.severity_notice(),
            )
        else:
            self.send_statustext(
                f"LiDAR OA mode {mode}: keepout {keepout_m:.1f}m proximity={'on' if proximity_enabled else 'off'} motion={motion_mode}.",
                self.severity_notice(),
            )

        while True:
            try:
                if self.lidar is None:
                    print(f"Attempting to open JT16 LiDAR on {lc.get('lidar_port', 'auto')}...")
                    self.lidar = LidarReader.open(
                        port=str(lc.get("lidar_port", "auto")),
                        baud=int(lc.get("lidar_baud", 3000000)),
                        min_valid_distance_m=float(lc.get("min_valid_distance_m", 0.15)),
                        max_valid_distance_m=float(lc.get("max_detection_range_m", 7.0)),
                        min_points_per_sector=int(lc.get("min_points_per_sector", lc.get("min_points_per_zone", 1))),
                    )
                    print("LiDAR connection established.")
                    self.send_statustext("LiDAR connected. Obstacle scan active.", self.severity_notice())

                start_s = time.time()
                self.update_fc_state()
                self.update_rc_toggle_state()
                if self.proximity_publishing_enabled():
                    self.maybe_configure_ardupilot_avoidance()
                else:
                    self.maybe_disable_ardupilot_prx()
                period_s = 1.0 / self.current_tick_rate_hz()

                snap = self.lidar.poll(duration_s=0.02)
                now_s = time.time()
                stale_timeout_s = float(lc.get("stale_timeout_sec", 0.5))
                fresh_sectors = snap.fresh_sector_distances(stale_timeout_s, now_s)
                has_fresh_scan = any(distance_m > 0.0 for distance_m in fresh_sectors)
                has_recent_packet = snap.point_packets > 0 and now_s - snap.timestamp_s <= stale_timeout_s

                if not has_fresh_scan:
                    if has_recent_packet:
                        if self.is_stale:
                            self.is_stale = False
                            self.send_statustext("LiDAR data restored. Avoidance enabled.", self.severity_notice())
                        clear_command = AvoidanceCommand(active=False, reason="scan_fresh")
                        self.publish_native_avoidance(fresh_sectors, [0.0] * 8)
                        self.send_periodic_active_status(clear_command)
                        self.send_periodic_rc_toggle_status()
                    elif not self.is_stale:
                        self.is_stale = True
                        self.send_statustext("LiDAR data stale. Avoidance disabled.", self.severity_warning())
                        self.publish_lua_obstacle_status(90, force=True)
                    time.sleep(max(0.0, period_s - (time.time() - start_s)))
                    continue

                if self.is_stale:
                    self.is_stale = False
                    self.send_statustext("LiDAR data restored. Avoidance enabled.", self.severity_notice())

                command = compute_keepout_velocity(
                    fresh_sectors,
                    keepout_distance_m=keepout_m,
                    min_valid_distance_m=float(lc.get("min_valid_distance_m", 0.15)),
                    max_speed_m_s=float(av.get("max_velocity_cmd_mps", 1.0)),
                    critical_distance_m=float(av.get("critical_distance_m", 0.5)),
                    angle_offset_deg=float(av.get("angle_offset_deg", 0.0)),
                    speed_exponent=float(av.get("speed_exponent", 0.75)),
                )
                zones = reduce_to_zones(
                    fresh_sectors,
                    zone_count=8,
                    angle_offset_deg=float(av.get("angle_offset_deg", 0.0)),
                    max_distance_m=float(lc.get("max_detection_range_m", 7.0)),
                )
                pitch_deg, roll_deg = velocity_to_tilt_deg(
                    command.vx_m_s,
                    command.vy_m_s,
                    float(av.get("max_velocity_cmd_mps", 1.0)),
                    float(av.get("max_pitch_cmd", 0.12)),
                )

                self.handle_feedback(command, zones, pitch_deg, roll_deg)
                self.publish_native_avoidance(fresh_sectors, zones)
                self.send_periodic_active_status(command)
                self.send_periodic_rc_toggle_status()
                self.maybe_send_velocity_pulse(command)

                time.sleep(max(0.0, period_s - (time.time() - start_s)))

            except RuntimeError as exc:
                print(f"LiDAR error: {exc}. Retrying in 5s...")
                self.send_statustext(f"LiDAR error: {str(exc)[:35]}", self.severity_warning())
                self.lidar = None
                time.sleep(5.0)
            except KeyboardInterrupt:
                print("\nShutting down.")
                break
            except Exception as exc:  # noqa: BLE001
                print(f"Unexpected obstacle node error: {exc}. Retrying in 5s...")
                self.send_statustext(f"LiDAR node error: {str(exc)[:33]}", self.severity_warning())
                time.sleep(5.0)

        if self.lidar is not None:
            self.lidar.close()

    def handle_feedback(
        self,
        command: AvoidanceCommand,
        zones: list[float],
        pitch_deg: float,
        roll_deg: float,
    ) -> None:
        if not command.closest_distance_m:
            return
        now_s = time.time()
        av = self.avoidance_config
        keepout_m = float(av.get("keepout_distance_m", self.lidar_config.get("danger_distance_m", 1.5)))
        warning_m = float(self.lidar_config.get("warning_distance_m", keepout_m + 1.0))
        closest = command.closest_distance_m
        beep_enabled = bool(av.get("beep_on_obstacle_under_keepout", True))
        startup_beep_grace_s = max(float(av.get("startup_beep_grace_sec", 10.0)), 0.0)
        critical_beep_interval_s = max(float(av.get("critical_beep_interval_sec", 0.5)), 0.1)
        keepout_beep_interval_s = max(float(av.get("keepout_beep_interval_sec", 1.0)), 0.1)
        in_startup_beep_grace = now_s - self.started_s < startup_beep_grace_s

        if beep_enabled and not self.armed and 0.0 < closest < keepout_m:
            if now_s - self.last_beep_mute_notice_s > 10.0:
                self.send_statustext(
                    f"LiDAR audio muted until armed; closest {closest:.1f}m",
                    self.severity_warning(),
                )
                self.last_beep_mute_notice_s = now_s
        elif beep_enabled and in_startup_beep_grace and 0.0 < closest < keepout_m:
            if now_s - self.last_beep_mute_notice_s > 5.0:
                remaining_s = max(0.0, startup_beep_grace_s - (now_s - self.started_s))
                self.send_statustext(
                    f"LiDAR audio muted startup {remaining_s:.0f}s; closest {closest:.1f}m",
                    self.severity_warning(),
                )
                self.last_beep_mute_notice_s = now_s
        elif (
            beep_enabled
            and 0.0 < closest <= float(av.get("critical_distance_m", 0.5))
            and self.armed
        ):
            if now_s - self.last_beep_time_s > critical_beep_interval_s:
                self.send_tune(
                    self.tune_rapid_beeps,
                    f"LiDAR critical beep: obstacle {closest:.1f}m",
                    self.severity_critical(),
                )
                self.last_beep_time_s = now_s
        elif beep_enabled and 0.0 < closest < keepout_m:
            if now_s - self.last_beep_time_s > keepout_beep_interval_s:
                self.send_tune(
                    self.tune_single_beep,
                    f"LiDAR beep: obstacle <{keepout_m:.1f}m at {closest:.1f}m",
                    self.severity_warning(),
                )
                self.last_beep_time_s = now_s

        keepout_status_interval_s = max(float(av.get("keepout_status_interval_sec", 10.0)), 1.0)
        if now_s - self.last_warn_time_s <= keepout_status_interval_s:
            return

        if not self.armed and 0.0 < closest < keepout_m:
            self.last_warn_time_s = now_s
            return

        closest_zone_distance = min([distance_m for distance_m in zones if distance_m > 0.0] or [0.0])
        zone_name = "Unknown"
        if closest_zone_distance > 0.0:
            zone_name = ZONE_NAMES[zones.index(closest_zone_distance)]

        if 0.0 < closest < keepout_m:
            lua_code = 40 if closest <= float(av.get("critical_distance_m", 0.5)) else 30
            self.publish_lua_obstacle_status(lua_code)
            block_reason = self.motion_block_reason()
            if command.active:
                msg = (
                    f"LiDAR keepout: {zone_name} {closest:.1f}m "
                    f"push vx={command.vx_m_s:+.1f} vy={command.vy_m_s:+.1f}"
                )
                if block_reason:
                    msg = f"{msg} ({block_reason})"
                self.send_statustext(msg, self.severity_critical())
                print(
                    f"KEEP OUT: {zone_name} {closest:.2f}m | P={pitch_deg:+.1f} R={roll_deg:+.1f} "
                    f"| vx={command.vx_m_s:+.2f} vy={command.vy_m_s:+.2f} | block={block_reason or 'clear'}"
                )
            self.last_warn_time_s = now_s
        elif 0.0 < closest < warning_m:
            self.publish_lua_obstacle_status(20)
            self.send_statustext(f"LiDAR warning: {zone_name} {closest:.1f}m", self.severity_warning())
            self.last_warn_time_s = now_s

    def send_periodic_active_status(self, command: AvoidanceCommand) -> None:
        av = self.avoidance_config
        interval_s = max(float(av.get("active_status_interval_sec", 10.0)), 1.0)
        now_s = time.time()
        if now_s - self.last_active_status_time_s < interval_s:
            return
        self.last_active_status_time_s = now_s

        keepout_m = float(av.get("keepout_distance_m", self.lidar_config.get("danger_distance_m", 1.5)))
        mode = self.avoidance_mode()
        native_state = "avoid on" if self.native_avoidance_enabled() else "detect only"
        prefix = "OA ACTIVE"
        detail = mode
        if self.rc_toggle_enabled():
            prefix = "OA ENGAGED" if self.rc_toggle_engaged else "OA DETECT ONLY"
            channel = int(self.rc_toggle_config().get("channel", 7))
            if self.last_rc_update_s <= 0.0:
                detail = f"RC{channel}=?"
            else:
                detail = f"RC{channel}={self.rc_toggle_pwm()}"
        if command.closest_distance_m > 0.0:
            text = f"{prefix}: {detail} keepout {keepout_m:.1f}m closest {command.closest_distance_m:.1f}m"
        else:
            text = f"{prefix}: {detail} keepout {keepout_m:.1f}m scan fresh"
        self.send_statustext(text, self.severity_notice())
        lua_code = self.lua_obstacle_status_code(command)
        self.publish_lua_obstacle_status(lua_code)
        print(f"Obstacle avoidance status: {text} ({native_state})")

    def lua_obstacle_status_code(self, command: AvoidanceCommand) -> int:
        closest = command.closest_distance_m
        av = self.avoidance_config
        keepout_m = float(av.get("keepout_distance_m", self.lidar_config.get("danger_distance_m", 1.5)))
        warning_m = float(self.lidar_config.get("warning_distance_m", keepout_m + 1.0))
        critical_m = float(av.get("critical_distance_m", 0.5))
        active_clear_status = 10 if self.native_avoidance_enabled() else 12
        if not self.armed:
            return active_clear_status
        if closest <= 0.0:
            return active_clear_status
        if closest <= critical_m:
            return 40
        if closest < keepout_m:
            return 30
        if closest < warning_m:
            return 20
        return active_clear_status

    def publish_lua_obstacle_status(self, status_code: int, force: bool = False) -> None:
        av = self.avoidance_config
        if not av.get("enable_lua_obstacle_status", True):
            return
        if self.master is None or mavutil is None:
            return
        now_s = time.time()
        interval_s = max(float(av.get("lua_obstacle_status_interval_sec", 10.0)), 1.0)
        if not force and status_code == self.last_lua_obstacle_status and now_s - self.last_lua_obstacle_status_time_s < interval_s:
            return
        self.last_lua_obstacle_status = status_code
        self.last_lua_obstacle_status_time_s = now_s
        param_name = str(av.get("lua_obstacle_status_param", "SCR_USER4")).encode("ascii", errors="ignore")[:16]
        try:
            self.master.mav.param_set_send(
                self.master.target_system or 1,
                self.master.target_component or 1,
                param_name,
                float(status_code),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"Unable to publish Lua obstacle status: {exc}")

    def publish_native_avoidance(self, sectors: list[float], zones: list[float]) -> None:
        if self.master is None:
            return
        av = self.avoidance_config
        if not self.proximity_publishing_enabled():
            return
        now_s = time.time()
        publish_rate = self.current_native_publish_rate_hz()
        if now_s - self.last_native_publish_s < 1.0 / publish_rate:
            return
        self.last_native_publish_s = now_s

        max_distance_m = float(self.lidar_config.get("max_detection_range_m", 7.0))
        if av.get("send_obstacle_distance", True):
            try:
                send_obstacle_distance(self.master, sectors, max_distance_m)
            except Exception as exc:  # noqa: BLE001
                print(f"OBSTACLE_DISTANCE send failed: {exc}")
        if av.get("send_distance_sensor_zones", True):
            self.send_distance_sensor_zones(zones, max_distance_m)

    def send_distance_sensor_zones(self, zones: list[float], max_distance_m: float) -> None:
        if self.master is None or mavutil is None or not hasattr(self.master.mav, "distance_sensor_send"):
            return
        min_cm = int(round(float(self.lidar_config.get("min_valid_distance_m", 0.15)) * 100.0))
        max_cm = int(round(max(max_distance_m, 0.1) * 100.0))
        sensor_base_id = int(self.avoidance_config.get("distance_sensor_base_id", 40))
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

        for zone_index, distance_m in enumerate(zones[:8]):
            current_m = distance_m if distance_m > 0.0 else max_distance_m
            current_cm = int(round(max(0.02, min(current_m, max_distance_m)) * 100.0))
            try:
                self.master.mav.distance_sensor_send(
                    now_ms,
                    min_cm,
                    max_cm,
                    current_cm,
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    sensor_base_id + zone_index,
                    zone_index,
                    0,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"DISTANCE_SENSOR zone send failed: {exc}")
                return

    def maybe_send_velocity_pulse(self, command: AvoidanceCommand) -> None:
        if not command.active:
            return
        if not self.is_safe_to_move():
            return
        now_s = time.time()
        cooldown_s = float(self.avoidance_config.get("cooldown_ms", 100)) / 1000.0
        if now_s - self.last_pulse_time_s < cooldown_s:
            return
        print(f"Sending LiDAR avoidance velocity pulse vx={command.vx_m_s:+.2f} vy={command.vy_m_s:+.2f}")
        self.send_velocity_pulse(command.vx_m_s, command.vy_m_s)
        self.last_pulse_time_s = now_s

    def send_tune(self, tune: str, notice_text: str | None = None, severity: int | None = None) -> None:
        if self.master is None:
            return
        if not self.armed:
            return
        try:
            if notice_text:
                self.send_statustext(notice_text, severity or self.severity_notice())
            self.master.mav.play_tune_send(1, 1, tune.encode("utf-8"))
        except Exception:
            return

    def send_mode_tune(self, tune: str) -> None:
        if self.master is None:
            return
        try:
            self.master.mav.play_tune_send(1, 1, tune.encode("utf-8", errors="ignore"))
        except Exception:
            return

    def send_statustext(self, text: str, severity: int | None = None) -> None:
        print(f"GCS: {text}", flush=True)
        severity_value = severity if severity is not None else self.severity_notice()
        encoded = text.encode("utf-8", errors="ignore")
        chunks = [encoded[index : index + 50] for index in range(0, len(encoded), 50)] or [b""]
        self.send_statustext_to_gcs_udp(chunks, severity_value)
        if self.master is None or not bool(self.avoidance_config.get("send_statustext_to_cube", False)):
            return
        try:
            for chunk in chunks:
                self.master.mav.statustext_send(severity_value, chunk)
        except Exception:
            return

    def send_statustext_to_gcs_udp(self, chunks: list[bytes], severity: int) -> None:
        if mavutil is None:
            return
        self.ensure_gcs_status_sender()
        if self.gcs_status_sock is None or self.gcs_status_mav is None:
            return
        for chunk in chunks:
            try:
                msg = self.gcs_status_mav.statustext_encode(severity, chunk)
                msg.pack(self.gcs_status_mav)
                payload = bytes(msg.get_msgbuf())
            except Exception:
                return
            for target in self.gcs_status_targets():
                try:
                    self.gcs_status_sock.sendto(payload, target)
                except OSError:
                    continue

    def send_velocity_pulse(self, vx_m_s: float, vy_m_s: float) -> None:
        if self.master is None or mavutil is None:
            return
        try:
            # Use velocity only. Position, acceleration, yaw, and yaw-rate are ignored.
            type_mask = 0b0000110111000111
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system or 1,
                self.master.target_component or 1,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0,
                0,
                0,
                vx_m_s,
                vy_m_s,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"Failed to send velocity pulse: {exc}")

    @staticmethod
    def severity_notice() -> int:
        return mavutil.mavlink.MAV_SEVERITY_NOTICE if mavutil is not None else 5

    @staticmethod
    def severity_warning() -> int:
        return mavutil.mavlink.MAV_SEVERITY_WARNING if mavutil is not None else 4

    @staticmethod
    def severity_critical() -> int:
        return mavutil.mavlink.MAV_SEVERITY_CRITICAL if mavutil is not None else 2


def resolve_config_path(path_text: str) -> Path:
    path = Path(path_text)
    if path.exists():
        return path
    fallback = REPO_ROOT / "config" / "sensors.yaml"
    if fallback.exists():
        return fallback
    raise FileNotFoundError(path_text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="JT16 LiDAR obstacle avoidance publisher")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "sensors.yaml"))
    parser.add_argument("--mavport", default=os.getenv("SLAM_OBSTACLE_MAVPORT", DEFAULT_MAVPORT))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--mode",
        choices=sorted(VALID_AVOIDANCE_MODES),
        help="Override config avoidance.mode for this run.",
    )
    parser.add_argument("--enable-motion", action="store_true", help="Enable gated direct companion motion pulses.")
    parser.add_argument("--dry-run", action="store_true", help="Force direct motion pulses off.")
    parser.add_argument("--no-native", action="store_true", help="Disable ArduPilot native obstacle/proximity publishing.")
    args = parser.parse_args()

    node = AvoidanceNode(str(resolve_config_path(args.config)))
    if args.mode:
        node.avoidance_config["mode"] = args.mode
    if args.enable_motion:
        node.avoidance_config["mode"] = MODE_DIRECT_VELOCITY
        node.avoidance_config["enable_avoidance_motion"] = True
        node.avoidance_config["dry_run"] = False
    if args.dry_run:
        node.avoidance_config["dry_run"] = True
    if args.no_native:
        node.avoidance_config["mode"] = MODE_MONITOR_ONLY
        node.avoidance_config["use_ardupilot_native_avoidance"] = False

    node.run(mavlink_port=args.mavport)
