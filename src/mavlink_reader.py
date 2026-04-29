from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

import yaml
from pymavlink import mavutil


MONITORED_MESSAGES = [
    "HEARTBEAT",
    "SYS_STATUS",
    "ATTITUDE",
    "LOCAL_POSITION_NED",
    "VFR_HUD",
    "RC_CHANNELS",
    "BATTERY_STATUS",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
]


@dataclass
class MonitorConfig:
    port: str
    baud: int
    heartbeat_timeout_s: float
    reconnect_delay_s: float
    log_level: str
    log_file: Path
    log_max_bytes: int
    log_backup_count: int
    status_interval_s: float

    @classmethod
    def from_yaml(cls, path: str | Path) -> "MonitorConfig":
        config_path = Path(path).expanduser().resolve()
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        mavlink = payload.get("mavlink", {}) or {}
        logging_config = payload.get("logging", {}) or {}
        repo_root = config_path.parent

        log_file = Path(str(logging_config.get("file", "logs/slam_mavlink_monitor.log"))).expanduser()
        if not log_file.is_absolute():
            log_file = repo_root / log_file

        return cls(
            port=str(mavlink.get("port", "/dev/ttyACM0")),
            baud=int(mavlink.get("baud", 115200)),
            heartbeat_timeout_s=float(mavlink.get("heartbeat_timeout_s", 8.0)),
            reconnect_delay_s=float(mavlink.get("reconnect_delay_s", 3.0)),
            log_level=str(logging_config.get("level", "INFO")).upper(),
            log_file=log_file,
            log_max_bytes=int(logging_config.get("max_bytes", 262144)),
            log_backup_count=int(logging_config.get("backup_count", 3)),
            status_interval_s=float(logging_config.get("status_interval_s", 2.0)),
        )


@dataclass
class FlightHealth:
    mode: str = "UNKNOWN"
    armed: bool = False
    last_heartbeat_s: float = 0.0
    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None
    ekf_flags: int | None = None
    ekf_velocity_variance: float | None = None
    ekf_pos_horiz_variance: float | None = None
    local_altitude_m: float | None = None
    vfr_altitude_m: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    rc_channels: int | None = None
    rc_rssi: int | None = None
    last_statustext: str = ""

    def heartbeat_age_s(self) -> float:
        if self.last_heartbeat_s <= 0.0:
            return math.inf
        return time.monotonic() - self.last_heartbeat_s


def setup_logging(config: MonitorConfig) -> logging.Logger:
    config.log_file.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("slam_mavlink_monitor")
    logger.setLevel(getattr(logging, config.log_level, logging.INFO))
    logger.handlers.clear()
    logger.propagate = False

    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    file_handler = RotatingFileHandler(
        config.log_file,
        maxBytes=max(config.log_max_bytes, 4096),
        backupCount=max(config.log_backup_count, 1),
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    return logger


class MavlinkReader:
    def __init__(self, config: MonitorConfig, logger: logging.Logger):
        self.config = config
        self.logger = logger
        self.health = FlightHealth()
        self.master: Any | None = None

    def run(self, once: bool = False) -> None:
        while True:
            try:
                self._connect()
                should_exit = self._read_loop(once=once)
                if should_exit:
                    return
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001
                self.logger.warning("MAVLink reader reconnecting after error: %s", exc)
            finally:
                self._close()
            time.sleep(max(self.config.reconnect_delay_s, 0.2))

    def _connect(self) -> None:
        self.logger.info(
            "Connecting to Cube MAVLink port=%s baud=%s",
            self.config.port,
            self.config.baud,
        )
        self.master = mavutil.mavlink_connection(
            self.config.port,
            baud=self.config.baud,
            autoreconnect=False,
            robust_parsing=True,
        )
        heartbeat = self.master.wait_heartbeat(timeout=self.config.heartbeat_timeout_s)
        if heartbeat is None:
            raise TimeoutError("heartbeat not received")
        self._handle_heartbeat(heartbeat)
        self.logger.info(
            "Connected port=%s baud=%s mode=%s armed=%s",
            self.config.port,
            self.config.baud,
            self.health.mode,
            "yes" if self.health.armed else "no",
        )

    def _read_loop(self, once: bool) -> bool:
        last_status_s = 0.0
        while True:
            if self.health.heartbeat_age_s() > self.config.heartbeat_timeout_s:
                raise TimeoutError(f"heartbeat age {self.health.heartbeat_age_s():.1f}s")

            assert self.master is not None
            msg = self.master.recv_match(
                type=MONITORED_MESSAGES,
                blocking=True,
                timeout=1.0,
            )
            if msg is not None:
                self._handle_message(msg)

            now_s = time.monotonic()
            if now_s - last_status_s >= max(self.config.status_interval_s, 0.5):
                self._log_status()
                last_status_s = now_s
                if once:
                    return True

    def _handle_message(self, msg: Any) -> None:
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            self._handle_heartbeat(msg)
        elif msg_type == "SYS_STATUS":
            voltage_mv = int(getattr(msg, "voltage_battery", -1))
            if voltage_mv > 0:
                self.health.battery_voltage_v = voltage_mv / 1000.0
            remaining = int(getattr(msg, "battery_remaining", -1))
            if remaining >= 0:
                self.health.battery_remaining_pct = remaining
        elif msg_type == "BATTERY_STATUS":
            voltages = getattr(msg, "voltages", [])
            valid_cells = [int(value) for value in voltages if 0 < int(value) < 65535]
            if valid_cells:
                self.health.battery_voltage_v = sum(valid_cells) / 1000.0
            remaining = int(getattr(msg, "battery_remaining", -1))
            if remaining >= 0:
                self.health.battery_remaining_pct = remaining
        elif msg_type == "ATTITUDE":
            self.health.roll_deg = math.degrees(float(getattr(msg, "roll", 0.0)))
            self.health.pitch_deg = math.degrees(float(getattr(msg, "pitch", 0.0)))
            self.health.yaw_deg = math.degrees(float(getattr(msg, "yaw", 0.0)))
        elif msg_type == "LOCAL_POSITION_NED":
            self.health.local_altitude_m = -float(getattr(msg, "z", 0.0))
        elif msg_type == "VFR_HUD":
            self.health.vfr_altitude_m = float(getattr(msg, "alt", 0.0))
        elif msg_type == "RC_CHANNELS":
            self.health.rc_channels = int(getattr(msg, "chancount", 0))
            self.health.rc_rssi = int(getattr(msg, "rssi", 0))
        elif msg_type == "EKF_STATUS_REPORT":
            self.health.ekf_flags = int(getattr(msg, "flags", 0))
            self.health.ekf_velocity_variance = float(getattr(msg, "velocity_variance", 0.0))
            self.health.ekf_pos_horiz_variance = float(getattr(msg, "pos_horiz_variance", 0.0))
        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            self.health.last_statustext = str(text).strip()
            severity = int(getattr(msg, "severity", mavutil.mavlink.MAV_SEVERITY_INFO))
            self._log_statustext(severity, self.health.last_statustext)

    def _handle_heartbeat(self, msg: Any) -> None:
        assert self.master is not None
        self.health.mode = self._decode_mode(msg)
        try:
            self.master.post_message(msg)
            flightmode = str(getattr(self.master, "flightmode", "") or "").upper()
            if flightmode and not flightmode.startswith("MODE("):
                self.health.mode = flightmode
        except Exception:  # noqa: BLE001
            pass
        base_mode = int(getattr(msg, "base_mode", 0))
        self.health.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.health.last_heartbeat_s = time.monotonic()

    def _decode_mode(self, msg: Any) -> str:
        assert self.master is not None
        custom_mode = int(getattr(msg, "custom_mode", 0))
        try:
            mapping = self.master.mode_mapping() or {}
            for name, mode_id in mapping.items():
                if int(mode_id) == custom_mode:
                    return str(name).upper()
        except Exception:  # noqa: BLE001
            pass

        flightmode = str(getattr(self.master, "flightmode", "") or "").upper()
        if flightmode and not flightmode.startswith("MODE("):
            return flightmode
        try:
            mode_text = mavutil.mode_string_v10(msg).upper()
            if mode_text and not mode_text.startswith("MODE("):
                return mode_text
        except Exception:  # noqa: BLE001
            pass
        return f"MODE_{custom_mode}"

    def _log_statustext(self, severity: int, text: str) -> None:
        if not text:
            return
        if severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
            self.logger.warning("STATUSTEXT severity=%s text=%s", severity, text)
        else:
            self.logger.info("STATUSTEXT severity=%s text=%s", severity, text)

    def _log_status(self) -> None:
        self.logger.info(
            "status port=%s baud=%s mode=%s armed=%s batt=%s ekf=%s "
            "alt_local=%s alt_vfr=%s attitude=%s hb_age=%.1fs rc=%s statustext=%s",
            self.config.port,
            self.config.baud,
            self.health.mode,
            "yes" if self.health.armed else "no",
            self._battery_text(),
            self._ekf_text(),
            self._fmt(self.health.local_altitude_m, "m"),
            self._fmt(self.health.vfr_altitude_m, "m"),
            self._attitude_text(),
            self.health.heartbeat_age_s(),
            self._rc_text(),
            self.health.last_statustext or "none",
        )

    def _battery_text(self) -> str:
        if self.health.battery_voltage_v is None:
            return "unknown"
        suffix = ""
        if self.health.battery_remaining_pct is not None:
            suffix = f"/{self.health.battery_remaining_pct}%"
        return f"{self.health.battery_voltage_v:.2f}V{suffix}"

    def _ekf_text(self) -> str:
        if self.health.ekf_flags is None:
            return "unknown"
        variance = ""
        if self.health.ekf_pos_horiz_variance is not None:
            variance = f" pos_var={self.health.ekf_pos_horiz_variance:.3f}"
        return f"flags=0x{self.health.ekf_flags:x}{variance}"

    def _attitude_text(self) -> str:
        if self.health.roll_deg is None:
            return "unknown"
        return (
            f"roll={self.health.roll_deg:+.1f}deg "
            f"pitch={self.health.pitch_deg:+.1f}deg "
            f"yaw={self.health.yaw_deg:+.1f}deg"
        )

    def _rc_text(self) -> str:
        if self.health.rc_channels is None:
            return "unknown"
        rssi = "unknown" if self.health.rc_rssi is None else str(self.health.rc_rssi)
        return f"channels={self.health.rc_channels} rssi={rssi}"

    @staticmethod
    def _fmt(value: float | None, suffix: str) -> str:
        if value is None or not math.isfinite(value):
            return "unknown"
        return f"{value:.2f}{suffix}"

    def _close(self) -> None:
        if self.master is None:
            return
        try:
            self.master.close()
        except Exception:  # noqa: BLE001
            pass
        self.master = None
