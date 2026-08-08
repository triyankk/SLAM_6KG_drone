"""Headless arm-triggered flight recording service."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import signal
import threading
import time
from typing import Any, Callable

from .config import ConfigError, ProjectConfig, load_config
from .flight_analysis import analyze_session
from .flight_logger import (
    DEFAULT_CONFIG,
    DEFAULT_FLIGHT_ROOT,
    FlightSession,
    HesaiLidarRecorder,
    RealSensePointCloudRecorder,
)
from .obstacles import ObstacleFusion, ObstacleScan
from .paths import RUNTIME_DIR
from .visualizer_server import Im10aSource, MavlinkSource, TelemetryStore


DEFAULT_STATUS_PATH = RUNTIME_DIR / "flight_logger_status.json"
SUPERVISOR_STATUS_PATH = RUNTIME_DIR / "flight_supervisor_status.json"
UNCLEAN_STOP_REASON = "recovered_after_unclean_shutdown"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def recover_stale_service_sessions(output_root: Path) -> list[Path]:
    """Close service-owned manifests left recording after an unclean stop."""

    recovered: list[Path] = []
    for manifest_path in sorted(output_root.glob("*/manifest.json")):
        try:
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if (
            payload.get("status") != "recording"
            or payload.get("telemetry_url") != "direct://cube-uart"
        ):
            continue

        latest_mtime = manifest_path.stat().st_mtime
        try:
            latest_mtime = max(
                (
                    path.stat().st_mtime
                    for path in manifest_path.parent.rglob("*")
                    if (
                        path.is_file()
                        and path != manifest_path
                        and path.relative_to(manifest_path.parent).parts[0]
                        not in {"analysis", "cube"}
                        and ".recovered." not in path.name
                    )
                ),
                default=latest_mtime,
            )
        except OSError:
            pass
        ended_utc = datetime.fromtimestamp(
            latest_mtime, timezone.utc
        ).isoformat(timespec="milliseconds")
        recovered_utc = _utc_now()
        payload.update(
            status="interrupted",
            stop_reason=UNCLEAN_STOP_REASON,
            ended_utc=ended_utc,
            recovery={
                "files_preserved": True,
                "recovered_utc": recovered_utc,
                "reason": UNCLEAN_STOP_REASON,
            },
        )
        temporary = manifest_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(manifest_path)
        recovered.append(manifest_path.parent)
    return recovered


def latest_service_session(
    output_root: Path,
) -> tuple[Path, dict[str, Any]] | None:
    """Return the newest service-owned session with a readable manifest."""

    candidates: list[tuple[str, Path, dict[str, Any]]] = []
    for manifest_path in sorted(output_root.glob("*/manifest.json")):
        try:
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if payload.get("telemetry_url") != "direct://cube-uart":
            continue
        candidates.append(
            (
                str(payload.get("started_utc") or manifest_path.parent.name),
                manifest_path.parent,
                payload,
            )
        )
    if not candidates:
        return None
    _, session_path, payload = max(candidates, key=lambda item: item[0])
    return session_path, payload


@dataclass(frozen=True)
class ServiceSettings:
    sample_rate_hz: float = 30.0
    pre_roll_s: float = 5.0
    post_disarm_s: float = 10.0
    min_free_gb: float = 5.0
    pointcloud_rate_hz: float = 2.0
    point_stride: int = 8
    voxel_size_m: float = 0.08
    depth_enabled: bool = True
    realsense_bag_enabled: bool = True
    lidar_enabled: bool = True


class ArmTriggeredRecorder:
    """Create and finalize one passive session per armed period."""

    def __init__(
        self,
        config: ProjectConfig,
        config_path: Path,
        output_root: Path,
        settings: ServiceSettings,
        *,
        disk_free_gb: Callable[[], float] | None = None,
        obstacle_sink: Callable[[ObstacleScan], None] | None = None,
    ) -> None:
        self.config = config
        self.config_path = config_path
        self.output_root = output_root
        self.settings = settings
        self.obstacle_sink = obstacle_sink
        self.current_session: FlightSession | None = None
        self.current_session_stop: threading.Event | None = None
        self.current_sources: list[threading.Thread] = []
        self.disarmed_since_ns: int | None = None
        self.last_session: Path | None = None
        self.last_report: Path | None = None
        self.last_stop_reason: str | None = None
        self.last_error: str | None = None
        self.free_gb: float | None = None
        self.inhibit_reason: str | None = None
        self._inhibit_until_disarm = False
        self._last_disk_check_ns = 0
        self._disk_free_gb = disk_free_gb or (
            lambda: shutil.disk_usage(self.output_root).free / 1.0e9
        )
        self._pre_snapshots: deque[
            tuple[dict[str, Any], int, str]
        ] = deque()
        self._pre_events: deque[dict[str, Any]] = deque()

    @property
    def state(self) -> str:
        if self.current_session is None:
            if self._inhibit_until_disarm:
                return "inhibited_until_disarm"
            return "waiting_for_arm"
        if self.disarmed_since_ns is not None:
            return "post_disarm_tail"
        return "recording"

    def _trim_pre_roll(self, now_ns: int) -> None:
        cutoff = now_ns - round(self.settings.pre_roll_s * 1.0e9)
        while (
            self._pre_snapshots
            and self._pre_snapshots[0][1] < cutoff
        ):
            self._pre_snapshots.popleft()
        while (
            self._pre_events
            and int(self._pre_events[0].get("host_monotonic_ns", 0))
            < cutoff
        ):
            self._pre_events.popleft()

    def _buffer_disarmed(
        self,
        snapshot: dict[str, Any],
        monotonic_ns: int,
        host_time_utc: str,
        raw_events: list[dict[str, Any]],
    ) -> None:
        self._pre_snapshots.append(
            (snapshot, monotonic_ns, host_time_utc)
        )
        self._pre_events.extend(raw_events)
        self._trim_pre_roll(monotonic_ns)

    def _refresh_free_space(
        self, monotonic_ns: int, *, force: bool = False
    ) -> float | None:
        if (
            not force
            and self._last_disk_check_ns
            and monotonic_ns - self._last_disk_check_ns < 1_000_000_000
        ):
            return self.free_gb
        self._last_disk_check_ns = monotonic_ns
        try:
            self.free_gb = float(self._disk_free_gb())
        except OSError as exc:
            self.last_error = f"storage check failed: {exc}"
            self.free_gb = None
        return self.free_gb

    def _inhibit(self, reason: str, error: str) -> None:
        self._inhibit_until_disarm = True
        self.inhibit_reason = reason
        self.last_error = error

    def _start_session(self, monotonic_ns: int) -> None:
        free_gb = self._refresh_free_space(monotonic_ns, force=True)
        if free_gb is None:
            self._inhibit(
                "storage_check_failed",
                self.last_error or "storage check failed",
            )
            return
        if free_gb < self.settings.min_free_gb:
            self._inhibit(
                "insufficient_free_space",
                (
                    "recording not started: "
                    f"{free_gb:.2f} GB free is below the "
                    f"{self.settings.min_free_gb:.2f} GB minimum"
                ),
            )
            return

        session = FlightSession(
            self.output_root,
            "armed",
            self.config,
            self.config_path,
            "direct://cube-uart",
            "direct://sensor-event-bus",
        )
        session.event(
            "trigger",
            "armed_detected",
            {
                "pre_roll_s": self.settings.pre_roll_s,
                "pre_roll_telemetry_rows": len(self._pre_snapshots),
                "pre_roll_sensor_events": len(self._pre_events),
                "detected_monotonic_ns": monotonic_ns,
            },
        )
        session_stop = threading.Event()
        sources: list[threading.Thread] = []
        if self.settings.depth_enabled:
            sources.append(
                RealSensePointCloudRecorder(
                    session,
                    session_stop,
                    self.config,
                    pointcloud_rate_hz=self.settings.pointcloud_rate_hz,
                    point_stride=self.settings.point_stride,
                    voxel_size_m=self.settings.voxel_size_m,
                    record_bag=self.settings.realsense_bag_enabled,
                    obstacle_sink=self.obstacle_sink,
                )
            )
        if self.settings.lidar_enabled:
            sources.append(
                HesaiLidarRecorder(
                    session,
                    session_stop,
                    self.config,
                    obstacle_sink=self.obstacle_sink,
                )
            )
        self.current_session = session
        self.current_session_stop = session_stop
        self.current_sources = sources
        self.last_error = None
        for source in sources:
            source.start()

        for snapshot, timestamp_ns, host_time_utc in self._pre_snapshots:
            session.record_snapshot(
                snapshot,
                timestamp_ns,
                host_time_utc=host_time_utc,
            )
        for event in self._pre_events:
            session.record_sensor_event(event)
        self._pre_snapshots.clear()
        self._pre_events.clear()
        print(f"Armed: recording {session.path}", flush=True)

    def process(
        self,
        snapshot: dict[str, Any],
        monotonic_ns: int,
        host_time_utc: str,
        raw_events: list[dict[str, Any]],
    ) -> None:
        armed = bool(snapshot.get("vehicle", {}).get("armed"))
        if self.current_session is None:
            if not armed:
                self._inhibit_until_disarm = False
                self.inhibit_reason = None
                self._buffer_disarmed(
                    snapshot,
                    monotonic_ns,
                    host_time_utc,
                    raw_events,
                )
                return
            if self._inhibit_until_disarm:
                return
            self._start_session(monotonic_ns)

        session = self.current_session
        if session is None:
            return
        session.record_snapshot(
            snapshot,
            monotonic_ns,
            host_time_utc=host_time_utc,
        )
        for event in raw_events:
            session.record_sensor_event(event)

        free_gb = self._refresh_free_space(monotonic_ns)
        if free_gb is not None:
            session.set_source_stats(
                "storage",
                free_gb=round(free_gb, 3),
                minimum_free_gb=self.settings.min_free_gb,
            )
        if (
            free_gb is not None
            and free_gb < self.settings.min_free_gb
        ):
            session.event(
                "storage",
                "minimum_free_space_reached",
                {
                    "free_gb": free_gb,
                    "minimum_gb": self.settings.min_free_gb,
                },
            )
            error = (
                "recording stopped before filling the disk: "
                f"{free_gb:.2f} GB free"
            )
            self._inhibit("minimum_free_space_reached", error)
            self._finalize(
                "interrupted", "minimum_free_space_reached"
            )
            self.last_error = error
            return

        if armed:
            if self.disarmed_since_ns is not None:
                session.event(
                    "trigger",
                    "rearmed_during_tail",
                    {
                        "tail_elapsed_s": (
                            monotonic_ns - self.disarmed_since_ns
                        )
                        / 1.0e9
                    },
                )
            self.disarmed_since_ns = None
            return

        if self.disarmed_since_ns is None:
            self.disarmed_since_ns = monotonic_ns
            session.event(
                "trigger",
                "disarm_detected",
                {"post_disarm_s": self.settings.post_disarm_s},
            )
            return
        tail_s = (monotonic_ns - self.disarmed_since_ns) / 1.0e9
        if tail_s >= self.settings.post_disarm_s:
            self._finalize("complete", "post_disarm_tail_complete")

    def _finalize(self, status: str, reason: str) -> None:
        session = self.current_session
        if session is None:
            return
        session.event(
            "trigger",
            "finalizing",
            {"status": status, "reason": reason},
        )
        if self.current_session_stop is not None:
            self.current_session_stop.set()
        for source in self.current_sources:
            source.join()
        session.close(status=status, reason=reason)
        self.last_session = session.path
        self.last_stop_reason = reason
        try:
            self.last_report = analyze_session(session.path)
            print(f"Flight finalized: {self.last_report}", flush=True)
        except (OSError, RuntimeError, ValueError) as exc:
            self.last_error = f"analysis failed: {exc}"
            print(self.last_error, flush=True)
        self.current_session = None
        self.current_session_stop = None
        self.current_sources = []
        self.disarmed_since_ns = None

    def shutdown(self) -> None:
        if self.current_session is not None:
            self._finalize("interrupted", "logger_service_stopped")

    def status(self) -> dict[str, Any]:
        session = self.current_session
        return {
            "state": self.state,
            "current_session": (
                None if session is None else str(session.path)
            ),
            "last_session": (
                None if self.last_session is None else str(self.last_session)
            ),
            "last_report": (
                None if self.last_report is None else str(self.last_report)
            ),
            "last_stop_reason": self.last_stop_reason,
            "last_error": self.last_error,
            "free_gb": self.free_gb,
            "minimum_free_gb": self.settings.min_free_gb,
            "inhibit_reason": self.inhibit_reason,
            "pre_roll_telemetry_rows": len(self._pre_snapshots),
            "pre_roll_sensor_events": len(self._pre_events),
            "post_disarm_elapsed_s": (
                None
                if self.disarmed_since_ns is None
                else (time.monotonic_ns() - self.disarmed_since_ns) / 1.0e9
            ),
            "rows": (
                None
                if session is None
                else {
                    "telemetry": session.telemetry.rows,
                    "sensor_events": session.sensor_events.rows,
                    "sensor_timing": session.sensor_timing.rows,
                    "shadow_predictions": session.shadow.rows,
                }
            ),
        }


def _drain_raw_events(
    store: TelemetryStore, sequence: int
) -> tuple[list[dict[str, Any]], int]:
    collected: list[dict[str, Any]] = []
    while True:
        events, dropped = store.raw_events.wait_after(
            sequence, timeout=0.0
        )
        if not events:
            break
        if dropped:
            events[0] = dict(events[0])
            events[0]["dropped_before"] = dropped
        collected.extend(events)
        sequence = int(events[-1]["sequence"])
    return collected, sequence


def _write_status(
    path: Path,
    manager: ArmTriggeredRecorder,
    store: TelemetryStore,
    output_root: Path,
    *,
    service_started_utc: str,
    service_state: str | None = None,
) -> None:
    snapshot = store.snapshot()
    payload = {
        "schema_version": 1,
        "updated_utc": _utc_now(),
        "service_started_utc": service_started_utc,
        **manager.status(),
        "state": service_state or manager.state,
        "vehicle": snapshot["vehicle"],
        "link": snapshot["link"],
        "sensors": {
            "flow_age_ms": snapshot["flow"]["age_ms"],
            "flow_quality": snapshot["flow"]["quality"],
            "range_age_ms": snapshot["range"]["age_ms"],
            "cube_imu": snapshot["imu"]["message"],
            "cube_imu_age_ms": snapshot["imu"]["age_ms"],
            "external_imu_connected": snapshot["ros_imu"]["connected"],
            "external_imu_rate_hz": snapshot["ros_imu"]["sample_rate_hz"],
            "external_imu_age_ms": snapshot["ros_imu"]["age_ms"],
            "obstacles": snapshot["obstacles"],
        },
        "power": snapshot["power"],
        "disk_free_gb": round(
            shutil.disk_usage(output_root).free / 1.0e9, 3
        ),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _validate_settings(settings: ServiceSettings) -> None:
    for name, value in (
        ("sample_rate_hz", settings.sample_rate_hz),
        ("pre_roll_s", settings.pre_roll_s),
        ("post_disarm_s", settings.post_disarm_s),
        ("min_free_gb", settings.min_free_gb),
        ("pointcloud_rate_hz", settings.pointcloud_rate_hz),
        ("point_stride", settings.point_stride),
        ("voxel_size_m", settings.voxel_size_m),
    ):
        if float(value) <= 0:
            raise ConfigError(f"{name} must be positive")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--output-root", type=Path, default=DEFAULT_FLIGHT_ROOT
    )
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS_PATH)
    parser.add_argument("--sample-rate", type=float, default=30.0)
    parser.add_argument("--pre-roll", type=float, default=5.0)
    parser.add_argument("--post-disarm", type=float, default=10.0)
    parser.add_argument("--min-free-gb", type=float, default=5.0)
    parser.add_argument("--pointcloud-rate", type=float, default=2.0)
    parser.add_argument("--point-stride", type=int, default=8)
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--no-realsense-bag", action="store_true")
    parser.add_argument("--no-lidar", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    settings = ServiceSettings(
        sample_rate_hz=args.sample_rate,
        pre_roll_s=args.pre_roll,
        post_disarm_s=args.post_disarm,
        min_free_gb=args.min_free_gb,
        pointcloud_rate_hz=args.pointcloud_rate,
        point_stride=args.point_stride,
        voxel_size_m=args.voxel_size,
        depth_enabled=not args.no_depth,
        realsense_bag_enabled=not args.no_realsense_bag,
        lidar_enabled=not args.no_lidar,
    )
    try:
        _validate_settings(settings)
        config = load_config(args.config)
    except (ConfigError, OSError) as exc:
        print(f"Flight logger service configuration error: {exc}")
        return 2

    args.output_root.mkdir(parents=True, exist_ok=True)
    recovered_sessions = recover_stale_service_sessions(args.output_root)
    source_stop = threading.Event()
    service_stop = threading.Event()
    mount = config.flight_controller.cube_mount
    store = TelemetryStore(
        "flight_logger_service",
        cube_mount={
            "x_m": mount.x_m,
            "y_m": mount.y_m,
            "z_m": mount.z_m,
            "yaw_ccw_deg": mount.yaw_ccw_deg,
            "ahrs_orientation": mount.ahrs_orientation,
            "ahrs_orientation_name": mount.ahrs_orientation_name,
        },
        imu_axis_signs=(
            config.external_imu.body_axis_signs.x,
            config.external_imu.body_axis_signs.y,
            config.external_imu.body_axis_signs.z,
        ),
        imu_axis_map_verified=config.external_imu.axis_map_verified,
        imu_axis_map_verification=(
            config.external_imu.axis_map_verification
        ),
    )
    obstacle_settings = config.obstacle_avoidance
    store.update(
        "obstacles",
        stage=obstacle_settings.stage,
        mavlink_output_enabled=(
            obstacle_settings.mavlink_output_enabled
        ),
        clearance_reference="aircraft_cg",
        clearance_distance_metric="horizontal_xy",
        hard_cg_clearance_m=(
            obstacle_settings.hard_cg_clearance_m
        ),
        source_stale_timeout_s=(
            obstacle_settings.source_stale_timeout_s
        ),
        clearance_status="unknown",
        sector_increment_deg=(
            obstacle_settings.sector_increment_deg
        ),
        rc_toggle_channel=obstacle_settings.rc_toggle.channel,
        rc_toggle_pwm=None,
        rc_toggle_enabled=False,
        alert_zone="stale",
        alert_beep_rate_hz=0.0,
    )
    obstacle_fusion = ObstacleFusion(obstacle_settings)
    mavlink_source = MavlinkSource(
        store,
        source_stop,
        config.flight_controller.endpoint,
        config.flight_controller.baud,
        source_system=(
            config.flight_controller.companion_system_id
        ),
        source_component=(
            config.flight_controller.companion_component_id
        ),
        obstacle_max_age_s=(
            obstacle_settings.source_stale_timeout_s
        ),
        obstacle_output_enabled=(
            obstacle_settings.mavlink_output_enabled
        ),
        obstacle_settings=obstacle_settings,
        startup_tune_enabled=True,
    )

    def receive_obstacle_scan(scan: ObstacleScan) -> None:
        obstacle_fusion.update(scan)
        fused = obstacle_fusion.fused(monotonic_ns=scan.monotonic_ns)
        if fused is None:
            return
        clearance = fused.assess_clearance(
            obstacle_settings.hard_cg_clearance_m
        )
        store.update(
            "obstacles",
            source=fused.source,
            valid_sector_count=fused.valid_sector_count,
            nearest_distance_m=fused.nearest_distance_m,
            clearance_status=clearance.status,
            clearance_margin_m=clearance.margin_m,
            clearance_breached=clearance.breached,
            violating_sector_count=(
                clearance.violating_sector_count
            ),
            violating_sector_angles_deg=list(
                clearance.violating_sector_angles_deg
            ),
            sector_increment_deg=fused.increment_deg,
            distances_cm=list(fused.distances_cm),
            updated_monotonic=time.monotonic(),
        )
        store.publish_raw(
            "obstacle_fusion",
            "OBSTACLE_DISTANCE",
            {
                "source": fused.source,
                "valid_sector_count": fused.valid_sector_count,
                "nearest_distance_m": fused.nearest_distance_m,
                "clearance": clearance.as_dict(),
                "increment_deg": fused.increment_deg,
                "distances_cm": list(fused.distances_cm),
                "mavlink_output_enabled": (
                    obstacle_settings.mavlink_output_enabled
                ),
            },
        )
        # Alerts also consume the latest scan while active MAVLink proximity
        # output remains independently protected by the calibration gate.
        mavlink_source.queue_obstacle_scan(fused)

    manager = ArmTriggeredRecorder(
        config,
        args.config,
        args.output_root,
        settings,
        obstacle_sink=receive_obstacle_scan,
    )
    previous_session = latest_service_session(args.output_root)
    if previous_session is not None:
        previous_path, previous_manifest = previous_session
        manager.last_session = previous_path
        manager.last_stop_reason = previous_manifest.get("stop_reason")
        previous_report = previous_path / "analysis" / "report.json"
        if previous_report.exists():
            manager.last_report = previous_report
    if recovered_sessions:
        print(
            "Recovered interrupted flight session: "
            f"{recovered_sessions[-1]}",
            flush=True,
        )
    sources: list[threading.Thread] = [
        mavlink_source,
        Im10aSource(
            store,
            source_stop,
            config.external_imu.symlink,
            config.external_imu.baud,
        ),
    ]

    def request_stop(_signum=None, _frame=None) -> None:
        service_stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    for source in sources:
        source.start()

    service_started_utc = _utc_now()
    sequence = store.raw_events.latest_sequence()
    sample_period_s = 1.0 / settings.sample_rate_hz
    next_sample_s = time.monotonic()
    next_status_s = next_sample_s
    print(
        "Headless flight logger ready; waiting for armed heartbeat.",
        flush=True,
    )
    try:
        while not service_stop.is_set():
            now_s = time.monotonic()
            wait_s = max(0.0, next_sample_s - now_s)
            if service_stop.wait(wait_s):
                break
            timestamp_ns = time.monotonic_ns()
            raw_events, sequence = _drain_raw_events(store, sequence)
            manager.process(
                store.snapshot(),
                timestamp_ns,
                _utc_now(),
                raw_events,
            )
            now_s = time.monotonic()
            if now_s >= next_status_s:
                _write_status(
                    args.status_file,
                    manager,
                    store,
                    args.output_root,
                    service_started_utc=service_started_utc,
                )
                next_status_s = now_s + 1.0
            next_sample_s += sample_period_s
            if next_sample_s < now_s - sample_period_s:
                next_sample_s = now_s + sample_period_s
    finally:
        timestamp_ns = time.monotonic_ns()
        raw_events, sequence = _drain_raw_events(store, sequence)
        manager.process(
            store.snapshot(),
            timestamp_ns,
            _utc_now(),
            raw_events,
        )
        source_stop.set()
        for source in sources:
            source.join()
        manager.shutdown()
        _write_status(
            args.status_file,
            manager,
            store,
            args.output_root,
            service_started_utc=service_started_utc,
            service_state="stopped",
        )
    return 0


def status_main() -> int:
    parser = argparse.ArgumentParser(
        description="Show the headless flight logger service status"
    )
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS_PATH)
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args()
    try:
        supervisor = json.loads(
            SUPERVISOR_STATUS_PATH.read_text(encoding="utf-8")
        )
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="ascii"
        ).strip()
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        supervisor = {}
        boot_id = ""
    supervisor_state = str(supervisor.get("state", ""))
    if (
        supervisor.get("boot_id") == boot_id
        and supervisor_state
        in {"shadow_starting", "shadow_waiting_for_flight", "failed"}
    ):
        if args.as_json:
            print(json.dumps(supervisor, indent=2, sort_keys=True))
            return 0
        print(f"STATE={supervisor_state}")
        print("MODE=SLAM_SHADOW_QGC_GUIDED")
        print("ACTIVE_CONTROL=false")
        print(f"REPORT={supervisor.get('report') or '-'}")
        print(f"ERROR={supervisor.get('error') or '-'}")
        return 0
    try:
        payload = json.loads(args.status_file.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as exc:
        print(f"Flight logger status unavailable: {exc}")
        return 2
    if args.as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    print(f"STATE={payload.get('state', 'unknown')}")
    print(f"LINK={str(bool(payload.get('link', {}).get('connected'))).lower()}")
    print(f"ARMED={str(bool(payload.get('vehicle', {}).get('armed'))).lower()}")
    print(f"MODE={payload.get('vehicle', {}).get('mode', 'UNKNOWN')}")
    print(f"SESSION={payload.get('current_session') or '-'}")
    print(f"LAST_SESSION={payload.get('last_session') or '-'}")
    print(f"LAST_REPORT={payload.get('last_report') or '-'}")
    obstacles = payload.get("sensors", {}).get("obstacles", {})
    print(f"OBSTACLE_STAGE={obstacles.get('stage', 'unknown')}")
    print(
        "OBSTACLE_OUTPUT="
        f"{str(bool(obstacles.get('mavlink_output_enabled'))).lower()}"
    )
    print(f"OBSTACLE_SOURCE={obstacles.get('source') or '-'}")
    print(f"OBSTACLE_AGE_MS={obstacles.get('age_ms')}")
    print(
        "OBSTACLE_NEAREST_M="
        f"{obstacles.get('nearest_distance_m')}"
    )
    print(
        "CG_CLEARANCE_LIMIT_M="
        f"{obstacles.get('hard_cg_clearance_m')}"
    )
    print(
        "CG_CLEARANCE_STATUS="
        f"{obstacles.get('clearance_status', 'unknown')}"
    )
    print(
        "CG_CLEARANCE_MARGIN_M="
        f"{obstacles.get('clearance_margin_m')}"
    )
    print(
        "OBSTACLE_RC_CHANNEL="
        f"{obstacles.get('rc_toggle_channel')}"
    )
    print(f"OBSTACLE_RC_PWM={obstacles.get('rc_toggle_pwm')}")
    print(
        "OBSTACLE_RC_ENABLED="
        f"{str(bool(obstacles.get('rc_toggle_enabled'))).lower()}"
    )
    print(
        "OBSTACLE_ALERT_ZONE="
        f"{obstacles.get('alert_zone', 'inactive')}"
    )
    print(
        "OBSTACLE_BEEP_RATE_HZ="
        f"{obstacles.get('alert_beep_rate_hz', 0.0)}"
    )
    print(
        "STARTUP_TUNE_SENT="
        f"{str(bool(obstacles.get('startup_tune_sent'))).lower()}"
    )
    print(f"DISK_FREE_GB={payload.get('disk_free_gb')}")
    print(f"ERROR={payload.get('last_error') or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
