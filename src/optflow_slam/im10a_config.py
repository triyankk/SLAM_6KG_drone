"""Audit and reversibly configure the Hiwonder IM10A serial profile."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any

from .config import ConfigError, ProjectConfig, load_config
from .im10a import Im10aDecoder
from .paths import PROJECT_ROOT


REGISTER_SAVE = 0x00
REGISTER_OUTPUT_CONTENT = 0x02
REGISTER_OUTPUT_RATE = 0x03
REGISTER_BAUD = 0x04
REGISTER_KEY = 0x69
KEY_UNLOCK = 0xB588
SAVE_PARAMETERS = 0x0000
REGISTER_SETTLE_S = 0.020

OUTPUT_TIME = 0x001
OUTPUT_ACCEL = 0x002
OUTPUT_GYRO = 0x004
LIO_OUTPUT_MASK = OUTPUT_TIME | OUTPUT_ACCEL | OUTPUT_GYRO

RATE_TO_REGISTER = {
    0.2: 0x01,
    0.5: 0x02,
    1.0: 0x03,
    2.0: 0x04,
    5.0: 0x05,
    10.0: 0x06,
    20.0: 0x07,
    50.0: 0x08,
    100.0: 0x09,
    125.0: 0x0A,
    200.0: 0x0B,
}
BAUD_TO_REGISTER = {
    4800: 1,
    9600: 2,
    19200: 3,
    38400: 4,
    57600: 5,
    115200: 6,
    230400: 7,
}
PROBE_BAUDS = (9600, 230400, 115200, 57600, 38400, 19200, 4800)
FRAME_TYPE_TO_MASK = {
    0x50: 0x001,
    0x51: 0x002,
    0x52: 0x004,
    0x53: 0x008,
    0x54: 0x010,
    0x55: 0x020,
    0x56: 0x040,
    0x57: 0x080,
    0x58: 0x100,
    0x59: 0x200,
    0x5A: 0x400,
}


@dataclass(frozen=True)
class Im10aStreamAudit:
    endpoint: str
    baud: int
    duration_s: float
    valid_frames: int
    checksum_errors: int
    payload_errors: int
    discarded_bytes: int
    frame_type_counts: dict[str, int]
    output_mask_observed: int
    sample_rate_hz: float | None
    sensor_period_median_ms: float | None
    sensor_period_p95_error_ms: float | None
    sensor_time_non_monotonic: int
    sensor_time_calendar_valid: bool | None
    estimated_drops: int
    lio_profile: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _write_register(port: Any, register: int, value: int) -> None:
    packet = bytes(
        (
            0xFF,
            0xAA,
            register & 0xFF,
            value & 0xFF,
            (value >> 8) & 0xFF,
        )
    )
    written = port.write(packet)
    port.flush()
    if written != len(packet):
        raise OSError(f"short IM10A register write at 0x{register:02x}")


def _write_unlocked(port: Any, register: int, value: int) -> None:
    _write_register(port, REGISTER_KEY, KEY_UNLOCK)
    time.sleep(REGISTER_SETTLE_S)
    _write_register(port, register, value)
    time.sleep(REGISTER_SETTLE_S)


def _nearest_rate(value: float | None) -> float:
    if value is None or not math.isfinite(value):
        return 10.0
    return min(RATE_TO_REGISTER, key=lambda candidate: abs(candidate - value))


def _observed_output_mask(decoder: Im10aDecoder) -> int:
    mask = 0
    for frame_type in decoder.frame_type_counts:
        mask |= FRAME_TYPE_TO_MASK.get(frame_type, 0)
    return mask


def audit_stream(
    endpoint: str,
    baud: int,
    *,
    duration_s: float = 2.0,
) -> Im10aStreamAudit:
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(f"pyserial is unavailable: {exc}") from exc

    decoder = Im10aDecoder()
    sensor_times: list[float] = []
    sensor_calendar_validity: list[bool] = []
    started = time.monotonic()
    with serial.Serial(
        endpoint,
        baud,
        timeout=0.02,
        exclusive=True,
    ) as port:
        port.reset_input_buffer()
        while time.monotonic() - started < duration_s:
            data = port.read(max(1, port.in_waiting))
            if not data:
                continue
            for measurement in decoder.feed(data):
                if (
                    measurement.kind == "sensor_time"
                    and measurement.sensor_time_s is not None
                ):
                    sensor_times.append(measurement.sensor_time_s)
                    if measurement.sensor_time_calendar_valid is not None:
                        sensor_calendar_validity.append(
                            measurement.sensor_time_calendar_valid
                        )

    elapsed_s = max(1.0e-6, time.monotonic() - started)
    output_mask = _observed_output_mask(decoder)
    per_sample_counts = [
        count
        for frame_type, count in decoder.frame_type_counts.items()
        if frame_type in FRAME_TYPE_TO_MASK
    ]
    samples = max(per_sample_counts, default=0)
    sample_rate_hz = samples / elapsed_s if samples else None

    periods = [
        current - previous
        for previous, current in zip(sensor_times, sensor_times[1:])
    ]
    positive_periods = [period for period in periods if period > 0.0]
    non_monotonic = len(periods) - len(positive_periods)
    median_period_s = (
        statistics.median(positive_periods) if positive_periods else None
    )
    period_errors_ms: list[float] = []
    estimated_drops = 0
    if median_period_s is not None and median_period_s > 0.0:
        period_errors_ms = [
            abs(period - median_period_s) * 1000.0
            for period in positive_periods
        ]
        estimated_drops = sum(
            max(0, int(round(period / median_period_s)) - 1)
            for period in positive_periods
        )
    p95_error_ms = (
        float(statistics.quantiles(period_errors_ms, n=20)[18])
        if len(period_errors_ms) >= 20
        else (max(period_errors_ms) if period_errors_ms else None)
    )
    lio_profile = (
        output_mask == LIO_OUTPUT_MASK
        and sample_rate_hz is not None
        and 180.0 <= sample_rate_hz <= 220.0
        and len(sensor_times) >= 10
        and decoder.checksum_errors == 0
        and decoder.payload_errors == 0
        and non_monotonic == 0
        and estimated_drops == 0
    )
    return Im10aStreamAudit(
        endpoint=endpoint,
        baud=baud,
        duration_s=elapsed_s,
        valid_frames=decoder.valid_frames,
        checksum_errors=decoder.checksum_errors,
        payload_errors=decoder.payload_errors,
        discarded_bytes=decoder.discarded_bytes,
        frame_type_counts={
            f"0x{frame_type:02x}": count
            for frame_type, count in sorted(decoder.frame_type_counts.items())
        },
        output_mask_observed=output_mask,
        sample_rate_hz=sample_rate_hz,
        sensor_period_median_ms=(
            median_period_s * 1000.0
            if median_period_s is not None
            else None
        ),
        sensor_period_p95_error_ms=p95_error_ms,
        sensor_time_non_monotonic=non_monotonic,
        sensor_time_calendar_valid=(
            all(sensor_calendar_validity)
            if sensor_calendar_validity
            else None
        ),
        estimated_drops=estimated_drops,
        lio_profile=lio_profile,
    )


def find_stream(
    endpoint: str,
    *,
    duration_s: float = 0.8,
) -> Im10aStreamAudit:
    failures: list[str] = []
    for baud in PROBE_BAUDS:
        try:
            audit = audit_stream(endpoint, baud, duration_s=duration_s)
        except OSError as exc:
            failures.append(f"{baud}: {exc}")
            continue
        if audit.valid_frames >= 2:
            return audit
    detail = "; ".join(failures[-3:])
    raise RuntimeError(
        f"no valid IM10A stream found on {endpoint}"
        + (f" ({detail})" if detail else "")
    )


def _cube_is_disarmed(config: ProjectConfig) -> bool:
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError(f"pymavlink is unavailable: {exc}") from exc

    connection = mavutil.mavlink_connection(
        config.flight_controller.endpoint,
        baud=config.flight_controller.baud,
        source_system=config.flight_controller.companion_system_id,
        source_component=config.flight_controller.companion_component_id,
        autoreconnect=False,
    )
    try:
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=config.flight_controller.heartbeat_timeout_s,
        )
        if heartbeat is None:
            raise RuntimeError("Cube heartbeat timed out; refusing IM10A write")
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        return not bool(int(heartbeat.base_mode) & armed_flag)
    finally:
        connection.close()


def _set_profile(
    endpoint: str,
    current_baud: int,
    *,
    output_mask: int,
    rate_hz: float,
    target_baud: int,
    save: bool,
) -> None:
    import serial

    rate_register = RATE_TO_REGISTER[rate_hz]
    baud_register = BAUD_TO_REGISTER[target_baud]
    if target_baud != current_baud:
        with serial.Serial(
            endpoint,
            current_baud,
            timeout=0.1,
            exclusive=True,
        ) as port:
            port.reset_input_buffer()
            _write_unlocked(port, REGISTER_BAUD, baud_register)
        time.sleep(0.1)

    with serial.Serial(
        endpoint,
        target_baud,
        timeout=0.1,
        exclusive=True,
    ) as port:
        port.reset_input_buffer()
        _write_unlocked(port, REGISTER_OUTPUT_CONTENT, output_mask)
        _write_unlocked(port, REGISTER_OUTPUT_RATE, rate_register)
        if save:
            _write_unlocked(port, REGISTER_SAVE, SAVE_PARAMETERS)


def apply_lio_profile(
    config: ProjectConfig,
    *,
    backup_dir: Path,
    validation_s: float = 10.0,
) -> tuple[Path, Im10aStreamAudit]:
    if not _cube_is_disarmed(config):
        raise RuntimeError("Cube is armed; refusing to configure the IM10A")

    before = find_stream(config.external_imu.symlink)
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup_path = backup_dir / f"{stamp}-im10a-profile-backup.json"
    backup_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "captured_utc": datetime.now(timezone.utc).isoformat(),
                "audit": before.as_dict(),
                "restore_rate_hz": _nearest_rate(before.sample_rate_hz),
                "restore_output_mask": before.output_mask_observed,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="ascii",
    )

    restore_mask = before.output_mask_observed or (
        OUTPUT_ACCEL | OUTPUT_GYRO | 0x008 | 0x200
    )
    restore_rate = _nearest_rate(before.sample_rate_hz)
    try:
        _set_profile(
            config.external_imu.symlink,
            before.baud,
            output_mask=LIO_OUTPUT_MASK,
            rate_hz=200.0,
            target_baud=230400,
            save=False,
        )
        after = audit_stream(
            config.external_imu.symlink,
            230400,
            duration_s=validation_s,
        )
        if not after.lio_profile:
            failed_audit_path = backup_path.with_name(
                backup_path.name.replace(
                    "-profile-backup.json",
                    "-profile-failed-audit.json",
                )
            )
            failed_audit_path.write_text(
                json.dumps(after.as_dict(), indent=2, sort_keys=True) + "\n",
                encoding="ascii",
            )
            raise RuntimeError(
                "IM10A did not pass the 200 Hz time+accel+gyro validation; "
                f"failed audit: {failed_audit_path}"
            )
        _set_profile(
            config.external_imu.symlink,
            230400,
            output_mask=LIO_OUTPUT_MASK,
            rate_hz=200.0,
            target_baud=230400,
            save=True,
        )
        final = audit_stream(
            config.external_imu.symlink,
            230400,
            duration_s=max(2.0, min(validation_s, 5.0)),
        )
        if not final.lio_profile:
            raise RuntimeError("saved IM10A profile failed its final audit")
        return backup_path, final
    except Exception:
        recovered = find_stream(config.external_imu.symlink)
        _set_profile(
            config.external_imu.symlink,
            recovered.baud,
            output_mask=restore_mask,
            rate_hz=restore_rate,
            target_baud=before.baud,
            save=True,
        )
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Audit or reversibly configure the IM10A LIO stream",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "system.yaml",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--apply-lio-profile",
        action="store_true",
        help="Set 230400 baud, 200 Hz, and TIME+ACC+GYRO after a disarm check",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config)
        if args.apply_lio_profile:
            backup, audit = apply_lio_profile(
                config,
                backup_dir=PROJECT_ROOT / "data" / "calibrations" / "im10a",
                validation_s=max(5.0, args.duration),
            )
            payload = {
                "result": "applied",
                "backup": str(backup),
                "audit": audit.as_dict(),
                "config_update_required": {
                    "baud": 230400,
                    "expected_rate_hz": 200,
                    "sensor_time_enabled": True,
                },
            }
        else:
            payload = {
                "result": "read_only_audit",
                "audit": find_stream(
                    config.external_imu.symlink,
                    duration_s=args.duration,
                ).as_dict(),
            }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    except (ConfigError, OSError, RuntimeError, ValueError) as exc:
        print(f"IM10A configuration error: {exc}")
        return 2
