"""Stationary, read-only IM10A noise characterization for LIO."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any, Callable

import numpy as np

from .config import ConfigError, ProjectConfig, load_config
from .im10a import Im10aDecoder, Im10aSampleAssembler, STANDARD_GRAVITY_MSS
from .im10a_config import find_stream
from .jt16_calibration import CubeCalibrationLink
from .paths import CALIBRATION_DIR, CONFIG_DIR


DEFAULT_CONFIG = CONFIG_DIR / "system.yaml"
FLIGHT_SERVICE = "optflow-flight-logger.service"
DEFAULT_DURATION_S = 1800.0
DEFAULT_SETTLE_S = 10.0
PROGRESS_INTERVAL_S = 30.0
MOTION_GYRO_THRESHOLD_RADS = 0.05
MOTION_ACCEL_THRESHOLD_MPSS = 0.50
STATIONARY_GYRO_P999_LIMIT_RADS = 0.03
STATIONARY_ACCEL_P999_LIMIT_MPSS = 0.30
STATIONARY_GYRO_DRIFT_LIMIT_RADS = 0.01
STATIONARY_ACCEL_DRIFT_LIMIT_MPSS = 0.10
ACCEL_LSB_MPSS = 16.0 * STANDARD_GRAVITY_MSS / 32768.0
GYRO_LSB_RADS = math.radians(2000.0) / 32768.0


class MotionDetected(RuntimeError):
    """Raised when the supposedly stationary sensor is physically moved."""


def _service_is_active() -> bool:
    completed = subprocess.run(
        ("systemctl", "--user", "is-active", FLIGHT_SERVICE),
        capture_output=True,
        text=True,
        check=False,
    )
    return completed.returncode == 0 and completed.stdout.strip() == "active"


def _service_action(action: str) -> None:
    completed = subprocess.run(
        ("systemctl", "--user", action, FLIGHT_SERVICE),
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(
            f"unable to {action} {FLIGHT_SERVICE}: {detail}"
        )


def allan_deviation(
    samples: np.ndarray,
    sample_period_s: float,
    *,
    maximum_points: int = 48,
) -> tuple[np.ndarray, np.ndarray]:
    """Return non-overlapping Allan deviation for each sample column."""
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, np.newaxis]
    if values.ndim != 2 or values.shape[0] < 100:
        raise ValueError("at least 100 one- or multi-axis samples are required")
    if sample_period_s <= 0.0 or not math.isfinite(sample_period_s):
        raise ValueError("sample_period_s must be positive and finite")

    maximum_cluster = max(1, values.shape[0] // 10)
    cluster_sizes = np.unique(
        np.geomspace(1, maximum_cluster, maximum_points).astype(np.int64)
    )
    taus: list[float] = []
    deviations: list[np.ndarray] = []
    for cluster_size in cluster_sizes:
        cluster_count = values.shape[0] // int(cluster_size)
        if cluster_count < 10:
            continue
        usable = cluster_count * int(cluster_size)
        means = values[:usable].reshape(
            cluster_count,
            int(cluster_size),
            values.shape[1],
        ).mean(axis=1)
        differences = np.diff(means, axis=0)
        deviations.append(
            np.sqrt(0.5 * np.mean(np.square(differences), axis=0))
        )
        taus.append(float(cluster_size) * sample_period_s)
    return np.asarray(taus), np.stack(deviations)


def noise_density_from_allan(
    taus_s: np.ndarray,
    deviations: np.ndarray,
) -> np.ndarray:
    """Estimate white-noise density from the short-tau Allan region."""
    taus = np.asarray(taus_s, dtype=np.float64)
    values = np.asarray(deviations, dtype=np.float64)
    mask = (taus >= 0.02) & (taus <= 1.0)
    if np.count_nonzero(mask) < 3:
        mask = np.arange(len(taus)) < min(6, len(taus))
    return np.median(values[mask] * np.sqrt(taus[mask, np.newaxis]), axis=0)


def stationary_metrics(
    accel_mss: np.ndarray,
    gyro_rads: np.ndarray,
    sample_rate_hz: float,
) -> dict[str, Any]:
    accel = np.asarray(accel_mss, dtype=np.float64)
    gyro = np.asarray(gyro_rads, dtype=np.float64)
    if accel.shape != gyro.shape or accel.ndim != 2 or accel.shape[1] != 3:
        raise ValueError("accel and gyro must be matching Nx3 arrays")
    if accel.shape[0] < 100 or sample_rate_hz <= 0.0:
        raise ValueError("stationary metrics require at least 100 samples")

    accel_center = np.median(accel, axis=0)
    gyro_center = np.median(gyro, axis=0)
    accel_residual_norm = np.linalg.norm(accel - accel_center, axis=1)
    gyro_residual_norm = np.linalg.norm(gyro - gyro_center, axis=1)
    window = min(accel.shape[0] // 2, max(10, round(sample_rate_hz * 60.0)))
    accel_drift = np.mean(accel[-window:], axis=0) - np.mean(
        accel[:window], axis=0
    )
    gyro_drift = np.mean(gyro[-window:], axis=0) - np.mean(
        gyro[:window], axis=0
    )
    accel_p999 = float(np.percentile(accel_residual_norm, 99.9))
    gyro_p999 = float(np.percentile(gyro_residual_norm, 99.9))
    accel_drift_norm = float(np.linalg.norm(accel_drift))
    gyro_drift_norm = float(np.linalg.norm(gyro_drift))
    gravity_norm = float(np.linalg.norm(np.mean(accel, axis=0)))
    stationary = bool(
        accel_p999 <= STATIONARY_ACCEL_P999_LIMIT_MPSS
        and gyro_p999 <= STATIONARY_GYRO_P999_LIMIT_RADS
        and accel_drift_norm <= STATIONARY_ACCEL_DRIFT_LIMIT_MPSS
        and gyro_drift_norm <= STATIONARY_GYRO_DRIFT_LIMIT_RADS
        and abs(gravity_norm - STANDARD_GRAVITY_MSS) <= 0.35
    )
    return {
        "stationary": stationary,
        "gravity_norm_mpss": gravity_norm,
        "accel_mean_mpss": np.mean(accel, axis=0).tolist(),
        "gyro_mean_rads": np.mean(gyro, axis=0).tolist(),
        "accel_std_mpss": np.std(accel, axis=0, ddof=1).tolist(),
        "gyro_std_rads": np.std(gyro, axis=0, ddof=1).tolist(),
        "accel_variance_mpss2": np.var(accel, axis=0, ddof=1).tolist(),
        "gyro_variance_rads2": np.var(gyro, axis=0, ddof=1).tolist(),
        "accel_residual_norm_p999_mpss": accel_p999,
        "gyro_residual_norm_p999_rads": gyro_p999,
        "accel_first_to_last_60s_drift_mpss": accel_drift.tolist(),
        "gyro_first_to_last_60s_drift_rads": gyro_drift.tolist(),
        "accel_drift_norm_mpss": accel_drift_norm,
        "gyro_drift_norm_rads": gyro_drift_norm,
        "limits": {
            "accel_residual_norm_p999_mpss": (
                STATIONARY_ACCEL_P999_LIMIT_MPSS
            ),
            "gyro_residual_norm_p999_rads": (
                STATIONARY_GYRO_P999_LIMIT_RADS
            ),
            "accel_drift_norm_mpss": STATIONARY_ACCEL_DRIFT_LIMIT_MPSS,
            "gyro_drift_norm_rads": STATIONARY_GYRO_DRIFT_LIMIT_RADS,
            "gravity_error_mpss": 0.35,
        },
    }


def _grow(array: np.ndarray) -> np.ndarray:
    grown = np.empty((max(array.shape[0] + 1, int(array.shape[0] * 1.5)), *array.shape[1:]), dtype=array.dtype)
    grown[: array.shape[0]] = array
    return grown


def collect_samples(
    config: ProjectConfig,
    *,
    duration_s: float,
    settle_s: float,
    progress: Callable[[float, int], None] | None = None,
) -> dict[str, Any]:
    if duration_s <= 0.0 or settle_s < 0.0:
        raise ValueError("duration must be positive and settle time non-negative")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(f"pyserial is unavailable: {exc}") from exc

    expected_rate = config.external_imu.expected_rate_hz
    capacity = max(1000, math.ceil((duration_s + 5.0) * expected_rate * 1.05))
    sensor_times = np.empty(capacity, dtype=np.float64)
    host_times = np.empty(capacity, dtype=np.int64)
    accel = np.empty((capacity, 3), dtype=np.float64)
    gyro = np.empty((capacity, 3), dtype=np.float64)
    count = 0
    decoder = Im10aDecoder()
    assembler = Im10aSampleAssembler()
    signs = np.asarray(
        (
            config.external_imu.body_axis_signs.x,
            config.external_imu.body_axis_signs.y,
            config.external_imu.body_axis_signs.z,
        ),
        dtype=np.float64,
    )
    baseline_accel: np.ndarray | None = None
    baseline_gyro: np.ndarray | None = None
    motion_windows = 0
    last_sensor_time: float | None = None
    sensor_time_offset_s = 0.0
    started_s = time.monotonic()
    capture_started_s = started_s + settle_s
    capture_ends_s = capture_started_s + duration_s
    next_progress_s = capture_started_s + PROGRESS_INTERVAL_S
    recent_accel: list[np.ndarray] = []
    recent_gyro: list[np.ndarray] = []

    with serial.Serial(
        config.external_imu.symlink,
        config.external_imu.baud,
        timeout=0.02,
        exclusive=True,
    ) as port:
        port.reset_input_buffer()
        while time.monotonic() < capture_ends_s:
            now_s = time.monotonic()
            data = port.read(max(1, port.in_waiting))
            if not data:
                continue
            for measurement in decoder.feed(data):
                sample = assembler.push(measurement, time.monotonic_ns())
                if sample is None or now_s < capture_started_s:
                    continue
                if count >= sensor_times.shape[0]:
                    sensor_times = _grow(sensor_times)
                    host_times = _grow(host_times)
                    accel = _grow(accel)
                    gyro = _grow(gyro)
                sensor_time_s = sample.sensor_time_s
                if (
                    last_sensor_time is not None
                    and sensor_time_s + sensor_time_offset_s
                    < last_sensor_time - 43_200.0
                ):
                    sensor_time_offset_s += 86_400.0
                sensor_time_s += sensor_time_offset_s
                last_sensor_time = sensor_time_s
                body_accel = np.asarray(sample.accel_mss) * signs
                body_gyro = np.asarray(sample.gyro_rads) * signs
                sensor_times[count] = sensor_time_s
                host_times[count] = sample.host_monotonic_ns
                accel[count] = body_accel
                gyro[count] = body_gyro
                count += 1
                recent_accel.append(body_accel)
                recent_gyro.append(body_gyro)

                window_samples = max(20, round(expected_rate))
                baseline_samples = max(100, round(expected_rate * 5.0))
                if baseline_accel is None and count >= baseline_samples:
                    baseline_accel = np.median(accel[:count], axis=0)
                    baseline_gyro = np.median(gyro[:count], axis=0)
                    recent_accel.clear()
                    recent_gyro.clear()
                elif (
                    baseline_accel is not None
                    and baseline_gyro is not None
                    and len(recent_accel) >= window_samples
                ):
                    accel_peak = float(
                        np.percentile(
                            np.linalg.norm(
                                np.asarray(recent_accel) - baseline_accel,
                                axis=1,
                            ),
                            95,
                        )
                    )
                    gyro_peak = float(
                        np.percentile(
                            np.linalg.norm(
                                np.asarray(recent_gyro) - baseline_gyro,
                                axis=1,
                            ),
                            95,
                        )
                    )
                    recent_accel.clear()
                    recent_gyro.clear()
                    if (
                        accel_peak > MOTION_ACCEL_THRESHOLD_MPSS
                        or gyro_peak > MOTION_GYRO_THRESHOLD_RADS
                    ):
                        motion_windows += 1
                    else:
                        motion_windows = 0
                    if motion_windows >= 2:
                        raise MotionDetected(
                            "IM10A moved during stationary capture "
                            f"(accel p95 {accel_peak:.3f} m/s^2, "
                            f"gyro p95 {gyro_peak:.4f} rad/s)"
                        )

            if progress is not None and now_s >= next_progress_s:
                progress(min(duration_s, now_s - capture_started_s), count)
                next_progress_s += PROGRESS_INTERVAL_S

    if count < 100:
        raise RuntimeError(f"only {count} complete IM10A samples were captured")
    return {
        "sensor_time_s": sensor_times[:count].copy(),
        "host_monotonic_ns": host_times[:count].copy(),
        "accel_mss": accel[:count].copy(),
        "gyro_rads": gyro[:count].copy(),
        "decoder": {
            "valid_frames": decoder.valid_frames,
            "checksum_errors": decoder.checksum_errors,
            "payload_errors": decoder.payload_errors,
            "discarded_bytes": decoder.discarded_bytes,
            "frame_type_counts": {
                f"0x{frame_type:02x}": frame_count
                for frame_type, frame_count in sorted(
                    decoder.frame_type_counts.items()
                )
            },
            "completed_samples": assembler.completed_samples,
            "incomplete_samples": assembler.incomplete_samples,
        },
    }


def analyze_capture(capture: dict[str, Any]) -> dict[str, Any]:
    sensor_times = np.asarray(capture["sensor_time_s"], dtype=np.float64)
    accel = np.asarray(capture["accel_mss"], dtype=np.float64)
    gyro = np.asarray(capture["gyro_rads"], dtype=np.float64)
    periods = np.diff(sensor_times)
    positive_periods = periods[periods > 0.0]
    non_monotonic = int(np.count_nonzero(periods <= 0.0))
    if positive_periods.size == 0:
        raise RuntimeError("IM10A sensor time did not advance")
    period_s = float(np.median(positive_periods))
    duration_s = float(sensor_times[-1] - sensor_times[0])
    rate_hz = (sensor_times.size - 1) / duration_s
    estimated_drops = int(
        np.sum(np.maximum(0, np.rint(positive_periods / period_s).astype(int) - 1))
    )
    metrics = stationary_metrics(accel, gyro, rate_hz)
    taus, accel_allan = allan_deviation(accel, period_s)
    _, gyro_allan = allan_deviation(gyro, period_s)
    accel_density = noise_density_from_allan(taus, accel_allan)
    gyro_density = noise_density_from_allan(taus, gyro_allan)
    accel_quantization_density = (
        ACCEL_LSB_MPSS / math.sqrt(12.0) * math.sqrt(period_s)
    )
    gyro_quantization_density = (
        GYRO_LSB_RADS / math.sqrt(12.0) * math.sqrt(period_s)
    )
    effective_accel_density = np.maximum(
        accel_density,
        accel_quantization_density,
    )
    effective_gyro_density = np.maximum(
        gyro_density,
        gyro_quantization_density,
    )

    def bias_observation(
        deviation: np.ndarray,
    ) -> tuple[list[float | None], list[float | None]]:
        values: list[float | None] = []
        minimum_taus: list[float | None] = []
        for axis in range(deviation.shape[1]):
            positive = np.flatnonzero(deviation[:, axis] > 0.0)
            if positive.size == 0:
                values.append(None)
                minimum_taus.append(None)
                continue
            index = int(positive[np.argmin(deviation[positive, axis])])
            values.append(float(deviation[index, axis] / 0.664))
            minimum_taus.append(float(taus[index]))
        return values, minimum_taus

    accel_bias, accel_bias_taus = bias_observation(accel_allan)
    gyro_bias, gyro_bias_taus = bias_observation(gyro_allan)
    accel_unique_levels = [
        int(np.unique(accel[:, axis]).size) for axis in range(3)
    ]
    gyro_unique_levels = [
        int(np.unique(gyro[:, axis]).size) for axis in range(3)
    ]
    decoder = capture["decoder"]
    passed = bool(
        metrics["stationary"]
        and 180.0 <= rate_hz <= 220.0
        and non_monotonic == 0
        and estimated_drops == 0
        and int(decoder["checksum_errors"]) == 0
        and int(decoder["payload_errors"]) == 0
        and int(decoder["incomplete_samples"]) == 0
    )
    return {
        "result": "pass" if passed else "fail",
        "samples": int(sensor_times.size),
        "sensor_duration_s": duration_s,
        "sample_rate_hz": rate_hz,
        "sensor_period_median_ms": period_s * 1000.0,
        "sensor_period_p95_error_ms": float(
            np.percentile(np.abs(positive_periods - period_s), 95) * 1000.0
        ),
        "sensor_time_non_monotonic": non_monotonic,
        "estimated_drops": estimated_drops,
        "decoder": decoder,
        "stationary": metrics,
        "allan": {
            "tau_s": taus.tolist(),
            "accel_deviation_mpss": accel_allan.tolist(),
            "gyro_deviation_rads": gyro_allan.tolist(),
            "accel_white_noise_density_mpss_sqrt_s": accel_density.tolist(),
            "gyro_white_noise_density_rads_sqrt_s": gyro_density.tolist(),
            "effective_accel_white_noise_density_mpss_sqrt_s": (
                effective_accel_density.tolist()
            ),
            "effective_gyro_white_noise_density_rads_sqrt_s": (
                effective_gyro_density.tolist()
            ),
            "accel_bias_instability_mpss": accel_bias,
            "gyro_bias_instability_rads": gyro_bias,
            "accel_bias_minimum_tau_s": accel_bias_taus,
            "gyro_bias_minimum_tau_s": gyro_bias_taus,
        },
        "quantization": {
            "accel_lsb_mpss": ACCEL_LSB_MPSS,
            "gyro_lsb_rads": GYRO_LSB_RADS,
            "accel_uniform_noise_std_mpss": ACCEL_LSB_MPSS / math.sqrt(12.0),
            "gyro_uniform_noise_std_rads": GYRO_LSB_RADS / math.sqrt(12.0),
            "accel_noise_density_floor_mpss_sqrt_s": (
                accel_quantization_density
            ),
            "gyro_noise_density_floor_rads_sqrt_s": (
                gyro_quantization_density
            ),
            "accel_unique_levels": accel_unique_levels,
            "gyro_unique_levels": gyro_unique_levels,
            "gyro_allan_noise_observable": [
                levels > 1 for levels in gyro_unique_levels
            ],
        },
        "fast_lio_candidate": {
            "apply_automatically": False,
            "acc_cov_measurement_noise_candidate": float(
                np.max(np.square(effective_accel_density))
            ),
            "gyr_cov_measurement_noise_candidate": float(
                np.max(np.square(effective_gyro_density))
            ),
            "b_acc_cov": None,
            "b_gyr_cov": None,
            "note": (
                "White-noise candidates require shadow A/B validation; "
                "bias random-walk covariance is not inferred from one "
                "30-minute stationary run."
            ),
        },
    }


def _write_capture(
    session: Path,
    config_path: Path,
    capture: dict[str, Any],
    analysis: dict[str, Any],
) -> tuple[Path, str]:
    session.mkdir(parents=True, exist_ok=False)
    raw_path = session / "samples.npz"
    temporary_raw = session / "samples.npz.tmp"
    with temporary_raw.open("wb") as target:
        np.savez_compressed(
            target,
            sensor_time_s=capture["sensor_time_s"],
            host_monotonic_ns=capture["host_monotonic_ns"],
            accel_mss=capture["accel_mss"],
            gyro_rads=capture["gyro_rads"],
        )
    temporary_raw.replace(raw_path)
    raw_digest = hashlib.sha256(raw_path.read_bytes()).hexdigest()
    config_digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
    report = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "kind": "im10a_stationary_noise_profile",
        "read_only": True,
        "cube_parameter_writes": False,
        "fast_lio_parameter_writes": False,
        "config_source": str(config_path),
        "config_sha256": config_digest,
        "raw_samples": raw_path.name,
        "raw_samples_sha256": raw_digest,
        **analysis,
    }
    report_path = session / "report.json"
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    temporary_report = report_path.with_suffix(".json.tmp")
    temporary_report.write_bytes(report_bytes)
    temporary_report.replace(report_path)
    digest = hashlib.sha256(report_bytes).hexdigest()
    report_path.with_suffix(".sha256").write_text(
        f"{digest}  {report_path.name}\n",
        encoding="ascii",
    )
    return report_path, digest


def run_calibration(
    config: ProjectConfig,
    config_path: Path,
    *,
    duration_s: float,
    settle_s: float,
    output_root: Path,
    progress: Callable[[float, int], None] | None = None,
) -> tuple[Path, dict[str, Any], str]:
    live_audit = find_stream(
        config.external_imu.symlink,
        duration_s=2.0,
    )
    if not live_audit.lio_profile:
        raise RuntimeError(
            "live IM10A is not the verified 200 Hz TIME+ACC+GYRO profile"
        )
    capture = collect_samples(
        config,
        duration_s=duration_s,
        settle_s=settle_s,
        progress=progress,
    )
    analysis = analyze_capture(capture)
    analysis["pre_capture_audit"] = live_audit.as_dict()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session = output_root / f"{stamp}-stationary-noise"
    report_path, digest = _write_capture(
        session,
        config_path,
        capture,
        analysis,
    )
    return report_path, analysis, digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--settle-seconds", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=CALIBRATION_DIR / "im10a" / "noise",
    )
    parser.add_argument("--no-beep", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    service_was_active = False
    link: CubeCalibrationLink | None = None
    beep_sent = False

    def intervention_beep() -> None:
        nonlocal beep_sent
        if args.no_beep or beep_sent or link is None:
            return
        link.beep()
        beep_sent = True

    def progress(elapsed_s: float, samples: int) -> None:
        print(
            "IMU_NOISE_PROGRESS "
            f"elapsed_s={elapsed_s:.0f} samples={samples}",
            flush=True,
        )

    try:
        if args.duration < 60.0:
            raise ConfigError("IMU noise capture must be at least 60 seconds")
        if args.settle_seconds < 0.0:
            raise ConfigError("settle time must be non-negative")
        config_path = args.config.resolve()
        config = load_config(config_path)
        if not config.external_imu.sensor_time_enabled:
            raise ConfigError("IM10A sensor time must be enabled")
        service_was_active = _service_is_active()
        if service_was_active:
            _service_action("stop")
        link = CubeCalibrationLink(config.flight_controller)
        link.open()
        print(
            "IMU_NOISE_CAPTURE_STARTED "
            f"duration_s={args.duration:.0f} settle_s={args.settle_seconds:.0f}",
            flush=True,
        )
        report_path, analysis, digest = run_calibration(
            config,
            config_path,
            duration_s=args.duration,
            settle_s=args.settle_seconds,
            output_root=args.output_root.resolve(),
            progress=progress,
        )
        intervention_beep()
        print(
            json.dumps(
                {
                    "result": analysis["result"],
                    "report": str(report_path),
                    "sha256": digest,
                    "samples": analysis["samples"],
                    "sample_rate_hz": analysis["sample_rate_hz"],
                    "intervention": "capture complete; the drone may be touched",
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if analysis["result"] == "pass" else 1
    except KeyboardInterrupt:
        intervention_beep()
        print("IM10A noise calibration interrupted; keep the drone disarmed")
        return 130
    except (ConfigError, MotionDetected, OSError, RuntimeError, ValueError) as exc:
        intervention_beep()
        print(f"IM10A noise calibration error: {exc}")
        return 2
    finally:
        if link is not None:
            link.close()
        if service_was_active:
            try:
                _service_action("start")
            except RuntimeError as exc:
                print(f"Flight logger restore error: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
