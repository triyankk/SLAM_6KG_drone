"""Robustly map independent sensor clocks onto Jetson monotonic time."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ClockFit:
    samples: int
    inliers: int
    span_s: float
    slope: float | None
    offset_s: float | None
    drift_ppm: float | None
    residual_rms_ms: float | None
    residual_p95_ms: float | None
    resets: int
    ready: bool

    def as_dict(self) -> dict[str, int | float | bool | None]:
        return asdict(self)


class AffineClockMapper:
    """Fit ``host_monotonic = slope * sensor + offset`` without epoch assumptions."""

    def __init__(
        self,
        *,
        window_samples: int = 2000,
        minimum_samples: int = 50,
        minimum_span_s: float = 1.0,
        maximum_drift_ppm: float = 5000.0,
        maximum_residual_p95_ms: float = 10.0,
        maximum_window_span_s: float | None = None,
        reset_tolerance_s: float = 0.001,
    ) -> None:
        if window_samples < minimum_samples or minimum_samples < 3:
            raise ValueError("clock sample limits are invalid")
        if minimum_span_s <= 0.0:
            raise ValueError("minimum_span_s must be positive")
        if maximum_drift_ppm <= 0.0 or maximum_residual_p95_ms <= 0.0:
            raise ValueError("clock fit thresholds must be positive")
        if (
            maximum_window_span_s is not None
            and maximum_window_span_s < minimum_span_s
        ):
            raise ValueError(
                "maximum_window_span_s must cover minimum_span_s"
            )
        self.window_samples = window_samples
        self.minimum_samples = minimum_samples
        self.minimum_span_s = minimum_span_s
        self.maximum_drift_ppm = maximum_drift_ppm
        self.maximum_residual_p95_ms = maximum_residual_p95_ms
        self.maximum_window_span_s = maximum_window_span_s
        self.reset_tolerance_s = reset_tolerance_s
        self._pairs: deque[tuple[float, float]] = deque(maxlen=window_samples)
        self._fit: ClockFit | None = None
        self._resets = 0

    @property
    def resets(self) -> int:
        return self._resets

    @property
    def ready(self) -> bool:
        return bool(self.fit.ready)

    @property
    def fit(self) -> ClockFit:
        if self._fit is None:
            span_s = (
                self._pairs[-1][0] - self._pairs[0][0]
                if len(self._pairs) >= 2
                else 0.0
            )
            return ClockFit(
                samples=len(self._pairs),
                inliers=0,
                span_s=max(0.0, span_s),
                slope=None,
                offset_s=None,
                drift_ppm=None,
                residual_rms_ms=None,
                residual_p95_ms=None,
                resets=self._resets,
                ready=False,
            )
        return self._fit

    def clear(self, *, count_reset: bool = False) -> None:
        self._pairs.clear()
        self._fit = None
        if count_reset:
            self._resets += 1

    def add(self, sensor_time_s: float, host_monotonic_s: float) -> ClockFit:
        if not (
            math.isfinite(sensor_time_s)
            and math.isfinite(host_monotonic_s)
        ):
            raise ValueError("clock observations must be finite")
        if self._pairs:
            last_sensor, last_host = self._pairs[-1]
            if (
                sensor_time_s < last_sensor - self.reset_tolerance_s
                or host_monotonic_s < last_host - self.reset_tolerance_s
            ):
                self.clear(count_reset=True)
            elif sensor_time_s <= last_sensor:
                return self.fit

        self._pairs.append((sensor_time_s, host_monotonic_s))
        self._trim_window()
        self._fit = self._calculate_fit()
        return self.fit

    def _trim_window(self) -> None:
        maximum_span_s = self.maximum_window_span_s
        if maximum_span_s is None or len(self._pairs) < 2:
            return
        latest_sensor_s = self._pairs[-1][0]
        while (
            len(self._pairs) > self.minimum_samples
            and latest_sensor_s - self._pairs[0][0] > maximum_span_s
        ):
            self._pairs.popleft()

    def map(self, sensor_time_s: float) -> float:
        fit = self.fit
        if not fit.ready or fit.slope is None or fit.offset_s is None:
            raise RuntimeError("sensor clock is not synchronized")
        mapped = fit.slope * float(sensor_time_s) + fit.offset_s
        if not math.isfinite(mapped):
            raise RuntimeError("sensor clock mapping produced a non-finite time")
        return mapped

    def _calculate_fit(self) -> ClockFit:
        count = len(self._pairs)
        sensor = np.fromiter(
            (pair[0] for pair in self._pairs),
            dtype=np.float64,
            count=count,
        )
        host = np.fromiter(
            (pair[1] for pair in self._pairs),
            dtype=np.float64,
            count=count,
        )
        span_s = float(sensor[-1] - sensor[0]) if count >= 2 else 0.0
        if count < 3 or span_s <= 0.0:
            return ClockFit(
                samples=count,
                inliers=0,
                span_s=max(0.0, span_s),
                slope=None,
                offset_s=None,
                drift_ppm=None,
                residual_rms_ms=None,
                residual_p95_ms=None,
                resets=self._resets,
                ready=False,
            )

        sensor_origin = float(np.median(sensor))
        host_origin = float(np.median(host))
        centered_sensor = sensor - sensor_origin
        centered_host = host - host_origin
        denominator = float(np.dot(centered_sensor, centered_sensor))
        if denominator <= np.finfo(np.float64).eps:
            return ClockFit(
                samples=count,
                inliers=0,
                span_s=span_s,
                slope=None,
                offset_s=None,
                drift_ppm=None,
                residual_rms_ms=None,
                residual_p95_ms=None,
                resets=self._resets,
                ready=False,
            )

        slope = float(
            np.dot(centered_sensor, centered_host) / denominator
        )
        offset_s = host_origin - slope * sensor_origin
        residual = host - (slope * sensor + offset_s)
        residual_center = float(np.median(residual))
        mad = float(np.median(np.abs(residual - residual_center)))
        robust_sigma = 1.4826 * mad
        trim_limit_s = max(0.0005, 4.0 * robust_sigma)
        inlier_mask = np.abs(residual - residual_center) <= trim_limit_s
        inliers = int(np.count_nonzero(inlier_mask))

        if inliers >= 3 and inliers < count:
            sensor_inlier = sensor[inlier_mask]
            host_inlier = host[inlier_mask]
            sensor_origin = float(np.mean(sensor_inlier))
            host_origin = float(np.mean(host_inlier))
            centered_sensor = sensor_inlier - sensor_origin
            denominator = float(np.dot(centered_sensor, centered_sensor))
            if denominator > np.finfo(np.float64).eps:
                slope = float(
                    np.dot(
                        centered_sensor,
                        host_inlier - host_origin,
                    )
                    / denominator
                )
                offset_s = host_origin - slope * sensor_origin
            evaluation_residual = (
                host_inlier - (slope * sensor_inlier + offset_s)
            )
        else:
            evaluation_residual = residual
            inliers = count

        residual_ms = np.abs(evaluation_residual) * 1000.0
        rms_ms = float(np.sqrt(np.mean(np.square(residual_ms))))
        p95_ms = float(np.percentile(residual_ms, 95))
        drift_ppm = (slope - 1.0) * 1.0e6
        ready = (
            count >= self.minimum_samples
            and inliers >= max(3, int(math.ceil(0.8 * count)))
            and span_s >= self.minimum_span_s
            and slope > 0.0
            and abs(drift_ppm) <= self.maximum_drift_ppm
            and p95_ms <= self.maximum_residual_p95_ms
        )
        return ClockFit(
            samples=count,
            inliers=inliers,
            span_s=span_s,
            slope=slope,
            offset_s=offset_s,
            drift_ppm=drift_ppm,
            residual_rms_ms=rms_ms,
            residual_p95_ms=p95_ms,
            resets=self._resets,
            ready=ready,
        )
