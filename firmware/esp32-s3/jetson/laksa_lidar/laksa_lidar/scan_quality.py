"""Bounded, ROS-independent LaserScan quality calculations."""

from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Iterable, Optional


GOOD = "GOOD"
DEGRADED = "DEGRADED"
BAD = "BAD"


@dataclass(frozen=True)
class Thresholds:
    expected_min_rate_hz: float
    expected_max_rate_hz: float
    stale_timeout_sec: float
    minimum_angular_coverage_deg: float
    maximum_consecutive_invalid: int
    provisional_degraded_valid_ratio: float
    sample_count_tolerance_fraction: float
    sample_count_tolerance_absolute: int
    maximum_scan_time_sec: float


@dataclass(frozen=True)
class ScanMetrics:
    structural_valid: bool
    errors: tuple[str, ...]
    frame_id: str
    sample_count: int
    expected_sample_count: int
    angular_coverage_rad: float
    finite_count: int
    finite_ratio: float
    valid_return_count: int
    valid_return_ratio: float
    nan_count: int
    inf_count: int
    zero_count: int
    below_range_count: int
    above_range_count: int
    finite_min: Optional[float]
    finite_median: Optional[float]
    finite_max: Optional[float]


def _valid_stamp(stamp) -> bool:
    sec = getattr(stamp, "sec", -1)
    nanosec = getattr(stamp, "nanosec", -1)
    return sec >= 0 and 0 <= nanosec < 1_000_000_000 and (sec != 0 or nanosec != 0)


def analyze_scan(scan, thresholds: Thresholds) -> ScanMetrics:
    """Validate scan geometry and calculate return statistics without altering it."""
    errors: list[str] = []
    frame_id = str(getattr(getattr(scan, "header", None), "frame_id", ""))
    stamp = getattr(getattr(scan, "header", None), "stamp", None)
    angle_min = float(getattr(scan, "angle_min", math.nan))
    angle_max = float(getattr(scan, "angle_max", math.nan))
    angle_increment = float(getattr(scan, "angle_increment", math.nan))
    range_min = float(getattr(scan, "range_min", math.nan))
    range_max = float(getattr(scan, "range_max", math.nan))
    scan_time = float(getattr(scan, "scan_time", math.nan))
    ranges: Iterable[float] = getattr(scan, "ranges", ())
    values = list(ranges)

    if not frame_id:
        errors.append("empty_frame_id")
    if stamp is None or not _valid_stamp(stamp):
        errors.append("invalid_timestamp")
    if not math.isfinite(angle_increment) or angle_increment == 0.0:
        errors.append("invalid_angle_increment")
    if not math.isfinite(angle_min) or not math.isfinite(angle_max) or angle_max <= angle_min:
        errors.append("invalid_angle_limits")
    if not math.isfinite(range_min) or not math.isfinite(range_max) or range_min < 0.0 or range_max <= range_min:
        errors.append("invalid_range_limits")
    if not values:
        errors.append("empty_ranges")
    if not math.isfinite(scan_time) or scan_time < 0.0 or scan_time > thresholds.maximum_scan_time_sec:
        errors.append("invalid_scan_time")

    coverage = angle_max - angle_min if math.isfinite(angle_min) and math.isfinite(angle_max) else 0.0
    expected_count = 0
    if coverage > 0.0 and math.isfinite(angle_increment) and angle_increment != 0.0:
        expected_count = int(round(coverage / abs(angle_increment))) + 1
        tolerance = max(
            thresholds.sample_count_tolerance_absolute,
            int(math.ceil(expected_count * thresholds.sample_count_tolerance_fraction)),
        )
        if abs(len(values) - expected_count) > tolerance:
            errors.append("sample_count_mismatch")

    finite_values: list[float] = []
    usable_values: list[float] = []
    nan_count = inf_count = zero_count = below_count = above_count = 0
    limits_valid = math.isfinite(range_min) and math.isfinite(range_max) and range_max > range_min
    for raw_value in values:
        value = float(raw_value)
        if math.isnan(value):
            nan_count += 1
        elif math.isinf(value):
            inf_count += 1
        else:
            finite_values.append(value)
            if value == 0.0:
                zero_count += 1
            if limits_valid and value < range_min:
                below_count += 1
            elif limits_valid and value > range_max:
                above_count += 1
            elif limits_valid:
                usable_values.append(value)

    count = len(values)
    finite_ratio = len(finite_values) / count if count else 0.0
    valid_ratio = len(usable_values) / count if count else 0.0
    return ScanMetrics(
        structural_valid=not errors,
        errors=tuple(errors),
        frame_id=frame_id,
        sample_count=count,
        expected_sample_count=expected_count,
        angular_coverage_rad=max(0.0, coverage),
        finite_count=len(finite_values),
        finite_ratio=finite_ratio,
        valid_return_count=len(usable_values),
        valid_return_ratio=valid_ratio,
        nan_count=nan_count,
        inf_count=inf_count,
        zero_count=zero_count,
        below_range_count=below_count,
        above_range_count=above_count,
        finite_min=min(finite_values) if finite_values else None,
        finite_median=statistics.median(finite_values) if finite_values else None,
        finite_max=max(finite_values) if finite_values else None,
    )


class RateWindow:
    """Fixed-size monotonic timestamp window."""

    def __init__(self, size: int):
        self._times = deque(maxlen=max(2, int(size)))

    def add(self, timestamp_sec: float) -> None:
        self._times.append(float(timestamp_sec))

    @property
    def rate_hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        duration = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / duration if duration > 0.0 else 0.0


def health_state(
    metrics: Optional[ScanMetrics],
    rate_hz: float,
    age_sec: float,
    tf_valid: bool,
    consecutive_invalid: int,
    thresholds: Thresholds,
) -> tuple[str, str]:
    if metrics is None or age_sec > thresholds.stale_timeout_sec:
        return BAD, "scan_stale_or_missing"
    critical_errors = set(metrics.errors) - {"sample_count_mismatch"}
    if critical_errors:
        return BAD, "malformed_scan"
    if not tf_valid:
        return BAD, "scan_to_base_tf_unavailable"
    if not metrics.structural_valid:
        if consecutive_invalid >= thresholds.maximum_consecutive_invalid:
            return BAD, "persistent_invalid_scan"
        return DEGRADED, "transient_sample_count_mismatch"
    if rate_hz > 0.0 and (
        rate_hz < thresholds.expected_min_rate_hz or rate_hz > thresholds.expected_max_rate_hz
    ):
        return DEGRADED, "scan_rate_outside_expected_range"
    if math.degrees(metrics.angular_coverage_rad) < thresholds.minimum_angular_coverage_deg:
        return DEGRADED, "angular_coverage_below_threshold"
    if metrics.valid_return_ratio < thresholds.provisional_degraded_valid_ratio:
        return DEGRADED, "valid_return_ratio_below_provisional_threshold"
    return GOOD, "healthy"
