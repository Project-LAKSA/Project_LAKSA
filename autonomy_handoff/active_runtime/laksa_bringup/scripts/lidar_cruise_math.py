#!/usr/bin/env python3

"""Pure geometry helpers for the LAKSA reactive LiDAR cruise controller."""

import math


class ReverseRecoveryGate:
    """Permit bounded reverse only after a persistent, rear-clear blockage."""

    FORWARD = "FORWARD"
    WAITING = "WAITING"
    REVERSE = "REVERSE"

    def __init__(self, blocked_before_sec: float, reverse_duration_sec: float,
                 cooldown_sec: float) -> None:
        if min(blocked_before_sec, reverse_duration_sec, cooldown_sec) <= 0.0:
            raise ValueError("Recovery timing must be positive")
        self._blocked_before_ns = int(blocked_before_sec * 1e9)
        self._reverse_duration_ns = int(reverse_duration_sec * 1e9)
        self._cooldown_ns = int(cooldown_sec * 1e9)
        self.reset()

    def reset(self) -> None:
        self._blocked_since_ns = 0
        self._reversing_until_ns = 0
        self._cooldown_until_ns = 0

    def update(self, now_ns: int, front_blocked: bool, rear_clear: bool) -> str:
        if now_ns < self._reversing_until_ns:
            return self.REVERSE
        if not front_blocked:
            self._blocked_since_ns = 0
            return self.FORWARD
        if self._blocked_since_ns == 0:
            self._blocked_since_ns = now_ns
        if (
            now_ns - self._blocked_since_ns >= self._blocked_before_ns
            and now_ns >= self._cooldown_until_ns
            and rear_clear
        ):
            self._reversing_until_ns = now_ns + self._reverse_duration_ns
            self._cooldown_until_ns = now_ns + self._cooldown_ns
            return self.REVERSE
        return self.WAITING


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values, fraction: float, default: float) -> float:
    """Return a deterministic nearest-rank percentile for finite values."""
    clean = sorted(float(value) for value in values if math.isfinite(value))
    if not clean:
        return default
    fraction = max(0.0, min(1.0, float(fraction)))
    index = int(round(fraction * (len(clean) - 1)))
    return clean[index]


def sector_percentile(
    ranges,
    angle_min: float,
    angle_increment: float,
    lidar_yaw: float,
    center: float,
    half_width: float,
    fraction: float,
    range_min: float,
    range_max: float,
) -> float:
    """Measure a percentile in a sector expressed in the vehicle base frame."""
    samples = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        lidar_angle = angle_min + index * angle_increment
        base_angle = wrap_angle(lidar_angle + lidar_yaw)
        if abs(wrap_angle(base_angle - center)) <= half_width:
            samples.append(distance)
    return percentile(samples, fraction, range_max)


def yaw_rate_from_steering(speed: float, steering: float, wheelbase: float) -> float:
    """Convert an Ackermann road-wheel angle to a ROS yaw rate."""
    if abs(speed) < 1e-9:
        return 0.0
    return speed * math.tan(steering) / wheelbase
