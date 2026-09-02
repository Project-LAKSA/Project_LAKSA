#!/usr/bin/env python3

"""Pure Ackermann command math shared by runtime code and unit tests."""

import math


def map_geometry_is_sane(
    width: int,
    height: int,
    resolution: float,
    *,
    maximum_cells: int = 4_000_000,
    maximum_span_m: float = 60.0,
) -> bool:
    """Reject corrupt SLAM grids before they overload navigation or the UI."""
    if width <= 0 or height <= 0 or not math.isfinite(resolution):
        return False
    if resolution <= 0.0 or width * height > maximum_cells:
        return False
    return (
        width * resolution <= maximum_span_m
        and height * resolution <= maximum_span_m
    )


def limited_ackermann_command(
    raw_speed: float,
    raw_yaw_rate: float,
    speed_limit: float,
    wheelbase: float,
    left_wheel_limit: float,
    right_wheel_limit: float,
    servo_reported_limit: float,
) -> tuple[float, float]:
    """Limit speed without changing the curvature requested by Nav2.

    Returns the limited linear speed and the normalized steering convention
    consumed by the ESP32. Positive steering follows REP-103 (left).
    """
    values = (
        raw_speed,
        raw_yaw_rate,
        speed_limit,
        wheelbase,
        left_wheel_limit,
        right_wheel_limit,
        servo_reported_limit,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Ackermann command inputs must be finite")
    if min(
        speed_limit,
        wheelbase,
        left_wheel_limit,
        right_wheel_limit,
        servo_reported_limit,
    ) <= 0.0:
        raise ValueError("Ackermann geometry and speed limit must be positive")

    limited_speed = max(-speed_limit, min(speed_limit, raw_speed))
    if abs(raw_speed) <= 0.01:
        return limited_speed, 0.0

    # Twist encodes curvature as angular.z / linear.x. Compute it before
    # limiting linear speed; retaining the original angular.z after clipping
    # would command a tighter turn than the trajectory evaluated by MPPI.
    curvature = raw_yaw_rate / raw_speed
    road_wheel_steering = math.atan(wheelbase * curvature)
    physical_limit = (
        left_wheel_limit if road_wheel_steering >= 0.0 else right_wheel_limit
    )
    road_wheel_steering = max(
        -physical_limit, min(physical_limit, road_wheel_steering)
    )
    normalized_steering = (
        road_wheel_steering * servo_reported_limit / physical_limit
    )
    return limited_speed, normalized_steering
