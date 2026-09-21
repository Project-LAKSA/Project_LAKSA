#!/usr/bin/env python3

"""Pure Ackermann command math shared by runtime code and unit tests."""

import math


def canonical_map_invalid_reason(
    width: int,
    height: int,
    resolution: float,
    data_length: int,
    frame_id: str,
    expected_frame: str,
) -> str | None:
    """Validate persistent OccupancyGrid content without using message age."""
    if width <= 0 or height <= 0:
        return "MAP_INVALID_DIMENSIONS"
    if not math.isfinite(resolution) or resolution <= 0.0:
        return "MAP_INVALID_RESOLUTION"
    if data_length != width * height:
        return "MAP_INVALID_DATA"
    if frame_id != expected_frame:
        return "MAP_FRAME_MISMATCH"
    if not map_geometry_is_sane(width, height, resolution):
        return "MAP_INVALID_GEOMETRY"
    return None


def route_data_preflight_reason(
    now_ns: int,
    map_received: bool,
    map_invalid_reason: str | None,
    dynamic_checks,
) -> str | None:
    """Return the first route-data failure; a valid map has no age timeout."""
    if not map_received:
        return "MAP_NOT_RECEIVED"
    if map_invalid_reason:
        return map_invalid_reason
    for stamp_ns, timeout_ns, failure in dynamic_checks:
        if stamp_ns == 0 or now_ns - stamp_ns > timeout_ns:
            return failure
    return None


def tf_preflight_reason(error: str | None) -> str | None:
    return None if not error else f"TF_UNAVAILABLE: {error}"


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
