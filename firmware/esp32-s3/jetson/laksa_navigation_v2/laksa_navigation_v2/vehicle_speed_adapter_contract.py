"""Observational custom-message to standard Twist contract; no command path."""

from __future__ import annotations

import math


def measured_speed_is_usable(telemetry_fresh: bool, vehicle_linear_velocity_mps: float, variance_mps2: float) -> bool:
    return bool(telemetry_fresh) and all(math.isfinite(value) for value in (vehicle_linear_velocity_mps, variance_mps2)) and variance_mps2 > 0.0


def standard_twist_contract() -> dict:
    return {
        "input": "/laksa/vesc/state laksa_interfaces/msg/VescState",
        "input_field": "vehicle_linear_velocity_mps",
        "freshness_field": "telemetry_fresh",
        "output": "/laksa/vehicle/speed geometry_msgs/msg/TwistWithCovarianceStamped",
        "frame_id": "base_footprint",
        "fused_variable": "vx",
        "observational_only": True,
        "command_is_measurement": False,
        "physical_variance": "PENDING_REPEATED_VESC_TELEMETRY_CHARACTERIZATION",
    }
