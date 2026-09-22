"""Observational custom-message to standard Twist contract; no command path."""

from __future__ import annotations

import math
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_MANIFEST = PACKAGE_ROOT / "config" / "GENERATED_VEHICLE_CONTRACT_MANIFEST.json"


def canonical_speed_per_erpm_mps() -> float:
    """Read the G1-generated, V004-identified measurement scale once.

    This is deliberately separate from the ESP32 command-model geometry. G2
    estimates motion from observed VESC eRPM only.
    """
    data = json.loads(CONTRACT_MANIFEST.read_text(encoding="utf-8"))
    value = float(data["derived_values"]["speed_per_erpm_mps"])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("canonical VESC eRPM scale is invalid")
    return value


def measured_speed_is_usable(telemetry_fresh: bool, measured_erpm: float, variance_mps2: float, age_sec: float, max_age_sec: float) -> bool:
    return bool(telemetry_fresh) and all(math.isfinite(value) for value in (measured_erpm, variance_mps2, age_sec, max_age_sec)) and variance_mps2 > 0.0 and 0.0 <= age_sec <= max_age_sec


def measured_erpm_to_vx(measured_erpm: float) -> float:
    if not math.isfinite(measured_erpm):
        raise ValueError("measured eRPM is not finite")
    return measured_erpm * canonical_speed_per_erpm_mps()


def standard_twist_contract() -> dict:
    return {
        "input": "/laksa/vesc/state laksa_interfaces/msg/VescState",
        "input_field": "measured_erpm",
        "freshness_field": "telemetry_fresh",
        "output": "/laksa/vehicle/speed geometry_msgs/msg/TwistWithCovarianceStamped",
        "frame_id": "base_footprint",
        "fused_variable": "vx",
        "observational_only": True,
        "command_is_measurement": False,
        "physical_variance": "PENDING_REPEATED_VESC_TELEMETRY_CHARACTERIZATION",
        "speed_scale_mps_per_erpm": canonical_speed_per_erpm_mps(),
        "speed_scale_provenance": "IDENTIFIED: laksa_physical_v004_calibrated.yaml / T22",
    }
