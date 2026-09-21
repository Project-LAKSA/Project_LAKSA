#!/usr/bin/env python3
"""Compile the single LAKSA V2 vehicle contract into deterministic artifacts.

The source carries a `.yaml` name but deliberately uses JSON syntax: JSON is a
valid YAML subset and the stdlib parser keeps this safety-critical compiler
dependency-free and reproducible on CI and the Jetson.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


GENERATOR_VERSION = "1.0.0"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
CONTRACT_PATH = CONFIG_DIR / "vehicle_contract.yaml"
SCHEMA_PATH = CONFIG_DIR / "vehicle_contract.schema.json"
MANIFEST_PATH = CONFIG_DIR / "GENERATED_VEHICLE_CONTRACT_MANIFEST.json"
AUDIT_PATH = PACKAGE_ROOT / "VEHICLE_GEOMETRY_SOURCE_AUDIT.json"
GENERATED_DIR = CONFIG_DIR / "generated"
URDF_DIR = PACKAGE_ROOT / "urdf"
REPOSITORY_ROOT = PACKAGE_ROOT.parents[3]

PROVENANCE_CLASSES = {"MEASURED", "IDENTIFIED", "DERIVED", "ESTIMATED", "UNKNOWN"}
LITERALS = ("0.324", "0.90", "1.09", "0.523", "0.288", "0.419", "0.149", "0.148", "0.17165", "0.0545")
EXCLUDED_AUDIT_PARTS = {".git", "node_modules", "build", "install", "__pycache__"}


class ContractError(ValueError):
    """A malformed or unsafe canonical contract."""


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _finite_positive(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ContractError(f"{name} must be a finite positive number")
    return float(value)


def _quantity(contract: dict[str, Any], *path: str, units: str | None = None) -> float:
    cursor: Any = contract
    for key in path:
        cursor = cursor[key]
    if not isinstance(cursor, dict) or "value" not in cursor:
        raise ContractError(f"{'.'.join(path)} must be a quantity")
    if units and cursor.get("units") != units:
        raise ContractError(f"{'.'.join(path)} units must be {units}")
    provenance = cursor.get("provenance")
    if not isinstance(provenance, dict) or provenance.get("classification") not in PROVENANCE_CLASSES:
        raise ContractError(f"{'.'.join(path)} has invalid provenance")
    return _finite_positive(cursor["value"], ".".join(path))


def load_contract(path: Path = CONTRACT_PATH) -> dict[str, Any]:
    try:
        contract = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot parse canonical YAML/JSON contract: {exc}") from exc
    validate_contract(contract)
    return contract


def validate_contract(contract: dict[str, Any]) -> None:
    if not isinstance(contract, dict) or contract.get("contract_version") != 2:
        raise ContractError("contract_version must be 2")
    identity = contract.get("identity")
    if not isinstance(identity, dict) or identity.get("body_frame") != "base_link" or identity.get("navigation_frame") != "base_footprint":
        raise ContractError("identity must define base_link body and base_footprint navigation frames")
    wheelbase = _quantity(contract, "geometry", "wheelbase_m", units="m")
    _quantity(contract, "geometry", "track_width_m", units="m")
    _quantity(contract, "geometry", "wheel_radius_m", units="m")
    _quantity(contract, "geometry", "footprint_padding_m", units="m")
    for side in ("front", "rear", "left", "right"):
        _quantity(contract, "geometry", "footprint_m", side, units="m")
    for side in ("left_limit_rad", "right_limit_rad"):
        steering = contract["steering"].get(side)
        if not isinstance(steering, dict) or steering.get("semantics") != "EQUIVALENT_BICYCLE_STEERING_ANGLE":
            raise ContractError(f"steering.{side} must be an equivalent bicycle steering angle")
        angle = _quantity(contract, "steering", side, units="rad")
        if not 0.0 < angle < math.pi / 2:
            raise ContractError(f"steering.{side} must be within (0, pi/2)")
    if wheelbase <= 0 or contract["steering"].get("actuator_interface") != "ASYMMETRIC_SERVO_TRAVEL_NORMALIZED_FROM_EQUIVALENT_BICYCLE_ANGLE":
        raise ContractError("steering actuator contract is incomplete")
    if contract["kinematics"].get("planner_radius_selection_policy") != "MAX_OF_DIRECTIONAL_GEOMETRIC_RADII_UNTIL_EFFECTIVE_RADII_ARE_MEASURED":
        raise ContractError("planner must select the least-maneuverable directional geometric radius")
    for side in ("left", "right"):
        effective = contract["kinematics"].get(f"measured_effective_radius_{side}_m")
        if not isinstance(effective, dict) or effective.get("value") is not None or effective.get("provenance") != "EFFECTIVE_RADIUS_NEEDS_FUTURE_PHYSICAL_MEASUREMENT":
            raise ContractError("effective radii must remain explicitly pending physical measurement")
    sensors = contract.get("sensors", {})
    for sensor in ("zed", "lidar"):
        item = sensors.get(sensor, {})
        if not (isinstance(item.get("xyz_m"), list) and len(item["xyz_m"]) == 3 and isinstance(item.get("rpy_rad"), list) and len(item["rpy_rad"]) == 3):
            raise ContractError(f"{sensor} extrinsic is incomplete")
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in item["xyz_m"] + item["rpy_rad"]):
            raise ContractError(f"{sensor} extrinsic contains a non-finite value")
    if sensors["zed"]["rpy_rad"][1] <= 0.0:
        raise ContractError("ZED measured mount pitch must retain its positive recovered sign")
    imu = sensors.get("imu", {})
    if imu.get("xyz_m") is not None or imu.get("rpy_rad") is not None or imu.get("provenance", {}).get("classification") != "UNKNOWN":
        raise ContractError("IMU extrinsic may not be invented before measurement")
    if contract.get("legacy_assumptions", {}).get("minimum_turning_radius_m", {}).get("classification") != "LEGACY_ASSUMPTION_NOT_ACTIVE_V2":
        raise ContractError("legacy 0.90 m radius must be explicitly non-active")


def derived_values(contract: dict[str, Any]) -> dict[str, Any]:
    wheelbase = _quantity(contract, "geometry", "wheelbase_m", units="m")
    left = _quantity(contract, "steering", "left_limit_rad", units="rad")
    right = _quantity(contract, "steering", "right_limit_rad", units="rad")
    radius_left = wheelbase / math.tan(abs(left))
    radius_right = wheelbase / math.tan(abs(right))
    footprint = contract["geometry"]["footprint_m"]
    padding = _quantity(contract, "geometry", "footprint_padding_m", units="m")
    front, rear = float(footprint["front"]["value"]), float(footprint["rear"]["value"])
    left_extent, right_extent = float(footprint["left"]["value"]), float(footprint["right"]["value"])
    canonical = [[front, left_extent], [front, -right_extent], [-rear, -right_extent], [-rear, left_extent]]
    padded = [[front + padding, left_extent + padding], [front + padding, -(right_extent + padding)], [-(rear + padding), -(right_extent + padding)], [-(rear + padding), left_extent + padding]]
    return {
        "wheelbase_m": wheelbase,
        "track_width_m": _quantity(contract, "geometry", "track_width_m", units="m"),
        "wheel_radius_m": _quantity(contract, "geometry", "wheel_radius_m", units="m"),
        "steering_left_rad": left,
        "steering_right_rad": right,
        "geometric_radius_left_m": radius_left,
        "geometric_radius_right_m": radius_right,
        "planner_min_turning_radius_m": max(radius_left, radius_right),
        "planner_radius_selection_policy": contract["kinematics"]["planner_radius_selection_policy"],
        "canonical_footprint": canonical,
        "padded_footprint": padded,
        "footprint_padding_m": padding,
        "zed": contract["sensors"]["zed"],
        "lidar": contract["sensors"]["lidar"],
        "mass_kg": _quantity(contract, "twin_v004", "mass_kg", units="kg"),
        "speed_per_erpm_mps": _quantity(contract, "twin_v004", "speed_per_erpm_mps", units="m/s/eRPM"),
    }


def _xacro_properties(values: dict[str, Any]) -> bytes:
    zed, lidar = values["zed"], values["lidar"]
    fields = {
        "wheelbase_m": values["wheelbase_m"], "track_width_m": values["track_width_m"], "wheel_radius_m": values["wheel_radius_m"],
        "zed_x_m": zed["xyz_m"][0], "zed_y_m": zed["xyz_m"][1], "zed_z_m": zed["xyz_m"][2],
        "zed_roll_rad": zed["rpy_rad"][0], "zed_pitch_rad": zed["rpy_rad"][1], "zed_yaw_rad": zed["rpy_rad"][2],
        "lidar_x_m": lidar["xyz_m"][0], "lidar_y_m": lidar["xyz_m"][1], "lidar_z_m": lidar["xyz_m"][2],
        "lidar_roll_rad": lidar["rpy_rad"][0], "lidar_pitch_rad": lidar["rpy_rad"][1], "lidar_yaw_rad": lidar["rpy_rad"][2],
    }
    lines = ["<?xml version=\"1.0\"?>", "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"laksa_v2_geometry\">"]
    lines.extend(f"  <xacro:property name=\"{name}\" value=\"{value:.12g}\"/>" for name, value in fields.items())
    lines.append("</robot>")
    return ("\n".join(lines) + "\n").encode("utf-8")


def _tf_contract() -> dict[str, Any]:
    return {
        "contract_version": 2,
        "root": "map",
        "base_frame_architecture": "OPTION_B_ODOM_TO_BASE_FOOTPRINT_PLANAR__BASE_FOOTPRINT_TO_BASE_LINK_DYNAMIC_BODY_ATTITUDE",
        "mode_specific_global_authority": {
            "mapping": "MAPPING_SYSTEM",
            "navigation": "LOCALIZATION_SYSTEM",
            "three_d_reconstruction": "OBSERVATIONAL_NO_NAV2_MAP_TO_ODOM_AUTHORITY"
        },
        "edges": [
            {"parent": "map", "child": "odom", "authority": "GLOBAL_LOCALIZATION_MODE_SELECTOR", "type": "dynamic", "failure": "autonomy_disarm"},
            {"parent": "odom", "child": "base_footprint", "authority": "LOCAL_STATE_ESTIMATOR", "type": "dynamic_planar", "failure": "autonomy_disarm"},
            {"parent": "base_footprint", "child": "base_link", "authority": "BODY_ATTITUDE_PROJECTION_ADAPTER", "type": "dynamic_derived", "failure": "autonomy_disarm"},
            {"parent": "base_link", "child": "chassis_link", "authority": "robot_state_publisher", "type": "static", "failure": "none"},
            {"parent": "base_link", "child": "rear_axle_link", "authority": "robot_state_publisher", "type": "static", "failure": "none"},
            {"parent": "rear_axle_link", "child": "front_axle_link", "authority": "robot_state_publisher", "type": "static", "failure": "none"},
            {"parent": "base_link", "child": "zed_camera_link", "authority": "robot_state_publisher", "type": "static", "failure": "sensor_unavailable"},
            {"parent": "base_link", "child": "lidar_link", "authority": "robot_state_publisher", "type": "static", "failure": "sensor_unavailable"}
        ],
        "unmeasured_edges": ["base_link->imu_link"],
        "forbidden_authorities": ["ZED_ODOLOGY_TF", "RTAB_MAP_TF_IN_NAVIGATION_MODE", "MAPPING_LAUNCH_STATIC_SENSOR_TF"]
    }


def _artifact_payloads(contract: dict[str, Any], values: dict[str, Any]) -> dict[Path, bytes]:
    geometric = {
        "contract_version": 2,
        "directional_geometric_radius_m": {"left": values["geometric_radius_left_m"], "right": values["geometric_radius_right_m"]},
        "planner_min_turning_radius_m": values["planner_min_turning_radius_m"],
        "planner_radius_selection_policy": values["planner_radius_selection_policy"],
        "formulas": {"left": "wheelbase_m / tan(abs(left_limit_rad))", "right": "wheelbase_m / tan(abs(right_limit_rad))"}
    }
    return {
        GENERATED_DIR / "nav2_footprints.json": _json_bytes({"frame": "base_footprint", "canonical_polygon_m": values["canonical_footprint"], "padded_polygon_m": values["padded_footprint"], "padding_m": values["footprint_padding_m"]}),
        GENERATED_DIR / "kinematics.json": _json_bytes(geometric),
        GENERATED_DIR / "state_lattice_geometry.json": _json_bytes({"wheelbase_m": values["wheelbase_m"], "minimum_turning_radius_m": values["planner_min_turning_radius_m"], "selection_policy": values["planner_radius_selection_policy"]}),
        GENERATED_DIR / "mppi_ackermann_geometry.json": _json_bytes({"min_turning_r": values["planner_min_turning_radius_m"], "selection_policy": values["planner_radius_selection_policy"]}),
        GENERATED_DIR / "ros2_control_geometry.json": _json_bytes({"wheelbase_m": values["wheelbase_m"], "track_width_m": values["track_width_m"], "wheel_radius_m": values["wheel_radius_m"], "left_limit_rad": values["steering_left_rad"], "right_limit_rad": values["steering_right_rad"]}),
        GENERATED_DIR / "gazebo_v004_geometry.json": _json_bytes({"wheelbase_m": values["wheelbase_m"], "track_width_m": values["track_width_m"], "wheel_radius_m": values["wheel_radius_m"], "mass_kg": values["mass_kg"], "speed_per_erpm_mps": values["speed_per_erpm_mps"], "cg": "UNKNOWN_HARDWARE_MEASUREMENT_REQUIRED"}),
        CONFIG_DIR / "tf_authority_contract.json": _json_bytes(_tf_contract()),
        URDF_DIR / "laksa_v2_geometry.generated.xacro": _xacro_properties(values),
    }


def _literal_semantics(literal: str) -> str:
    return {
        "0.324": "wheelbase_m", "0.90": "legacy planner radius assumption", "1.09": "legacy conservative planner radius", "0.523": "legacy left equivalent bicycle steering limit", "0.288": "legacy right equivalent bicycle steering limit", "0.419": "front footprint extent", "0.149": "rear footprint extent", "0.148": "lateral footprint extent", "0.17165": "track_width_m", "0.0545": "wheel_radius_m"
    }[literal]


def geometry_source_audit() -> dict[str, Any]:
    records = []
    pattern = re.compile(r"(?<![0-9])(" + "|".join(re.escape(value) for value in LITERALS) + r")(?![0-9])")
    for path in sorted(REPOSITORY_ROOT.rglob("*")):
        if not path.is_file() or any(part in EXCLUDED_AUDIT_PARTS for part in path.parts):
            continue
        if path.resolve() in {AUDIT_PATH.resolve(), MANIFEST_PATH.resolve()}:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        relative = path.relative_to(REPOSITORY_ROOT).as_posix()
        for line_number, line in enumerate(text.splitlines(), start=1):
            for match in pattern.finditer(line):
                literal = match.group(1)
                is_v2 = relative.startswith("firmware/esp32-s3/jetson/laksa_navigation_v2/")
                generated = "/config/generated/" in relative or relative.endswith("tf_authority_contract.json") or relative.endswith("laksa_v2_geometry.generated.xacro")
                test_only = "/test/" in relative
                tooling = relative.endswith("laksa_navigation_v2/generate_contract_artifacts.py")
                legacy_nonactive = "LEGACY_ASSUMPTION_NOT_ACTIVE_V2" in line
                records.append({
                    "parameter": _literal_semantics(literal), "value": float(literal), "units": "m" if literal in {"0.324", "0.90", "1.09", "0.419", "0.149", "0.148", "0.17165", "0.0545"} else "rad",
                    "file": relative, "line": line_number, "context": line.strip()[:240], "semantic_meaning": _literal_semantics(literal),
                    "provenance": "CONTRACT_GENERATED" if generated else ("V2_CONTRACT" if relative.endswith("vehicle_contract.yaml") else "HISTORICAL_OR_LEGACY_SOURCE"),
                    "v2_active": is_v2 and not test_only and not tooling and not legacy_nonactive, "disposition": "GENERATED" if generated else ("CANONICAL_SOURCE" if relative.endswith("vehicle_contract.yaml") else ("ASSERTED_TEST_ONLY" if test_only else ("CONFIGURATION_TOOLING" if tooling else "RETAIN_HISTORICAL_EVIDENCE_OR_REWIRE_LATER")))
                })
    return {"audit_version": 1, "scope": "all tracked-readable repository sources; vendor/build/install excluded", "records": records}


def active_v2_geometry_divergences(audit: dict[str, Any]) -> list[dict[str, Any]]:
    """Return non-generated active V2 geometry copies outside the contract."""
    allowed = {"CANONICAL_SOURCE", "GENERATED"}
    return [record for record in audit["records"] if record["v2_active"] and record["disposition"] not in allowed]


def validate_generated_geometry_inputs(values: dict[str, Any], artifacts: dict[str, dict[str, Any]]) -> list[str]:
    """Verify future consumer inputs remain facts derived from this contract."""
    errors: list[str] = []
    for name in ("state_lattice", "gazebo", "ros2_control"):
        if not math.isclose(float(artifacts[name]["wheelbase_m"]), values["wheelbase_m"], rel_tol=0.0, abs_tol=1e-12):
            errors.append(f"inconsistent_{name}_wheelbase")
    if not math.isclose(float(artifacts["mppi"]["min_turning_r"]), values["planner_min_turning_radius_m"], rel_tol=0.0, abs_tol=1e-12):
        errors.append("inconsistent_mppi_radius")
    if artifacts["nav2"]["canonical_polygon_m"] != values["canonical_footprint"]:
        errors.append("duplicated_or_inconsistent_nav2_footprint")
    return errors


def checked_in_geometry_inputs() -> dict[str, dict[str, Any]]:
    return {
        "nav2": json.loads((GENERATED_DIR / "nav2_footprints.json").read_text()),
        "state_lattice": json.loads((GENERATED_DIR / "state_lattice_geometry.json").read_text()),
        "mppi": json.loads((GENERATED_DIR / "mppi_ackermann_geometry.json").read_text()),
        "ros2_control": json.loads((GENERATED_DIR / "ros2_control_geometry.json").read_text()),
        "gazebo": json.loads((GENERATED_DIR / "gazebo_v004_geometry.json").read_text()),
    }


def generate(output_root: Path = PACKAGE_ROOT) -> dict[str, Any]:
    if output_root.resolve() != PACKAGE_ROOT.resolve():
        raise ContractError("generation only supports the checked-in V2 package root")
    contract_bytes = CONTRACT_PATH.read_bytes()
    contract = load_contract()
    values = derived_values(contract)
    payloads = _artifact_payloads(contract, values)
    for path, payload in payloads.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    audit = geometry_source_audit()
    AUDIT_PATH.write_bytes(_json_bytes(audit))
    output_hashes = {path.relative_to(PACKAGE_ROOT).as_posix(): _sha256_bytes(path.read_bytes()) for path in sorted(payloads)}
    output_hashes[AUDIT_PATH.relative_to(PACKAGE_ROOT).as_posix()] = _sha256_bytes(AUDIT_PATH.read_bytes())
    manifest = {
        "generator_version": GENERATOR_VERSION,
        "source_contract": "config/vehicle_contract.yaml",
        "source_contract_sha256": _sha256_bytes(contract_bytes),
        "schema": "config/vehicle_contract.schema.json",
        "generated_artifact_paths": sorted(output_hashes),
        "output_sha256": output_hashes,
        "derived_values": values,
        "derivation_formulas": {"radius": "wheelbase_m / tan(abs(equivalent_bicycle_steering_angle_rad))", "planner_radius": "max(geometric_radius_left_m, geometric_radius_right_m)", "padded_footprint": "axis-aligned footprint extents expanded by footprint_padding_m"},
        "provenance_references": {"steering": contract["steering"], "sensors": contract["sensors"], "legacy_radius": contract["legacy_assumptions"]}
    }
    MANIFEST_PATH.write_bytes(_json_bytes(manifest))
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail when checked-in generated artifacts differ")
    args = parser.parse_args(argv)
    expected_paths = [MANIFEST_PATH, AUDIT_PATH, CONFIG_DIR / "tf_authority_contract.json", URDF_DIR / "laksa_v2_geometry.generated.xacro", *sorted(GENERATED_DIR.glob("*.json"))]
    before = {path: path.read_bytes() if path.exists() else None for path in expected_paths}
    manifest = generate()
    if args.check:
        changed = [path for path, data in before.items() if path.read_bytes() != data]
        if changed:
            raise ContractError("generated artifacts were stale: " + ", ".join(str(path) for path in changed))
    print(json.dumps({"generator_version": GENERATOR_VERSION, "planner_min_turning_radius_m": manifest["derived_values"]["planner_min_turning_radius_m"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
