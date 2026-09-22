"""Static G2 contracts around the official robot_localization EKF."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
EKF_CONFIG = CONFIG_DIR / "ekf_local_odom.yaml"
SYNTHETIC_EKF_CONFIG = CONFIG_DIR / "ekf_local_odom_synthetic.yaml"
VY_EXPERIMENT_EKF_CONFIG = CONFIG_DIR / "ekf_local_odom_synthetic_vy_constraint.yaml"
SPEED_ENABLED_EKF_CONFIG = CONFIG_DIR / "ekf_local_odom_speed_enabled.yaml"
ZED_OVERLAY = CONFIG_DIR / "zed_local_vio_overlay.yaml"
G2_GRAPH = CONFIG_DIR / "g2_launch_graph_contract.json"

STATE_VECTOR = ("x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "vroll", "vpitch", "vyaw", "ax", "ay", "az")


class G2ContractError(ValueError):
    pass


def load_json_yaml(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise G2ContractError(f"invalid JSON-compatible YAML {path}: {exc}") from exc


def _parameters(path: Path) -> dict[str, Any]:
    config = load_json_yaml(path)
    try:
        return config["ekf_local_odom"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise G2ContractError("EKF configuration lacks ekf_local_odom.ros__parameters") from exc


def validate_ekf_config(path: Path = EKF_CONFIG, synthetic: bool = False, speed_enabled: bool = False) -> list[str]:
    params = _parameters(path)
    errors: list[str] = []
    expected = {
        "world_frame": "odom", "odom_frame": "odom", "base_link_frame": "base_footprint",
        "base_link_output_frame": "base_footprint", "two_d_mode": True, "publish_tf": True,
        "publish_acceleration": False, "print_diagnostics": True, "debug": False,
        "use_control": False, "odom0": "/laksa/vio/base_odom", "odom0_relative": False,
        "odom0_differential": False, "reset_on_time_jump": True, "smooth_lagged_data": False,
        "permit_corrected_publication": False,
    }
    for key, value in expected.items():
        if params.get(key) != value:
            errors.append(f"wrong_{key}")
    if not isinstance(params.get("frequency"), (int, float)) or params["frequency"] <= 0:
        errors.append("invalid_frequency")
    if not isinstance(params.get("sensor_timeout"), (int, float)) or params["sensor_timeout"] < 0.5:
        errors.append("invalid_sensor_timeout")
    expected_vectors = [("odom0_config", {0, 1, 5})]
    if synthetic or speed_enabled:
        expected_vectors.append(("twist0_config", {6}))
    for key, selected in expected_vectors:
        vector = params.get(key)
        if not isinstance(vector, list) or len(vector) != len(STATE_VECTOR):
            errors.append(f"invalid_{key}")
        elif {index for index, enabled in enumerate(vector) if enabled} != selected:
            errors.append(f"wrong_{key}_selection")
    if any("imu" in key.lower() for key in params):
        errors.append("raw_imu_double_fusion")
    if not synthetic and "twist1" in params:
        errors.append("test_only_vy_constraint_leaked_to_production_intent")
    if not synthetic and not speed_enabled and "twist0" in params:
        errors.append("uncalibrated_vehicle_speed_fusion_enabled")
    if speed_enabled and params.get("twist0") != "/laksa/vehicle/speed":
        errors.append("wrong_twist0")
    if synthetic:
        for key in ("odom0_pose_rejection_threshold", "twist0_rejection_threshold"):
            if not isinstance(params.get(key), (int, float)) or params[key] <= 0:
                errors.append(f"missing_{key}")
    elif "odom0_pose_rejection_threshold" in params or "twist0_rejection_threshold" in params:
        errors.append("synthetic_threshold_leaked_to_production_intent")
    return errors


def validate_zed_vio_contract() -> list[str]:
    overlay = load_json_yaml(ZED_OVERLAY)
    params = overlay.get("wrapper_parameter_contract", {})
    expected = {
        "pos_tracking.pos_tracking_enabled": True,
        "pos_tracking.pos_tracking_mode": "GEN_3",
        "pos_tracking.imu_fusion": True,
        "pos_tracking.area_memory": False,
        "pos_tracking.two_d_mode": False,
        "pos_tracking.reset_odom_with_loop_closure": False,
        "pos_tracking.publish_odom_pose": True,
        "publish_tf": False,
        "publish_map_tf": False,
    }
    errors = [f"wrong_zed_{key}" for key, value in expected.items() if params.get(key) != value]
    if overlay.get("role") != "RAW_VIO_MEASUREMENT_SOURCE_ONLY":
        errors.append("wrong_zed_role")
    raw = overlay.get("raw_odometry_contract", {})
    if overlay.get("base_measurement_topic") != "/laksa/vio/base_odom" or overlay.get("native_output_topic") != "/zed/zed_node/odom":
        errors.append("wrong_zed_topic_contract")
    if raw.get("header_frame_id") != "odom" or raw.get("child_frame_id") != "zed_camera_link":
        errors.append("wrong_zed_frame_contract")
    return errors


def validate_vy_experiment_config() -> list[str]:
    """The A/B pseudo-constraint is confined to the explicit synthetic config."""
    params = _parameters(VY_EXPERIMENT_EKF_CONFIG)
    errors = validate_ekf_config(VY_EXPERIMENT_EKF_CONFIG, synthetic=True)
    if params.get("twist1") != "/laksa/test_only/nonholonomic_vy":
        errors.append("missing_test_only_vy_topic")
    vector = params.get("twist1_config")
    if not isinstance(vector, list) or {index for index, enabled in enumerate(vector) if enabled} != {7}:
        errors.append("wrong_test_only_vy_selection")
    if params.get("two_d_mode") is not True:
        errors.append("vy_experiment_not_planar")
    return errors


def validate_g2_launch_graph() -> list[str]:
    graph = load_json_yaml(G2_GRAPH)
    errors: list[str] = []
    if graph.get("test_only") is not True:
        errors.append("g2_not_test_only")
    publishers = set(graph.get("publishers", []))
    forbidden = set(graph.get("forbidden_publishers", []))
    if publishers & forbidden:
        errors.append("actuator_publisher_present")
    if not {"/laksa/vio/odom", "/laksa/vehicle/speed", "/laksa/odometry/local", "/tf"}.issubset(publishers):
        errors.append("missing_required_g2_topics")
    if graph.get("global_tf") != "ABSENT_IN_PRODUCTION_INTENT_G2":
        errors.append("map_to_odom_present")
    return errors


def covariance_is_valid(covariance: list[float], expected_variance: float) -> bool:
    if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
        return False
    if covariance[0] <= 0 or not math.isclose(covariance[0], expected_variance, rel_tol=1e-12):
        return False
    for row in range(6):
        if covariance[row * 6 + row] < 0:
            return False
        for column in range(6):
            if not math.isclose(covariance[row * 6 + column], covariance[column * 6 + row], rel_tol=0.0, abs_tol=1e-15):
                return False
    # Gershgorin is a deterministic sufficient check for the diagonal matrices
    # emitted by G2; it rejects negative/obviously non-PSD matrices without a
    # numerical dependency in the package runtime.
    return all(covariance[row * 6 + row] + 1e-15 >= sum(abs(covariance[row * 6 + column]) for column in range(6) if column != row) for row in range(6))


def allowed_synthetic_differences() -> set[str]:
    return {
        "odom0_pose_rejection_threshold", "twist0_rejection_threshold",
        "twist0", "twist0_config", "twist0_queue_size", "twist0_nodelay",
    }
