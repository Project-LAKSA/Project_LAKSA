from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent


def test_canonical_navigation_uses_only_official_core_nodes():
    launch = (ROOT / "launch" / "navigation_saved_map.launch.py").read_text()
    for package in (
        'package="nav2_map_server"', 'package="nav2_amcl"',
        'package="nav2_planner"', 'package="nav2_controller"',
        'package="robot_localization"',
    ):
        assert package in launch
    assert 'get_package_share_directory("zed_wrapper")' in launch
    assert "rtabmap" not in launch.lower()
    assert "custom_planner" not in launch
    assert "custom_controller" not in launch
    assert '("scan", "/laksa/lidar/scan_validated")' in launch


def test_zed_timing_override_defaults_to_navigation_only_profile():
    launch = (ROOT / "launch" / "navigation_saved_map.launch.py").read_text()
    assert 'DeclareLaunchArgument(\n            "zed_params_override_path"' in launch
    assert '"config" / "navigation_zed_timing.yaml"' in launch
    assert '"ros_params_override_path": zed_params_override_path' in launch


def test_navigation_zed_timing_keeps_source_stamps_and_removes_compute_cap():
    config = yaml.safe_load((ROOT / "config" / "navigation_zed_timing.yaml").read_text())
    parameters = config["/**"]["ros__parameters"]
    assert parameters["general"]["grab_compute_capping_fps"] == 0.0
    assert parameters.get("debug", {}).get("use_pub_timestamps", False) is False


def test_saved_map_localization_has_canonical_tf_contract():
    config = yaml.safe_load((ROOT / "config" / "saved_map_localization.yaml").read_text())
    params = config["amcl"]["ros__parameters"]
    assert params["global_frame_id"] == "map"
    assert params["odom_frame_id"] == "odom"
    assert params["base_frame_id"] == "base_footprint"
    assert config["map_server"]["ros__parameters"]["frame_id"] == "map"


def test_mode_manifest_separates_map_to_odom_authorities():
    manifest = yaml.safe_load(
        (REPO / "laksa_bringup" / "config" / "runtime_modes.yaml").read_text()
    )
    contracts = manifest["contracts"]
    assert contracts["frames"] == ["map", "odom", "base_footprint"]
    assert contracts["fused_odom_topic"] == "/laksa/odometry/fused"
    assert contracts["wheelbase_m"] == 0.324
    assert contracts["minimum_turning_radius_m"] == 1.09
    mapping = manifest["modes"]["LAKSA_MAPPING_MODE"]
    navigation = manifest["modes"]["LAKSA_NAVIGATION_MODE"]
    assert mapping["map_to_odom_authority"].endswith("/rtabmap")
    assert navigation["map_to_odom_authority"] == "/amcl"
    assert "/amcl" in mapping["forbidden_nodes"]
    assert "/laksa/fused_mapping/rtabmap" in navigation["forbidden_nodes"]


def test_active_navigation_contract_has_no_legacy_frames_or_radius():
    paths = [
        ROOT / "config" / "saved_map_localization.yaml",
        ROOT / "launch" / "navigation_saved_map.launch.py",
        REPO / "laksa_bringup" / "config" / "runtime_modes.yaml",
        REPO / "laksa_bringup" / "config" / "nav2_ackermann.yaml",
    ]
    text = "\n".join(path.read_text() for path in paths)
    assert not re.search(r"\blaksa_odom\b|\blaksa_base_footprint\b", text)
    assert not re.search(r"/laksa/odom\b", text)
    assert "minimum_turning_radius: 0.90" not in text
    assert "min_turning_r: 0.90" not in text
    assert "minimum_turning_radius: 1.09" in text
    assert "min_turning_r: 1.09" in text


def test_snapshot_and_followpath_fail_closed_contract():
    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text()
    assert 'app.router.add_post("/api/snapshot/take"' in server
    assert 'mapping.get("state") not in ("MAPPING", "DEGRADED")' in server
    assert 'navigation.get("state") not in ("READY", "LIVE_MAPPING_READY")' in server
    assert 'navigation.get("localization_valid") is not True' in server
    assert 'body.get("confirmed") is not True' in server
    assert 'exact_path = self._planned_path' in server
    assert 'action_goal.path = exact_path' in server
    assert 'ActionClient(self, FollowPath, "/follow_path")' in server


def test_snapshot_pose_continuity_is_measured_and_gates_ready():
    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text()
    assert "def _pose_continuity" in server
    assert '"within_target": position_delta <= 0.05 and yaw_delta_deg <= 2.0' in server
    assert '"hard_failure": position_delta > 0.15 or yaw_delta_deg > 5.0' in server
    assert "continuity_passes >= 3" in server
    assert "localized = bool(self._amcl_pose_received)" in server
    assert "canonical_pose_valid = self._canonical_pose_rejection_reason() is None" in server
    assert 'return "AMCL localization initialization is unavailable"' in server
    assert "time.monotonic() - self._amcl_pose_received > self._pose_timeout" not in server
    assert "message.pose.covariance[0] = 0.05 ** 2" in server
    assert "Non-canonical laksa-planning-preview.service is active" in server
