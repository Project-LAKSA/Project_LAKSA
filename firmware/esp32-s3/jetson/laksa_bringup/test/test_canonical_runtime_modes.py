from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[2]


def _systemd_source() -> Path:
    candidates = [ROOT / "systemd"]
    candidates.extend(
        parent / "firmware" / "esp32-s3" / "jetson" / "systemd"
        for parent in Path(__file__).resolve().parents
    )
    candidates.extend(
        parent / "src" / "Project_LAKSA" / "firmware" / "esp32-s3" / "jetson" / "systemd"
        for parent in Path(__file__).resolve().parents
    )
    return next(path for path in candidates if path.is_dir())


SYSTEMD = _systemd_source()


def test_canonical_ros_services_require_one_runtime_environment():
    services = (
        "laksa-control-navigation.service",
        "laksa-lidar.service",
        "laksa-lidar-mapping.service",
        "laksa-mapping-cockpit.service",
        "laksa-planning-preview.service",
        "laksa-zed-camera.service",
    )
    for name in services:
        text = (SYSTEMD / name).read_text(encoding="utf-8")
        assert "EnvironmentFile=/etc/laksa/ros-runtime.env" in text, name
    environment = (SYSTEMD / "laksa-ros-runtime.env").read_text(encoding="utf-8")
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in environment
    assert "CYCLONEDDS_URI=/etc/laksa/cyclonedds.xml" in environment


def test_mode_manifest_has_one_tf_authority_per_mode_and_no_custom_core():
    manifest = yaml.safe_load(
        (ROOT / "laksa_bringup" / "config" / "runtime_modes.yaml").read_text(encoding="utf-8")
    )
    modes = manifest["modes"]
    assert set(modes) == {"LAKSA_MAPPING_MODE", "LAKSA_NAVIGATION_MODE"}
    assert modes["LAKSA_MAPPING_MODE"]["map_to_odom_authority"] == "/laksa/fused_mapping/rtabmap"
    assert modes["LAKSA_NAVIGATION_MODE"]["map_to_odom_authority"] == "/amcl"
    for mode in modes.values():
        for node in mode["expected_nodes"]:
            assert node["classification"] != "CUSTOM_CORE_ALGORITHM"


def test_active_contract_forbids_legacy_frames_topics_and_geometry():
    active = [
        ROOT / "laksa_bringup" / "config" / "nav2_ackermann.yaml",
        ROOT / "laksa_bringup" / "config" / "runtime_modes.yaml",
        ROOT / "laksa_dashboard" / "config" / "planning_geometry.yaml",
        ROOT / "laksa_dashboard" / "config" / "saved_map_localization.yaml",
        ROOT / "laksa_dashboard" / "launch" / "navigation_saved_map.launch.py",
    ]
    text = "\n".join(path.read_text(encoding="utf-8") for path in active)
    assert not re.search(r"\blaksa_odom\b|\blaksa_base_footprint\b", text)
    assert not re.search(r"/laksa/odom\b", text)
    assert not re.search(
        r"(?:minimum_turning_radius|min_turning_r)\s*:\s*0\.90(?:0*)?\b", text
    )
    assert "/laksa/odometry/fused" in text
