import unittest
from pathlib import Path

from laksa_mapping.fused_policy import FUSED_MAPPING
from laksa_mapping.shadow_policy import HYBRID_SHADOW, ZED_ONLY, hybrid_ready, output_prefix


ROOT = Path(__file__).resolve().parents[1]


class ShadowMappingTest(unittest.TestCase):
    def test_production_default_is_fused_and_shadow_namespaces_are_isolated(self):
        manager = (ROOT / "laksa_mapping" / "session_manager.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("mapping_source", FUSED_MAPPING)', manager)
        self.assertNotIn("ZED_ONLY", manager)
        self.assertEqual(output_prefix(ZED_ONLY), "/zed_rtabmap")
        self.assertEqual(output_prefix(HYBRID_SHADOW), "/laksa/mapping_shadow")

    def test_hybrid_uses_only_validated_canonical_scan(self):
        launch = (ROOT / "launch" / "mapping_shadow.launch.py").read_text(encoding="utf-8")
        self.assertIn('("scan", "/laksa/lidar/scan_validated")', launch)
        manager = (ROOT / "laksa_mapping" / "session_manager.py").read_text(encoding="utf-8")
        self.assertIn('msg.header.frame_id != "lidar_link"', manager)
        self.assertNotIn('"/scan_raw"', launch)
        self.assertNotIn("static_transform_publisher", launch)
        self.assertNotIn("laksa_lidar", launch)

    def test_no_motion_or_controller_authority(self):
        launch = (ROOT / "launch" / "mapping_shadow.launch.py").read_text(encoding="utf-8")
        forbidden = ("cmd_vel", "DriveCommand", "controller_server", "planner_server", "VESC")
        for value in forbidden:
            self.assertNotIn(value, launch)

    def test_missing_or_bad_scan_fails_closed(self):
        self.assertFalse(hybrid_ready(HYBRID_SHADOW, "GOOD", None, 1.0))
        self.assertFalse(hybrid_ready(HYBRID_SHADOW, "BAD", 0.1, 1.0))
        self.assertFalse(hybrid_ready(HYBRID_SHADOW, "GOOD", 1.1, 1.0))
        self.assertTrue(hybrid_ready(HYBRID_SHADOW, "GOOD", 0.1, 1.0))


if __name__ == "__main__":
    unittest.main()
