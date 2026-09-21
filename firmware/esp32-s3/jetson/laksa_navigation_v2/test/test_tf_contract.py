import json
import math
import unittest
from pathlib import Path

from laksa_navigation_v2.tf_contract import BodyPose, Edge, project_body_to_base_footprint, validate, validate_authority_contract


ROOT = Path(__file__).resolve().parents[1]


class TfContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = json.loads((ROOT / "config/tf_authority_contract.json").read_text())
        self.vehicle = json.loads((ROOT / "config/vehicle_contract.yaml").read_text())

    def test_complete_rep105_tree_has_one_authority_per_child(self):
        self.assertEqual(validate_authority_contract(self.contract), [])

    def test_camera_is_not_vehicle_root(self):
        errors = validate([Edge("zed_camera_link", "base_link", "bad"), Edge("base_link", "lidar_link", "rsp")], root="zed_camera_link")
        self.assertIn("camera_as_vehicle_root", errors)

    def test_duplicate_map_to_odom_authority_fails(self):
        edges = [Edge(item["parent"], item["child"], item["authority"]) for item in self.contract["edges"]]
        errors = validate(edges + [Edge("map", "odom", "bad_mapping")])
        self.assertIn("duplicate_child:odom", errors)
        self.assertIn("duplicate_authority:map->odom", errors)

    def test_duplicate_local_odom_authority_fails(self):
        edges = [Edge(item["parent"], item["child"], item["authority"]) for item in self.contract["edges"]]
        self.assertIn("duplicate_child:base_footprint", validate(edges + [Edge("odom", "base_footprint", "zed")]))

    def test_loop_and_missing_sensor_fail(self):
        errors = validate([Edge("map", "odom", "global"), Edge("odom", "base_footprint", "local"), Edge("base_footprint", "map", "bad")])
        self.assertTrue(any(error.startswith("loop:") for error in errors))
        self.assertIn("missing_sensor_edge:lidar_link", errors)

    def test_ramp_projection_keeps_navigation_planar(self):
        for pitch in (math.radians(10), math.radians(-10)):
            projection = project_body_to_base_footprint(BodyPose(1.2, -0.4, 0.13, math.radians(2), pitch, 0.7))
            nav = projection.navigation_xyz_rpy
            body = projection.body_from_navigation_xyz_rpy
            self.assertEqual(nav[2:5], (0.0, 0.0, 0.0))
            self.assertAlmostEqual(nav[5], 0.7)
            self.assertAlmostEqual(body[2], 0.13)
            self.assertAlmostEqual(body[3], math.radians(2))
            self.assertAlmostEqual(body[4], pitch)

    def test_camera_mount_pitch_remains_independent_of_ramp_pitch(self):
        zed_pitch = self.vehicle["sensors"]["zed"]["rpy_rad"][1]
        self.assertGreater(zed_pitch, 0.0)
        self.assertTrue(math.isclose(zed_pitch, 0.06981317008, abs_tol=1e-12))

    def test_imu_is_explicitly_unknown(self):
        self.assertIsNone(self.vehicle["sensors"]["imu"]["xyz_m"])
        self.assertIn("base_link->imu_link", self.contract["unmeasured_edges"])

    def test_offline_harness_is_explicitly_test_only_and_non_motion(self):
        harness = json.loads((ROOT / "config/g1_offline_tf_harness.json").read_text())
        self.assertTrue(harness["test_only"])
        self.assertIn("motion command publishers", harness["forbidden"])
        launch = (ROOT / "launch/g1_offline_tf_harness.launch.py").read_text()
        self.assertIn('default_value="false"', launch)
        self.assertNotIn("nav2_", launch)


if __name__ == "__main__":
    unittest.main()
