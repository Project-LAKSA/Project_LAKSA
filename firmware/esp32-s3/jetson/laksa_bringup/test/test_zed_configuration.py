#!/usr/bin/env python3

"""Hardware-independent checks for the calibrated ZED profile."""

import ast
from pathlib import Path
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


class ZedConfigurationTest(unittest.TestCase):
    def test_mount_uses_measured_vehicle_transform(self):
        document = yaml.safe_load(
            (PACKAGE / "config" / "zed_mount.yaml").read_text(encoding="utf-8")
        )
        mount = document["camera_mount"]
        self.assertIs(mount["configured"], True)
        self.assertEqual(mount["parent_frame"], "laksa_base_footprint")
        self.assertEqual(mount["child_frame"], "zed_camera_link")
        self.assertAlmostEqual(float(mount["x_m"]), 0.10807, places=5)
        self.assertAlmostEqual(float(mount["y_m"]), 0.0, places=5)
        self.assertAlmostEqual(float(mount["z_m"]), 0.140, places=5)
        self.assertAlmostEqual(float(mount["roll_rad"]), 0.0, places=5)
        self.assertAlmostEqual(float(mount["pitch_rad"]), 0.0698132, places=5)
        self.assertAlmostEqual(float(mount["yaw_rad"]), 0.0, places=5)

    def test_zed_profile_has_deterministic_robot_limits(self):
        document = yaml.safe_load(
            (PACKAGE / "config" / "zed2i_robot.yaml").read_text(encoding="utf-8")
        )
        parameters = document["/**"]["ros__parameters"]
        self.assertEqual(parameters["general"]["grab_frame_rate"], 30)
        self.assertEqual(parameters["general"]["pub_resolution"], "CUSTOM")
        self.assertEqual(parameters["general"]["grab_compute_capping_fps"], 15.0)
        self.assertEqual(parameters["general"]["pub_frame_rate"], 10.0)
        self.assertEqual(parameters["depth"]["point_cloud_freq"], 5.0)
        self.assertEqual(parameters["depth"]["depth_mode"], "NEURAL_LIGHT")
        self.assertTrue(parameters["depth"]["voxel_point_cloud"])
        self.assertFalse(parameters["mapping"]["mapping_enabled"])
        self.assertFalse(parameters["object_detection"]["od_enabled"])

    def test_launch_file_parses(self):
        source = (
            PACKAGE / "launch" / "laksa_3d_mapping.launch.py"
        ).read_text(encoding="utf-8")
        ast.parse(source)


if __name__ == "__main__":
    unittest.main()
