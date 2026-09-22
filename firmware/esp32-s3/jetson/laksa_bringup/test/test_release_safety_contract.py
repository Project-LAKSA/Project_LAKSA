#!/usr/bin/env python3

"""Static release gates for the fail-closed Ackermann FollowPath wiring."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ReleaseSafetyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.supervisor = (ROOT / "scripts/drive_supervisor_node.py").read_text()
        cls.system_launch = (ROOT / "launch/laksa_system.launch.py").read_text()
        cls.manual_launch = (ROOT / "launch/manual_control.launch.py").read_text()
        cls.drive = (ROOT / "config/drive_supervisor.yaml").read_text()
        cls.nav = (ROOT / "config/nav2_ackermann.yaml").read_text()

    def test_autonomy_and_actuation_are_opt_in(self):
        self.assertIn('"autonomy_enabled": False', self.supervisor)
        self.assertIn('"actuation_enabled": False', self.supervisor)
        self.assertIn("self._autonomous = False", self.supervisor)
        self.assertIn('"enable_autonomy",\n                default_value="false"', self.system_launch)
        self.assertIn('"enable_actuation",\n                default_value="false"', self.system_launch)
        self.assertIn('{"autonomy_enabled": True, "actuation_enabled": True}', self.manual_launch)

    def test_production_supervisor_uses_accepted_odom_and_tf_contract(self):
        self.assertIn('"odom_topic": "/laksa/odometry/fused"', self.supervisor)
        self.assertIn('"base_frame": "base_footprint"', self.supervisor)
        self.assertIn("self._odom_topic", self.supervisor)
        self.assertNotIn('create_subscription(Odometry, "/laksa/odom"', self.supervisor)
        self.assertNotIn('"laksa_base_footprint",\n                    Time()', self.supervisor)

    def test_dry_run_has_observable_candidate_but_forces_brake(self):
        self.assertIn('"/laksa/autonomy_candidate_command"', self.supervisor)
        self.assertIn("if not self._actuation_enabled:", self.supervisor)
        self.assertIn("command = DriveCommand()", self.supervisor)
        self.assertIn("command.brake = True", self.supervisor)

    def test_explicit_arm_disarm_and_manual_preemption(self):
        self.assertIn('"/laksa/autonomy/set_armed"', self.supervisor)
        self.assertIn('self._abort_autonomy("Xbox manual override")', self.supervisor)
        self.assertIn('response.message = "AUTONOMY_DISARMED"', self.supervisor)
        self.assertIn('"/laksa/lidar/scan_validated"', self.supervisor)
        self.assertIn('"/local_costmap/costmap_raw"', self.supervisor)
        self.assertIn('"OBSTACLE_SOURCE_STALE: local costmap"', self.supervisor)

    def test_a043_map_age_is_diagnostic_only(self):
        self.assertNotIn('"SLAM map is stale"', self.supervisor)
        self.assertNotIn("self._map_timeout_ns", self.supervisor)
        self.assertNotIn("\n    map_timeout_sec:", self.drive)
        self.assertIn("canonical_map_invalid_reason", self.supervisor)
        self.assertIn('"MAP_NOT_RECEIVED"', self.supervisor)
        self.assertIn("tf_preflight_reason(str(error))", self.supervisor)

    def test_controller_watchdog_and_initial_speed_cap(self):
        self.assertIn("nav_timeout_sec: 0.25", self.drive)
        self.assertIn("navigation_max_erpm: 620.0", self.drive)
        self.assertIn("vx_max: 0.15", self.nav)

    def test_installed_humble_ackermann_and_dynamic_costmap_contract(self):
        self.assertIn("plugin: nav2_mppi_controller::MPPIController", self.nav)
        self.assertIn("motion_model: Ackermann", self.nav)
        self.assertIn("motion_model_for_search: REEDS_SHEPP", self.nav)
        self.assertIn("plugin: nav2_costmap_2d::VoxelLayer", self.nav)
        self.assertIn("topic: /laksa/lidar/scan_validated", self.nav)
        self.assertIn("topic: /zed/zed_node/point_cloud/cloud_registered", self.nav)


if __name__ == "__main__":
    unittest.main()
