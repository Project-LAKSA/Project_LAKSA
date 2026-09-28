"""Deterministic contracts for the pinned RPP plus LAKSA feasibility delta."""

import hashlib
import math
import unittest
from pathlib import Path

import yaml

from laksa_speed_race.gym_adapter_node import StateStampGate
from laksa_speed_race.nav2_ackermann_adapter_node import twist_to_ackermann
from laksa_speed_race.nav2_raceline_node import (
    FROZEN_FULL_RACELINE_SHA256,
    FROZEN_RACELINE_SHA256,
    UNROLLED_POSES,
    load_frozen_raceline,
    monotonic_progress_index,
    unroll_closed_raceline,
)


ROOT = Path(__file__).resolve().parents[1]
COURSE = ROOT / "course" / "canonical" / "speed_course"


class Nav2RppContractTests(unittest.TestCase):
    def setUp(self):
        self.raceline_path = COURSE / "pure_pursuit_raceline.csv"
        self.points = load_frozen_raceline(self.raceline_path)

    def test_frozen_raceline_hash_and_parser_semantics(self):
        self.assertEqual(hashlib.sha256(self.raceline_path.read_bytes()).hexdigest(), FROZEN_RACELINE_SHA256)
        self.assertEqual(len(self.points), 547)
        self.assertEqual(self.points[0], self.points[-1])
        self.assertNotIn(self.points[0], self.points[1:-1])
        self.assertEqual(
            hashlib.sha256((COURSE / "speed_course_raceline.csv").read_bytes()).hexdigest(),
            FROZEN_FULL_RACELINE_SHA256,
        )

    def test_four_copy_ring_unrolling_is_exact(self):
        unrolled = unroll_closed_raceline(self.points)
        self.assertEqual(len(unrolled), UNROLLED_POSES)
        for copy in range(4):
            self.assertEqual(unrolled[copy * 546], self.points[0])
        self.assertEqual(unrolled[-1], self.points[0])

    def test_seam_progression_is_monotonic(self):
        unrolled = unroll_closed_raceline(self.points)
        at_second_copy = monotonic_progress_index(
            unrolled, unrolled[546].x_m, unrolled[546].y_m, 540
        )
        self.assertEqual(at_second_copy, 546)
        self.assertGreaterEqual(
            monotonic_progress_index(unrolled, unrolled[547].x_m, unrolled[547].y_m, at_second_copy),
            at_second_copy,
        )

    def test_rpp_initial_configuration_enables_selected_upstream_mechanisms(self):
        config = yaml.safe_load((ROOT / "config" / "c1_nav2_rpp.yaml").read_text())
        host = config["/c1/rpp_lockstep_host"]["ros__parameters"]
        self.assertEqual(
            host["controller_plugin"],
            "laksa_speed_race_nav2::AckermannFeasibleRppController",
        )
        params = host["RPP"]
        self.assertTrue(params["use_interpolation"])
        self.assertTrue(params["use_velocity_scaled_lookahead_dist"])
        self.assertTrue(params["use_regulated_linear_velocity_scaling"])
        self.assertTrue(params["use_collision_detection"])
        self.assertFalse(params["allow_reversing"])
        self.assertFalse(params["use_rotate_to_heading"])
        self.assertEqual(params["max_robot_pose_search_dist"], 2.0)
        self.assertEqual(params["desired_linear_vel"], 1.0)

    def test_mppi_ackermann_configuration_preserves_frozen_physics(self):
        config = yaml.safe_load((ROOT / "config" / "c1_nav2_mppi.yaml").read_text())
        host = config["/c1/mppi_lockstep_host"]["ros__parameters"]
        self.assertEqual(host["controller_plugin"], "nav2_mppi_controller::MPPIController")
        self.assertTrue(host["independent_safety_veto"])
        params = host["MPPI"]
        self.assertEqual(params["motion_model"], "Ackermann")
        self.assertAlmostEqual(params["AckermannConstraints"]["min_turning_r"], 1.0937226373133722)
        self.assertEqual(params["vx_min"], 0.0)
        self.assertLessEqual(params["vx_max"], 1.0)
        self.assertTrue(params["CostCritic"]["consider_footprint"])
        self.assertEqual(params["model_dt"], 0.01)
        self.assertFalse(params["regenerate_noises"])

    def test_mppi_command_is_validated_and_vetoed_before_publication(self):
        source = (
            ROOT.parent / "laksa_speed_race_nav2" / "src" / "rpp_lockstep_host.cpp"
        ).read_text()
        validation = source.index("validate_ackermann_twist(")
        veto = source.index("independent_safety_check(", validation)
        publish = source.index("command_pub_->publish(command)", veto)
        self.assertLess(validation, veto)
        self.assertLess(veto, publish)
        self.assertIn("physical_feasibility_violation", source[validation:publish])
        self.assertIn("independent_full_footprint_safety_veto", source[veto:publish])

    def test_arm64_path_align_disposition_is_per_trajectory_and_test_only(self):
        patcher = (ROOT / "docker" / "patch_nav2_mppi_path_align.py").read_text()
        self.assertIn("EXPECT_FLOAT_EQ(cost, 6.6f)", patcher)
        self.assertIn("for (const auto cost : costs)", patcher)
        self.assertNotIn("6600.0, 2e-2", patcher)

    def test_ackermann_analytical_sign_and_saturation_cases(self):
        speed, positive, saturated = twist_to_ackermann(1.0, 0.5)
        self.assertEqual(speed, 1.0)
        self.assertAlmostEqual(positive, math.atan(0.324 * 0.5))
        self.assertFalse(saturated)
        _, negative, _ = twist_to_ackermann(1.0, -0.5)
        self.assertAlmostEqual(negative, -positive)
        _, clamped, saturated = twist_to_ackermann(0.428551, -0.91431)
        self.assertEqual(clamped, -0.288)
        self.assertTrue(saturated)

    def test_ackermann_zero_and_reverse_fail_closed(self):
        self.assertEqual(twist_to_ackermann(0.0, 0.0), (0.0, 0.0, False))
        with self.assertRaises(ValueError):
            twist_to_ackermann(0.0, 0.1)
        with self.assertRaises(ValueError):
            twist_to_ackermann(-0.1, 0.0)

    def test_constrained_initial_command_is_not_materially_changed_downstream(self):
        wheelbase = 0.324
        steering_limit = 0.288
        curvature_limit = math.tan(steering_limit) / wheelbase
        linear = 0.4285508430
        angular = linear * -curvature_limit
        speed, steering, saturated = twist_to_ackermann(linear, angular)
        self.assertEqual(speed, linear)
        self.assertAlmostEqual(steering, -steering_limit, places=15)
        self.assertFalse(saturated)

    def test_ttc_and_returned_twist_share_the_feasible_command(self):
        source = (
            ROOT.parent
            / "laksa_speed_race_nav2"
            / "src"
            / "ackermann_feasible_rpp_controller.cpp"
        ).read_text()
        constraint = source.index("const auto feasible = constrain_ackermann_curvature(")
        ttc = source.index("isCollisionImminent(pose, linear_vel, angular_vel", constraint)
        returned_linear = source.index("cmd_vel.twist.linear.x = linear_vel", ttc)
        returned_angular = source.index("cmd_vel.twist.angular.z = angular_vel", ttc)
        self.assertLess(constraint, ttc)
        self.assertLess(ttc, returned_linear)
        self.assertLess(ttc, returned_angular)
        self.assertIn("feasible.commanded_curvature_1pm", source[constraint:ttc])

    def test_duplicate_state_stamp_cannot_be_accepted_twice(self):
        gate = StateStampGate()
        gate.update_state(1_000_000_000)
        self.assertEqual(gate.accept_command(1_000_000_000), "ACCEPT")
        self.assertEqual(gate.accept_command(1_000_000_000), "DUPLICATE")
        gate.update_state(1_010_000_000)
        self.assertEqual(gate.accept_command(1_000_000_000), "DUPLICATE")
        self.assertEqual(gate.accept_command(1_010_000_001), "MISMATCH")
        self.assertEqual(gate.accept_command(1_010_000_000), "ACCEPT")

    def test_lockstep_host_has_no_control_timer_and_one_compute_site(self):
        source = (ROOT.parent / "laksa_speed_race_nav2" / "src" / "rpp_lockstep_host.cpp").read_text()
        self.assertNotIn("create_wall_timer", source)
        self.assertNotIn("create_timer", source)
        self.assertEqual(source.count("computeVelocityCommands("), 1)
        self.assertIn("Rejected duplicate odometry stamp", source)
        self.assertIn('"/c1/nav2_cmd_vel"', source)
        self.assertIn("setUsingDedicatedThread(true)", source)

    def test_three_lap_gate_is_byte_identical_to_c1_1(self):
        gate = ROOT / "laksa_speed_race" / "three_lap_gate.py"
        self.assertEqual(
            hashlib.sha256(gate.read_bytes()).hexdigest(),
            "5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1",
        )

    def test_c1_1_geometry_preflight_remains_present(self):
        report = yaml.safe_load((COURSE / "course_manifest.json").read_text())
        self.assertEqual(
            report["generated_asset_sha256"]["pure_pursuit_raceline.csv"],
            FROZEN_RACELINE_SHA256,
        )
        self.assertEqual(
            report["generated_asset_sha256"]["speed_course_raceline.csv"],
            FROZEN_FULL_RACELINE_SHA256,
        )


if __name__ == "__main__":
    unittest.main()
