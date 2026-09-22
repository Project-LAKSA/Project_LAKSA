"""Offline, non-ROS qualification of G2's estimator and safety contracts."""

from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from laksa_navigation_v2.g2_synthetic_inputs import (
    SCENARIOS,
    SPEED_STD_MPS,
    VIO_POSITION_STD_M,
    all_cases,
    generate_case,
)
from laksa_navigation_v2.state_estimation_contract import (
    EKF_CONFIG,
    G2ContractError,
    SYNTHETIC_EKF_CONFIG,
    covariance_is_valid,
    load_json_yaml,
    validate_ekf_config,
    validate_g2_launch_graph,
    validate_vy_experiment_config,
    validate_zed_vio_contract,
)
from laksa_navigation_v2.vehicle_speed_adapter_contract import measured_speed_is_usable


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class G2StateEstimationContractTest(unittest.TestCase):
    def test_production_intent_ekf_contract(self) -> None:
        self.assertEqual(validate_ekf_config(), [])

    def test_synthetic_ekf_contract(self) -> None:
        self.assertEqual(validate_ekf_config(SYNTHETIC_EKF_CONFIG, synthetic=True), [])

    def test_vy_constraint_is_explicitly_test_only(self) -> None:
        self.assertEqual(validate_vy_experiment_config(), [])

    def test_zed_is_a_measurement_source_not_a_tf_authority(self) -> None:
        self.assertEqual(validate_zed_vio_contract(), [])

    def test_test_only_graph_has_no_actuator_or_global_navigation(self) -> None:
        self.assertEqual(validate_g2_launch_graph(), [])
        launch = (PACKAGE_ROOT / "launch" / "g2_local_estimation_offline.launch.py").read_text(encoding="utf-8")
        self.assertIn('default_value="false"', launch)
        self.assertNotIn("/laksa/command", launch)
        self.assertNotIn("nav2", launch.lower())

    def test_no_imu_or_command_prediction(self) -> None:
        params = load_json_yaml(EKF_CONFIG)["ekf_local_odom"]["ros__parameters"]
        self.assertFalse(params["use_control"])
        self.assertFalse(any("imu" in key.lower() for key in params))
        self.assertEqual([index for index, enabled in enumerate(params["odom0_config"]) if enabled], [0, 1, 5])
        self.assertEqual([index for index, enabled in enumerate(params["twist0_config"]) if enabled], [6])

    def test_synthetic_covariance_matches_noise(self) -> None:
        self.assertTrue(covariance_is_valid([VIO_POSITION_STD_M ** 2] + [1.0] * 35, VIO_POSITION_STD_M ** 2))
        self.assertTrue(math.isfinite(SPEED_STD_MPS ** 2))

    def test_covariance_matrix_has_no_invented_off_diagonal_correlation(self) -> None:
        from laksa_navigation_v2.g2_synthetic_inputs import speed_twist_covariance, vio_pose_covariance
        for covariance in (vio_pose_covariance(), speed_twist_covariance()):
            self.assertEqual(len(covariance), 36)
            for row in range(6):
                for column in range(6):
                    if row != column:
                        self.assertEqual(covariance[row * 6 + column], 0.0)

    def test_measured_speed_requires_fresh_telemetry_and_calibrated_variance(self) -> None:
        self.assertTrue(measured_speed_is_usable(True, 0.2, 0.01))
        self.assertFalse(measured_speed_is_usable(False, 0.2, 0.01))
        self.assertFalse(measured_speed_is_usable(True, 0.2, -1.0))

    def test_all_required_scenarios_are_deterministic_and_finite(self) -> None:
        self.assertEqual(len(SCENARIOS), 15)
        for name, samples in all_cases():
            self.assertEqual(samples, generate_case(name))
            self.assertGreater(len(samples), 1)
            for sample in samples:
                for value in (sample.truth.x_m, sample.truth.y_m, sample.truth.yaw_rad, sample.truth.vx_mps, sample.vio_x_m, sample.vio_y_m, sample.vio_yaw_rad, sample.speed_mps):
                    self.assertTrue(math.isfinite(value), name)

    def test_fault_fixtures_contain_the_intended_fault(self) -> None:
        self.assertFalse(generate_case("G2_S008_VIO_DROPOUT")[-1].vio_available)
        self.assertFalse(generate_case("G2_S009_SPEED_DROPOUT")[-1].speed_available)
        self.assertFalse(generate_case("G2_S010_ALL_INPUT_DROPOUT")[-1].vio_available)
        self.assertFalse(generate_case("G2_S010_ALL_INPUT_DROPOUT")[-1].speed_available)
        normal = generate_case("G2_S002_STRAIGHT_FORWARD")[40]
        jump = generate_case("G2_S011_VIO_POSITION_OUTLIER")[40]
        self.assertGreater(abs(jump.vio_x_m - normal.vio_x_m), 2.0)
        delayed = generate_case("G2_S013_DELAYED_SAMPLE")
        self.assertLess(delayed[40].vio_stamp_sec, delayed[39].vio_stamp_sec)
        out_of_order = generate_case("G2_S014_OUT_OF_ORDER_SAMPLE")
        self.assertLess(out_of_order[40].vio_stamp_sec, out_of_order[39].vio_stamp_sec)
        self.assertEqual(out_of_order[41].vio_stamp_sec, out_of_order[40].vio_stamp_sec)

    def test_directional_radius_cases_are_not_symmetric_magic_constants(self) -> None:
        left = generate_case("G2_S004_LEFT_RADIUS")
        right = generate_case("G2_S005_RIGHT_RADIUS")
        self.assertGreater(left[-1].truth.yaw_rad, 0.0)
        self.assertLess(right[-1].truth.yaw_rad, 0.0)
        self.assertNotAlmostEqual(abs(left[-1].truth.yaw_rad), abs(right[-1].truth.yaw_rad), places=3)

    def test_bad_contract_is_rejected(self) -> None:
        data = load_json_yaml(EKF_CONFIG)
        data["ekf_local_odom"]["ros__parameters"]["world_frame"] = "map"
        temporary = PACKAGE_ROOT / "test" / ".bad_ekf_contract.json"
        try:
            temporary.write_text(json.dumps(data), encoding="utf-8")
            self.assertIn("wrong_world_frame", validate_ekf_config(temporary))
        finally:
            temporary.unlink(missing_ok=True)

    def test_non_json_yaml_is_rejected_deterministically(self) -> None:
        temporary = PACKAGE_ROOT / "test" / ".invalid_contract.yaml"
        try:
            temporary.write_text("not: [valid", encoding="utf-8")
            with self.assertRaises(G2ContractError):
                load_json_yaml(temporary)
        finally:
            temporary.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
