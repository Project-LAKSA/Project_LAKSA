#!/usr/bin/env python3

"""Regression tests for the hardest simulated Ackermann kitchen maneuver."""

import sys
import unittest
from pathlib import Path

SIMULATION = Path(__file__).resolve().parent
SCRIPTS = SIMULATION.parent / "laksa_bringup" / "scripts"
sys.path[:0] = [str(SIMULATION), str(SCRIPTS)]

from ackermann_kitchen_sim import simulate  # noqa: E402
from lidar_rollout_core import (  # noqa: E402
    AckermannRolloutController,
    RolloutParameters,
)


class KitchenSimulationTests(unittest.TestCase):
    def test_noisy_ninety_degree_dead_end_recovery(self) -> None:
        result = simulate(
            AckermannRolloutController(RolloutParameters()),
            "corner",
            1,
            duration=90.0,
            seed=29,
            sensor_noise_std=0.012,
            command_latency_sec=0.15,
            steering_time_constant_sec=0.22,
            odom_position_noise_std=0.008,
            odom_yaw_noise_std=0.008,
        )
        self.assertTrue(result.reached_coverage_target)
        self.assertGreaterEqual(result.coverage, 0.98)
        self.assertEqual(result.collisions, 0)
        self.assertLessEqual(result.recoveries, 4)
        self.assertLessEqual(result.reverse_meters, 1.85)


if __name__ == "__main__":
    unittest.main()
