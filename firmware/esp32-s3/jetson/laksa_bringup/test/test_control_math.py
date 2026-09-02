#!/usr/bin/env python3

"""Unit tests for LAKSA's ROS-independent Ackermann command conversion."""

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from laksa_control_math import (  # noqa: E402
    limited_ackermann_command,
    map_geometry_is_sane,
)


class LimitedAckermannCommandTest(unittest.TestCase):
    def setUp(self) -> None:
        self.wheelbase = 0.324
        self.left_limit = 0.523
        self.right_limit = 0.288
        self.servo_limit = 0.523

    def convert(self, speed, yaw_rate, limit=0.24):
        return limited_ackermann_command(
            speed,
            yaw_rate,
            limit,
            self.wheelbase,
            self.left_limit,
            self.right_limit,
            self.servo_limit,
        )

    def test_speed_limit_preserves_forward_curvature(self) -> None:
        road_angle = 0.20
        raw_speed = 0.36
        yaw_rate = raw_speed * math.tan(road_angle) / self.wheelbase

        speed, steering = self.convert(raw_speed, yaw_rate)

        self.assertAlmostEqual(speed, 0.24)
        self.assertAlmostEqual(steering, road_angle, places=7)

    def test_speed_limit_preserves_reverse_curvature(self) -> None:
        road_angle = 0.20
        raw_speed = -0.36
        yaw_rate = raw_speed * math.tan(road_angle) / self.wheelbase

        speed, steering = self.convert(raw_speed, yaw_rate)

        self.assertAlmostEqual(speed, -0.24)
        self.assertAlmostEqual(steering, road_angle, places=7)

    def test_asymmetric_right_endpoint_maps_to_full_servo_command(self) -> None:
        requested_angle = -0.40
        speed = 0.20
        yaw_rate = speed * math.tan(requested_angle) / self.wheelbase

        _, steering = self.convert(speed, yaw_rate)

        self.assertAlmostEqual(steering, -self.servo_limit, places=7)

    def test_zero_linear_speed_cannot_request_a_spin(self) -> None:
        speed, steering = self.convert(0.0, 0.8)
        self.assertEqual(speed, 0.0)
        self.assertEqual(steering, 0.0)

    def test_non_finite_command_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.convert(math.nan, 0.0)


class MapGeometryTest(unittest.TestCase):
    def test_kitchen_scale_map_is_accepted(self) -> None:
        self.assertTrue(map_geometry_is_sane(400, 300, 0.05))

    def test_diverged_map_is_rejected(self) -> None:
        self.assertFalse(map_geometry_is_sane(9714, 6070, 0.05))

    def test_invalid_resolution_is_rejected(self) -> None:
        self.assertFalse(map_geometry_is_sane(100, 100, math.nan))


if __name__ == "__main__":
    unittest.main()
