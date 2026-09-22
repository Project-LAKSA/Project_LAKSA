#!/usr/bin/env python3

"""Unit tests for LAKSA's ROS-independent Ackermann command conversion."""

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from laksa_control_math import (  # noqa: E402
    canonical_map_invalid_reason,
    limited_ackermann_command,
    map_geometry_is_sane,
    route_data_preflight_reason,
    tf_preflight_reason,
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


class A043RoutePreflightTest(unittest.TestCase):
    def test_valid_old_map_does_not_expire(self) -> None:
        # The retained map may be arbitrarily old; only genuinely dynamic
        # stamps participate in freshness checks.
        now_ns = 1_000_000_000_000
        fresh = now_ns - 100_000_000
        reason = route_data_preflight_reason(
            now_ns,
            map_received=True,
            map_invalid_reason=None,
            dynamic_checks=((fresh, 750_000_000, "ODOM_STALE"),),
        )
        self.assertIsNone(reason)

    def test_no_map_is_explicit(self) -> None:
        self.assertEqual(
            route_data_preflight_reason(10, False, None, ()),
            "MAP_NOT_RECEIVED",
        )

    def test_invalid_map_reasons_are_explicit(self) -> None:
        self.assertEqual(
            canonical_map_invalid_reason(0, 10, 0.05, 0, "map", "map"),
            "MAP_INVALID_DIMENSIONS",
        )
        self.assertEqual(
            canonical_map_invalid_reason(10, 10, 0.05, 99, "map", "map"),
            "MAP_INVALID_DATA",
        )

    def test_stale_odometry_still_fails(self) -> None:
        self.assertEqual(
            route_data_preflight_reason(
                2_000_000_000,
                True,
                None,
                ((1, 750_000_000, "ODOM_STALE"),),
            ),
            "ODOM_STALE",
        )

    def test_old_header_stamp_is_not_a_validity_input(self) -> None:
        self.assertIsNone(
            canonical_map_invalid_reason(10, 10, 0.05, 100, "map", "map")
        )

    def test_missing_tf_still_fails_explicitly(self) -> None:
        self.assertEqual(
            tf_preflight_reason("map to base unavailable"),
            "TF_UNAVAILABLE: map to base unavailable",
        )


if __name__ == "__main__":
    unittest.main()
