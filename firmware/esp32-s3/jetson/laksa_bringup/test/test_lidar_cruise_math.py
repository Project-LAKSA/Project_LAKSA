import math
import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from lidar_cruise_math import (  # noqa: E402
    ReverseRecoveryGate,
    sector_percentile,
    wrap_angle,
    yaw_rate_from_steering,
)


class LidarCruiseMathTests(unittest.TestCase):
    def test_wrap_angle(self) -> None:
        self.assertTrue(math.isclose(wrap_angle(3.0 * math.pi), -math.pi))
        self.assertTrue(math.isclose(wrap_angle(-0.25), -0.25))

    def test_lidar_yaw_pi_makes_raw_rear_the_vehicle_front(self) -> None:
        ranges = [10.0] * 360
        # Raw LiDAR angle -pi points along vehicle +x after applying yaw pi.
        ranges[0] = 0.35
        measured = sector_percentile(
            ranges,
            -math.pi,
            2.0 * math.pi / 360.0,
            math.pi,
            0.0,
            math.radians(0.2),
            0.0,
            0.05,
            12.0,
        )
        self.assertTrue(math.isclose(measured, 0.35))

    def test_reverse_with_opposite_steering_rotates_toward_same_open_side(self) -> None:
        forward_yaw = yaw_rate_from_steering(0.24, 0.27, 0.324)
        recovery_yaw = yaw_rate_from_steering(-0.22, -0.27, 0.324)
        self.assertGreater(forward_yaw, 0.0)
        self.assertGreater(recovery_yaw, 0.0)

    def test_stopped_ackermann_command_has_no_yaw_rate(self) -> None:
        self.assertEqual(yaw_rate_from_steering(0.0, 0.27, 0.324), 0.0)

    def test_reverse_requires_persistent_blockage_and_rear_clearance(self) -> None:
        gate = ReverseRecoveryGate(2.0, 0.75, 8.0)
        start = 1_000_000_000
        self.assertEqual(gate.update(start, True, True), gate.WAITING)
        self.assertEqual(
            gate.update(start + 1_999_999_999, True, True), gate.WAITING
        )
        self.assertEqual(
            gate.update(start + 2_000_000_000, True, False), gate.WAITING
        )
        self.assertEqual(
            gate.update(start + 2_000_000_001, True, True), gate.REVERSE
        )

    def test_reverse_is_bounded_and_cooldown_prevents_oscillation(self) -> None:
        gate = ReverseRecoveryGate(2.0, 0.75, 8.0)
        start = 1_000_000_000
        gate.update(start, True, True)
        reverse_start = start + 2_000_000_000
        self.assertEqual(gate.update(reverse_start, True, True), gate.REVERSE)
        self.assertEqual(
            gate.update(reverse_start + 749_999_999, True, True), gate.REVERSE
        )
        self.assertEqual(
            gate.update(reverse_start + 750_000_000, True, True), gate.WAITING
        )
        self.assertEqual(
            gate.update(reverse_start + 7_999_999_999, True, True), gate.WAITING
        )

    def test_clear_front_resets_blockage_evidence(self) -> None:
        gate = ReverseRecoveryGate(2.0, 0.75, 8.0)
        start = 1_000_000_000
        gate.update(start, True, True)
        self.assertEqual(gate.update(start + 1_500_000_000, False, True), gate.FORWARD)
        self.assertEqual(gate.update(start + 3_000_000_000, True, True), gate.WAITING)


if __name__ == "__main__":
    unittest.main()
