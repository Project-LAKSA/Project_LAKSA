import math
from types import SimpleNamespace
import unittest

from laksa_lidar.scan_quality import BAD, DEGRADED, GOOD, RateWindow, Thresholds, analyze_scan, health_state


T = Thresholds(5.0, 15.0, 1.0, 350.0, 3, 0.05, 0.02, 4, 0.5)


def scan(ranges=None, **changes):
    count = 361
    data = dict(
        header=SimpleNamespace(frame_id="lidar_link", stamp=SimpleNamespace(sec=10, nanosec=1)),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / (count - 1),
        range_min=0.2,
        range_max=12.0,
        scan_time=0.1,
        ranges=[2.0] * count if ranges is None else ranges,
    )
    data.update(changes)
    return SimpleNamespace(**data)


def state(metrics, rate=10.0, age=0.1, tf=True, invalid=0):
    return health_state(metrics, rate, age, tf, invalid, T)[0]


class ScanQualityTests(unittest.TestCase):
    def test_valid_normal_scan_is_good(self):
        metrics = analyze_scan(scan(), T)
        self.assertTrue(metrics.structural_valid)
        self.assertEqual(metrics.valid_return_ratio, 1.0)
        self.assertEqual(state(metrics), GOOD)

    def test_empty_ranges(self):
        self.assertIn("empty_ranges", analyze_scan(scan(ranges=[]), T).errors)

    def test_bad_angle_increment(self):
        metrics = analyze_scan(scan(angle_increment=0.0), T)
        self.assertIn("invalid_angle_increment", metrics.errors)
        self.assertEqual(state(metrics), BAD)

    def test_invalid_range_limits(self):
        self.assertIn("invalid_range_limits", analyze_scan(scan(range_min=2.0, range_max=1.0), T).errors)

    def test_nan_heavy_scan_is_degraded(self):
        metrics = analyze_scan(scan([math.nan] * 350 + [1.0] * 11), T)
        self.assertTrue(metrics.structural_valid)
        self.assertEqual(metrics.nan_count, 350)
        self.assertEqual(state(metrics), DEGRADED)

    def test_all_inf_is_structurally_accepted(self):
        metrics = analyze_scan(scan([math.inf] * 361), T)
        self.assertTrue(metrics.structural_valid)
        self.assertEqual(metrics.inf_count, 361)
        self.assertEqual(state(metrics), DEGRADED)

    def test_partial_finite_returns(self):
        metrics = analyze_scan(scan([1.0] * 100 + [math.inf] * 200 + [0.0] * 61), T)
        self.assertEqual(metrics.finite_count, 161)
        self.assertEqual(metrics.valid_return_count, 100)
        self.assertEqual(metrics.zero_count, 61)

    def test_stale_and_missing_tf_are_bad(self):
        metrics = analyze_scan(scan(), T)
        self.assertEqual(state(metrics, age=1.01), BAD)
        self.assertEqual(state(metrics, tf=False), BAD)

    def test_rate_calculation_and_health(self):
        window = RateWindow(4)
        for timestamp in (0.0, 0.1, 0.2, 0.3):
            window.add(timestamp)
        self.assertAlmostEqual(window.rate_hz, 10.0)
        self.assertEqual(state(analyze_scan(scan(), T), rate=2.0), DEGRADED)

    def test_health_transitions_and_consecutive_invalid(self):
        valid = analyze_scan(scan(), T)
        mismatch = analyze_scan(scan([1.0] * 300), T)
        self.assertEqual(state(valid), GOOD)
        self.assertEqual(state(mismatch, invalid=1), DEGRADED)
        self.assertEqual(state(mismatch, invalid=3), BAD)


if __name__ == "__main__":
    unittest.main()
