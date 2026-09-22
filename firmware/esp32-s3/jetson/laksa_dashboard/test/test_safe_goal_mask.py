import base64
import math
import random
import unittest

from laksa_dashboard.safe_goal_mask import (
    _build_safe_mask_scipy,
    build_safe_mask,
    build_safe_mask_reference,
    circumscribed_radius,
    encoded_mask,
    mask_contains,
    scipy_distance_transform_edt,
)


class SafeGoalMaskTest(unittest.TestCase):
    def bits(self, data, width, height, resolution, radius):
        packed, _ = build_safe_mask(data, width, height, resolution, radius)
        return [mask_contains(packed, index) for index in range(width * height)]

    def test_all_unknown_and_all_free(self):
        self.assertFalse(any(self.bits([-1] * 25, 5, 5, 0.1, 0.2)))
        free = self.bits([0] * 81, 9, 9, 0.1, 0.2)
        self.assertTrue(free[4 * 9 + 4])
        self.assertFalse(free[0])

    def test_lethal_and_inscribed_are_blocked(self):
        for cost in (99, 100):
            data = [0] * 225; data[7 * 15 + 7] = cost
            bits = self.bits(data, 15, 15, 0.1, 0.15)
            self.assertFalse(bits[7 * 15 + 7])
            self.assertTrue(bits[3 * 15 + 3])

    def test_border_is_blocked_space(self):
        bits = self.bits([0] * 121, 11, 11, 0.05, 0.12)
        self.assertFalse(bits[5 * 11])
        self.assertTrue(bits[5 * 11 + 5])

    def test_clearance_threshold_is_conservative(self):
        data = [0] * 49; data[3 * 7 + 3] = 100
        exact = 1.0 - math.sqrt(0.5)
        # The native backend intentionally adds a 1e-9-cell conservative
        # margin. A point just inside the mathematical boundary remains safe;
        # the boundary and any larger radius remain blocked.
        self.assertTrue(self.bits(data, 7, 7, 1.0, exact - 2.0e-9)[3 * 7 + 4])
        self.assertFalse(self.bits(data, 7, 7, 1.0, exact)[3 * 7 + 4])
        self.assertFalse(self.bits(data, 7, 7, 1.0, exact + 1.0e-4)[3 * 7 + 4])

    def test_resolutions_and_encoding(self):
        radius = circumscribed_radius(0.419, 0.149, 0.148, 0.148)
        self.assertAlmostEqual(radius, math.hypot(0.419, 0.148))
        for resolution in (0.025, 0.05, 0.10):
            text, count = encoded_mask([0] * 400, 20, 20, resolution, radius)
            self.assertEqual(len(base64.b64decode(text)), 50)
            self.assertGreaterEqual(count, 0)

    @unittest.skipIf(scipy_distance_transform_edt is None, "optional native SciPy EDT unavailable")
    def test_native_edt_never_marks_reference_unsafe_cell_safe(self):
        generator = random.Random(2701)
        for width, height in ((5, 7), (11, 9), (17, 13)):
            for _ in range(20):
                data = [generator.choice((0, 0, 0, -1, 99, 100)) for _ in range(width * height)]
                reference, _ = build_safe_mask_reference(data, width, height, 0.05, 0.19)
                native, _ = _build_safe_mask_scipy(data, width, height, 0.05, 0.19)
                for index in range(width * height):
                    if mask_contains(native, index):
                        self.assertTrue(mask_contains(reference, index))


if __name__ == "__main__":
    unittest.main()
