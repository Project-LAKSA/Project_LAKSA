import math
import unittest

from laksa_dashboard.lidar_projection import project_scan


class LidarProjectionTest(unittest.TestCase):
    def test_planar_scan_uses_full_rigid_transform_without_extrusion(self):
        points = project_scan(
            [1.0, 2.0, math.inf], 0.0, math.pi / 2.0, 0.1, 10.0,
            (1.0, 2.0, 0.4), (0.0, 0.0, 0.0, 1.0),
        )
        for actual, expected in zip(points, [2.0, 2.0, 0.4, 1.0, 4.0, 0.4]):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(len(points), 2 * 3)

    def test_rotation_preserves_real_sensor_plane(self):
        half = math.sqrt(0.5)
        points = project_scan(
            [1.0], 0.0, 1.0, 0.1, 10.0,
            (0.0, 0.0, 0.2), (0.0, half, 0.0, half),
        )
        self.assertAlmostEqual(points[0], 0.0, places=6)
        self.assertAlmostEqual(points[1], 0.0, places=6)
        self.assertAlmostEqual(points[2], -0.8, places=6)


if __name__ == "__main__":
    unittest.main()
