import unittest

import numpy as np

from laksa_dashboard.spatial_sampling import spatial_voxel_sample_indices


class SpatialSamplingTest(unittest.TestCase):
    def test_is_exact_deterministic_and_unique(self):
        rng = np.random.default_rng(42)
        cloud = rng.normal(size=(30000, 3))
        first = spatial_voxel_sample_indices(cloud, 15000)
        second = spatial_voxel_sample_indices(cloud, 15000)
        self.assertEqual(len(first), 15000)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(len(np.unique(first)), 15000)

    def test_preserves_small_cloud_and_handles_degenerate_axes(self):
        line = np.column_stack((np.arange(50), np.zeros(50), np.zeros(50)))
        np.testing.assert_array_equal(
            spatial_voxel_sample_indices(line, 100), np.arange(50)
        )
        selected = spatial_voxel_sample_indices(line, 17)
        self.assertEqual(len(selected), 17)
        self.assertEqual(len(np.unique(selected)), 17)

    def test_spatial_voxels_cover_sparse_region_better_than_input_stride(self):
        dense = np.column_stack((
            np.linspace(0.0, 1.0, 9900), np.zeros(9900), np.zeros(9900)
        ))
        sparse = np.column_stack((
            np.linspace(10.0, 20.0, 100), np.ones(100), np.zeros(100)
        ))
        cloud = np.vstack((dense, sparse))
        chosen = spatial_voxel_sample_indices(cloud, 100)
        self.assertGreater(np.count_nonzero(chosen >= len(dense)), 10)


if __name__ == "__main__":
    unittest.main()
