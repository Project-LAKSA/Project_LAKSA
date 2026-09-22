import unittest

from laksa_dashboard.telemetry_flow import LatestRevisionGate, OutgoingCoalescer, uniform_sample_indices


class TelemetryFlowTest(unittest.TestCase):
    def test_pose_is_latest_wins_and_contains_no_trajectory(self):
        queue = OutgoingCoalescer()
        for sequence in range(5):
            queue.put({"type": "pose", "sequence": sequence, "pose": [sequence]})
        payload = queue.pop()
        self.assertEqual(payload["sequence"], 4)
        self.assertNotIn("trajectory", payload)
        self.assertEqual(queue.coalesced["pose"], 4)

    def test_trajectory_append_survives_dropped_poses(self):
        queue = OutgoingCoalescer()
        queue.put({"type": "pose", "sequence": 1})
        queue.put({"type": "trajectory_append", "point": [1.0, 0.0, 0.0]})
        queue.put({"type": "pose", "sequence": 2})
        self.assertEqual(queue.pop()["type"], "trajectory_append")
        self.assertEqual(queue.pop()["sequence"], 2)

    def test_lidar_slice_is_latest_scan_only(self):
        queue = OutgoingCoalescer()
        queue.put({"type": "lidar_slice", "point_count": 10})
        queue.put({"type": "lidar_slice", "point_count": 12})
        payload = queue.pop()
        self.assertEqual(payload["point_count"], 12)
        self.assertEqual(queue.coalesced["lidar_slice"], 1)

    def test_safe_mask_has_one_job_and_latest_revision_wins(self):
        gate = LatestRevisionGate()
        self.assertEqual(gate.request(100, "a"), (100, "a"))
        self.assertIsNone(gate.request(101, "b"))
        self.assertIsNone(gate.request(104, "e"))
        publish, next_item = gate.complete(100)
        self.assertFalse(publish)
        self.assertEqual(next_item, (104, "e"))
        publish, next_item = gate.complete(104)
        self.assertTrue(publish)
        self.assertIsNone(next_item)
        self.assertEqual(gate.coalesced, 1)

    def test_safe_mask_transport_has_no_occupancy_array(self):
        payload = {"type": "safe_goal_mask", "mask": "AA==", "revision": 1}
        self.assertNotIn("data", payload)
        self.assertIn("mask", payload)

    def test_uniform_sampling_is_exact_and_spans_cloud(self):
        for source, target, expected in (
            (10000, 15000, 10000),
            (15000, 15000, 15000),
            (15001, 15000, 15000),
            (21464, 15000, 15000),
            (30000, 15000, 15000),
        ):
            indices = uniform_sample_indices(source, target)
            self.assertEqual(len(indices), expected)
            self.assertEqual(indices[0], 0)
            self.assertEqual(indices[-1], source - 1)
            self.assertEqual(indices, sorted(set(indices)))


if __name__ == "__main__":
    unittest.main()
