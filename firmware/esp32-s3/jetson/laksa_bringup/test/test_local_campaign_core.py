import tempfile
import time
import unittest
from pathlib import Path

from local_campaign_core import CampaignState, assess_observation, required_topics


class LocalCampaignCoreTest(unittest.TestCase):
    def samples(self, count, hz=20, end=None):
        end = time.monotonic_ns() if end is None else end
        return [end - int((count - index - 1) * 1e9 / hz) for index in range(count)]

    def test_t21b_rejects_existing_but_silent_trajectory_topic(self):
        now = time.monotonic_ns()
        observations = {topic: self.samples(spec["minimum_samples"], end=now)
                        for topic, spec in required_topics("T21B").items()}
        observations["/laksa/odometry/fused"] = []  # endpoint can exist; no samples cannot arm
        ok, failures, _ = assess_observation(required_topics("T21B"), observations, freshness_s=.5, now_ns=now)
        self.assertFalse(ok)
        self.assertIn("INSUFFICIENT_SAMPLES:/laksa/odometry/fused", failures)

    def test_t21b_rejects_low_rate_trajectory(self):
        now = time.monotonic_ns()
        observations = {topic: self.samples(spec["minimum_samples"], end=now)
                        for topic, spec in required_topics("T21B").items()}
        observations["/laksa/odometry/fused"] = self.samples(10, hz=2, end=now)
        ok, failures, _ = assess_observation(required_topics("T21B"), observations, freshness_s=.5, now_ns=now)
        self.assertFalse(ok)
        self.assertIn("LOW_RATE:/laksa/odometry/fused", failures)

    def test_stale_safety_channel_rejects_before_motion(self):
        now = time.monotonic_ns()
        observations = {topic: self.samples(spec["minimum_samples"], end=now)
                        for topic, spec in required_topics("T23").items()}
        observations["/joy"] = self.samples(3, end=now - 2_000_000_000)
        ok, failures, _ = assess_observation(required_topics("T23"), observations, freshness_s=.5, now_ns=now)
        self.assertFalse(ok)
        self.assertIn("STALE:/joy", failures)

    def test_stale_active_state_is_recovered_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            state = CampaignState(Path(directory) / "state.json")
            state.transition("T21B:left", "MOTION")
            value = state.load()
            self.assertIsNone(value["active"])
            self.assertEqual("BLOCKED", value["tests"]["T21B:left"]["state"])


if __name__ == "__main__":
    unittest.main()
