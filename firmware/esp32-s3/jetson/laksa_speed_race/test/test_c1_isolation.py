"""Static C1 ROS/container physical-isolation contract."""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class IsolationTests(unittest.TestCase):
    def test_launch_and_config_use_only_c1_motion_topics(self):
        sources = [
            ROOT / "launch" / "c1_three_lap.launch.py",
            ROOT / "config" / "c1_pure_pursuit.yaml",
            ROOT / "docker-compose.c1.yaml",
        ]
        forbidden = ("/drive", "/cmd_vel", "/laksa/command", "/laksa/set_drive_command")
        for source in sources:
            text = source.read_text()
            for topic in forbidden:
                self.assertIsNone(re.search(rf"(?<![A-Za-z0-9_]){re.escape(topic)}(?![A-Za-z0-9_])", text), source)
        controller = (ROOT / "config" / "c1_pure_pursuit.yaml").read_text()
        self.assertIn("/c1/drive_request", controller)
        self.assertIn("/c1/odom", controller)

    def test_runtime_container_has_no_host_or_device_authority(self):
        compose = (ROOT / "docker-compose.c1.yaml").read_text()
        self.assertGreaterEqual(compose.count("network_mode: none"), 2)
        self.assertGreaterEqual(compose.count("privileged: false"), 2)
        self.assertNotIn("/dev/", compose)
        self.assertNotIn("/home/ubuntu/laksa_ws", compose)
        self.assertNotIn("/var/run/docker.sock", compose)
        self.assertNotIn("network_mode: host", compose)


if __name__ == "__main__":
    unittest.main()
