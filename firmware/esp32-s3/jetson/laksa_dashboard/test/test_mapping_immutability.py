from hashlib import sha256
from pathlib import Path
import unittest


REPO = Path(__file__).resolve().parents[2]
EXPECTED = {
    "laksa_mapping/config/rtabmap_common.yaml": "60a10a0bea93f9d8e6982997a998d60a9e5e6052c8ca7aea4220c7791a0946ae",
    "laksa_mapping/config/rtabmap_fused.yaml": "80cbfe5d349b08af493cea4ca1cf0ee02a8c0a8aeeaeb2346c5266057b1cbdd7",
    "laksa_mapping/config/indoor_live_zed.yaml": "aae8a161eea00f556e6f9b519cd9d68834613513fb26ac7c9cdc445af141ab15",
    "laksa_mapping/config/profiles.yaml": "f336f553d7c789be5b8da565856d278c56982b79c55008306bf620f3eef76d0b",
    "laksa_mapping/launch/mapping_stack.launch.py": "d6d60c79769e47c1ca5c109ce4d029bcfbf845556205afe0ae5c695926439b3b",
    "laksa_mapping/laksa_mapping/session_manager.py": "ddf689d984087b705d9af840d90c2b6077b3a5c83bd37a8314985e2d39d27c2a",
    "laksa_lidar/config/a2m12.yaml": "7bbae168f8dc5402c663dd8a2d64d29866773090c67c00814e60b82bfd8a51d2",
    "laksa_lidar/launch/lidar_guard.launch.py": "46db4d9136865282dee8d48bfafbae1e1fc85fe38c04230abba76ef407dc0f35",
}


class MappingImmutabilityTest(unittest.TestCase):
    def test_mapping_inputs_and_behavior_are_bit_identical(self):
        for relative, expected in EXPECTED.items():
            actual = sha256((REPO / relative).read_bytes()).hexdigest()
            self.assertEqual(actual, expected, relative)


if __name__ == "__main__":
    unittest.main()
