import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "laksa_description/urdf/laksa_visualization.urdf"
ACTIVE_LAUNCH = ROOT / "laksa_bringup/launch/lidar_mapping.launch.py"
MAPPING_LAUNCH = ROOT / "laksa_mapping/launch/mapping_stack.launch.py"


class CanonicalLidarTfTest(unittest.TestCase):
    def test_measured_urdf_extrinsics(self):
        root = ET.parse(DESCRIPTION).getroot()
        joint = next(node for node in root.findall("joint") if node.attrib.get("name") == "base_to_lidar")
        self.assertEqual(joint.find("parent").attrib["link"], "base_footprint")
        self.assertEqual(joint.find("child").attrib["link"], "lidar_link")
        xyz = [float(value) for value in joint.find("origin").attrib["xyz"].split()]
        rpy = [float(value) for value in joint.find("origin").attrib["rpy"].split()]
        for actual, expected in zip(xyz, (0.31542, 0.0, 0.13542)):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(rpy, (0.0, 0.0, math.pi)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_active_driver_is_canonical_and_legacy_tf_is_opt_in(self):
        source = ACTIVE_LAUNCH.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('default_value="lidar_link"', source)
        self.assertIn('"legacy_lidar_tf",\n                default_value="false"', source)
        self.assertIn("condition=IfCondition(legacy_lidar_tf)", source)
        self.assertIn('"base_frame_id": "base_footprint"', source)
        self.assertNotIn("UnlessCondition", source)

    def test_mapping_stack_does_not_start_second_robot_description(self):
        source = MAPPING_LAUNCH.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn("description.launch.py", source)
        self.assertNotIn("robot_state_publisher", source)

    def test_field_lab_is_the_single_normal_description_owner(self):
        source = (ROOT / "laksa_dashboard/launch/cockpit.launch.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertEqual(source.count("description.launch.py"), 1)


if __name__ == "__main__":
    unittest.main()
