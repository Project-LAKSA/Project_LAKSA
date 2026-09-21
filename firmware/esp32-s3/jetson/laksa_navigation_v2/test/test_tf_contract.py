import json, math, unittest, xml.etree.ElementTree as ET
from pathlib import Path
from laksa_navigation_v2.tf_contract import Edge, validate

ROOT = Path(__file__).resolve().parents[1]
class TfContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = json.loads((ROOT / "config/vehicle_contract.yaml").read_text())
        self.tree = ET.parse(ROOT / "urdf/laksa_v2.urdf.xacro")
    def edges(self):
        return [Edge(j.find("parent").attrib["link"], j.find("child").attrib["link"], "robot_state_publisher") for j in self.tree.findall("joint")]
    def test_contract_provenance(self):
        self.assertEqual(self.contract["sensors"]["imu"]["provenance"], "UNKNOWN")
        self.assertGreater(self.contract["geometry"]["wheelbase_m"]["value"], 0)
    def test_missing_imu_extrinsic_is_explicit_not_invented(self):
        tf = json.loads((ROOT / "config/tf_authority_contract.json").read_text())
        self.assertEqual(self.contract["sensors"]["imu"]["xyz_m"], None)
        self.assertIn("base_link->imu_link", tf["unmeasured_edges"])
        self.assertIsNone(self.tree.find("joint[@name='base_to_imu']"))
    def test_tree_and_root(self):
        self.assertEqual(validate(self.edges()), [])
        self.assertNotIn("zed_camera_link", [edge.parent for edge in self.edges()])
    def test_camera_pitch_sign(self):
        pitch = float(self.tree.find("joint[@name='base_to_zed']").find("origin").attrib["rpy"].split()[1])
        self.assertGreater(pitch, 0)
        self.assertTrue(math.isclose(pitch, self.contract["sensors"]["zed"]["rpy_rad"][1], abs_tol=1e-9))
    def test_missing_sensor(self):
        self.assertIn("missing_sensor_edge:lidar_link", validate([Edge("base_link", "zed_camera_link", "rsp")]))
    def test_duplicate_authority(self):
        self.assertIn("duplicate_child:zed_camera_link", validate(self.edges() + [Edge("map", "zed_camera_link", "bad")]))
    def test_loop_and_wrong_root(self):
        errors = validate([Edge("base_link", "zed_camera_link", "rsp"), Edge("zed_camera_link", "base_link", "bad"), Edge("base_link", "lidar_link", "rsp")])
        self.assertIn("wrong_root:base_link", errors); self.assertTrue(any(e.startswith("loop:") for e in errors))
    def test_complete_rep105_tree_has_one_authority_per_edge(self):
        tf = json.loads((ROOT / "config/tf_authority_contract.json").read_text())
        edges = [Edge(**{key: entry[key] for key in ("parent", "child", "authority")}) for entry in tf["edges"]]
        self.assertEqual(validate(edges, root=tf["root"]), [])
        self.assertEqual(len({(e.parent, e.child) for e in edges}), len(edges))

if __name__ == "__main__": unittest.main()
