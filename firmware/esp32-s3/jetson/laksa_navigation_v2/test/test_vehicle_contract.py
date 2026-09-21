import copy
import hashlib
import json
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from laksa_navigation_v2.generate_contract_artifacts import (
    AUDIT_PATH,
    CONTRACT_PATH,
    GENERATED_DIR,
    MANIFEST_PATH,
    URDF_DIR,
    ContractError,
    active_v2_geometry_divergences,
    checked_in_geometry_inputs,
    derived_values,
    generate,
    load_contract,
    validate_contract,
    validate_generated_geometry_inputs,
)


class VehicleContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = load_contract()
        self.values = derived_values(self.contract)

    def test_contract_schema_and_provenance(self):
        self.assertEqual(self.contract["contract_version"], 2)
        self.assertEqual(self.contract["steering"]["left_limit_rad"]["semantics"], "EQUIVALENT_BICYCLE_STEERING_ANGLE")
        self.assertEqual(self.contract["sensors"]["imu"]["provenance"]["classification"], "UNKNOWN")

    def test_directional_radii_are_derived_from_equivalent_bicycle_angles(self):
        self.assertTrue(math.isclose(self.values["geometric_radius_left_m"], 0.324 / math.tan(0.523), rel_tol=1e-12))
        self.assertTrue(math.isclose(self.values["geometric_radius_right_m"], 0.324 / math.tan(0.288), rel_tol=1e-12))
        self.assertEqual(self.values["planner_min_turning_radius_m"], self.values["geometric_radius_right_m"])

    def test_footprints_are_canonical_and_padded(self):
        self.assertEqual(self.values["canonical_footprint"], [[0.419, 0.148], [0.419, -0.148], [-0.149, -0.148], [-0.149, 0.148]])
        self.assertTrue(math.isclose(self.values["padded_footprint"][0][0], 0.439))
        self.assertTrue(math.isclose(abs(self.values["padded_footprint"][0][1]), 0.168))

    def test_negative_wheelbase_mismatch_fails(self):
        broken = copy.deepcopy(self.contract)
        broken["geometry"]["wheelbase_m"]["value"] = 0.0
        with self.assertRaises(ContractError):
            validate_contract(broken)

    def test_unsafe_planner_radius_policy_fails(self):
        broken = copy.deepcopy(self.contract)
        broken["kinematics"]["planner_radius_selection_policy"] = "MIN_DIRECTIONAL_RADIUS"
        with self.assertRaises(ContractError):
            validate_contract(broken)

    def test_invented_imu_fails(self):
        broken = copy.deepcopy(self.contract)
        broken["sensors"]["imu"]["xyz_m"] = [0.0, 0.0, 0.0]
        with self.assertRaises(ContractError):
            validate_contract(broken)

    def test_wrong_zed_pitch_sign_and_missing_lidar_fail(self):
        wrong_sign = copy.deepcopy(self.contract)
        wrong_sign["sensors"]["zed"]["rpy_rad"][1] *= -1
        with self.assertRaises(ContractError):
            validate_contract(wrong_sign)
        missing_lidar = copy.deepcopy(self.contract)
        del missing_lidar["sensors"]["lidar"]
        with self.assertRaises(ContractError):
            validate_contract(missing_lidar)

    def test_generator_is_deterministic_and_manifest_hashes_match(self):
        first = generate()
        digest_before = hashlib.sha256(MANIFEST_PATH.read_bytes()).hexdigest()
        second = generate()
        self.assertEqual(first, second)
        self.assertEqual(digest_before, hashlib.sha256(MANIFEST_PATH.read_bytes()).hexdigest())
        self.assertEqual(first["source_contract_sha256"], hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest())
        for relative, expected in first["output_sha256"].items():
            self.assertEqual(hashlib.sha256((MANIFEST_PATH.parent.parent / relative).read_bytes()).hexdigest(), expected)

    def test_urdf_consumes_generated_properties(self):
        wrapper = (URDF_DIR / "laksa_v2.urdf.xacro").read_text()
        self.assertIn("laksa_v2_geometry.generated.xacro", wrapper)
        self.assertIn("${wheelbase_m}", wrapper)
        generated = ET.parse(URDF_DIR / "laksa_v2_geometry.generated.xacro")
        values = {item.attrib["name"]: item.attrib["value"] for item in generated.findall("{http://www.ros.org/wiki/xacro}property")}
        self.assertEqual(float(values["wheelbase_m"]), self.values["wheelbase_m"])
        self.assertEqual(float(values["zed_pitch_rad"]), self.contract["sensors"]["zed"]["rpy_rad"][1])
        self.assertEqual(float(values["lidar_x_m"]), self.contract["sensors"]["lidar"]["xyz_m"][0])

    def test_generated_artifacts_and_zero_active_divergences(self):
        self.assertTrue((GENERATED_DIR / "nav2_footprints.json").is_file())
        audit = json.loads(AUDIT_PATH.read_text())
        self.assertEqual(active_v2_geometry_divergences(audit), [])
        self.assertTrue(MANIFEST_PATH.is_file())

    def test_generated_consumer_inputs_and_negative_divergences(self):
        artifacts = checked_in_geometry_inputs()
        self.assertEqual(validate_generated_geometry_inputs(self.values, artifacts), [])
        bad_simulator = copy.deepcopy(artifacts)
        bad_simulator["gazebo"]["wheelbase_m"] += 0.001
        self.assertIn("inconsistent_gazebo_wheelbase", validate_generated_geometry_inputs(self.values, bad_simulator))
        bad_footprint = copy.deepcopy(artifacts)
        bad_footprint["nav2"]["canonical_polygon_m"] = [[0.0, 0.0]] * 4
        self.assertIn("duplicated_or_inconsistent_nav2_footprint", validate_generated_geometry_inputs(self.values, bad_footprint))


if __name__ == "__main__":
    unittest.main()
