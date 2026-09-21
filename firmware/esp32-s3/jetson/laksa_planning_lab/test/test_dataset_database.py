import json
from pathlib import Path

from laksa_planning_lab.database import ResultStore
from laksa_planning_lab.map_dataset import MapData
from laksa_planning_lab.scenario_generator import bearing_class, distance_class
from laksa_planning_lab.scenario_generator import load_scenarios, save_scenarios


def test_clearance_and_unknown_invalid():
    metadata = {"resolution": 0.1, "origin": [0, 0, 0], "negate": 0, "occupied_thresh": .65, "free_thresh": .196}
    pixels = [254] * 25
    pixels[12] = 0
    data = MapData("tiny", Path("map.yaml"), metadata, pixels, 5, 5)
    assert data.clearance_at(0.25, 0.25) == 0.0
    assert data.clearance_at(0.05, 0.05) > 0.0
    assert distance_class(0.75) == "VERY_NEAR"
    assert bearing_class(0.0) == "FRONT"


def test_sqlite_resume_and_configuration_hash(tmp_path):
    store = ResultStore(tmp_path / "lab.sqlite3")
    scenario = {"scenario_id": "S1", "map_id": "M1", "map_sha256": "abc", "distance_class": "NEAR", "bearing_class": "FRONT"}
    store.register_scenario(scenario)
    digest = store.register_configuration("HYBRID_RAW", {"method": "HYBRID_RAW"})
    assert not store.completed("abc", "S1", digest)
    store.record(scenario, "HYBRID_RAW", digest, {"planning_success": True, "collision_free": True,
                 "kinematically_feasible": True, "total_pipeline_time_ms": 1.0})
    assert store.completed("abc", "S1", digest)
    assert digest == store.register_configuration("HYBRID_RAW", {"method": "HYBRID_RAW"})


def test_failure_taxonomy_cannot_be_recorded_as_success(tmp_path):
    store = ResultStore(tmp_path / "lab.sqlite3")
    scenario = {"scenario_id": "S1", "map_id": "M1", "map_sha256": "abc", "distance_class": "NEAR", "bearing_class": "FRONT"}
    digest = store.register_configuration("HYBRID_RAW", {"method": "HYBRID_RAW"})
    store.record(scenario, "HYBRID_RAW", digest, {
        "planning_success": True, "collision_free": True,
        "kinematically_feasible": True, "failure_type": "ENDPOINT_ERROR",
    })
    success, failure_type = store.connection.execute(
        "SELECT success,failure_type FROM results"
    ).fetchone()
    assert success == 0
    assert failure_type == "ENDPOINT_ERROR"


def test_scenario_serialization(tmp_path):
    dataset = {"schema_version": 1, "seed": 2906, "scenarios": [{"scenario_id": "S1"}]}
    path = tmp_path / "scenarios.json"
    save_scenarios(dataset, path)
    assert load_scenarios(path) == dataset


def test_validation_artifacts_preserve_primary_and_secondary_evidence(tmp_path):
    store = ResultStore(tmp_path / "lab.sqlite3")
    scenario = {"scenario_id": "S1", "map_id": "M1", "map_sha256": "abc", "distance_class": "NEAR", "bearing_class": "FRONT"}
    digest = store.register_configuration("HYBRID_RAW", {"method": "HYBRID_RAW", "schema": "1.2"})
    store.record(scenario, "HYBRID_RAW", digest, {
        "planning_success": True, "planner_success": True, "smoother_success": None,
        "collision_free": False, "collision_valid": False,
        "kinematically_feasible": False, "kinematic_valid": False,
        "endpoint_valid": True, "failure_type": "COLLISION_FAILURE",
        "primary_result": "COLLISION_FAILURE",
        "secondary_diagnostic_flags": ["KINEMATIC_VIOLATION"],
        "collision_pose_index": 3,
        "collision_evidence": {"pose_index": 3, "x": 1.0, "y": 2.0, "yaw": 0.2,
                               "offending_cells": [{"mx": 4, "my": 5, "occupancy": -1}],
                               "footprint_polygon": [[0.0, 0.0]], "reason": "unknown"},
        "kinematic_violation_pose_index": 5,
        "max_abs_curvature_1pm": 1.2,
        "min_implied_radius_m": 0.833,
    })
    store.export_summaries(tmp_path)
    assert (tmp_path / "validation.csv").is_file()
    payload = json.loads((tmp_path / "validation.json").read_text())
    assert payload[0]["primary_result"] == "COLLISION_FAILURE"
    assert payload[0]["secondary_diagnostic_flags"] == ["KINEMATIC_VIOLATION"]
    assert payload[0]["collision_evidence"]["offending_cells"][0]["occupancy"] == -1
