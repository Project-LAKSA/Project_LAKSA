import math

from laksa_planning_lab.geometry import R_LEFT_M, R_RIGHT_M
from laksa_planning_lab.lattice_tools import validate_lattice


def _tiny_lattice():
    return {"version": "test", "date_generated": "1970-01-01",
            "lattice_metadata": {"motion_model": "ackermann", "turning_radius": R_RIGHT_M,
                                 "grid_resolution": .05, "num_of_headings": 8,
                                 "heading_angles": [i * math.pi / 4 for i in range(8)], "number_of_trajectories": 1},
            "primitives": [{"trajectory_id": 0, "start_angle_index": 0, "end_angle_index": 0, "left_turn": True,
                            "trajectory_radius": 0.0, "trajectory_length": .1, "arc_length": 0.0,
                            "straight_length": .1, "poses": [[0, 0, 0], [.1, 0, 0]]}]}


def test_json_structure_and_primitive_validation_is_deterministic():
    data = _tiny_lattice()
    assert validate_lattice(data, R_LEFT_M, R_RIGHT_M)["valid"]
    assert validate_lattice(data, R_LEFT_M, R_RIGHT_M) == validate_lattice(data, R_LEFT_M, R_RIGHT_M)
