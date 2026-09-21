import math
import signal
import subprocess
from pathlib import Path

from laksa_planning_lab.map_dataset import MapData, _read_pgm, representative_cell_indices
from laksa_planning_lab.scenario_generator import SEED, SMOKE_STRATA
from laksa_planning_lab.worker_state import INFRASTRUCTURE_FAILURES, bounded_process_shutdown, classify_readiness_timeout, missing_readiness
from laksa_planning_lab.worker_state import failure_metrics, validation_result


def _map(cells, width, height, resolution=0.1):
    metadata = {
        "resolution": resolution, "origin": [0, 0, 0], "mode": "trinary",
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }
    pixels = [254 if cell == 0 else 205 if cell < 0 else 0 for cell in cells]
    return MapData("test", Path("map.yaml"), metadata, pixels, width, height)


def test_readiness_requires_active_lifecycle_and_all_endpoints():
    snapshot = {
        "map_server_active": True, "planner_server_active": False,
        "planner_action": True, "map_received": True,
        "costmap_received": True, "tf_ready": True,
    }
    assert missing_readiness(snapshot, False) == ["planner_server_active"]
    snapshot["planner_server_active"] = True
    assert missing_readiness(snapshot, False) == []
    assert missing_readiness(snapshot, True) == [
        "smoother_server_active", "smoother_action", "footprint_received", "base_link_tf_ready"
    ]


def test_startup_timeout_has_infrastructure_taxonomy():
    assert classify_readiness_timeout({}, False) == "STACK_READY_TIMEOUT"
    snapshot = {
        "map_server_active": True, "planner_server_active": True,
        "planner_action": True, "map_received": True,
        "costmap_received": True, "tf_ready": False,
    }
    assert classify_readiness_timeout(snapshot, False) == "TF_FAILURE"
    assert failure_metrics("STACK_READY_TIMEOUT")["failure_type"] == "STACK_READY_TIMEOUT"


def test_occupied_unknown_and_insufficient_clearance_are_rejected():
    cells = [0] * (15 * 15)
    cells[7 * 15 + 7] = 100
    cells[2 * 15 + 2] = -1
    data = _map(cells, 15, 15)
    assert not data.safe_center(*data.grid_to_world(7, 7))
    assert not data.safe_center(*data.grid_to_world(2, 2))
    assert not data.safe_center(*data.grid_to_world(8, 7))


def test_pgm_row_order_and_trinary_map_server_semantics(tmp_path):
    pgm = tmp_path / "map.pgm"
    pgm.write_bytes(b"P2\n2 2\n255\n0 205\n254 100\n")
    pixels, width, height = _read_pgm(pgm)
    assert (width, height) == (2, 2)
    assert pixels == [254, 100, 0, 205]


def test_binary_pgm_does_not_consume_whitespace_valued_first_pixel(tmp_path):
    pgm = tmp_path / "binary.pgm"
    pgm.write_bytes(b"P5\n2 1\n255\n" + bytes((10, 254)))
    pixels, width, height = _read_pgm(pgm)
    assert (width, height) == (2, 1)
    assert pixels == [10, 254]


def test_map_server_parity_samples_cover_all_present_semantics():
    cells = [-1, -1, 0, 0, 100, 100]
    sampled = {cells[index] for index in representative_cell_indices(cells)}
    assert sampled == {-1, 0, 100}


def test_oriented_rectangle_can_fit_where_circumscribed_circle_cannot():
    width = height = 60
    cells = [0] * (width * height)
    for mx in range(width):
        cells[25 * width + mx] = 100
    data = _map(cells, width, height, resolution=0.05)
    assert data.clearance_at(1.0, 1.0) < data.circumscribed_radius
    assert data.pose_collision_free(1.0, 1.0, 0.0)


def test_rectangle_collision_reports_unknown_cell_and_polygon():
    width = height = 60
    cells = [0] * (width * height)
    cells[20 * width + 21] = -1
    data = _map(cells, width, height, resolution=0.05)
    report = data.pose_collision_evidence(1.0, 1.0, 0.0)
    assert not report["collision_free"]
    assert report["offending_cells"][0]["occupancy"] == -1
    assert len(report["footprint_polygon"]) == 4


def test_map_origin_rotation_is_respected():
    data = _map([0] * 100, 10, 10, resolution=0.1)
    data.origin = (2.0, 3.0, math.pi / 2)
    world = data.grid_to_world(1, 2)
    assert data.world_to_grid(*world) == (1, 2)


def test_failure_precedence_and_unsupported_policy():
    primary, secondary = validation_result({
        "collision_valid": False, "kinematic_valid": False, "endpoint_valid": False,
    })
    assert primary == "COLLISION_FAILURE"
    assert secondary == ["KINEMATIC_VIOLATION", "ENDPOINT_ERROR"]
    assert "UNSUPPORTED" not in INFRASTRUCTURE_FAILURES


def test_smoother_config_has_canonical_tf_and_exact_footprint():
    root = Path(__file__).resolve().parents[1]
    launch = (root / "launch" / "planning_lab.launch.py").read_text(encoding="utf-8")
    config = (root / "config" / "hybrid_constrained.yaml").read_text(encoding="utf-8")
    assert '"base_footprint", "base_link"' in launch
    assert "robot_base_frame: base_footprint" in config
    assert "footprint_topic: /laksa_planning_lab/global_costmap/published_footprint" in config
    assert '[[0.419, 0.148], [0.419, -0.148], [-0.149, -0.148], [-0.149, 0.148]]' in config


def test_smoke_strata_are_balanced_and_seed_is_fixed():
    assert SEED == 2906
    assert len(SMOKE_STRATA) == 8
    assert ("FRONT", "NEAR", "OPEN_BASELINE") in SMOKE_STRATA
    assert ("FRONT", "MEDIUM", "OBSTACLE_DETOUR") in SMOKE_STRATA
    assert {item[0] for item in SMOKE_STRATA} == {"FRONT", "LATERAL_LEFT", "LATERAL_RIGHT", "BEHIND"}
    assert "VERY_NEAR" not in {item[1] for item in SMOKE_STRATA}


class _StubbornProcess:
    pid = 123

    def __init__(self):
        self.waits = 0
        self.done = False

    def poll(self):
        return 0 if self.done else None

    def wait(self, timeout):
        self.waits += 1
        if self.waits < 3:
            raise subprocess.TimeoutExpired("planner", timeout)
        self.done = True
        return 0


def test_worker_shutdown_is_bounded_and_escalates_only_as_needed():
    process = _StubbornProcess()
    sent = []
    signals = bounded_process_shutdown(process, lambda pid, value: sent.append((pid, value)))
    assert signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert process.poll() == 0


def test_worker_shutdown_tolerates_process_group_exit_race():
    process = _StubbornProcess()

    def already_gone(_pid, _value):
        raise ProcessLookupError

    assert bounded_process_shutdown(process, already_gone) == []


def test_intentional_teardown_is_marked_before_process_shutdown():
    source = (Path(__file__).resolve().parents[1] / "laksa_planning_lab" / "benchmark_runner.py").read_text(encoding="utf-8")
    close_body = source[source.index("    def close(self)"):source.index("    @property", source.index("    def close(self)"))]
    assert close_body.index("begin_shutdown()") < close_body.index("bounded_process_shutdown")
    assert "_record_failure" not in close_body
