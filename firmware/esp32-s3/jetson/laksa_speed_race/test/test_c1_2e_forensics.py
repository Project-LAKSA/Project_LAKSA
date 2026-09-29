"""C1.2e passive-instrumentation and handoff regressions."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HANDOFF = _load("c1_2e_forensic_handoff", ROOT / "tools" / "c1_2e_forensic_handoff.py")
INSTRUMENT = _load(
    "instrument_nav2_mppi_forensics",
    ROOT / "docker" / "instrument_nav2_mppi_forensics.py",
)


def _write_csv(path: Path, row: dict[str, object]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)


def _dynamic_fixture(root: Path, *, x: str, stamp: str) -> None:
    root.mkdir()
    _write_csv(
        root / "controller_telemetry.csv",
        {
            "sequence": "1",
            "x_m": x,
            "y_m": "2",
            "yaw_rad": "0",
            "actual_velocity_mps": "0.2",
            "progress_index": "3",
            "path_copy": "0",
            "kappa_req_1pm": "0.1",
            "kappa_max_1pm": "0.9",
            "kappa_cmd_1pm": "0.1",
            "upstream_linear_mps": "0.2",
            "omega_pre_feasibility_rps": "0.02",
            "upstream_angular_rps": "0.02",
            "delta_equivalent_rad": "0.032",
            "delta_raw_rad": "0.032",
            "delta_applied_rad": "0.032",
            "curvature_saturated": "false",
            "downstream_steering_saturated": "false",
            "physical_feasibility": "true",
            "safety_veto_pass": "true",
            "state_stamp_ns": stamp,
            "controller_latency_ms": "99",
        },
    )
    _write_csv(
        root / "trajectory.csv",
        {
            "step": "1",
            "sim_time_s": "0.01",
            "x_m": x,
            "y_m": "2",
            "yaw_rad": "0",
            "actual_speed_mps": "0.2",
            "signed_cte_m": "0.3",
            "heading_error_rad": "0.01",
            "collision": "0",
            "off_track": "0",
            "full_body_clearance_m": "0.2",
            "lap_count": "0",
            "wall_clock_ns": stamp,
        },
    )
    _write_csv(
        root / "commands.csv",
        {
            "step": "1",
            "sim_time_s": "0.01",
            "requested_steering_rad": "0.032",
            "requested_speed_mps": "0.2",
            "applied_steering_rad": "0.032",
            "applied_speed_mps": "0.2",
            "mission_state": "RUNNING",
            "process_id": stamp,
        },
    )


def test_dynamic_hash_excludes_only_declared_nondeterministic_metadata(tmp_path: Path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    changed = tmp_path / "changed"
    _dynamic_fixture(first, x="1", stamp="100")
    _dynamic_fixture(second, x="1", stamp="200")
    _dynamic_fixture(changed, x="1.0000001", stamp="100")
    assert HANDOFF.dynamic_hash(first) == HANDOFF.dynamic_hash(second)
    assert HANDOFF.dynamic_hash(first) != HANDOFF.dynamic_hash(changed)


def test_frozen_raceline_unrolls_to_expected_finite_path():
    raceline = HANDOFF.load_unrolled_raceline(
        ROOT / "course" / "canonical" / "speed_course" / "pure_pursuit_raceline.csv"
    )
    assert len(raceline) == 2185
    assert HANDOFF.path_length(raceline) == pytest.approx(436.5201662822104, abs=1e-12)


def test_instrumentation_rejects_unknown_upstream_source(tmp_path: Path):
    source = tmp_path / "nav2_mppi_controller" / "src" / "critic_manager.cpp"
    source.parent.mkdir(parents=True)
    source.write_text("// not the pinned Humble source\n", encoding="utf-8")
    with pytest.raises(SystemExit, match="unexpected pinned CriticManager source SHA256"):
        INSTRUMENT.apply(tmp_path)


def test_reference_metrics_remain_frozen():
    assert HANDOFF.REFERENCE["off_track_step"] == 1326
    assert HANDOFF.REFERENCE["cte_final"] == 0.512046343006913
    assert HANDOFF.REFERENCE["min_clearance"] == -0.00007242838671644991
