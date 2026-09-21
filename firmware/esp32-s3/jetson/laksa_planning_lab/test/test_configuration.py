from pathlib import Path

import yaml

from laksa_planning_lab.optimizer import candidate_configurations
from laksa_planning_lab.runtime_config import render


def test_runtime_configuration_is_isolated_and_overridden(tmp_path):
    share = Path(__file__).resolve().parents[1]
    output = tmp_path / "runtime.yaml"
    render(share, "HYBRID_RAW", 0.1, output, {"planner": {"reverse_penalty": 2.5}})
    data = yaml.safe_load(output.read_text())
    planner = data["/laksa_planning_lab/planner_server"]["ros__parameters"]["GridBased"]
    costmap = data["/laksa_planning_lab/global_costmap/global_costmap"]["ros__parameters"]
    assert planner["reverse_penalty"] == 2.5
    assert planner["smooth_path"] is False
    assert costmap["resolution"] == 0.1


def test_optimizer_is_seeded_and_contains_both_pipelines():
    search = Path(__file__).resolve().parents[1] / "config" / "search_space.yaml"
    first = candidate_configurations(search, 20)
    assert first == candidate_configurations(search, 20)
    assert {item["method"] for item in first} == {"HYBRID_RAW", "HYBRID_CONSTRAINED"}
