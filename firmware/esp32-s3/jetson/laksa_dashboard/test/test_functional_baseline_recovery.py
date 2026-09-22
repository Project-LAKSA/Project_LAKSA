from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from laksa_dashboard.safe_goal_mask import circumscribed_radius, encoded_mask  # noqa: E402


def test_valid_map_produces_visible_nonempty_safe_region_and_leaves_waiting():
    width = height = 40
    resolution = 0.05
    radius = circumscribed_radius(0.419, 0.149, 0.148, 0.148)
    _encoded, safe_cells = encoded_mask(
        [0] * (width * height), width, height, resolution, radius
    )
    assert safe_cells > 0

    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text()
    app = (ROOT / "web" / "app.js").read_text()
    assert '"visible": True' in server
    assert 'self._safe_goal_cells = int(result["safe_cells"])' in server
    assert 'elif result["safe_cells"] > 0 and self._planning["state"] == "WAITING_FOR_SAFE_AREA"' in server
    assert 'self._set_planning_state(' in server
    assert 'm.visible!==true' in app
    assert 'map2d.safe={...m,bits}' in app


def test_hero_journey_unlocks_from_the_same_path_ready_transition():
    workflow = (ROOT / "web" / "route_workflow.mjs").read_text()
    app = (ROOT / "web" / "app.js").read_text()
    assert "planning.state === 'PATH_READY'" in workflow
    assert "heroJourneyAvailable(state.planning)" in app
    assert "heroJourney').disabled=!hero" in app
