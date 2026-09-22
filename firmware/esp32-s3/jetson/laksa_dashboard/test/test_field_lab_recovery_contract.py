from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def test_required_controls_and_production_copy_are_visible():
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    app = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    assert 'id="takeSnapshot"' in html
    assert 'id="toggleLidar"' in html
    assert "TAKE SNAPSHOT" in html
    assert not re.search(r"\bDiagnostics?\b", html, re.IGNORECASE)
    assert "Occupancy Cloud · Diagnostic" not in app


def test_2d_and_3d_robots_share_the_canonical_pose():
    app = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    apply_pose = app.split("function applyPose(m){", 1)[1].split("}\n", 1)[0]
    assert "robot.position.set(p[0],p[1],p[2])" in apply_pose
    assert "robot.quaternion.set(p[3],p[4],p[5],p[6])" in apply_pose
    assert "map2d.pose=p" in apply_pose
    assert 'Odometry, "/laksa/odometry/fused"' in (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text()


def test_safe_overlay_remains_continuous_without_lattice():
    app = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    assert "safeRaster" in app
    assert "drawRaster(ctx,map2d.safeImage" in app
    assert "candidateDots" not in app
    assert "nearestCandidate" not in app


def test_lidar_is_real_tf_projected_latest_only_and_navigation_visible():
    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text(encoding="utf-8")
    app = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    telemetry = (ROOT / "laksa_dashboard" / "telemetry_flow.py").read_text(encoding="utf-8")
    assert 'LaserScan, "/laksa/lidar/scan_validated"' in server
    assert 'lookup_transform(\n                "map", msg.header.frame_id' in server
    assert "Time.from_msg(msg.header.stamp)" in server
    assert '"source_frame": msg.header.frame_id' in server
    assert 'color:0xd946ef' in app
    assert 'size:.065' in app
    assert "lidarSlice.visible=lidarEnabled&&n>0" in app
    assert "lidarSlice.visible=active" not in app
    assert '"lidar_slice"' in telemetry


def test_frontend_has_runtime_identity_and_revalidation():
    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text(encoding="utf-8")
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    assert "__LAKSA_ASSET_VERSION__" in html
    assert '"Cache-Control"] = "no-cache, must-revalidate"' in server
    assert '"dashboard_build": self._build_identity' in server


def test_navigation_keeps_heavy_zed_cloud_disabled():
    server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text(encoding="utf-8")
    assert 'mapping.get("state") in ("STARTING", "MAPPING", "DEGRADED")' in server
    assert "self.destroy_subscription(self._zed_cloud_subscription)" in server
