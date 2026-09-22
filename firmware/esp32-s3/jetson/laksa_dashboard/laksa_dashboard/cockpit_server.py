#!/usr/bin/env python3
"""Field Lab telemetry and safe mapping/navigation lifecycle gateway."""

from __future__ import annotations

import asyncio
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from aiohttp import web
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import Costmap
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml
import numpy as np

from .safe_goal_mask import circumscribed_radius, edt_backend, encoded_mask
from .lidar_projection import project_scan
from .snapshot_workflow import SnapshotError, create_snapshot
from .telemetry_flow import LatestRevisionGate, OutgoingCoalescer, uniform_sample_indices


FUSED_MAPPING = "FUSED_MAPPING"
MAPPING_SOURCES = (FUSED_MAPPING,)


def _dashboard_build_identity(web_root: Path) -> dict:
    """Identify the exact static tree served by this process."""
    assets = ("index.html", "app.js", "style.css", "planning.css", "lidar.css",
              "geometry.mjs", "telemetry.mjs", "route_workflow.mjs")
    digest = hashlib.sha256()
    newest_mtime = 0.0
    for name in assets:
        path = web_root / name
        if path.is_file():
            data = path.read_bytes()
            digest.update(name.encode("utf-8")); digest.update(b"\0"); digest.update(data)
            newest_mtime = max(newest_mtime, path.stat().st_mtime)
    commit = os.environ.get("LAKSA_GIT_COMMIT", "")
    if not commit:
        for repository in (Path("/home/ubuntu/src/Project_LAKSA"), Path(__file__).resolve().parents[5]):
            try:
                commit = subprocess.run(
                    ["git", "-C", str(repository), "rev-parse", "--short=12", "HEAD"],
                    check=True, capture_output=True, text=True, timeout=2,
                ).stdout.strip()
                if commit:
                    break
            except (OSError, subprocess.SubprocessError):
                continue
    return {
        "asset_hash": digest.hexdigest(), "build_id": digest.hexdigest()[:12],
        "git_commit": commit or "unknown",
        "build_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(newest_mtime)),
        "package_prefix": str(web_root.parent.parent),
    }


def _transform_xyz(points: np.ndarray, translation, quaternion) -> np.ndarray:
    """Apply a ROS rigid transform to an Nx3 point array."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("cloud transform quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.asarray((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    ), dtype=np.float64)
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def _compose_transforms(first, second) -> tuple[list[float], list[float]]:
    """Compose map->odom and odom->base without forcing a stale common time."""
    b_t = second.transform.translation; b_q = second.transform.rotation
    return _compose_transform_pose(first, b_t, b_q)


def _compose_transform_pose(first, b_t, b_q) -> tuple[list[float], list[float]]:
    """Compose a stamped map->odom transform with an odometry pose."""
    a_t = first.transform.translation; a_q = first.transform.rotation
    bx_t, by_t, bz_t = (
        (b_t.x, b_t.y, b_t.z) if hasattr(b_t, "x") else b_t
    )
    bx, by, bz, bw = (
        (b_q.x, b_q.y, b_q.z, b_q.w) if hasattr(b_q, "x") else b_q
    )
    rotated = _transform_xyz(
        np.asarray(((bx_t, by_t, bz_t),), dtype=np.float64),
        (a_t.x, a_t.y, a_t.z), (a_q.x, a_q.y, a_q.z, a_q.w),
    )[0]
    ax, ay, az, aw = a_q.x, a_q.y, a_q.z, a_q.w
    quaternion = [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
    return rotated.tolist(), quaternion


def _read_official_ply_preview(path: Path, max_points: int) -> dict:
    """Read a bounded preview from the binary PLY written by rtabmap-export."""
    scalar_types = {
        "char": "i1", "uchar": "u1", "int8": "i1", "uint8": "u1",
        "short": "<i2", "ushort": "<u2", "int16": "<i2", "uint16": "<u2",
        "int": "<i4", "uint": "<u4", "int32": "<i4", "uint32": "<u4",
        "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
    }
    vertex_count = 0
    vertex_properties = []
    in_vertices = False
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        if stream.readline().strip() != b"format binary_little_endian 1.0":
            raise ValueError("only official binary_little_endian PLY is supported")
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("PLY header is incomplete")
            line = raw.decode("ascii", "strict").strip()
            fields = line.split()
            if fields[:2] == ["element", "vertex"]:
                vertex_count = int(fields[2])
                in_vertices = True
            elif fields and fields[0] == "element":
                in_vertices = False
            elif in_vertices and fields[:1] == ["property"] and len(fields) == 3:
                if fields[1] not in scalar_types:
                    raise ValueError(f"unsupported PLY scalar type {fields[1]}")
                vertex_properties.append((fields[2], scalar_types[fields[1]]))
            if line == "end_header":
                data_offset = stream.tell()
                break
    if vertex_count <= 0 or not all(
        name in {item[0] for item in vertex_properties} for name in ("x", "y", "z")
    ):
        raise ValueError("PLY has no XYZ vertices")
    cloud = np.memmap(
        path, mode="r", dtype=np.dtype(vertex_properties),
        offset=data_offset, shape=(vertex_count,),
    )
    finite = np.isfinite(cloud["x"]) & np.isfinite(cloud["y"]) & np.isfinite(cloud["z"])
    finite_indices = np.flatnonzero(finite)
    chosen = uniform_sample_indices(len(finite_indices), max_points)
    selected = finite_indices[np.asarray(chosen, dtype=np.intp)] if chosen else np.empty(0, dtype=np.intp)
    xyz = np.column_stack((cloud["x"][selected], cloud["y"][selected], cloud["z"][selected]))
    points = np.round(xyz.astype(np.float64, copy=False), 3).reshape(-1).tolist()
    names = cloud.dtype.names or ()
    if all(name in names for name in ("red", "green", "blue")):
        colors_array = np.column_stack(
            (cloud["red"][selected], cloud["green"][selected], cloud["blue"][selected])
        ).astype(np.float64) / 255.0
    else:
        colors_array = np.tile(np.asarray((0.25, 0.75, 1.0)), (len(selected), 1))
    return {
        "type": "cloud", "frame": "map", "source": "final_optimized_rgbd_map",
        "source_points": int(vertex_count), "sent_points": int(len(selected)),
        "downsampled": len(selected) < vertex_count,
        "timestamp": path.stat().st_mtime, "payload_source_bytes": path.stat().st_size,
        "points": points,
        "colors": np.round(colors_array, 3).reshape(-1).tolist(),
    }


def _compute_safe_mask(request: dict) -> dict:
    started = time.monotonic()
    mask_base64, safe_cells = encoded_mask(
        request["data"], request["width"], request["height"], request["resolution"], request["radius"]
    )
    return {
        "revision": request["revision"],
        "mask": mask_base64,
        "safe_cells": safe_cells,
        "compute_ms": (time.monotonic() - started) * 1000.0,
        "backend": edt_backend(),
    }


class CockpitServer(Node):
    def __init__(self) -> None:
        super().__init__("laksa_mapping_cockpit")
        self.declare_parameter("port", 8090)
        self.declare_parameter("max_cloud_points", 15000)
        self.declare_parameter("cloud_period_sec", 1.0)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("snapshots_root", "/home/ubuntu/laksa_maps/snapshots")
        self._port = int(self.get_parameter("port").value)
        self._max_points = int(self.get_parameter("max_cloud_points").value)
        self._cloud_period = float(self.get_parameter("cloud_period_sec").value)
        self._web_root = Path(get_package_share_directory("laksa_dashboard")) / "web"
        self._build_identity = _dashboard_build_identity(self._web_root)
        default_geometry = Path(get_package_share_directory("laksa_dashboard")) / "config" / "planning_geometry.yaml"
        self.declare_parameter("planning_geometry_path", str(default_geometry))
        self._pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self._snapshots_root = Path(str(self.get_parameter("snapshots_root").value))
        self._planning_geometry = self._load_planning_geometry(Path(str(self.get_parameter("planning_geometry_path").value)))
        self._loop = asyncio.new_event_loop()
        self._ws_clients = set()
        self._pose_ws_clients = set()
        self._websocket_client_count = 0
        self._pose_websocket_client_count = 0
        self._outgoing = OutgoingCoalescer()
        self._drain_task = None
        self._latest_pose_outgoing = None
        self._pose_drain_task = None
        initial_planning_state = "WAITING_FOR_MAP" if self._planning_geometry["calibrated"] else "UNCALIBRATED"
        self._planning = {"state": initial_planning_state, "preview_only": True, "heading_mode": "AUTO_HEADING", "planning_time_ms": None, "path_length_m": None, "message": self._planning_geometry["message"]}
        self._latest = {
            "mapping": {"state": "IDLE", "mapping_source": FUSED_MAPPING, "fusion_state": "IDLE"},
            "health": {}, "planning": self._planning,
            "navigation": {"state": "IDLE", "mode": "MAPPING", "snapshot": None},
            "field_lab_latency": {},
        }
        self._last_cloud = 0.0
        self._last_lidar_slice = 0.0
        self._last_grid = 0.0
        self._grid_received = 0.0
        self._grid = None
        self._planning_grid = None
        self._safe_mask_payload = None
        self._safe_goal_cells = None
        self._safe_mask_gate = LatestRevisionGate()
        self._safe_mask_worker = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context("spawn")
        )
        self._grid_revision = 0
        self._visual_grid_revision = 0
        self._trajectory = []
        self._last_pose = None
        self._latest_fused_odom_pose = None
        self._last_trajectory_pose = None
        self._pose_received = 0.0
        self._pose_sequence = 0
        self._active_session_id = ""
        self._final_cloud_payload = None
        self._loaded_final_session = ""
        self._planning_token = 0
        self._planning_request = None
        self._active_planner_goal = None
        self._planned_path = None
        self._follow_token = 0
        self._active_follow_goal = None
        self._follow_starting = False
        self._planner_active = False
        self._controller_active = False
        self._autonomy_armed = False
        self._autonomy_health = "UNKNOWN"
        self._controller_state_future = None
        self._planner_state_future = None
        self._local_costmap_received = 0.0
        self._navigation_process = None
        self._navigation_kind = "NONE"
        self._navigation_log = None
        self._live_navigation_task = None
        self._snapshot_task = None
        self._initial_pose = None
        self._amcl_pose_received = 0.0
        self._amcl_pose_sequence = 0
        self._latest_amcl_pose = None
        self._nav_state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("map_server", "amcl", "planner_server", "controller_server")
        }
        self._lock = threading.RLock()
        self._perf = {
            "safe_mask_revision": 0,
            "safe_mask_compute_ms": 0.0,
            "safe_mask_backend": "pending",
            "cloud_source_points": 0,
            "cloud_sent_points": 0,
            "cloud_extract_ms": 0.0,
            "cloud_sample_ms": 0.0,
            "cloud_pack_ms": 0.0,
            "cloud_payload_bytes": 0,
            "cloud_source": "occupancy_cloud",
            "cloud_rendered_points": 0,
        }
        qos = QoSProfile(depth=1); qos.reliability = ReliabilityPolicy.RELIABLE; qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/laksa/mapping/state", self._mapping_cb, qos)
        self.create_subscription(String, "/laksa/health/summary", self._health_cb, qos)
        # Cloud subscriptions are demand-driven.  RTAB-Map's MapsManager builds
        # assembled clouds for subscribers, so keeping this subscription while
        # no Field Lab client is present would couple visualization load back
        # into mapping quality.
        self._global_cloud_subscription = None
        # The registered ZED cloud is a multi-megabyte live-mapping preview.
        # Saved-map navigation renders the accepted static PLY and must not pay
        # Python PointCloud2 deserialization cost for data it will discard.
        self._zed_cloud_subscription = None
        self.create_subscription(
            OccupancyGrid, "/map", self._grid_cb, 10,
        )
        self.create_subscription(
            LaserScan, "/laksa/lidar/scan_validated",
            self._lidar_scan_cb, qos_profile_sensor_data,
        )
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._planning_grid_cb, qos)
        self.create_subscription(Costmap, "/local_costmap/costmap_raw", self._local_costmap_cb, 10)
        self._profile_pub = self.create_publisher(String, "/laksa/mapping/profile_request", 10)
        self._source_pub = self.create_publisher(String, "/laksa/mapping/source_request", 10)
        self._record_pub = self.create_publisher(Bool, "/laksa/mapping/record_svo_request", 10)
        self._start_client = self.create_client(Trigger, "/laksa/mapping/start")
        self._stop_client = self.create_client(Trigger, "/laksa/mapping/stop")
        self._planner_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self._follow_client = ActionClient(self, FollowPath, "/follow_path")
        self._planner_state_client = self._nav_state_clients["planner_server"]
        self._controller_state_client = self._nav_state_clients["controller_server"]
        self._autonomy_arm_client = self.create_client(SetBool, "/laksa/autonomy/set_armed")
        self._preview_path_pub = self.create_publisher(NavigationPath, "/laksa/planning/preview_path", 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )
        self.create_subscription(Empty, "/laksa/cancel_navigation", self._cancel_navigation_cb, 10)
        self.create_subscription(Bool, "/laksa/autonomous_enabled", self._autonomy_mode_cb, qos)
        self.create_subscription(String, "/laksa/autonomy_health", self._autonomy_health_cb, qos)
        self._tf_buffer = Buffer(); self._tf_listener = TransformListener(self._tf_buffer, self)
        # Heavy PointCloud2 conversion shares the node's default mutually-exclusive
        # callback group.  Keep the small authoritative odometry callback independent
        # so Field Lab cannot display an old TF-buffer sample while ROS odometry is fresh.
        self._pose_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Odometry, "/laksa/odometry/fused", self._fused_odom_cb, 10,
            callback_group=self._pose_callback_group,
        )
        self.create_timer(0.5, self._planning_tick)
        self.create_timer(0.5, self._update_cloud_subscriptions)
        self.create_timer(5.0, self._performance_tick)
        self._thread = threading.Thread(target=self._run_web, daemon=True); self._thread.start()

    @staticmethod
    def _load_planning_geometry(path: Path) -> dict:
        result = {"calibrated": False, "message": "LAKSA navigation geometry is not calibrated", "path": str(path)}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            geometry = data.get("planning_geometry", {})
        except (OSError, yaml.YAMLError):
            return result
        required = ("wheelbase_m", "front_m", "rear_m", "left_m", "right_m")
        values = [geometry.get(name) for name in required]
        direct_radius = geometry.get("minimum_turning_radius_m")
        steering_deg = geometry.get("effective_max_road_wheel_steering_deg")
        numeric = lambda value: isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) and float(value) > 0.0
        radius_source_valid = numeric(direct_radius) or (numeric(steering_deg) and float(steering_deg) < 90.0)
        calibrated = geometry.get("calibrated") is True and all(numeric(value) for value in values) and radius_source_valid
        result.update({"calibrated": calibrated, "values": geometry})
        if calibrated:
            result["message"] = "Geometry calibrated"
        return result

    def _run_web(self) -> None:
        asyncio.set_event_loop(self._loop)
        @web.middleware
        async def cache_identity(request, handler):
            response = await handler(request)
            if request.path == "/" or request.path.startswith(("/static/", "/vendor/")):
                response.headers["Cache-Control"] = "no-cache, must-revalidate"
            response.headers["X-LAKSA-Dashboard-Build"] = self._build_identity["build_id"]
            return response

        app = web.Application(client_max_size=2 * 1024 * 1024, middlewares=[cache_identity])
        app.router.add_get("/", self._index)
        app.router.add_get("/api/state", self._api_state)
        app.router.add_post("/api/telemetry/pose-latency", self._api_pose_latency)
        app.router.add_post("/api/mapping/start", self._api_start)
        app.router.add_post("/api/mapping/stop", self._api_stop)
        app.router.add_post("/api/snapshot/take", self._api_take_snapshot)
        app.router.add_post("/api/planning/preview", self._api_preview_path)
        app.router.add_post("/api/planning/clear", self._api_clear_path)
        app.router.add_post("/api/planning/follow", self._api_follow_path)
        app.router.add_post("/api/planning/cancel_follow", self._api_cancel_follow)
        app.router.add_get("/ws", self._ws)
        app.router.add_get("/ws/pose", self._pose_ws)
        # Colcon --symlink-install deliberately exposes package assets through
        # symlinks; aiohttp must be allowed to follow only within these roots.
        app.router.add_static("/static", self._web_root, follow_symlinks=True)
        app.router.add_static("/vendor", self._web_root / "vendor", follow_symlinks=True)
        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, "0.0.0.0", self._port)
        self._loop.run_until_complete(site.start())
        self.get_logger().info(f"LAKSA Field Lab listening on http://0.0.0.0:{self._port}")
        self._loop.run_forever()

    async def _index(self, _request):
        html = (self._web_root / "index.html").read_text(encoding="utf-8")
        html = html.replace("__LAKSA_ASSET_VERSION__", self._build_identity["build_id"])
        return web.Response(text=html, content_type="text/html")

    async def _api_state(self, _request):
        return web.json_response({**self._latest, "dashboard_build": self._build_identity})

    async def _api_pose_latency(self, request):
        payload = await request.json()
        allowed_metrics = (
            "source_age_at_server_ms", "source_to_render_ms", "server_queue_ms",
            "websocket_ms", "browser_queue_ms",
        )
        validated = {"received_utc": time.time()}
        for name in allowed_metrics:
            metric = payload.get(name)
            if not isinstance(metric, dict):
                raise web.HTTPBadRequest(text=f"Missing latency metric: {name}")
            values = {key: metric.get(key) for key in ("n", "p50", "p95", "p99", "max")}
            if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values.values()):
                raise web.HTTPBadRequest(text=f"Invalid latency metric: {name}")
            validated[name] = values
        with self._lock:
            self._latest["field_lab_latency"] = validated
        return web.json_response({"success": True})

    async def _api_start(self, request):
        self._cancel_live_navigation_task()
        if self._navigation_process is not None:
            await self._call_set_bool(self._autonomy_arm_client, False)
            await asyncio.to_thread(self._terminate_navigation)
        body = await request.json()
        profile = str(body.get("profile", "indoor_live"))
        if profile not in ("indoor_live", "indoor_high_quality", "outdoor_structured", "outdoor_open_field"):
            raise web.HTTPBadRequest(text="Unknown profile")
        mapping_source = str(body.get("mapping_source", FUSED_MAPPING))
        if mapping_source not in MAPPING_SOURCES:
            raise web.HTTPBadRequest(text="Unknown mapping source")
        self._profile_pub.publish(String(data=profile))
        self._source_pub.publish(String(data=mapping_source))
        self._record_pub.publish(Bool(data=bool(body.get("record_svo", False))))
        await asyncio.sleep(0.15)
        result = await self._call_service(self._start_client)
        if result["success"]:
            self._schedule_live_navigation()
        return web.json_response(result, status=200 if result["success"] else 409)

    async def _api_stop(self, _request):
        self._cancel_live_navigation_task()
        result = await self._call_service(self._stop_client)
        if result["success"] and self._navigation_kind == "LIVE_MAPPING":
            await self._call_set_bool(self._autonomy_arm_client, False)
            await asyncio.to_thread(self._terminate_navigation)
        return web.json_response(result, status=200 if result["success"] else 409)

    async def _api_take_snapshot(self, _request):
        with self._lock:
            if self._snapshot_task is not None and not self._snapshot_task.done():
                return web.json_response(
                    {"success": False, "message": "Snapshot transaction already running"},
                    status=409,
                )
            mapping = dict(self._latest.get("mapping", {}))
            if mapping.get("state") not in ("MAPPING", "DEGRADED"):
                return web.json_response(
                    {"success": False, "message": "TAKE SNAPSHOT requires an active mapping session"},
                    status=409,
                )
            if self._grid is None or not any(value >= 0 for value in self._grid.get("data", ())):
                return web.json_response(
                    {"success": False, "message": "TAKE SNAPSHOT requires a valid occupancy map"},
                    status=409,
                )
            health = self._latest.get("health", {})
            vehicle = health.get("vehicle")
            if not isinstance(vehicle, dict) or vehicle.get("linear_velocity_mps") is None:
                return web.json_response(
                    {"success": False, "message": "Vehicle stopped state is unavailable"},
                    status=409,
                )
            speed = float(vehicle["linear_velocity_mps"])
            if not math.isfinite(speed):
                return web.json_response(
                    {"success": False, "message": "Vehicle stopped state is invalid"},
                    status=409,
                )
            if abs(speed) > 0.01:
                return web.json_response(
                    {"success": False, "message": "Stop the vehicle before taking a snapshot"},
                    status=409,
                )
            if mapping.get("fusion_state") == "FUSED_ERROR" or mapping.get("state") == "ERROR":
                return web.json_response(
                    {"success": False, "message": "Mapping is in fatal teardown"}, status=409
                )
            self._snapshot_task = self._loop.create_task(self._snapshot_transaction(mapping))
        return web.json_response(
            {"success": True, "message": "Snapshot validation started"}, status=202
        )

    def _set_navigation_state(self, state: str, **values) -> None:
        navigation = dict(self._latest.get("navigation", {}))
        navigation.update(values)
        navigation["state"] = state
        if state in ("STARTING_LIVE_NAVIGATION", "LIVE_MAPPING_READY"):
            navigation["mode"] = "LIVE_MAPPING"
        elif state in ("STARTING_NAVIGATION", "LOCALIZING", "READY"):
            navigation["mode"] = "NAVIGATION"
        else:
            navigation["mode"] = "MAPPING"
        self._latest["navigation"] = navigation
        self._broadcast({"type": "navigation", "data": navigation})

    async def _snapshot_transaction(self, mapping: dict) -> None:
        try:
            self._set_navigation_state("VALIDATING", message="Validating current map")
            db_path = Path(str(mapping.get("db_path", "")))
            if not db_path.is_file():
                raise SnapshotError("RTAB database does not exist")
            session_dir = db_path.parent.parent
            with self._lock:
                pose = list(self._last_pose) if self._last_pose is not None else None
                pose_age_sec = time.monotonic() - self._pose_received if self._pose_received else math.inf
            if pose is None:
                raise SnapshotError("Final map pose is unavailable")
            if pose_age_sec > 0.5:
                raise SnapshotError(
                    f"Final map pose is stale ({pose_age_sec:.3f} s); snapshot transition refused"
                )
            final_pose = {
                "x": pose[0], "y": pose[1], "z": pose[2],
                "qx": pose[3], "qy": pose[4], "qz": pose[5], "qw": pose[6],
                "captured_utc": time.time(),
                "source_pose_age_ms": round(pose_age_sec * 1000.0, 3),
            }
            self._set_navigation_state("SAVING", message="Stopping and exporting mapping")
            self._cancel_live_navigation_task()
            if self._navigation_kind == "LIVE_MAPPING":
                await self._call_set_bool(self._autonomy_arm_client, False)
                await asyncio.to_thread(self._terminate_navigation)
            stop = await self._call_service(self._stop_client)
            if not stop["success"]:
                raise SnapshotError(stop["message"])
            deadline = time.monotonic() + 360.0
            while time.monotonic() < deadline:
                await asyncio.sleep(0.25)
                state = self._latest.get("mapping", {}).get("state")
                if state in ("COMPLETE", "ERROR"):
                    break
            if self._latest.get("mapping", {}).get("state") != "COMPLETE":
                error = self._latest.get("mapping", {}).get("error", "mapping finalization failed")
                raise SnapshotError(str(error))
            geometry = self._planning_geometry["values"]
            runtime = {
                "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
                "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
                "wheelbase_m": float(geometry["wheelbase_m"]),
                "minimum_turning_radius_m": float(geometry["minimum_turning_radius_m"]),
                "fused_odom_topic": "/laksa/odometry/fused",
                **self._runtime_provenance(),
            }
            manifest = await asyncio.to_thread(
                create_snapshot, session_dir, self._snapshots_root,
                final_pose=final_pose, runtime_context=runtime,
            )
            self._initial_pose = final_pose
            self._start_navigation(Path(manifest["path"]))
            self._set_navigation_state(
                "STARTING_NAVIGATION", message="Starting official saved-map navigation",
                snapshot=manifest,
            )
            await self._await_navigation_ready()
        except Exception as error:
            self._request_disarm("snapshot/navigation transition failed")
            self._set_navigation_state("SNAPSHOT_FAILED", message=str(error))
            self.get_logger().error(f"TAKE SNAPSHOT failed: {error}")

    @staticmethod
    def _runtime_provenance() -> dict:
        packages = (
            "ros-humble-rtabmap-ros", "ros-humble-nav2-map-server",
            "ros-humble-nav2-amcl", "ros-humble-nav2-smac-planner",
            "ros-humble-nav2-mppi-controller", "ros-humble-robot-localization",
            "ros-humble-rmw-cyclonedds-cpp",
        )
        result = subprocess.run(
            ["dpkg-query", "-W", "-f=${Package}=${Version}\\n", *packages],
            capture_output=True, text=True, timeout=5, check=False,
        )
        versions = dict(
            line.split("=", 1) for line in result.stdout.splitlines() if "=" in line
        )
        ros_versions = {}
        for package in (
            "zed_wrapper", "rtabmap_ros", "rtabmap_sync", "rtabmap_slam",
            "robot_localization", "nav2_map_server", "nav2_amcl",
            "nav2_smac_planner", "nav2_mppi_controller", "laksa_mapping",
            "laksa_dashboard", "laksa_bringup", "laksa_lidar",
        ):
            try:
                package_xml = Path(get_package_share_directory(package)) / "package.xml"
                ros_versions[package] = ET.parse(package_xml).getroot().findtext("version")
            except Exception:
                ros_versions[package] = None
        commit = subprocess.run(
            ["git", "-C", "/home/ubuntu/src/Project_LAKSA", "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=5, check=False,
        ).stdout.strip()
        return {
            "git_commit": commit or None,
            "debian_package_versions": versions,
            "ros_package_versions": ros_versions,
        }

    def _cancel_live_navigation_task(self) -> None:
        task = self._live_navigation_task
        self._live_navigation_task = None
        if task is not None and not task.done():
            task.cancel()

    def _schedule_live_navigation(self) -> None:
        """Schedule one canonical Nav2 planner/controller for the live RTAB map."""
        if (
            self._navigation_kind == "LIVE_MAPPING"
            and self._navigation_process is not None
            and self._navigation_process.poll() is None
        ):
            return
        if self._live_navigation_task is not None:
            return
        mapping = self._latest.get("mapping", {})
        if mapping.get("state") not in ("STARTING", "MAPPING", "DEGRADED"):
            return
        session_id = str(mapping.get("session_id", ""))
        if not session_id:
            return
        self._live_navigation_task = self._loop.create_task(
            self._live_navigation_transaction(session_id)
        )

    async def _live_navigation_transaction(self, session_id: str) -> None:
        try:
            await asyncio.to_thread(self._start_live_navigation, session_id)
            self._set_navigation_state(
                "STARTING_LIVE_NAVIGATION",
                message="Starting planner and controller on the live RTAB map",
                snapshot=None,
                localization_source="RTAB",
                localization_valid=False,
            )
            await self._await_live_navigation_ready(session_id)
        except asyncio.CancelledError:
            return
        except Exception as error:
            self._request_disarm("live-map navigation startup failed")
            self._set_navigation_state(
                "LIVE_MAPPING_FAILED",
                message=str(error),
                localization_source="RTAB",
                localization_valid=False,
            )
            self.get_logger().error(f"Live-map navigation failed: {error}")
        finally:
            if self._live_navigation_task is asyncio.current_task():
                self._live_navigation_task = None

    def _start_live_navigation(self, session_id: str) -> None:
        self._terminate_navigation()
        preview = subprocess.run(
            ["systemctl", "is-active", "--quiet", "laksa-planning-preview.service"],
            check=False, timeout=3,
        )
        if preview.returncode == 0:
            raise RuntimeError(
                "Non-canonical laksa-planning-preview.service is active; refusing duplicate "
                "planner_server/controller_server authorities"
            )
        log_root = Path("/home/ubuntu/laksa_diagnostics/navigation")
        log_root.mkdir(parents=True, exist_ok=True)
        self._navigation_log = (log_root / f"live-{session_id}.log").open(
            "a", encoding="utf-8", buffering=1
        )
        self._navigation_process = subprocess.Popen(
            ["ros2", "launch", "laksa_dashboard", "planning_preview.launch.py"],
            stdout=self._navigation_log,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
            start_new_session=True,
        )
        self._navigation_kind = "LIVE_MAPPING"

    async def _await_live_navigation_ready(self, session_id: str) -> None:
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            process = self._navigation_process
            if (
                process is None
                or self._navigation_kind != "LIVE_MAPPING"
                or process.poll() is not None
            ):
                raise RuntimeError("Canonical live-map navigation launch exited")
            mapping = self._latest.get("mapping", {})
            if (
                str(mapping.get("session_id", "")) != session_id
                or mapping.get("state") not in ("STARTING", "MAPPING", "DEGRADED")
            ):
                raise RuntimeError("Live mapping session ended before navigation became ready")
            active = {
                name: await self._lifecycle_active(name)
                for name in ("planner_server", "controller_server")
            }
            map_error = self._map_validity_reason()
            costmap_error = self._planning_costmap_validity_reason()
            pose_error = self._canonical_pose_rejection_reason()
            local_costmap_fresh = bool(
                self._local_costmap_received
                and time.monotonic() - self._local_costmap_received <= 1.0
            )
            if all(active.values()) and not map_error and not costmap_error and not pose_error and local_costmap_fresh:
                # The readiness transaction and the periodic status poll share
                # the same lifecycle clients.  Publish the authoritative result
                # here as well so an older in-flight poll cannot leave planning
                # visually disabled after both Nav2 servers reached ACTIVE.
                self._planner_active = True
                self._controller_active = True
                self._set_navigation_state(
                    "LIVE_MAPPING_READY",
                    message="Live RTAB map, localization, planner, and controller ready",
                    lifecycle={
                        "map_server": False,
                        "amcl": False,
                        **active,
                    },
                    localization_source="RTAB",
                    localization_valid=True,
                )
                return
            reason = (
                map_error
                or costmap_error
                or pose_error
                or ("Local obstacle costmap is stale" if not local_costmap_fresh else None)
                or "Waiting for planner and controller lifecycle activation"
            )
            self._set_navigation_state(
                "STARTING_LIVE_NAVIGATION",
                message=reason,
                lifecycle={"map_server": False, "amcl": False, **active},
                localization_source="RTAB",
                localization_valid=False,
            )
            await asyncio.sleep(0.5)
        raise RuntimeError("Live-map planner/controller did not reach ACTIVE state")

    def _start_navigation(self, snapshot: Path) -> None:
        self._terminate_navigation()
        preview = subprocess.run(
            ["systemctl", "is-active", "--quiet", "laksa-planning-preview.service"],
            check=False, timeout=3,
        )
        if preview.returncode == 0:
            raise RuntimeError(
                "Non-canonical laksa-planning-preview.service is active; refusing duplicate "
                "planner_server/controller_server authorities"
            )
        log_root = Path("/home/ubuntu/laksa_diagnostics/navigation")
        log_root.mkdir(parents=True, exist_ok=True)
        self._navigation_log = (log_root / f"{snapshot.name}.log").open(
            "a", encoding="utf-8", buffering=1
        )
        command = [
            "ros2", "launch", "laksa_dashboard", "navigation_saved_map.launch.py",
            f"map_yaml:={snapshot / 'map.yaml'}",
        ]
        self._navigation_process = subprocess.Popen(
            command, stdout=self._navigation_log, stderr=subprocess.STDOUT,
            env=os.environ.copy(), start_new_session=True,
        )
        self._navigation_kind = "SAVED_MAP"

    async def _lifecycle_active(self, name: str) -> bool:
        client = self._nav_state_clients[name]
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            return False
        try:
            return int(future.result().current_state.id) == 3
        except Exception:
            return False

    async def _await_navigation_ready(self) -> None:
        deadline = time.monotonic() + 90.0
        last_initial_pose = 0.0
        last_amcl_sequence = 0
        continuity_passes = 0
        self._set_navigation_state("LOCALIZING", message="Waiting for official AMCL")
        while time.monotonic() < deadline:
            process = self._navigation_process
            if process is None or process.poll() is not None:
                raise RuntimeError("Canonical navigation launch exited")
            active = {
                name: await self._lifecycle_active(name) for name in self._nav_state_clients
            }
            if active["amcl"] and (
                last_initial_pose == 0.0
                or (not self._amcl_pose_received and time.monotonic() - last_initial_pose >= 3.0)
            ):
                self._publish_initial_pose()
                last_initial_pose = time.monotonic()
            # AMCL only publishes a new amcl_pose after a laser update.  A
            # stationary vehicle can therefore have one valid official
            # initialization for the entire wait.  Treat that initialization
            # as session-scoped and use the live, timestamped canonical TF
            # below for the consecutive freshness/continuity observations.
            localized = bool(self._amcl_pose_received)
            continuity = self._pose_continuity()
            if continuity is not None and self._amcl_pose_sequence != last_amcl_sequence:
                last_amcl_sequence = self._amcl_pose_sequence
                if continuity["hard_failure"] and last_amcl_sequence >= 3:
                    raise RuntimeError(
                        "Snapshot pose continuity failed: "
                        f"position jump {continuity['position_delta_m']:.3f} m, "
                        f"yaw jump {continuity['yaw_delta_deg']:.2f} deg"
                    )
            canonical_pose_valid = self._canonical_pose_rejection_reason() is None
            if continuity is not None and localized and continuity["within_target"] and canonical_pose_valid:
                continuity_passes += 1
            else:
                continuity_passes = 0
            continuity_valid = continuity is not None and continuity_passes >= 3
            if all(active.values()) and localized and continuity_valid:
                try:
                    self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                    self._tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                except TransformException:
                    pass
                else:
                    self._clear_preview("Saved map ready; select a destination")
                    self._set_navigation_state(
                        "READY", message="Saved map and AMCL localization ready",
                        lifecycle=active, localization_valid=True,
                        pose_continuity=continuity,
                    )
                    return
            detail = "Waiting for official AMCL"
            if continuity is not None:
                detail = (
                    "Validating stationary pose continuity: "
                    f"{continuity['position_delta_m']:.3f} m, "
                    f"{continuity['yaw_delta_deg']:.2f} deg"
                )
            self._set_navigation_state(
                "LOCALIZING", message=detail,
                lifecycle=active, localization_valid=False,
                pose_continuity=continuity,
            )
            await asyncio.sleep(0.5)
        raise RuntimeError("Saved-map navigation did not reach localized ACTIVE state")

    def _publish_initial_pose(self) -> None:
        pose = self._initial_pose
        if not pose:
            return
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        message.pose.pose.position.z = float(pose.get("z", 0.0))
        message.pose.pose.orientation.x = float(pose["qx"])
        message.pose.pose.orientation.y = float(pose["qy"])
        message.pose.pose.orientation.z = float(pose["qz"])
        message.pose.pose.orientation.w = float(pose["qw"])
        # The source pose was captured while stationary immediately before
        # mapping shutdown.  Keep AMCL's official particle initialization
        # inside the product continuity gate instead of seeding a 22-32 cm
        # cloud that can visibly teleport the robot.
        message.pose.covariance[0] = 0.05 ** 2
        message.pose.covariance[7] = 0.05 ** 2
        message.pose.covariance[35] = math.radians(2.0) ** 2
        self._initial_pose_pub.publish(message)

    def _amcl_pose_cb(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map":
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = [
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        ]
        if not all(math.isfinite(value) for value in pose):
            return
        with self._lock:
            self._latest_amcl_pose = {
                "x": position.x, "y": position.y,
                "yaw": self._pose_yaw(pose),
            }
            self._amcl_pose_received = time.monotonic()
            self._amcl_pose_sequence += 1

    def _pose_continuity(self) -> dict | None:
        expected = self._initial_pose
        actual = self._latest_amcl_pose
        if not expected or not actual:
            return None
        expected_pose = [
            expected["x"], expected["y"], expected.get("z", 0.0),
            expected["qx"], expected["qy"], expected["qz"], expected["qw"],
        ]
        position_delta = math.hypot(
            float(actual["x"]) - float(expected["x"]),
            float(actual["y"]) - float(expected["y"]),
        )
        yaw_delta = abs(math.atan2(
            math.sin(float(actual["yaw"]) - self._pose_yaw(expected_pose)),
            math.cos(float(actual["yaw"]) - self._pose_yaw(expected_pose)),
        ))
        yaw_delta_deg = math.degrees(yaw_delta)
        return {
            "position_delta_m": round(position_delta, 6),
            "yaw_delta_deg": round(yaw_delta_deg, 6),
            "target_position_m": 0.05,
            "target_yaw_deg": 2.0,
            "within_target": position_delta <= 0.05 and yaw_delta_deg <= 2.0,
            "hard_failure": position_delta > 0.15 or yaw_delta_deg > 5.0,
            "samples": self._amcl_pose_sequence,
        }

    def _fused_odom_cb(self, message: Odometry) -> None:
        if message.header.frame_id != "odom" or message.child_frame_id != "base_footprint":
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        stamp = message.header.stamp
        sample = (
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            (stamp.sec, stamp.nanosec),
        )
        with self._lock:
            self._latest_fused_odom_pose = sample
        self._publish_pose(sample)

    def _terminate_navigation(self) -> None:
        process = self._navigation_process
        self._navigation_process = None
        self._navigation_kind = "NONE"
        if process is not None and process.poll() is None:
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5)):
                try:
                    os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
                except ProcessLookupError:
                    break
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        if self._navigation_log is not None:
            self._navigation_log.close()
            self._navigation_log = None
        self._amcl_pose_received = 0.0
        self._amcl_pose_sequence = 0
        self._latest_amcl_pose = None
        self._set_navigation_state("IDLE", message="Navigation stopped")

    async def _api_preview_path(self, request):
        try:
            body = await request.json()
            goal_x = float(body["x"]); goal_y = float(body["y"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            raise web.HTTPBadRequest(text="Goal must contain finite x and y coordinates")
        if not math.isfinite(goal_x) or not math.isfinite(goal_y):
            raise web.HTTPBadRequest(text="Goal must contain finite x and y coordinates")
        with self._lock:
            self._cancel_active_follow("A new goal was selected")
            self._cancel_active_planner_goal()
            self._planned_path = None
            self._planning_token += 1
            token = self._planning_token
            self._planning_request = None
            reason = self._planning_rejection_reason(goal_x, goal_y, str(body.get("session_id", "")))
            if reason:
                base_state = self._planning_base_state()
                self.get_logger().warning(f"Planning preview rejected before ComputePathToPose: {reason}")
                self._set_planning_state(
                    "INVALID_GOAL" if base_state == "READY" else base_state,
                    reason, goal=None, path=[], planning_time_ms=None, path_length_m=None,
                )
                return web.json_response({"success": False, "message": reason, "planning": self._planning}, status=409)
            start_yaw = self._pose_yaw(self._last_pose)
            if not self._candidate_footprint_is_free(
                self._planning_grid, self._last_pose[0], self._last_pose[1], start_yaw
            ):
                reason = "Current vehicle footprint intersects non-free planning space"
                self._set_planning_state(
                    "START_BLOCKED", reason, goal=None, path=[],
                    planning_time_ms=None, path_length_m=None,
                )
                return web.json_response({"success": False, "message": reason, "planning": self._planning}, status=409)
            goal_yaw = self._auto_goal_heading(goal_x, goal_y)
            if not self._candidate_footprint_is_free(self._planning_grid, goal_x, goal_y, goal_yaw):
                reason = "Selected destination is no longer collision-free"
                self._set_planning_state(
                    "GOAL_BLOCKED", reason,
                    goal={"x": goal_x, "y": goal_y, "yaw": goal_yaw}, path=[],
                    planning_time_ms=None, path_length_m=None,
                )
                return web.json_response({"success": False, "message": reason, "planning": self._planning}, status=409)
            self._planning_request = {
                "token": token,
                "session_id": self._active_session_id,
                "grid_revision": self._grid_revision,
                "started": time.monotonic(),
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_yaw": goal_yaw,
            }
            self._set_planning_state(
                "PLANNING",
                "Computing one Ackermann path with automatic terminal heading",
                goal={"x": goal_x, "y": goal_y, "yaw": goal_yaw},
                path=[], planning_time_ms=None, path_length_m=None,
            )
            start = self._pose_stamped(self._last_pose)
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
            goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
            action_goal = ComputePathToPose.Goal()
            action_goal.start = start
            action_goal.goal = goal
            action_goal.planner_id = "GridBased"
            # Use the exact canonical map->base_footprint pose displayed by
            # Field Lab.  Planning and visualization must never diverge.
            action_goal.use_start = True
            future = self._planner_client.send_goal_async(action_goal)
            future.add_done_callback(lambda done: self._planner_goal_response(done, token))
        return web.json_response({"success": True, "message": "Automatic-heading planning requested"}, status=202)

    async def _api_clear_path(self, _request):
        with self._lock:
            self._clear_preview("Preview route cleared")
        return web.json_response({"success": True, "message": "Preview route cleared"})

    async def _api_follow_path(self, request):
        """Send the exact planner result to official Nav2 FollowPath.

        Confirmation is explicit. Internal arming remains delegated to
        DriveSupervisor; this endpoint never publishes actuator commands or
        bypasses Xbox-first arbitration.
        """
        try:
            body = await request.json()
        except (json.JSONDecodeError, TypeError):
            body = {}
        if body.get("confirmed") is not True:
            return web.json_response(
                {"success": False, "message": "Route start confirmation is required"},
                status=409,
            )
        with self._lock:
            if self._follow_starting or self._active_follow_goal is not None:
                return web.json_response(
                    {"success": False, "message": "Route start is already in progress"}, status=409
                )
            blocker = self._follow_preflight_reason()
            if blocker:
                return web.json_response({"success": False, "message": blocker}, status=409)
            exact_path = self._planned_path
            self._follow_starting = True
            self._set_planning_state(
                "STARTING_ROUTE", "Checking route safety and requesting vehicle authority",
                follow_state="STARTING", physical_output_enabled=False,
            )
        armed = await self._call_set_bool(self._autonomy_arm_client, True)
        if not armed["success"]:
            detail = str(armed.get("message", "safety supervisor rejected route start"))
            for internal, product in (
                ("DISARMED", "inhibited"), ("ARMED", "enabled"),
                ("DISARM", "stop"), ("ARM", "start"),
                ("disarmed", "inhibited"), ("armed", "enabled"),
                ("disarm", "stop"), ("arm", "start"),
            ):
                detail = detail.replace(internal, product)
            return await self._rollback_follow_start(f"Route preflight failed: {detail}")
        with self._lock:
            blocker = self._follow_preflight_reason()
            if self._planned_path is not exact_path:
                blocker = "Route changed during preflight; vehicle authority was released"
            if not blocker:
                self._follow_token += 1
                token = self._follow_token
                action_goal = FollowPath.Goal()
                action_goal.path = exact_path
                action_goal.controller_id = "FollowPath"
                action_goal.goal_checker_id = "goal_checker"
                future = self._follow_client.send_goal_async(action_goal)
        if blocker:
            return await self._rollback_follow_start(blocker)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            return await self._rollback_follow_start(
                "Official FollowPath did not acknowledge the route in time"
            )
        try:
            goal_handle = future.result()
        except Exception as error:
            return await self._rollback_follow_start(f"FollowPath request failed: {error}")
        if not goal_handle.accepted:
            return await self._rollback_follow_start("Official FollowPath rejected the route")
        with self._lock:
            if token != self._follow_token or self._planned_path is not exact_path:
                goal_handle.cancel_goal_async()
                canceled_before_start = True
            else:
                canceled_before_start = False
                self._follow_starting = False
                self._active_follow_goal = (token, goal_handle)
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(lambda done: self._follow_result(done, token))
                self._set_planning_state(
                    "FOLLOWING",
                    "Official Nav2 FollowPath accepted; Xbox input immediately takes manual control",
                    follow_state="FOLLOWING", physical_output_enabled=True,
                )
        if canceled_before_start:
            return await self._rollback_follow_start(
                "Route changed before FollowPath acceptance; vehicle authority was released"
            )
        return web.json_response(
            {"success": True, "message": "Route accepted by official Nav2 FollowPath"},
            status=200,
        )

    async def _rollback_follow_start(self, message: str):
        await self._call_set_bool(self._autonomy_arm_client, False)
        with self._lock:
            self._follow_starting = False
            state = "PATH_READY" if self._planned_path is not None and self._planned_path.poses else self._planning_base_state()
            self._set_planning_state(
                state, message, follow_state="IDLE", physical_output_enabled=False,
            )
        return web.json_response({"success": False, "message": message}, status=409)

    async def _api_cancel_follow(self, _request):
        with self._lock:
            canceled = self._cancel_active_follow("FollowPath canceled by operator")
            self._request_disarm("operator canceled FollowPath")
        return web.json_response({
            "success": True,
            "message": "FollowPath cancel requested" if canceled else "No active FollowPath goal",
        })

    def _planning_rejection_reason(self, x: float, y: float, requested_session: str) -> str | None:
        if not self._planning_geometry["calibrated"]:
            return "Planner is uncalibrated; enter measured LAKSA geometry first"
        if requested_session != self._active_session_id or not requested_session:
            return "The mapping session changed; tap the current map again"
        map_error = self._map_validity_reason()
        if map_error:
            return map_error
        planning_map_error = self._planning_costmap_validity_reason()
        if planning_map_error:
            return planning_map_error
        pose_error = self._canonical_pose_rejection_reason()
        if pose_error:
            return pose_error
        if not self._planner_active:
            return "SmacPlannerHybrid is not ACTIVE"
        if not self._planner_client.server_is_ready():
            return "Planning preview service is unavailable"
        return None

    def _canonical_pose_rejection_reason(self) -> str | None:
        if self._last_pose is None or time.monotonic() - self._pose_received > self._pose_timeout:
            return "Canonical map->base_footprint pose is unavailable or stale"
        if self._navigation_kind == "SAVED_MAP" and not self._amcl_pose_received:
            return "AMCL localization initialization is unavailable"
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except TransformException as error:
            return f"Canonical map->base_footprint transform is unavailable: {error}"
        stamp_ns = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
        if stamp_ns <= 0:
            return "Canonical map->base_footprint transform has no timestamp"
        age_sec = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
        if not math.isfinite(age_sec) or age_sec < -0.1 or age_sec > self._pose_timeout:
            return f"Canonical map->base_footprint transform is stale ({age_sec:.3f} s)"
        return None

    def _pose_stamped(self, pose: list) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "map"
        stamped.pose.position.x = float(pose[0])
        stamped.pose.position.y = float(pose[1])
        stamped.pose.position.z = float(pose[2])
        stamped.pose.orientation.x = float(pose[3])
        stamped.pose.orientation.y = float(pose[4])
        stamped.pose.orientation.z = float(pose[5])
        stamped.pose.orientation.w = float(pose[6])
        return stamped

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _pose_yaw(pose: list) -> float:
        qx, qy, qz, qw = pose[3:7]
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _auto_goal_heading(self, goal_x: float, goal_y: float, pose: list | None = None) -> float:
        pose = self._last_pose if pose is None else pose
        robot_yaw = self._pose_yaw(pose)
        dx = goal_x - pose[0]; dy = goal_y - pose[1]
        bearing = math.atan2(dy, dx)
        distance = math.hypot(dx, dy)
        turning_radius = float(self._planning_geometry["values"]["minimum_turning_radius_m"])
        desired_delta = self._wrap_to_pi(bearing - robot_yaw)
        max_heading_change = min(math.pi, distance / turning_radius)
        limited_delta = max(-max_heading_change, min(max_heading_change, desired_delta))
        return self._wrap_to_pi(robot_yaw + limited_delta)

    def _map_validity_reason(self) -> str | None:
        grid = self._grid
        if grid is None:
            return "Occupancy map has not been received for the current mapping session"
        if not self._active_session_id or grid.get("session_id") != self._active_session_id:
            return "Occupancy map does not belong to the current mapping session"
        if grid.get("frame") != "map":
            return f"Occupancy map frame is {grid.get('frame')!r}, expected 'map'"
        width = grid.get("width"); height = grid.get("height"); resolution = grid.get("resolution")
        if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
            return "Occupancy map dimensions are invalid"
        if not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0.0:
            return "Occupancy map resolution is invalid"
        if len(grid.get("data", [])) != width * height:
            return "Occupancy map data size is invalid"
        return None

    def _planning_costmap_validity_reason(self) -> str | None:
        grid = self._planning_grid
        if grid is None:
            return "Nav2 global planning costmap has not been received"
        if not self._active_session_id or grid.get("session_id") != self._active_session_id:
            return "Nav2 global planning costmap does not belong to the current mapping session"
        if grid.get("frame") != "map":
            return f"Nav2 global planning costmap frame is {grid.get('frame')!r}, expected 'map'"
        width = grid.get("width"); height = grid.get("height"); resolution = grid.get("resolution")
        if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
            return "Nav2 global planning costmap dimensions are invalid"
        if not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0.0:
            return "Nav2 global planning costmap resolution is invalid"
        if len(grid.get("data", [])) != width * height:
            return "Nav2 global planning costmap data size is invalid"
        return None

    @staticmethod
    def _world_to_grid_in(grid: dict, x: float, y: float) -> tuple[int, int] | None:
        origin = grid["origin"]
        yaw = math.atan2(2.0 * (origin[6] * origin[5] + origin[3] * origin[4]), 1.0 - 2.0 * (origin[4] ** 2 + origin[5] ** 2))
        dx = x - origin[0]; dy = y - origin[1]
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        column = math.floor(local_x / grid["resolution"])
        row = math.floor(local_y / grid["resolution"])
        if column < 0 or row < 0 or column >= grid["width"] or row >= grid["height"]:
            return None
        return column, row

    @staticmethod
    def _world_to_grid_float_in(grid: dict, x: float, y: float) -> tuple[float, float]:
        origin = grid["origin"]
        yaw = math.atan2(2.0 * (origin[6] * origin[5] + origin[3] * origin[4]), 1.0 - 2.0 * (origin[4] ** 2 + origin[5] ** 2))
        dx = x - origin[0]; dy = y - origin[1]
        return (
            (math.cos(yaw) * dx + math.sin(yaw) * dy) / grid["resolution"],
            (-math.sin(yaw) * dx + math.cos(yaw) * dy) / grid["resolution"],
        )

    @staticmethod
    def _grid_to_world_in(grid: dict, column: float, row: float) -> tuple[float, float]:
        origin = grid["origin"]
        yaw = math.atan2(2.0 * (origin[6] * origin[5] + origin[3] * origin[4]), 1.0 - 2.0 * (origin[4] ** 2 + origin[5] ** 2))
        local_x = column * grid["resolution"]; local_y = row * grid["resolution"]
        return (origin[0] + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
                origin[1] + math.sin(yaw) * local_x + math.cos(yaw) * local_y)

    def _candidate_footprint_is_free(self, grid: dict, x: float, y: float, heading: float) -> bool:
        geometry = self._planning_geometry["values"]
        front = float(geometry["front_m"]); rear = float(geometry["rear_m"])
        left = float(geometry["left_m"]); right = float(geometry["right_m"])
        cos_heading = math.cos(heading); sin_heading = math.sin(heading)
        footprint = []
        for longitudinal, lateral in ((front, left), (front, -right), (-rear, -right), (-rear, left)):
            world_x = x + cos_heading * longitudinal - sin_heading * lateral
            world_y = y + sin_heading * longitudinal + cos_heading * lateral
            footprint.append(self._world_to_grid_float_in(grid, world_x, world_y))
        epsilon = 1.0e-9
        min_column = math.floor(min(point[0] for point in footprint) - epsilon)
        max_column = math.floor(max(point[0] for point in footprint) + epsilon)
        min_row = math.floor(min(point[1] for point in footprint) - epsilon)
        max_row = math.floor(max(point[1] for point in footprint) + epsilon)
        width = grid["width"]; height = grid["height"]; data = grid["data"]
        first_edge = (footprint[1][0] - footprint[0][0], footprint[1][1] - footprint[0][1])
        second_edge = (footprint[2][0] - footprint[1][0], footprint[2][1] - footprint[1][1])
        footprint_axes = ((1.0, 0.0), (0.0, 1.0),
                          (-first_edge[1], first_edge[0]), (-second_edge[1], second_edge[0]))
        separating_axes = []
        for axis_x, axis_y in footprint_axes:
            projection = [point[0] * axis_x + point[1] * axis_y for point in footprint]
            separating_axes.append((
                axis_x, axis_y, min(projection), max(projection),
                min(0.0, axis_x) + min(0.0, axis_y),
                max(0.0, axis_x) + max(0.0, axis_y),
            ))
        for row in range(min_row, max_row + 1):
            for column in range(min_column, max_column + 1):
                intersects = True
                for axis_x, axis_y, polygon_min, polygon_max, cell_min_offset, cell_max_offset in separating_axes:
                    cell_origin = column * axis_x + row * axis_y
                    cell_min = cell_origin + cell_min_offset
                    cell_max = cell_origin + cell_max_offset
                    if polygon_max < cell_min - epsilon or cell_max < polygon_min - epsilon:
                        intersects = False
                        break
                if not intersects:
                    continue
                if column < 0 or row < 0 or column >= width or row >= height:
                    return False
                cost = data[row * width + column]
                if cost < 0 or cost >= 99:
                    return False
        return True

    def _cancel_active_planner_goal(self) -> None:
        active = self._active_planner_goal
        self._active_planner_goal = None
        if active is not None:
            try:
                active[1].cancel_goal_async()
            except Exception:
                pass

    def _request_for_token(self, token: int):
        request = self._planning_request
        if request is None or request["token"] != token:
            return None
        if request["session_id"] != self._active_session_id:
            return None
        return request

    def _planner_goal_response(self, future, token: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_planning_failure(token, f"Planner request failed: {exc}")
            return
        with self._lock:
            if self._request_for_token(token) is None:
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return
            if not goal_handle.accepted:
                self._finish_planning_failure(token, "Planner rejected the goal")
                return
            self._active_planner_goal = (token, goal_handle)
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda done: self._planner_result(done, token))

    def _planner_result(self, future, token: int) -> None:
        try:
            result = future.result().result
            path = result.path
        except Exception as exc:
            self._finish_planning_failure(token, f"Planner result failed: {exc}")
            return
        with self._lock:
            request = self._request_for_token(token)
            if request is None:
                return
            if self._active_planner_goal is not None and self._active_planner_goal[0] == token:
                self._active_planner_goal = None
            if request["grid_revision"] != self._grid_revision:
                self._planning_request = None
                self._planned_path = None
                self._set_planning_state(
                    self._planning_base_state(), "Map changed while planning; tap again",
                    goal=None, path=[], planning_time_ms=None, path_length_m=None,
                )
                return
            if path.header.frame_id != "map" or not path.poses:
                self._planning_request = None
                self._planned_path = None
                self._set_planning_state(
                    "NO_PATH", "No collision-free Ackermann path was found",
                    goal=None, path=[], planning_time_ms=None, path_length_m=None,
                )
                return
            points = []
            total_length = 0.0
            for stamped in path.poses:
                position = stamped.pose.position; orientation = stamped.pose.orientation
                pose_yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
                point = [position.x, position.y, pose_yaw]
                if points:
                    total_length += math.hypot(point[0] - points[-1][0], point[1] - points[-1][1])
                points.append(point)
            planning_ms = round((time.monotonic() - request["started"]) * 1000.0, 1)
            self._preview_path_pub.publish(path)
            self._planned_path = path
            self._planning_request = None
            self._set_planning_state(
                "PATH_READY",
                "Ackermann preview path ready",
                planning_time_ms=planning_ms,
                path_length_m=round(total_length, 3),
                goal={"x": request["goal_x"], "y": request["goal_y"], "yaw": request["goal_yaw"]},
                path=points,
            )

    def _finish_planning_failure(self, token: int, message: str) -> None:
        with self._lock:
            if self._request_for_token(token) is None:
                return
            self._active_planner_goal = None
            self._planning_request = None
            self._planned_path = None
            self._set_planning_state(
                "NO_PATH", message, goal=None, path=[],
                planning_time_ms=None, path_length_m=None,
            )

    def _follow_goal_response(self, future, token: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_follow(token, f"FollowPath request failed: {exc}")
            return
        with self._lock:
            if token != self._follow_token:
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return
            if not goal_handle.accepted:
                self._finish_follow(token, "Official FollowPath rejected the route")
                return
            self._active_follow_goal = (token, goal_handle)
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda done: self._follow_result(done, token))

    def _follow_result(self, future, token: int) -> None:
        try:
            wrapped = future.result()
            succeeded = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            message = "Route complete" if succeeded else f"FollowPath ended with action status {wrapped.status}"
        except Exception as exc:
            succeeded = False
            message = f"FollowPath result failed: {exc}"
        self._finish_follow(token, message, succeeded=succeeded)

    def _finish_follow(self, token: int, message: str, *, succeeded: bool = False) -> None:
        with self._lock:
            if token != self._follow_token:
                return
            self._active_follow_goal = None
            self._request_disarm("FollowPath finished")
            if succeeded:
                state = "ROUTE_COMPLETE"
                follow_state = "COMPLETE"
            else:
                state = "FOLLOW_FAILED" if self._planned_path is not None and self._planned_path.poses else self._planning_base_state()
                follow_state = "FAILED"
            self._set_planning_state(
                state, message, follow_state=follow_state, physical_output_enabled=False,
            )

    def _cancel_active_follow(self, message: str) -> bool:
        active = self._active_follow_goal
        starting = self._follow_starting
        self._follow_starting = False
        self._follow_token += 1
        self._active_follow_goal = None
        if active is None and not starting:
            return False
        if active is not None:
            active[1].cancel_goal_async()
        self._request_disarm(message)
        self._set_planning_state(
            "PATH_READY" if self._planned_path is not None else self._planning_base_state(),
            message, follow_state="IDLE", physical_output_enabled=False,
        )
        return True

    def _cancel_navigation_cb(self, _message: Empty) -> None:
        with self._lock:
            self._cancel_active_follow("FollowPath canceled by LAKSA safety supervisor")

    def _autonomy_mode_cb(self, message: Bool) -> None:
        with self._lock:
            self._autonomy_armed = bool(message.data)

    def _autonomy_health_cb(self, message: String) -> None:
        with self._lock:
            self._autonomy_health = str(message.data)

    def _request_disarm(self, reason: str) -> None:
        if self._autonomy_arm_client.service_is_ready():
            self._autonomy_arm_client.call_async(SetBool.Request(data=False))
        self.get_logger().warning(f"Autonomy DISARM requested: {reason}")

    def _planning_base_state(self) -> str:
        if not self._planning_geometry["calibrated"]:
            return "UNCALIBRATED"
        if self._map_validity_reason() is not None or self._planning_costmap_validity_reason() is not None:
            return "WAITING_FOR_MAP"
        if self._safe_goal_cells == 0:
            return "WAITING_FOR_SAFE_AREA"
        if self._canonical_pose_rejection_reason() is not None:
            return "WAITING_FOR_POSE"
        if not self._planner_active or not self._planner_client.server_is_ready():
            return "DISABLED"
        return "READY"

    def _planning_tick(self) -> None:
        self._poll_planner_state()
        self._poll_controller_state()
        with self._lock:
            base = self._planning_base_state()
            if base != "READY" and self._planning["state"] in ("PLANNING", "PATH_READY", "STARTING_ROUTE", "FOLLOWING"):
                # Keep the exact planner result visible during transient map,
                # pose, or DDS-status fluctuations.  FOLLOW ROUTE is still
                # fail-closed: _follow_preflight_reason revalidates every path
                # footprint against the current costmap and all live gates.
                follow_available = self._follow_available()
                if self._planning.get("follow_available") != follow_available:
                    self._set_planning_state(
                        self._planning["state"], self._planning.get("message", "")
                    )
                return
            if base != "READY":
                if self._planning["state"] != base:
                    messages = {
                        "UNCALIBRATED": self._planning_geometry["message"],
                        "WAITING_FOR_MAP": self._map_validity_reason() or self._planning_costmap_validity_reason() or "Waiting for current planning maps",
                        "WAITING_FOR_SAFE_AREA": "Map still initializing; no collision-free goal area is available yet",
                        "WAITING_FOR_POSE": self._canonical_pose_rejection_reason() or "Canonical pose is unavailable",
                        "DISABLED": "SmacPlannerHybrid is not ACTIVE",
                    }
                    self._set_planning_state(base, messages.get(base, ""))
            elif self._planning["state"] in ("UNCALIBRATED", "WAITING_FOR_MAP", "WAITING_FOR_POSE", "DISABLED"):
                self._set_planning_state("READY", "Tap inside a shaded safe goal area")
            else:
                follow_available = self._follow_available()
                if self._planning.get("follow_available") != follow_available:
                    self._set_planning_state(self._planning["state"], self._planning.get("message", ""))

    def _set_planning_state(self, state: str, message: str = "", **values) -> None:
        current_goal = values.pop("goal", self._planning.get("goal"))
        current_path = values.pop("path", self._planning.get("path", []))
        self._planning = {
            "state": state, "preview_only": True, "heading_mode": "AUTO_HEADING",
            "planning_time_ms": values.pop("planning_time_ms", self._planning.get("planning_time_ms")),
            "path_length_m": values.pop("path_length_m", self._planning.get("path_length_m")),
            "message": message, "goal": current_goal, "path": current_path,
            "follow_available": self._follow_available(),
            "follow_state": values.pop("follow_state", self._planning.get("follow_state", "IDLE")),
            "physical_output_enabled": bool(values.pop("physical_output_enabled", self._autonomy_armed)),
            "autonomy_armed": self._autonomy_armed,
            "autonomy_health": self._autonomy_health,
            **values,
        }
        self._latest["planning"] = self._planning
        self._broadcast({"type": "planning", "data": self._planning})

    def _follow_available(self) -> bool:
        return self._follow_preflight_reason() is None

    def _follow_preflight_reason(self) -> str | None:
        path = self._planned_path
        if path is None or not path.poses or path.header.frame_id != "map":
            return "Plan a valid route first"
        planning_grid = self._planning_grid
        if planning_grid is None:
            return "Nav2 global planning costmap has not been received"
        for stamped in path.poses:
            pose = stamped.pose
            heading = math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
            )
            if not self._candidate_footprint_is_free(
                planning_grid, pose.position.x, pose.position.y, heading
            ):
                return "The live map changed and the displayed route is no longer collision-free"
        navigation = self._latest.get("navigation", {})
        if (
            navigation.get("state") not in ("READY", "LIVE_MAPPING_READY")
            or navigation.get("localization_valid") is not True
        ):
            return "Map localization and navigation are not READY"
        pose_error = self._canonical_pose_rejection_reason()
        if pose_error:
            return pose_error
        if not self._planner_active:
            return "SmacPlannerHybrid is not ACTIVE"
        if not self._controller_active or not self._follow_client.server_is_ready():
            return "Official MPPI FollowPath controller is not ACTIVE"
        if not self._autonomy_arm_client.service_is_ready():
            return "LAKSA vehicle safety supervisor is unavailable"
        if not self._local_costmap_received or time.monotonic() - self._local_costmap_received > 1.0:
            return "Local obstacle costmap is stale"
        if self._autonomy_health != "READY":
            return f"Route safety channel is not ready: {self._autonomy_health}"
        mapping = self._latest.get("mapping", {})
        if mapping.get("state") == "ERROR" or mapping.get("fusion_state") == "FUSED_ERROR":
            return "Mapping has a fatal health state; movement remains inhibited"
        health = self._latest.get("health", {})
        stamp = health.get("stamp")
        if not isinstance(stamp, (int, float)) or time.time() - float(stamp) > 2.0:
            return "Vehicle health telemetry is stale"
        subsystems = health.get("subsystems", {})
        xbox = subsystems.get("xbox_control", {})
        if xbox.get("state") != "ONLINE":
            return "Xbox manual-control safety channel is not ONLINE"
        if xbox.get("children", {}).get("manual_path", {}).get("state") != "ONLINE":
            return "Xbox manual command path is not ONLINE"
        lidar = subsystems.get("jetson", {}).get("children", {}).get("lidar", {})
        if lidar.get("state") != "ONLINE":
            return "LiDAR safety input is not ONLINE"
        esp32 = subsystems.get("esp32", {})
        children = esp32.get("children", {})
        if children.get("micro_ros", {}).get("state") != "ONLINE":
            return "ESP32 command link is not ONLINE"
        if children.get("pca9685", {}).get("state") != "ONLINE":
            return "Steering controller is not ONLINE"
        vesc = children.get("vesc", {})
        if vesc.get("state") != "ONLINE" or int(vesc.get("fault_code") or 0) != 0:
            return "VESC telemetry is stale or faulted"
        speed = health.get("vehicle", {}).get("linear_velocity_mps")
        if not isinstance(speed, (int, float)) or not math.isfinite(float(speed)):
            return "Vehicle stopped state is unavailable"
        if abs(float(speed)) > 0.01:
            return f"Stop the vehicle before starting the route ({float(speed):.3f} m/s)"
        return None

    def _poll_planner_state(self) -> None:
        future = self._planner_state_future
        if future is not None and not future.done():
            return
        if not self._planner_state_client.service_is_ready():
            # DDS discovery can briefly withdraw a lifecycle service even
            # while the validated launch and server remain healthy.  Preserve
            # the last authoritative lifecycle result in that transient; the
            # launch monitor and follow action readiness still fail closed.
            process = self._navigation_process
            if process is None or process.poll() is not None:
                self._planner_active = False
            return
        future = self._planner_state_client.call_async(GetState.Request())
        self._planner_state_future = future
        future.add_done_callback(self._planner_state_done)

    def _planner_state_done(self, future) -> None:
        try:
            active = int(future.result().current_state.id) == 3
        except Exception:
            return
        with self._lock:
            self._planner_active = active

    def _poll_controller_state(self) -> None:
        future = self._controller_state_future
        if future is not None and not future.done():
            return
        if not self._controller_state_client.service_is_ready():
            process = self._navigation_process
            if process is None or process.poll() is not None:
                self._controller_active = False
            return
        future = self._controller_state_client.call_async(GetState.Request())
        self._controller_state_future = future
        future.add_done_callback(self._controller_state_done)

    def _controller_state_done(self, future) -> None:
        try:
            active = int(future.result().current_state.id) == 3
        except Exception:
            return
        with self._lock:
            self._controller_active = active

    def _clear_preview(self, message: str) -> None:
        self._cancel_active_follow("Route cleared")
        self._cancel_active_planner_goal()
        self._planning_token += 1
        self._planning_request = None
        self._planned_path = None
        empty = NavigationPath(); empty.header.stamp = self.get_clock().now().to_msg(); empty.header.frame_id = "map"
        self._preview_path_pub.publish(empty)
        self._set_planning_state(self._planning_base_state(), message, goal=None, path=[])

    async def _call_service(self, client):
        if not client.service_is_ready(): return {"success": False, "message": "Mapping manager service unavailable"}
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline: await asyncio.sleep(0.03)
        if not future.done(): return {"success": False, "message": "Mapping manager timeout"}
        try:
            response = future.result(); return {"success": bool(response.success), "message": response.message}
        except Exception as exc: return {"success": False, "message": str(exc)}

    async def _call_set_bool(self, client, enabled: bool) -> dict:
        if not client.service_is_ready():
            return {"success": False, "message": "LAKSA autonomy supervisor is unavailable"}
        future = client.call_async(SetBool.Request(data=enabled))
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            return {"success": False, "message": "LAKSA route safety preflight timeout"}
        try:
            response = future.result()
            return {"success": bool(response.success), "message": str(response.message)}
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    async def _ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20); await ws.prepare(request); self._ws_clients.add(ws)
        self._websocket_client_count = len(self._ws_clients)
        await ws.send_json({"type": "snapshot", "data": self._latest})
        with self._lock:
            trajectory = list(self._trajectory)
        await ws.send_json({"type": "trajectory_snapshot", "points": trajectory})
        if self._grid is not None:
            await ws.send_json({"type": "occupancy", **self._grid})
        if self._final_cloud_payload is not None:
            await ws.send_json(self._final_cloud_payload)
        if self._safe_mask_payload is not None:
            await ws.send_json(self._safe_mask_payload)
        try:
            async for message in ws:
                if message.type == web.WSMsgType.TEXT and message.data == "ping": await ws.send_str("pong")
        finally:
            self._ws_clients.discard(ws)
            self._websocket_client_count = len(self._ws_clients)
        return ws

    async def _pose_ws(self, request):
        ws = web.WebSocketResponse(heartbeat=10); await ws.prepare(request)
        self._pose_ws_clients.add(ws)
        self._pose_websocket_client_count = len(self._pose_ws_clients)
        try:
            async for message in ws:
                if message.type == web.WSMsgType.TEXT and message.data == "ping":
                    await ws.send_str("pong")
        finally:
            self._pose_ws_clients.discard(ws)
            self._pose_websocket_client_count = len(self._pose_ws_clients)
        return ws

    def _broadcast(self, payload: dict) -> None:
        if payload.get("type") == "pose":
            if self._pose_websocket_client_count and self._loop.is_running():
                self._loop.call_soon_threadsafe(self._enqueue_pose, payload)
            return
        if self._websocket_client_count == 0 or not self._loop.is_running():
            return
        self._loop.call_soon_threadsafe(self._enqueue_broadcast, payload)

    def _enqueue_pose(self, payload: dict) -> None:
        self._latest_pose_outgoing = payload
        if self._pose_drain_task is None or self._pose_drain_task.done():
            self._pose_drain_task = self._loop.create_task(self._drain_pose())

    async def _drain_pose(self) -> None:
        while self._latest_pose_outgoing is not None:
            payload = self._latest_pose_outgoing
            self._latest_pose_outgoing = None
            payload["server_queue_delay_ms"] = round(
                (time.monotonic() - payload.pop("_server_receive_monotonic")) * 1000.0, 3
            )
            payload["source_age_at_server_ms"] = round(
                max(0.0, (time.time() - payload["timestamp"]) * 1000.0), 3
            )
            payload["server_send_epoch_sec"] = time.time()
            text = json.dumps(payload, separators=(",", ":"), allow_nan=False)
            dead = []
            for ws in tuple(self._pose_ws_clients):
                try:
                    await ws.send_str(text)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._pose_ws_clients.discard(ws)
            self._pose_websocket_client_count = len(self._pose_ws_clients)
            await asyncio.sleep(0)

    def _enqueue_broadcast(self, payload: dict) -> None:
        if payload.get("type") == "session_reset":
            self._outgoing.clear_ephemeral()
        self._outgoing.put(payload)
        if self._drain_task is None or self._drain_task.done():
            self._drain_task = self._loop.create_task(self._drain_broadcasts())

    async def _drain_broadcasts(self) -> None:
        while self._outgoing:
            payload = self._outgoing.pop()
            text = json.dumps(payload, separators=(",", ":"), allow_nan=False)
            if payload.get("type") == "cloud":
                self._perf["cloud_payload_bytes"] = len(text.encode("utf-8"))
            dead = []
            for ws in tuple(self._ws_clients):
                try:
                    await ws.send_str(text)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._ws_clients.discard(ws)
            self._websocket_client_count = len(self._ws_clients)
            await asyncio.sleep(0)

    def _mapping_cb(self, msg: String) -> None:
        try: mapping = json.loads(msg.data)
        except json.JSONDecodeError: return
        session_id = str(mapping.get("session_id", ""))
        with self._lock:
            self._latest["mapping"] = mapping
            if mapping.get("state") == "STARTING" and session_id and session_id != self._active_session_id:
                self._active_session_id = session_id
                self._trajectory.clear()
                self._last_pose = None
                self._last_trajectory_pose = None
                self._pose_received = 0.0
                self._pose_sequence = 0
                self._grid = None
                self._planning_grid = None
                self._safe_mask_payload = None
                self._safe_goal_cells = None
                self._grid_received = 0.0
                self._grid_revision += 1
                self._visual_grid_revision += 1
                self._last_cloud = 0.0
                self._final_cloud_payload = None
                self._loaded_final_session = ""
                self._last_lidar_slice = 0.0
                self._last_grid = 0.0
                self._clear_preview("New mapping session; previous preview discarded")
                self._broadcast({"type": "session_reset", "session_id": session_id})
            elif session_id and not self._active_session_id:
                # Attach to a mapping session already in progress when this
                # dashboard process starts after the mapping manager.
                self._active_session_id = session_id
                if self._grid is not None and not self._grid.get("session_id"):
                    self._grid["session_id"] = session_id
            if (
                mapping.get("state") == "COMPLETE"
                and session_id
                and session_id != self._loaded_final_session
            ):
                self._loaded_final_session = session_id
                threading.Thread(
                    target=self._load_final_cloud,
                    args=(session_id, str(mapping.get("db_path", ""))),
                    daemon=True,
                ).start()
        self._update_cloud_subscriptions()
        self._broadcast({"type": "mapping", "data": self._latest["mapping"]})
        if mapping.get("state") in ("STARTING", "MAPPING", "DEGRADED"):
            self._loop.call_soon_threadsafe(self._schedule_live_navigation)

    def _update_cloud_subscriptions(self) -> None:
        mapping = self._latest.get("mapping", {})
        preview_requested = bool(
            self._websocket_client_count
            and mapping.get("state") in ("STARTING", "MAPPING", "DEGRADED")
        )
        dense_requested = preview_requested and bool(mapping.get("dense_cloud_enabled"))
        occupancy_requested = preview_requested and not dense_requested
        if occupancy_requested and self._global_cloud_subscription is None:
            self._global_cloud_subscription = self.create_subscription(
                PointCloud2, "/laksa/fused_mapping/cloud_map",
                lambda msg: self._cloud_cb(msg, "occupancy_cloud"),
                qos_profile_sensor_data,
            )
        elif not occupancy_requested and self._global_cloud_subscription is not None:
            self.destroy_subscription(self._global_cloud_subscription)
            self._global_cloud_subscription = None
        if dense_requested and self._zed_cloud_subscription is None:
            self._zed_cloud_subscription = self.create_subscription(
                PointCloud2, "/zed/zed_node/point_cloud/cloud_registered",
                lambda msg: self._cloud_cb(msg, "live_rgbd_preview"),
                qos_profile_sensor_data,
            )
        elif not dense_requested and self._zed_cloud_subscription is not None:
            self.destroy_subscription(self._zed_cloud_subscription)
            self._zed_cloud_subscription = None

    def _load_final_cloud(self, session_id: str, db_path: str) -> None:
        try:
            database = Path(db_path)
            ply = database.parent.parent / "export" / "map_cloud_cloud.ply"
            payload = _read_official_ply_preview(ply, self._max_points)
            payload["session_id"] = session_id
            with self._lock:
                if session_id != self._active_session_id:
                    return
                self._final_cloud_payload = payload
                self._perf.update({
                    "cloud_source_points": payload["source_points"],
                    "cloud_sent_points": payload["sent_points"],
                    "cloud_source": payload["source"],
                    "cloud_rendered_points": payload["sent_points"],
                    "cloud_payload_bytes": payload["payload_source_bytes"],
                })
            self._broadcast(payload)
            self.get_logger().info(
                f"Loaded final optimized 3D map: {payload['source_points']} source points, "
                f"{payload['sent_points']} sent"
            )
        except Exception as error:
            self.get_logger().error(f"Could not load final optimized PLY: {error}")

    def _health_cb(self, msg: String) -> None:
        try: self._latest["health"] = json.loads(msg.data)
        except json.JSONDecodeError: return
        self._broadcast({"type": "health", "data": self._latest["health"]})

    @staticmethod
    def _structured_points(msg: PointCloud2):
        dtype = point_cloud2.dtype_from_fields(msg.fields, point_step=msg.point_step)
        count = int(msg.width) * int(msg.height)
        if msg.row_step == msg.point_step * msg.width:
            return np.frombuffer(msg.data, dtype=dtype, count=count)
        rows = []
        view = memoryview(msg.data)
        for row in range(msg.height):
            start = row * msg.row_step
            rows.append(np.frombuffer(
                view[start:start + msg.point_step * msg.width], dtype=dtype, count=msg.width
            ))
        return np.concatenate(rows) if rows else np.empty(0, dtype=dtype)

    def _mapping_source(self) -> str:
        source = self._latest.get("mapping", {}).get("mapping_source", FUSED_MAPPING)
        return source if source in MAPPING_SOURCES else FUSED_MAPPING

    def _cloud_cb(self, msg: PointCloud2, source: str) -> None:
        if self._latest.get("mapping", {}).get("state") == "COMPLETE":
            return
        desired_source = "live_rgbd_preview" if self._latest.get("mapping", {}).get("dense_cloud_enabled") else "occupancy_cloud"
        if source != desired_source:
            return
        now = time.monotonic()
        if now - self._last_cloud < self._cloud_period or self._websocket_client_count == 0:
            return
        self._last_cloud = now
        if msg.point_step <= 0:
            return
        cloud_transform = None
        output_frame = msg.header.frame_id
        if source == "live_rgbd_preview":
            try:
                cloud_transform = self._tf_buffer.lookup_transform(
                    "map", msg.header.frame_id, Time.from_msg(msg.header.stamp)
                ).transform
            except TransformException:
                # ZED can deliver the cloud just before the matching dynamic
                # map transform reaches this process.  For a non-authoritative
                # live preview, the latest transform is the safe bounded
                # fallback; the saved release cloud still uses optimized poses.
                try:
                    cloud_transform = self._tf_buffer.lookup_transform(
                        "map", msg.header.frame_id, rclpy.time.Time()
                    ).transform
                except TransformException:
                    return
            output_frame = "map"
        extract_started = time.monotonic()
        cloud = self._structured_points(msg)
        names = cloud.dtype.names or ()
        if not all(name in names for name in ("x", "y", "z")):
            return
        total = len(cloud)
        finite = np.isfinite(cloud["x"]) & np.isfinite(cloud["y"]) & np.isfinite(cloud["z"])
        finite_indices = np.flatnonzero(finite)
        extract_ms = (time.monotonic() - extract_started) * 1000.0

        sample_started = time.monotonic()
        chosen = uniform_sample_indices(len(finite_indices), self._max_points)
        selected = finite_indices[np.asarray(chosen, dtype=np.intp)] if chosen else np.empty(0, dtype=np.intp)
        sample_ms = (time.monotonic() - sample_started) * 1000.0

        pack_started = time.monotonic()
        xyz = np.column_stack((cloud["x"][selected], cloud["y"][selected], cloud["z"][selected]))
        if cloud_transform is not None:
            t = cloud_transform.translation
            q = cloud_transform.rotation
            xyz = _transform_xyz(xyz, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        points = np.round(xyz.astype(np.float64, copy=False), 3).reshape(-1).tolist()
        rgb_name = "rgb" if "rgb" in names else "rgba" if "rgba" in names else None
        if rgb_name is None:
            colors_array = np.tile(np.asarray((0.25, 0.75, 1.0)), (len(selected), 1))
        else:
            rgb_values = np.ascontiguousarray(cloud[rgb_name][selected])
            if rgb_values.dtype.kind == "f" and rgb_values.dtype.itemsize == 4:
                packed_rgb = rgb_values.view(np.uint32)
            else:
                packed_rgb = rgb_values.astype(np.uint32, copy=False)
            colors_array = np.column_stack((
                (packed_rgb >> 16) & 255,
                (packed_rgb >> 8) & 255,
                packed_rgb & 255,
            )) / 255.0
        colors = np.round(colors_array, 3).reshape(-1).tolist()
        pack_ms = (time.monotonic() - pack_started) * 1000.0
        sent = len(selected)
        self._perf.update({
            "cloud_source_points": total,
            "cloud_sent_points": sent,
            "cloud_extract_ms": extract_ms,
            "cloud_sample_ms": sample_ms,
            "cloud_pack_ms": pack_ms,
            "cloud_source": source,
            "cloud_rendered_points": sent,
        })
        self._broadcast({
            "type": "cloud", "frame": output_frame,
            "source": source,
            "source_points": total, "sent_points": sent,
            "downsampled": sent < total,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9,
            "payload_source_bytes": len(msg.data),
            "points": points, "colors": colors,
        })

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        now = time.monotonic()
        origin = msg.info.origin
        with self._lock:
            grid = {"frame": msg.header.frame_id, "session_id": self._active_session_id, "width": msg.info.width, "height": msg.info.height, "resolution": msg.info.resolution, "origin": [origin.position.x, origin.position.y, origin.position.z, origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w], "data": list(msg.data)}
            previous = self._grid
            unchanged = previous is not None and all(
                previous.get(key) == grid.get(key)
                for key in ("frame", "session_id", "width", "height", "resolution", "origin", "data")
            )
            if unchanged:
                self._grid_received = now
                return
            self._visual_grid_revision += 1
            grid["revision"] = self._visual_grid_revision
            self._grid = grid
            self._grid_received = now
        if self._websocket_client_count:
            self._broadcast({"type": "occupancy", **grid})

    def _lidar_scan_cb(self, msg: LaserScan) -> None:
        if self._websocket_client_count == 0 or not msg.header.frame_id:
            return
        now = time.monotonic()
        if now - self._last_lidar_slice < 0.2:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
            )
        except TransformException:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        points = project_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max,
            (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
        )
        self._last_lidar_slice = now
        self._broadcast({
            "type": "lidar_slice", "frame": "map",
            "source_frame": msg.header.frame_id,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9,
            "point_count": len(points) // 3, "points": points,
        })

    def _planning_grid_cb(self, msg: OccupancyGrid) -> None:
        origin = msg.info.origin
        grid = {"frame": msg.header.frame_id, "session_id": self._active_session_id, "width": msg.info.width, "height": msg.info.height, "resolution": msg.info.resolution, "origin": [origin.position.x, origin.position.y, origin.position.z, origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w], "data": list(msg.data)}
        start_item = None
        with self._lock:
            previous = self._planning_grid
            unchanged = previous is not None and all(
                previous.get(key) == grid.get(key)
                for key in ("frame", "session_id", "width", "height", "resolution", "origin", "data")
            )
            if unchanged:
                return
            geometry = self._planning_geometry["values"]
            radius = circumscribed_radius(
                float(geometry["front_m"]), float(geometry["rear_m"]),
                float(geometry["left_m"]), float(geometry["right_m"]),
            )
            self._planning_grid = grid
            self._grid_revision += 1
            revision = self._grid_revision
            request = {
                "revision": revision, "frame": grid["frame"], "session_id": grid["session_id"],
                "width": grid["width"], "height": grid["height"], "resolution": grid["resolution"],
                "origin": grid["origin"], "radius": radius, "data": grid["data"],
            }
            start_item = self._safe_mask_gate.request(revision, request)
        if start_item is not None:
            self._start_safe_mask_job(start_item[1])

    def _local_costmap_cb(self, msg: Costmap) -> None:
        metadata = msg.metadata
        if (
            int(metadata.size_x) > 0
            and int(metadata.size_y) > 0
            and math.isfinite(float(metadata.resolution))
            and float(metadata.resolution) > 0.0
        ):
            self._local_costmap_received = time.monotonic()

    def _start_safe_mask_job(self, request: dict) -> None:
        future = self._safe_mask_worker.submit(_compute_safe_mask, request)
        future.add_done_callback(lambda done, current=request: self._safe_mask_done(done, current))

    def _safe_mask_done(self, future, request: dict) -> None:
        try:
            result = future.result()
        except Exception as error:
            result = None
            self.get_logger().error(f"Safe-goal mask computation failed: {error}")
        payload = None
        with self._lock:
            publish_completed, next_item = self._safe_mask_gate.complete(request["revision"])
            if result is not None and publish_completed and request["revision"] == self._grid_revision:
                payload = {
                    "type": "safe_goal_mask",
                    "visible": True,
                    "frame": request["frame"], "session_id": request["session_id"],
                    "width": request["width"], "height": request["height"],
                    "resolution": request["resolution"], "origin": request["origin"],
                    "revision": request["revision"], "encoding": "bitset-lsb-base64",
                    "circumscribed_radius_m": request["radius"],
                    "safe_cells": result["safe_cells"], "mask": result["mask"],
                    "reason": "" if result["safe_cells"] else "Map still initializing; no collision-free goal area is available yet",
                }
                self._safe_mask_payload = payload
                self._safe_goal_cells = int(result["safe_cells"])
                self._perf.update({
                    "safe_mask_revision": request["revision"],
                    "safe_mask_compute_ms": result["compute_ms"],
                    "safe_mask_backend": result["backend"],
                })
        if payload is not None:
            self._broadcast(payload)
            with self._lock:
                if result["safe_cells"] == 0 and self._planning["state"] not in (
                    "PLANNING", "PATH_READY", "STARTING_ROUTE", "FOLLOWING"
                ):
                    self._set_planning_state(
                        "WAITING_FOR_SAFE_AREA", payload["reason"],
                        goal=None, path=[], planning_time_ms=None, path_length_m=None,
                    )
                elif result["safe_cells"] > 0 and self._planning["state"] == "WAITING_FOR_SAFE_AREA":
                    base = self._planning_base_state()
                    self._set_planning_state(
                        base,
                        "Tap inside a shaded safe goal area" if base == "READY" else self._planning.get("message", ""),
                    )
        if next_item is not None:
            self._start_safe_mask_job(next_item[1])

    def _publish_pose(self, odom_sample) -> None:
        try:
            map_to_odom = self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        except TransformException: return
        position, orientation, stamp_parts = odom_sample
        translation, quaternion = _compose_transform_pose(map_to_odom, position, orientation)
        pose = translation + quaternion
        now = time.monotonic()
        trajectory_point = None
        with self._lock:
            self._pose_received = now
            self._pose_sequence += 1
            sequence = self._pose_sequence
            if self._last_trajectory_pose is None or math.dist(pose[:3], self._last_trajectory_pose[:3]) > 0.025:
                trajectory_point = pose[:3]
                self._trajectory.append(trajectory_point)
                self._last_trajectory_pose = pose
            self._last_pose = pose
        if trajectory_point is not None:
            self._broadcast({"type": "trajectory_append", "point": trajectory_point})
        stamp_sec, stamp_nanosec = stamp_parts
        map_stamp = map_to_odom.header.stamp
        self._broadcast({
            "type": "pose", "sequence": sequence,
            "timestamp": stamp_sec + stamp_nanosec * 1.0e-9,
            "map_correction_timestamp": map_stamp.sec + map_stamp.nanosec * 1.0e-9,
            "server_receive_epoch_sec": time.time(),
            "_server_receive_monotonic": time.monotonic(),
            "pose": pose,
        })

    def _performance_tick(self) -> None:
        pose_age_ms = -1.0 if not self._pose_received else (time.monotonic() - self._pose_received) * 1000.0
        pending = self._safe_mask_gate.pending
        pending_revision = pending[0] if pending is not None else 0
        self.get_logger().info(
            "FIELD_LAB_PERF "
            f"latest_pose_age_ms={pose_age_ms:.1f} pose_received={self._pose_sequence} "
            f"pose_coalesced={self._outgoing.coalesced['pose']} trajectory_points={len(self._trajectory)} "
            f"safe_mask_revision={self._perf['safe_mask_revision']} "
            f"safe_mask_compute_ms={self._perf['safe_mask_compute_ms']:.1f} "
            f"safe_mask_pending_revision={pending_revision} "
            f"safe_mask_revisions_coalesced={self._safe_mask_gate.coalesced} "
            f"websocket_clients={self._websocket_client_count} "
            f"pose_websocket_clients={self._pose_websocket_client_count}"
        )
        self.get_logger().info(
            "SAFE_GOAL_MASK_REFRESH "
            f"revision={self._perf['safe_mask_revision']} "
            f"compute_ms={self._perf['safe_mask_compute_ms']:.1f} "
            f"backend={self._perf['safe_mask_backend']}"
        )
        self.get_logger().info(
            "CLOUD_PREVIEW "
            f"source={self._perf['cloud_source']} "
            f"source_points={self._perf['cloud_source_points']} "
            f"sent_points={self._perf['cloud_sent_points']} "
            f"rendered_points={self._perf['cloud_rendered_points']} "
            f"extract_ms={self._perf['cloud_extract_ms']:.1f} "
            f"sample_ms={self._perf['cloud_sample_ms']:.1f} "
            f"pack_ms={self._perf['cloud_pack_ms']:.1f} "
            f"payload_bytes={self._perf['cloud_payload_bytes']}"
        )

    def destroy_node(self):
        self._request_disarm("Field Lab shutdown")
        self._terminate_navigation()
        self._safe_mask_worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = CockpitServer()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
