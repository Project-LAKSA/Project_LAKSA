#!/usr/bin/env python3
"""Read-only robot telemetry and mapping lifecycle web gateway."""

from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path
import struct
import threading
import time

from aiohttp import web
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class CockpitServer(Node):
    def __init__(self) -> None:
        super().__init__("laksa_mapping_cockpit")
        self.declare_parameter("port", 8090)
        self.declare_parameter("max_cloud_points", 10000)
        self.declare_parameter("cloud_period_sec", 1.0)
        self._port = int(self.get_parameter("port").value)
        self._max_points = int(self.get_parameter("max_cloud_points").value)
        self._cloud_period = float(self.get_parameter("cloud_period_sec").value)
        self._web_root = Path(get_package_share_directory("laksa_dashboard")) / "web"
        self._loop = asyncio.new_event_loop()
        self._ws_clients = set()
        self._latest = {"mapping": {"state": "IDLE"}, "health": {}}
        self._last_cloud = 0.0
        self._last_grid = 0.0
        self._trajectory = []
        self._last_pose = None
        qos = QoSProfile(depth=1); qos.reliability = ReliabilityPolicy.RELIABLE; qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/laksa/mapping/state", self._mapping_cb, qos)
        self.create_subscription(String, "/laksa/health/summary", self._health_cb, qos)
        self.create_subscription(PointCloud2, "/zed_rtabmap/cloud_map", self._cloud_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/zed_rtabmap/map", self._grid_cb, 10)
        self._profile_pub = self.create_publisher(String, "/laksa/mapping/profile_request", 10)
        self._record_pub = self.create_publisher(Bool, "/laksa/mapping/record_svo_request", 10)
        self._start_client = self.create_client(Trigger, "/laksa/mapping/start")
        self._stop_client = self.create_client(Trigger, "/laksa/mapping/stop")
        self._tf_buffer = Buffer(); self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._pose_tick)
        self._thread = threading.Thread(target=self._run_web, daemon=True); self._thread.start()

    def _run_web(self) -> None:
        asyncio.set_event_loop(self._loop)
        app = web.Application(client_max_size=2 * 1024 * 1024)
        app.router.add_get("/", self._index)
        app.router.add_get("/api/state", self._api_state)
        app.router.add_post("/api/mapping/start", self._api_start)
        app.router.add_post("/api/mapping/stop", self._api_stop)
        app.router.add_get("/ws", self._ws)
        # Colcon --symlink-install deliberately exposes package assets through
        # symlinks; aiohttp must be allowed to follow only within these roots.
        app.router.add_static("/static", self._web_root, follow_symlinks=True)
        app.router.add_static("/vendor", self._web_root / "vendor", follow_symlinks=True)
        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, "0.0.0.0", self._port)
        self._loop.run_until_complete(site.start())
        self.get_logger().info(f"LAKSA mapping cockpit listening on http://0.0.0.0:{self._port}")
        self._loop.run_forever()

    async def _index(self, _request): return web.FileResponse(self._web_root / "index.html")
    async def _api_state(self, _request): return web.json_response(self._latest)

    async def _api_start(self, request):
        body = await request.json()
        profile = str(body.get("profile", "indoor_live"))
        if profile not in ("indoor_live", "indoor_high_quality", "outdoor_structured", "outdoor_open_field"):
            raise web.HTTPBadRequest(text="Unknown profile")
        self._profile_pub.publish(String(data=profile))
        self._record_pub.publish(Bool(data=bool(body.get("record_svo", False))))
        await asyncio.sleep(0.15)
        result = await self._call_service(self._start_client)
        return web.json_response(result, status=200 if result["success"] else 409)

    async def _api_stop(self, _request):
        result = await self._call_service(self._stop_client)
        return web.json_response(result, status=200 if result["success"] else 409)

    async def _call_service(self, client):
        if not client.service_is_ready(): return {"success": False, "message": "Mapping manager service unavailable"}
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline: await asyncio.sleep(0.03)
        if not future.done(): return {"success": False, "message": "Mapping manager timeout"}
        try:
            response = future.result(); return {"success": bool(response.success), "message": response.message}
        except Exception as exc: return {"success": False, "message": str(exc)}

    async def _ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20); await ws.prepare(request); self._ws_clients.add(ws)
        await ws.send_json({"type": "snapshot", "data": self._latest})
        try:
            async for message in ws:
                if message.type == web.WSMsgType.TEXT and message.data == "ping": await ws.send_str("pong")
        finally: self._ws_clients.discard(ws)
        return ws

    def _broadcast(self, payload: dict) -> None:
        if not self._ws_clients or not self._loop.is_running(): return
        asyncio.run_coroutine_threadsafe(self._broadcast_async(payload), self._loop)

    async def _broadcast_async(self, payload):
        dead = []
        text = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        for ws in tuple(self._ws_clients):
            try: await ws.send_str(text)
            except Exception: dead.append(ws)
        for ws in dead: self._ws_clients.discard(ws)

    def _mapping_cb(self, msg: String) -> None:
        try: self._latest["mapping"] = json.loads(msg.data)
        except json.JSONDecodeError: return
        self._broadcast({"type": "mapping", "data": self._latest["mapping"]})

    def _health_cb(self, msg: String) -> None:
        try: self._latest["health"] = json.loads(msg.data)
        except json.JSONDecodeError: return
        self._broadcast({"type": "health", "data": self._latest["health"]})

    @staticmethod
    def _field_map(msg): return {field.name: field.offset for field in msg.fields}

    def _cloud_cb(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_cloud < self._cloud_period or not self._ws_clients: return
        self._last_cloud = now
        fields = self._field_map(msg)
        if not all(key in fields for key in ("x", "y", "z")) or msg.point_step <= 0: return
        total = len(msg.data) // msg.point_step
        stride = max(1, math.ceil(total / self._max_points))
        points = []; colors = []
        rgb_offset = fields.get("rgb", fields.get("rgba"))
        for index in range(0, total, stride):
            base = index * msg.point_step
            x = struct.unpack_from("<f", msg.data, base + fields["x"])[0]
            y = struct.unpack_from("<f", msg.data, base + fields["y"])[0]
            z = struct.unpack_from("<f", msg.data, base + fields["z"])[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)): continue
            points.extend((round(x, 3), round(y, 3), round(z, 3)))
            if rgb_offset is not None:
                packed = struct.unpack_from("<I", msg.data, base + rgb_offset)[0]
                colors.extend((((packed >> 16) & 255) / 255.0, ((packed >> 8) & 255) / 255.0, (packed & 255) / 255.0))
            else: colors.extend((0.25, 0.75, 1.0))
        self._broadcast({"type": "cloud", "frame": msg.header.frame_id, "points": points, "colors": colors})

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        now = time.monotonic()
        if now - self._last_grid < 1.0 or not self._ws_clients: return
        self._last_grid = now
        origin = msg.info.origin
        self._broadcast({"type": "occupancy", "frame": msg.header.frame_id, "width": msg.info.width, "height": msg.info.height, "resolution": msg.info.resolution, "origin": [origin.position.x, origin.position.y, origin.position.z, origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w], "data": list(msg.data)})

    def _pose_tick(self) -> None:
        if not self._ws_clients: return
        try: transform = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException: return
        t = transform.transform.translation; q = transform.transform.rotation
        pose = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
        if self._last_pose is None or math.dist(pose[:3], self._last_pose[:3]) > 0.025:
            self._trajectory.append(pose[:3]); self._trajectory = self._trajectory[-1000:]; self._last_pose = pose
        self._broadcast({"type": "pose", "pose": pose, "trajectory": self._trajectory})


def main(args=None):
    rclpy.init(args=args); node = CockpitServer()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
