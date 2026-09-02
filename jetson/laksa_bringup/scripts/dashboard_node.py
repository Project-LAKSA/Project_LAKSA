#!/usr/bin/env python3

"""Dependency-light web dashboard for LAKSA diagnostics and Nav2 goals."""

import asyncio
import json
import math
import os
import queue
import signal
import subprocess
import threading
from pathlib import Path

from aiohttp import web
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("laksa_dashboard")
        self.declare_parameter("port", 8080)
        self.declare_parameter("web_root", "")
        self._port = int(self.get_parameter("port").value)
        root = str(self.get_parameter("web_root").value)
        self._web_root = Path(root) if root else Path(__file__).parent.parent / "web"

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor = QoSProfile(depth=5)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self._lock = threading.Lock()
        self._snapshot = {
            "mode": "MANUAL",
            "connected": False,
            "command": {},
            "vehicle": {},
            "imu": {},
            "pose": {},
            "goal": {"active": False},
            "home": {"set": False},
            "path": [],
            "emergency_stop": False,
            "exploration_status": "IDLE",
            "autonomy_health": "INITIALIZING",
            "manual_speed_erpm": 900,
            "map_reset_status": "READY",
        }
        self._map = None
        self._map_grid = None
        self._map_revision = 0
        self._last_state_ns = 0
        self._requests = queue.Queue()
        self._goal_handle = None
        self._awaiting_fresh_map = False

        self.create_subscription(
            VehicleState, "/laksa/state", self._state_cb, sensor
        )
        self.create_subscription(Imu, "/laksa/imu/data", self._imu_cb, sensor)
        self.create_subscription(
            DriveCommand, "/laksa/command", self._command_cb, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched)
        self.create_subscription(Odometry, "/laksa/odom", self._odom_cb, 10)
        self.create_subscription(
            String, "/laksa/mission_state", self._mission_state_cb, latched
        )
        self.create_subscription(
            String, "/laksa/autonomy_health", self._autonomy_health_cb, latched
        )
        self.create_subscription(
            String,
            "/laksa/exploration_status",
            self._exploration_status_cb,
            latched,
        )
        self.create_subscription(
            Bool, "/laksa/emergency_stop", self._emergency_stop_cb, latched
        )
        self.create_subscription(NavigationPath, "/plan", self._path_cb, 10)
        self.create_subscription(
            Empty, "/laksa/cancel_navigation", self._cancel_navigation_cb, 10
        )
        self._nav_enabled = self.create_publisher(
            Bool, "/laksa/dashboard_navigation_enabled", latched
        )
        self._exploration_enabled = self.create_publisher(
            Bool, "/laksa/dashboard_exploration_enabled", latched
        )
        self._manual_speed = self.create_publisher(
            Int32, "/laksa/manual_speed_erpm", latched
        )
        self._reset_odometry = self.create_publisher(
            Empty, "/laksa/reset_odometry", 10
        )
        self._navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.1, self._process_requests)
        self.create_timer(0.2, self._update_pose)
        self.create_timer(0.5, self._connection_watchdog)

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"LAKSA dashboard listening on all interfaces, port {self._port}"
        )

    @staticmethod
    def _v3(value):
        return {"x": value.x, "y": value.y, "z": value.z}

    @staticmethod
    def _quat(value):
        return {"x": value.x, "y": value.y, "z": value.z, "w": value.w}

    def _state_cb(self, msg: VehicleState) -> None:
        v = msg.vesc
        self._last_state_ns = self.get_clock().now().nanoseconds
        with self._lock:
            self._snapshot["connected"] = True
            self._snapshot["vehicle"] = {
                "imu_available": msg.imu_available,
                "orientation_accuracy_rad": msg.orientation_accuracy_rad,
                "steering_target_rad": msg.steering_target_rad,
                "steering_current_rad": msg.steering_current_rad,
                "endpoint_relief": msg.steering_endpoint_relief_active,
                "requested_erpm": v.requested_erpm,
                "active_erpm": v.active_erpm,
                "measured_erpm": v.measured_erpm,
                "speed_mps": v.vehicle_linear_velocity_mps,
                "motor_current_a": v.motor_current_a,
                "input_current_a": v.input_current_a,
                "input_voltage_v": v.input_voltage_v,
                "duty_cycle": v.duty_cycle,
                "temp_mosfet_c": v.temp_mosfet_c,
                "temp_motor_c": v.temp_motor_c,
                "fault_code": v.fault_code,
                "telemetry_fresh": (
                    int(v.telemetry_sequence) > 0
                    and int(v.telemetry_age_ms) <= 1000
                ),
                "telemetry_sequence": v.telemetry_sequence,
                "telemetry_age_ms": v.telemetry_age_ms,
                "command_fresh": v.command_fresh,
                "brake_active": v.brake_active,
            }

    def _connection_watchdog(self) -> None:
        fresh = (
            self._last_state_ns != 0
            and self.get_clock().now().nanoseconds - self._last_state_ns < 1_500_000_000
        )
        with self._lock:
            self._snapshot["connected"] = fresh

    def _imu_cb(self, msg: Imu) -> None:
        with self._lock:
            self._snapshot["imu"] = {
                "orientation": self._quat(msg.orientation),
                "angular_velocity": self._v3(msg.angular_velocity),
                "linear_acceleration": self._v3(msg.linear_acceleration),
            }

    def _command_cb(self, msg: DriveCommand) -> None:
        with self._lock:
            self._snapshot["command"] = {
                "speed_mps": msg.speed_mps,
                "steering_rad": msg.steering_angle_rad,
                "brake": msg.brake,
            }

    def _mission_state_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["mode"] = msg.data

    def _autonomy_health_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["autonomy_health"] = msg.data

    def _exploration_status_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["exploration_status"] = msg.data

    def _emergency_stop_cb(self, msg: Bool) -> None:
        with self._lock:
            self._snapshot["emergency_stop"] = bool(msg.data)
        if msg.data:
            self._cancel_goal("emergency stop")

    def _path_cb(self, msg: NavigationPath) -> None:
        poses = msg.poses
        stride = max(1, len(poses) // 250)
        path = [
            [float(p.pose.position.x), float(p.pose.position.y)]
            for p in poses[::stride]
        ]
        with self._lock:
            self._snapshot["path"] = path

    def _cancel_navigation_cb(self, _msg: Empty) -> None:
        self._cancel_goal("Xbox cancel")

    def _odom_cb(self, _msg: Odometry) -> None:
        pass

    def _map_cb(self, msg: OccupancyGrid) -> None:
        runs = []
        last = None
        count = 0
        for value in msg.data:
            value = int(value)
            if value == last and count < 65535:
                count += 1
            else:
                if count:
                    runs.extend((last, count))
                last, count = value, 1
        if count:
            runs.extend((last, count))
        reset_completed = False
        with self._lock:
            if self._awaiting_fresh_map:
                self._awaiting_fresh_map = False
                self._snapshot["map_reset_status"] = "CLEARING COSTMAPS"
                self._snapshot["clear_map"] = False
                reset_completed = True
            self._map_revision += 1
            self._map = {
                "revision": self._map_revision,
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "runs": runs,
            }
            self._map_grid = {
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": tuple(int(value) for value in msg.data),
            }
        if reset_completed:
            self._clear_navigation_costmaps()

    def _clear_navigation_costmaps(self) -> None:
        unavailable = []
        for label, client in (
            ("local", self._clear_local_costmap),
            ("global", self._clear_global_costmap),
        ):
            if not client.service_is_ready():
                unavailable.append(label)
                continue
            client.call_async(ClearEntireCostmap.Request())
        with self._lock:
            if unavailable:
                self._snapshot["map_reset_status"] = (
                    "ERROR: " + ", ".join(unavailable) + " COSTMAP UNAVAILABLE"
                )
            else:
                self._snapshot["map_reset_status"] = "READY"
        if unavailable:
            self.get_logger().error(
                "Map reset could not clear " + ", ".join(unavailable)
                + " costmap"
            )
        else:
            self.get_logger().warn(
                "Fresh SLAM map received; local and global costmaps cleared"
            )

    def _update_pose(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "laksa_base_footprint", Time()
            )
        except TransformException:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._snapshot["pose"] = {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "yaw": yaw,
            }

    def _set_nav_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._nav_enabled.publish(msg)

    def _set_exploration_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._exploration_enabled.publish(msg)

    def _cancel_goal(self, result: str = "canceled") -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._set_nav_enabled(False)
        with self._lock:
            self._snapshot["goal"] = {"active": False, "result": result}
            self._snapshot["path"] = []

    def _process_requests(self) -> None:
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            if request[0] == "goal":
                self._send_goal(*request[1:])
            elif request[0] == "cancel":
                self._set_exploration_enabled(False)
                self._cancel_goal()
            elif request[0] == "explore":
                with self._lock:
                    health = self._snapshot.get("autonomy_health", "INITIALIZING")
                if health != "READY":
                    self.get_logger().warning(
                        f"Exploration rejected: autonomy is not ready: {health}"
                    )
                    continue
                self._cancel_goal("exploration started")
                self._set_exploration_enabled(True)
            elif request[0] == "manual_speed":
                erpm = int(request[1])
                if erpm not in (900, 1300, 1500, 2000, 3000):
                    continue
                message = Int32()
                message.data = erpm
                self._manual_speed.publish(message)
                with self._lock:
                    self._snapshot["manual_speed_erpm"] = erpm
            elif request[0] == "set_home":
                with self._lock:
                    pose = dict(self._snapshot.get("pose", {}))
                    if "x" in pose:
                        self._snapshot["home"] = {
                            "set": True,
                            "x": pose["x"],
                            "y": pose["y"],
                            "yaw": pose["yaw"],
                        }
            elif request[0] == "go_home":
                with self._lock:
                    home = dict(self._snapshot.get("home", {}))
                if home.get("set"):
                    self._send_goal(home["x"], home["y"], home["yaw"])
            elif request[0] == "reset_map":
                self._set_exploration_enabled(False)
                self._cancel_goal("map reset")
                self._reset_odometry.publish(Empty())
                with self._lock:
                    self._snapshot["home"] = {"set": False}
                    self._snapshot["map_reset_status"] = "RESETTING"
                    self._snapshot["clear_map"] = True
                    self._snapshot["path"] = []
                    self._map = None
                    self._map_grid = None
                    self._awaiting_fresh_map = True
                threading.Thread(
                    target=self._request_map_reset, daemon=True
                ).start()

    def _request_map_reset(self) -> None:
        try:
            result = subprocess.run(
                [
                    "/usr/bin/systemctl",
                    "show",
                    "--property=MainPID",
                    "--value",
                    "laksa-lidar-mapping.service",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f"Map reset request failed: {error}")
            with self._lock:
                self._snapshot["map_reset_status"] = "ERROR"
            return
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self.get_logger().error(f"Map reset request failed: {detail}")
            with self._lock:
                self._snapshot["map_reset_status"] = "ERROR"
            return
        try:
            pid = int(result.stdout.strip())
            if pid <= 1:
                raise RuntimeError("mapping service has no valid MainPID")
            if os.stat(f"/proc/{pid}").st_uid != os.getuid():
                raise RuntimeError("mapping service is not owned by dashboard user")
            with open(f"/proc/{pid}/cgroup", encoding="utf-8") as cgroup_file:
                cgroup = cgroup_file.read()
            if "laksa-lidar-mapping.service" not in cgroup:
                raise RuntimeError("MainPID cgroup validation failed")
            os.kill(pid, signal.SIGTERM)
            self.get_logger().warn(
                "SLAM process stopped for a fresh map; systemd will restart it"
            )
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"Map reset validation failed: {error}")
            with self._lock:
                self._snapshot["map_reset_status"] = "ERROR"

    def _reject_goal(self, reason: str) -> None:
        self.get_logger().warning(f"Dashboard goal rejected: {reason}")
        with self._lock:
            self._snapshot["goal"] = {
                "active": False,
                "result": f"REJECTED: {reason}",
            }

    def _goal_is_safe(self, x: float, y: float) -> tuple[bool, str]:
        with self._lock:
            grid = self._map_grid
        if grid is None:
            return False, "map is not available"
        column = math.floor((x - grid["origin_x"]) / grid["resolution"])
        row = math.floor((y - grid["origin_y"]) / grid["resolution"])
        if not (0 <= column < grid["width"] and 0 <= row < grid["height"]):
            return False, "point is outside the current map"

        # Require a known-free disk around the destination. This keeps the
        # vehicle footprint away from walls and rejects unknown gray cells.
        clearance_cells = max(1, math.ceil(0.25 / grid["resolution"]))
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dx * dx + dy * dy > clearance_cells * clearance_cells:
                    continue
                check_x, check_y = column + dx, row + dy
                if not (
                    0 <= check_x < grid["width"]
                    and 0 <= check_y < grid["height"]
                ):
                    return False, "point is too close to the map boundary"
                occupancy = grid["data"][check_y * grid["width"] + check_x]
                if occupancy != 0:
                    return False, "point is unknown or too close to an obstacle"
        return True, ""

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self._reject_goal("coordinates are not finite")
            return
        with self._lock:
            estop = bool(self._snapshot.get("emergency_stop"))
            health = self._snapshot.get("autonomy_health", "INITIALIZING")
        if estop:
            self._reject_goal("emergency stop is latched")
            return
        if health != "READY":
            self._reject_goal(f"autonomy is not ready: {health}")
            return
        safe, reason = self._goal_is_safe(x, y)
        if not safe:
            self._reject_goal(reason)
            return
        if not self._navigate.wait_for_server(timeout_sec=0.05):
            self._reject_goal("Nav2 is unavailable")
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._set_nav_enabled(True)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._navigate.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        with self._lock:
            self._snapshot["goal"] = {
                "active": True, "x": x, "y": y, "yaw": yaw
            }

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._set_nav_enabled(False)
            with self._lock:
                self._snapshot["goal"] = {"active": False, "result": "rejected"}
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        status = int(future.result().status)
        names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        self._goal_handle = None
        self._set_nav_enabled(False)
        with self._lock:
            self._snapshot["goal"] = {
                "active": False, "result": names.get(status, f"STATUS {status}")
            }

    async def _index(self, _request):
        return web.FileResponse(self._web_root / "index.html")

    async def _ws(self, request):
        socket = web.WebSocketResponse(heartbeat=10.0)
        await socket.prepare(request)
        map_revision = -1

        async def transmit():
            nonlocal map_revision
            while not socket.closed:
                with self._lock:
                    payload = dict(self._snapshot)
                    if self._map and self._map["revision"] != map_revision:
                        payload["map"] = self._map
                        map_revision = self._map["revision"]
                await socket.send_str(json.dumps(payload, separators=(",", ":")))
                await asyncio.sleep(0.2)

        sender = asyncio.create_task(transmit())
        try:
            async for message in socket:
                if message.type != web.WSMsgType.TEXT:
                    continue
                data = json.loads(message.data)
                if data.get("type") == "goal":
                    self._requests.put(("goal", float(data["x"]),
                                        float(data["y"]), float(data["yaw"])))
                elif data.get("type") == "cancel":
                    self._requests.put(("cancel",))
                elif data.get("type") == "explore":
                    self._requests.put(("explore",))
                elif data.get("type") == "manual_speed":
                    self._requests.put(("manual_speed", int(data["erpm"])))
                elif data.get("type") == "set_home":
                    self._requests.put(("set_home",))
                elif data.get("type") == "go_home":
                    self._requests.put(("go_home",))
                elif data.get("type") == "reset_map":
                    self._requests.put(("reset_map",))
        finally:
            sender.cancel()
        return socket

    def _serve(self) -> None:
        asyncio.set_event_loop(self._loop)
        app = web.Application()
        app.router.add_get("/", self._index)
        app.router.add_get("/ws", self._ws)
        runner = web.AppRunner(app, access_log=None)

        async def start():
            await runner.setup()
            await web.TCPSite(runner, "0.0.0.0", self._port).start()

        self._loop.run_until_complete(start())
        self._loop.run_forever()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
