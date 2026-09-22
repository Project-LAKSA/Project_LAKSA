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
import time
from pathlib import Path

from aiohttp import web
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from laksa_control_math import map_geometry_is_sane
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Bool, Empty, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


VESC_FAULT_NAMES = (
    "NONE",
    "OVER_VOLTAGE",
    "UNDER_VOLTAGE",
    "DRV",
    "ABS_OVER_CURRENT",
    "OVER_TEMP_FET",
    "OVER_TEMP_MOTOR",
    "GATE_DRIVER_OVER_VOLTAGE",
    "GATE_DRIVER_UNDER_VOLTAGE",
    "MCU_UNDER_VOLTAGE",
    "WATCHDOG_RESET",
    "ENCODER_SPI",
    "ENCODER_SINCOS_LOW",
    "ENCODER_SINCOS_HIGH",
    "FLASH_CORRUPTION",
    "CURRENT_SENSOR_1_OFFSET",
    "CURRENT_SENSOR_2_OFFSET",
    "CURRENT_SENSOR_3_OFFSET",
    "UNBALANCED_CURRENTS",
    "BRAKE_DRIVER",
    "RESOLVER_LOT",
    "RESOLVER_DOS",
    "RESOLVER_LOS",
    "APP_CONFIG_FLASH_CORRUPTION",
    "MOTOR_CONFIG_FLASH_CORRUPTION",
    "ENCODER_NO_MAGNET",
    "ENCODER_MAGNET_TOO_STRONG",
    "PHASE_FILTER",
)


def vesc_fault_name(code: int) -> str:
    return VESC_FAULT_NAMES[code] if 0 <= code < len(VESC_FAULT_NAMES) else "UNKNOWN"


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
            "emergency_stop_reason": "",
            "exploration_status": "IDLE",
            "autonomy_health": "INITIALIZING",
            "manual_speed_erpm": 900,
            "map_reset_status": "READY",
        }
        self._map = None
        self._map_grid = None
        self._map_revision = 0
        self._camera_jpeg = None
        self._camera_stamp_ns = 0
        self._last_state_ns = 0
        self._requests = queue.Queue()
        self._goal_handle = None
        self._goal_generation = 0
        self._awaiting_fresh_map = False
        self._fresh_map_after_ns = 0
        self._map_reset_in_progress = False
        self._map_reset_generation = 0
        self._map_reset_deadline = 0.0
        self._costmap_clear_pending = set()
        self._costmap_clear_inflight = set()

        self.create_subscription(
            VehicleState, "/laksa/state", self._state_cb, sensor
        )
        self.create_subscription(Imu, "/laksa/imu/data", self._imu_cb, sensor)
        self.create_subscription(
            CompressedImage,
            "/zed/zed_node/rgb/color/rect/image/compressed",
            self._camera_cb,
            sensor,
        )
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
        self.create_subscription(
            String,
            "/laksa/emergency_stop_reason",
            self._emergency_stop_reason_cb,
            latched,
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
        self._map_reset_interlock = self.create_publisher(
            Bool, "/laksa/map_reset_in_progress", latched
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
        self.create_timer(0.5, self._reset_housekeeping)
        self._publish_map_reset_interlock(False)

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
                "fault_name": vesc_fault_name(int(v.fault_code)),
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

    def _camera_cb(self, msg: CompressedImage) -> None:
        # The ZED ROS wrapper already performs JPEG compression. Keeping the
        # encoded frame avoids an extra OpenCV/cv_bridge copy in the dashboard.
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if now_ns - self._camera_stamp_ns < 200_000_000:
                return
            self._camera_jpeg = bytes(msg.data)
            self._camera_stamp_ns = now_ns

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

    def _emergency_stop_reason_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["emergency_stop_reason"] = msg.data

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
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        with self._lock:
            if self._map_reset_in_progress:
                # Ignore every map until systemd has replaced the complete
                # LiDAR/RF2O/SLAM process. Once armed, accept only a map whose
                # source timestamp belongs to the new process generation.
                if not self._awaiting_fresh_map:
                    return
                if stamp_ns < self._fresh_map_after_ns:
                    return

        if not map_geometry_is_sane(
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
        ):
            with self._lock:
                self._map = None
                self._map_grid = None
                self._snapshot["clear_map"] = True
                self._snapshot["map_reset_status"] = (
                    "ERROR: SLAM MAP GEOMETRY IS INVALID"
                )
            self.get_logger().error(
                "Rejected physically implausible SLAM map geometry",
                throttle_duration_sec=2.0,
            )
            return

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
            self._snapshot["clear_map"] = False
            if str(self._snapshot.get("map_reset_status", "")).startswith(
                "ERROR: SLAM MAP GEOMETRY"
            ):
                self._snapshot["map_reset_status"] = "READY"
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
        self._costmap_clear_pending = {"local", "global"}
        self._costmap_clear_inflight.clear()
        self._try_clear_navigation_costmaps()

    def _try_clear_navigation_costmaps(self) -> None:
        clients = {
            "local": self._clear_local_costmap,
            "global": self._clear_global_costmap,
        }
        for label in tuple(self._costmap_clear_pending):
            if label in self._costmap_clear_inflight:
                continue
            client = clients[label]
            if not client.service_is_ready():
                continue
            self._costmap_clear_inflight.add(label)
            future = client.call_async(ClearEntireCostmap.Request())
            future.add_done_callback(
                lambda completed, name=label: self._costmap_clear_response(
                    completed, name
                )
            )

    def _costmap_clear_response(self, future, label: str) -> None:
        self._costmap_clear_inflight.discard(label)
        if label not in self._costmap_clear_pending:
            # A reset timeout or a newer reset generation invalidated this
            # asynchronous response.
            return
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(
                f"{label.capitalize()} costmap clear failed; retrying: {error}"
            )
            return
        self._costmap_clear_pending.discard(label)
        if self._costmap_clear_pending:
            return
        with self._lock:
            self._snapshot["map_reset_status"] = "READY"
        self._map_reset_in_progress = False
        self._publish_map_reset_interlock(False)
        self.get_logger().warn(
            "Fresh SLAM map received and both navigation costmaps were cleared"
        )

    def _publish_map_reset_interlock(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self._map_reset_interlock.publish(message)

    def _reset_housekeeping(self) -> None:
        if not self._map_reset_in_progress:
            return
        if self._costmap_clear_pending:
            self._try_clear_navigation_costmaps()
        if self._map_reset_deadline and time.monotonic() > self._map_reset_deadline:
            self._set_map_reset_error("timed out waiting for fresh mapping data")

    def _set_map_reset_error(self, reason: str) -> None:
        self._map_reset_generation += 1
        self._awaiting_fresh_map = False
        self._costmap_clear_pending.clear()
        self._costmap_clear_inflight.clear()
        self._map_reset_deadline = 0.0
        with self._lock:
            self._snapshot["map_reset_status"] = f"ERROR: {reason.upper()}"
        # A failed maintenance transaction must never trap the operator in an
        # unrecoverable interlock. Return to MANUAL; the ordinary telemetry,
        # VESC-fault, and neutral-throttle gates still prevent unsafe motion.
        self._map_reset_in_progress = False
        self._publish_map_reset_interlock(False)
        self._set_exploration_enabled(False)
        self._set_nav_enabled(False)
        self.get_logger().error(f"Map reset failed: {reason}")

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
        # Invalidate both accepted and not-yet-accepted action requests. A late
        # acceptance callback will see the stale generation and cancel its own
        # handle instead of resurrecting motion after B/X/dashboard cancel.
        self._goal_generation += 1
        handle = self._goal_handle
        self._goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:  # action transport may already be gone
                self.get_logger().warning(f"Goal cancel request failed: {error}")
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
                self._begin_map_reset()
            elif request[0] == "map_reset_ready":
                self._arm_fresh_map_wait(*request[1:])
            elif request[0] == "map_reset_error":
                generation, reason = request[1:]
                if generation == self._map_reset_generation:
                    self._set_map_reset_error(reason)

    def _begin_map_reset(self) -> None:
        with self._lock:
            current_status = self._snapshot.get("map_reset_status", "READY")
        if self._map_reset_in_progress and not current_status.startswith("ERROR"):
            self.get_logger().warning("Ignoring duplicate map reset request")
            return

        self._map_reset_generation += 1
        generation = self._map_reset_generation
        self._map_reset_in_progress = True
        self._awaiting_fresh_map = False
        self._fresh_map_after_ns = 0
        self._costmap_clear_pending.clear()
        self._costmap_clear_inflight.clear()
        self._map_reset_deadline = time.monotonic() + 35.0
        self._publish_map_reset_interlock(True)
        self._set_exploration_enabled(False)
        self._cancel_goal("map reset")
        with self._lock:
            self._snapshot["home"] = {"set": False}
            self._snapshot["map_reset_status"] = "STOPPING VEHICLE"
            self._snapshot["clear_map"] = True
            self._snapshot["path"] = []
            self._map = None
            self._map_grid = None
        threading.Thread(
            target=self._request_map_reset,
            args=(generation,),
            daemon=True,
        ).start()

    def _vehicle_is_stopped(self) -> bool:
        with self._lock:
            vehicle = dict(self._snapshot.get("vehicle", {}))
            connected = bool(self._snapshot.get("connected"))
        if not connected:
            return False
        speed = vehicle.get("speed_mps")
        measured_erpm = vehicle.get("measured_erpm")
        if speed is None or measured_erpm is None:
            return False
        return abs(float(speed)) <= 0.03 and abs(float(measured_erpm)) <= 80.0

    @staticmethod
    def _mapping_main_pid() -> int:
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
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(detail or "systemd did not return a mapping PID")
        pid = int(result.stdout.strip())
        if pid <= 1:
            raise RuntimeError("mapping service has no valid MainPID")
        return pid

    @staticmethod
    def _validate_mapping_pid(pid: int) -> None:
        if os.stat(f"/proc/{pid}").st_uid != os.getuid():
            raise RuntimeError("mapping service is not owned by dashboard user")
        with open(f"/proc/{pid}/cgroup", encoding="utf-8") as cgroup_file:
            cgroup = cgroup_file.read()
        if "laksa-lidar-mapping.service" not in cgroup:
            raise RuntimeError("MainPID cgroup validation failed")

    def _request_map_reset(self, generation: int) -> None:
        try:
            with self._lock:
                self._snapshot["map_reset_status"] = "WAITING FOR STOP"
            stop_deadline = time.monotonic() + 6.0
            stopped_samples = 0
            while time.monotonic() < stop_deadline:
                if generation != self._map_reset_generation:
                    return
                if self._vehicle_is_stopped():
                    stopped_samples += 1
                    if stopped_samples >= 3:
                        break
                else:
                    stopped_samples = 0
                time.sleep(0.1)
            else:
                raise RuntimeError("vehicle did not report a stable stopped state")

            self._reset_odometry.publish(Empty())
            old_pid = self._mapping_main_pid()
            self._validate_mapping_pid(old_pid)
            with self._lock:
                self._snapshot["map_reset_status"] = "RESTARTING MAPPING"
            os.kill(old_pid, signal.SIGTERM)
            self.get_logger().warn(
                "Mapping process stopped; waiting for a new systemd generation"
            )
            stop_deadline = time.monotonic() + 3.0
            while Path(f"/proc/{old_pid}").exists() and time.monotonic() < stop_deadline:
                time.sleep(0.05)
            if Path(f"/proc/{old_pid}").exists():
                raise RuntimeError("mapping process did not stop before database reset")
            # RTAB-Map persists its graph independently of the ROS map topic.
            # Remove all SQLite files only inside the operator-confirmed reset
            # transaction, before systemd starts the next mapping generation.
            database = Path.home() / ".ros" / "laksa_rtabmap.db"
            for suffix in ("", "-shm", "-wal"):
                database.with_name(database.name + suffix).unlink(missing_ok=True)
            restart_deadline = time.monotonic() + 20.0
            new_pid = 0
            while time.monotonic() < restart_deadline:
                if generation != self._map_reset_generation:
                    return
                try:
                    candidate = self._mapping_main_pid()
                    if candidate != old_pid:
                        self._validate_mapping_pid(candidate)
                        new_pid = candidate
                        break
                except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired):
                    pass
                time.sleep(0.2)
            if new_pid <= 1:
                raise RuntimeError("systemd did not start a new mapping process")
            # System time is also ROS time on the physical Jetson. This gate
            # rejects a queued/transient map from the old publisher.
            fresh_after_ns = time.time_ns() - 250_000_000
            self._requests.put(
                ("map_reset_ready", generation, new_pid, fresh_after_ns)
            )
        except (
            OSError,
            RuntimeError,
            ValueError,
            subprocess.TimeoutExpired,
        ) as error:
            self._requests.put(("map_reset_error", generation, str(error)))

    def _arm_fresh_map_wait(
        self, generation: int, new_pid: int, fresh_after_ns: int
    ) -> None:
        if generation != self._map_reset_generation:
            return
        self._awaiting_fresh_map = True
        self._fresh_map_after_ns = fresh_after_ns
        self._map_reset_deadline = time.monotonic() + 20.0
        with self._lock:
            self._snapshot["map_reset_status"] = "WAITING FOR FRESH MAP"
        self.get_logger().warn(
            f"Mapping generation {new_pid} is running; waiting for its first map"
        )

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
        previous_handle = self._goal_handle
        if previous_handle is not None:
            try:
                previous_handle.cancel_goal_async()
            except Exception as error:  # action transport may already be gone
                self.get_logger().warning(
                    f"Previous goal cancel request failed: {error}"
                )
        self._goal_handle = None
        self._goal_generation += 1
        generation = self._goal_generation
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
        future.add_done_callback(
            lambda completed, token=generation: self._goal_response(completed, token)
        )
        with self._lock:
            self._snapshot["goal"] = {
                "active": True, "x": x, "y": y, "yaw": yaw
            }

    def _goal_response(self, future, generation: int) -> None:
        try:
            handle = future.result()
        except Exception as error:  # action transport failure
            if generation == self._goal_generation:
                self._goal_generation += 1
                self._set_nav_enabled(False)
                with self._lock:
                    self._snapshot["goal"] = {
                        "active": False,
                        "result": f"ERROR: {error}",
                    }
            return

        if generation != self._goal_generation:
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception as error:
                    self.get_logger().warning(
                        f"Stale goal cancel request failed: {error}"
                    )
            return
        if not handle.accepted:
            self._goal_generation += 1
            self._set_nav_enabled(False)
            with self._lock:
                self._snapshot["goal"] = {"active": False, "result": "rejected"}
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda completed, token=generation: self._goal_result(completed, token)
        )

    def _goal_result(self, future, generation: int) -> None:
        if generation != self._goal_generation:
            return
        try:
            status = int(future.result().status)
        except Exception as error:  # action transport failure
            result = f"ERROR: {error}"
        else:
            names = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
            }
            result = names.get(status, f"STATUS {status}")
        self._goal_generation += 1
        self._goal_handle = None
        self._set_nav_enabled(False)
        with self._lock:
            self._snapshot["goal"] = {"active": False, "result": result}

    async def _index(self, _request):
        return web.FileResponse(self._web_root / "index.html")

    async def _camera(self, _request):
        with self._lock:
            frame = self._camera_jpeg
            age_ns = self.get_clock().now().nanoseconds - self._camera_stamp_ns
        if frame is None or age_ns > 2_000_000_000:
            raise web.HTTPServiceUnavailable(text="ZED camera frame unavailable")
        return web.Response(
            body=frame,
            content_type="image/jpeg",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

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
        app.router.add_get("/camera.jpg", self._camera)
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
