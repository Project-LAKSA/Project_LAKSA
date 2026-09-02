#!/usr/bin/env python3

"""Single authority for LAKSA manual and autonomous drive commands."""

import math

import rclpy
from geometry_msgs.msg import Twist
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, Empty, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


class DriveSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("drive_supervisor")

        defaults = {
            "steering_axis": 2,
            "forward_axis": 1,
            "steering_sign": 1.0,
            "forward_sign": 1.0,
            "deadzone": 0.15,
            "a_button": 0,
            "b_button": 1,
            "x_button": 2,
            "y_button": 3,
            "auto_hold_sec": 3.0,
            "manual_speed_presets_erpm": [900, 1300, 1500, 2000, 3000],
            "default_manual_speed_erpm": 900,
            "exploration_max_erpm": 1000.0,
            "navigation_max_erpm": 1500.0,
            "motor_pole_pairs": 2.0,
            "gear_reduction": 11.82,
            "wheel_diameter_m": 0.109,
            "wheelbase_m": 0.324,
            "max_steering_rad": 0.523,
            "left_road_wheel_limit_rad": 0.523,
            "right_road_wheel_limit_rad": 0.288,
            "publish_rate_hz": 20.0,
            "joy_timeout_sec": 0.50,
            "nav_timeout_sec": 0.50,
            "state_timeout_sec": 1.0,
            "scan_timeout_sec": 0.75,
            "odom_timeout_sec": 0.75,
            "map_timeout_sec": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._steering_axis = int(self.get_parameter("steering_axis").value)
        self._forward_axis = int(self.get_parameter("forward_axis").value)
        self._steering_sign = float(self.get_parameter("steering_sign").value)
        self._forward_sign = float(self.get_parameter("forward_sign").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._a_button = int(self.get_parameter("a_button").value)
        self._b_button = int(self.get_parameter("b_button").value)
        self._x_button = int(self.get_parameter("x_button").value)
        self._y_button = int(self.get_parameter("y_button").value)
        self._auto_hold_ns = int(
            float(self.get_parameter("auto_hold_sec").value) * 1e9
        )
        self._manual_speed_presets = tuple(
            int(value)
            for value in self.get_parameter("manual_speed_presets_erpm").value
        )
        self._manual_speed_erpm = int(
            self.get_parameter("default_manual_speed_erpm").value
        )
        self._exploration_max_erpm = float(
            self.get_parameter("exploration_max_erpm").value
        )
        self._navigation_max_erpm = float(
            self.get_parameter("navigation_max_erpm").value
        )
        self._pole_pairs = float(self.get_parameter("motor_pole_pairs").value)
        self._gear_reduction = float(self.get_parameter("gear_reduction").value)
        self._wheel_diameter = float(
            self.get_parameter("wheel_diameter_m").value
        )
        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._max_steering = float(
            self.get_parameter("max_steering_rad").value
        )
        self._left_wheel_limit = float(
            self.get_parameter("left_road_wheel_limit_rad").value
        )
        self._right_wheel_limit = float(
            self.get_parameter("right_road_wheel_limit_rad").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self._joy_timeout_ns = int(
            float(self.get_parameter("joy_timeout_sec").value) * 1e9
        )
        self._nav_timeout_ns = int(
            float(self.get_parameter("nav_timeout_sec").value) * 1e9
        )
        self._state_timeout_ns = int(
            float(self.get_parameter("state_timeout_sec").value) * 1e9
        )
        self._scan_timeout_ns = int(
            float(self.get_parameter("scan_timeout_sec").value) * 1e9
        )
        self._odom_timeout_ns = int(
            float(self.get_parameter("odom_timeout_sec").value) * 1e9
        )
        self._map_timeout_ns = int(
            float(self.get_parameter("map_timeout_sec").value) * 1e9
        )
        self._vesc_telemetry_timeout_ms = self._state_timeout_ns // 1_000_000

        positive = (
            self._exploration_max_erpm,
            self._navigation_max_erpm,
            self._pole_pairs,
            self._gear_reduction,
            self._wheel_diameter,
            self._wheelbase,
            self._max_steering,
            self._left_wheel_limit,
            self._right_wheel_limit,
            publish_rate,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("Drive geometry, rates, and limits must be positive")
        if not self._manual_speed_presets or any(
            value <= 0 for value in self._manual_speed_presets
        ):
            raise ValueError("Manual speed presets must be positive")
        if self._manual_speed_erpm not in self._manual_speed_presets:
            raise ValueError("Default manual speed must be one of the presets")
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._command_pub = self.create_publisher(
            DriveCommand, "/laksa/command", 10
        )
        self._brake_pub = self.create_publisher(Bool, "/laksa/brake", 10)
        self._mode_pub = self.create_publisher(
            Bool, "/laksa/autonomous_enabled", mode_qos
        )
        self._exploration_pub = self.create_publisher(
            Bool, "/laksa/exploration_enabled", mode_qos
        )
        self._estop_pub = self.create_publisher(
            Bool, "/laksa/emergency_stop", mode_qos
        )
        self._mission_state_pub = self.create_publisher(
            String, "/laksa/mission_state", mode_qos
        )
        self._autonomy_health_pub = self.create_publisher(
            String, "/laksa/autonomy_health", mode_qos
        )
        self._cancel_navigation_pub = self.create_publisher(
            Empty, "/laksa/cancel_navigation", 10
        )
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/laksa/odom", self._odom_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, mode_qos)
        self.create_subscription(
            Twist, "/laksa/nav_cmd_vel", self._nav_callback, 10
        )
        self.create_subscription(
            VehicleState,
            "/laksa/state",
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/laksa/dashboard_navigation_enabled",
            self._dashboard_navigation_callback,
            mode_qos,
        )
        self.create_subscription(
            Bool,
            "/laksa/dashboard_exploration_enabled",
            self._dashboard_exploration_callback,
            mode_qos,
        )
        self.create_subscription(
            Bool,
            "/laksa/exploration_complete",
            self._exploration_complete_callback,
            mode_qos,
        )
        self.create_subscription(
            String,
            "/laksa/exploration_status",
            self._exploration_status_callback,
            mode_qos,
        )
        self.create_subscription(
            Int32,
            "/laksa/manual_speed_erpm",
            self._manual_speed_callback,
            mode_qos,
        )
        self.create_timer(1.0 / publish_rate, self._update)

        self._joy_axes = []
        self._buttons = []
        self._last_joy_ns = 0
        self._last_nav_ns = 0
        self._last_state_ns = 0
        self._last_scan_ns = 0
        self._last_odom_ns = 0
        self._last_map_ns = 0
        self._vesc_telemetry_ok = False
        self._vesc_fault_code = 0
        self._nav_twist = Twist()
        self._autonomous = False
        self._exploring = False
        self._a_started_ns = 0
        self._a_latched = False
        self._estop_latched = False
        self._exploration_complete = False
        self._exploration_blocked = False
        self._previous_b = False
        self._previous_x = False
        self._previous_y = False
        self._reported_reason = ""
        self._reported_autonomy_health = ""
        self._last_health_publish_ns = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publish_mode()
        self.get_logger().info(
            "MANUAL mode: left stick drives and right stick steers, "
            "hold A 3 s for exploration, "
            "X returns to manual, B latches the emergency stop, Y rearms"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _erpm_to_mps(self, erpm: float) -> float:
        circumference = math.pi * self._wheel_diameter
        return erpm * circumference / (
            60.0 * self._gear_reduction * self._pole_pairs
        )

    def _publish_mode(self) -> None:
        message = Bool()
        message.data = self._autonomous
        self._mode_pub.publish(message)
        exploration = Bool()
        exploration.data = self._exploring
        self._exploration_pub.publish(exploration)
        estop = Bool()
        estop.data = self._estop_latched
        self._estop_pub.publish(estop)
        mission = String()
        if self._estop_latched:
            mission.data = "EMERGENCY_STOP"
        elif self._exploring:
            mission.data = "EXPLORING"
        elif self._autonomous:
            mission.data = "NAVIGATING"
        elif self._exploration_complete:
            mission.data = "EXPLORATION_COMPLETE"
        elif self._exploration_blocked:
            mission.data = "EXPLORATION_BLOCKED"
        else:
            mission.data = "MANUAL"
        self._mission_state_pub.publish(mission)

    def _set_autonomous(
        self, enabled: bool, reason: str, exploring: bool = False
    ) -> None:
        exploring = enabled and exploring
        if enabled:
            ready, health = self._autonomy_ready()
            if not ready:
                self._publish_autonomy_health(health)
                self.get_logger().error(
                    f"Autonomous request rejected: {health}"
                )
                return
        if enabled == self._autonomous and exploring == self._exploring:
            return
        self._autonomous = enabled
        self._exploring = exploring
        if enabled:
            self._exploration_complete = False
            self._exploration_blocked = False
        self._publish_mode()
        if exploring:
            mode = "AUTONOMOUS EXPLORATION"
        elif enabled:
            mode = "NAVIGATE TO DASHBOARD GOAL"
        else:
            mode = "MANUAL"
        self.get_logger().warn(f"Mode -> {mode}: {reason}")

    def _dashboard_navigation_callback(self, message: Bool) -> None:
        if message.data:
            if self._estop_latched:
                self.get_logger().warning(
                    "Dashboard goal ignored while emergency stop is latched"
                )
                return
            self._set_autonomous(True, "dashboard goal", exploring=False)
        elif self._autonomous and not self._exploring:
            self._set_autonomous(False, "dashboard goal finished or canceled")

    def _dashboard_exploration_callback(self, message: Bool) -> None:
        if message.data:
            if self._estop_latched:
                self.get_logger().warning(
                    "Exploration request ignored while emergency stop is latched"
                )
                return
            self._set_autonomous(True, "dashboard exploration request", exploring=True)
        elif self._exploring:
            self._set_autonomous(False, "dashboard exploration canceled")
        elif self._exploration_complete:
            self._exploration_complete = False
            self._publish_mode()
            self.get_logger().warn("Mode -> MANUAL: dashboard confirmation")

    def _exploration_complete_callback(self, message: Bool) -> None:
        if not message.data or not self._exploring:
            return
        self._autonomous = False
        self._exploring = False
        self._exploration_complete = True
        self._exploration_blocked = False
        self._publish_mode()
        self.get_logger().warn("Mode -> EXPLORATION COMPLETE")

    def _exploration_status_callback(self, message: String) -> None:
        if message.data != "BLOCKED" or not self._exploring:
            return
        self._autonomous = False
        self._exploring = False
        self._exploration_complete = False
        self._exploration_blocked = True
        self._publish_mode()
        self.get_logger().error(
            "Mode -> EXPLORATION BLOCKED; autonomous commands stopped"
        )

    def _manual_speed_callback(self, message: Int32) -> None:
        requested = int(message.data)
        if requested not in self._manual_speed_presets:
            self.get_logger().warning(
                f"Rejected unsupported manual speed preset: {requested} eRPM"
            )
            return
        self._manual_speed_erpm = requested
        self.get_logger().warn(f"Manual speed preset -> {requested} eRPM")

    def _button(self, index: int) -> bool:
        return 0 <= index < len(self._buttons) and bool(self._buttons[index])

    def _joy_callback(self, message: Joy) -> None:
        self._last_joy_ns = self._now_ns()
        self._joy_axes = list(message.axes)
        self._buttons = list(message.buttons)

        b_pressed = self._button(self._b_button)
        y_pressed = self._button(self._y_button)
        if b_pressed and not self._previous_b:
            self._estop_latched = True
            self._exploration_complete = False
            self._set_autonomous(False, "Xbox B emergency stop")
            self._cancel_navigation_pub.publish(Empty())
            self._publish_mode()
        elif (
            y_pressed
            and not self._previous_y
            and self._estop_latched
            and not b_pressed
        ):
            self._estop_latched = False
            self._exploration_complete = False
            self._publish_mode()
            self.get_logger().warn("Emergency stop REARMED by Xbox Y; MANUAL mode")
        self._previous_b = b_pressed
        self._previous_y = y_pressed

        x_pressed = self._button(self._x_button)
        if x_pressed and not self._previous_x:
            self._exploration_complete = False
            self._exploration_blocked = False
            self._set_autonomous(False, "Xbox X pressed")
            self._cancel_navigation_pub.publish(Empty())
            self._publish_mode()
        self._previous_x = x_pressed

        a_pressed = self._button(self._a_button)
        if not a_pressed:
            self._a_started_ns = 0
            self._a_latched = False
        elif (
            not self._autonomous
            and not self._estop_latched
            and not self._a_latched
        ):
            if self._a_started_ns == 0:
                self._a_started_ns = self._last_joy_ns
                self.get_logger().info("A hold started")
            elif self._last_joy_ns - self._a_started_ns >= self._auto_hold_ns:
                self._a_latched = True
                self._set_autonomous(
                    True, "Xbox A held for 3 seconds", exploring=True
                )

    def _nav_callback(self, message: Twist) -> None:
        self._last_nav_ns = self._now_ns()
        self._nav_twist = message

    def _scan_callback(self, _message: LaserScan) -> None:
        self._last_scan_ns = self._now_ns()

    def _odom_callback(self, _message: Odometry) -> None:
        self._last_odom_ns = self._now_ns()

    def _map_callback(self, _message: OccupancyGrid) -> None:
        self._last_map_ns = self._now_ns()

    def _state_callback(self, message: VehicleState) -> None:
        self._last_state_ns = self._now_ns()
        self._vesc_telemetry_ok = (
            int(message.vesc.telemetry_sequence) > 0
            and int(message.vesc.telemetry_age_ms)
            <= self._vesc_telemetry_timeout_ms
        )
        self._vesc_fault_code = int(message.vesc.fault_code)

    def _axis(self, index: int) -> float:
        if 0 <= index < len(self._joy_axes):
            return float(self._joy_axes[index])
        return 0.0

    def _autonomy_ready(self):
        now_ns = self._now_ns()
        checks = (
            (self._last_state_ns, self._state_timeout_ns, "ESP32 state is stale"),
            (self._last_odom_ns, self._odom_timeout_ns, "odometry is stale"),
            (self._last_scan_ns, self._scan_timeout_ns, "LiDAR scan is stale"),
            (self._last_map_ns, self._map_timeout_ns, "SLAM map is stale"),
        )
        for stamp, timeout, failure in checks:
            if stamp == 0 or now_ns - stamp > timeout:
                return False, failure
        if not self._vesc_telemetry_ok:
            return False, "VESC telemetry is stale"
        if self._vesc_fault_code != 0:
            return False, f"VESC fault code {self._vesc_fault_code}"
        try:
            self._tf_buffer.lookup_transform(
                "map",
                "laksa_base_footprint",
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            return False, f"localization transform is unavailable: {error}"
        return True, "READY"

    def _publish_autonomy_health(self, health: str) -> None:
        now_ns = self._now_ns()
        if (
            health == self._reported_autonomy_health
            and now_ns - self._last_health_publish_ns < 1_000_000_000
        ):
            return
        self._reported_autonomy_health = health
        self._last_health_publish_ns = now_ns
        message = String()
        message.data = health
        self._autonomy_health_pub.publish(message)

    def _manual_command(self, joy_fresh: bool) -> DriveCommand:
        command = DriveCommand()
        if not joy_fresh:
            return command

        steering_input = self._axis(self._steering_axis) * self._steering_sign
        steering_input = max(-1.0, min(1.0, steering_input))
        command.steering_angle_rad = steering_input * self._max_steering

        throttle = self._axis(self._forward_axis) * self._forward_sign
        throttle = max(-1.0, min(1.0, throttle))
        if abs(throttle) > self._deadzone:
            command.speed_mps = math.copysign(
                self._erpm_to_mps(self._manual_speed_erpm), throttle
            )
        return command

    def _autonomous_command(self, nav_fresh: bool) -> DriveCommand:
        command = DriveCommand()
        if not nav_fresh:
            return command

        speed = float(self._nav_twist.linear.x)
        mode_limit_erpm = (
            self._exploration_max_erpm
            if self._exploring
            else self._navigation_max_erpm
        )
        mode_limit = self._erpm_to_mps(mode_limit_erpm)
        speed = max(-mode_limit, min(mode_limit, speed))
        command.speed_mps = speed
        if abs(speed) > 0.01:
            road_wheel_steering = math.atan(
                self._wheelbase * float(self._nav_twist.angular.z) / speed
            )
            physical_limit = (
                self._left_wheel_limit
                if road_wheel_steering >= 0.0
                else self._right_wheel_limit
            )
            road_wheel_steering = max(
                -physical_limit, min(physical_limit, road_wheel_steering)
            )
            # Convert the desired road-wheel angle into the ESP32's normalized
            # servo-angle convention. The measured left and right endpoints
            # are asymmetric, while Nav2's Ackermann model is symmetric.
            command.steering_angle_rad = (
                road_wheel_steering * self._max_steering / physical_limit
            )
        return command

    def _update(self) -> None:
        now_ns = self._now_ns()
        joy_fresh = (
            self._last_joy_ns != 0
            and now_ns - self._last_joy_ns <= self._joy_timeout_ns
        )
        nav_fresh = (
            self._last_nav_ns != 0
            and now_ns - self._last_nav_ns <= self._nav_timeout_ns
        )
        state_fresh = (
            self._last_state_ns != 0
            and now_ns - self._last_state_ns <= self._state_timeout_ns
        )
        autonomy_ready, autonomy_health = self._autonomy_ready()
        self._publish_autonomy_health(autonomy_health)

        if self._autonomous:
            command = self._autonomous_command(nav_fresh)
        else:
            command = self._manual_command(joy_fresh)

        reason = ""
        if self._estop_latched:
            reason = "Emergency stop latched; press Xbox Y to rearm"
            command.steering_angle_rad = 0.0
        elif self._exploration_complete:
            reason = "Exploration complete; press Xbox X for manual mode"
            command.steering_angle_rad = 0.0
        elif not state_fresh:
            reason = "ESP32 state timeout"
        elif not self._vesc_telemetry_ok:
            reason = "VESC telemetry stale"
        elif self._vesc_fault_code != 0:
            reason = f"VESC fault code {self._vesc_fault_code}"
        elif self._autonomous and not autonomy_ready:
            reason = f"Autonomy unavailable: {autonomy_health}"
        elif not joy_fresh:
            # The Xbox controller is the operator's brake/mode escape device.
            # Never allow autonomous motion if that safety link disappears.
            reason = "Xbox controller timeout"
        if reason:
            command.speed_mps = 0.0
            command.brake = True
            if reason != self._reported_reason:
                self.get_logger().warn(reason)
        self._reported_reason = reason
        brake = Bool()
        brake.data = command.brake
        self._brake_pub.publish(brake)
        self._command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
