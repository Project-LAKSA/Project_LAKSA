#!/usr/bin/env python3
"""Exercise LAKSA arbitration in an isolated domain with actuation disabled."""

import argparse
import json
import math
import struct
import threading
import time

from geometry_msgs.msg import Twist
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool


class SafetyFixture(Node):
    def __init__(self):
        super().__init__("laksa_supervisor_safety_fixture")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.nav_pub = self.create_publisher(Twist, "/laksa/nav_cmd_vel", 10)
        self.state_pub = self.create_publisher(VehicleState, "/laksa/state", qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, "/laksa/lidar/scan_validated", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/laksa/odometry/fused", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.costmap_pub = self.create_publisher(Costmap, "/local_costmap/costmap_raw", 10)
        self.create_subscription(DriveCommand, "/laksa/command", self._output_cb, 10)
        self.create_subscription(DriveCommand, "/laksa/autonomy_candidate_command", self._candidate_cb, 10)
        self.create_subscription(Bool, "/laksa/autonomous_enabled", self._mode_cb, latched)
        self.create_subscription(Empty, "/laksa/cancel_navigation", self._cancel_cb, 10)
        self.arm_client = self.create_client(SetBool, "/laksa/autonomy/set_armed")
        self.publish_nav = True
        self.override = False
        self.outputs = []
        self.candidates = []
        self.autonomous = False
        self.cancel_times = []
        self.mode_times = []
        self.create_timer(0.05, self._publish)

    def _output_cb(self, msg):
        self.outputs.append((time.monotonic(), msg.speed_mps, msg.steering_angle_rad, msg.brake))

    def _candidate_cb(self, msg):
        self.candidates.append((time.monotonic(), msg.speed_mps, msg.steering_angle_rad, msg.brake))

    def _mode_cb(self, msg):
        self.autonomous = bool(msg.data)
        self.mode_times.append((time.monotonic(), self.autonomous))

    def _cancel_cb(self, _msg):
        self.cancel_times.append(time.monotonic())

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        joy = Joy()
        joy.header.stamp = stamp
        joy.axes = [0.0, 0.60 if self.override else 0.0, 0.0]
        joy.buttons = [0, 0, 0, 0]
        self.joy_pub.publish(joy)
        state = VehicleState()
        state.stamp = stamp
        state.orientation.w = 1.0
        state.vesc.telemetry_sequence = 1
        state.vesc.telemetry_age_ms = 0
        state.vesc.telemetry_fresh = True
        self.state_pub.publish(state)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [float("inf")] * 361
        self.scan_pub.publish(scan)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base_footprint"
        cloud.height = cloud.width = 1
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = cloud.row_step = 12
        cloud.data = struct.pack("<fff", 3.0, 2.0, 0.3)
        cloud.is_dense = True
        self.cloud_pub.publish(cloud)
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = grid.info.height = 20
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 400
        self.map_pub.publish(grid)
        costmap = Costmap()
        costmap.header.stamp = stamp
        costmap.header.frame_id = "odom"
        costmap.metadata.resolution = 0.05
        costmap.metadata.size_x = costmap.metadata.size_y = 20
        costmap.metadata.origin.orientation.w = 1.0
        costmap.data = [0] * 400
        self.costmap_pub.publish(costmap)
        if self.publish_nav:
            nav = Twist()
            nav.linear.x = 0.20
            nav.angular.z = 0.10
            self.nav_pub.publish(nav)


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    rclpy.init()
    node = SafetyFixture()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if not node.arm_client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("ARM/DISARM service unavailable")
        time.sleep(1.0)
        request = SetBool.Request()
        request.data = True
        armed = node.arm_client.call_async(request)
        if not wait_future(armed, 3.0) or not armed.result().success:
            raise RuntimeError(f"ARM rejected: {armed.result().message if armed.done() else 'timeout'}")
        time.sleep(0.6)
        armed_candidates = list(node.candidates)
        dry_outputs = list(node.outputs)

        node.publish_nav = False
        watchdog_started = time.monotonic()
        time.sleep(0.40)
        watchdog_zero = next(
            (t for t, speed, _, brake in node.candidates if t >= watchdog_started and speed == 0.0 and brake),
            None,
        )

        node.publish_nav = True
        time.sleep(0.15)
        override_started = time.monotonic()
        node.override = True
        deadline = override_started + 0.5
        while node.autonomous and time.monotonic() < deadline:
            time.sleep(0.005)
        override_event = next(
            (t for t, enabled in node.mode_times if t >= override_started and not enabled),
            None,
        )
        cancel_event = next((t for t in node.cancel_times if t >= override_started), None)
        override_latency_ms = (
            (max(override_event, cancel_event) - override_started) * 1000.0
            if override_event and cancel_event else None
        )
        time.sleep(0.10)
        recent_outputs = [sample for sample in node.outputs if sample[0] >= override_started]
        report = {
            "pass": bool(
                armed_candidates
                and any(abs(speed) > 0.0 for _, speed, _, _ in armed_candidates)
                and all(abs(speed) <= 0.150001 and math.isfinite(steer) for _, speed, steer, _ in armed_candidates)
                and dry_outputs
                and all(speed == 0.0 and brake for _, speed, _, brake in dry_outputs)
                and watchdog_zero is not None
                and (watchdog_zero - watchdog_started) <= 0.30
                and override_latency_ms is not None
                and override_latency_ms < 100.0
                and not node.autonomous
                and recent_outputs
                and all(speed == 0.0 and brake for _, speed, _, brake in recent_outputs)
            ),
            "armed_candidate_count": len(armed_candidates),
            "candidate_max_speed_mps": max((abs(x[1]) for x in armed_candidates), default=0.0),
            "physical_output_samples": len(dry_outputs),
            "physical_outputs_all_zero_braked": bool(dry_outputs) and all(
                speed == 0.0 and brake for _, speed, _, brake in dry_outputs
            ),
            "watchdog_zero_latency_ms": (
                round((watchdog_zero - watchdog_started) * 1000.0, 3)
                if watchdog_zero else None
            ),
            "manual_override_latency_ms": (
                round(override_latency_ms, 3) if override_latency_ms is not None else None
            ),
            "autonomy_after_override": node.autonomous,
        }
        text = json.dumps(report, indent=2, sort_keys=True)
        print(text)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as stream:
                stream.write(text + "\n")
        raise SystemExit(0 if report["pass"] else 1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
