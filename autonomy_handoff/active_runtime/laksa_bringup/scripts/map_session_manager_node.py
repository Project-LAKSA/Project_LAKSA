#!/usr/bin/env python3

"""Reset the systemd-managed SLAM mapping session on an explicit ROS request."""

import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class MapSessionManager(Node):
    def __init__(self) -> None:
        super().__init__("map_session_manager")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status = self.create_publisher(
            String, "/laksa/map_reset_status", latched
        )
        self.create_subscription(
            Bool, "/laksa/reset_map_requested", self._reset_callback, 10
        )
        self._resetting = False
        self._publish_status("READY")

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status.publish(message)

    def _reset_callback(self, message: Bool) -> None:
        if not message.data or self._resetting:
            return
        self._resetting = True
        self._publish_status("RESETTING")
        self.get_logger().warn("Starting a fresh SLAM mapping session")
        threading.Thread(target=self._restart_mapping, daemon=True).start()

    def _restart_mapping(self) -> None:
        try:
            result = subprocess.run(
                [
                    "/usr/bin/systemctl",
                    "restart",
                    "laksa-lidar-mapping.service",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self._publish_status("ERROR")
            self.get_logger().error(f"Map reset failed: {error}")
            self._resetting = False
            return
        if result.returncode == 0:
            self._publish_status("READY")
            self.get_logger().info("Fresh SLAM mapping session started")
        else:
            detail = (result.stderr or result.stdout).strip()
            self._publish_status("ERROR")
            self.get_logger().error(f"Map reset failed: {detail}")
        self._resetting = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSessionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
