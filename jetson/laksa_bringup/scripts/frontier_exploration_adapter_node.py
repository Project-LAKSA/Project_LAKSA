#!/usr/bin/env python3

"""Bridge LAKSA mode control to the community frontier explorer service."""

import rclpy
from frontier_exploration_ros2.srv import ControlExploration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class FrontierExplorationAdapter(Node):
    """Translate the supervisor's latched Boolean into idempotent start/stop calls."""

    def __init__(self) -> None:
        super().__init__("frontier_exploration_adapter")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._client = self.create_client(
            ControlExploration, "/frontier_explorer/control_exploration"
        )
        self._status = self.create_publisher(
            String, "/laksa/exploration_status", latched
        )
        self._complete = self.create_publisher(
            Bool, "/laksa/exploration_complete", latched
        )
        self._requested_enabled = False
        self._applied_enabled = None
        self._request_in_flight = False

        self.create_subscription(
            Bool,
            "/laksa/exploration_enabled",
            self._enabled_callback,
            latched,
        )
        self.create_timer(0.5, self._reconcile)
        self._publish_status("IDLE")
        self._publish_complete(False)

    def _enabled_callback(self, message: Bool) -> None:
        self._requested_enabled = message.data
        self._reconcile()

    def _reconcile(self) -> None:
        if self._request_in_flight:
            return
        if self._applied_enabled == self._requested_enabled:
            return
        if not self._client.service_is_ready():
            self._publish_status("WAITING_FOR_EXPLORER")
            return

        request = ControlExploration.Request()
        request.action = (
            ControlExploration.Request.ACTION_START
            if self._requested_enabled
            else ControlExploration.Request.ACTION_STOP
        )
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        self._request_in_flight = True
        self._publish_status(
            "STARTING" if self._requested_enabled else "STOPPING"
        )
        future = self._client.call_async(request)
        future.add_done_callback(self._control_response)

    def _control_response(self, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # ROS service transport failure
            self.get_logger().error(f"Explorer control request failed: {error}")
            self._publish_status("CONTROL_ERROR")
            return

        if not response.accepted:
            self.get_logger().warning(
                f"Explorer rejected control request: {response.message}"
            )
            self._publish_status("CONTROL_REJECTED")
            return

        self._applied_enabled = self._requested_enabled
        self._publish_complete(False)
        self._publish_status("IN_PROGRESS" if self._applied_enabled else "IDLE")

    def _publish_status(self, value: str) -> None:
        message = String()
        message.data = value
        self._status.publish(message)

    def _publish_complete(self, value: bool) -> None:
        message = Bool()
        message.data = value
        self._complete.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
