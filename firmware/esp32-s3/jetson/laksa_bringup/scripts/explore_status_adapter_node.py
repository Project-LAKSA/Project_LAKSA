#!/usr/bin/env python3

"""Expose explore_lite mission state through the LAKSA dashboard API."""

import rclpy
from explore_lite_msgs.msg import ExploreStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class ExploreStatusAdapter(Node):
    def __init__(self) -> None:
        super().__init__("explore_status_adapter")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status = self.create_publisher(
            String, "/laksa/exploration_status", latched
        )
        self._complete = self.create_publisher(
            Bool, "/laksa/exploration_complete", latched
        )
        self.create_subscription(
            ExploreStatus, "/explore/status", self._status_callback, latched
        )

    def _status_callback(self, message: ExploreStatus) -> None:
        source = message.status
        names = {
            ExploreStatus.EXPLORATION_STARTED: "STARTING",
            ExploreStatus.EXPLORATION_IN_PROGRESS: "IN_PROGRESS",
            ExploreStatus.EXPLORATION_PAUSED: "PAUSED",
            ExploreStatus.EXPLORATION_COMPLETE: "COMPLETE",
            ExploreStatus.EXPLORATION_BLOCKED: "BLOCKED",
            ExploreStatus.RETURNING_TO_ORIGIN: "RETURNING_HOME",
            ExploreStatus.RETURNED_TO_ORIGIN: "AT_HOME",
        }
        status = String()
        status.data = names.get(source, source.upper())
        self._status.publish(status)
        complete = Bool()
        complete.data = source == ExploreStatus.EXPLORATION_COMPLETE
        self._complete.publish(complete)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExploreStatusAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
