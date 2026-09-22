#!/usr/bin/env python3

"""Bridge LAKSA mode control to the community frontier explorer service."""

import rclpy
from frontier_exploration_ros2.srv import ControlExploration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


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
        self._request_in_flight = None
        self._request_generation = 0
        self._request_started_ns = 0
        self._request_timeout_ns = 3_000_000_000
        self._service_was_ready = False
        self._last_verified_ns = 0
        self._verification_period_ns = 2_000_000_000
        self._completion_latched = False

        self.create_subscription(
            Bool,
            "/laksa/exploration_enabled",
            self._enabled_callback,
            latched,
        )
        self.create_subscription(
            Empty,
            "/laksa/exploration_complete_event",
            self._completion_callback,
            latched,
        )
        self.create_timer(0.5, self._reconcile)
        self._publish_status("IDLE")
        self._publish_complete(False)

    def _enabled_callback(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self._requested_enabled:
            self._completion_latched = False
            self._publish_complete(False)
        elif (
            not requested
            and self._requested_enabled
            and not self._completion_latched
        ):
            self._publish_complete(False)
        self._requested_enabled = requested
        self._reconcile()

    def _completion_callback(self, _message: Empty) -> None:
        # The upstream event is transient-local. Ignore an event left over from
        # an earlier mission unless this adapter currently owns a running one.
        if not self._requested_enabled or self._applied_enabled is not True:
            self.get_logger().warning(
                "Ignoring exploration completion event without an active mission"
            )
            return
        self._completion_latched = True
        self._requested_enabled = False
        self._publish_complete(True)
        self._publish_status("COMPLETE")
        self._reconcile()

    def _reconcile(self) -> None:
        service_ready = self._client.service_is_ready()
        if not service_ready:
            if self._service_was_ready:
                # Invalidate an unresolved callback and forget the runtime state.
                # A respawned explorer always starts in cold idle.
                self._request_generation += 1
                self._request_in_flight = None
                self._request_started_ns = 0
                self._applied_enabled = None
            self._service_was_ready = False
            self._publish_status("WAITING_FOR_EXPLORER")
            return

        if not self._service_was_ready:
            self._service_was_ready = True
            self._applied_enabled = None
            self._last_verified_ns = 0

        now_ns = self.get_clock().now().nanoseconds
        if self._request_in_flight is not None:
            if now_ns - self._request_started_ns < self._request_timeout_ns:
                return
            # A ready service can still strand a future when its server dies
            # during the RPC. Invalidate that callback and retry on the next
            # reconciliation tick.
            self._request_generation += 1
            self._request_in_flight = None
            self._request_started_ns = 0
            self._applied_enabled = None
            self.get_logger().error("Explorer control request timed out")
            self._publish_status("CONTROL_ERROR")
            return

        needs_reconcile = self._applied_enabled != self._requested_enabled
        needs_running_verification = (
            self._requested_enabled
            and self._applied_enabled is True
            and now_ns - self._last_verified_ns >= self._verification_period_ns
        )
        if not needs_reconcile and not needs_running_verification:
            return

        target_enabled = self._requested_enabled
        request = ControlExploration.Request()
        request.action = (
            ControlExploration.Request.ACTION_START
            if target_enabled
            else ControlExploration.Request.ACTION_STOP
        )
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        self._request_generation += 1
        generation = self._request_generation
        self._request_in_flight = target_enabled
        self._request_started_ns = now_ns
        if needs_reconcile and not self._completion_latched:
            self._publish_status("STARTING" if target_enabled else "STOPPING")
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda completed, requested=target_enabled, token=generation: (
                self._control_response(completed, requested, token)
            )
        )

    def _control_response(self, future, requested: bool, generation: int) -> None:
        if generation != self._request_generation:
            return
        self._request_in_flight = None
        self._request_started_ns = 0
        try:
            response = future.result()
        except Exception as error:  # ROS service transport failure
            self._applied_enabled = None
            self.get_logger().error(f"Explorer control request failed: {error}")
            self._publish_status("CONTROL_ERROR")
            return

        if not response.accepted:
            self._applied_enabled = None
            self.get_logger().warning(
                f"Explorer rejected control request: {response.message}"
            )
            self._publish_status("CONTROL_REJECTED")
            return

        # Record what this specific RPC applied, not the mutable desired state.
        # A stop may arrive while a start call is in flight.
        self._applied_enabled = requested
        self._last_verified_ns = self.get_clock().now().nanoseconds
        if self._completion_latched:
            self._publish_status("COMPLETE")
        elif requested:
            self._publish_status("IN_PROGRESS")
        else:
            self._publish_status("IDLE")
        # Immediately converge if operator intent changed during the RPC.
        self._reconcile()

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
