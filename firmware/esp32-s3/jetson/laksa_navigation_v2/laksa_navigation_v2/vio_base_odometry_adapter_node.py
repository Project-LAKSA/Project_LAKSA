"""Transform raw ZED camera odometry into the canonical robot-base measurement.

ZED Wrapper 5.4.1 publishes pose as `odom -> zed_camera_link`. Humble
robot_localization transforms measurement axes but does not reinterpret the
Odometry child origin as `base_footprint`; a non-zero camera offset otherwise
appears directly in the fused robot pose. This narrow, observational adapter
uses the canonical TF chain to compose the actual rigid transforms before the
official EKF consumes the standard Odometry message. It publishes no TF and no
motion command.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from .vio_base_transform_contract import compose, quaternion_multiply as _mul, rotate as _rotate


def compose_odom_camera_to_base(raw: Odometry, camera_to_base: TransformStamped) -> Odometry:
    """Compose `odom->camera` and `camera->base`; reject malformed frames."""
    if raw.header.frame_id != "odom" or raw.child_frame_id != "zed_camera_link":
        raise ValueError("LOCAL_ODOMETRY_INPUT_INVALID: raw ZED frame contract")
    t = camera_to_base.transform.translation
    r = camera_to_base.transform.rotation
    q_oc = (raw.pose.pose.orientation.x, raw.pose.pose.orientation.y, raw.pose.pose.orientation.z, raw.pose.pose.orientation.w)
    q_cb = (r.x, r.y, r.z, r.w)
    if not all(math.isfinite(value) for value in (*q_oc, *q_cb, raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z, t.x, t.y, t.z)):
        raise ValueError("LOCAL_ODOMETRY_INPUT_INVALID: non-finite pose")
    (position, q_ob) = compose((raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z), q_oc, (t.x, t.y, t.z), q_cb)
    message = Odometry()
    message.header = raw.header
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w = q_ob
    # G2 fuses only X/Y/yaw. The mounted ZED has zero static yaw, so its
    # selected planar covariance components preserve meaning; full covariance
    # rotation is intentionally deferred until a 6-DoF body-attitude contract.
    message.pose.covariance = raw.pose.covariance
    message.twist = raw.twist
    return message


class VioBaseOdometryAdapter(Node):
    def __init__(self) -> None:
        super().__init__("vio_base_odometry_adapter")
        self._buffer = Buffer(); self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(Odometry, "/laksa/vio/base_odom", 20)
        self.create_subscription(Odometry, "/laksa/vio/odom", self._callback, 20)

    def _callback(self, raw: Odometry) -> None:
        try:
            transform = self._buffer.lookup_transform("zed_camera_link", "base_footprint", raw.header.stamp, timeout=Duration(seconds=0.1))
            self._publisher.publish(compose_odom_camera_to_base(raw, transform))
        except (TransformException, ValueError) as exc:
            self.get_logger().error(str(exc))


def main() -> None:
    rclpy.init(); node = VioBaseOdometryAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
