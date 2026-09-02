#!/usr/bin/env python3

"""Keep calibrated static sensor transforms available across ROS restarts."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class StaticTransformKeepalive(Node):
    def __init__(self) -> None:
        super().__init__("laksa_tf_static_keepalive")
        self.declare_parameter("publish_period_sec", 0.1)
        self.declare_parameter("lidar_x", 0.31542)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", 0.13542)
        self.declare_parameter("lidar_yaw", math.pi)
        self.declare_parameter("camera_x", 0.10807)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.140)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", math.radians(4.0))
        self.declare_parameter("camera_yaw", 0.0)

        self._fixed_children = {
            "laksa_lidar",
            "zed_camera_link",
            "zed_camera_center",
            "zed_left_camera_frame",
            "zed_left_camera_frame_optical",
        }
        self._transforms = {
            "laksa_lidar": self._make_transform(
                "laksa_base_footprint",
                "laksa_lidar",
                "lidar",
            ),
            "zed_camera_link": self._make_transform(
                "laksa_base_footprint",
                "zed_camera_link",
                "camera",
            ),
            "zed_camera_center": self._literal_transform(
                "zed_camera_link",
                "zed_camera_center",
                0.0,
                0.0,
                0.015,
                (0.0, 0.0, 0.0, 1.0),
            ),
            "zed_left_camera_frame": self._literal_transform(
                "zed_camera_center",
                "zed_left_camera_frame",
                -0.01,
                0.06,
                0.0,
                (0.0, 0.0, 0.0, 1.0),
            ),
            "zed_left_camera_frame_optical": self._literal_transform(
                "zed_left_camera_frame",
                "zed_left_camera_frame_optical",
                0.0,
                0.0,
                0.0,
                (0.5, -0.5, 0.5, -0.5),
            ),
        }
        qos = QoSProfile(depth=100)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, "/tf_static", self._tf_callback, qos)
        period = max(
            0.05,
            float(self.get_parameter("publish_period_sec").value),
        )
        self.create_timer(period, self._publish)
        self._publish()
        self.get_logger().info(
            "Static TF keepalive is protecting LiDAR and ZED mount transforms"
        )

    def _make_transform(
        self,
        parent: str,
        child: str,
        prefix: str,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(
            self.get_parameter(f"{prefix}_x").value
        )
        transform.transform.translation.y = float(
            self.get_parameter(f"{prefix}_y").value
        )
        transform.transform.translation.z = float(
            self.get_parameter(f"{prefix}_z").value
        )
        roll = float(self.get_parameter(f"{prefix}_roll").value) \
            if self.has_parameter(f"{prefix}_roll") else 0.0
        pitch = float(self.get_parameter(f"{prefix}_pitch").value) \
            if self.has_parameter(f"{prefix}_pitch") else 0.0
        yaw = float(self.get_parameter(f"{prefix}_yaw").value)
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    @staticmethod
    def _literal_transform(
        parent: str,
        child: str,
        x: float,
        y: float,
        z: float,
        quaternion,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def _tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child or child in self._fixed_children:
                continue
            self._transforms[child] = transform

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = list(self._transforms.values())
        for transform in transforms:
            transform.header.stamp = stamp
        self._broadcaster.sendTransform(transforms)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTransformKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
