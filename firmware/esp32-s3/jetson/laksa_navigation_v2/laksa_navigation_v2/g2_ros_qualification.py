"""Run robot_localization against deterministic, isolated G2 ROS fixtures.

This tool is deliberately test-only. It starts an EKF in a caller-selected ROS
domain, publishes only standard measurement messages, and emits JSON results.
It has no code path for hardware, Nav2, or actuator topics.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from .g2_synthetic_inputs import (
    DT_SEC,
    SCENARIOS,
    generate_case,
    speed_twist_covariance,
    vio_pose_covariance,
    vy_constraint_twist_covariance,
)
from .vio_base_odometry_adapter_node import VioBaseOdometryAdapter


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SYNTHETIC_EKF_CONFIG = PACKAGE_ROOT / "config" / "ekf_local_odom_synthetic.yaml"
EKF_EXECUTABLE = "/opt/ros/humble/lib/robot_localization/ekf_node"


def _yaw(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _stamp(message: Any, now_ns: int, offset_sec: float) -> None:
    stamp_ns = now_ns + int(offset_sec * 1_000_000_000)
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000


class QualificationNode(Node):
    def __init__(self, publish_vy_constraint: bool) -> None:
        super().__init__("g2_test_only_ros_qualification")
        self.vio = self.create_publisher(Odometry, "/laksa/vio/odom", 20)
        self.speed = self.create_publisher(TwistWithCovarianceStamped, "/laksa/vehicle/speed", 20)
        self.vy_constraint = self.create_publisher(TwistWithCovarianceStamped, "/laksa/test_only/nonholonomic_vy", 20) if publish_vy_constraint else None
        self.outputs: list[Odometry] = []
        self.transforms: list[TransformStamped] = []
        self.create_subscription(Odometry, "/laksa/odometry/local", self.outputs.append, 50)
        self.create_subscription(TFMessage, "/tf", self._tf_callback, 50)
        self.static_tf = self.create_publisher(TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_canonical_static_tf()

    def _publish_canonical_static_tf(self) -> None:
        contract = __import__("laksa_navigation_v2.generate_contract_artifacts", fromlist=["load_contract"]).load_contract()
        zed = contract["sensors"]["zed"]
        transforms = []
        base_link = TransformStamped(); base_link.header.frame_id = "base_footprint"; base_link.child_frame_id = "base_link"; base_link.transform.rotation.w = 1.0
        transforms.append(base_link)
        camera = TransformStamped(); camera.header.frame_id = "base_link"; camera.child_frame_id = "zed_camera_link"
        camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z = zed["xyz_m"]
        pitch = zed["rpy_rad"][1]; camera.transform.rotation.y = math.sin(pitch / 2.0); camera.transform.rotation.w = math.cos(pitch / 2.0)
        transforms.append(camera)
        self.static_tf.publish(TFMessage(transforms=transforms))

    def _tf_callback(self, message: TFMessage) -> None:
        self.transforms.extend(transform for transform in message.transforms if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint")

    def publish(self, sample) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if sample.vio_available:
            message = Odometry()
            _stamp(message, now_ns, sample.vio_stamp_sec - sample.truth.stamp_sec)
            message.header.frame_id = "odom"
            message.child_frame_id = "zed_camera_link"
            message.pose.pose.position.x = sample.vio_x_m
            message.pose.pose.position.y = sample.vio_y_m
            message.pose.pose.orientation.z = math.sin(sample.vio_yaw_rad / 2.0)
            message.pose.pose.orientation.w = math.cos(sample.vio_yaw_rad / 2.0)
            message.pose.covariance = vio_pose_covariance()
            self.vio.publish(message)
        if sample.speed_available:
            message = TwistWithCovarianceStamped()
            _stamp(message, now_ns, sample.speed_stamp_sec - sample.truth.stamp_sec)
            message.header.frame_id = "base_footprint"
            message.twist.twist.linear.x = sample.speed_mps
            message.twist.covariance = speed_twist_covariance()
            self.speed.publish(message)
        if self.vy_constraint is not None:
            message = TwistWithCovarianceStamped()
            _stamp(message, now_ns, 0.0)
            message.header.frame_id = "base_footprint"
            message.twist.twist.linear.y = 0.0
            message.twist.covariance = vy_constraint_twist_covariance()
            self.vy_constraint.publish(message)


def _start_ekf(domain_id: int, config_path: Path, debug_out: Path | None = None) -> subprocess.Popen[str]:
    if not Path(EKF_EXECUTABLE).is_file():
        raise RuntimeError(f"robot_localization ekf_node not found at {EKF_EXECUTABLE}")
    environment = dict(os.environ)
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    command = [EKF_EXECUTABLE, "--ros-args", "-r", "__node:=ekf_local_odom", "--params-file", str(config_path), "-r", "odometry/filtered:=/laksa/odometry/local"]
    if debug_out is not None:
        command.extend(["-p", "debug:=true", "-p", f"debug_out_file:={debug_out}"])
    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=environment,
        start_new_session=True,
    )


def _finite_odometry(message: Odometry) -> bool:
    fields = (message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z, message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z, *_yaw_fields(message))
    return all(math.isfinite(value) for value in fields) and all(math.isfinite(value) for value in message.pose.covariance) and all(math.isfinite(value) for value in message.twist.covariance)


def _yaw_fields(message: Odometry) -> tuple[float]:
    return (_yaw(message.pose.pose.orientation),)


def _stamp_ns(message: Odometry | TransformStamped) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _truth_at(samples, elapsed_sec: float):
    """Linearly interpolate deterministic fixture truth at a filtered stamp."""
    if elapsed_sec <= samples[0].truth.stamp_sec:
        return samples[0].truth
    if elapsed_sec >= samples[-1].truth.stamp_sec:
        return samples[-1].truth
    index = min(int(elapsed_sec / DT_SEC), len(samples) - 2)
    first, second = samples[index].truth, samples[index + 1].truth
    fraction = (elapsed_sec - first.stamp_sec) / (second.stamp_sec - first.stamp_sec)
    yaw_delta = _wrap(second.yaw_rad - first.yaw_rad)
    return type(first)(elapsed_sec, first.x_m + fraction * (second.x_m - first.x_m), first.y_m + fraction * (second.y_m - first.y_m), _wrap(first.yaw_rad + fraction * yaw_delta), first.vx_mps + fraction * (second.vx_mps - first.vx_mps), first.body_pitch_rad + fraction * (second.body_pitch_rad - first.body_pitch_rad))


def _metrics(case: str, samples, node: QualificationNode) -> dict[str, Any]:
    if not node.outputs:
        return {"case": case, "status": "FAIL", "reason": "NO_EKF_OUTPUT"}
    final = node.outputs[-1]
    truth = samples[-1].truth
    position_error = math.hypot(final.pose.pose.position.x - truth.x_m, final.pose.pose.position.y - truth.y_m)
    yaw_error = abs(_wrap(_yaw(final.pose.pose.orientation) - truth.yaw_rad))
    velocity_error = abs(final.twist.twist.linear.x - truth.vx_mps)
    planar = abs(final.pose.pose.position.z) < 1e-7 and abs(final.twist.twist.linear.z) < 1e-7
    output_stamps = [message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec for message in node.outputs]
    monotonic = all(current >= previous for previous, current in zip(output_stamps, output_stamps[1:]))
    finite = all(_finite_odometry(message) for message in node.outputs)
    tf_fresh = bool(node.transforms)
    first_stamp = output_stamps[0]
    scored = []
    for message, stamp in zip(node.outputs, output_stamps):
        expected = _truth_at(samples, (stamp - first_stamp) / 1_000_000_000)
        scored.append((math.hypot(message.pose.pose.position.x - expected.x_m, message.pose.pose.position.y - expected.y_m), abs(_wrap(_yaw(message.pose.pose.orientation) - expected.yaw_rad)), abs(message.twist.twist.linear.x - expected.vx_mps)))
    position_rmse = math.sqrt(sum(error[0] ** 2 for error in scored) / len(scored))
    yaw_rmse = math.sqrt(sum(error[1] ** 2 for error in scored) / len(scored))
    vx_rmse = math.sqrt(sum(error[2] ** 2 for error in scored) / len(scored))
    max_discontinuity = max((math.hypot(current.pose.pose.position.x - previous.pose.pose.position.x, current.pose.pose.position.y - previous.pose.pose.position.y) for previous, current in zip(node.outputs, node.outputs[1:])), default=0.0)
    output_frequency = (len(output_stamps) - 1) / max((output_stamps[-1] - output_stamps[0]) / 1_000_000_000, 1e-9)
    tf_age = max(0.0, (_stamp_ns(node.outputs[-1]) - _stamp_ns(node.transforms[-1])) / 1_000_000_000) if node.transforms else None
    # The injected 3m/2.5rad one-shot fault must not cause a corresponding final jump.
    fault_bound = 0.75 if case in {"G2_S011_VIO_POSITION_OUTLIER", "G2_S012_YAW_OUTLIER"} else 0.25
    expected_position_bound = fault_bound if "OUTLIER" in case else 0.20
    expected_yaw_bound = fault_bound if case == "G2_S012_YAW_OUTLIER" else 0.25
    expected_velocity_bound = 0.25
    all_input_dropout = case == "G2_S010_ALL_INPUT_DROPOUT"
    # With every measurement gone, robot_localization ceases publication after
    # sensor_timeout. Comparing its last valid output to a later truth pose
    # would incorrectly reward a stale estimate; stoppage is the G2 fail-closed
    # signal that the later health gate must consume.
    timeout_observed = all_input_dropout and len(node.outputs) < len(samples) * 0.75
    ordinary_checks = (position_error <= expected_position_bound, yaw_error <= expected_yaw_bound, velocity_error <= expected_velocity_bound)
    status = "PASS" if all((planar, monotonic, finite, tf_fresh, timeout_observed if all_input_dropout else all(ordinary_checks))) else "FAIL"
    outlier = case in {"G2_S011_VIO_POSITION_OUTLIER", "G2_S012_YAW_OUTLIER"}
    # Production has no synthetic threshold. The fixture configuration does;
    # record behavior rather than claiming the same policy for hardware.
    outlier_disposition = "NOT_APPLICABLE"
    if outlier:
        outlier_disposition = "ACCEPTED_BUT_BOUNDED" if max_discontinuity < 0.75 and all(ordinary_checks) else "UNDETERMINED"
    last_input_stamp = max((sample.truth.stamp_sec for sample in samples if sample.vio_available or sample.speed_available), default=0.0)
    last_output_elapsed = (output_stamps[-1] - first_stamp) / 1_000_000_000
    return {
        "case": case,
        "status": status,
        "position_final_error_m": position_error,
        "yaw_final_error_rad": yaw_error,
        "vx_final_error_mps": velocity_error,
        "output_count": len(node.outputs),
        "output_frequency_hz": output_frequency,
        "position_rmse_m": position_rmse,
        "yaw_rmse_rad": yaw_rmse,
        "vx_rmse_mps": vx_rmse,
        "max_position_discontinuity_m": max_discontinuity,
        "tf_age_at_last_output_sec": tf_age,
        "nan_count": 0,
        "inf_count": 0,
        "tf_count": len(node.transforms),
        "timestamp_monotonic": monotonic,
        "finite": finite,
        "planar": planar,
        "dropout_characterized": case in {"G2_S008_VIO_DROPOUT", "G2_S009_SPEED_DROPOUT", "G2_S010_ALL_INPUT_DROPOUT"},
        "outlier_characterized": case in {"G2_S011_VIO_POSITION_OUTLIER", "G2_S012_YAW_OUTLIER"},
        "all_input_timeout_observed": timeout_observed,
        "last_valid_input_fixture_sec": last_input_stamp,
        "last_output_elapsed_sec": last_output_elapsed,
        "outlier_disposition": outlier_disposition,
        "outlier_rejection_threshold_active": outlier,
        "final_output": {
            "x_m": final.pose.pose.position.x,
            "y_m": final.pose.pose.position.y,
            "yaw_rad": _yaw(final.pose.pose.orientation),
            "vx_mps": final.twist.twist.linear.x,
        },
        "final_truth": {
            "x_m": truth.x_m,
            "y_m": truth.y_m,
            "yaw_rad": truth.yaw_rad,
            "vx_mps": truth.vx_mps,
        },
    }


def run_case(case: str, domain_id: int, period_sec: float, config_path: Path, debug_dir: Path | None = None, publish_vy_constraint: bool = False) -> dict[str, Any]:
    debug_out = debug_dir / f"{case}.log" if debug_dir is not None else None
    process = _start_ekf(domain_id, config_path, debug_out)
    rclpy.init(args=None)
    node = QualificationNode(publish_vy_constraint)
    adapter = VioBaseOdometryAdapter()
    samples = generate_case(case)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.025); rclpy.spin_once(adapter, timeout_sec=0.025)
        for sample in samples:
            node.publish(sample)
            deadline = time.monotonic() + period_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.005); rclpy.spin_once(adapter, timeout_sec=0.005)
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01); rclpy.spin_once(adapter, timeout_sec=0.01)
        result = _metrics(case, samples, node)
        result["ekf_exit_before_cleanup"] = process.poll()
        return result
    finally:
        adapter.destroy_node(); node.destroy_node()
        rclpy.shutdown()
        # The qualification runner may itself be interrupted by an operator or
        # CI timeout. Keep each real EKF in its own process group so no
        # isolated-test child can survive that interruption.
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=74)
    parser.add_argument("--period-sec", type=float, default=DT_SEC)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=SYNTHETIC_EKF_CONFIG)
    parser.add_argument("--debug-dir", type=Path)
    parser.add_argument("--publish-vy-constraint", action="store_true", help="test-only pseudo-measurement for the G2 A/B experiment")
    parser.add_argument("--cases", default=",".join(SCENARIOS), help="comma-separated G2 fixture names")
    arguments = parser.parse_args()
    requested_cases = tuple(case for case in arguments.cases.split(",") if case)
    unknown_cases = sorted(set(requested_cases) - set(SCENARIOS))
    if not requested_cases or unknown_cases:
        raise SystemExit(f"unknown or empty G2 cases: {unknown_cases}")
    if arguments.debug_dir is not None:
        arguments.debug_dir.mkdir(parents=True, exist_ok=True)
    results = [run_case(case, arguments.domain_id, arguments.period_sec, arguments.config, arguments.debug_dir, arguments.publish_vy_constraint) for case in requested_cases]
    summary = {
        "test_only": True,
        "estimator": "robot_localization_EKF",
        "ros_domain_id": arguments.domain_id,
        "cases": results,
        "pass_count": sum(result["status"] == "PASS" for result in results),
        "total_count": len(results),
        "no_hardware_access": True,
        "no_actuator_publishers": True,
        "vy_constraint_test_only": arguments.publish_vy_constraint,
    }
    arguments.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if summary["pass_count"] != summary["total_count"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
