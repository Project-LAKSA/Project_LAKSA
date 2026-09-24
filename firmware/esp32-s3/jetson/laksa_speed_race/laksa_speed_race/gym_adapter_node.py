"""Simulation-only ROS adapter around the pinned F1TENTH Gym core.

The stock Gym ROS bridge cannot inject LAKSA_PROXY_V0 or provide the audited
three-lap terminal semantics. This node is therefore the single stepping
authority and contains only configuration, ROS translation, safety gates, and
evidence capture. All vehicle dynamics remain in the pinned Gym core and all
tracking intelligence remains in pinned Waterloo Pure Pursuit.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import platform
from datetime import datetime, timezone
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .c1_contract import (
    BASE_FRAME,
    CONTROLLER_REQUEST_TOPIC,
    DT_S,
    MAP_FRAME,
    MAX_LAPS,
    MAX_SPEED_MPS,
    SEED,
    SIMULATOR_APPLIED_TOPIC,
    SIMULATOR_ODOM_TOPIC,
    STEERING_LIMIT_RAD,
)
from .metrics import C1Metrics
from .run_validation import persist_run
from .three_lap_gate import MissionState, ThreeLapGate


class GymEnvironment(Protocol):
    def step(self, action: Any) -> tuple[Any, float, bool, bool, dict[str, Any]]: ...


@dataclass(frozen=True)
class Command:
    steering_rad: float
    speed_mps: float


@dataclass(frozen=True)
class StepResult:
    pose: tuple[float, float, float]
    speed_mps: float
    sim_time_s: float
    lap_count: int
    terminal: bool
    fault: str | None


class CourseEnvelope:
    """Evaluate the full LAKSA rectangle against the frozen centerline corridor."""

    def __init__(self, centerline_path: Path, *, half_width_m: float, length_m: float, width_m: float, center_x_m: float):
        with centerline_path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self._samples = [(float(row["x_m"]), float(row["y_m"])) for row in rows]
        self._half_width = half_width_m
        self._half_length = length_m / 2.0
        self._half_body_width = width_m / 2.0
        self._center_x = center_x_m

    def contains_body(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        center_x = x_m + self._center_x * cosine
        center_y = y_m + self._center_x * sine
        for longitudinal in (-self._half_length, self._half_length):
            for lateral in (-self._half_body_width, self._half_body_width):
                corner_x = center_x + longitudinal * cosine - lateral * sine
                corner_y = center_y + longitudinal * sine + lateral * cosine
                nearest_sq = min((corner_x - x) ** 2 + (corner_y - y) ** 2 for x, y in self._samples)
                if nearest_sq > self._half_width * self._half_width:
                    return False
        return True


class GymStepAuthority:
    """Atomic command -> Gym step -> lap/safety/evidence transaction."""

    def __init__(self, env: GymEnvironment, envelope: CourseEnvelope, metrics: C1Metrics):
        self.env = env
        self.envelope = envelope
        self.metrics = metrics
        self.gate = ThreeLapGate(max_laps=MAX_LAPS)
        self.gate.ready()
        self.gate.start()
        self.previous_lap_count = 0
        self.last_requested = Command(0.0, 0.0)
        self.last_applied = Command(0.0, 0.0)

    @staticmethod
    def _observation(obs: dict[str, Any]) -> tuple[float, float, float, float, float, float, bool]:
        agent = obs["agent_0"]
        state = agent["std_state"]
        frenet = agent["frenet_pose"]
        return (
            float(state[0]),
            float(state[1]),
            float(state[4]),
            float(state[3]),
            float(frenet[1]),
            float(frenet[2]),
            bool(agent["collision"]),
        )

    def apply(self, command: Command) -> StepResult:
        if not math.isfinite(command.speed_mps) or not math.isfinite(command.steering_rad):
            self.metrics.invalid_command_events += 1
            self.gate.fail("non_finite_command")
            return StepResult((0.0, 0.0, 0.0), 0.0, 0.0, self.gate.lap_count, True, self.gate.fault)
        if command.speed_mps < 0.0:
            self.metrics.reverse_command_events += 1
            self.gate.fail("reverse_command")
            return StepResult((0.0, 0.0, 0.0), 0.0, 0.0, self.gate.lap_count, True, self.gate.fault)
        if command.speed_mps > MAX_SPEED_MPS + 1e-9 or abs(command.steering_rad) > STEERING_LIMIT_RAD + 1e-9:
            self.metrics.invalid_command_events += 1
            self.gate.fail("command_outside_c1_limits")
            return StepResult((0.0, 0.0, 0.0), 0.0, 0.0, self.gate.lap_count, True, self.gate.fault)

        self.gate.authorize_step()
        self.last_requested = command
        self.last_applied = command
        import numpy as np

        obs, _, done, truncated, info = self.env.step(
            np.asarray([[command.steering_rad, command.speed_mps]], dtype=np.float32)
        )
        x_m, y_m, yaw_rad, actual_speed_mps, cte_m, heading_error_rad, collision = self._observation(obs)
        track = getattr(self.env, "track", None)
        if track is not None:
            _, cte_m, heading_error_rad = track.cartesian_to_frenet(
                x_m,
                y_m,
                yaw_rad,
                use_raceline=True,
                use_s_guess=False,
            )
            cte_m = float(cte_m)
            heading_error_rad = float(heading_error_rad)
        sim_time_s = float(info["sim_time"])
        lap_count = int(info["lap_counts"][0])
        if lap_count < self.previous_lap_count or lap_count > self.previous_lap_count + 1:
            self.gate.fail("illegal_lap_sequence")
        elif lap_count == self.previous_lap_count + 1:
            self.gate.record_lap(float(info["lap_times"][0]))
            self.previous_lap_count = lap_count

        off_track = not self.envelope.contains_body(x_m, y_m, yaw_rad)
        self.metrics.record_step(
            step=self.metrics.simulator_steps + 1,
            sim_time_s=sim_time_s,
            requested_speed_mps=command.speed_mps,
            requested_steering_rad=command.steering_rad,
            applied_speed_mps=command.speed_mps,
            applied_steering_rad=command.steering_rad,
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            cte_m=cte_m,
            heading_error_rad=heading_error_rad,
            collision=collision,
            off_track=off_track,
            lap_count=lap_count,
            state=self.gate.state.value,
        )
        if collision:
            self.gate.fail("collision")
        if off_track:
            self.gate.fail("off_track")
        if truncated:
            self.gate.fail("simulator_truncated")
        if done:
            if lap_count != MAX_LAPS or self.gate.state is not MissionState.STOPPING:
                self.gate.fail("simulator_done_before_three_laps")
        elif self.gate.state is MissionState.STOPPING:
            self.gate.fail("lap_three_without_simulator_done")
        terminal = done or self.gate.state is MissionState.FAULT
        return StepResult((x_m, y_m, yaw_rad), actual_speed_mps, sim_time_s, lap_count, terminal, self.gate.fault)

    def terminal_zero(self) -> Command:
        zero = Command(0.0, 0.0)
        self.last_applied = zero
        self.gate.record_terminal_zero(zero.speed_mps, zero.steering_rad)
        return zero


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def create_gym_environment(course_dir: Path):
    import numpy as np
    from f1tenth_gym.envs import F110Env
    from f1tenth_gym.envs.action import LongitudinalActionType, SteerActionType
    from f1tenth_gym.envs.dynamic_models import DynamicModel, VehicleParameters
    from f1tenth_gym.envs.env_config import ControlConfig, EnvConfig, LoopCounterMode, ObservationConfig, SimulationConfig
    from f1tenth_gym.envs.integrators import IntegratorType
    from f1tenth_gym.envs.lidar import LiDARConfig
    from f1tenth_gym.envs.observation import ObservationType

    params = VehicleParameters(
        mu=1.0489,
        C_Sf=4.718,
        C_Sr=5.4562,
        lf=0.162,
        lr=0.162,
        h=0.074,
        m=3.75,
        I=0.04712,
        s_min=-STEERING_LIMIT_RAD,
        s_max=STEERING_LIMIT_RAD,
        sv_min=-3.2,
        sv_max=3.2,
        v_switch=7.319,
        a_max=9.51,
        v_min=-5.0,
        v_max=20.0,
        width=0.296,
        length=0.568,
        collision_body_center_x=0.135,
        collision_body_center_y=0.0,
    )
    config = EnvConfig(
        seed=SEED,
        map_name=course_dir / "speed_course.yaml",
        params=params,
        num_agents=1,
        control_config=ControlConfig(
            longitudinal_mode=LongitudinalActionType.SPEED,
            steering_mode=SteerActionType.STEERING_ANGLE,
        ),
        simulation_config=SimulationConfig(
            timestep=DT_S,
            integrator_timestep=DT_S,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.KS,
            loop_counter=LoopCounterMode.FRENET_BASED,
            compute_frenet_frame=True,
            max_laps=MAX_LAPS,
        ),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=LiDARConfig(
            enabled=True,
            num_beams=1080,
            field_of_view=math.radians(270.0),
            angle_min=math.radians(-135.0),
            angle_max=math.radians(135.0),
            range_min=0.0,
            range_max=30.0,
            noise_std=0.0,
            base_link_to_lidar_tf=(0.31542, 0.0, math.pi),
        ),
        render_enabled=False,
    )
    env = F110Env(config=config)
    start = np.asarray([[20.62863090812533, 2.2986877548722693, 0.0]], dtype=np.float32)
    observation, _ = env.reset(seed=SEED, options={"poses": start})
    return env, observation


def main(args: list[str] | None = None) -> None:
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    class AdapterNode(Node):
        def __init__(self):
            super().__init__("c1_gym_adapter")
            share = Path(get_package_share_directory("laksa_speed_race"))
            self.share = share
            course_dir = share / "course" / "canonical" / "speed_course"
            self.course_dir = course_dir
            self.declare_parameter("output_dir", "/tmp/laksa-c1-results/official")
            self.declare_parameter("max_laps", MAX_LAPS)
            if int(self.get_parameter("max_laps").value) != MAX_LAPS:
                raise ValueError("C1 max_laps is frozen at exactly 3")
            self.output_dir = Path(self.get_parameter("output_dir").value)
            self.started_at_utc = datetime.now(timezone.utc).isoformat()
            env, observation = create_gym_environment(course_dir)
            envelope = CourseEnvelope(
                course_dir / "centerline.csv",
                half_width_m=0.4572,
                length_m=0.568,
                width_m=0.296,
                center_x_m=0.135,
            )
            self.authority = GymStepAuthority(env, envelope, C1Metrics(seed=SEED))
            self.last_observation = observation
            terminal_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.applied_pub = self.create_publisher(AckermannDriveStamped, SIMULATOR_APPLIED_TOPIC, terminal_qos)
            self.odom_pub = self.create_publisher(Odometry, SIMULATOR_ODOM_TOPIC, 10)
            self.tf_pub = TransformBroadcaster(self)
            self.subscription = self.create_subscription(
                AckermannDriveStamped, CONTROLLER_REQUEST_TOPIC, self.on_command, 10
            )
            self.terminal = False
            self.initial_observation = observation
            self.readiness_ticks = 0
            self.initial_odom_sent = False
            self.readiness_timer = self.create_timer(0.05, self.publish_readiness)

        def publish_applied(self, command: Command) -> None:
            message = AckermannDriveStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = BASE_FRAME
            message.drive.steering_angle = command.steering_rad
            message.drive.speed = command.speed_mps
            self.applied_pub.publish(message)

        def publish_pose(self, observation: dict[str, Any], *, publish_odom: bool = True) -> None:
            state = observation["agent_0"]["std_state"]
            now = self.get_clock().now().to_msg()
            yaw = float(state[4])
            z = math.sin(yaw / 2.0)
            w = math.cos(yaw / 2.0)
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = MAP_FRAME
            transform.child_frame_id = BASE_FRAME
            transform.transform.translation.x = float(state[0])
            transform.transform.translation.y = float(state[1])
            transform.transform.rotation.z = z
            transform.transform.rotation.w = w
            self.tf_pub.sendTransform(transform)
            if not publish_odom:
                return
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = MAP_FRAME
            odom.child_frame_id = BASE_FRAME
            odom.pose.pose.position.x = float(state[0])
            odom.pose.pose.position.y = float(state[1])
            odom.pose.pose.orientation.z = z
            odom.pose.pose.orientation.w = w
            odom.twist.twist.linear.x = float(state[3])
            self.odom_pub.publish(odom)

        def publish_readiness(self) -> None:
            # Fill the controller's TF buffer before sending its first odometry
            # callback. This avoids changing Waterloo's controller while making
            # process startup deterministic.
            self.publish_pose(self.initial_observation, publish_odom=False)
            if self.odom_pub.get_subscription_count() == 0:
                self.readiness_ticks = 0
                return
            self.readiness_ticks += 1
            if self.readiness_ticks >= 20 and not self.initial_odom_sent:
                self.initial_odom_sent = True
                self.publish_pose(self.initial_observation, publish_odom=True)

        def on_command(self, message: AckermannDriveStamped) -> None:
            if self.terminal:
                return
            if not self.initial_odom_sent:
                return
            self.readiness_timer.cancel()
            command = Command(float(message.drive.steering_angle), float(message.drive.speed))
            result = self.authority.apply(command)
            if not result.terminal:
                self.publish_applied(command)
                self.publish_pose(self.authority.env.observation_type.observe())
                return
            self.terminal = True
            zero = self.authority.terminal_zero()
            self.publish_applied(zero)
            metadata = {
                "branch": "competition/speed-race-track",
                "commit": os.environ.get("LAKSA_GIT_SHA", "UNKNOWN"),
                "simulator_repo": "f1tenth/f1tenth_gym",
                "simulator_sha": "bdaec1420c3b0f103858d289866d0d4e2e597c30",
                "controller_repo": "CL2-UWaterloo/f1tenth_ws",
                "controller_sha": "c20cf63d04b9841ffdb6b2f963bd737d78074136",
                "raceline_repo": "CL2-UWaterloo/Raceline-Optimization",
                "raceline_sha": "9290c5d503462e46f7e3e9033002e7ddf165ba7b",
                "trajectory_helpers_sha": "fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56",
                "course_source_sha256": "222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e",
                "course_geometry_sha256": "2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858",
                "raceline_file_sha256": _sha256(self.course_dir / "speed_course_raceline.csv"),
                "controller_config_sha256": _sha256(self.share / "config" / "c1_pure_pursuit.yaml"),
                "proxy_config_sha256": _sha256(self.share / "config" / "laksa_proxy_v0.yaml"),
                "seed": SEED,
                "dt_s": DT_S,
                "dynamics_model": "KS",
                "max_laps": MAX_LAPS,
                "started_at_utc": self.started_at_utc,
                "ended_at_utc": datetime.now(timezone.utc).isoformat(),
                "runtime_environment": {
                    "execution_path": "NATIVE_HUMBLE",
                    "ros_distro": os.environ.get("ROS_DISTRO", "UNKNOWN"),
                    "python_version": platform.python_version(),
                    "machine": platform.machine(),
                },
                "shutdown_state": "REQUESTED_AFTER_EVIDENCE_FLUSH",
            }
            persist_run(
                self.output_dir,
                metrics=self.authority.metrics,
                gate=self.authority.gate,
                sim_time_s=result.sim_time_s,
                metadata=metadata,
                final_requested_command={
                    "steering_rad": self.authority.last_requested.steering_rad,
                    "speed_mps": self.authority.last_requested.speed_mps,
                },
                final_applied_command={"steering_rad": 0.0, "speed_mps": 0.0},
            )
            self.create_timer(0.2, self.shutdown_once)

        def shutdown_once(self) -> None:
            if rclpy.ok():
                rclpy.shutdown()

    rclpy.init(args=args)
    node = AdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
