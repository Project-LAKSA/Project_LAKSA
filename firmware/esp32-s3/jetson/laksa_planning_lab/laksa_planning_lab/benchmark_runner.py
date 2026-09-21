"""Headless, persistent-worker, planner-only LAKSA benchmark orchestrator."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose, SmoothPath
from nav2_msgs.srv import IsPathValid, LoadMap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .database import ResultStore
from .geometry import stable_hash
from .lattice_tools import generate_lattices
from .map_dataset import discover_maps, load_map, representative_cell_indices, write_manifest
from .metrics import evaluate_path
from .optimizer import candidate_configurations, select_survivors, stage_plan
from .runtime_config import render
from .scenario_generator import PRESETS, generate_scenarios, load_scenarios, save_scenarios, scenario_targets, validate_scenario
from .worker_state import INFRASTRUCTURE_FAILURES, bounded_process_shutdown, classify_readiness_timeout, failure_metrics, missing_readiness, validation_result

METHODS = (
    "HYBRID_PRODUCTION", "HYBRID_RAW", "HYBRID_CONSTRAINED",
    "LATTICE_CONSERVATIVE", "LATTICE_ASYMMETRIC_FORWARD",
)
class LabFailure(RuntimeError):
    def __init__(self, failure_type: str, message: str, diagnostics: dict | None = None):
        super().__init__(message)
        self.failure_type = failure_type
        self.diagnostics = diagnostics or {}


def _pose_message(values: dict, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id, pose.header.stamp = "map", stamp
    pose.pose.position.x, pose.pose.position.y = float(values["x"]), float(values["y"])
    pose.pose.orientation.z = math.sin(float(values["yaw"]) / 2.0)
    pose.pose.orientation.w = math.cos(float(values["yaw"]) / 2.0)
    return pose


def _path_tuples(path):
    result = []
    for stamped in path.poses:
        q = stamped.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        result.append((stamped.pose.position.x, stamped.pose.position.y, yaw))
    return result


def _duration_ms(duration) -> float:
    return duration.sec * 1000.0 + duration.nanosec / 1.0e6


class PlannerClient(Node):
    """One ROS client reused for all maps and scenarios in a worker."""

    def __init__(self, smoothing: bool, worker_id: str):
        super().__init__(f"benchmark_runner_{worker_id}", namespace="laksa_planning_lab")
        self.smoothing = smoothing
        self.planner = ActionClient(self, ComputePathToPose, "/laksa_planning_lab/compute_path_to_pose")
        # PlannerServer owns the official Nav2 IsPathValid service.  Keep this
        # client in the persistent worker so a returned Path can be checked by
        # Nav2 itself without regenerating or serializing it through a CLI.
        self.is_path_valid = self.create_client(IsPathValid, "/laksa_planning_lab/is_path_valid")
        self.smoother = ActionClient(self, SmoothPath, "/laksa_planning_lab/smooth_path") if smoothing else None
        self.load_map_client = self.create_client(LoadMap, "/laksa_planning_lab/map_server/load_map")
        self.lifecycle = {
            name: self.create_client(GetState, f"/laksa_planning_lab/{name}/get_state")
            for name in ("map_server", "planner_server", "smoother_server")
            if smoothing or name != "smoother_server"
        }
        self.lifecycle_change = {
            name: self.create_client(ChangeState, f"/laksa_planning_lab/{name}/change_state")
            for name in self.lifecycle
        }
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/laksa_planning_lab/map", self._map_cb, qos)
        self.create_subscription(OccupancyGrid, "/laksa_planning_lab/global_costmap/costmap", self._costmap_cb, qos)
        self.create_subscription(
            PolygonStamped, "/laksa_planning_lab/global_costmap/published_footprint",
            self._footprint_cb, 10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.expected_map = None
        self.map_received = False
        self.costmap_received = False
        self.map_semantics_match = False
        self.footprint_received = False
        self.published_footprint = []
        self.costmap_snapshot = None
        self.costmap_updates = 0
        self.last_path = None
        self.shutdown_requested = False

    def expect_map(self, map_data) -> None:
        self.expected_map = map_data
        self.map_received = False
        self.costmap_received = False
        self.map_semantics_match = False

    def _map_cb(self, msg: OccupancyGrid) -> None:
        expected = self.expected_map
        if expected is None:
            return
        dimensions_match = (
            msg.info.width == expected.width and msg.info.height == expected.height
            and math.isclose(msg.info.resolution, expected.resolution, rel_tol=0.0, abs_tol=1.0e-9)
        )
        self.map_received = dimensions_match
        if not dimensions_match or len(msg.data) != len(expected.cells):
            self.map_semantics_match = False
            return
        representatives = representative_cell_indices(expected.cells)
        self.map_semantics_match = all(int(msg.data[index]) == expected.cells[index] for index in representatives)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        expected = self.expected_map
        if expected is None:
            return
        self.costmap_received = (
            msg.info.width == expected.width and msg.info.height == expected.height
            and math.isclose(msg.info.resolution, expected.resolution, rel_tol=0.0, abs_tol=1.0e-9)
        )
        self.costmap_snapshot = {
            "frame_id": msg.header.frame_id,
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "yaw_quaternion": {
                    "x": msg.info.origin.orientation.x,
                    "y": msg.info.origin.orientation.y,
                    "z": msg.info.origin.orientation.z,
                    "w": msg.info.origin.orientation.w,
                },
            },
            "cell_count": len(msg.data),
            "data_sha256": hashlib.sha256(bytes((int(value) + 256) % 256 for value in msg.data)).hexdigest(),
        }
        self.costmap_updates += 1

    def _footprint_cb(self, msg: PolygonStamped) -> None:
        points = msg.polygon.points
        self.footprint_received = len(points) >= 4 and all(
            math.isfinite(value)
            for point in points for value in (point.x, point.y, point.z)
        )
        self.published_footprint = [[point.x, point.y, point.z] for point in points]

    def _state_is_active(self, name: str) -> bool:
        client = self.lifecycle[name]
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
        return bool(
            future.done() and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )

    def readiness_snapshot(self) -> dict:
        snapshot = {
            "map_server_active": self._state_is_active("map_server"),
            "planner_server_active": self._state_is_active("planner_server"),
            "planner_action": self.planner.server_is_ready(),
            "is_path_valid_service": self.is_path_valid.service_is_ready(),
            "map_received": self.map_received and self.map_semantics_match,
            "costmap_received": self.costmap_received,
            "tf_ready": self.tf_buffer.can_transform(
                "map", "base_footprint", Time(), timeout=Duration(seconds=0.0)
            ),
        }
        if self.smoothing:
            snapshot.update({
                "smoother_server_active": self._state_is_active("smoother_server"),
                "smoother_action": bool(self.smoother and self.smoother.server_is_ready()),
                "footprint_received": self.footprint_received,
                "base_link_tf_ready": self.tf_buffer.can_transform(
                    "base_footprint", "base_link", Time(), timeout=Duration(seconds=0.0)
                ),
            })
        return snapshot

    def wait_ready(self, process, timeout: float = 90.0) -> tuple[float, dict]:
        started = time.monotonic()
        last_snapshot = {}
        while time.monotonic() - started < timeout:
            if process.poll() is not None:
                raise LabFailure("STACK_START_FAILURE", f"planner launch exited with code {process.returncode}")
            rclpy.spin_once(self, timeout_sec=0.1)
            last_snapshot = self.readiness_snapshot()
            if not missing_readiness(last_snapshot, self.smoothing) and last_snapshot["is_path_valid_service"]:
                return (time.monotonic() - started) * 1000.0, last_snapshot
        if self.map_received and not self.map_semantics_match:
            raise LabFailure("MAP_LOAD_FAILURE", "published map disagrees with offline PGM/YAML semantics")
        failure = classify_readiness_timeout(last_snapshot, self.smoothing)
        raise LabFailure(failure, "readiness timeout; missing " + ", ".join(missing_readiness(last_snapshot, self.smoothing)))

    def load_map(self, process, yaml_path: str, map_data, timeout: float = 90.0) -> float:
        self.expect_map(map_data)
        deadline = time.monotonic() + timeout
        while not self.load_map_client.service_is_ready() and time.monotonic() < deadline:
            if process.poll() is not None:
                raise LabFailure("STACK_START_FAILURE", "planner worker exited before map switch")
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.load_map_client.service_is_ready():
            raise LabFailure("MAP_LOAD_FAILURE", "map_server load_map service unavailable")
        request = LoadMap.Request()
        request.map_url = yaml_path
        future = self.load_map_client.call_async(request)
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done() or future.result() is None or int(future.result().result) != 0:
            code = "timeout" if not future.done() else str(getattr(future.result(), "result", "unknown"))
            raise LabFailure("MAP_LOAD_FAILURE", f"load_map failed with result {code}")
        ready_ms, _ = self.wait_ready(process, timeout=max(0.1, deadline - time.monotonic()))
        return ready_ms

    def _execute(self, client, goal, timeout: float, smoother: bool = False):
        failure = "SMOOTHER_FAILURE" if smoother else "PLANNER_TIMEOUT"
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=timeout)
        if not sent.done() or sent.result() is None:
            raise LabFailure(failure, "action goal request timed out")
        if not sent.result().accepted:
            raise LabFailure("SMOOTHER_FAILURE" if smoother else "PLANNER_NO_PATH", "action goal rejected")
        result_future = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done() or result_future.result() is None:
            sent.result().cancel_goal_async()
            raise LabFailure(failure, "action result timed out")
        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise LabFailure(
                "SMOOTHER_FAILURE" if smoother else "PLANNER_NO_PATH",
                f"action failed with status {result_future.result().status}",
            )
        return result_future.result().result

    def plan(self, scenario: dict, timeout: float = 15.0):
        stamp = self.get_clock().now().to_msg()
        goal = ComputePathToPose.Goal()
        goal.start = _pose_message(scenario["start"], stamp)
        goal.goal = _pose_message(scenario["goal"], stamp)
        goal.planner_id, goal.use_start = "GridBased", True
        wall_start = time.perf_counter()
        planned = self._execute(self.planner, goal, timeout)
        if not planned.path.poses:
            raise LabFailure("PLANNER_NO_PATH", "planner returned an empty path")
        planning_ms = _duration_ms(planned.planning_time)
        path, smoothing_ms = planned.path, 0.0
        if self.smoothing:
            smooth_goal = SmoothPath.Goal()
            smooth_goal.path, smooth_goal.smoother_id = path, "SmoothPath"
            smooth_goal.max_smoothing_duration.sec = 5
            smooth_goal.check_for_collisions = True
            try:
                smoothed = self._execute(self.smoother, smooth_goal, timeout, smoother=True)
            except LabFailure as error:
                error.diagnostics.update({"planner_success": True, "smoother_success": False})
                raise
            path, smoothing_ms = smoothed.path, _duration_ms(smoothed.smoothing_duration)
        self.last_path = path
        return _path_tuples(path), planning_ms, smoothing_ms, (time.perf_counter() - wall_start) * 1000.0

    def validate_path(self, path, timeout: float = 15.0) -> dict:
        """Call PlannerServer's official Nav2 IsPathValid on this exact Path."""
        deadline = time.monotonic() + timeout
        while not self.is_path_valid.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.is_path_valid.service_is_ready():
            raise LabFailure("OFFICIAL_VALIDATOR_UNAVAILABLE", "planner_server is_path_valid service unavailable")
        request = IsPathValid.Request()
        request.path = path
        future = self.is_path_valid.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done() or future.result() is None:
            raise LabFailure("OFFICIAL_VALIDATOR_TIMEOUT", "planner_server is_path_valid service timed out")
        response = future.result()
        return {
            "service": "/laksa_planning_lab/is_path_valid",
            "interface": "nav2_msgs/srv/IsPathValid",
            "is_valid": bool(response.is_valid),
            "invalid_pose_indices": [int(index) for index in response.invalid_pose_indices],
        }

    def settle_costmap(self, duration: float = 1.2) -> None:
        """Let the active static layer publish after a LoadMap transition.

        PlannerServer returns ``is_valid=false`` without invalid indices when
        its costmap is not current.  This is an observation/test-harness guard,
        not a planner or costmap configuration change.
        """
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))

    def lifecycle_shutdown(self) -> None:
        for transition_id in (Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP):
            for name in reversed(tuple(self.lifecycle_change)):
                client = self.lifecycle_change[name]
                if not client.service_is_ready():
                    continue
                request = ChangeState.Request()
                request.transition.id = transition_id
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

    def begin_shutdown(self) -> None:
        """Stop readiness/failure interpretation before intentional launch teardown."""
        self.shutdown_requested = True


class PlannerWorker:
    """Own exactly one Nav2 stack for one method/configuration."""

    def __init__(self, runner, method, configuration, digest, first_entry, first_map):
        self.runner, self.method, self.configuration, self.digest = runner, method, configuration, digest
        self.process = self.log = self.client = None
        self.closed = False
        self.cold_start_ms = None
        self.map_switch_ms = []
        try:
            self._start(first_entry, first_map)
        except Exception:
            self.close()
            raise

    def _start(self, entry, map_data) -> None:
        try:
            lattice = ""
            if self.method.startswith("LATTICE_"):
                generated = self.runner._ensure_lattices(float(entry["resolution"]))
                lattice = generated["conservative" if self.method == "LATTICE_CONSERVATIVE" else "asymmetric_forward"]
            runtime_path = self.runner.results / "runtime" / f"{self.method.lower()}_{self.digest[:12]}.yaml"
            render(self.runner.share, self.method, float(entry["resolution"]), runtime_path, self.configuration, lattice)
            log_path = self.runner.results / "logs" / f"{self.method.lower()}_{self.digest[:12]}.log"
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log = log_path.open("a", encoding="utf-8", buffering=1)
            command = [
                "ros2", "launch", "laksa_planning_lab", "planning_lab.launch.py",
                f"map:={entry['yaml']}", f"params_file:={runtime_path}",
                f"use_smoother:={'true' if self.method == 'HYBRID_CONSTRAINED' else 'false'}",
            ]
            environment = os.environ.copy()
            environment["ROS_DOMAIN_ID"], environment["ROS_LOCALHOST_ONLY"] = "71", "1"
            self.process = subprocess.Popen(
                command, stdout=self.log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True,
            )
            self.client = PlannerClient(self.method == "HYBRID_CONSTRAINED", self.digest[:10])
            self.client.expect_map(map_data)
            self.cold_start_ms, _ = self.client.wait_ready(self.process)
        except LabFailure:
            raise
        except (FileNotFoundError, NotImplementedError) as error:
            raise LabFailure("UNSUPPORTED", str(error)) from error
        except Exception as error:
            raise LabFailure("STACK_START_FAILURE", f"{type(error).__name__}: {error}") from error

    def switch_map(self, entry, map_data) -> None:
        elapsed = self.client.load_map(self.process, entry["yaml"], map_data)
        self.map_switch_ms.append(elapsed)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            if self.client is not None:
                self.client.begin_shutdown()
                if self.log is not None:
                    self.log.write("TEARDOWN_BEGIN shutdown_requested=true\n")
            bounded_process_shutdown(self.process)
        finally:
            if self.client is not None:
                self.client.destroy_node()
            if self.log is not None:
                self.log.close()

    @property
    def process_stopped(self) -> bool:
        return self.process is None or self.process.poll() is not None


class LabRunner:
    def __init__(self, args):
        self.args = args
        self.results = args.results_dir.resolve()
        self.results.mkdir(parents=True, exist_ok=True)
        self.share = Path(get_package_share_directory("laksa_planning_lab"))
        self.store = ResultStore(self.results / "planning_lab.sqlite3")
        self.last_progress = 0.0
        self.completed_count = self.total_count = 0
        self.started = time.monotonic()
        self.entries, self.maps, self.scenarios = self._prepare_data()
        self.lattices = {}
        self.performance = {"workers": [], "warm_planning_ms": []}

    def _prepare_data(self):
        manifest = self.results / "map_manifest.yaml"
        entries = discover_maps(self.args.sessions_root)
        if not entries:
            raise RuntimeError("No real map_server-compatible PGM map exists under the LAKSA mapping sessions")
        write_manifest(entries, manifest)
        for entry in entries:
            self.store.register_map(entry)
        maps = {entry["map_id"]: load_map(Path(entry["yaml"]), entry["map_id"]) for entry in entries}
        scenario_path = self.results / f"scenarios_{self.args.preset}_{self.args.seed}.json"
        expected = self.args.count or PRESETS[self.args.preset]
        regenerate = True
        if scenario_path.is_file():
            dataset = load_scenarios(scenario_path)
            known = {entry["map_id"]: entry["sha256"] for entry in entries}
            scenarios = dataset.get("scenarios", [])
            valid = all(
                item.get("map_id") in maps and validate_scenario(item, maps[item["map_id"]])[0]
                for item in scenarios
            )
            balanced = self.args.preset != "smoke" or [
                (item.get("bearing_class"), item.get("distance_class"), item.get("scenario_kind"))
                for item in scenarios
            ] == scenario_targets(expected)
            regenerate = (
                dataset.get("schema_version") != 2 or len(scenarios) != expected or not valid or not balanced
                or any(known.get(item.get("map_id")) != item.get("map_sha256") for item in scenarios)
            )
        if regenerate:
            dataset = generate_scenarios(manifest, expected, self.args.seed)
            save_scenarios(dataset, scenario_path)
        scenarios = dataset["scenarios"]
        for scenario in scenarios:
            self.store.register_scenario(scenario)
        return entries, maps, scenarios

    def _ensure_lattices(self, resolution: float):
        key = round(resolution, 9)
        if key not in self.lattices:
            self.lattices[key] = generate_lattices(self.results / "lattices", resolution)
        return self.lattices[key]

    def _record_failure(self, scenario, method, digest, failure_type, message, diagnostics=None) -> None:
        metrics = failure_metrics(failure_type)
        metrics.update(diagnostics or {})
        self.store.record(
            scenario, method, digest, metrics,
            f"{failure_type}: {message}",
        )

    def _progress(self, method, digest):
        now = time.monotonic()
        if now - self.last_progress < 30 and self.completed_count < self.total_count:
            return
        elapsed = max(now - self.started, 1.0e-6)
        rate = self.completed_count / elapsed
        eta = (self.total_count - self.completed_count) / rate if rate else float("inf")
        successes, count = self.store.connection.execute(
            "SELECT COALESCE(SUM(success),0),COUNT(*) FROM results"
        ).fetchone()
        print(
            f"completed={self.completed_count}/{self.total_count} method={method} "
            f"config={digest[:12]} success_rate={successes/max(count,1):.3f} "
            f"eta_sec={eta:.0f} db={self.store.path}", flush=True,
        )
        self.last_progress = now

    @staticmethod
    def _path_failure(metrics: dict, map_data) -> str:
        metrics["planner_success"] = bool(metrics.get("planning_success"))
        metrics.setdefault("smoother_success", True)
        metrics["collision_valid"] = bool(metrics.get("collision_free"))
        metrics["kinematic_valid"] = bool(metrics.get("kinematically_feasible"))
        metrics["endpoint_valid"] = metrics.get("endpoint_position_error", math.inf) <= max(0.10, map_data.resolution * 1.5)
        primary, secondary = validation_result(metrics)
        metrics["primary_result"] = primary
        metrics["secondary_diagnostic_flags"] = secondary
        return primary

    def run_configuration(self, configuration: dict, scenarios: list[dict]):
        method = configuration["method"]
        recorded_configuration = dict(configuration)
        recorded_configuration["planning_lab_schema"] = "1.2"
        recorded_configuration["scenario_corpus_hash"] = stable_hash(scenarios)
        digest = self.store.register_configuration(method, recorded_configuration)
        self.total_count += len(scenarios)
        by_map = {}
        for scenario in scenarios:
            by_map.setdefault(scenario["map_id"], []).append(scenario)
        map_batches = []
        for map_id, items in by_map.items():
            pending = [item for item in items if not self.store.completed(item["map_sha256"], item["scenario_id"], digest)]
            self.completed_count += len(items) - len(pending)
            if pending:
                entry = next(value for value in self.entries if value["map_id"] == map_id)
                map_batches.append((map_id, entry, pending))
        if not map_batches:
            return digest

        worker = None
        try:
            first_map_id, first_entry, _ = map_batches[0]
            worker = PlannerWorker(self, method, configuration, digest, first_entry, self.maps[first_map_id])
            for batch_index, (map_id, entry, pending) in enumerate(map_batches):
                if batch_index:
                    try:
                        worker.switch_map(entry, self.maps[map_id])
                    except LabFailure as error:
                        for scenario in pending:
                            self._record_failure(scenario, method, digest, error.failure_type, str(error), error.diagnostics)
                            self.completed_count += 1
                        continue
                for scenario in pending:
                    valid, reason = validate_scenario(scenario, self.maps[map_id])
                    if not valid:
                        self._record_failure(scenario, method, digest, "INVALID_SCENARIO", reason)
                    else:
                        try:
                            poses, planning_ms, smoothing_ms, total_ms = worker.client.plan(scenario)
                            metrics = evaluate_path(poses, scenario, self.maps[map_id])
                            metrics["smoother_success"] = True if worker.client.smoothing else None
                            failure_type = self._path_failure(metrics, self.maps[map_id])
                            metrics.update(
                                failure_type=failure_type, planning_time_ms=planning_ms,
                                smoothing_time_ms=smoothing_ms, total_pipeline_time_ms=total_ms,
                            )
                            self.store.record(
                                scenario, method, digest, metrics,
                                "" if failure_type == "SUCCESS" else failure_type,
                            )
                            self.performance["warm_planning_ms"].append(total_ms)
                        except LabFailure as error:
                            self._record_failure(scenario, method, digest, error.failure_type, str(error), error.diagnostics)
                        except Exception as error:
                            self._record_failure(scenario, method, digest, "PLANNER_EXCEPTION", f"{type(error).__name__}: {error}")
                    self.completed_count += 1
                    self._progress(method, digest)
        except LabFailure as error:
            for _, _, pending in map_batches:
                for scenario in pending:
                    if not self.store.completed(scenario["map_sha256"], scenario["scenario_id"], digest):
                        self._record_failure(scenario, method, digest, error.failure_type, str(error), error.diagnostics)
                        self.completed_count += 1
        finally:
            if worker is not None:
                worker.close()
                self.performance["workers"].append({
                    "method": method, "configuration_hash": digest,
                    "cold_startup_ms": worker.cold_start_ms,
                    "map_switch_ms": worker.map_switch_ms,
                    "clean_shutdown": worker.process_stopped,
                })
        return digest

    def _write_performance(self) -> None:
        samples = sorted(self.performance["warm_planning_ms"])
        percentile = lambda q: samples[min(len(samples) - 1, int(math.ceil(q * len(samples))) - 1)] if samples else None
        payload = dict(self.performance)
        payload.update({
            "warm_planning_median_ms": percentile(0.50),
            "warm_planning_p95_ms": percentile(0.95),
            "scenario_throughput_hz": self.completed_count / max(1.0e-6, time.monotonic() - self.started),
        })
        (self.results / "performance.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def _smoke_passes(self, hashes: list[str]) -> bool:
        for digest in hashes:
            method = self.store.connection.execute(
                "SELECT method FROM configurations WHERE config_hash=?", (digest,)
            ).fetchone()[0]
            rows = self.store.connection.execute(
                "SELECT success,metrics_json FROM results WHERE config_hash=?", (digest,)
            ).fetchall()
            if not rows:
                return False
            failure_types = {json.loads(payload).get("failure_type") for _, payload in rows}
            if failure_types & INFRASTRUCTURE_FAILURES:
                return False
            if method in ("HYBRID_PRODUCTION", "HYBRID_RAW"):
                if sum(success for success, _ in rows) / len(rows) < 0.75:
                    return False
            elif failure_types == {"UNSUPPORTED"}:
                continue
        return all(worker["clean_shutdown"] for worker in self.performance["workers"])

    def run(self):
        fixed = [{"method": method} for method in (self.args.methods or METHODS)]
        fixed_hashes = []
        if self.args.preset != "nightly":
            for configuration in fixed:
                fixed_hashes.append(self.run_configuration(configuration, self.scenarios))
        else:
            for configuration in fixed:
                self.run_configuration(configuration, self.scenarios)
            search_path = self.share / "config" / "search_space.yaml"
            stages = {stage["name"]: stage for stage in stage_plan(search_path)}
            candidates = candidate_configurations(search_path, int(stages["coarse"]["configurations"]))
            smoke = stages["smoke"]
            for configuration in candidates[:int(smoke["configurations"])]:
                self.run_configuration(configuration, self.scenarios[:int(smoke["scenarios"])])
            coarse = stages["coarse"]
            hashes = [self.run_configuration(configuration, self.scenarios[:int(coarse["scenarios"])]) for configuration in candidates]
            survivors = set(select_survivors(self.store, hashes, int(coarse["survivors"])))
            semifinal = [configuration for configuration, digest in zip(candidates, hashes) if digest in survivors]
            semifinal_stage = stages["semifinal"]
            semifinal_hashes = [self.run_configuration(configuration, self.scenarios[:int(semifinal_stage["scenarios"])]) for configuration in semifinal]
            finalists = set(select_survivors(self.store, semifinal_hashes, int(semifinal_stage["survivors"])))
            final_scenarios = self.scenarios[:int(stages["final"]["scenarios"])]
            for configuration, digest in zip(semifinal, semifinal_hashes):
                if digest in finalists:
                    self.run_configuration(configuration, final_scenarios)
        self.store.export_summaries(self.results)
        self._write_performance()
        self._progress("COMPLETE", "-")
        if self.args.preset == "smoke" and not self._smoke_passes(fixed_hashes):
            raise RuntimeError("Smoke failed its infrastructure/baseline acceptance criteria")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="LAKSA isolated global-planning benchmark")
    parser.add_argument("--preset", choices=PRESETS, default="smoke")
    parser.add_argument("--resume", action="store_true", help="Resume is always safe; retained for explicit operator intent")
    parser.add_argument("--count", type=int)
    parser.add_argument("--seed", type=int, default=2906)
    parser.add_argument("--sessions-root", type=Path, default=Path("/home/ubuntu/laksa_mapping_sessions"))
    parser.add_argument("--results-dir", type=Path, default=Path("/home/ubuntu/laksa_planning_lab_results"))
    parser.add_argument("--methods", nargs="+", choices=METHODS)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if os.environ.get("ROS_DOMAIN_ID") != "71" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Safety gate: set ROS_DOMAIN_ID=71 and ROS_LOCALHOST_ONLY=1")
    rclpy.init(args=[])
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        LabRunner(args).run()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
