"""Isolated historical-state replay for the C1.2d MPPI lockstep host."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from .owned_process_session import OwnedProcessSession


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--w", type=float, required=True)
    parser.add_argument("--velocity", type=float, required=True)
    parser.add_argument("--stamp-sec", type=int, required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument("--workspace", default="/tmp/laksa-c1.2-runtime")
    parser.add_argument(
        "--result-root", default="/tmp/laksa-c1.2d-results/historical_replay"
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    return parser


def _spawn(command: list[str], output: Path) -> subprocess.Popen[bytes]:
    stream = output.open("wb")
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
    process._laksa_stream = stream  # type: ignore[attr-defined]
    return process


def _close_stream(process: subprocess.Popen[bytes]) -> None:
    stream = getattr(process, "_laksa_stream", None)
    if stream is not None:
        stream.close()


def _wait_required(process: subprocess.Popen[bytes], name: str, timeout: float) -> None:
    try:
        returncode = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"{name} did not finish within {timeout:.1f}s") from exc
    if returncode != 0:
        raise RuntimeError(f"{name} exited with status {returncode}")


def _worker(args: argparse.Namespace) -> int:
    workspace = Path(args.workspace)
    output = Path(args.result_root) / args.name
    output.mkdir(parents=True, exist_ok=True)
    config = workspace / "install/laksa_speed_race/share/laksa_speed_race/config/c1_nav2_mppi.yaml"
    map_yaml = workspace / "install/laksa_speed_race/share/laksa_speed_race/course/canonical/speed_course/speed_course_nav2.yaml"

    children: list[subprocess.Popen[bytes]] = []
    try:
        children.append(
            _spawn(["ros2", "run", "laksa_speed_race", "c1_nav2_raceline"], output / "path.log")
        )
        children.append(
            _spawn(
                [
                    "ros2", "run", "laksa_speed_race_nav2", "rpp_lockstep_host",
                    "--ros-args", "-r", "__node:=mppi_lockstep_host", "-r", "__ns:=/c1",
                    "--params-file", str(config), "-p", f"map_yaml_path:={map_yaml}",
                ],
                output / "host.log",
            )
        )
        time.sleep(4.0)
        if children[0].poll() is not None:
            raise RuntimeError(f"raceline node exited early with status {children[0].returncode}")
        if children[1].poll() is not None:
            raise RuntimeError(f"lockstep host exited early with status {children[1].returncode}")
        children.append(
            _spawn(
                ["timeout", "8", "ros2", "topic", "echo", "--no-daemon", "--once", "--full-length",
                 "/c1/controller_feasibility", "std_msgs/msg/String"],
                output / "feasibility.yaml",
            )
        )
        children.append(
            _spawn(
                ["timeout", "8", "ros2", "topic", "echo", "--no-daemon", "--once", "--full-length",
                 "/c1/nav2_cmd_vel", "geometry_msgs/msg/TwistStamped"],
                output / "command.yaml",
            )
        )
        children.append(
            _spawn(
                ["timeout", "8", "ros2", "topic", "echo", "--no-daemon", "--once", "--full-length",
                 "/c1/controller_fault", "std_msgs/msg/String"],
                output / "fault.yaml",
            )
        )
        time.sleep(1.0)
        message = (
            "{header: {stamp: {sec: %d, nanosec: 0}, frame_id: map}, "
            "child_frame_id: c1/base_link, pose: {pose: {position: {x: %.17g, y: %.17g, z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: %.17g, w: %.17g}}}, "
            "twist: {twist: {linear: {x: %.17g, y: 0.0, z: 0.0}, "
            "angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
            % (args.stamp_sec, args.x, args.y, args.z, args.w, args.velocity)
        )
        publisher = _spawn(
            ["ros2", "topic", "pub", "--once", "/c1/odom", "nav_msgs/msg/Odometry", message],
            output / "publish.log",
        )
        children.append(publisher)
        _wait_required(publisher, "odometry publisher", 8.0)
        _wait_required(children[2], "feasibility observer", 10.0)
        _wait_required(children[3], "command observer", 10.0)
        try:
            fault_returncode = children[4].wait(timeout=10.0)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("fault observer did not finish") from exc
        if fault_returncode == 0:
            raise RuntimeError("controller fault was published")
        if fault_returncode != 124:
            raise RuntimeError(f"fault observer exited with status {fault_returncode}")
        (output / "worker_complete.json").write_text(
            json.dumps({"name": args.name, "worker_pid": os.getpid()}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 0
    finally:
        for child in children:
            _close_stream(child)


def _controller(args: argparse.Namespace) -> int:
    output = Path(args.result_root) / args.name
    output.mkdir(parents=True, exist_ok=True)
    lifecycle_path = output / "lifecycle.json"
    driver_path = output / "driver.log"
    command = [sys.executable, "-m", "laksa_speed_race.historical_replay", *sys.argv[1:], "--worker"]
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"

    with driver_path.open("wb") as driver:
        session = OwnedProcessSession(command, env=env, stdout=driver, stderr=subprocess.STDOUT)
        worker_returncode = session.wait_for_leader(timeout=30.0)
        result = session.shutdown(graceful_timeout=3.0, term_timeout=2.0, kill_timeout=2.0)

    evidence = {
        "name": args.name,
        "worker_returncode": worker_returncode,
        "process_group": result.pgid,
        "observed_pids": result.observed_pids,
        "shutdown_phase": result.phase,
        "shutdown_success": result.success,
        "remaining_pids": result.remaining_pids,
        "leader_returncode": result.leader_returncode,
        "terminal_command": {"steering": 0.0, "speed": 0.0},
        "physical_authority": False,
    }
    lifecycle_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(evidence, sort_keys=True))
    return 0 if worker_returncode == 0 and result.success else 1


def main() -> int:
    args = _parser().parse_args()
    if args.worker:
        return _worker(args)
    return _controller(args)


if __name__ == "__main__":
    raise SystemExit(main())
