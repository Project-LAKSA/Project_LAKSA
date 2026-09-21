"""Pure readiness and failure policy for persistent Planning Lab workers."""

import os
import signal
import subprocess

FAILURE_TYPES = (
    "SUCCESS", "INVALID_SCENARIO", "STACK_START_FAILURE", "STACK_READY_TIMEOUT",
    "MAP_LOAD_FAILURE", "TF_FAILURE", "PLANNER_NO_PATH", "PLANNER_TIMEOUT",
    "PLANNER_EXCEPTION", "SMOOTHER_FAILURE", "COLLISION_FAILURE",
    "KINEMATIC_VIOLATION", "ENDPOINT_ERROR", "UNSUPPORTED",
)

INFRASTRUCTURE_FAILURES = {
    "INVALID_SCENARIO", "STACK_START_FAILURE", "STACK_READY_TIMEOUT",
    "MAP_LOAD_FAILURE", "TF_FAILURE",
}


def missing_readiness(snapshot: dict, smoother_required: bool) -> list[str]:
    required = [
        "map_server_active", "planner_server_active", "planner_action",
        "map_received", "costmap_received", "tf_ready",
    ]
    if smoother_required:
        required.extend(("smoother_server_active", "smoother_action", "footprint_received", "base_link_tf_ready"))
    return [name for name in required if not snapshot.get(name, False)]


def classify_readiness_timeout(snapshot: dict, smoother_required: bool) -> str:
    missing = missing_readiness(snapshot, smoother_required)
    if missing == ["tf_ready"]:
        return "TF_FAILURE"
    return "STACK_READY_TIMEOUT"


def failure_metrics(failure_type: str) -> dict:
    if failure_type not in FAILURE_TYPES:
        failure_type = "PLANNER_EXCEPTION"
    return {
        "failure_type": failure_type,
        "planning_success": False,
        "planner_success": False,
        "smoother_success": None,
        "collision_free": False,
        "collision_valid": None,
        "kinematically_feasible": False,
        "kinematic_valid": None,
        "endpoint_valid": None,
        "primary_result": failure_type,
        "secondary_diagnostic_flags": [],
        "total_pipeline_time_ms": 0.0,
    }


def validation_result(metrics: dict) -> tuple[str, list[str]]:
    """Apply deterministic validation precedence without hiding secondary flags."""
    failed = []
    if metrics.get("collision_valid") is False:
        failed.append("COLLISION_FAILURE")
    if metrics.get("kinematic_valid") is False:
        failed.append("KINEMATIC_VIOLATION")
    if metrics.get("endpoint_valid") is False:
        failed.append("ENDPOINT_ERROR")
    return (failed[0] if failed else "SUCCESS", failed[1:])


def bounded_process_shutdown(process, signal_group=os.killpg) -> list[int]:
    """Request launch shutdown, then use bounded TERM/KILL fallbacks."""
    sent = []
    if process is None or process.poll() is not None:
        return sent
    for value, timeout in (
        (signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3),
    ):
        try:
            signal_group(process.pid, value)
        except ProcessLookupError:
            break
        sent.append(value)
        try:
            process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
    return sent
