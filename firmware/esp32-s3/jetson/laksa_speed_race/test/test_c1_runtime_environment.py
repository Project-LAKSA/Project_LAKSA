"""Gate-B runtime-overlay and fail-fast harness regressions."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from laksa_speed_race import closed_loop_qualification as qualification
from laksa_speed_race.ros_domain import (
    C1_DOMAIN_MAX,
    C1_DOMAIN_MIN,
    RosDomainError,
    RosDomainLease,
    VERIFIED_DOMAIN_MAX,
    VERIFIED_DOMAIN_MIN,
    allocate_isolated_domain,
    bounded_c1_domain,
    parse_domain_id,
)
from laksa_speed_race.runtime_preflight import (
    RuntimePreflightError,
    validate_ackermann_runtime,
    validate_f1tenth_gym_runtime,
)


class AckermannDriveStamped:
    pass


def _importer(name: str):
    assert name == "ackermann_msgs.msg"
    return SimpleNamespace(AckermannDriveStamped=AckermannDriveStamped)


def _runner(prefix: Path, returncode: int = 0):
    def run(command, **kwargs):
        assert command == ["ros2", "pkg", "prefix", "ackermann_msgs"]
        assert kwargs["check"] is False
        return subprocess.CompletedProcess(
            command, returncode, stdout=f"{prefix}\n" if returncode == 0 else "", stderr="missing"
        )

    return run


def _prefix(tmp_path: Path) -> Path:
    prefix = tmp_path / "ackermann"
    setup = prefix / "share" / "ackermann_msgs" / "local_setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text("# fixture\n")
    return prefix


def _gym_checkout(tmp_path: Path) -> tuple[Path, SimpleNamespace]:
    checkout = tmp_path / "f1tenth_gym_checkout"
    package = checkout / "f1tenth_gym"
    package.mkdir(parents=True)
    module_file = package / "__init__.py"
    module_file.write_text("# fixture\n")
    return checkout, SimpleNamespace(__file__=str(module_file))


def _gym_runner(checkout: Path, sha: str, status: str = ""):
    def run(command, **kwargs):
        assert kwargs["check"] is False
        if command == ["git", "-C", str(checkout.resolve()), "rev-parse", "HEAD"]:
            return subprocess.CompletedProcess(command, 0, stdout=f"{sha}\n", stderr="")
        if command == ["git", "-C", str(checkout.resolve()), "status", "--short"]:
            return subprocess.CompletedProcess(command, 0, stdout=status, stderr="")
        raise AssertionError(f"unexpected command: {command}")

    return run


def _valid_gym_evidence(tmp_path: Path):
    checkout, module = _gym_checkout(tmp_path)
    sha = qualification.F1TENTH_GYM_SHA
    return validate_f1tenth_gym_runtime(
        checkout,
        sha,
        importer=lambda name: module if name == "f1tenth_gym" else None,
        command_runner=_gym_runner(checkout, sha),
    )


def _fake_domain_lease(tmp_path: Path, domain_id: int = 221) -> RosDomainLease:
    stream = (tmp_path / f"domain-{domain_id}.lock").open("a+")
    return RosDomainLease(domain_id, "test", Path(stream.name), stream)


def test_ackermann_preflight_passes_for_exact_overlay(tmp_path):
    prefix = _prefix(tmp_path)
    evidence = validate_ackermann_runtime(
        prefix, importer=_importer, command_runner=_runner(prefix)
    )
    assert evidence.expected_prefix == str(prefix.resolve())
    assert evidence.resolved_prefix == str(prefix.resolve())
    assert evidence.message_class.endswith(".AckermannDriveStamped")


def test_ackermann_preflight_fails_when_setup_is_missing(tmp_path):
    with pytest.raises(RuntimePreflightError, match="setup file is missing"):
        validate_ackermann_runtime(
            tmp_path / "missing", importer=_importer, command_runner=_runner(tmp_path)
        )


def test_ackermann_preflight_fails_when_python_module_is_missing(tmp_path):
    prefix = _prefix(tmp_path)

    def missing(_name: str):
        raise ModuleNotFoundError("ackermann_msgs")

    with pytest.raises(RuntimePreflightError, match="Python import failed"):
        validate_ackermann_runtime(prefix, importer=missing, command_runner=_runner(prefix))


def test_ackermann_preflight_fails_for_wrong_overlay(tmp_path):
    expected = _prefix(tmp_path)
    wrong = tmp_path / "wrong"
    with pytest.raises(RuntimePreflightError, match="expected"):
        validate_ackermann_runtime(
            expected, importer=_importer, command_runner=_runner(wrong)
        )


def test_ackermann_preflight_requires_message_class(tmp_path):
    prefix = _prefix(tmp_path)
    with pytest.raises(RuntimePreflightError, match="AckermannDriveStamped"):
        validate_ackermann_runtime(
            prefix,
            importer=lambda _name: SimpleNamespace(),
            command_runner=_runner(prefix),
        )


def test_f1tenth_gym_preflight_passes_for_exact_checkout(tmp_path):
    checkout, module = _gym_checkout(tmp_path)
    sha = qualification.F1TENTH_GYM_SHA
    evidence = validate_f1tenth_gym_runtime(
        checkout,
        sha,
        importer=lambda name: module if name == "f1tenth_gym" else None,
        command_runner=_gym_runner(checkout, sha),
    )
    assert evidence.checkout == str(checkout.resolve())
    assert evidence.actual_sha == sha
    assert evidence.expected_sha == sha
    assert Path(evidence.imported_from) == Path(module.__file__).resolve()


def test_wrong_f1tenth_gym_provenance_is_rejected(tmp_path):
    checkout, _module = _gym_checkout(tmp_path)
    wrong = tmp_path / "system-site-packages" / "f1tenth_gym" / "__init__.py"
    wrong.parent.mkdir(parents=True)
    wrong.write_text("# wrong fixture\n")
    sha = qualification.F1TENTH_GYM_SHA
    with pytest.raises(RuntimePreflightError, match="expected inside"):
        validate_f1tenth_gym_runtime(
            checkout,
            sha,
            importer=lambda _name: SimpleNamespace(__file__=str(wrong)),
            command_runner=_gym_runner(checkout, sha),
        )


def test_f1tenth_gym_sha_mismatch_is_rejected(tmp_path):
    checkout, module = _gym_checkout(tmp_path)
    with pytest.raises(RuntimePreflightError, match="SHA mismatch"):
        validate_f1tenth_gym_runtime(
            checkout,
            qualification.F1TENTH_GYM_SHA,
            importer=lambda _name: module,
            command_runner=_gym_runner(checkout, "0" * 40),
        )


def test_modified_f1tenth_gym_checkout_is_rejected(tmp_path):
    checkout, module = _gym_checkout(tmp_path)
    sha = qualification.F1TENTH_GYM_SHA
    with pytest.raises(RuntimePreflightError, match="checkout is modified"):
        validate_f1tenth_gym_runtime(
            checkout,
            sha,
            importer=lambda _name: module,
            command_runner=_gym_runner(checkout, sha, " M f1tenth_gym/envs/base_classes.py\n"),
        )


class _Process:
    def __init__(self, returncode=None):
        self.returncode = returncode

    def poll(self):
        return self.returncode


class _Session:
    def __init__(self, returncode=None):
        self.process = _Process(returncode)


def test_critical_child_exit_before_ready_fails_without_probe():
    calls = 0

    def probe():
        nonlocal calls
        calls += 1
        return set(), set()

    with pytest.raises(qualification.QualificationHarnessError, match="exited before ready"):
        qualification.wait_for_ready(_Session(7), timeout_s=0.1, probe=probe)
    assert calls == 0


def test_ready_requires_two_complete_graph_snapshots():
    snapshots = 0

    def probe():
        nonlocal snapshots
        snapshots += 1
        return set(qualification.REQUIRED_READY_NODES), set(qualification.REQUIRED_READY_TOPICS)

    evidence = qualification.wait_for_ready(
        _Session(), timeout_s=0.5, probe=probe, poll_s=0.0
    )
    assert snapshots == 2
    assert set(evidence["nodes"]) == qualification.REQUIRED_READY_NODES
    assert set(evidence["topics"]) == qualification.REQUIRED_READY_TOPICS


def test_runtime_graph_probe_uses_bounded_discovery_spin(monkeypatch):
    commands = []
    environments = []

    def run(command, **kwargs):
        commands.append(command)
        environments.append(kwargs["env"])
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(qualification.subprocess, "run", run)
    qualification.runtime_graph_snapshot({"ROS_DOMAIN_ID": "229"})
    assert commands == [
        ["ros2", "node", "list", "--no-daemon", "--spin-time", "3.0"],
        ["ros2", "topic", "list", "--no-daemon", "--spin-time", "3.0"],
    ]
    assert environments == [{"ROS_DOMAIN_ID": "229"}, {"ROS_DOMAIN_ID": "229"}]


def test_preflight_failure_does_not_start_runtime(monkeypatch, tmp_path):
    started = 0

    def fail(_prefix):
        raise RuntimePreflightError("fixture missing")

    def forbidden_start(*_args, **_kwargs):
        nonlocal started
        started += 1
        raise AssertionError("runtime must not start")

    monkeypatch.setattr(qualification, "validate_ackermann_runtime", fail)
    monkeypatch.setattr(qualification, "OwnedProcessSession", forbidden_start)
    monkeypatch.setattr(
        qualification,
        "claim_explicit_domain",
        lambda *_args, **_kwargs: _fake_domain_lease(tmp_path),
    )
    rc = qualification.main(
        [
            "--mode", "zero-step", "--ackermann-prefix", str(tmp_path / "missing"),
            "--output-dir", str(tmp_path / "result"), "--domain-id", "221",
        ]
    )
    assert rc == 1
    assert started == 0
    evidence = json.loads((tmp_path / "result" / "harness_result.json").read_text())
    assert evidence["status"] == "FAIL"
    assert "fixture missing" in evidence["error"]
    assert not (tmp_path / "result" / "summary.json").exists()


def test_preflight_mode_passes_without_starting_runtime(monkeypatch, tmp_path):
    prefix = _prefix(tmp_path)
    started = 0

    monkeypatch.setattr(
        qualification,
        "validate_ackermann_runtime",
        lambda _prefix: validate_ackermann_runtime(
            prefix, importer=_importer, command_runner=_runner(prefix)
        ),
    )
    monkeypatch.setattr(
        qualification,
        "validate_f1tenth_gym_runtime",
        lambda _checkout, _sha: _valid_gym_evidence(tmp_path),
    )

    def forbidden_start(*_args, **_kwargs):
        nonlocal started
        started += 1
        raise AssertionError("preflight-only mode must not launch")

    monkeypatch.setattr(qualification, "OwnedProcessSession", forbidden_start)
    monkeypatch.setattr(
        qualification,
        "claim_explicit_domain",
        lambda *_args, **_kwargs: _fake_domain_lease(tmp_path),
    )
    rc = qualification.main(
        [
            "--mode", "preflight", "--ackermann-prefix", str(prefix),
            "--output-dir", str(tmp_path / "result"), "--domain-id", "221",
        ]
    )
    assert rc == 0
    assert started == 0
    evidence = json.loads((tmp_path / "result" / "harness_result.json").read_text())
    assert evidence["status"] == "PASS"


def test_zero_step_artifacts_require_one_evaluation_and_no_gym_step(tmp_path):
    summary = {
        "simulator_step_count": 0,
        "accepted_drive_requests": 1,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary))
    with (tmp_path / "controller_telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "state_stamp_ns", "delta_applied_rad", "upstream_linear_mps",
                "physical_feasibility", "safety_veto_pass",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "state_stamp_ns": 123,
                "delta_applied_rad": 0.087,
                "upstream_linear_mps": 0.1,
                "physical_feasibility": "true",
                "safety_veto_pass": "true",
            }
        )
    evidence = qualification.validate_zero_step_artifacts(tmp_path)
    assert evidence["controller_evaluations"] == 1
    assert evidence["gym_steps"] == 0


def test_zero_step_artifacts_reject_any_gym_step(tmp_path):
    (tmp_path / "summary.json").write_text(json.dumps({"simulator_step_count": 1}))
    (tmp_path / "controller_telemetry.csv").write_text("state_stamp_ns\n")
    with pytest.raises(qualification.QualificationHarnessError, match="simulator_step_count"):
        qualification.validate_zero_step_artifacts(tmp_path)


def test_one_step_launch_permits_exactly_one_gym_step():
    args = SimpleNamespace(mode="one-step", output_dir="/tmp/result")
    command = qualification._launch_command(args)
    assert "qualification_step_limit:=1" in command
    assert "include_gym:=true" in command


def test_short_horizon_launch_uses_frozen_50_step_limit():
    args = SimpleNamespace(mode="short-horizon", output_dir="/tmp/result")
    command = qualification._launch_command(args)
    assert qualification.SHORT_HORIZON_STEPS == 50
    assert "qualification_step_limit:=50" in command
    assert "include_gym:=true" in command


def test_one_step_artifacts_require_exactly_one_transition(tmp_path):
    stamp = 1_000_000_000
    summary = {
        "simulator_step_count": 1,
        "accepted_drive_requests": 1,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
        "qualification_transition": {
            "command_index": 1,
            "state_n": {
                "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0, "speed_mps": 0.0,
                "stamp_ns": stamp, "full_body_clearance_m": 0.1,
            },
            "state_n1": {
                "x_m": 1.0001, "y_m": 2.0, "yaw_rad": 0.00001,
                "speed_mps": 0.01, "stamp_ns": stamp + 10_000_000,
                "full_body_clearance_m": 0.1, "collision": False, "off_track": False,
            },
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary))
    with (tmp_path / "controller_telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "state_stamp_ns", "delta_applied_rad", "upstream_linear_mps",
                "upstream_angular_rps", "kappa_cmd_1pm", "physical_feasibility",
                "safety_veto_pass", "downstream_steering_saturated",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "state_stamp_ns": stamp,
                "delta_applied_rad": 0.08,
                "upstream_linear_mps": 0.1,
                "upstream_angular_rps": 0.025,
                "kappa_cmd_1pm": 0.25,
                "physical_feasibility": "true",
                "safety_veto_pass": "true",
                "downstream_steering_saturated": "0",
            }
        )
    (tmp_path / "trajectory.csv").write_text(
        "step,sim_time_s,x_m,y_m,yaw_rad\n1,0.01,1.0001,2.0,0.00001\n"
    )
    evidence = qualification.validate_one_step_artifacts(tmp_path)
    assert evidence["controller_evaluations"] == 1
    assert evidence["gym_steps"] == 1
    assert evidence["steering_sign_semantics"] == "PASS"


def test_short_horizon_artifacts_require_exact_lockstep_counts(tmp_path):
    target = qualification.SHORT_HORIZON_STEPS
    summary = {
        "simulator_step_count": target,
        "accepted_drive_requests": target,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
        "sim_time_s": target * qualification.DT_S,
        "initial_state_evidence": {
            "speed_mps": 0.0,
            "signed_cte_m": 0.25,
            "heading_error_rad": 0.1,
            "full_body_clearance_m": 0.2,
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary))

    telemetry_fields = [
        "state_stamp_ns", "sequence", "upstream_linear_mps", "upstream_angular_rps",
        "kappa_cmd_1pm", "delta_applied_rad", "downstream_steering_saturated",
    ]
    feasibility_fields = [
        "state_stamp_ns", "controller_input_stamp_ns", "controller_output_stamp_ns",
        "controller_latency_ms", "physical_feasibility", "safety_veto_pass",
    ]
    trajectory_fields = [
        "step", "sim_time_s", "x_m", "y_m", "yaw_rad", "actual_speed_mps",
        "signed_cte_m", "heading_error_rad", "collision", "off_track",
        "full_body_clearance_m", "gym_step_latency_ms",
    ]
    command_fields = ["step", "applied_speed_mps", "applied_steering_rad"]
    streams = {}
    writers = {}
    for name, fields in (
        ("controller_telemetry.csv", telemetry_fields),
        ("controller_feasibility.csv", feasibility_fields),
        ("trajectory.csv", trajectory_fields),
        ("commands.csv", command_fields),
    ):
        streams[name] = (tmp_path / name).open("w", newline="")
        writers[name] = csv.DictWriter(streams[name], fieldnames=fields)
        writers[name].writeheader()
    try:
        for step in range(1, target + 1):
            stamp = 1_000_000_000 + step * 10_000_000
            steering = 0.08 + step * 1e-5
            writers["controller_telemetry.csv"].writerow(
                {
                    "state_stamp_ns": stamp,
                    "sequence": step,
                    "upstream_linear_mps": 0.1,
                    "upstream_angular_rps": 0.025,
                    "kappa_cmd_1pm": 0.25,
                    "delta_applied_rad": steering,
                    "downstream_steering_saturated": 0,
                }
            )
            writers["controller_feasibility.csv"].writerow(
                {
                    "state_stamp_ns": stamp,
                    "controller_input_stamp_ns": stamp,
                    "controller_output_stamp_ns": stamp,
                    "controller_latency_ms": 1.0,
                    "physical_feasibility": "true",
                    "safety_veto_pass": "true",
                }
            )
            writers["trajectory.csv"].writerow(
                {
                    "step": step,
                    "sim_time_s": step * qualification.DT_S,
                    "x_m": step * 0.001,
                    "y_m": 0.0,
                    "yaw_rad": step * 0.0001,
                    "actual_speed_mps": 0.09,
                    "signed_cte_m": 0.25 - step * 0.001,
                    "heading_error_rad": 0.1 - step * 0.001,
                    "collision": 0,
                    "off_track": 0,
                    "full_body_clearance_m": 0.2,
                    "gym_step_latency_ms": 0.5,
                }
            )
            writers["commands.csv"].writerow(
                {
                    "step": step,
                    "applied_speed_mps": 0.1,
                    "applied_steering_rad": steering,
                }
            )
    finally:
        for stream in streams.values():
            stream.close()

    evidence = qualification.validate_short_horizon_artifacts(tmp_path)
    assert evidence["executed_steps"] == target
    assert evidence["controller_evaluations"] == target
    assert evidence["gym_steps"] == target
    assert evidence["lockstep_causality"] == "PASS"
    assert (tmp_path / "short_horizon_telemetry.csv").is_file()


def test_trial1_artifacts_require_exact_three_lap_lockstep(tmp_path):
    summary = {
        "simulator_step_count": 3,
        "accepted_drive_requests": 3,
        "completed_laps": 3,
        "lap_count": 3,
        "done": True,
        "terminal_state": "COMPLETE",
        "fault": None,
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
        "terminal_zero_observed": True,
        "sim_time_s": 0.03,
        "acceptance": {"overall": "PASS"},
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary))
    field_sets = {
        "controller_telemetry.csv": [
            "state_stamp_ns", "sequence", "progress_index", "path_copy",
            "upstream_linear_mps", "upstream_angular_rps", "kappa_cmd_1pm",
            "delta_applied_rad", "downstream_steering_saturated",
        ],
        "controller_feasibility.csv": [
            "state_stamp_ns", "controller_input_stamp_ns", "controller_output_stamp_ns",
            "controller_latency_ms", "physical_feasibility", "safety_veto_pass",
        ],
        "trajectory.csv": [
            "step", "sim_time_s", "x_m", "y_m", "yaw_rad", "actual_speed_mps",
            "signed_cte_m", "heading_error_rad", "collision", "off_track",
            "full_body_clearance_m", "gym_step_latency_ms", "lap_count",
        ],
        "commands.csv": ["step", "applied_speed_mps", "applied_steering_rad"],
        "events.csv": ["step", "sim_time_s", "event", "value"],
    }
    streams = {}
    writers = {}
    for name, fields in field_sets.items():
        streams[name] = (tmp_path / name).open("w", newline="")
        writers[name] = csv.DictWriter(streams[name], fieldnames=fields)
        writers[name].writeheader()
    try:
        for step in range(1, 4):
            stamp = 1_000_000_000 + step * 10_000_000
            writers["controller_telemetry.csv"].writerow(
                {
                    "state_stamp_ns": stamp, "sequence": step,
                    "progress_index": step, "path_copy": step - 1,
                    "upstream_linear_mps": 0.1, "upstream_angular_rps": 0.025,
                    "kappa_cmd_1pm": 0.25, "delta_applied_rad": 0.08,
                    "downstream_steering_saturated": 0,
                }
            )
            writers["controller_feasibility.csv"].writerow(
                {
                    "state_stamp_ns": stamp, "controller_input_stamp_ns": stamp,
                    "controller_output_stamp_ns": stamp, "controller_latency_ms": 1.0,
                    "physical_feasibility": "true", "safety_veto_pass": "true",
                }
            )
            writers["trajectory.csv"].writerow(
                {
                    "step": step, "sim_time_s": step * qualification.DT_S,
                    "x_m": step * 0.1, "y_m": 0.0, "yaw_rad": 0.01 * step,
                    "actual_speed_mps": 0.09, "signed_cte_m": 0.2,
                    "heading_error_rad": 0.01, "collision": 0, "off_track": 0,
                    "full_body_clearance_m": 0.1, "gym_step_latency_ms": 0.5,
                    "lap_count": step,
                }
            )
            writers["commands.csv"].writerow(
                {"step": step, "applied_speed_mps": 0.1, "applied_steering_rad": 0.08}
            )
            writers["events.csv"].writerow(
                {
                    "step": step, "sim_time_s": step * qualification.DT_S,
                    "event": "lap_complete", "value": step,
                }
            )
        writers["events.csv"].writerow(
            {"step": 3, "sim_time_s": 0.03, "event": "terminal_state", "value": "COMPLETE"}
        )
    finally:
        for stream in streams.values():
            stream.close()

    evidence = qualification.validate_trial1_artifacts(tmp_path)
    assert evidence["status"] == "PASS"
    assert evidence["exact_three_laps"] == "PASS"
    assert evidence["controller_evaluations"] == 3
    assert evidence["gym_steps"] == 3
    assert (tmp_path / "trial1_telemetry.csv").is_file()


def test_native_launcher_sources_ackermann_before_c1_overlay():
    root = Path(__file__).resolve().parents[1]
    script = (root / "docker" / "c1_native_runtime.sh").read_text()
    base = script.index("source /opt/ros/humble/setup.bash")
    ackermann = script.index('source "${ACKERMANN_MSGS_SETUP}"')
    c1 = script.index('source "${LAKSA_C1_WORKSPACE}/install/setup.bash"')
    assert base < ackermann < c1
    assert 'LAKSA_C1_VENV="${LAKSA_C1_VENV:-${LAKSA_C1_WORKSPACE}/venv}"' in script
    assert 'F1TENTH_GYM_CHECKOUT="${F1TENTH_GYM_CHECKOUT:-/tmp/laksa-c1-native/f1tenth_gym}"' in script
    assert 'F1TENTH_GYM_PYDEPS="${F1TENTH_GYM_PYDEPS:-/tmp/laksa-c1-native/pydeps}"' in script
    assert 'export PYTHONPATH="${F1TENTH_GYM_CHECKOUT}:${F1TENTH_GYM_PYDEPS}:${LAKSA_C1_VENV}/lib/python3.10/site-packages:' in script
    assert '--f1tenth-gym-checkout "${F1TENTH_GYM_CHECKOUT}"' in script


def test_launch_shuts_down_when_each_critical_child_exits():
    root = Path(__file__).resolve().parents[1]
    source = (root / "launch" / "c1_nav2_mppi_three_lap.launch.py").read_text()
    for target in ("raceline", "host", "ackermann"):
        assert f"target_action={target}" in source
    assert 'DeclareLaunchArgument("include_gym"' in source
    assert 'DeclareLaunchArgument("gym_start_delay_s"' in source
    assert 'period=LaunchConfiguration("gym_start_delay_s")' in source


def test_zero_step_launch_delays_gym_until_after_readiness_window():
    args = SimpleNamespace(mode="zero-step", output_dir="/tmp/result")
    command = qualification._launch_command(args)
    assert "include_gym:=true" in command
    assert "gym_start_delay_s:=30.0" in command


def test_readiness_launch_excludes_gym_without_delay():
    args = SimpleNamespace(mode="readiness", output_dir="/tmp/result")
    command = qualification._launch_command(args)
    assert "include_gym:=false" in command
    assert "gym_start_delay_s:=0.0" in command


def test_trial1_launch_uses_unbounded_three_lap_gate():
    args = SimpleNamespace(mode="trial1", output_dir="/tmp/result")
    command = qualification._launch_command(args)
    assert "qualification_step_limit:=-1" in command
    assert "include_gym:=true" in command


@pytest.mark.parametrize("value", [VERIFIED_DOMAIN_MIN, VERIFIED_DOMAIN_MAX, "0", "232"])
def test_ros_domain_validation_accepts_boundaries(value):
    assert parse_domain_id(value, source="test") == int(value)


@pytest.mark.parametrize("value", [-1, 233, 999, "abc", "", "   "])
def test_ros_domain_validation_rejects_invalid_values(value):
    with pytest.raises(RosDomainError, match="allowed integer range is 0..232"):
        parse_domain_id(value, source="test override")


def test_allocator_is_bounded_for_wide_integer_input_range():
    for seed in range(-10000, 10001):
        assert C1_DOMAIN_MIN <= bounded_c1_domain(seed) <= C1_DOMAIN_MAX


def test_historical_233_allocator_input_maps_to_valid_domain():
    selected = bounded_c1_domain(233)
    assert selected == 202
    assert VERIFIED_DOMAIN_MIN <= selected <= VERIFIED_DOMAIN_MAX


def test_allocator_skips_occupied_candidate_and_holds_lease(tmp_path):
    first = bounded_c1_domain(233)
    seen = []

    def occupied(domain_id: int) -> bool:
        seen.append(domain_id)
        return domain_id == first

    lease = allocate_isolated_domain(233, occupied=occupied, lock_root=tmp_path)
    try:
        assert seen == [202, 203]
        assert lease.domain_id == 203
    finally:
        lease.release()


def test_explicit_invalid_override_fails_before_preflight_or_children(monkeypatch, tmp_path):
    preflight_calls = 0
    child_starts = 0

    def preflight(_prefix):
        nonlocal preflight_calls
        preflight_calls += 1
        raise AssertionError("preflight must not run for invalid domain")

    def child(*_args, **_kwargs):
        nonlocal child_starts
        child_starts += 1
        raise AssertionError("critical children must not start")

    monkeypatch.setattr(qualification, "validate_ackermann_runtime", preflight)
    monkeypatch.setattr(qualification, "OwnedProcessSession", child)
    rc = qualification.main(
        [
            "--mode", "zero-step", "--domain-id", "233",
            "--output-dir", str(tmp_path / "invalid-domain"),
            "--domain-lock-root", str(tmp_path / "locks"),
        ]
    )
    assert rc == 1
    assert preflight_calls == 0
    assert child_starts == 0
    evidence = json.loads(
        (tmp_path / "invalid-domain" / "harness_result.json").read_text()
    )
    assert evidence["status"] == "FAIL"
    assert "ROS_DOMAIN_ID 233" in evidence["error"]
