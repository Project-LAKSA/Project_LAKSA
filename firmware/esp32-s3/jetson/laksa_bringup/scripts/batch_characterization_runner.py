#!/usr/bin/env python3
"""Execute one finite manifest-authorized supervised batch, fail closed.

This process has no ROS publishers. Each physical trial is delegated to the
existing supervised runner, which owns the only request and gate publishers.
A host-renewed lease is checked here and in that runner.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

from batch_characterization_manifest import digest, first_batch, followup_next, right_revalidation, t21a_resolution, t21b_trajectory_capture, t22_authoritative, t22_rehearsal, unlocked_next, validate
from batch_session_lease import FileLease

ROOT = Path(__file__).resolve().parent


def atomic_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def materialize_manifest(campaign: Path, manifest: dict) -> Path:
    """Create a derived manifest once; never mutate a different prior one."""
    purpose = manifest.get("purpose", "")
    if purpose == "T22_REHEARSAL_ONLY_NOT_VALID_FOR_CALIBRATION":
        name = "BATCH_T22_REHEARSAL_MANIFEST.json"
    elif purpose == "T22_AUTHORITATIVE_SPEED_CALIBRATION":
        name = "BATCH_T22_AUTHORITATIVE_MANIFEST.json"
    elif purpose == "UNLOCKED_POST_T22_CHARACTERIZATION_BATCH":
        name = "BATCH_UNLOCKED_POST_T22_MANIFEST.json"
    elif purpose == "FOLLOWUP_POST_T22_REPEATABILITY_AND_REVERSE_BATCH":
        name = "BATCH_FOLLOWUP_POST_T22_MANIFEST.json"
    elif purpose == "T21A_RESOLUTION_AFTER_T22_AND_T20_REANALYSIS":
        name = "BATCH_T21A_RESOLUTION_MANIFEST.json"
    elif purpose == "T21B_ACCEPTED_TRAJECTORY_CAPTURE":
        name = "BATCH_T21B_TRAJECTORY_CAPTURE_MANIFEST.json"
    elif purpose:
        name = "BATCH_T21A_RIGHT_REVALIDATION_MANIFEST.json"
    else:
        name = "BATCH_T21A_PAIR_MANIFEST.json"
    path = campaign / name
    if path.exists():
        existing = json.loads(path.read_text())
        if existing.get("manifest_sha256") != manifest["manifest_sha256"]:
            raise RuntimeError("EXISTING_BATCH_MANIFEST_HASH_MISMATCH")
        return path
    atomic_json(path, manifest)
    written = json.loads(path.read_text())
    if written.get("manifest_sha256") != manifest["manifest_sha256"] or digest(written) != manifest["manifest_sha256"]:
        raise RuntimeError("MANIFEST_WRITE_HASH_FAILURE")
    return path


def new_sessions(campaign: Path, stage: str, before: set[Path]) -> list[Path]:
    """Select only the one session directory created for the requested stage."""
    return sorted(set(campaign.glob(f"{stage}_*")) - before)


def terminal_ok(session: Path, item: dict) -> tuple[bool, str]:
    try:
        report = json.loads((session / "analysis.json").read_text())
        motion, cleanup = report["motion"], report["cleanup"]
        if report.get("test_id") != item["stage"] or report.get("variant") != item["variant"]:
            return False, "SESSION_IDENTITY_MISMATCH"
        status = report.get("status")
        if status not in ("COMPLETE", "ABORT"):
            return False, "NONTERMINAL_TRIAL_STATUS"
        if not motion.get("non_neutral_motion_requested"):
            # A pre-motion abort is a valid terminal outcome.  It must remain
            # distinguishable from a completed physical trial, but it must
            # never poison the next invocation as an incomplete trial.
            if status == "ABORT" and report.get("trial_analysis", {}).get("status") == "NOT_RUN":
                return True, "ABORT_PRE_MOTION"
            return False, "NO_NON_NEUTRAL_REQUEST_EVIDENCE"
        if not cleanup.get("neutral_request_attempted") or not cleanup.get("gate_disable_attempted"):
            return False, "NEUTRAL_OR_GATE_CLEANUP_MISSING"
        if not (session / "raw.jsonl").is_file():
            return False, "RAW_EVIDENCE_MISSING"
        return True, "COMPLETE"
    except (KeyError, OSError, TypeError, json.JSONDecodeError):
        return False, "TERMINAL_ARTIFACT_INVALID"


def main(argv=None):
    parser = argparse.ArgumentParser(description="Finite manifest-bound LAKSA supervised batch")
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--authorize")
    parser.add_argument("--lease-file", type=Path)
    parser.add_argument("--right-revalidation", action="store_true")
    parser.add_argument("--direct-revalidation", action="store_true")
    parser.add_argument("--t22-rehearsal", action="store_true")
    parser.add_argument("--t22-authoritative", action="store_true")
    parser.add_argument("--unlocked-next", action="store_true")
    parser.add_argument("--followup-next", action="store_true")
    parser.add_argument("--t21a-resolution", action="store_true")
    parser.add_argument("--t21b-trajectory-capture", action="store_true")
    args = parser.parse_args(argv)
    cfg = yaml.safe_load((ROOT / "../config/supervised_characterization_protocols.yaml").resolve().read_text()) or {}
    if sum(bool(x) for x in (args.t22_rehearsal, args.t22_authoritative, args.unlocked_next, args.followup_next, args.t21a_resolution, args.t21b_trajectory_capture)) > 1:
        parser.error("select only one specialized batch mode")
    manifest = (t21b_trajectory_capture(args.campaign,cfg) if args.t21b_trajectory_capture else
                t21a_resolution(args.campaign,cfg) if args.t21a_resolution else
                followup_next(args.campaign,cfg) if args.followup_next else
                unlocked_next(args.campaign,cfg) if args.unlocked_next else
                t22_authoritative(args.campaign,cfg) if args.t22_authoritative else
                t22_rehearsal(args.campaign,cfg) if args.t22_rehearsal else
                right_revalidation(args.campaign, cfg) if args.right_revalidation else first_batch(args.campaign,cfg))
    report = {"schema_version": "laksa-characterization-batch-report-v1", "next_stage": "T21A",
              "manifest": manifest, "session_authorized": False, "motion_command_published": False,
              "gate_enabled": False, "non_neutral_requests": 0, "trials": [],
              "stop_before": manifest["stop_before"], "stop_reason": manifest["stop_reason"]}
    if args.dry_run:
        report["status"] = "DRY_RUN_OK"
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if not args.lease_file or (args.right_revalidation and not args.direct_revalidation) or (not args.right_revalidation and not args.authorize):
        parser.error("live execution requires --authorize and --lease-file")
    phrase = 'LAKSA_BATCH_AUTHORIZE:' + manifest['manifest_sha256'] if args.right_revalidation else args.authorize
    validate(manifest, phrase)
    lease = FileLease(args.lease_file, manifest["lease_max_age_s"])
    lease.require()
    manifest_path = materialize_manifest(args.campaign, manifest)
    report.update(session_authorized=True, manifest_path=str(manifest_path))
    report_name = ("T21B_TRAJECTORY_CAPTURE_BATCH_REPORT.json" if args.t21b_trajectory_capture else
                   "T21A_RESOLUTION_BATCH_REPORT.json" if args.t21a_resolution else
                   "FOLLOWUP_POST_T22_BATCH_REPORT.json" if args.followup_next else
                   "UNLOCKED_POST_T22_BATCH_REPORT.json" if args.unlocked_next else
                   "T22_AUTHORITATIVE_BATCH_REPORT.json" if args.t22_authoritative else
                   f"{manifest['trials'][0]['stage']}_BATCH_REPORT.json")
    report_path = args.campaign / "batch_reports" / report_name
    print(json.dumps({"status": "BATCH_PREFLIGHT_ENTERED", "manifest_sha256": manifest["manifest_sha256"],
                      "lease_max_age_s": manifest["lease_max_age_s"], "motion_command_published": False}, sort_keys=True), flush=True)
    for item in manifest["trials"]:
        if not lease.valid():
            report.update(status="ABORT", abort_reason="SESSION_LEASE_EXPIRED_BEFORE_TRIAL")
            atomic_json(report_path, report)
            print(json.dumps(report, indent=2, sort_keys=True))
            return 2
        before = set(args.campaign.glob(f"{item['stage']}_*"))
        command = [sys.executable, str(ROOT / "supervised_stage_runner.py"), "--test", item["stage"],
                   "--variant", item["variant"], "--output-root", str(args.campaign),
                   "--session-authorization", phrase, "--batch-manifest", str(manifest_path),
                   "--lease-file", str(args.lease_file)] + (["--rehearsal-only"] if args.t22_rehearsal else [])
        result = subprocess.run(command, text=True, capture_output=True)
        created = new_sessions(args.campaign, item["stage"], before)
        session = created[-1] if len(created) == 1 else None
        valid, reason = terminal_ok(session, item) if session else (False, "SESSION_CREATION_AMBIGUOUS")
        report["trials"].append({"stage": item["stage"], "variant": item["variant"],
            "runner_returncode": result.returncode, "session": str(session) if session else None,
            "terminal_valid": valid, "reason": reason, "stdout": result.stdout[-4096:], "stderr": result.stderr[-4096:]})
        if result.returncode or not valid or not lease.valid():
            report.update(status="ABORT", abort_reason=("SESSION_LEASE_EXPIRED_AFTER_TRIAL" if not lease.valid() else reason))
            atomic_json(report_path, report)
            print(json.dumps(report, indent=2, sort_keys=True))
            return 2
    report["status"] = "COMPLETE_PENDING_HOST_INGEST_AND_ANALYSIS"
    atomic_json(report_path, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
