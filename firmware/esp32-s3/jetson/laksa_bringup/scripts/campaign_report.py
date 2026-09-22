#!/usr/bin/env python3
"""Final machine-readable characterization completion report."""
from __future__ import annotations
import json
from pathlib import Path

def write_report(state: dict, output: Path, *, model_path=None, validation=None):
    tests = state.get("tests", {})
    parameters = state.get("identified_parameters", {})
    if model_path and Path(model_path).is_file() and not parameters:
        try:
            text = Path(model_path).read_text()
            try:
                parameters = json.loads(text).get("parameters", {})
            except json.JSONDecodeError:
                # PyYAML is present in the Jetson ROS environment; retain a
                # dependency-free fallback for Mac dry-runs.
                import yaml
                parameters = (yaml.safe_load(text) or {}).get("parameters", {})
        except (OSError, ImportError, ValueError):
            parameters = {}
    report = {
        "schema_version": "laksa-characterization-completion-v005",
        "status": state.get("status", "IN_PROGRESS"),
        "campaign_id": state.get("campaign_id"),
        "complete_test_matrix": tests,
        "accepted_evidence": state.get("accepted_evidence", {}),
        "accepted_datasets": [k for k,v in tests.items() if v.get("quality") == "DATA_QUALITY_PASS"],
        "rejected_datasets": [k for k,v in tests.items() if v.get("quality") == "DATA_QUALITY_FAIL"],
        "identified_parameters": parameters,
        "remaining_unknown_or_non_blocking": state.get("unknown", []) + [
            name for name, row in parameters.items()
            if isinstance(row, dict) and (row.get("value") == "UNKNOWN" or row.get("provenance") == "UNKNOWN")
        ],
        "v005_model": str(model_path) if model_path else None,
        "held_out_validation": validation or {},
        "readiness": "READY_FOR_PATH_FOLLOWING_CONTROLLER_SIMULATION" if validation and validation.get("status") == "PASS" else "NOT_READY",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report
