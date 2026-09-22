#!/usr/bin/env python3
"""Deterministic, provenance-preserving V005 fit from accepted evidence.

The fitter is deliberately conservative: it copies measured values from the
accepted V004 baseline, overlays only PASS evidence, and leaves unsupported
parameters UNKNOWN.  It never treats a failed/review dataset as calibration.
"""
from __future__ import annotations
import json
from pathlib import Path
try:
    import yaml
except ModuleNotFoundError:  # macOS dry-run path does not require PyYAML
    yaml = None

PROVENANCE = {"MEASURED", "IDENTIFIED", "DERIVED_FROM_MEASURED", "CATALOG_PRIOR",
              "ESTIMATED_LOW_CONFIDENCE", "UNKNOWN"}

def _read(path):
    if not Path(path).is_file():
        return {}
    if yaml is None:
        return {}
    try:
        return yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}

def _pass(report):
    return isinstance(report, dict) and report.get("status") in {"PASS", "DATA_QUALITY_PASS"}

def fit_v005(base_model: Path, evidence_root: Path, output: Path, *, campaign_id="UNKNOWN"):
    base = _read(base_model)
    params = {k: dict(v) if isinstance(v, dict) else {"value": v, "provenance": "UNKNOWN"}
              for k, v in (base.get("parameters") or {}).items()}
    sources = []
    for path in sorted(Path(evidence_root).rglob("analysis.json")):
        report = json.loads(path.read_text())
        if not _pass(report):
            continue
        test = report.get("test_id", path.parent.name.split("_")[0])
        sources.append({"test_id": test, "path": str(path), "status": report.get("status")})
        # The calibrated speed scale is the only automatically-overlaid
        # scalar whose schema is stable across historical evidence formats.
        gain = report.get("erpm_to_mps_gain") or report.get("erpm_to_speed_gain_m_per_s_per_erpm")
        if gain is not None:
            params["erpm_to_ground_speed_mps_per_erpm"] = {
                "value": float(gain), "units": "m/s/eRPM", "provenance": "MEASURED",
                "confidence": "MEDIUM", "supporting_evidence": [test]}
    for name, row in params.items():
        row.setdefault("provenance", "UNKNOWN")
        if row["provenance"] not in PROVENANCE:
            row["provenance"] = "UNKNOWN"
        row.setdefault("confidence", "UNKNOWN")
        row.setdefault("supporting_evidence", [])
    result = {
        "model_id": "laksa_physical_v005_calibrated",
        "status": "CALIBRATED_PENDING_HELD_OUT_VALIDATION",
        "baseline_model": str(base_model), "campaign": campaign_id,
        "model_scope": "characterization_and_digital_twin_only",
        "parameters": params, "fit_sources": sources,
        "quality_rule": "DATA_QUALITY_FAIL evidence is excluded from fitting",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    if yaml is None:
        # JSON is a strict YAML 1.2 subset and keeps the artifact consumable
        # on a minimal Mac install without adding a runtime dependency.
        output.write_text(json.dumps(result, indent=2, sort_keys=False) + "\n")
    else:
        output.write_text(yaml.safe_dump(result, sort_keys=False))
    return result
