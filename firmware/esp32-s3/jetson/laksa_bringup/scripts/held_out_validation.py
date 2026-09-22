#!/usr/bin/env python3
"""Held-out V005 validation and quantitative residual reporting."""
from __future__ import annotations
import json, math
from pathlib import Path

CHANNELS = ("position", "heading", "speed", "yaw_rate", "curvature", "timing", "stopping")

def validate_v005(payload: dict, model_path: Path, output: Path):
    if not payload.get("held_out"):
        result = {"status": "DATA_QUALITY_FAIL", "reason": "HELD_OUT_REQUIRED", "model_promoted": False}
    else:
        metrics = {}
        for channel, rows in (payload.get("channels") or {}).items():
            residuals = [float(r["physical"]) - float(r["simulated"])
                         for r in rows if r.get("physical") is not None and r.get("simulated") is not None]
            if residuals:
                metrics[channel] = {"n": len(residuals),
                    "rmse": math.sqrt(sum(x*x for x in residuals)/len(residuals)),
                    "mae": sum(abs(x) for x in residuals)/len(residuals),
                    "max_abs_residual": max(abs(x) for x in residuals)}
        result = {"status": "PASS" if metrics else "DATA_QUALITY_REVIEW", "held_out": True,
                  "model": str(model_path), "metrics": metrics, "model_promoted": False,
                  "supported_quantities": list(CHANNELS)}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result

