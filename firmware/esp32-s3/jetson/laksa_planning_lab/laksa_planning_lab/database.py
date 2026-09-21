"""SQLite checkpointing and constraint-first summaries."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sqlite3
from statistics import fmean, median

from .geometry import percentile, stable_hash


SCHEMA = """
PRAGMA journal_mode=WAL;
CREATE TABLE IF NOT EXISTS maps(map_id TEXT PRIMARY KEY, sha256 TEXT NOT NULL, metadata_json TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS scenarios(scenario_id TEXT PRIMARY KEY, map_id TEXT NOT NULL, scenario_json TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS configurations(config_hash TEXT PRIMARY KEY, method TEXT NOT NULL, config_json TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS runs(run_id INTEGER PRIMARY KEY AUTOINCREMENT, started_utc TEXT NOT NULL, preset TEXT NOT NULL, seed INTEGER NOT NULL);
CREATE TABLE IF NOT EXISTS results(
  map_sha256 TEXT NOT NULL, scenario_id TEXT NOT NULL, config_hash TEXT NOT NULL, method TEXT NOT NULL,
  distance_class TEXT NOT NULL, bearing_class TEXT NOT NULL, success INTEGER NOT NULL,
  failure_type TEXT NOT NULL DEFAULT 'SUCCESS', error TEXT, metrics_json TEXT NOT NULL,
  PRIMARY KEY(map_sha256, scenario_id, config_hash));
"""


class ResultStore:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.connection = sqlite3.connect(path)
        self.connection.executescript(SCHEMA)
        columns = {row[1] for row in self.connection.execute("PRAGMA table_info(results)")}
        if "failure_type" not in columns:
            self.connection.execute(
                "ALTER TABLE results ADD COLUMN failure_type TEXT NOT NULL DEFAULT 'SUCCESS'"
            )
        self.connection.commit()

    def register_map(self, entry: dict):
        self.connection.execute("INSERT OR REPLACE INTO maps VALUES(?,?,?)", (entry["map_id"], entry["sha256"], json.dumps(entry, sort_keys=True)))
        self.connection.commit()

    def register_scenario(self, scenario: dict):
        self.connection.execute("INSERT OR REPLACE INTO scenarios VALUES(?,?,?)", (scenario["scenario_id"], scenario["map_id"], json.dumps(scenario, sort_keys=True)))
        self.connection.commit()

    def register_configuration(self, method: str, config: dict) -> str:
        digest = stable_hash({"method": method, "config": config})
        self.connection.execute("INSERT OR IGNORE INTO configurations VALUES(?,?,?)", (digest, method, json.dumps(config, sort_keys=True)))
        self.connection.commit()
        return digest

    def completed(self, map_sha: str, scenario_id: str, config_hash: str) -> bool:
        return self.connection.execute("SELECT 1 FROM results WHERE map_sha256=? AND scenario_id=? AND config_hash=?", (map_sha, scenario_id, config_hash)).fetchone() is not None

    def record(self, scenario: dict, method: str, config_hash: str, metrics: dict, error: str = ""):
        constraints_pass = bool(
            metrics.get("planning_success")
            and metrics.get("collision_free")
            and metrics.get("kinematically_feasible")
        )
        failure_type = str(metrics.get("failure_type", "SUCCESS" if constraints_pass else "PLANNER_EXCEPTION"))
        success = constraints_pass and failure_type == "SUCCESS"
        self.connection.execute("""INSERT OR REPLACE INTO results(
                                map_sha256,scenario_id,config_hash,method,distance_class,bearing_class,
                                success,error,metrics_json,failure_type) VALUES(?,?,?,?,?,?,?,?,?,?)""",
                                (scenario["map_sha256"], scenario["scenario_id"], config_hash, method,
                                 scenario["distance_class"], scenario["bearing_class"], int(success), error,
                                 json.dumps(metrics, sort_keys=True, allow_nan=False), failure_type))
        self.connection.commit()

    def ranking_key(self, config_hash: str):
        rows = self.connection.execute("SELECT success, metrics_json FROM results WHERE config_hash=?", (config_hash,)).fetchall()
        if not rows:
            return (10**9,) * 8
        metrics = [json.loads(row[1]) for row in rows]
        collisions = sum(bool(m.get("planning_success")) and not m.get("collision_free", False) for m in metrics)
        kinematic = sum(bool(m.get("planning_success")) and not m.get("kinematically_feasible", False) for m in metrics)
        failures = sum(not m.get("planning_success", False) for m in metrics)
        return (collisions, kinematic, failures, -sum(row[0] for row in rows) / len(rows),
                fmean([m.get("cusps", 999) for m in metrics]), fmean([m.get("self_intersections", 999) for m in metrics]),
                fmean([m.get("excess_path_ratio", 999) for m in metrics]), fmean([m.get("total_pipeline_time_ms", 999999) for m in metrics]))

    def smoke_healthy(self, config_hash: str) -> bool:
        rows = self.connection.execute("SELECT success,error FROM results WHERE config_hash=?", (config_hash,)).fetchall()
        if not rows or any((error or "").startswith("stack failure:") for _, error in rows): return False
        return any(success for success, _ in rows)

    def export_summaries(self, output_dir: Path):
        rows = self.connection.execute("SELECT method,config_hash,distance_class,bearing_class,success,metrics_json FROM results").fetchall()
        groups = {}
        for method, digest, distance, bearing, success, payload in rows:
            metrics = json.loads(payload)
            for stratum, value in (("ALL", "ALL"), ("DISTANCE", distance), ("BEARING", bearing)):
                groups.setdefault((method, digest, stratum, value), []).append((success, metrics))
        output_dir.mkdir(parents=True, exist_ok=True)
        fields = ["method", "configuration_hash", "stratum", "class", "count", "success_rate", "collisions", "kinematic_violations",
                  "metric", "mean", "median", "p90", "p95", "worst"]
        output_rows = []
        for (method, digest, stratum, value), items in sorted(groups.items()):
            collisions = sum(m.get("planning_success", False) and not m.get("collision_free", False) for _, m in items)
            violations = sum(m.get("planning_success", False) and not m.get("kinematically_feasible", False) for _, m in items)
            for metric in ("path_length_m", "minimum_clearance_m", "reverse_fraction", "cusps", "self_intersections", "excess_path_ratio", "curvature_total_variation", "planning_time_ms", "total_pipeline_time_ms"):
                values = [float(m[metric]) for _, m in items if metric in m]
                if not values: continue
                output_rows.append({"method": method, "configuration_hash": digest, "stratum": stratum, "class": value,
                                    "count": len(items), "success_rate": sum(ok for ok, _ in items) / len(items),
                                    "collisions": collisions, "kinematic_violations": violations, "metric": metric,
                                    "mean": fmean(values), "median": median(values), "p90": percentile(values, .90),
                                    "p95": percentile(values, .95), "worst": min(values) if metric == "minimum_clearance_m" else max(values)})
        with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(output_rows)
        failure_rows = self.connection.execute(
            """SELECT method,config_hash,failure_type,COUNT(*) FROM results
               GROUP BY method,config_hash,failure_type ORDER BY method,config_hash,failure_type"""
        ).fetchall()
        with (output_dir / "failure_summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("method", "configuration_hash", "failure_type", "count"))
            writer.writerows(failure_rows)
        validation_fields = (
            "scenario_id", "method", "planner_success", "smoother_success",
            "collision_valid", "kinematic_valid", "endpoint_valid", "primary_result",
            "path_length_m", "cusps", "self_intersections", "max_abs_curvature",
            "min_implied_radius", "minimum_clearance", "collision_pose_index",
            "kinematic_violation_pose_index",
        )
        validation_rows = []
        detailed_rows = []
        for scenario_id, method, failure_type, payload in self.connection.execute(
            "SELECT scenario_id,method,failure_type,metrics_json FROM results ORDER BY scenario_id,method"
        ):
            metrics = json.loads(payload)
            compact = {
                "scenario_id": scenario_id,
                "method": method,
                "planner_success": metrics.get("planner_success", metrics.get("planning_success", False)),
                "smoother_success": metrics.get("smoother_success"),
                "collision_valid": metrics.get("collision_valid"),
                "kinematic_valid": metrics.get("kinematic_valid"),
                "endpoint_valid": metrics.get("endpoint_valid"),
                "primary_result": metrics.get("primary_result", failure_type),
                "path_length_m": metrics.get("path_length_m"),
                "cusps": metrics.get("cusps"),
                "self_intersections": metrics.get("self_intersections"),
                "max_abs_curvature": metrics.get("max_abs_curvature_1pm", metrics.get("max_abs_curvature")),
                "min_implied_radius": metrics.get("min_implied_radius_m"),
                "minimum_clearance": metrics.get("minimum_clearance_m"),
                "collision_pose_index": metrics.get("collision_pose_index"),
                "kinematic_violation_pose_index": metrics.get("kinematic_violation_pose_index"),
            }
            validation_rows.append(compact)
            detailed_rows.append({
                **compact,
                "secondary_diagnostic_flags": metrics.get("secondary_diagnostic_flags", []),
                "collision_evidence": metrics.get("collision_evidence"),
                "kinematic_evidence": {
                    key: metrics.get(key) for key in (
                        "segment_length_m", "heading_delta_rad", "signed_direction",
                        "nearest_cusp_distance_m", "curvature_violation_count",
                    )
                },
            })
        with (output_dir / "validation.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=validation_fields)
            writer.writeheader()
            writer.writerows(validation_rows)
        (output_dir / "validation.json").write_text(
            json.dumps(detailed_rows, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        lines = ["# LAKSA Planning Lab summary", "", "Hard constraints (collisions and kinematic violations) disqualify candidates.", "",
                 "| Method | Config | Stratum | Class | N | Success | Collisions | Kinematic violations |", "|---|---|---|---|---:|---:|---:|---:|"]
        seen = set()
        for row in output_rows:
            key = tuple(row[field] for field in fields[:8])
            if key in seen: continue
            seen.add(key)
            lines.append(f"| {row['method']} | `{row['configuration_hash'][:12]}` | {row['stratum']} | {row['class']} | {row['count']} | {row['success_rate']:.3f} | {row['collisions']} | {row['kinematic_violations']} |")
        (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        comparisons = []
        for (method, digest), items in sorted({(row[0], row[1]): [] for row in rows}.items()):
            selected = [(success, json.loads(payload)) for row_method, row_digest, _, _, success, payload in rows
                        if row_method == method and row_digest == digest]
            valid = [metrics for _, metrics in selected if metrics.get("planning_success", False)]
            collisions = sum(not metrics.get("collision_free", False) for metrics in valid)
            violations = sum(not metrics.get("kinematically_feasible", False) for metrics in valid)
            comparisons.append({"method": method, "configuration_hash": digest, "count": len(selected),
                                "hard_constraint_qualified": int(collisions == 0 and violations == 0),
                                "collisions": collisions, "kinematic_violations": violations,
                                "planner_failures": len(selected) - len(valid),
                                "feasible_success_rate": sum(success for success, _ in selected) / max(1, len(selected)),
                                "mean_cusps": fmean([m.get("cusps", 0) for m in valid]) if valid else float("nan"),
                                "mean_self_intersections": fmean([m.get("self_intersections", 0) for m in valid]) if valid else float("nan"),
                                "mean_path_length_m": fmean([m.get("path_length_m", 0) for m in valid]) if valid else float("nan"),
                                "mean_clearance_m": fmean([m.get("minimum_clearance_m", 0) for m in valid]) if valid else float("nan"),
                                "mean_pipeline_time_ms": fmean([m.get("total_pipeline_time_ms", 0) for m in valid]) if valid else float("nan")})
        comparison_fields = list(comparisons[0]) if comparisons else ["method", "configuration_hash"]
        with (output_dir / "configuration_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=comparison_fields); writer.writeheader(); writer.writerows(comparisons)
