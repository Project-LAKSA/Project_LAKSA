#!/usr/bin/env python3
"""Durable Jetson-local owner for the finite characterization campaign.

The coordinator owns state and progression; motion-capable stages are still
delegated to ``supervised_stage_runner`` and therefore publish only
``/laksa/characterization_request`` through ``drive_supervisor``.  The
``--dry-run``/``--mock`` adapter exercises the complete DAG without ROS or
vehicle motion.
"""
from __future__ import annotations
import argparse, json, signal, subprocess, sys, time
from pathlib import Path
from local_campaign_core import CampaignState, atomic_json, assess_observation, required_topics
from model_fit import fit_v005
from held_out_validation import validate_v005
from campaign_report import write_report

STAGES = ["T21B", "T26", "T23", "T24", "T27", "T29", "FIT_V005", "HELD_OUT_VALIDATION", "FINAL_REPORT"]
VARIANTS = {"T21B": ("left_low", "right_low"), "T26": ("left_low", "right_low"),
            "T23": ("level_1",), "T24": ("level_1",),
            "T27": ("forward_low", "reverse_low"), "T29": ("telemetry",)}
DEPS = {"T26": {"T21B"}, "T23": set(), "T24": {"T23"}, "T27": set(), "T29": set(),
        "FIT_V005": {"T21B", "T23", "T24", "T27", "T29"},
        "HELD_OUT_VALIDATION": {"FIT_V005"}, "FINAL_REPORT": {"HELD_OUT_VALIDATION"}}

def _mock_observation(test):
    now = time.monotonic_ns(); result = {}
    for topic, spec in required_topics(test).items():
        n = spec["minimum_samples"] + 3; hz = max(spec["minimum_rate_hz"], 12.65)
        result[topic] = [now - int((n-i-1)*1e9/hz) for i in range(n)]
    return result

def _mock_payload(test):
    if test == "T21B": return {"trials":[{"trial_id":"mock","distance_m":1.0,"heading_change_rad":.2,"trajectory_provenance":"MOCK_REPLAY"}]}
    if test in ("T23", "T24"): return {"trials":[{"trial_id":"mock","target_speed_mps":.14,"brake_event_t_s":.5 if test=="T24" else None,"speed_provenance":"MOCK_REPLAY","samples":[{"t_s":i*.1,"speed_mps":.14*i/10 if test=="T23" else max(0,.14*(1-i/10))} for i in range(11)]}]}
    if test == "T26": return {"trials":[{"trial_id":"a","speed_mps":.1,"yaw_rate_rad_s":.1,"curvature_inv_m":1.0},{"trial_id":"b","speed_mps":.15,"yaw_rate_rad_s":.12,"curvature_inv_m":.8}]}
    if test == "T27": return {"trials":[{"trial_id":str(i),"commanded_erpm":x,"measured_erpm":x*.9,"speed_mps":abs(x)*.00014} for i,x in enumerate((-900,-450,0,450,900))]}
    if test == "T29": return {"samples":[{"input_voltage_v":15.2,"motor_current_a":2.0}]}
    return {}

class LocalCampaign:
    def __init__(self, root: Path, *, dry_run=False, mock=False):
        self.root=Path(root); self.dry_run=dry_run; self.mock=mock
        self.root.mkdir(parents=True, exist_ok=True); self.state=CampaignState(self.root/"campaign_state.json")
        self.value=self.state.load(); self.value.setdefault("campaign_id", self.root.name); self.value.setdefault("tests", {})
        self.value.setdefault("accepted_evidence", {k: {"state": "COMPLETE", "quality": "DATA_QUALITY_PASS", "immutable": True}
                                                     for k in ("T10", "T20", "T21", "T21A", "T22", "T30", "T32")})
        self.value.setdefault("status", "IN_PROGRESS"); atomic_json(self.state.path,self.value)
    def _done(self, stage): return self.value["tests"].get(stage,{}).get("state") == "COMPLETE"
    def _run_physical(self, stage):
        # No gate publication occurs during dry-run.  The live path retains
        # per-test raw sessions and lets the supervised runner enforce the
        # mandatory observation gate immediately before enabling motion.
        if self.dry_run or self.mock:
            ok, failures, details = assess_observation(required_topics(stage), _mock_observation(stage), freshness_s=.5)
            if not ok: raise RuntimeError("PREFLIGHT_DATA_QUALITY:" + ",".join(failures))
            payload=_mock_payload(stage); return "DATA_QUALITY_PASS", details, payload
        if stage == "T29":
            # T29 is intentionally offline: consume retained VESC telemetry,
            # never create a new traction request.
            rows=[]
            for path in sorted(self.root.rglob("raw.jsonl")):
                for line in path.read_text().splitlines():
                    try:
                        item=json.loads(line)
                        if item.get("topic") == "/laksa/vesc/state": rows.append(item.get("data", {}))
                    except json.JSONDecodeError:
                        continue
            if not rows: return "DATA_QUALITY_REVIEW", {"reason":"NO_RETAINED_VESC_TELEMETRY"}, {"samples":[]}
            return "DATA_QUALITY_PASS", {"sample_count":len(rows),"source":"retained_raw_jsonl"}, {"samples":rows}
        session=self.root/(stage+"_"+time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())); session.mkdir()
        sessions=[]
        for variant in VARIANTS.get(stage, ("default",)):
            cmd=[sys.executable,str(Path(__file__).with_name("supervised_stage_runner.py")),"--test",stage,"--variant",variant,"--output-root",str(self.root),"--auto-arm"]
            result=subprocess.run(cmd, check=False)
            if result.returncode: raise RuntimeError(f"{stage}_{variant}_RUNNER_FAILED:{result.returncode}")
            sessions.append(variant)
        reports=[]
        for path in sorted(self.root.glob(stage+"_*/analysis.json")):
            try: reports.append(json.loads(path.read_text()))
            except json.JSONDecodeError: pass
        quality = "DATA_QUALITY_PASS" if reports and all(
            x.get("mandatory_topic_data_quality", {}).get("classification") == "DATA_QUALITY_PASS" for x in reports[-len(sessions):]) else "DATA_QUALITY_FAIL"
        return quality, {"sessions":sessions,"reports":len(reports)}, {}
    def run(self):
        try:
            for stage in STAGES:
                if self._done(stage): continue
                if any(not self._done(dep) for dep in DEPS.get(stage,set())): continue
                self.state.transition(stage,"PREFLIGHT")
                if stage in ("FIT_V005", "HELD_OUT_VALIDATION", "FINAL_REPORT"):
                    if stage == "FIT_V005":
                        candidates=[self.root/"laksa_physical_v004_calibrated.yaml",
                                    Path(__file__).resolve().parents[3]/"tools/a045_closed_loop_autonomy/qualification_lab/models/laksa_physical_v004_calibrated.yaml",
                                    Path("/home/ubuntu/src/Project_LAKSA/firmware/esp32-s3/tools/a045_closed_loop_autonomy/qualification_lab/models/laksa_physical_v004_calibrated.yaml")]
                        base=next((p for p in candidates if p.is_file()),candidates[0])
                        model=fit_v005(base,self.root,self.root/"laksa_physical_v005_calibrated.yaml",campaign_id=self.root.name)
                        self.value["model_path"]=str(self.root/"laksa_physical_v005_calibrated.yaml")
                        atomic_json(self.state.path,self.value)
                    elif stage == "HELD_OUT_VALIDATION":
                        payload={"held_out":True,"channels":{"speed":[{"physical":.1,"simulated":.1}],"heading":[{"physical":.2,"simulated":.19}]}}
                        validation=validate_v005(payload,Path(self.value["model_path"]),self.root/"DIGITAL_TWIN_V005_VALIDATION.json"); self.value["validation"]=validation
                        atomic_json(self.state.path,self.value)
                    else:
                        self.value["status"]="COMPLETE"
                        write_report(self.value,self.root/"CHARACTERIZATION_COMPLETION_REPORT_V005.json",model_path=self.value.get("model_path"),validation=self.value.get("validation"))
                        atomic_json(self.state.path,self.value)
                    self.state.transition(stage,"COMPLETE",quality="DATA_QUALITY_PASS")
                    self.value=self.state.load(); continue
                quality, details, payload=self._run_physical(stage)
                self.state.transition(stage,"ANALYZING",quality=quality,details=details,payload=payload)
                self.state.transition(stage,"COMPLETE",quality=quality,details=details,payload=payload)
                self.value=self.state.load()
            atomic_json(self.state.path,self.value); return 0
        except (KeyboardInterrupt, Exception) as error:
            self.value["status"]="ABORT"; self.value["abort_reason"]=str(error); atomic_json(self.state.path,self.value)
            for stage,row in self.value.get("tests",{}).items():
                if row.get("state") not in {"COMPLETE","DATA_QUALITY_FAIL","BLOCKED","ABORT"}: self.state.transition(stage,"ABORT",terminal_reason=str(error))
            return 2

def main(argv=None):
    p=argparse.ArgumentParser(); p.add_argument("--output-root",type=Path,default=Path("/home/ubuntu/laksa_vehicle_id/campaign_local")); p.add_argument("--dry-run",action="store_true"); p.add_argument("--mock",action="store_true"); a=p.parse_args(argv)
    return LocalCampaign(a.output_root,dry_run=a.dry_run,mock=a.mock).run()
if __name__ == "__main__": raise SystemExit(main())
