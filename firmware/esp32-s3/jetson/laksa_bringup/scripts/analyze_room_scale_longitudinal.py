#!/usr/bin/env python3
"""Offline T10 analysis for ROOM_SCALE_LONGITUDINAL_V1 evidence.

It never changes raw JSONL, never publishes ROS traffic, and keeps unsupported
ground-speed, distance, force, braking-distance and slip claims UNKNOWN.
"""
from __future__ import annotations

import argparse, hashlib, json, math
from pathlib import Path
from statistics import mean, median
from typing import Any
import yaml


def sha256(path: Path) -> str:
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1<<20),b""): h.update(block)
    return h.hexdigest()


def finite(value: Any):
    try:
        value=float(value); return value if math.isfinite(value) else None
    except (TypeError,ValueError): return None


def read_records(trial_dir: Path) -> list[dict]:
    rows=[]
    for path in sorted((trial_dir/"raw_bag").glob("*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                row=json.loads(line);row["_source_file"]=str(path);rows.append(row)
    return sorted(rows,key=lambda x:x["received_monotonic"])


def values(records: list[dict], field: str) -> list[tuple[float,float]]:
    # Prefer the direct VESC topic to avoid counting the same telemetry twice
    # when VehicleState embeds a VESC snapshot.  VehicleState is fallback only.
    direct=[]; fallback=[]
    for row in records:
        value=finite(row.get("data",{}).get(field))
        if value is None: continue
        item=(float(row["received_monotonic"]),value)
        if row.get("topic")=="/laksa/vesc/state": direct.append(item)
        elif row.get("topic")=="/laksa/state": fallback.append(item)
    return sorted(direct or fallback)


def imu_x(records: list[dict]) -> list[tuple[float,float]]:
    out=[]
    for row in records:
        if row.get("topic")!="/laksa/imu/data":continue
        value=finite(row.get("data",{}).get("linear_acceleration_m_s2",{}).get("x"))
        if value is not None:out.append((float(row["received_monotonic"]),value))
    return sorted(out)


def command_output(records: list[dict]) -> list[tuple[float,float]]:
    return sorted((float(row["received_monotonic"]), value) for row in records if row.get("topic")=="/laksa/command" for value in [finite(row.get("data",{}).get("speed_mps"))] if value is not None)


def timing(samples: list[tuple[float,float]]) -> dict:
    ds=[b[0]-a[0] for a,b in zip(samples,samples[1:]) if b[0]>a[0]]
    if not ds:return {"count":0,"median_interval_s":None,"p95_interval_s":None,"max_gap_s":None,"sample_rate_hz":None,"duplicate_held_fraction":None}
    ds.sort();med=median(ds)
    held=sum(1 for (_,a),(_,b) in zip(samples,samples[1:]) if a==b)
    return {"count":len(ds),"median_interval_s":med,"p95_interval_s":ds[math.ceil(.95*len(ds))-1],"max_gap_s":max(ds),"sample_rate_hz":1/med if med else None,"duplicate_held_fraction":held/max(1,len(samples)-1)}


def transitions(requested, active_start, active_end, target):
    baseline=requested[0][1] if requested else 0.;band=max(25.,.1*abs(target));command=None;neutral=None
    for t,v in requested:
        if active_start is not None and t<active_start-.5:continue
        if abs(v-baseline)>=band and (not target or math.copysign(1,v or target)==math.copysign(1,target)):
            command=t;break
    if active_end is not None:
        neutral=next((t for t,v in requested if t>=active_end and abs(v)<=band),None)
    return {"metadata_active_start_monotonic":active_start,"metadata_active_end_monotonic":active_end,"command_transition_monotonic":command,"neutral_transition_monotonic":neutral,"baseline_requested_erpm":baseline}


def response(measured,target,command_t,neutral_t,active_end):
    if not measured or command_t is None or target==0:return {"status":"UNOBSERVABLE","reason":"measured eRPM, command transition and nonzero target required"}
    baseline=median([v for t,v in measured if t<command_t][-10:] or [0.]);sgn=1 if target>0 else -1;delta=abs(target-baseline)
    directed=[(t,sgn*(v-baseline)) for t,v in measured if t>=command_t]
    t10=next((t for t,v in directed if v>=.1*delta),None);t50=next((t for t,v in directed if v>=.5*delta),None);t90=next((t for t,v in directed if v>=.9*delta),None)
    steady=[v for t,v in directed if active_end is not None and command_t+.5<=t<=active_end]
    peak=max((v for _,v in directed),default=None);near_band=max(25.,.1*abs(target))
    neutral_decay=next((t-neutral_t for t,v in measured if neutral_t is not None and t>=neutral_t and abs(v)<=near_band),None)
    intervals=[b[0]-a[0] for a,b in zip(measured,measured[1:]) if b[0]>a[0]]; resolution=median(intervals) if intervals else None
    crossing=lambda t: {"classification":"BOUNDED_BY_SAMPLE_RATE" if resolution else "UNOBSERVABLE","observed_crossing_latency_sec":None if t is None else t-command_t,"crossing_interval_lower_sec":None if t is None or resolution is None else max(0.,t-command_t-resolution),"crossing_interval_upper_sec":None if t is None else t-command_t,"timing_resolution_sec":resolution}
    return {"status":"OBSERVABLE","baseline_measured_erpm":baseline,"observable_response_transition_monotonic":t10,
            "effective_response_latency_s":None if t10 is None else t10-command_t,"latency_classification":"BOUNDED_BY_SAMPLE_RATE" if resolution else "UNOBSERVABLE","crossings":{"10_percent":crossing(t10),"50_percent":crossing(t50),"90_percent":crossing(t90)},"rise_time_10_90_s":None if t10 is None or t90 is None else t90-t10,
            "peak_measured_erpm_signed":None if peak is None else sgn*peak,"steady_state_measured_erpm_signed":None if len(steady)<3 else sgn*mean(steady),
            "steady_state_error_erpm":None if len(steady)<3 else target-sgn*mean(steady),"overshoot_erpm":None if peak is None else peak-delta,
            "near_zero_band_erpm":near_band,"time_to_near_zero_s":neutral_decay,"warning":None if len(steady)>=3 else "NO_STEADY_INTERVAL"}


def huber(residuals,scale=25.):return sum(.5*r*r if abs(r)<=scale else scale*(abs(r)-.5*scale) for r in residuals)


def fit_level_2(measured,target,command_t,active_end):
    """A conservative effective delay + first-order fit; no calibration claim."""
    if command_t is None or active_end is None or active_end-command_t<.4:return {"fit_validity":"FIT_UNIDENTIFIABLE","reason":"insufficient active duration"}
    data=[(t-command_t,v) for t,v in measured if command_t<=t<=active_end]
    if len(data)<6:return {"fit_validity":"FIT_UNIDENTIFIABLE","reason":"fewer than six active samples"}
    baseline=median([v for t,v in measured if t<command_t][-10:] or [0.]);best=None
    for dms in range(0,301,10):
        delay=dms/1000
        for tms in range(30,801,10):
            tau=tms/1000;pred=[baseline+(target-baseline)*(0 if t<delay else 1-math.exp(-(t-delay)/tau)) for t,_ in data]
            residuals=[y-p for (_,y),p in zip(data,pred)];score=huber(residuals)
            if best is None or score<best[0]:best=(score,delay,tau,residuals)
    _,delay,tau,residuals=best;rmse=math.sqrt(sum(r*r for r in residuals)/len(residuals));mae=mean(abs(r) for r in residuals)
    return {"model_type":"LEVEL_2_EFFECTIVE_DELAY_FIRST_ORDER","fit_validity":"FIT_VALID" if len(data)>=12 and rmse<=max(50.,.15*abs(target)) else "FIT_WEAK",
            "parameters":{"effective_delay_s":delay,"time_constant_s":tau,"gain":1.0},"fit_rmse_erpm":rmse,"fit_mae_erpm":mae,"maximum_residual_erpm":max(map(abs,residuals)),"N":len(data),"data_interval_s":[0.,active_end-command_t],"optimizer_status":"DETERMINISTIC_GRID_HUBER","parameter_uncertainty":None,"identifiability_warning":"Effective delay, VESC ramp and motor response are correlated in a 1 s low-amplitude trial."}


def trial_report(trial_dir: Path) -> dict:
    records=read_records(trial_dir);meta_path=trial_dir/"metadata.yaml";meta=yaml.safe_load(meta_path.read_text()) if meta_path.exists() else {}
    target=float(meta.get("target_erpm",0.));requested=values(records,"requested_erpm");measured=values(records,"measured_erpm")
    events={x.get("event"):float(x.get("monotonic_ns"))/1e9 for x in meta.get("events",[]) if x.get("monotonic_ns")}
    active_start=meta.get("observed_active_start_monotonic",events.get("STATE_EXCITATION"));active_end=meta.get("observed_active_end_monotonic",events.get("STATE_NEUTRAL"))
    tr=transitions(requested,active_start,active_end,target)
    # Exact runner timestamp is causal truth for the request; requested eRPM is
    # separately observed at the VESC state boundary.
    if events.get("STATE_EXCITATION") is not None: tr["command_transition_monotonic"]=events["STATE_EXCITATION"]
    if events.get("STATE_NEUTRAL") is not None: tr["neutral_transition_monotonic"]=events["STATE_NEUTRAL"]
    resp=response(measured,target,tr["command_transition_monotonic"],tr["neutral_transition_monotonic"],active_end)
    motor=values(records,"motor_current_a");input_current=values(records,"input_current_a");voltage=values(records,"input_voltage_v");duty=values(records,"duty_cycle");accel=imu_x(records)
    canonical=command_output(records)
    initial=[v for t,v in accel if tr["command_transition_monotonic"] is not None and tr["command_transition_monotonic"]<=t<=tr["command_transition_monotonic"]+.5]
    freshness=[r.get("data",{}) for r in records if r.get("topic") in ("/laksa/state","/laksa/vesc/state")]
    direction=1 if target>=0 else -1
    directed_peak=lambda series: max((direction*v for _,v in series),default=None)
    output_first=next((t for t,v in canonical if tr["command_transition_monotonic"] is not None and t>=tr["command_transition_monotonic"] and abs(v)>1e-5),None)
    return {"schema_version":"laksa-t10-longitudinal-analysis-v3","test_id":"T10","trial":trial_dir.name,"direction":meta.get("direction"),"target_erpm":target,"metadata":meta,"raw_record_count":len(records),"raw_files":[str(p) for p in sorted((trial_dir/"raw_bag").glob("*.jsonl"))],"transitions":tr,"timing_layers":{"request_to_supervisor_output":{"classification":"BOUNDED_BY_SAMPLE_RATE" if canonical else "UNKNOWN","observed_latency_sec":None if output_first is None or tr["command_transition_monotonic"] is None else output_first-tr["command_transition_monotonic"],"output_observation_sample_quality":timing(canonical)}},"response":resp,"model_identification":{"level_0_observable_metrics":resp,"level_2_candidate_fit":fit_level_2(measured,target,tr["command_transition_monotonic"],active_end)},"electrical":{"peak_motor_current_signed_a":None if directed_peak(motor) is None else direction*directed_peak(motor),"peak_input_current_signed_a":None if directed_peak(input_current) is None else direction*directed_peak(input_current),"peak_duty_signed":None if directed_peak(duty) is None else direction*directed_peak(duty),"minimum_input_voltage_v":min((v for _,v in voltage),default=None),"voltage_sag_v":max((v for _,v in voltage),default=None)-min((v for _,v in voltage),default=None) if voltage else None},"imu":{"peak_abs_longitudinal_acceleration_m_s2":max((abs(v) for _,v in accel),default=None),"mean_first_half_second_m_s2":mean(initial) if initial else None,"acceleration_response_latency":"UNKNOWN without IMU axis/orientation validation"},"telemetry_quality":{"requested_erpm":timing(requested),"measured_erpm":timing(measured),"imu_acceleration":timing(accel),"telemetry_fresh_observed":all(bool(x.get("telemetry_fresh",False)) for x in freshness),"command_fresh_observed":all(bool(x.get("command_fresh",False)) for x in freshness)},"unknown_parameters":["absolute ground speed","travel distance","braking distance","tire longitudinal force","wheel slip"],"physical_truth_status":"PHYSICAL_DATA_COLLECTED" if records else "TOOLING_READY"}


def session_report(session_dir: Path) -> dict:
    trials=[trial_report(p) for p in sorted((session_dir/"trials").glob("trial_*"))]
    session_meta_path=session_dir/"session_metadata.yaml"; session_meta=yaml.safe_load(session_meta_path.read_text()) if session_meta_path.exists() else {}
    return {"schema_version":"laksa-t10-session-analysis-v2","test_id":"T10","session_dir":str(session_dir.resolve()),"physical_data_state":"PHYSICAL_DATA_COLLECTED" if trials else "TOOLING_READY","session_metadata":session_meta,"trials":trials,"analysis_provenance":{"analyzer_file":str(Path(__file__).resolve()),"analyzer_sha256":sha256(Path(__file__))},"configured_values":{"vesc_ramp_erpm_s":{"value":3000,"status":"CONFIGURED","source":"vehicle_identification_dry_run.py:VESC_BASELINE; runtime readback still required"}},"derived_estimates":{"forward_reverse_separate":True,"note":"No symmetry assumption."},"unknown_parameters":["ground_speed","distance","traction_force","slip"]}


def markdown_report(report: dict) -> str:
    lines=["# LAKSA T10 longitudinal analysis", "", f"Physical data state: `{report['physical_data_state']}`", ""]
    for trial in report["trials"]:
        response=trial["response"]; fit=trial["model_identification"]["level_2_candidate_fit"]
        lines += [f"## {trial['trial']}", "", f"- Target: `{trial['target_erpm']} eRPM`", f"- Response status: `{response.get('status')}`", f"- Effective latency: `{response.get('effective_response_latency_s')} s`", f"- 10–90 rise: `{response.get('rise_time_10_90_s')} s`", f"- Fit: `{fit.get('fit_validity')}` / `{fit.get('model_type')}`", f"- RMSE: `{fit.get('fit_rmse_erpm')} eRPM`", ""]
    lines += ["No ground speed, distance, braking-distance, force or slip is inferred by this report.", ""]
    return "\n".join(lines)


def main() -> int:
    p=argparse.ArgumentParser(description="Offline analysis for existing T10 JSONL sessions.");p.add_argument("session_dir",type=Path);p.add_argument("--output",type=Path);a=p.parse_args();report=session_report(a.session_dir);text=json.dumps(report,indent=2,sort_keys=True)
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text+"\n")
        a.output.with_suffix(".md").write_text(markdown_report(report))
    print(text);return 0
if __name__=="__main__":raise SystemExit(main())
