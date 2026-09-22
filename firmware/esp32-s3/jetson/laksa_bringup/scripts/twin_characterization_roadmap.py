#!/usr/bin/env python3
"""Pure-Python dependency graph and offline analyzers for LAKSA twin evidence.

This module deliberately imports no ROS package and creates no publisher.  It
turns immutable recorded evidence into explicit readiness/blocked decisions;
physical runners remain separately supervised by drive_supervisor.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import statistics
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'characterization_suite.yaml'
ORDER = ('T21_IMU_SANITY','T21A','T22','T28','T25','T23','T24','T27','T29','T30_SENSOR_MODEL','T31','T21B','T26','T32','FINAL_TWIN_VALIDATION')

# This is an executable-proof index, not protocol metadata.  A stage may only
# be reported IMPLEMENTED when its runner/analyzer pair actually exists.
IMPLEMENTATION = {
    'T21_IMU_SANITY': {'runner':'t21_imu_sanity_runner.py','analyzer':'t21_imu_sanity_runner.py','recorder':'subscriber-only Imu recorder'},
    'T21A': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace + terminal outcome artifact'},
    'T21B': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'subscriber-only ROS trace + accepted trajectory payload'},
    'T22': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace + measured distance payload'},
    'T23': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace'},
    'T24': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace'},
    'T25': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised steering/physical-angle payload'},
    'T26': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace + trajectory payload'},
    'T27': {'runner':'supervised_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'supervised ROS trace + level payload'},
    'T28': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'guided value/unit/method/uncertainty/provenance payload'},
    'T29': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'existing/new immutable VESC telemetry'},
    'T30_SENSOR_MODEL': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'immutable sensor-channel payload'},
    'T31': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'immutable repeated-sample payload'},
    'T32': {'runner':'qualification_lab/lab.py','analyzer':'qualification_lab/manual_twin.py','recorder':'digital-only Xbox trace'},
    'FINAL_TWIN_VALIDATION': {'runner':'characterization_stage_runner.py','analyzer':'characterization_stage_tools.py','recorder':'held-out physical/twin aligned payload'},
}

def read(path):
    return yaml.safe_load(Path(path).read_text()) or {}

def _t10(campaign, direction):
    try:return json.loads((Path(campaign)/'reports'/f't10_{direction}.json').read_text()).get('status') == 'DATA_QUALITY_PASS'
    except Exception:return False

def _t20(campaign):
    for path in Path(campaign).glob('T20_SUPERVISED_*/T20_static_steering.yaml'):
        try:
            if read(path).get('status') == 'COMPLETE': return True
        except Exception:pass
    return False

def _analysis_pass(campaign, prefix):
    for path in Path(campaign).glob(f'{prefix}_*/analysis.json'):
        try:
            report=json.loads(path.read_text())
            review=path.parent/'provenance_review.json'
            qualification=json.loads(review.read_text()).get('classification') if review.is_file() else None
            if report.get('status') == 'PASS' and qualification not in ('SUSPECT_REPEAT_REQUIRED','REJECTED'): return True
        except Exception:pass
    return False

def _immutable_report_pass(campaign, name, *, classification=None):
    """Accept only an explicit immutable report, never a session-name guess."""
    try:
        report=json.loads((Path(campaign)/name).read_text())
        return report.get('status') == 'PASS' and (classification is None or report.get('classification') == classification)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return False

def _t21_offline_adjudication_pass(campaign):
    """Accept only a hash-bound, immutable-evidence T21 adjudication."""
    root=Path(campaign); path=root/'T21_IMU_SANITY_OFFLINE_ADJUDICATION.json'
    try:
        report=json.loads(path.read_text())
        if report.get('status') != 'PASS' or report.get('classification') != 'ACCEPTED_FOR_T21_SANITY_ONLY': return False
        session=root/report['source_session']; sources=report['source_artifacts']
        raw=session/'raw.jsonl'; analysis=session/'analysis.json'
        return (raw.is_file() and analysis.is_file() and
                hashlib.sha256(raw.read_bytes()).hexdigest()==sources['raw_jsonl_sha256'] and
                hashlib.sha256(analysis.read_bytes()).hexdigest()==sources['analysis_json_sha256'])
    except Exception:
        return False

def evidence(campaign):
    root=Path(campaign)
    return {
        't10_forward_pass':_t10(root,'forward'), 't10_reverse_pass':_t10(root,'reverse'),
        't20_complete':_t20(root), 't21_imu_sanity_pass':_analysis_pass(root,'T21_IMU_SANITY') or _t21_offline_adjudication_pass(root),
        # These two adjudications are explicit, source-listed physical
        # evidence contracts.  The older per-session terminal artifacts are
        # deliberately not treated as parameter-identification PASS records.
        't21a_pass':_immutable_report_pass(root, 'T21A_RESOLUTION_ANALYSIS.json', classification='SIGNED_OPPOSED_YAW_RESPONSE_REPRODUCED') or _analysis_pass(root,'T21A_DYNAMIC_YAW'),
        't22_physical_speed_pass':_immutable_report_pass(root, 'T22_20260920T214059Z/authoritative_calibration.json') or _analysis_pass(root,'T22_PHYSICAL_SPEED'),
        'accepted_trajectory_source':(root/'accepted_trajectory_source.yaml').is_file() and read(root/'accepted_trajectory_source.yaml').get('status') == 'ACCEPTED',
        't21b_pass':_analysis_pass(root,'T21B_CURVATURE'),
        't25_repeatability_or_equivalent':_analysis_pass(root,'T25_STEERING_HYSTERESIS'),
    }

def status(campaign, suite=CONFIG):
    spec=read(suite); have=evidence(campaign); rows=[]
    for test_id in ORDER:
        test=spec['tests'][test_id]; missing=[item for item in test.get('depends_on',[]) if not have.get(item,False)]
        ready=not missing
        proof=IMPLEMENTATION.get(test_id)
        # Metadata alone never earns an IMPLEMENTED state.
        if not proof:
            state='SPECIFIED_NOT_IMPLEMENTED'
        elif test_id=='T21_IMU_SANITY' and have['t21_imu_sanity_pass']:
            state='COMPLETE_PHYSICAL_EVIDENCE'
        elif test_id=='T21A' and have['t21a_pass']:
            state='COMPLETE_PHYSICAL_EVIDENCE'
        elif test_id=='T22' and have['t22_physical_speed_pass']:
            state='COMPLETE_PHYSICAL_EVIDENCE'
        elif test_id in ('T21_IMU_SANITY','T28','T32') and ready:
            state='IMPLEMENTED_READY'
        elif proof:
            state='IMPLEMENTED_BLOCKED_BY_PHYSICAL_DEPENDENCY'
        else:
            state='SPECIFIED_NOT_IMPLEMENTED'
        rows.append({'test_id':test_id,'implementation_state':state,'missing_evidence':missing,
                     'physical_action':test['physical_action'],'automation':test['automation'],'identifies':test['identifies'],
                     'implementation_proof':proof})
    return {'schema_version':'laksa-twin-roadmap-status-v1','campaign':str(Path(campaign)),'evidence':have,'tests':rows}

def next_test(campaign):
    report=status(campaign)
    imu=next(row for row in report['tests'] if row['test_id']=='T21_IMU_SANITY')
    if imu['implementation_state']=='IMPLEMENTED_READY': return imu
    # A dependency-satisfied human physical session is the next scientific
    # action, even though its observer-only recorder never commands motion.
    for row in report['tests']:
        if row['implementation_state']=='IMPLEMENTED_BLOCKED_BY_PHYSICAL_DEPENDENCY' and not row['missing_evidence']:
            return row
    for row in report['tests']:
        if row['missing_evidence']==['requires_operator_physical_session']: return row
    for row in report['tests']:
        if row['implementation_state']=='IMPLEMENTED_READY': return row
    return imu

def analyze_speed_trials(payload):
    """Fit only measured distance/timestamp trials; never infer ground speed."""
    trials=payload.get('trials') or []; usable=[]; rejected=[]
    for trial in trials:
        try:
            distance=float(trial['measured_distance_m']); start=float(trial['start_monotonic_s']); end=float(trial['end_monotonic_s']); erpm=float(trial['measured_erpm'])
            if distance<=0 or end<=start or abs(erpm)<1e-9: raise ValueError('invalid_distance_time_or_erpm')
            usable.append({'measured_erpm':erpm,'speed_mps':distance/(end-start),'duration_s':end-start,'distance_m':distance,'timing_uncertainty_s':trial.get('timing_uncertainty_s')})
        except (KeyError,TypeError,ValueError) as error: rejected.append({'trial':trial,'reason':str(error)})
    ratios=[row['speed_mps']/row['measured_erpm'] for row in usable]
    uncertainty=statistics.pstdev(ratios) if len(ratios)>1 else None
    return {'test_id':'T22','status':'PASS' if usable else 'NEEDS_REVIEW','physical_truth':'MEASURED' if usable else 'UNKNOWN',
            'usable_trials':usable,'rejected_trials':rejected,'erpm_to_mps_gain':statistics.median(ratios) if ratios else None,
            'gain_spread':uncertainty,'uncertainty_status':'REPEATABILITY_NOT_ESTIMABLE' if len(ratios)<2 else 'SAMPLE_SPREAD_ONLY',
            'limitations':['No loaded radius is inferred without independently measured gear reduction and wheel kinematics.','No braking, slip, or force is inferred.']}

def analyze_repeatability(values):
    values=[float(v) for v in values]
    return {'n':len(values),'mean':statistics.mean(values) if values else None,'std':statistics.stdev(values) if len(values)>1 else None,
            'range':(max(values)-min(values)) if values else None,'uncertainty_status':'UNKNOWN_SINGLE_SAMPLE' if len(values)<2 else 'SAMPLE_SPREAD'}

def analyze_yaw_trials(payload):
    """Summarize synchronized T21A records without claiming wheel dynamics."""
    trials=[]; rejected=[]
    for trial in payload.get('trials',[]):
        try:
            command=float(trial['steering_target_rad']); yaw=[float(v) for v in trial['imu_yaw_rate_rad_s']]
            if not yaw or command==0: raise ValueError('missing_yaw_excitation_or_zero_steering')
            steady=statistics.mean(yaw[-max(1,len(yaw)//3):]); peak=max(yaw,key=abs)
            trials.append({'steering_target_rad':command,'peak_yaw_rate_rad_s':peak,'steady_yaw_rate_rad_s':steady,
                           'yaw_gain_per_software_rad':steady/command,'steering_current_rad':trial.get('steering_current_rad'),
                           'physical_road_wheel_angle_deg':trial.get('physical_road_wheel_angle_deg'),'notes':'software/proxy steering unless independently measured wheel angle supplied'})
        except (KeyError,TypeError,ValueError) as error: rejected.append({'trial':trial,'reason':str(error)})
    signs={math.copysign(1,row['steady_yaw_rate_rad_s']) for row in trials if row['steady_yaw_rate_rad_s']}
    return {'test_id':'T21A','status':'PASS' if trials else 'NEEDS_REVIEW','trials':trials,'rejected_trials':rejected,
            'left_right_response_observed':len(signs)>1,'limitations':['No curvature or turn radius without accepted physical speed and trajectory.','No physical steering-angle dynamics without a wheel-angle observer.']}

def analyze_electrical_samples(samples):
    """T29 opportunistic telemetry summary; no new load test is implied."""
    voltage=[float(row['input_voltage_v']) for row in samples if row.get('input_voltage_v') is not None]
    motor=[float(row['motor_current_a']) for row in samples if row.get('motor_current_a') is not None]
    faults=[row.get('fault_code') for row in samples if row.get('fault_code') not in (None,0)]
    return {'test_id':'T29','status':'PASS' if voltage else 'NEEDS_REVIEW','sample_count':len(samples),
            'minimum_input_voltage_v':min(voltage) if voltage else None,'observed_voltage_span_v':max(voltage)-min(voltage) if voltage else None,
            'peak_abs_motor_current_a':max((abs(v) for v in motor),default=None),'faults':faults,
            'limitations':['Voltage span is not battery sag unless compared within a controlled maneuver and battery state.']}

def geometry_template():
    names=('wheelbase_axle_center_m','front_wheel_center_track_m','rear_wheel_center_track_m','loaded_wheel_radius_m','total_mass_kg','front_axle_mass_kg','rear_axle_mass_kg','cg_height_m')
    return {'schema_version':'laksa-t28-geometry-session-v1','test_id':'T28','physical_truth':'PENDING_OPERATOR_MEASUREMENT','measurements':[{'parameter':name,'value':None,'units':'m' if name.endswith('_m') else 'kg','method':None,'uncertainty':None,'timestamp_utc':None,'provenance':'UNKNOWN'} for name in names]}

def protocols():
    return {'T21A':{'arm':'each left/right low-speed trial separately','sequence':['neutral settle','ARM','bounded single-direction trial','neutral settle','gate disable'],'record':['characterization request','supervisor command','steering target/current proxy','requested/active/measured eRPM','IMU yaw rate','ROS and monotonic timestamps']},
            'T22':{'operator_setup':['enter independently measured course length m','place start/finish markers','position at start'], 'record':['measured distance','automatic monotonic start/end event when available','measured eRPM','timing uncertainty'], 'rule':'No manual stopwatch is treated as precise ground truth.'},
            'T23_T24_T27':{'rule':'Blocked until T22 physical speed is accepted; every motion level is independently armed and separated by neutral settle.'},
            'T25':{'rule':'Zero traction only; preserve approach direction and repeats; do not call software steering a road-wheel measurement.'},
            'T28':{'rule':'Every entered dimension/mass requires method, units, uncertainty, timestamp and provenance.'}}

def held_out_plan():
    return {'schema_version':'laksa-held-out-twin-validation-v1','status':'BLOCKED_PENDING_CALIBRATION_EVIDENCE','rule':'Maneuvers used for fit are excluded from validation.',
            'required_channels':['speed_mps','measured_erpm','yaw_rate_rad_s','heading_change_rad','curvature_inv_m','timing'],
            'metrics':['rmse','mae','max_abs_residual','timing_offset_s'],'no_automatic_model_promotion':True}

def main(argv=None):
    parser=argparse.ArgumentParser(description='LAKSA twin characterization roadmap; no ROS motion path.')
    parser.add_argument('--campaign',type=Path,required=True);parser.add_argument('--status',action='store_true');parser.add_argument('--dry-run',action='store_true');parser.add_argument('--speed-input',type=Path);parser.add_argument('--output',type=Path);parser.add_argument('--geometry-template',action='store_true');parser.add_argument('--held-out-plan',action='store_true');args=parser.parse_args(argv)
    if args.speed_input: result=analyze_speed_trials(json.loads(args.speed_input.read_text()))
    elif args.geometry_template: result=geometry_template()
    elif args.held_out_plan: result=held_out_plan()
    else: result=status(args.campaign);result['next_test']=next_test(args.campaign);result['dry_run']=bool(args.dry_run);result['protocols']=protocols()
    if args.output: args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,indent=2,sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
