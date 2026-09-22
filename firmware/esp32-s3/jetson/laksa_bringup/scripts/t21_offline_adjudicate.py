#!/usr/bin/env python3
"""Offline, immutable-evidence adjudication for the T21 IMU sanity test.

This tool has no ROS imports, no publishers, and never commands a vehicle.
It verifies hashes before deriving an adjudication from an existing v2 session.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

from t21_imu_sanity_runner import MAX_SAMPLE_GAP_SECONDS, RETURN_DWELL_SECONDS, analyze_stage, threshold_for

MIN_PEAK_TO_THRESHOLD = 10.0
MAX_CROSS_AXIS_RATIO = 0.20
MAX_ATTITUDE_EXCURSION_DEG = 5.0
MAX_ACCELERATION_DEVIATION_M_S2 = 0.50


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def _same_float(left, right, tolerance=1e-9):
    return left is not None and right is not None and abs(float(left)-float(right)) <= tolerance


def reconstruct_stage(rows, baseline, recorded):
    """Recover the original stage window from its immutable analysis signature."""
    count=int(recorded['sample_count'])
    onset=float(recorded['onset_recv_monotonic'])
    end=float(recorded['motion_end_recv_monotonic'])
    matches=[]
    for start in range(0, len(rows)-count+1):
        candidate=rows[start:start+count]
        result=analyze_stage(candidate,baseline,timeout=recorded.get('timed_out',False))
        if (_same_float(result.get('onset_recv_monotonic'),onset) and
                _same_float(result.get('motion_end_recv_monotonic'),end) and
                _same_float(result.get('return_to_baseline_residual_rad_s'),recorded.get('return_to_baseline_residual_rad_s')) and
                _same_float(result.get('integrated_relative_yaw_delta_rad'),recorded.get('integrated_relative_yaw_delta_rad'))):
            matches.append((start,candidate,result))
    if len(matches) != 1:
        raise ValueError(f'could not uniquely reconstruct immutable stage window ({len(matches)} matches)')
    return matches[0]


def _roll_pitch_degrees(row):
    q=row['orientation_quaternion']; x,y,z,w=(float(q[key]) for key in ('x','y','z','w'))
    roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    pitch=math.degrees(math.asin(max(-1.0,min(1.0,2*(w*y-z*x)))))
    return roll,pitch


def _range(values):
    return {'min':min(values),'max':max(values),'span':max(values)-min(values)}


def _first_raw_index(rows, timestamp):
    return next(index for index,row in enumerate(rows) if _same_float(row['recv_monotonic'],timestamp))


def _post_motion_dwell(rows, stage_end_index, motion_end, threshold, median):
    """Use only continuous already-recorded samples immediately after motion.

    The continuation may extend beyond the capture window, but must stop at the
    first newly-active sample.  This is evidence of a stationary IMU, not a
    fabricated extension of the armed operator action.
    """
    first=None; last=None; samples=[]
    for row in rows[stage_end_index+1:]:
        if abs(float(row['yaw_rad_s'])-median) >= threshold:
            break
        if first is None:
            first=row
        samples.append(row); last=row
    # Include the quiet samples held in the original capture as well.
    motion_end_index=_first_raw_index(rows,motion_end)
    in_stage_quiet=[]
    for row in rows[motion_end_index+1:stage_end_index+1]:
        if abs(float(row['yaw_rad_s'])-median) >= threshold:
            raise ValueError('reconstructed stage has non-quiet data after its declared motion end')
        in_stage_quiet.append(row)
    combined=in_stage_quiet+samples
    if not combined:
        return {'return_to_baseline':False,'reason':'no_post_motion_quiet_samples'}
    max_gap=max((float(b['recv_monotonic'])-float(a['recv_monotonic']) for a,b in zip(combined,combined[1:])),default=0.0)
    duration=float(combined[-1]['recv_monotonic'])-float(combined[0]['recv_monotonic'])
    return {'return_to_baseline':duration >= RETURN_DWELL_SECONDS and max_gap <= MAX_SAMPLE_GAP_SECONDS,
            'quiet_start_recv_monotonic':combined[0]['recv_monotonic'],
            'quiet_end_recv_monotonic':combined[-1]['recv_monotonic'],
            'quiet_duration_s':duration,'quiet_sample_count':len(combined),'max_receive_gap_s':max_gap,
            'source':'immutable raw.jsonl continuation after reconstructed capture window'}


def _vector_screen(stage_rows, active_rows, baseline_rows):
    required=('angular_velocity_rad_s','linear_acceleration_m_s2','orientation_quaternion')
    if not all(all(field in row for field in required) for row in stage_rows):
        return {'available':False,'status':'REJECT','reason':'vector_channels_missing'}
    yaw_peak=max(abs(float(row['angular_velocity_rad_s']['z'])) for row in active_rows)
    cross_peak=max(abs(float(row['angular_velocity_rad_s'][axis])) for row in active_rows for axis in ('x','y'))
    acceleration_baseline={axis:statistics.mean(float(row['linear_acceleration_m_s2'][axis]) for row in baseline_rows) for axis in ('x','y','z')}
    acceleration_deviation=max(abs(float(row['linear_acceleration_m_s2'][axis])-acceleration_baseline[axis]) for row in active_rows for axis in ('x','y','z'))
    attitudes=[_roll_pitch_degrees(row) for row in active_rows]
    attitude_excursion=max(max(value)-min(value) for value in zip(*attitudes))
    cross_ratio=cross_peak/yaw_peak if yaw_peak else float('inf')
    passed=(cross_ratio <= MAX_CROSS_AXIS_RATIO and attitude_excursion <= MAX_ATTITUDE_EXCURSION_DEG and
            acceleration_deviation <= MAX_ACCELERATION_DEVIATION_M_S2)
    return {'available':True,'status':'PASS' if passed else 'REJECT','cross_axis_peak_rad_s':cross_peak,
            'yaw_axis_peak_rad_s':yaw_peak,'cross_axis_ratio':cross_ratio,
            'attitude_roll_pitch_excursion_deg':attitude_excursion,
            'max_linear_acceleration_deviation_m_s2':acceleration_deviation,
            'criteria':{'max_cross_axis_ratio':MAX_CROSS_AXIS_RATIO,'max_attitude_excursion_deg':MAX_ATTITUDE_EXCURSION_DEG,
                        'max_linear_acceleration_deviation_m_s2':MAX_ACCELERATION_DEVIATION_M_S2}}


def adjudicate(session):
    session=Path(session); analysis_path=session/'analysis.json'; raw_path=session/'raw.jsonl'
    original=json.loads(analysis_path.read_text()); rows=_load_jsonl(raw_path)
    if original.get('schema_version') != 'laksa-t21-imu-sanity-v2':
        raise ValueError('offline vector adjudication requires a v2 session')
    baseline_count=int(original['baseline']['n']); baseline_rows=rows[:baseline_count]
    if len(baseline_rows) != baseline_count:
        raise ValueError('raw evidence is shorter than declared baseline')
    median=float(original['baseline'].get('median') or 0.0); threshold=threshold_for(original['baseline'])
    sides={}; accepted=True
    for side in ('left','right'):
        start,stage_rows,replayed=reconstruct_stage(rows,original['baseline'],original[side])
        end=start+len(stage_rows)-1; active=[row for row in stage_rows if abs(float(row['yaw_rad_s'])-median)>=threshold]
        post=_post_motion_dwell(rows,end,original[side]['motion_end_recv_monotonic'],threshold,median)
        vector=_vector_screen(stage_rows,active,baseline_rows)
        peak_ratio=abs(float(original[side]['peak_yaw_rad_s'])-median)/threshold
        side_ok=(not original[side]['timed_out'] and not original[side]['dropped_or_stale_samples'] and
                 bool(active) and peak_ratio >= MIN_PEAK_TO_THRESHOLD and post['return_to_baseline'] and vector['status']=='PASS')
        sides[side]={'source_window_rows':[start,end],'replayed_runner_status':replayed['status'],
                     'detected_motion':bool(active),'dominant_sign':original[side]['dominant_sign'],
                     'peak_yaw_rad_s':original[side]['peak_yaw_rad_s'],
                     'integrated_relative_yaw_delta_rad':original[side]['integrated_relative_yaw_delta_rad'],
                     'signal_to_noise_ratio':original[side]['signal_to_noise_ratio'],
                     'peak_to_detection_threshold':peak_ratio,'timed_out':original[side]['timed_out'],
                     'post_motion_stationarity':post,'vector_quality':vector,'accepted':side_ok}
        accepted=accepted and side_ok
    opposite=sides['left']['dominant_sign'] != sides['right']['dominant_sign']
    accepted=accepted and original['baseline'].get('stable') and opposite
    return {'schema_version':'laksa-t21-offline-adjudication-v1','test_id':'T21_IMU_SANITY','status':'PASS' if accepted else 'NEEDS_REVIEW',
            'classification':'ACCEPTED_FOR_T21_SANITY_ONLY' if accepted else 'REJECTED',
            'adjudication_method':'offline immutable raw evidence; no ROS, no actuator publisher, no physical motion',
            'source_session':session.name,'source_artifacts':{'raw_jsonl_sha256':sha256(raw_path),'analysis_json_sha256':sha256(analysis_path)},
            'original_runner_status':original.get('status'),'original_failure_explanation':'capture completion used a backward-looking quiet window while analysis required a forward dwell; offline evaluation uses continuous recorded post-stage samples',
            'baseline':original['baseline'],'threshold_rad_s':threshold,'opposite_left_right_signs':opposite,
            'sides':sides,'scope_limitations':['Establishes manual yaw observability, sign opposition, stationary return, and frame/sign sanity only.',
                                 'Does not identify steering dynamics, yaw gain, curvature, lateral slip, or full vehicle dynamics.'],
            'safety':{'motion_command_published':False,'publishers_created':0,'characterization_gate_enabled':False,
                      'non_neutral_characterization_requests':0}}


def main(argv=None):
    parser=argparse.ArgumentParser(description='Derive a T21 adjudication from immutable evidence only')
    parser.add_argument('--session',type=Path,required=True); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv); report=adjudicate(args.session)
    # Derived evidence is atomically replaced; raw.jsonl and analysis.json are
    # deliberately never opened for writing.
    temporary=args.output.with_suffix(args.output.suffix+'.tmp')
    temporary.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    temporary.replace(args.output)
    print(json.dumps({'status':report['status'],'output':str(args.output),'motion_command_published':False,'publishers_created':0},indent=2))
    return 0 if report['status']=='PASS' else 2


if __name__=='__main__':
    raise SystemExit(main())
