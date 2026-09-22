#!/usr/bin/env python3
"""Offline analyzers for post-T20 LAKSA characterization evidence.

The functions in this file are deliberately ROS-free.  They accept immutable
JSON/YAML evidence captured by ``characterization_stage_runner.py`` and return
explicitly qualified estimates; they never manufacture missing physical data.
"""
from __future__ import annotations

import math
import statistics
from collections import defaultdict


def _mean(values):
    return statistics.mean(values) if values else None


def _std(values):
    return statistics.stdev(values) if len(values) > 1 else None


def _series(rows, key):
    return [(float(r['t_s']), float(r[key])) for r in rows if r.get('t_s') is not None and r.get(key) is not None]


def _rate_and_gaps(rows):
    times = sorted(float(row['t_s']) for row in rows if row.get('t_s') is not None)
    gaps = [b-a for a, b in zip(times, times[1:]) if b >= a]
    return {
        'sample_count': len(times),
        'median_period_s': statistics.median(gaps) if gaps else None,
        'sample_rate_hz': 1.0 / statistics.median(gaps) if gaps and statistics.median(gaps) > 0 else None,
        'max_gap_s': max(gaps) if gaps else None,
    }


def analyze_t21a(payload):
    trials, rejected = [], []
    for row in payload.get('trials', []):
        yaw = [float(v) for v in row.get('imu_yaw_rate_rad_s', [])]
        command = row.get('steering_target_rad')
        if command is None or not yaw or float(command) == 0:
            rejected.append({'trial_id': row.get('trial_id'), 'reason': 'MISSING_YAW_OR_NONZERO_STEERING'})
            continue
        tail = yaw[-max(1, len(yaw)//3):]
        steady = _mean(tail)
        samples = sorted(row.get('samples', []), key=lambda x: float(x.get('t_s', 0)))
        signal = [(float(x['t_s']), float(x['yaw_rate_rad_s'])) for x in samples if x.get('t_s') is not None and x.get('yaw_rate_rad_s') is not None]
        onset, rise = None, None
        if signal:
            base=statistics.median([x[1] for x in signal[:max(1,len(signal)//5)]])
            peak=max((abs(x[1]-base) for x in signal),default=0.0)
            if peak>0:
                transition=row.get('steering_transition_t_s')
                t10=next((t for t,v in signal if abs(v-base)>=.1*peak),None)
                t90=next((t for t,v in signal if abs(v-base)>=.9*peak),None)
                onset=t10-float(transition) if t10 is not None and transition is not None else None
                rise=t90-t10 if t10 is not None and t90 is not None else None
        trials.append({
            'trial_id': row.get('trial_id'), 'steering_target_rad': float(command),
            'steering_current_rad': row.get('steering_current_rad'),
            'peak_yaw_rate_rad_s': max(yaw, key=abs), 'steady_yaw_rate_rad_s': steady,
            'yaw_gain_per_software_rad': steady / float(command), 'steering_proxy_to_yaw_latency_s':onset,
            'yaw_rise_time_s':rise, 'quality': _rate_and_gaps(samples),
        })
    signs = {math.copysign(1.0, x['steady_yaw_rate_rad_s']) for x in trials if x['steady_yaw_rate_rad_s']}
    return {'test_id': 'T21A', 'status': 'PASS' if len(trials) >= 2 and len(signs) >= 2 else 'NEEDS_REVIEW',
            'trials': trials, 'rejected_trials': rejected, 'left_right_response_observed': len(signs) >= 2,
            'model_fields': ['steering_proxy_to_yaw_latency_s', 'yaw_rise_time_s', 'yaw_gain_per_software_rad', 'left_right_yaw_asymmetry'],
            'limitations': ['Yaw gain is measured against software steering until T21B supplies a physical steering-to-curvature mapping.', 'No absolute heading is inferred.']}


def analyze_t21b(payload):
    """Analyze a measured low-speed turn; never derive radius without distance."""
    rows, rejected = [], []
    for row in payload.get('trials', []):
        try:
            distance, heading = float(row['distance_m']), float(row['heading_change_rad'])
            if distance <= 0 or abs(heading) <= 1e-9: raise ValueError('INVALID_DISTANCE_OR_HEADING_CHANGE')
            curvature = heading/distance
            rows.append({'trial_id':row.get('trial_id'),'distance_m':distance,'heading_change_rad':heading,
                         'curvature_inv_m':curvature,'turn_radius_m':abs(1.0/curvature),
                         'steering_target_rad':row.get('steering_target_rad'),'trajectory_provenance':row.get('trajectory_provenance','UNKNOWN')})
        except (KeyError,TypeError,ValueError) as exc: rejected.append({'trial_id':row.get('trial_id'),'reason':str(exc)})
    return {'test_id':'T21B','status':'PASS' if rows else 'NEEDS_REVIEW','trials':rows,'rejected_trials':rejected,
            'model_fields':['curvature_inv_m','turn_radius_m','steering_to_curvature'],
            'limitations':['Distance and heading must originate from an accepted physical trajectory source; IMU-only yaw is insufficient.']}


def analyze_t22(payload):
    usable, rejected = [], []
    for row in payload.get('trials', []):
        try:
            distance, start, end, erpm = (float(row[k]) for k in ('measured_distance_m', 'start_monotonic_s', 'end_monotonic_s', 'measured_erpm'))
            if distance <= 0 or end <= start or abs(erpm) < 1e-9:
                raise ValueError('INVALID_DISTANCE_TIME_OR_ERPM')
            usable.append({'trial_id': row.get('trial_id'), 'measured_distance_m': distance, 'duration_s': end-start,
                           'speed_mps': distance/(end-start), 'measured_erpm': erpm,
                           'distance_uncertainty_m': row.get('distance_uncertainty_m'),
                           'timing_uncertainty_s': row.get('timing_uncertainty_s')})
        except (KeyError, TypeError, ValueError) as exc:
            rejected.append({'trial_id': row.get('trial_id'), 'reason': str(exc)})
    gains = [x['speed_mps']/x['measured_erpm'] for x in usable]
    return {'test_id': 'T22', 'status': 'PASS' if usable else 'NEEDS_REVIEW', 'physical_truth': 'MEASURED' if usable else 'UNKNOWN',
            'usable_trials': usable, 'rejected_trials': rejected, 'erpm_to_mps_gain': statistics.median(gains) if gains else None,
            'gain_spread': _std(gains), 'uncertainty_status': 'REPEATABILITY_NOT_ESTIMABLE' if len(gains) < 2 else 'SAMPLE_SPREAD_ONLY',
            'model_fields': ['erpm_to_speed_gain_m_per_s_per_erpm', 'speed_repeatability'],
            'limitations': ['No rolling radius, gearing, force, slip, or braking distance is inferred from a distance/time trial alone.']}


def _response_trial(row, *, brake=False):
    samples = sorted(row.get('samples', []), key=lambda x: float(x.get('t_s', 0)))
    speed = _series(samples, 'speed_mps')
    if len(speed) < 3:
        return None, 'INSUFFICIENT_SPEED_SAMPLES'
    baseline = speed[0][1]
    peak = max((x[1] for x in speed), default=baseline)
    if brake:
        onset = row.get('brake_event_t_s')
        if onset is None: return None, 'MISSING_BRAKE_EVENT'
        after = [(t, v) for t, v in speed if t >= float(onset)]
        if len(after) < 2: return None, 'INSUFFICIENT_POST_BRAKE_SAMPLES'
        decels = [-(b[1]-a[1])/(b[0]-a[0]) for a,b in zip(after, after[1:]) if b[0] > a[0]]
        threshold=after[0][1]*.95
        response_t=next((t for t,v in after if v <= threshold),None)
        return {'brake_event_t_s': float(onset), 'initial_speed_mps': after[0][1], 'brake_delay_s':response_t-float(onset) if response_t is not None else None,
                'peak_braking_deceleration_mps2': max(decels) if decels else None,
                'mean_braking_deceleration_mps2': _mean(decels), 'speed_provenance': row.get('speed_provenance', 'UNKNOWN')}, None
    target = row.get('target_speed_mps', peak)
    if target is None or float(target) <= baseline: return None, 'NO_OBSERVABLE_ACCELERATION'
    ten, ninety = baseline + .1*(float(target)-baseline), baseline + .9*(float(target)-baseline)
    crossing = lambda threshold: next((t for t,v in speed if v >= threshold), None)
    t10, t90 = crossing(ten), crossing(ninety)
    return {'baseline_speed_mps': baseline, 'target_speed_mps': float(target), 't10_s': t10, 't90_s': t90,
            'rise_time_10_90_s': t90-t10 if t10 is not None and t90 is not None else None,
            'peak_speed_mps': peak, 'speed_provenance': row.get('speed_provenance', 'UNKNOWN')}, None


def analyze_t23(payload):
    rows, rejected = [], []
    for row in payload.get('trials', []):
        result, reason = _response_trial(row)
        (rows if result else rejected).append({'trial_id': row.get('trial_id'), **(result or {'reason': reason})})
    return {'test_id': 'T23', 'status': 'PASS' if rows else 'NEEDS_REVIEW', 'trials': rows, 'rejected_trials': rejected,
            'model_fields': ['acceleration_rise_time_s', 'neutral_coastdown_proxy'],
            'limitations': ['Speed provenance remains ESTIMATED unless a T22 measured-distance calibration is linked.']}


def analyze_t24(payload):
    rows, rejected = [], []
    for row in payload.get('trials', []):
        result, reason = _response_trial(row, brake=True)
        (rows if result else rejected).append({'trial_id': row.get('trial_id'), **(result or {'reason': reason})})
    return {'test_id': 'T24', 'status': 'PASS' if rows else 'NEEDS_REVIEW', 'trials': rows, 'rejected_trials': rejected,
            'model_fields': ['brake_delay_s', 'braking_deceleration_mps2'],
            'limitations': ['No stopping distance is claimed without independent position evidence.']}


def analyze_t25(payload):
    groups = defaultdict(lambda: defaultdict(list))
    for row in payload.get('samples', []):
        if row.get('software_steering_rad') is None or row.get('left_road_wheel_deg') is None or row.get('right_road_wheel_deg') is None:
            continue
        groups[round(float(row['software_steering_rad']), 6)][str(row.get('approach_direction', 'UNKNOWN'))].append(row)
    points, hysteresis = [], []
    for command, by_approach in sorted(groups.items()):
        summary = {}
        for approach, rows in by_approach.items():
            summary[approach] = {'n': len(rows), 'left_mean_deg': _mean([float(x['left_road_wheel_deg']) for x in rows]),
                                'right_mean_deg': _mean([float(x['right_road_wheel_deg']) for x in rows]),
                                'left_std_deg': _std([float(x['left_road_wheel_deg']) for x in rows]),
                                'right_std_deg': _std([float(x['right_road_wheel_deg']) for x in rows])}
        points.append({'software_steering_rad': command, 'approaches': summary})
        if len(summary) > 1:
            vals = list(summary.values()); hysteresis.append({'software_steering_rad': command,
                'left_span_deg': max(x['left_mean_deg'] for x in vals)-min(x['left_mean_deg'] for x in vals),
                'right_span_deg': max(x['right_mean_deg'] for x in vals)-min(x['right_mean_deg'] for x in vals)})
    return {'test_id': 'T25', 'status': 'PASS' if points else 'NEEDS_REVIEW', 'points': points,
            'hysteresis': hysteresis, 'hysteresis_evidence_preserved': bool(hysteresis),
            'model_fields': ['steering_hysteresis_deg', 'steering_repeatability_deg'],
            'limitations': ['Approach branches are preserved; they are not averaged into a false symmetric steering mapping.']}


def analyze_t26(payload):
    usable = []
    for row in payload.get('trials', []):
        speed, yaw, curvature = row.get('speed_mps'), row.get('yaw_rate_rad_s'), row.get('curvature_inv_m')
        if None in (speed, yaw, curvature) or float(speed) <= 0: continue
        usable.append({'trial_id': row.get('trial_id'), 'speed_mps': float(speed), 'yaw_rate_rad_s': float(yaw),
                       'curvature_inv_m': float(curvature), 'yaw_gain_per_mps': float(yaw)/float(speed)})
    trend = None
    if len(usable) >= 2:
        xs, ys = [x['speed_mps'] for x in usable], [x['yaw_gain_per_mps'] for x in usable]
        xbar, ybar = _mean(xs), _mean(ys); denom=sum((x-xbar)**2 for x in xs)
        trend = sum((x-xbar)*(y-ybar) for x,y in zip(xs,ys))/denom if denom else None
    return {'test_id': 'T26', 'status': 'PASS' if len(usable) >= 2 else 'NEEDS_REVIEW', 'trials': usable,
            'speed_dependent_yaw_gain_slope': trend, 'model_fields': ['speed_dependent_yaw_gain', 'curvature_trend', 'lateral_slip_trend'],
            'limitations': ['This reports a trend, not tire-slip force, unless independent lateral velocity is available.']}


def analyze_t27(payload):
    points = []
    for row in payload.get('trials', []):
        if row.get('commanded_erpm') is None or row.get('measured_erpm') is None or row.get('speed_mps') is None: continue
        points.append({'trial_id': row.get('trial_id'), 'commanded_erpm': float(row['commanded_erpm']),
                       'measured_erpm': float(row['measured_erpm']), 'speed_mps': float(row['speed_mps']),
                       'direction': 'FORWARD' if float(row['commanded_erpm']) >= 0 else 'REVERSE'})
    forward = sorted((x for x in points if x['direction']=='FORWARD'), key=lambda x:x['commanded_erpm'])
    reverse = sorted((x for x in points if x['direction']=='REVERSE'), key=lambda x:x['commanded_erpm'])
    monotonic = all(b['speed_mps'] >= a['speed_mps'] for a,b in zip(forward,forward[1:]))
    deadband = max((abs(x['commanded_erpm']) for x in points if abs(x['speed_mps']) <= .02), default=None)
    by_abs=defaultdict(dict)
    for row in points: by_abs[abs(row['commanded_erpm'])][row['direction']]=row['speed_mps']
    asymmetry=[{'abs_commanded_erpm':k,'forward_speed_mps':v['FORWARD'],'reverse_speed_mps':v['REVERSE'],'speed_sum_mps':v['FORWARD']+v['REVERSE']} for k,v in by_abs.items() if 'FORWARD' in v and 'REVERSE' in v]
    return {'test_id': 'T27', 'status': 'PASS' if len(points) >= 3 else 'NEEDS_REVIEW', 'points': points,
            'forward_monotonic': monotonic, 'observable_deadband_erpm': deadband,
            'saturation_observed': len(points) >= 2 and abs(points[-1]['speed_mps']-points[-2]['speed_mps']) < .02,
            'forward_reverse_asymmetry':asymmetry,
            'model_fields': ['speed_deadband_erpm', 'erpm_speed_monotonicity', 'speed_saturation', 'forward_reverse_asymmetry'],
            'limitations': ['Saturation is only reported when the sampled range visibly plateaus; absence is not proof of no saturation.']}


def analyze_t28(payload):
    required = ('value', 'units', 'method', 'uncertainty', 'provenance')
    valid, rejected = [], []
    for row in payload.get('measurements', []):
        missing = [key for key in required if row.get(key) in (None, '')]
        try: float(row.get('value'))
        except (TypeError, ValueError): missing.append('numeric_value')
        if missing: rejected.append({'parameter': row.get('parameter'), 'reason': 'MISSING_' + '_'.join(missing)})
        else: valid.append(row)
    return {'test_id': 'T28', 'status': 'PASS' if valid and not rejected else 'NEEDS_REVIEW', 'measurements': valid, 'rejected_measurements': rejected,
            'model_fields': [x.get('parameter') for x in valid], 'schema_requires': list(required)}


def analyze_t29(payload):
    samples = payload.get('samples', [])
    voltage = [float(x['input_voltage_v']) for x in samples if x.get('input_voltage_v') is not None]
    current = [float(x['motor_current_a']) for x in samples if x.get('motor_current_a') is not None]
    return {'test_id': 'T29', 'status': 'PASS' if voltage else 'NEEDS_REVIEW', 'sample_count': len(samples),
            'minimum_input_voltage_v': min(voltage) if voltage else None, 'voltage_span_v': max(voltage)-min(voltage) if voltage else None,
            'peak_abs_motor_current_a': max(map(abs,current)) if current else None, 'model_fields': ['input_voltage_sag_proxy', 'motor_current_envelope'],
            'limitations': ['Span is not controlled-load battery sag without synchronized maneuver and battery-state evidence.']}


def analyze_t30_sensor_model(payload):
    channels = {}
    for name, rows in (payload.get('channels') or {}).items():
        quality = _rate_and_gaps(rows)
        numeric = [float(x['value']) for x in rows if x.get('value') is not None]
        channels[name] = {**quality, 'mean': _mean(numeric), 'std': _std(numeric), 'available': bool(rows)}
    return {'test_id': 'T30_SENSOR_MODEL', 'status': 'PASS' if channels and any(x['available'] for x in channels.values()) else 'NEEDS_REVIEW',
            'channels': channels, 'model_fields': ['imu_noise', 'sensor_sample_rate_hz', 'sensor_timestamp_gap_s', 'zed_availability', 'lidar_availability'],
            'limitations': ['A telemetry noise summary is not sensor extrinsic calibration.']}


def analyze_t31(payload):
    groups = defaultdict(list)
    for row in payload.get('samples', []):
        if row.get('condition') is not None and row.get('value') is not None: groups[str(row['condition'])].append(float(row['value']))
    result = {k:{'n':len(v), 'mean':_mean(v), 'std':_std(v), 'range':max(v)-min(v) if v else None,
                 'uncertainty_status':'UNKNOWN_SINGLE_SAMPLE' if len(v)<2 else 'SAMPLE_SPREAD'} for k,v in groups.items()}
    return {'test_id': 'T31', 'status':'PASS' if any(x['n']>=2 for x in result.values()) else 'NEEDS_REVIEW', 'groups':result,
            'model_fields':['repeatability_spread','trial_variance'], 'raw_samples_preserved':True}


def analyze_final_twin_validation(payload):
    if not payload.get('held_out', False):
        return {'test_id':'FINAL_TWIN_VALIDATION','status':'NEEDS_REVIEW','reason':'HELD_OUT_EVIDENCE_REQUIRED','model_promoted':False}
    metrics = {}
    for channel, rows in (payload.get('channels') or {}).items():
        residuals = [float(x['physical'])-float(x['simulated']) for x in rows if x.get('physical') is not None and x.get('simulated') is not None]
        if residuals: metrics[channel]={'n':len(residuals),'rmse':math.sqrt(_mean([x*x for x in residuals])), 'mae':_mean([abs(x) for x in residuals]), 'max_abs_residual':max(map(abs,residuals))}
    return {'test_id':'FINAL_TWIN_VALIDATION','status':'PASS' if metrics else 'NEEDS_REVIEW','held_out':True,'metrics':metrics,
            'model_fields':['held_out_speed_residual','held_out_yaw_residual','held_out_heading_residual','held_out_curvature_residual','held_out_timing_residual'],
            'model_promoted':False, 'limitations':['PASS is evidence review readiness, never automatic model promotion.']}


ANALYZERS = {'T21A':analyze_t21a,'T21B':analyze_t21b,'T22':analyze_t22,'T23':analyze_t23,'T24':analyze_t24,'T25':analyze_t25,'T26':analyze_t26,'T27':analyze_t27,'T28':analyze_t28,'T29':analyze_t29,'T30_SENSOR_MODEL':analyze_t30_sensor_model,'T31':analyze_t31,'FINAL_TWIN_VALIDATION':analyze_final_twin_validation}


def analyze(test_id, payload):
    if test_id not in ANALYZERS: raise ValueError(f'No analyzer for {test_id}')
    return ANALYZERS[test_id](payload)
