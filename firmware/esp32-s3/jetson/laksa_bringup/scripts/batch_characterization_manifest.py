#!/usr/bin/env python3
"""Hash-bound finite authorization manifests for supervised batch trials.

Pure Python: this module has no ROS or actuator authority.
"""
from __future__ import annotations

import hashlib
import json

from supervised_stage_protocols import build

GATE_ACK_MAX_S = 3.0
POST_NEUTRAL_MAX_S = 8.0
LEASE_MAX_AGE_S = 2.0


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'))


def digest(manifest):
    body={key:value for key,value in manifest.items() if key != 'manifest_sha256'}
    return hashlib.sha256(canonical(body).encode()).hexdigest()


def trial(test_id, variant, config):
    phases=build(test_id,variant,config)
    powered=[phase for phase in phases if phase.speed_mps or phase.steering_rad or phase.brake]
    speed_time=sum(phase.duration_s for phase in phases if phase.speed_mps)
    return {
        'stage':test_id, 'variant':variant,
        'phases':[phase.__dict__ for phase in phases],
        'powered_duration_s':speed_time,
        'nominal_path_m':sum(abs(phase.speed_mps)*phase.duration_s for phase in phases),
        'max_wall_clock_s':GATE_ACK_MAX_S+sum(phase.duration_s for phase in phases)+POST_NEUTRAL_MAX_S,
        'max_speed_mps':max(abs(phase.speed_mps) for phase in phases),
        'max_steering_rad':max(abs(phase.steering_rad) for phase in phases),
        'required_sensors':['/joy','/laksa/state','/laksa/vesc/state','/laksa/imu/data'],
        'preconditions':['xbox_fresh','manual_neutral','estop_false','autonomy_false','gate_false','vehicle_stationary','vesc_fresh','vesc_fault_clear'],
        'abort_conditions':['SESSION_LEASE_EXPIRED','STALE_TOPIC','XBOX_MANUAL_OVERRIDE','ESTOP','AUTONOMY_CONFLICT','VESC_FAULT_OR_STALE','GATE_ACK_TIMEOUT','STATIONARY_TIMEOUT'],
    }


def first_batch(campaign, config):
    """The finite current batch ends before the first missing physical truth."""
    rows=[trial('T21A','left',config),trial('T21A','right',config)]
    manifest={'schema_version':'laksa-characterization-batch-v1','campaign':str(campaign),
              'trials':rows,'stop_before':'T22','stop_reason':'MEASURED_COURSE_DISTANCE_REQUIRED',
              'lease_max_age_s':LEASE_MAX_AGE_S,
              'maximum_envelope':{'speed_mps':max(row['max_speed_mps'] for row in rows),
                                  'steering_rad':max(row['max_steering_rad'] for row in rows),
                                  'individual_wall_clock_s':max(row['max_wall_clock_s'] for row in rows)},
              'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256']=digest(manifest)
    return manifest


def right_revalidation(campaign, config):
    """A distinct one-trial manifest; never reuses a consumed pair manifest."""
    row = trial('T21A', 'right', config)
    manifest = {'schema_version':'laksa-characterization-batch-v1','campaign':str(campaign),
                'trials':[row], 'purpose':'T21A_RIGHT_REVALIDATION_AFTER_NON_OPPOSED_RESPONSE',
                'stop_before':'NO_FURTHER_TRIALS', 'stop_reason':'SINGLE_DIRECTLY_AUTHORIZED_REVALIDATION_ONLY',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':row['max_speed_mps'],'steering_rad':row['max_steering_rad'],
                                    'individual_wall_clock_s':row['max_wall_clock_s']},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest

def t22_rehearsal(campaign, config):
    row=trial('T22','forward',config); manifest={'schema_version':'laksa-characterization-batch-v1','campaign':str(campaign),'trials':[row],'purpose':'T22_REHEARSAL_ONLY_NOT_VALID_FOR_CALIBRATION','stop_before':'NO_FURTHER_TRIALS','stop_reason':'REHEARSAL_ONLY','lease_max_age_s':LEASE_MAX_AGE_S,'maximum_envelope':{'speed_mps':row['max_speed_mps'],'steering_rad':0.0,'individual_wall_clock_s':row['max_wall_clock_s']},'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'};manifest['manifest_sha256']=digest(manifest);return manifest


def t22_authoritative(campaign, config):
    """One fresh measured-distance T22 calibration, distinct from rehearsal evidence."""
    row = trial('T22', 'forward', config)
    manifest = {'schema_version':'laksa-characterization-batch-v1', 'campaign':str(campaign),
                'trials':[row], 'purpose':'T22_AUTHORITATIVE_SPEED_CALIBRATION',
                'stop_before':'NO_FURTHER_TRIALS', 'stop_reason':'ONE_MEASURED_DISTANCE_CALIBRATION',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':row['max_speed_mps'],'steering_rad':0.0,
                                    'individual_wall_clock_s':row['max_wall_clock_s']},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest


def unlocked_next(campaign, config):
    """Hash-bound batch for stages unlocked by authoritative T22 evidence."""
    rows = [trial('T25', 'positive', config), trial('T25', 'negative', config),
            trial('T23', 'level_1', config), trial('T24', 'level_1', config),
            trial('T27', 'forward_low', config)]
    manifest = {'schema_version':'laksa-characterization-batch-v1', 'campaign':str(campaign),
                'trials':rows, 'purpose':'UNLOCKED_POST_T22_CHARACTERIZATION_BATCH',
                'stop_before':'T21B_AND_T26', 'stop_reason':'T21A_AND_ACCEPTED_TRAJECTORY_REQUIRED',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':max(r['max_speed_mps'] for r in rows),
                                    'steering_rad':max(r['max_steering_rad'] for r in rows),
                                    'individual_wall_clock_s':max(r['max_wall_clock_s'] for r in rows)},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest


def followup_next(campaign, config):
    """Follow-up repeats for T25 and the missing reverse T27 level."""
    rows = [trial('T25', 'positive', config), trial('T25', 'negative', config),
            trial('T27', 'reverse_low', config)]
    manifest = {'schema_version':'laksa-characterization-batch-v1', 'campaign':str(campaign),
                'trials':rows, 'purpose':'FOLLOWUP_POST_T22_REPEATABILITY_AND_REVERSE_BATCH',
                'stop_before':'T21B_AND_T26', 'stop_reason':'T21A_AND_ACCEPTED_TRAJECTORY_REQUIRED',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':max(r['max_speed_mps'] for r in rows),
                                    'steering_rad':max(r['max_steering_rad'] for r in rows),
                                    'individual_wall_clock_s':max(r['max_wall_clock_s'] for r in rows)},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest


def t21a_resolution(campaign, config):
    """Targeted T21A resolution using the immutable T20 steering commands."""
    rows = [trial('T21A', 'left', config), trial('T21A', 'right', config)]
    manifest = {'schema_version':'laksa-characterization-batch-v1', 'campaign':str(campaign),
                'trials':rows, 'purpose':'T21A_RESOLUTION_AFTER_T22_AND_T20_REANALYSIS',
                'stop_before':'T21B', 'stop_reason':'SIGNED_YAW_RESPONSE_AND_TRAJECTORY_REVIEW',
                'steering_mapping_source':'T20 immutable measured wheel angles',
                'speed_scale_source':'T22_20260920T214059Z authoritative measured calibration',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':max(r['max_speed_mps'] for r in rows),
                                    'steering_rad':max(r['max_steering_rad'] for r in rows),
                                    'individual_wall_clock_s':max(r['max_wall_clock_s'] for r in rows)},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest


def t21b_trajectory_capture(campaign, config):
    """Frozen two-direction trajectory capture for the remaining T21B input."""
    rows = [trial('T21B', 'left_low', config), trial('T21B', 'right_low', config)]
    manifest = {'schema_version':'laksa-characterization-batch-v1', 'campaign':str(campaign),
                'trials':rows, 'purpose':'T21B_ACCEPTED_TRAJECTORY_CAPTURE',
                'execution_revision':'T21B_TRAJECTORY_CAPTURE_MANIFEST_V3_ODOMETRY_ORIENTATION_CAPTURE',
                'stop_before':'T26', 'stop_reason':'ANALYZE_AND_ACCEPT_TRAJECTORY_BEFORE_MULTI_SPEED_TURNS',
                'trajectory_source':'/laksa/odometry/fused plus synchronized IMU/VESC trace',
                'speed_scale_source':'T22_20260920T214059Z authoritative measured calibration',
                'steering_mapping_source':'T20 immutable measured wheel angles',
                'lease_max_age_s':LEASE_MAX_AGE_S,
                'maximum_envelope':{'speed_mps':max(r['max_speed_mps'] for r in rows),
                                    'steering_rad':max(r['max_steering_rad'] for r in rows),
                                    'individual_wall_clock_s':max(r['max_wall_clock_s'] for r in rows)},
                'authorization_phrase_format':'LAKSA_BATCH_AUTHORIZE:<manifest_sha256>'}
    manifest['manifest_sha256'] = digest(manifest)
    return manifest


def validate(manifest, phrase):
    if manifest.get('schema_version') != 'laksa-characterization-batch-v1':
        raise ValueError('unsupported manifest schema')
    if digest(manifest) != manifest.get('manifest_sha256'):
        raise ValueError('manifest hash mismatch')
    expected='LAKSA_BATCH_AUTHORIZE:'+manifest['manifest_sha256']
    if phrase != expected:
        raise ValueError('authorization is not bound to this manifest')
    return True
