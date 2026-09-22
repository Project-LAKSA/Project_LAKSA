#!/usr/bin/env python3
"""Subscriber-only recorder and offline analyzer for future characterization stages.

No publisher, service client, action client, direct VESC or PCA9685 access is
present.  The human retains Xbox/manual authority for any future physical
maneuver.  ``--dry-run`` is the required offline proof path.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
import time
from pathlib import Path

from characterization_stage_tools import ANALYZERS, analyze

RECORDED_STAGES = ('T21A','T21B','T22','T23','T24','T25','T26','T27','T29')
OFFLINE_STAGES = ('T28','T30_SENSOR_MODEL','T31','FINAL_TWIN_VALIDATION')
ALL = RECORDED_STAGES + OFFLINE_STAGES


def stamp(): return dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def git_identity():
    root=next((p for p in Path(__file__).resolve().parents if (p/'.git').exists()),None)
    if root is None:return {'commit':'UNKNOWN'}
    return {'commit':subprocess.run(['git','-C',str(root),'rev-parse','HEAD'],text=True,capture_output=True).stdout.strip() or 'UNKNOWN'}


def template(test_id):
    if test_id == 'T21A': return {'trials':[{'trial_id':'left_or_right_01','steering_target_rad':None,'steering_current_rad':None,'steering_transition_t_s':None,'imu_yaw_rate_rad_s':[],'samples':[{'t_s':None,'yaw_rate_rad_s':None}],'provenance':'RAW_SESSION'}]}
    if test_id == 'T21B': return {'trials':[{'trial_id':'turn_01','distance_m':None,'heading_change_rad':None,'steering_target_rad':None,'trajectory_provenance':'ACCEPTED_PHYSICAL_TRAJECTORY_REQUIRED'}]}
    if test_id == 'T22': return {'trials':[{'trial_id':'straight_01','measured_distance_m':None,'distance_uncertainty_m':None,'start_monotonic_s':None,'end_monotonic_s':None,'timing_uncertainty_s':None,'measured_erpm':None,'measurement_method':None,'provenance':'MEASURED_START_FINISH'}]}
    if test_id in ('T23','T24'): return {'trials':[{'trial_id':'response_01','target_speed_mps':None,'brake_event_t_s':None,'speed_provenance':'T22_LINK_REQUIRED','samples':[{'t_s':None,'speed_mps':None}]}]}
    if test_id == 'T25': return {'samples':[{'software_steering_rad':None,'left_road_wheel_deg':None,'right_road_wheel_deg':None,'approach_direction':None,'measurement_method':None,'measurement_uncertainty_deg':None,'provenance':'MEASURED'}]}
    if test_id == 'T26': return {'trials':[{'trial_id':'turn_speed_01','speed_mps':None,'yaw_rate_rad_s':None,'curvature_inv_m':None,'trajectory_provenance':'ACCEPTED_PHYSICAL_TRAJECTORY_REQUIRED'}]}
    if test_id == 'T27': return {'trials':[{'trial_id':'level_01','commanded_erpm':None,'measured_erpm':None,'speed_mps':None,'speed_provenance':'T22_LINK_REQUIRED'}]}
    if test_id == 'T29': return {'samples':[{'t_s':None,'input_voltage_v':None,'motor_current_a':None,'provenance':'RAW_VESC_TELEMETRY'}]}
    if test_id == 'T28':
        return {'measurements':[{'parameter':'wheelbase_axle_center_m','value':None,'units':'m','method':None,'uncertainty':None,'provenance':'UNKNOWN'}]}
    if test_id == 'FINAL_TWIN_VALIDATION': return {'held_out':True,'channels':{'speed_mps':[],'yaw_rate_rad_s':[],'heading_rad':[],'curvature_inv_m':[],'timing_s':[]}}
    if test_id == 'T30_SENSOR_MODEL': return {'channels':{'imu_yaw_rate_rad_s':[],'lidar_scan_rate_hz':[],'zed_odom_rate_hz':[]}}
    return {'trials':[]}


def write(path, value): path.write_text(json.dumps(value, indent=2, sort_keys=True)+'\n')


def collect_t28(prompt=input):
    """Guided non-motion geometry/mass entry; blank values remain UNKNOWN."""
    definitions=(('wheelbase_axle_center_m','m'),('front_wheel_center_track_m','m'),('rear_wheel_center_track_m','m'),('loaded_wheel_radius_m','m'),('total_mass_kg','kg'),('front_axle_mass_kg','kg'),('rear_axle_mass_kg','kg'),('cg_height_m','m'))
    rows=[]
    print('T28 GEOMETRY/MASS — no vehicle motion. Each value needs units, method, uncertainty, and provenance. Blank value records UNKNOWN.',flush=True)
    for parameter, units in definitions:
        value=prompt(f'{parameter} [{units}] value (blank=UNKNOWN): ').strip()
        if not value:
            rows.append({'parameter':parameter,'value':None,'units':units,'method':None,'uncertainty':None,'provenance':'UNKNOWN'});continue
        method=prompt(f'{parameter} method: ').strip()
        uncertainty=prompt(f'{parameter} uncertainty [{units}]: ').strip()
        provenance=prompt(f'{parameter} provenance [MEASURED/CONFIGURED/ESTIMATED]: ').strip().upper() or 'UNKNOWN'
        rows.append({'parameter':parameter,'value':value,'units':units,'method':method or None,'uncertainty':uncertainty or None,'provenance':provenance})
    return {'measurements':rows}


def record(test_id, root):
    """Capture a deliberately raw ROS trace.  No actuator authority is created."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from laksa_interfaces.msg import VehicleState, VescState, DriveCommand
    class Watch(Node):
        def __init__(self):
            super().__init__('laksa_'+test_id.lower()+'_observer'); self.rows=[]
            self.create_subscription(Imu,'/laksa/imu/data',lambda m:self.add('/laksa/imu/data',{'yaw_rate_rad_s':float(m.angular_velocity.z)}),qos_profile_sensor_data)
            self.create_subscription(VehicleState,'/laksa/state',lambda m:self.add('/laksa/state',{'steering_target_rad':float(m.steering_target_rad),'steering_current_rad':float(m.steering_current_rad),'measured_erpm':float(m.vesc.measured_erpm)}),qos_profile_sensor_data)
            self.create_subscription(VescState,'/laksa/vesc/state',lambda m:self.add('/laksa/vesc/state',{'requested_erpm':float(m.requested_erpm),'active_erpm':float(m.active_erpm),'measured_erpm':float(m.measured_erpm),'input_voltage_v':float(m.input_voltage_v),'motor_current_a':float(m.motor_current_a)}),qos_profile_sensor_data)
            self.create_subscription(DriveCommand,'/laksa/command',lambda m:self.add('/laksa/command',{'speed_mps':float(m.speed_mps),'steering_angle_rad':float(m.steering_angle_rad),'brake':bool(m.brake)}),10)
            self.create_subscription(Odometry,'/laksa/odometry/fused',lambda m:self.add('/laksa/odometry/fused',{'x_m':float(m.pose.pose.position.x),'y_m':float(m.pose.pose.position.y),'linear_speed_mps':float(m.twist.twist.linear.x),'frame_id':m.header.frame_id}),qos_profile_sensor_data)
        def add(self, topic, data): self.rows.append({'topic':topic,'t_s':time.monotonic(),'data':data})
    rclpy.init(); node=Watch(); started=time.monotonic(); root.mkdir(parents=True)
    print(f'{test_id}: OBSERVER ONLY. Human/Xbox owns all motion. Press ENTER to begin recording; Ctrl-C safely preserves evidence.',flush=True)
    interrupted=False
    try:
        input(); begin=time.monotonic(); print('RECORDING. This ENTER is the START event. Perform only the approved human maneuver; press ENTER at the FINISH event.',flush=True)
        while True:
            rclpy.spin_once(node,timeout_sec=.05)
            # input is intentionally blocking only after a short polling window.
            if time.monotonic()-begin >= .25:
                import select
                if select.select([sys.stdin],[],[],0)[0]: input(); break
    except KeyboardInterrupt:
        interrupted=True
    finally:
        # Raw capture is evidence even on Ctrl-C; analysis stays separate.
        write(root/'raw.json',{'test_id':test_id,'rows':node.rows,'operator_events':{'start_monotonic_s':begin if 'begin' in locals() else None,'finish_monotonic_s':time.monotonic()},'started_monotonic_s':started,'finished_monotonic_s':time.monotonic(),'interrupted':interrupted})
        node.destroy_node(); rclpy.shutdown()


def main(argv=None):
    p=argparse.ArgumentParser(description='LAKSA observer-only characterization stage runner')
    p.add_argument('--test',choices=ALL,required=True);p.add_argument('--dry-run',action='store_true');p.add_argument('--output-root',type=Path,default=Path('/home/ubuntu/laksa_vehicle_id'))
    p.add_argument('--input',type=Path,help='immutable operator payload to analyze');p.add_argument('--analyze-session',type=Path,help='write analysis beside an existing immutable recorded session');p.add_argument('--template',action='store_true');a=p.parse_args(argv)
    if a.template: print(json.dumps(template(a.test),indent=2,sort_keys=True));return 0
    if a.dry_run:
        print(json.dumps({'status':'DRY_RUN_OK','test_id':a.test,'runner':__file__,'analyzer':'characterization_stage_tools.py','motion_command_published':False,'publishers_created':0,'service_clients_created':0,'action_clients_created':0,'registry_status':'HOST_INGEST_ON_COMPLETED_SESSION'},indent=2));return 0
    if a.analyze_session:
        if not a.input: raise SystemExit('--analyze-session requires --input')
        session=a.analyze_session
        metadata=json.loads((session/'session_metadata.json').read_text())
        if metadata.get('test_id') != a.test: raise SystemExit('session test_id mismatch')
        payload=json.loads(a.input.read_text()); write(session/'operator_payload.json',payload); report=analyze(a.test,payload);write(session/'analysis.json',report)
        print(json.dumps({'status':report['status'],'session':str(session),'registry_run_id':'HOST_INGEST_REQUIRED','raw_evidence_preserved':True},indent=2));return 0
    session=a.output_root/f'{a.test}_{stamp()}';session.mkdir(parents=True)
    meta={'test_id':a.test,'session_id':session.name,'created_utc':dt.datetime.now(dt.timezone.utc).isoformat(),'physical_motion_authority':'HUMAN_XBOX_ONLY','publisher_count':0,'service_client_count':0,'action_client_count':0,'source':git_identity(),'registry_status':'PENDING_HOST_INGEST'}
    write(session/'session_metadata.json',meta)
    if a.input:
        payload=json.loads(a.input.read_text()); write(session/'operator_payload.json',payload); report=analyze(a.test,payload);write(session/'analysis.json',report)
    elif a.test in RECORDED_STAGES:
        record(a.test,session); write(session/'operator_payload_template.json',template(a.test)); report={'test_id':a.test,'status':'CAPTURED_NEEDS_OPERATOR_PAYLOAD','raw_artifact':'raw.json','operator_payload_template':'operator_payload_template.json','model_promoted':False};write(session/'analysis.json',report)
    elif a.test == 'T28':
        payload=collect_t28();write(session/'operator_payload.json',payload);report=analyze(a.test,payload);write(session/'analysis.json',report)
    else:
        payload=template(a.test);write(session/'operator_payload_template.json',payload);report={'test_id':a.test,'status':'AWAITING_OPERATOR_EVIDENCE','template':'operator_payload_template.json','model_promoted':False};write(session/'analysis.json',report)
    print(json.dumps({'status':report['status'],'session':str(session),'registry_run_id':'HOST_INGEST_REQUIRED'},indent=2));return 0


if __name__=='__main__': raise SystemExit(main())
