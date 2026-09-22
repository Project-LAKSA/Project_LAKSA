#!/usr/bin/env python3
"""T21 human chassis-rotation IMU sanity capture; intentionally subscriber-only.

The ROS executor stays in a dedicated thread while the operator is at stdin.
Each manual motion is captured *after* its stage is armed, not after the
operator has stopped moving the chassis.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import statistics
import time
from pathlib import Path

from interactive_ros import RosCallbackPump

BASELINE_SECONDS = 4.0
STAGE_TIMEOUT_SECONDS = 15.0
RETURN_DWELL_SECONDS = 1.0
MAX_SAMPLE_GAP_SECONDS = 0.20
MIN_SIGNAL_RAD_S = 0.01


def stamp(): return dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def stats(rows):
    z = [float(r['yaw_rad_s']) for r in rows]
    ordered = sorted(set(z)); steps = [b-a for a,b in zip(ordered, ordered[1:]) if b>a]
    return {'n':len(z),'mean':statistics.mean(z) if z else None,'median':statistics.median(z) if z else None,
            'std':statistics.pstdev(z) if len(z)>1 else None,'min':min(z) if z else None,'max':max(z) if z else None,
            'zero_fraction':sum(x==0 for x in z)/len(z) if z else None,
            'quantization_step_rad_s':min(steps) if steps else None}


def threshold_for(baseline):
    """Threshold deliberately above sensor quantization and baseline noise."""
    return max(MIN_SIGNAL_RAD_S, 5.0*float(baseline.get('std') or 0.0),
               3.0*float(baseline.get('quantization_step_rad_s') or 0.0))


def _max_gap(rows):
    return max((float(b['recv_monotonic'])-float(a['recv_monotonic']) for a,b in zip(rows,rows[1:])),default=0.0)


def quiet_dwell_after_motion(rows, baseline):
    """Return the measured quiet dwell after the final above-threshold sample.

    This is intentionally shared by capture and analysis.  The former used to
    look backwards one second from its newest sample while the latter measured
    forwards from the first quiet sample.  At a finite IMU cadence those are
    not equivalent and could make a capture complete while its own analysis
    rejected it by a few milliseconds.
    """
    threshold=threshold_for(baseline); median=float(baseline.get('median') or 0.0)
    active=[i for i,row in enumerate(rows) if abs(float(row['yaw_rad_s'])-median)>=threshold]
    if not active:
        return {'observed':False,'returned':False,'quiet_start_recv_monotonic':None,'quiet_duration_s':0.0}
    last=active[-1]; quiet_start=None
    for row in rows[last+1:]:
        quiet=abs(float(row['yaw_rad_s'])-median)<threshold
        if quiet and quiet_start is None:
            quiet_start=float(row['recv_monotonic'])
        elif not quiet:
            quiet_start=None
        if quiet_start is not None and float(row['recv_monotonic'])-quiet_start>=RETURN_DWELL_SECONDS:
            return {'observed':True,'returned':True,'quiet_start_recv_monotonic':quiet_start,
                    'quiet_duration_s':float(row['recv_monotonic'])-quiet_start}
    end=float(rows[-1]['recv_monotonic']) if rows else 0.0
    return {'observed':True,'returned':False,'quiet_start_recv_monotonic':quiet_start,
            'quiet_duration_s':end-quiet_start if quiet_start is not None else 0.0}


def analyze_stage(rows, baseline, *, timeout=False):
    """Analyze one complete armed-stage capture without assuming yaw sign."""
    threshold=threshold_for(baseline); median=float(baseline.get('median') or 0.0)
    active=[i for i,row in enumerate(rows) if abs(float(row['yaw_rad_s'])-median)>=threshold]
    common={'threshold_rad_s':threshold,'sample_count':len(rows),'dropped_or_stale_samples':_max_gap(rows)>MAX_SAMPLE_GAP_SECONDS,
            'max_receive_gap_s':_max_gap(rows),'timed_out':bool(timeout)}
    if not active:return {**common,'status':'NO_MOTION_DETECTED'}
    first,last=active[0],active[-1]; motion=rows[first:last+1]; peak=max(motion,key=lambda row:abs(float(row['yaw_rad_s'])-median))
    yaw=sum((float(b['yaw_rad_s'])+float(a['yaw_rad_s'])-2*median)*.5*(float(b['recv_monotonic'])-float(a['recv_monotonic'])) for a,b in zip(motion,motion[1:]))
    tail=rows[last+1:]; dwell=quiet_dwell_after_motion(rows,baseline); returned=dwell['returned']
    residual=(statistics.mean(float(row['yaw_rad_s']) for row in tail)-median) if tail else None
    peak_value=float(peak['yaw_rad_s']); noise=max(float(baseline.get('std') or 0.0),float(baseline.get('quantization_step_rad_s') or 0.0),1e-12)
    return {**common,'status':'MOTION_DETECTED' if returned and not timeout and not common['dropped_or_stale_samples'] else 'MOTION_NEEDS_REVIEW',
            'onset_recv_monotonic':rows[first]['recv_monotonic'],'onset_ros_stamp':rows[first]['ros_stamp'],
            'motion_end_recv_monotonic':rows[last]['recv_monotonic'],'motion_end_ros_stamp':rows[last]['ros_stamp'],
            'peak_yaw_rad_s':peak_value,'dominant_sign':'POSITIVE' if peak_value-median>0 else 'NEGATIVE',
            'integrated_relative_yaw_delta_rad':yaw,'duration_s':float(rows[last]['recv_monotonic'])-float(rows[first]['recv_monotonic']),
            'signal_to_noise_ratio':abs(peak_value-median)/noise,'return_to_baseline':returned,
            'quiet_dwell_duration_s':dwell['quiet_duration_s'],
            'return_to_baseline_residual_rad_s':residual}


def capture_stage(rows, label, baseline, *, clock=time.monotonic, sleeper=time.sleep, prompt=input, timeout_s=STAGE_TIMEOUT_SECONDS):
    """Start capture immediately after ENTER; operator never ends capture with a key."""
    prompt(f'Press ENTER when ready for {label} manual rotation. After pressing ENTER, slowly rotate LAKSA {label} by hand and stop. Software is recording continuously: ')
    start=len(rows); began=clock(); threshold=threshold_for(baseline); median=float(baseline.get('median') or 0.0); print(f'[INFO] {label} stage armed and recording. Rotate now; waiting for motion and stable return.',flush=True)
    # Use the same forward-time dwell predicate as analyze_stage.  This keeps
    # a finite-rate sample boundary from producing a self-contradictory result.
    while clock()-began<timeout_s:
        stage=list(rows[start:])
        if stage and any(abs(float(row['yaw_rad_s'])-median)>=threshold for row in stage):
            if quiet_dwell_after_motion(stage,baseline)['returned']:return stage,False
        sleeper(.05)
    return list(rows[start:]),True


def main(argv=None):
    p=argparse.ArgumentParser(description='Subscriber-only T21 IMU yaw sanity capture.');p.add_argument('--output-root',type=Path,default=Path('/home/ubuntu/laksa_vehicle_id'));p.add_argument('--dry-run',action='store_true');a=p.parse_args(argv)
    if a.dry_run:
        print(json.dumps({'status':'DRY_RUN_OK','motion_command_published':False,'publishers_created':0,'test':'T21_IMU_SANITY','registry_status':'HOST_INGEST_ON_COMPLETED_SESSION'},indent=2));return 0
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    class Watch(Node):
        def __init__(self):super().__init__('t21_imu_sanity_observer');self.rows=[];self.create_subscription(Imu,'/laksa/imu/data',self.cb,qos_profile_sensor_data)
        def cb(self,msg):self.rows.append({
            'recv_monotonic':time.monotonic(),'ros_stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'yaw_rad_s':float(msg.angular_velocity.z),'frame_id':msg.header.frame_id,
            'angular_velocity_rad_s':{'x':float(msg.angular_velocity.x),'y':float(msg.angular_velocity.y),'z':float(msg.angular_velocity.z)},
            'linear_acceleration_m_s2':{'x':float(msg.linear_acceleration.x),'y':float(msg.linear_acceleration.y),'z':float(msg.linear_acceleration.z)},
            'orientation_quaternion':{'x':float(msg.orientation.x),'y':float(msg.orientation.y),'z':float(msg.orientation.z),'w':float(msg.orientation.w)},
        })
    rclpy.init();node=Watch();pump=RosCallbackPump(node);pump.start();root=a.output_root/f'T21_IMU_SANITY_{stamp()}';root.mkdir(parents=True)
    try:
        print('T21 IMU YAW SANITY — HUMAN_PHYSICAL_NO_TRACTION. This software has no actuator publishers.',flush=True)
        print(f'Leave LAKSA stationary: recording automatic baseline for {BASELINE_SECONDS:.0f} seconds.',flush=True);time.sleep(BASELINE_SECONDS);baseline_rows=list(node.rows);baseline=stats(baseline_rows);baseline['max_receive_gap_s']=_max_gap(baseline_rows);baseline['dropped_or_stale_samples']=baseline['max_receive_gap_s']>MAX_SAMPLE_GAP_SECONDS;baseline['stable']=bool(baseline['n'] and not baseline['dropped_or_stale_samples'])
        left_rows,left_timeout=capture_stage(node.rows,'LEFT',baseline);right_rows,right_timeout=capture_stage(node.rows,'RIGHT',baseline)
        left=analyze_stage(left_rows,baseline,timeout=left_timeout);right=analyze_stage(right_rows,baseline,timeout=right_timeout)
        opposite=left.get('dominant_sign') and right.get('dominant_sign') and left['dominant_sign']!=right['dominant_sign'];passed=baseline['stable'] and left.get('status')=='MOTION_DETECTED' and right.get('status')=='MOTION_DETECTED' and opposite
        report={'schema_version':'laksa-t21-imu-sanity-v2','test_id':'T21_IMU_SANITY','status':'PASS' if passed else 'NEEDS_REVIEW','physical_motion_authority':'HUMAN_PHYSICAL_NO_TRACTION','motion_command_published':False,'publishers_created':0,'baseline':baseline,'left':left,'right':right,'imu_yaw_sign_convention':{'discovered_from':'manual LEFT/RIGHT data','result':'OPPOSITE_RESPONSES_REQUIRED'},'frame_id':next((row['frame_id'] for row in node.rows if row['frame_id']),'UNKNOWN'),'recorded_imu_channels':['angular_velocity.x/y/z','linear_acceleration.x/y/z','orientation quaternion'],'limitations':['relative gyro integration only','no curvature','no radius','no absolute heading accuracy','no steering dynamics','cross-axis and attitude channels are recorded for quality review, not automatically interpreted as vehicle dynamics'],'registry_status':'PENDING_HOST_INGEST'}
        (root/'raw.jsonl').write_text('\n'.join(json.dumps(row,sort_keys=True) for row in node.rows)+'\n');(root/'analysis.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
        print(json.dumps({'status':report['status'],'session':str(root),'registry_status':'PENDING_HOST_INGEST','registry_run_id':'HOST_INGEST_REQUIRED'},indent=2));return 0 if passed else 2
    finally:pump.stop();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
