#!/usr/bin/env python3
"""Human-armed, zero-traction T20 static-steering characterization."""
from __future__ import annotations
import argparse, datetime as dt, json, time
from pathlib import Path
import yaml
from interactive_ros import RosCallbackPump

SIGN = "Physical road-wheel angle is relative to vehicle longitudinal centerline; positive is left. Software steering is explicitly not a wheel angle."
FRESHNESS_NS = 500_000_000

def utc(): return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")

def main(argv=None):
    p=argparse.ArgumentParser(description="One-at-a-time, human-armed T20 steering samples.")
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config'/'characterization_tests.yaml')
    p.add_argument('--output-root',type=Path,default=Path('/home/ubuntu/laksa_vehicle_id'))
    p.add_argument('--lab-root',type=Path,default=Path('/home/ubuntu/src/Project_LAKSA/firmware/esp32-s3/tools/a045_closed_loop_autonomy'))
    p.add_argument('--dry-run',action='store_true');a=p.parse_args(argv)
    cfg=yaml.safe_load(a.config.read_text()); t20=cfg['t20']
    if a.dry_run:
        print(json.dumps({'status':'DRY_RUN_OK','targets_rad':t20['targets_rad'],'traction_speed_mps':0.0,'publisher':'/laksa/characterization_request','motion_command_published':False},indent=2));return 0
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from laksa_interfaces.msg import DriveCommand, VehicleState, VescState
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool

    class Capture(Node):
        def __init__(self):
            super().__init__('t20_supervised_steering')
            self.pub=self.create_publisher(DriveCommand,'/laksa/characterization_request',10)
            q=QoSProfile(depth=1);q.reliability=ReliabilityPolicy.RELIABLE;q.durability=DurabilityPolicy.TRANSIENT_LOCAL
            self.gate_pub=self.create_publisher(Bool,'/laksa/characterization_enable',q)
            self.state=None;self.vesc=None;self.joy_ns=0;self.state_ns=0;self.vesc_ns=0;self.estop=False;self.auto=False;self.gate_enabled=None;self.target_steer=0.0
            self.create_subscription(VehicleState,'/laksa/state',self.scb,qos_profile_sensor_data)
            self.create_subscription(VescState,'/laksa/vesc/state',self.vcb,qos_profile_sensor_data)
            self.create_subscription(Joy,'/joy',lambda _m:setattr(self,'joy_ns',time.monotonic_ns()),10)
            self.create_subscription(Bool,'/laksa/emergency_stop',lambda m:setattr(self,'estop',bool(m.data)),q)
            self.create_subscription(Bool,'/laksa/autonomous_enabled',lambda m:setattr(self,'auto',bool(m.data)),q)
            self.create_subscription(Bool,'/laksa/characterization_enabled',lambda m:setattr(self,'gate_enabled',bool(m.data)),q)
            self.create_timer(1.0/float(t20['request_heartbeat_hz']),self.heartbeat)
        def scb(self,m): self.state=m;self.state_ns=time.monotonic_ns()
        def vcb(self,m): self.vesc=m;self.vesc_ns=time.monotonic_ns()
        def health(self):
            now=time.monotonic_ns();ages={'/laksa/state':(now-self.state_ns)/1e9 if self.state_ns else None,'/laksa/vesc/state':(now-self.vesc_ns)/1e9 if self.vesc_ns else None,'/joy':(now-self.joy_ns)/1e9 if self.joy_ns else None}
            missing=[topic for topic,age in ages.items() if age is None];stale=[(topic,age) for topic,age in ages.items() if age is not None and age>FRESHNESS_NS/1e9]
            if missing:return False,{'reason':'MISSING_REQUIRED_TOPIC','topics':missing,'ages_sec':ages}
            if stale:
                topic,age=max(stale,key=lambda x:x[1]);return False,{'reason':'STALE_REQUIRED_TOPIC','topic':topic,'age_sec':age,'ages_sec':ages}
            if self.estop:return False,{'reason':'ESTOP'}
            if self.auto:return False,{'reason':'AUTONOMY_CONFLICT'}
            if self.vesc.fault_code or not self.vesc.telemetry_fresh:return False,{'reason':'VESC_UNHEALTHY'}
            if abs(float(self.vesc.requested_erpm))>0 or abs(float(self.vesc.active_erpm))>0:return False,{'reason':'NONZERO_TRACTION_EVIDENCE','requested_erpm':float(self.vesc.requested_erpm),'active_erpm':float(self.vesc.active_erpm)}
            return True,{'reason':'READY','ages_sec':ages}
        def heartbeat(self):
            q=DriveCommand();q.speed_mps=0.;q.steering_angle_rad=float(self.target_steer);q.brake=False;self.pub.publish(q)
        def set_gate(self,enabled): q=Bool();q.data=bool(enabled);self.gate_pub.publish(q)
        def wait_gate(self,enabled,timeout_sec=3.):
            until=time.monotonic()+timeout_sec
            while time.monotonic()<until:
                self.set_gate(enabled)
                if self.gate_enabled is enabled:return True
                time.sleep(.05)
            return self.gate_enabled is enabled
        def safe_zero(self): self.target_steer=0.;self.set_gate(False)

    root=a.output_root/f'T20_SUPERVISED_{utc()}';root.mkdir(parents=True);raw=root/'T20_static_steering.jsonl'
    meta={'schema_version':'laksa-t20-supervised-v2','test_id':'T20','session_id':root.name,'subscriber_plus_low_priority_supervised_request':True,'traction_speed_mps':0.0,'sign_convention':SIGN,'resolved_config':cfg,'samples':[],'created_utc':dt.datetime.now(dt.timezone.utc).isoformat(),'interactive_callback_architecture':'dedicated ROS executor thread'}
    rclpy.init();n=Capture();pump=RosCallbackPump(n);pump.start()
    def fail(sample_id,detail):
        message=f"[FAIL] T20 sample {sample_id}: {detail['reason']}"
        if detail.get('topic'):message+=f": required topic stale: {detail['topic']}, age={detail['age_sec']:.3f} sec"
        print(message,flush=True);return message
    def fresh_preflight(timeout_sec=3.):
        until=time.monotonic()+timeout_sec;detail={'reason':'MISSING_REQUIRED_TOPIC'}
        while time.monotonic()<until:
            ok,detail=n.health()
            if ok:return True,detail
            time.sleep(.05)
        return False,detail
    try:
        if not n.wait_gate(False):raise RuntimeError('GATE_DISABLE_ACK_TIMEOUT')
        for idx,target in enumerate(t20['targets_rad'],1):
            n.safe_zero();ok,detail=fresh_preflight()
            if not ok:
                meta['samples'].append({'sample_id':idx,'software_steering_target_rad':target,'status':'ABORTED','failure':detail});raise RuntimeError(fail(idx,detail))
            print(f'Sample {idx}: software steering target {target:+.3f} rad, zero traction. Confirm clear wheels/space then type ARM; otherwise type SKIP or QUIT.',flush=True)
            action=input('ARM> ').strip().upper()
            if action=='QUIT':meta['status']='QUIT';break
            if action!='ARM':meta['samples'].append({'sample_id':idx,'software_steering_target_rad':target,'status':'SKIPPED'});continue
            ok,detail=fresh_preflight()
            if not ok:
                meta['samples'].append({'sample_id':idx,'software_steering_target_rad':target,'status':'ABORTED','failure':detail});raise RuntimeError(fail(idx,detail))
            if not n.wait_gate(True):raise RuntimeError('ENABLE_GATE_ACK_TIMEOUT')
            n.target_steer=float(target);time.sleep(float(t20['settle_sec']))
            ok,detail=n.health()
            if not ok:
                meta['samples'].append({'sample_id':idx,'software_steering_target_rad':target,'status':'ABORTED','failure':detail});raise RuntimeError(fail(idx,detail))
            left=input('Left physical road-wheel angle deg (blank=UNKNOWN): ').strip();right=input('Right physical road-wheel angle deg (blank=UNKNOWN): ').strip();method=input('Measurement method: ').strip() or 'UNKNOWN';unc=input('Uncertainty deg (blank=UNKNOWN): ').strip()
            sample={'sample_id':idx,'timestamp_monotonic_ns':time.monotonic_ns(),'software_steering_target_rad':target,'software_steering_current_rad':float(n.state.steering_current_rad),'left_physical_road_wheel_angle_deg':float(left) if left else None,'right_physical_road_wheel_angle_deg':float(right) if right else None,'measurement_method':method,'measurement_uncertainty_deg':float(unc) if unc else None,'status':'CAPTURED','post_measurement_health':n.health()[1]};meta['samples'].append(sample);raw.open('a').write(json.dumps(sample,sort_keys=True)+'\n');n.safe_zero();n.wait_gate(False)
        if meta.get('status')!='QUIT':meta['status']='COMPLETE'
        (root/'T20_static_steering.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
        from automated_characterization_runner import register_dataset
        rid=register_dataset(a.lab_root,root,meta,'NEEDS_REVIEW' if meta['status']=='COMPLETE' else 'QUIT');print(json.dumps({'status':meta['status'],'session':str(root),'registry_run_id':rid},indent=2));return 0
    except Exception as e:
        if meta.get('status')!='QUIT':meta['status']='ABORTED';meta['abort_reason']=str(e)
        (root/'T20_static_steering.yaml').write_text(yaml.safe_dump(meta,sort_keys=False));from automated_characterization_runner import register_dataset;register_dataset(a.lab_root,root,meta,'ABORTED');return 2
    finally:
        n.safe_zero();n.wait_gate(False);pump.stop();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
