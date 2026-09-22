#!/usr/bin/env python3
"""Guided, zero-traction T25/T31 steering hysteresis and repeatability capture."""
from __future__ import annotations
import argparse, datetime as dt, json, os, tempfile, time
from pathlib import Path
import yaml
from interactive_ros import RosCallbackPump

# T20 already has one valid physical value at every nonzero command.  Only
# missing repeat/return branches are collected; peak positions are traversed
# without a duplicate measurement to preserve the hysteresis approach path.
SEQUENCE=[]
for repeat in range(1,4):
    record_inc=repeat < 3
    SEQUENCE += [(repeat,'positive',.12,'increasing',record_inc),(repeat,'positive',.24,'increasing',record_inc),(repeat,'positive',.48,'peak_transition',False),(repeat,'positive',.24,'decreasing',True),(repeat,'positive',.12,'decreasing',True),(repeat,'positive',0.,'center_return',True)]
for repeat in range(1,4):
    record_inc=repeat < 3
    SEQUENCE += [(repeat,'negative',-.08,'increasing_magnitude',record_inc),(repeat,'negative',-.16,'increasing_magnitude',record_inc),(repeat,'negative',-.27,'peak_transition',False),(repeat,'negative',-.16,'decreasing_magnitude',True),(repeat,'negative',-.08,'decreasing_magnitude',True),(repeat,'negative',0.,'center_return',True)]

def atomic(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False) as f:
        json.dump(value,f,indent=2,sort_keys=True);f.write('\n'); name=f.name
    os.replace(name,path)
def stamp(): return dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
def valid(value):
    value=float(value)
    if not -90<=value<=90: raise ValueError('wheel angle must be within [-90,90] deg')
    return value

def analyze(root, samples, t20):
    from characterization_stage_tools import analyze
    t25=analyze('T25',{'samples':samples})
    values=[]
    for sample in samples:
        for wheel in ('left_road_wheel_deg','right_road_wheel_deg'):
            values.append({'condition':f"{wheel}:{sample['software_steering_rad']:+.3f}:{sample['approach_direction']}", 'value':sample[wheel]})
    t31=analyze('T31',{'samples':values})
    report={'schema_version':'laksa-t25-t31-guided-analysis-v1','status':'PASS' if t25['status']=='PASS' and t31['status']=='PASS' else 'NEEDS_REVIEW',
            'T25':t25,'T31':t31,'t20_static_map_consistency':{'source':str(t20) if t20 else None,'classification':'INDEPENDENT_STATIC_MAP_REFERENCE_ONLY'}}
    atomic(root/'analysis.json',report); return report

def main(argv=None):
    p=argparse.ArgumentParser(description='Guided T25/T31 zero-traction road-wheel measurement capture')
    p.add_argument('--output-root',type=Path,default=Path('/home/ubuntu/laksa_vehicle_id'))
    p.add_argument('--resume',type=Path);p.add_argument('--dry-run',action='store_true');a=p.parse_args(argv)
    plan=[{'index':i+1,'repeat':r,'sign':s,'software_steering_rad':q,'approach_direction':d,'record_physical_angle':record} for i,(r,s,q,d,record) in enumerate(SEQUENCE)]
    if a.dry_run:
        print(json.dumps({'status':'DRY_RUN_OK','test_ids':['T25','T31'],'traction_speed_mps':0.,'gate_initial':False,'gate_final':False,'motion_command_published':False,'points':plan,'resume':'atomic per-point JSON'},indent=2));return 0
    root=a.resume or a.output_root/f'T25_T31_GUIDED_{stamp()}'; root.mkdir(parents=True,exist_ok=True)
    state_path=root/'capture_state.json'; raw=root/'raw.jsonl'
    state=json.loads(state_path.read_text()) if state_path.exists() else {'schema_version':'laksa-t25-t31-guided-v1','status':'PREFLIGHT','samples':[],'next_index':0,'plan':plan}
    if state.get('plan')!=plan: raise RuntimeError('RESUME_PLAN_MISMATCH')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from laksa_interfaces.msg import DriveCommand, VehicleState, VescState
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool
    class Node_(Node):
        def __init__(self):
            super().__init__('t25_t31_guided_capture'); self.target=0.;self.state=self.vesc=None;self.n={};self.gate=False
            self.pub=self.create_publisher(DriveCommand,'/laksa/characterization_request',10);q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.gate_pub=self.create_publisher(Bool,'/laksa/characterization_enable',q)
            self.create_subscription(VehicleState,'/laksa/state',lambda x:(setattr(self,'state',x),self.n.__setitem__('state',time.monotonic_ns())),qos_profile_sensor_data);self.create_subscription(VescState,'/laksa/vesc/state',lambda x:(setattr(self,'vesc',x),self.n.__setitem__('vesc',time.monotonic_ns())),qos_profile_sensor_data);self.create_subscription(Joy,'/joy',lambda x:self.n.__setitem__('joy',time.monotonic_ns()),10);self.create_subscription(Bool,'/laksa/characterization_enabled',lambda x:setattr(self,'gate',bool(x.data)),q);self.create_timer(.05,self.heart)
        def heart(self):
            x=DriveCommand();x.speed_mps=0.;x.steering_angle_rad=self.target;x.brake=False;self.pub.publish(x)
        def setgate(self,v): x=Bool();x.data=v;self.gate_pub.publish(x)
        def health(self):
            now=time.monotonic_ns(); missing=[k for k in ('state','vesc','joy') if now-self.n.get(k,0)>500_000_000]
            if missing:return False,'STALE_'+','.join(missing)
            if self.vesc.fault_code or not self.vesc.telemetry_fresh or not self.vesc.command_fresh:return False,'VESC_UNHEALTHY'
            if any(abs(float(getattr(self.vesc,k)))>75 for k in ('requested_erpm','active_erpm','measured_erpm')):return False,'NONZERO_TRACTION'
            return True,'READY'
        def neutral(self): self.target=0.;self.setgate(False)
    rclpy.init();n=Node_();pump=RosCallbackPump(n);pump.start()
    try:
        n.neutral(); time.sleep(.7)
        for point in plan[state.get('next_index',0):]:
            ok,reason=n.health()
            if not ok: raise RuntimeError('PREFLIGHT_'+reason)
            n.target=point['software_steering_rad']; n.setgate(True); time.sleep(1.5)
            ok,reason=n.health()
            if not ok: raise RuntimeError('POINT_'+reason)
            if not point['record_physical_angle']:
                state['next_index']=point['index'];atomic(state_path,state);continue
            while True:
                print(f"T25/T31 {point['index']}/{len(plan)} | {point['sign']} sweep {point['repeat']} | command {point['software_steering_rad']:+.3f} rad | approach={point['approach_direction']}",flush=True)
                left=input('LEFT road-wheel angle (deg): ').strip(); right=input('RIGHT road-wheel angle (deg; BACK re-enters this point): ').strip()
                if right.upper()=='BACK': continue
                sample={**point,'timestamp_utc':dt.datetime.now(dt.timezone.utc).isoformat(),'timestamp_monotonic_ns':time.monotonic_ns(),'left_road_wheel_deg':valid(left),'right_road_wheel_deg':valid(right),'measurement_method':'physical manual road-wheel angle measurement','measurement_uncertainty_deg':1.0,'software_steering_current_rad':float(n.state.steering_current_rad)}
                state['samples'].append(sample);state['next_index']=point['index'];state['status']='CAPTURING';atomic(state_path,state)
                with raw.open('a') as f:f.write(json.dumps(sample,sort_keys=True)+'\n')
                break
        n.neutral();time.sleep(.7);ok,reason=n.health()
        if not ok: raise RuntimeError('FINAL_NEUTRAL_'+reason)
        t20=next(iter(sorted(a.output_root.glob('campaign_*/T20_SUPERVISED_*/T20_static_steering.yaml'))),None)
        report=analyze(root,state['samples'],t20);state['status']='COMPLETE';state['analysis_status']=report['status'];atomic(state_path,state);print(json.dumps({'status':state['status'],'session':str(root),'analysis':report['status']},indent=2));return 0
    except Exception as e:
        state['status']='ABORTED';state['reason']=str(e);atomic(state_path,state);raise
    finally:
        n.neutral();time.sleep(.2);pump.stop();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
