#!/usr/bin/env python3
"""Pure declarative phase engine for supervised physical identification."""
from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class Phase:
    name: str; duration_s: float; speed_mps: float=0.; steering_rad: float=0.; brake: bool=False

def build(test, variant, cfg):
    p=cfg['protocols'][test][variant]
    neutral=lambda name,d:Phase(name,d)
    if test=='T21A': return [neutral('PRE_SETTLE',1),Phase('TRACTION_SETTLE',1,p['speed_mps']),Phase('STEER_HOLD',p['steer_hold_sec'],p['speed_mps'],p['steering_rad']),Phase('STEER_NEUTRAL',1,p['speed_mps']),neutral('TRACTION_NEUTRAL',2)]
    if test=='T22': return [neutral('PRE_SETTLE',1),Phase('MEASURED_RUN',p['max_drive_sec'],p['speed_mps']),neutral('NEUTRAL',2)]
    if test=='T23': return [neutral('PRE_SETTLE',1),Phase('STEP_HOLD',p['hold_sec'],p['speed_mps']),neutral('COASTDOWN',p['coast_sec'])]
    if test=='T24': return [neutral('PRE_SETTLE',1),Phase('ACCELERATE',p['hold_sec'],p['speed_mps']),Phase('BRAKE',p['brake_sec'],0.,0.,True),neutral('POST_BRAKE',p['coast_sec'])]
    if test=='T25': return [neutral('PRE_SETTLE',1),Phase('STEER_TARGET',p['hold_sec'],0.,p['steering_rad']),neutral('STEER_NEUTRAL',2)]
    if test in ('T21B','T26'): return [neutral('PRE_SETTLE',1),Phase('TURN_HOLD',p['hold_sec'],p['speed_mps'],p['steering_rad']),Phase('TURN_NEUTRAL',1,p['speed_mps']),neutral('TRACTION_NEUTRAL',2)]
    if test=='T27': return [neutral('PRE_SETTLE',1),Phase('LEVEL_HOLD',p['hold_sec'],p['speed_mps']),neutral('NEUTRAL',2)]
    raise ValueError(test)

def validate(phases):
    """No opposite steering/direction is permitted without an intervening neutral."""
    prior_speed=prior_steer=0.
    for phase in phases:
        if prior_speed*phase.speed_mps<0: raise ValueError('UNSAFE_DIRECTION_CHANGE_WITHOUT_NEUTRAL')
        if prior_steer*phase.steering_rad<0: raise ValueError('UNSAFE_STEERING_CHANGE_WITHOUT_NEUTRAL')
        if phase.speed_mps==0: prior_speed=0.
        else: prior_speed=phase.speed_mps
        if phase.steering_rad==0: prior_steer=0.
        else: prior_steer=phase.steering_rad
    return True

class Engine:
    """Testable ARM/gate/phase state machine; no ROS or motion I/O."""
    def __init__(self, phases): validate(phases);self.phases=phases;self.state='PREFLIGHT';self.index=-1;self.abort_reason=None
    def arm(self):
        if self.state!='WAIT_ARM':raise RuntimeError('ARM_REQUIRES_PREFLIGHT')
        self.state='ENABLE'
    def preflight_ok(self):
        if self.state!='PREFLIGHT':raise RuntimeError('PREFLIGHT_TRANSITION')
        self.state='WAIT_ARM'
    def enabled(self, now):
        if self.state!='ENABLE':raise RuntimeError('ENABLE_TRANSITION')
        self.index=0;self.started=now;self.state='PHASE'
    def tick(self, now, safe=True, request_heartbeat=True, gate_heartbeat=True):
        if self.state!='PHASE':return self.state
        if not gate_heartbeat:return self.abort('GATE_HEARTBEAT_LOST')
        if not request_heartbeat:return self.abort('REQUEST_HEARTBEAT_LOST')
        if not safe:return self.abort('SAFETY_LOST')
        if now-self.started>=self.phases[self.index].duration_s:
            self.index+=1;self.started=now
            if self.index>=len(self.phases):self.state='SETTLE'
        return self.state
    def finish(self):
        if self.state=='PHASE': self.index=len(self.phases)-1;self.started=0
    def settled(self):
        if self.state=='SETTLE':self.state='COMPLETE'
        return self.state
    def abort(self, reason):self.abort_reason=reason;self.state='ABORT';return self.state
    @property
    def request(self):return self.phases[self.index] if self.state=='PHASE' else Phase('NEUTRAL',0)

def guard(snapshot):
    """Pure fail-closed classification used by synthetic safety fixtures."""
    if snapshot.get('estop'):return 'ESTOP'
    if snapshot.get('manual_override'):return 'XBOX_MANUAL_OVERRIDE'
    if snapshot.get('stale'):return 'STALE_TOPIC'
    if snapshot.get('vesc_fault'):return 'VESC_FAULT_OR_STALE'
    return 'READY'
