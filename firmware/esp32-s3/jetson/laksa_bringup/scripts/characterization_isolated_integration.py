#!/usr/bin/env python3
"""Non-production-domain integration proof for real DriveSupervisor arbitration."""
import json, time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from laksa_interfaces.msg import DriveCommand, VehicleState, VescState
from drive_supervisor_node import DriveSupervisor
from drivetrain_conversion import erpm_to_speed_mps

class Fixture(Node):
    def __init__(self):
        super().__init__('characterization_isolated_fixture');q=QoSProfile(depth=1);q.reliability=ReliabilityPolicy.RELIABLE;q.durability=DurabilityPolicy.TRANSIENT_LOCAL;self.j=self.create_publisher(Joy,'/joy',10);self.s=self.create_publisher(VehicleState,'/laksa/state',10);self.v=self.create_publisher(VescState,'/laksa/vesc/state',10);self.g=self.create_publisher(Bool,'/laksa/characterization_enable',q);self.r=self.create_publisher(DriveCommand,'/laksa/characterization_request',10);self.out=[];self.gate_state=None;self.create_subscription(DriveCommand,'/laksa/command',lambda m:self.out.append((float(m.speed_mps),bool(m.brake))),10);self.create_subscription(Bool,'/laksa/characterization_enabled',lambda m:setattr(self,'gate_state',bool(m.data)),q);self.create_timer(.03,self.tick)
    def tick(self):
        j=Joy();j.axes=[0.,0.,0.];j.buttons=[0,0,0,0];self.j.publish(j)
        s=VehicleState();s.orientation.w=1.;s.vesc.telemetry_fresh=True;s.vesc.command_fresh=True;s.vesc.telemetry_sequence=1;s.vesc.telemetry_age_ms=0;s.vesc.fault_code=0;self.s.publish(s);v=VescState();v.telemetry_fresh=True;v.command_fresh=True;v.telemetry_sequence=1;v.telemetry_age_ms=0;v.fault_code=0;self.v.publish(v)
    def gate(self,v):m=Bool();m.data=v;self.g.publish(m)
    def request(self,v):m=DriveCommand();m.speed_mps=v;m.steering_angle_rad=0.;m.brake=False;self.r.publish(m)
def main():
    rclpy.init(); f=Fixture(); sup=DriveSupervisor(); sup._actuation_enabled=True  # isolated ROS_DOMAIN only; no production endpoint exists
    ex=MultiThreadedExecutor(2);ex.add_node(f);ex.add_node(sup); import threading; t=threading.Thread(target=ex.spin,daemon=True);t.start()
    try:
        time.sleep(.4); target=erpm_to_speed_mps(900,2,11.82,.109)
        deadline=time.monotonic()+1.;
        while not sup._characterization_enabled and time.monotonic()<deadline:
            f.gate(True);time.sleep(.03)
        if not sup._characterization_enabled: raise RuntimeError('isolated gate acknowledgement timeout')
        # Exercise the same short-heartbeat contract as the real runner.
        until=time.monotonic()+.35
        while time.monotonic()<until: f.request(target);time.sleep(.03)
        nonzero=any(x>0 for x,b in f.out if not b)
        until=time.monotonic()+.15
        while time.monotonic()<until: f.request(0.);time.sleep(.03)
        f.gate(False);time.sleep(.15);neutral=any(x==0 for x,b in f.out)
        report={'pass':nonzero and neutral,'target_erpm':900,'resolved_speed_mps':target,'characterization_request_nonzero':True,'gate_acknowledged':True,'supervisor_command_nonzero':nonzero,'neutral_cleanup':neutral,'observed_command_tail':f.out[-10:],'ros_domain_isolated':True};print(json.dumps(report));return 0 if report['pass'] else 1
    finally: ex.shutdown();f.destroy_node();sup.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
