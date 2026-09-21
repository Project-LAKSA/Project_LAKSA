"""Pure safety predicates for the dormant characterization input."""
import math

def bounded_request(speed_mps, steering_rad, max_speed_mps, left_limit_rad, right_limit_rad):
    values=(speed_mps,steering_rad,max_speed_mps,left_limit_rad,right_limit_rad)
    # DriveCommand.speed_mps is float32 on the wire. Permit only its tiny
    # serialization rounding at an exactly configured eRPM limit; firmware
    # independently rounds and enforces the final eRPM ceiling.
    speed_rounding=1e-6
    return all(math.isfinite(float(v)) for v in values) and max_speed_mps>0 and left_limit_rad>0 and right_limit_rad>0 and abs(speed_mps)<=max_speed_mps+speed_rounding and -right_limit_rad<=steering_rad<=left_limit_rad

def allow_characterization(*, enabled, heartbeat_fresh, joy_fresh, state_fresh, telemetry_fresh, fault_code, estop, manual_neutral):
    if not enabled:return False,'CHARACTERIZATION_DISABLED'
    if not heartbeat_fresh:return False,'CHARACTERIZATION_HEARTBEAT_EXPIRED'
    if not joy_fresh:return False,'XBOX_STALE'
    if not state_fresh:return False,'ESP32_STATE_STALE'
    if not telemetry_fresh:return False,'VESC_TELEMETRY_STALE'
    if fault_code:return False,f'VESC_FAULT_{fault_code}'
    if estop:return False,'ESTOP_LATCHED'
    if not manual_neutral:return False,'XBOX_MANUAL_OVERRIDE'
    return True,'READY'
