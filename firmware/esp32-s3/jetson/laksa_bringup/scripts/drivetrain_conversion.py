"""Canonical DriveCommand/eRPM conversion used by the production supervisor."""
import math

def erpm_to_speed_mps(erpm, pole_pairs, gear_reduction, wheel_diameter_m):
    return float(erpm) * math.pi * float(wheel_diameter_m) / (60.0 * float(gear_reduction) * float(pole_pairs))

def speed_mps_to_erpm(speed_mps, pole_pairs, gear_reduction, wheel_diameter_m):
    return float(speed_mps) * 60.0 * float(gear_reduction) * float(pole_pairs) / (math.pi * float(wheel_diameter_m))
