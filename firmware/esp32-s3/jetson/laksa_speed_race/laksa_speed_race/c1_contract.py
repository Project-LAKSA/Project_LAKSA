"""C1 simulation-only interface and mission contracts.

This module exists because the stock Gym ROS bridge cannot inject the audited
LAKSA_PROXY_V0 parameters or expose the exact three-lap terminal contract.  It
contains only interface, validation, and safety constants; simulator dynamics
and controller intelligence remain in their pinned upstream projects.
"""

from __future__ import annotations


CONTROLLER_REQUEST_TOPIC = "/c1/drive_request"
SIMULATOR_APPLIED_TOPIC = "/c1/drive_applied"
SIMULATOR_ODOM_TOPIC = "/c1/odom"
FORBIDDEN_PHYSICAL_COMMAND_TOPICS = frozenset(
    {
        "/drive",
        "/laksa/command",
        "/cmd_vel",
        "/laksa/set_drive_command",
    }
)
MAX_LAPS = 3
STEERING_LIMIT_RAD = 0.288
MAX_SPEED_MPS = 1.0
SEED = 12345
DT_S = 0.01
MAP_FRAME = "map"
BASE_FRAME = "c1/base_link"
