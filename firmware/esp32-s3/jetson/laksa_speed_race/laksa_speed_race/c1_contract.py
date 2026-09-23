"""C1 simulation-only interface and mission contracts.

These declarations deliberately stop at orchestration and evidence collection.
They contain no controller, planner, simulator dynamics, or physical command
adapter. A later ROS launch may only connect ``/drive`` to F1TENTH Gym ROS.
"""

from __future__ import annotations


SIMULATOR_COMMAND_TOPIC = "/drive"
FORBIDDEN_PHYSICAL_COMMAND_TOPICS = frozenset(
    {
        "/laksa/command",
        "/cmd_vel",
        "/laksa/set_drive_command",
    }
)
MAX_LAPS = 3
