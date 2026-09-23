#!/usr/bin/env python3
"""Regenerate only the approved Speed Course from its frozen LAKSA geometry.

This wrapper deliberately invokes the recovered deterministic generator rather
than redrawing or fitting course geometry. It contains no simulator, planner,
controller, or vehicle command logic.
"""

from rebuild import build


if __name__ == "__main__":
    build("speed_course")
