#!/usr/bin/env python3
"""Declare Waterloo Pure Pursuit's existing nav_msgs build dependency.

The pinned CMake/source already require nav_msgs, but the upstream package.xml
omits it. This packaging-only adapter does not touch controller mathematics.
"""

from pathlib import Path
import sys


path = Path(sys.argv[1])
text = path.read_text()
declaration = "  <depend>nav_msgs</depend>"
if declaration in text:
    raise SystemExit(0)
marker = "  <depend>geometry_msgs</depend>"
if text.count(marker) != 1:
    raise SystemExit("unexpected Waterloo package.xml layout")
path.write_text(text.replace(marker, marker + "\n" + declaration, 1))
