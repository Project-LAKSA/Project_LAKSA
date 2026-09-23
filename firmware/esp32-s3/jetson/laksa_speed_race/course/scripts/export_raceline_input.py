#!/usr/bin/env python3
"""Export the recovered centreline in the pinned upstream optimizer's CSV ABI.

The external Raceline-Optimization tool requires rows ordered as
``x_m,y_m,width_right_m,width_left_m`` without a header. This is a lossless
field-order conversion of the canonical centreline; it performs no centreline
extraction, path planning, optimization, or vehicle-dynamics calculation.
"""

from __future__ import annotations

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CANONICAL = ROOT / "canonical" / "speed_course"


def export() -> Path:
    output = CANONICAL / "raceline_input.csv"
    with (CANONICAL / "centerline.csv").open(newline="") as source, output.open("w", newline="") as destination:
        reader = csv.DictReader(source)
        writer = csv.writer(destination, lineterminator="\n")
        for row in reader:
            writer.writerow((row["x_m"], row["y_m"], row["right_clearance_m"], row["left_clearance_m"]))
    return output


if __name__ == "__main__":
    print(export())
