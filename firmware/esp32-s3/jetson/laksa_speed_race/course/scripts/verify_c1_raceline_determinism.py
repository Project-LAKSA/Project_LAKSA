#!/usr/bin/env python3
"""Generate the pinned C1 raceline twice and require byte-identical assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import tempfile
from pathlib import Path

from generate_c1_raceline import CANONICAL, OUTPUT_NAMES, generate


def hashes(directory: Path) -> dict[str, str]:
    return {
        name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
        for name in OUTPUT_NAMES
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course-dir", type=Path, default=CANONICAL)
    parser.add_argument("--waterloo-root", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
        first_result = generate(args.course_dir, Path(first), args.waterloo_root)
        second_result = generate(args.course_dir, Path(second), args.waterloo_root)
        if hashes(Path(first)) != hashes(Path(second)):
            raise SystemExit("C1 raceline regeneration is not deterministic")
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "hashes": hashes(Path(first)),
                    "maximum_abs_curvature_1pm": first_result["maximum_abs_curvature_1pm"],
                    "sample_count": first_result["sample_count"],
                    "second_run_matches": first_result["hashes"] == second_result["hashes"],
                },
                sort_keys=True,
            )
        )


if __name__ == "__main__":
    main()
