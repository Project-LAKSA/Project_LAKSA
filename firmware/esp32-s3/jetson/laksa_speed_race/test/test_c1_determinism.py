"""Run the recovered generator twice in its explicitly supplied tool environment."""

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]


def hashes() -> dict[str, str]:
    canonical = ROOT / "course" / "canonical" / "speed_course"
    return {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(canonical.iterdir())
        if path.is_file() and path.name != "course_manifest.json"
    }


class C1DeterminismTests(unittest.TestCase):
    def test_rebuild_twice_is_byte_deterministic(self):
        python = os.environ.get("LAKSA_COURSE_TOOL_PYTHON", sys.executable)
        script = ROOT / "course" / "scripts" / "rebuild_speed_course.py"
        subprocess.run([python, str(script)], check=True)
        first = hashes()
        subprocess.run([python, str(script)], check=True)
        self.assertEqual(first, hashes())


if __name__ == "__main__":
    unittest.main()
