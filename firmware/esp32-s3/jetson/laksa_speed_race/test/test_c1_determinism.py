"""Run the recovered generator twice in its explicitly supplied tool environment."""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def hashes(course_root: Path) -> dict[str, str]:
    canonical = course_root / "canonical" / "speed_course"
    return {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(canonical.iterdir())
        if path.is_file() and path.name != "course_manifest.json"
    }


class C1DeterminismTests(unittest.TestCase):
    def test_rebuild_twice_is_byte_deterministic(self):
        python = os.environ.get("LAKSA_COURSE_TOOL_PYTHON", sys.executable)
        with tempfile.TemporaryDirectory() as temp_dir:
            course_root = Path(temp_dir) / "course"
            shutil.copytree(ROOT / "course", course_root)
            script = course_root / "scripts" / "rebuild_speed_course.py"
            subprocess.run([python, str(script)], check=True)
            first = hashes(course_root)
            subprocess.run([python, str(script)], check=True)
            self.assertEqual(first, hashes(course_root))


if __name__ == "__main__":
    unittest.main()
