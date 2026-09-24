"""ROS packaging contracts required by colcon/ament_python."""

from pathlib import Path
from unittest import mock
import runpy
import unittest


ROOT = Path(__file__).resolve().parents[1]


class PackagingTests(unittest.TestCase):
    def test_data_file_sources_are_relative_for_ament_python(self):
        with mock.patch("setuptools.setup") as setup:
            runpy.run_path(str(ROOT / "setup.py"), run_name="__main__")
        data_files = setup.call_args.kwargs["data_files"]
        for _, sources in data_files:
            for source in sources:
                self.assertFalse(Path(source).is_absolute(), source)


if __name__ == "__main__":
    unittest.main()
