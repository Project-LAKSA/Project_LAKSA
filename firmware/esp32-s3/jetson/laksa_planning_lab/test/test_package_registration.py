import ast
import os
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


class PackageRegistrationTest(unittest.TestCase):
    def test_ament_metadata(self):
        xml = ET.parse(PACKAGE / "package.xml").getroot()
        self.assertEqual(xml.findtext("name"), "laksa_planning_lab")
        build_types = [node.text for node in xml.findall("buildtool_depend")]
        self.assertIn("ament_python", build_types)
        self.assertEqual(xml.find("export/build_type").text, "ament_python")
        self.assertTrue((PACKAGE / "resource" / "laksa_planning_lab").is_file())
        setup_cfg = (PACKAGE / "setup.cfg").read_text(encoding="utf-8")
        self.assertIn("script_dir=$base/lib/laksa_planning_lab", setup_cfg)
        self.assertIn("install_scripts=$base/lib/laksa_planning_lab", setup_cfg)

    def test_setup_installs_registration_runtime_data_and_cli(self):
        captured = {}
        previous = Path.cwd()
        try:
            os.chdir(PACKAGE)
            with patch("setuptools.setup", side_effect=lambda **kwargs: captured.update(kwargs)):
                runpy.run_path(str(PACKAGE / "setup.py"), run_name="__main__")
        finally:
            os.chdir(previous)

        self.assertEqual(captured["packages"], ["laksa_planning_lab"])
        installed = {destination: files for destination, files in captured["data_files"]}
        marker_destination = "share/ament_index/resource_index/packages"
        self.assertIn("resource/laksa_planning_lab", installed[marker_destination])
        self.assertIn("package.xml", installed["share/laksa_planning_lab"])
        destinations = tuple(installed)
        for directory in ("launch", "config", "maps", "lattices"):
            self.assertTrue(any(path == f"share/laksa_planning_lab/{directory}" or path.startswith(f"share/laksa_planning_lab/{directory}/") for path in destinations))
        all_files = [path for files in installed.values() for path in files]
        self.assertFalse(any("__pycache__" in path or path.endswith(".pyc") for path in all_files))

        entry = captured["entry_points"]["console_scripts"]
        self.assertIn("planning_lab = laksa_planning_lab.benchmark_runner:main", entry)
        tree = ast.parse((PACKAGE / "laksa_planning_lab" / "benchmark_runner.py").read_text(encoding="utf-8"))
        self.assertTrue(any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "main" for node in tree.body))


if __name__ == "__main__":
    unittest.main()
