import os
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


class PackageRegistrationTest(unittest.TestCase):
    def test_ament_metadata_and_marker_exist(self):
        xml = ET.parse(PACKAGE / "package.xml").getroot()
        self.assertEqual(xml.findtext("name"), "laksa_dashboard")
        self.assertIn("ament_python", [node.text for node in xml.findall("buildtool_depend")])
        self.assertEqual(xml.find("export/build_type").text, "ament_python")
        self.assertTrue((PACKAGE / "resource" / "laksa_dashboard").is_file())

    def test_setup_explicitly_installs_registration_and_runtime_data(self):
        captured = {}
        previous = Path.cwd()
        try:
            os.chdir(PACKAGE)
            with patch("setuptools.setup", side_effect=lambda **kwargs: captured.update(kwargs)):
                runpy.run_path(str(PACKAGE / "setup.py"), run_name="__main__")
        finally:
            os.chdir(previous)

        installed = {destination: files for destination, files in captured["data_files"]}
        marker_destination = "share/ament_index/resource_index/packages"
        self.assertIn("resource/laksa_dashboard", installed[marker_destination])
        self.assertIn("package.xml", installed["share/laksa_dashboard"])
        for directory in ("launch", "config", "web"):
            self.assertTrue(any(path.startswith(f"share/laksa_dashboard/{directory}") for path in installed))
        self.assertIn("cockpit_server = laksa_dashboard.cockpit_server:main", captured["entry_points"]["console_scripts"])


if __name__ == "__main__":
    unittest.main()
