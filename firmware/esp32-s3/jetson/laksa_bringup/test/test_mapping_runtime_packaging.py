"""Static contract for LAKSA-owned executables in the fused mapping launch.

The production mapper is launched from source-controlled package metadata.
This test prevents a successful build from omitting a launch-required
executable from its package install rule.
"""

from __future__ import annotations

import ast
import os
import re
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[2]
MAPPING_LAUNCH = JETSON_ROOT / "laksa_mapping" / "launch" / "mapping_stack.launch.py"
BRINGUP_CMAKE = JETSON_ROOT / "laksa_bringup" / "CMakeLists.txt"
MAPPING_SETUP = JETSON_ROOT / "laksa_mapping" / "setup.py"


def _mapping_launch_nodes() -> set[tuple[str, str]]:
    tree = ast.parse(MAPPING_LAUNCH.read_text(encoding="utf-8"))
    nodes: set[tuple[str, str]] = set()
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        values = {
            keyword.arg: keyword.value.value
            for keyword in call.keywords
            if keyword.arg in {"package", "executable"}
            and isinstance(keyword.value, ast.Constant)
            and isinstance(keyword.value.value, str)
        }
        if {"package", "executable"} <= values.keys():
            nodes.add((values["package"], values["executable"]))
    return nodes


def _bringup_installed_programs() -> set[str]:
    cmake = BRINGUP_CMAKE.read_text(encoding="utf-8")
    match = re.search(
        r"install\s*\(\s*PROGRAMS(?P<programs>.*?)DESTINATION\s+lib/\$\{PROJECT_NAME\}",
        cmake,
        flags=re.DOTALL,
    )
    assert match, "laksa_bringup must install its runtime programs"
    return set(re.findall(r"scripts/([^\s)]+)", match.group("programs")))


def test_fused_mapping_launch_packages_every_laksa_executable() -> None:
    """Every LAKSA-owned Node executable used by the fused mapper is packaged."""
    local_nodes = {
        node for node in _mapping_launch_nodes() if node[0] in {"laksa_bringup", "laksa_mapping"}
    }
    assert local_nodes == {
        ("laksa_bringup", "state_measurements_node.py"),
        ("laksa_mapping", "zed_base_pose_adapter"),
    }

    assert "state_measurements_node.py" in _bringup_installed_programs()
    state_measurements = JETSON_ROOT / "laksa_bringup" / "scripts" / "state_measurements_node.py"
    assert os.access(state_measurements, os.X_OK), (
        "state_measurements_node.py must be executable so install(PROGRAMS) and ros2 can resolve it"
    )

    mapping_setup = MAPPING_SETUP.read_text(encoding="utf-8")
    assert re.search(
        r"zed_base_pose_adapter\s*=\s*laksa_mapping\.zed_base_pose_adapter:main",
        mapping_setup,
    ), "laksa_mapping must install zed_base_pose_adapter as its console entry point"
