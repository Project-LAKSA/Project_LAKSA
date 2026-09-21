from pathlib import Path
from setuptools import setup

package_name = "laksa_planning_lab"


def install_tree(directory: str):
    """Preserve runtime resource subdirectories without cache artifacts."""
    root = Path(directory)
    result = []
    parents = [root] + sorted(path for path in root.rglob("*") if path.is_dir())
    for parent in parents:
        files = sorted(
            str(path) for path in parent.iterdir()
            if path.is_file() and path.suffix != ".pyc" and "__pycache__" not in path.parts
        )
        if files:
            destination = Path("share") / package_name / parent
            result.append((str(destination), files))
    return result

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ] + install_tree("launch") + install_tree("config") + install_tree("maps") + install_tree("lattices"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project LAKSA",
    maintainer_email="ubuntu@localhost",
    description="Offline deterministic global-planning laboratory",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "planning_lab = laksa_planning_lab.benchmark_runner:main",
        "planning_lab_maps = laksa_planning_lab.map_dataset:main",
        "planning_lab_scenarios = laksa_planning_lab.scenario_generator:main",
        "planning_lab_lattice = laksa_planning_lab.lattice_tools:main",
    ]},
)
