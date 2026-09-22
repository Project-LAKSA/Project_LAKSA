from glob import glob
from setuptools import setup

package_name = "laksa_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project LAKSA",
    maintainer_email="project-laksa@invalid.local",
    description="Deterministic manual mapping lifecycle and profiles for LAKSA.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mapping_session_manager = laksa_mapping.session_manager:main",
            "zed_base_pose_adapter = laksa_mapping.zed_base_pose_adapter:main",
            "mapping_shadow_compare = laksa_mapping.comparison_report:main",
        ]
    },
)
