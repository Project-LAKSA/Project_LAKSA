from glob import glob
from setuptools import find_packages, setup

package_name = "laksa_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    scripts=["scripts/record_lidar_validation"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project LAKSA",
    maintainer_email="project-laksa@invalid.local",
    description="Validated RPLIDAR A2M12 sensor boundary and health gate.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["lidar_guard = laksa_lidar.lidar_guard:main"]},
)
