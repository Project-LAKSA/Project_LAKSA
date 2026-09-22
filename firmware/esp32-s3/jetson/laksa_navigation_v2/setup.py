from glob import glob
from setuptools import setup


package_name = "laksa_navigation_v2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/laksa_navigation_v2"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.json")),
        (f"share/{package_name}/config/generated", glob("config/generated/*.json")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "g2_synthetic_sensor_node = laksa_navigation_v2.g2_synthetic_sensor_node:main",
        "g2_ros_qualification = laksa_navigation_v2.g2_ros_qualification:main",
        "vehicle_speed_adapter_node = laksa_navigation_v2.vehicle_speed_adapter_node:main",
    ]},
)
