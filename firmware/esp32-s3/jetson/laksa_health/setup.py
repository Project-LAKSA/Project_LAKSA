from setuptools import setup

package_name = "laksa_health"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]), ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools", "psutil"], zip_safe=True,
    maintainer="Project LAKSA", maintainer_email="project-laksa@invalid.local",
    description="Standard diagnostics and normalized telemetry for LAKSA.", license="Apache-2.0",
    entry_points={"console_scripts": ["health_monitor = laksa_health.health_monitor:main"]},
)
