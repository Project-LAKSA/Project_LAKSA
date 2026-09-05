from glob import glob
from setuptools import setup

package_name = "laksa_dashboard"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob("web/*.html") + glob("web/*.css") + glob("web/*.js")),
        ("share/" + package_name + "/web/vendor", glob("web/vendor/*")),
    ],
    install_requires=["setuptools", "aiohttp", "numpy"], zip_safe=True,
    maintainer="Project LAKSA", maintainer_email="project-laksa@invalid.local",
    description="Mobile-first, mapping-only LAKSA cockpit.", license="Apache-2.0",
    entry_points={"console_scripts": ["cockpit_server = laksa_dashboard.cockpit_server:main"]},
)
