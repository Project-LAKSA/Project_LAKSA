from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "laksa_speed_race"
ROOT = Path(__file__).resolve().parent


def course_data_files():
    files = []
    for path in sorted((ROOT / "course").rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts:
            files.append((str(Path("share") / PACKAGE_NAME / path.parent.relative_to(ROOT)), [str(path)]))
    return files


def directory_data_files(directory):
    return [
        (str(Path("share") / PACKAGE_NAME / directory), [str(path)])
        for path in sorted((ROOT / directory).glob("*"))
        if path.is_file()
    ]


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/laksa_speed_race"]),
        (str(Path("share") / PACKAGE_NAME), ["package.xml", "README.md", "ARCHITECTURE.md", "SIM_ASSUMPTIONS.json", "SPEED_RACE_RESULTS.json", "UPSTREAM_PROVENANCE.md", "speed_race_upstream.repos"]),
        *directory_data_files("config"),
        *directory_data_files("launch"),
        *course_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Leobardo Gomez",
    maintainer_email="luisleo181196@hotmail.com",
    description="Simulation-only LAKSA Speed Course contracts and canonical assets.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "validate_speed_course = laksa_speed_race.course_validation:main",
            "c1_gym_adapter = laksa_speed_race.gym_adapter_node:main",
        ],
    },
)
