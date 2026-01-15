import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sala4"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gary",
    maintainer_email="pinales@usc.edu",
    description="TODO: Package description",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapper_multiranger = sala4.mapper_multiranger:main",
            "wall_following_multiranger = sala4.wall_following_multiranger:main",
            "arming = sala4.arming:main",
            "waypoint_following = sala4.waypoint_following:main",
            "trajectory_following = sala4.trajectory_following:main",
        ],
    },
)
