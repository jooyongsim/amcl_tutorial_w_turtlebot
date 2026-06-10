import os
from glob import glob

from setuptools import find_packages, setup

package_name = "amcl_nav_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch / config / map / rviz files into the package share dir.
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="course staff",
    maintainer_email="aykim515@gmail.com",
    description="AMCL + Nav2 navigation tutorial (using packages).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run amcl_nav_pkg waypoint_navigator
            "waypoint_navigator = amcl_nav_pkg.waypoint_navigator:main",
        ],
    },
)
