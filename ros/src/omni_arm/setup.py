from glob import glob

from setuptools import setup

package_name = "omni_arm"

setup(
    name=package_name,
    version="0.11.0",
    packages=[package_name],
    package_data={package_name: ["config/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # the robot config for omni_cpp's arm_driver too
        ("share/" + package_name + "/config", glob("omni_arm/config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Fernando",
    description="omni-robot: the SO-101 arm, its servo bus and the tidy task (design.md 11)",
    license="Proprietary",
    entry_points={"console_scripts": [
        "arm_brain = omni_arm.arm_brain:main",
        "floor_obstacles = omni_arm.floor_obstacles_node:main",
        "height_map = omni_arm.height_map_node:main",
    ]},
)
