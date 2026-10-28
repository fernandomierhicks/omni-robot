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
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Fernando",
    description="omni-robot: the SO-101 arm, its servo bus and the tidy task (design.md 11)",
    license="Proprietary",
    entry_points={"console_scripts": [
        "arm_node = omni_arm.arm_node:main",
    ]},
)
