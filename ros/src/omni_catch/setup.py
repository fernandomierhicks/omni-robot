from setuptools import setup

package_name = "omni_catch"

setup(
    name=package_name,
    version="0.11.0",
    packages=[package_name],
    package_data={package_name: ["config/*.json", "native/*.cpp"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Fernando",
    description="omni-robot: the catch path, the play-catch behavior and robot modes (design.md 30)",
    license="Proprietary",
    entry_points={"console_scripts": [
        "catch_perception = omni_catch.perception_node:main",
        "catch_behavior = omni_catch.behavior_node:main",
        "mode_manager = omni_catch.mode_node:main",
    ]},
)
