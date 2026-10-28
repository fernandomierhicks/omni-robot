from setuptools import setup

package_name = "omni_slam"

setup(
    name=package_name,
    version="0.11.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando",
    description="omni-robot AprilTag landmark SLAM and localization health (design.md 10)",
    license="Proprietary",
    entry_points={"console_scripts": ["tag_slam = omni_slam.tag_slam_node:main",
                                      "tag_detector = omni_slam.tag_detector:main"]},
)
