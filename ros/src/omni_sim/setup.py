from setuptools import setup

package_name = "omni_sim"

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
    description="omni-robot MuJoCo <-> ROS 2 bridge and scoring (design.md 28, 30)",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "bridge = omni_sim.bridge_node:main",
            "catch_bridge = omni_sim.catch_bridge:main",
            "catch_scorer = omni_sim.catch_scorer:main",
            "scorer = omni_sim.scorer:main",
            "score_map = omni_sim.score_map:main",
            "nav_goals = omni_sim.nav_goals:main",
            "joy_relay = omni_sim.joy_relay:main",
        ],
    },
)
