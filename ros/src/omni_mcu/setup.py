from setuptools import setup

package_name = "omni_mcu"

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
    description="omni-robot: the robot computer's end of the MCU link (design.md 30)",
    license="Proprietary",
    # mcu_link is omni_cpp's (C++); this package keeps the protocol and clock map the sim and the console use
    entry_points={"console_scripts": []},
)
