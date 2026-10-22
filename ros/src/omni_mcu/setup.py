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
    description="omni-robot: the Jetson's end of the MCU link (design.md 30)",
    license="Proprietary",
    entry_points={"console_scripts": ["mcu_link = omni_mcu.link_node:main"]},
)
