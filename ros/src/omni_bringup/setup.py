from glob import glob

from setuptools import setup

package_name = "omni_bringup"

setup(
    name=package_name,
    version="0.10.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando",
    description="omni-robot launch + config (design.md 28)",
    license="Proprietary",
    entry_points={"console_scripts": ["save_map = omni_bringup.save_map:main"]},
)
