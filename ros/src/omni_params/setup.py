from setuptools import setup

package_name = "omni_params"

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
    description="omni-robot: described ROS 2 parameters for the Python nodes (omni_cpp/params.hpp's twin)",
    license="Proprietary",
)
