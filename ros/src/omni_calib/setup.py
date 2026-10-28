from setuptools import setup

package_name = "omni_calib"

setup(
    name=package_name,
    version="0.11.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando",
    description="omni-robot calibration solvers and checks (design.md 10)",
    license="Proprietary",
    entry_points={"console_scripts": ["calib_report = omni_calib.cli:main"]},
)
