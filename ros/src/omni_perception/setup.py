from setuptools import setup

package_name = "omni_perception"

setup(
    name=package_name,
    version="0.10.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando",
    description="omni-robot scan preparation (design.md 28)",
    license="Proprietary",
    entry_points={"console_scripts": ["image_preview = omni_perception.image_preview:main",
                                    "people = omni_perception.people_node:main"]},
)
