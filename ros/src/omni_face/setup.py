from setuptools import setup

package_name = "omni_face"

setup(
    name=package_name,
    version="0.11.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/systemd", ["systemd/omni-face.service"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando",
    description="omni-robot face and voice: the 7 in display and the speaker (design.md 2)",
    license="Proprietary",
    entry_points={"console_scripts": ["face_node = omni_face.face_node:main",
                                      "face_display = omni_face.display:main"]},
)
