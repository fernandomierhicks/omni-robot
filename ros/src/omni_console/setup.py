import os

from setuptools import setup

package_name = "omni_console"


def web_files():
    """web/ and its subfolders, installed under share/omni_console/web (the page console_web serves)."""
    out = []
    for root, _, files in os.walk("web"):
        if files:
            out.append((os.path.join("share", package_name, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/robot_steps.json"]),
        ("share/" + package_name + "/systemd", ["systemd/omni-console.service", "systemd/README.md"]),
    ] + web_files(),
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Fernando",
    description="omni-robot: the operator console (mission switching, settings, status, the web page)",
    license="Proprietary",
    entry_points={"console_scripts": [
        "mission_node = omni_console.mission_node:main",
        "console_web = omni_console.web_server:main",
        "supervisor = omni_console.supervisor:main",
    ]},
)
