"""config/robot.yaml against the nodes that read it: every value names a parameter its node declares, with the type the
node declares (a double as a float, an integer as an int) and inside the node's declared range. No ROS needed: the
declarations are read from the sources (omni_cpp's params::number / integer / flag / text, arm_brain's P(...)).
    python -m pytest -q ros/src/omni_bringup/test
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

SRC = Path(__file__).resolve().parents[2]
ROBOT_YAML = SRC / "omni_bringup" / "config" / "robot.yaml"
NODES = {
    "mode_manager": SRC / "omni_cpp" / "src" / "mode_manager_node.cpp",
    "mcu_link": SRC / "omni_cpp" / "src" / "mcu_link_node.cpp",
    "scan_prep": SRC / "omni_cpp" / "src" / "scan_prep_node.cpp",
    "heading_node": SRC / "omni_cpp" / "src" / "heading_node.cpp",
    "arm_driver": SRC / "omni_cpp" / "src" / "arm_driver_node.cpp",
    "catch_perception": SRC / "omni_cpp" / "src" / "catch_perception_node.cpp",
    "catch_behavior": SRC / "omni_cpp" / "src" / "catch_behavior_node.cpp",
    "arm_node": SRC / "omni_arm" / "omni_arm" / "arm_brain.py",
    "floor_obstacles": SRC / "omni_arm" / "omni_arm" / "floor_obstacles_node.py",
    "height_map": SRC / "omni_arm" / "omni_arm" / "height_map_node.py",
    "tag_slam": SRC / "omni_slam" / "omni_slam" / "tag_slam_node.py",
    "tag_detector": SRC / "omni_slam" / "omni_slam" / "tag_detector.py",
    "omni_face": SRC / "omni_face" / "omni_face" / "face_node.py",
    "image_preview": SRC / "omni_perception" / "omni_perception" / "image_preview.py",
    "people": SRC / "omni_perception" / "omni_perception" / "people_node.py",
}
NUM = r"(-?[0-9.eE+-]+|[A-Z_]+|c\.[a-z_]+)"


def declarations(path: Path) -> dict:
    """{name: (kind, lo, hi)}: kind number | integer | flag | text | array; lo / hi None when not given."""
    s = re.sub(r"\s+", " ", path.read_text())
    out = {}
    for kind, name, rest in re.findall(r"\b(?:P|params)::(number|integer|flag|text)\((?:\*this|n), \"(\w+)\", ([^;]*)", s):
        m = re.match(rf"[^,]+, {NUM}, {NUM},", rest) if kind in ("number", "integer") else None
        lo, hi = (float(m.group(1)), float(m.group(2))) if m and re.match(r"-?[0-9]", m.group(1)) else (None, None)
        out[name] = (kind, lo, hi)
    for name in re.findall(r"declare_parameter<std::vector<[^>]+>>\( ?\"(\w+)\"", s):
        out[name] = ("array", None, None)
    # the Python nodes: omni_params' p.number(name, default, text, (lo, hi), ...) / p.flag / p.text / p.array
    for kind, name, default, rest in re.findall(r"\bp\.(number|flag|text|array)\(\"(\w+)\", ([^,]+), ([^;]*?)\)\s*(?:[#\n]|$|\w)", s):
        m = re.search(r"\((-?[0-9.]+), (-?[0-9.]+)\)", rest) if kind == "number" else None
        if kind == "number":
            is_int = re.fullmatch(r"-?[0-9]+", default.strip()) is not None
            kind = "integer" if is_int else "number"
        out[name] = (kind, float(m.group(1)) if m else None, float(m.group(2)) if m else None)
    return out


def test_every_robot_parameter_is_declared_with_its_type_and_range():
    values = yaml.safe_load(ROBOT_YAML.read_text())
    assert set(values) <= set(NODES), f"robot.yaml names nodes this test doesn't know: {set(values) - set(NODES)}"
    for node, section in values.items():
        decl = declarations(NODES[node])
        for name, v in section["ros__parameters"].items():
            assert name in decl, f"{node}: robot.yaml sets {name}, which the node doesn't declare"
            kind, lo, hi = decl[name]
            if kind == "number":
                assert isinstance(v, float), f"{node}.{name}: a double parameter needs a float ({v!r})"
            elif kind == "integer":
                assert isinstance(v, int) and not isinstance(v, bool), f"{node}.{name}: an integer ({v!r})"
            elif kind == "flag":
                assert isinstance(v, bool), f"{node}.{name}: true / false ({v!r})"
            if lo is not None:
                assert lo <= v <= hi, f"{node}.{name} = {v} is outside the node's range [{lo}, {hi}]"


def test_the_launch_loads_robot_yaml_into_each_robot_node():
    stack = (SRC / "omni_bringup" / "omni_bringup" / "stack.py").read_text()
    assert stack.count("robot_params()") >= 15
