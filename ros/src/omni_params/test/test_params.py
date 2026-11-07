"""omni_params with a stand-in node (no ROS graph needed): read-only by default, ranges by type, live setters."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

rcl = pytest.importorskip("rcl_interfaces.msg")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from omni_params import Params  # noqa: E402


class _P:
    def __init__(self, name, value):
        self.name, self.value = name, value


class FakeNode:
    def __init__(self, overrides=None):
        self.decl, self.cb, self.over = {}, None, overrides or {}

    def add_post_set_parameters_callback(self, cb):
        self.cb = cb

    def declare_parameter(self, name, default, desc):
        self.decl[name] = (default, desc)
        return _P(name, self.over.get(name, default))


def test_ranges_follow_the_default_type_and_read_only_is_the_default():
    n = FakeNode({"rate": 7.5})
    p = Params(n)
    assert p.number("rate", 5.0, "Hz", (0.1, 30.0)) == 7.5
    assert p.number("width", 320, "px", (16, 4096)) == 320
    d_rate, d_w = n.decl["rate"][1], n.decl["width"][1]
    assert d_rate.read_only and d_rate.floating_point_range[0].to_value == 30.0 and not d_rate.integer_range
    assert d_w.integer_range[0].from_value == 16 and not d_w.floating_point_range


def test_a_live_parameter_writes_its_attribute():
    class Obj:
        pass

    n, o = FakeNode(), Obj()
    p = Params(n)
    o.r = p.number("forget_radius", 0.5, "m", (0.0, 3.0), live=o, attr="r")
    assert not n.decl["forget_radius"][1].read_only
    n.cb([_P("forget_radius", 1.25)])
    assert o.r == 1.25
