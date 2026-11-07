"""The robot's tunable values as described ROS 2 parameters in the Python nodes (design.md 2.1), as omni_cpp/params.hpp
does in C++: each with a description, a range for numbers, and read-only (set at start) or live. ROS refuses an
out-of-range or read-only `ros2 param set`; a live one calls its setter at once. Values: omni_bringup/config/robot.yaml.

    from omni_params import Params
    p = Params(node)
    rate = p.number("rate", 5.0, "Hz: depth frames used", (0.1, 30.0))                 # read-only
    self.radius = p.number("forget_radius", 0.5, "m: ...", (0.0, 3.0), live=self, attr="forget_r")
    topic = p.text("depth_topic", "/d435/depth/image_rect_raw", "the depth image")
"""

from __future__ import annotations

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


class Params:
    def __init__(self, node):
        self.node = node
        self._setters = {}
        node.add_post_set_parameters_callback(self._on_set)

    def _on_set(self, params):
        for p in params:                          # already range- and read-only-checked by ROS
            fn = self._setters.get(p.name)
            if fn is not None:
                fn(p.value)

    def _declare(self, name, default, text, read_only, rng=None, setter=None):
        d = ParameterDescriptor(description=text, read_only=read_only)
        if rng is not None:
            if isinstance(default, bool):
                raise TypeError(f"{name}: a flag has no range")
            if isinstance(default, int):
                d.integer_range = [IntegerRange(from_value=int(rng[0]), to_value=int(rng[1]), step=1)]
            else:
                d.floating_point_range = [FloatingPointRange(from_value=float(rng[0]), to_value=float(rng[1]),
                                                             step=0.0)]
        value = self.node.declare_parameter(name, default, d).value
        if setter is not None:
            self._setters[name] = setter
        return value

    def number(self, name, default, text, rng, live=None, attr=None, convert=None):
        """A float (default a float) or an integer (default an int) in rng = (lo, hi). live=obj: a set writes
        obj.<attr> (attr defaults to name; convert(value) first if given), and the parameter is not read-only."""
        setter = None
        if live is not None:
            a, conv = attr or name, convert or (lambda v: v)
            setter = lambda v: setattr(live, a, conv(v))    # noqa: E731
        value = self._declare(name, default, text, live is None, rng, setter)
        return convert(value) if convert else value

    def flag(self, name, default: bool, text, live=None, attr=None):
        setter = (lambda v: setattr(live, attr or name, bool(v))) if live is not None else None
        return self._declare(name, bool(default), text, live is None, None, setter)

    def text(self, name, default: str, text):
        return self._declare(name, default, text, True)

    def array(self, name, default: list, text):
        return self._declare(name, default, text, True)

    def on_set(self, name, fn):
        """A custom live setter for a parameter declared live (e.g. two parameters feeding one tuple)."""
        self._setters[name] = fn
