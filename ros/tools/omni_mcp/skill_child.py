"""A skill's own process (skillbox.py starts it as `python -I skill_child.py`): restricted Python with one `robot`
object. Standard library only, and only json/sys/math/random/traceback of it: nothing of the repo is importable here.

Protocol, JSON lines: stdin's first line {"code", "args"}; then for each robot call this writes {"call": name, "args":
{...}} and reads {"ok": value} | {"error": text} | {"stop": reason}; at the end it writes {"done": result} or
{"failed": text} or {"stopped": reason}.

The skill can't reach the parent's objects, files, the network or ROS: it gets a short list of builtins, imports only
math and random, and skillbox.check() (run again here) refuses names and attributes that start with "_", exec / eval
/ open and the like. That keeps out the ordinary ways round it; it is not a hardened sandbox, and the robot's safety
never depends on it: every action is a console verb under the AI lease, the speed caps, the collision monitor and a
person's stop.
"""

import json
import math
import random
import sys
import traceback

ALLOWED_IMPORTS = ("math", "random")


class Stopped(BaseException):
    """A person stopped the robot (or the brain ended the skill): not catchable by `except Exception`."""


class RobotError(Exception):
    """A robot call was refused or failed (the message says why)."""


_out = sys.stdout
_in = sys.stdin


def _send(obj):
    _out.write(json.dumps(obj, default=str) + "\n")
    _out.flush()


def _rpc(name, args):
    _send({"call": name, "args": args})
    line = _in.readline()
    if not line:
        raise Stopped("the brain went away")
    r = json.loads(line)
    if "stop" in r:
        raise Stopped(r["stop"])
    if "error" in r:
        raise RobotError(r["error"])
    return r.get("ok")


class Robot:
    """The skill's robot: each method is a call the parent runs (skillbox.API documents them)."""

    def __init__(self, names):
        self._names = set(names)

    def __getattr__(self, name):
        if name.startswith("_") or name not in self._names:
            raise AttributeError(f"robot has no '{name}' (it has: {', '.join(sorted(self._names))})")

        def call(*a, **kw):
            if a:
                raise TypeError(f"robot.{name}: use keyword arguments (robot.{name}(name=value))")
            return _rpc(name, kw)
        return call


def _import(name, globals=None, locals=None, fromlist=(), level=0):
    if level or name not in ALLOWED_IMPORTS:
        raise ImportError(f"skills may import only {', '.join(ALLOWED_IMPORTS)}")
    return {"math": math, "random": random}[name]


SAFE = ("abs", "all", "any", "bool", "dict", "divmod", "enumerate", "filter", "float", "int", "isinstance", "len",
        "list", "map", "max", "min", "pow", "range", "reversed", "round", "set", "sorted", "str", "sum", "tuple",
        "zip", "next", "iter", "repr", "format", "chr", "ord", "Exception", "ValueError", "TypeError", "KeyError", "IndexError", "ZeroDivisionError", "RuntimeError",
        "True", "False", "None")


def main():
    import builtins
    req = json.loads(_in.readline())
    code, args = req["code"], req.get("args") or {}
    sys.path.insert(0, req.get("check_dir", ""))
    from skill_check import problems                    # the parent's check, run again here
    del sys.path[0]
    bad = problems(code)
    if bad:
        _send({"failed": "refused: " + "; ".join(bad)})
        return
    safe = {k: getattr(builtins, k) for k in SAFE if hasattr(builtins, k)}
    robot = Robot(req.get("api") or [])
    safe["__import__"] = _import
    safe["print"] = lambda *a, **k: _rpc("log", {"text": " ".join(str(x) for x in a)})
    g = {"__builtins__": safe, "__name__": "skill", "RobotError": RobotError}
    try:
        exec(compile(code, "<skill>", "exec"), g)
        if "run" not in g:
            raise RuntimeError("the skill has no run(robot, ...)")
        result = g["run"](robot, **args)
        _send({"done": result})
    except Stopped as e:
        _send({"stopped": str(e)})
    except BaseException as e:                              # noqa: BLE001  (the skill's error, reported)
        tb = traceback.extract_tb(e.__traceback__)
        where = next((f"line {f.lineno}" for f in reversed(tb) if f.filename == "<skill>"), "")
        _send({"failed": f"{type(e).__name__}: {e}" + (f" ({where})" if where else "")})


if __name__ == "__main__":
    main()
