"""One Robot call from the shell, on the running server's session (server.py, or server.py --hold): what an AI
session without the MCP tools (or a shell script) uses. Prints the reply as JSON; a picture goes to --img.

    python call.py status
    python call.py map_view --img map.png
    python call.py goto '{"x": 2.0, "y": 1.5, "yaw_deg": 90}'
    python call.py task '{"task_id": "t-0007", "wait_s": 30}'
    python call.py look '{"camera": "front"}' --img front.jpg
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from server import Robot  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("method")
    ap.add_argument("args", nargs="?", default="{}", help="keyword arguments as a JSON object")
    ap.add_argument("--img", help="where a picture goes (png or jpg, as the call returns it)")
    a = ap.parse_args()
    robot = Robot.attach()
    try:
        robot.rb.wait_up(5.0)
        if a.method in ("status", "map_view", "scan", "people", "look", "capabilities"):
            robot.rb.newest("/console/status")
            import time
            t_end = time.monotonic() + 3.0
            while robot.status_raw()[0] is None and time.monotonic() < t_end:
                time.sleep(0.1)
        r = getattr(robot, a.method)(**json.loads(a.args))
        img = None
        if isinstance(r, tuple):
            r, img = r
        if img is not None and a.img:
            Path(a.img).write_bytes(img)
            r = dict(r, image=a.img)
        print(json.dumps(r, indent=1, default=str))
    finally:
        robot.close()


if __name__ == "__main__":
    main()
