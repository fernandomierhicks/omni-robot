"""The bench camera head's robot_config: omni_catch's robot_v11.json with its stereo pair replaced by this one camera.

catch_perception projects the ball through the config's cameras (pose on the robot, vertical FOV, image size). The
bench has one Camera Module 3 (IMX708) at the 1536x864 mode, a 3072x1728 crop of the sensor (1.4 um pixels:
4.30 x 2.42 mm), scaled by the ISP to the published size; with the standard lens (f 4.74 mm) that is 28.6 deg
vertical, 48.8 horizontal (est: from the datasheet, not calibrated; the Wide module needs its own --fovy). It keeps the
robot's left camera pose, so tracks are in the robot's frame as if the module were mounted there. With one camera
catch_perception processes each frame at once; the tracker fits image position and ball size (no stereo).

On the Pi (pi.sh head runs it):  python3 pi_camera/bench_config.py [--width 768 --height 432 --fovy 28.6]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CROP_V_MM, F_MM = 1728 * 1.4e-3, 4.74          # the 120 fps mode's crop height, Camera Module 3 standard lens


def robot_v11() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("omni_catch")) / "config" / "robot_v11.json"
        if p.is_file():
            return p
    except Exception:                                              # no ROS sourced: the repo's or the workspace's
        pass
    for p in (HERE.parent / "src" / "omni_catch" / "omni_catch" / "config" / "robot_v11.json",
              HERE.parents[1] / "ros" / "src" / "omni_catch" / "omni_catch" / "config" / "robot_v11.json"):
        if p.is_file():
            return p
    raise SystemExit("robot_v11.json not found (source the workspace)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--width", type=int, default=768)
    ap.add_argument("--height", type=int, default=432)
    ap.add_argument("--fovy", type=float, default=round(math.degrees(2 * math.atan(CROP_V_MM / 2 / F_MM)), 1))
    ap.add_argument("--out", default=str(HERE / "robot_bench.json"))
    a = ap.parse_args()
    src = robot_v11()
    cfg = json.loads(src.read_text())
    left = next(c for c in cfg["cameras"] if c.get("name") == "left")
    cfg["cameras"] = [dict(left, fovy_deg=a.fovy, width=a.width, height=a.height,
                           note="bench: Camera Module 3 (IMX708) at the robot's left camera pose, FOV est")]
    cfg["generated_by"] = f"characterization/pi_camera/bench_config.py from {src}"
    Path(a.out).write_text(json.dumps(cfg, indent=1))
    print(f"bench config: 1 camera {a.width}x{a.height}, fovy {a.fovy} deg (est) -> {a.out}")


if __name__ == "__main__":
    main()
