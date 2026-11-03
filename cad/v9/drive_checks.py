"""Focused wheel / pod checks, also included in build.py's full report."""
from __future__ import annotations

import json
from pathlib import Path

import assembly as A
import build as B
import dims as D
import pod as P
import wheel as W


def report():
    robot = A.build(0, 0, "pods")
    own = [it for it in robot.items if it.group in ("pod0", "pod0u")]
    print("drive: static clashes", flush=True)
    out = {"clashes_static": B.clashes(own)}
    print("drive: thread engagement", flush=True)
    out["short"] = [r for r in B.engagement(own) if not r["ok"]]
    print("drive: screw access", flush=True)
    out["access"] = [r for r in B.access(own) if r["problems"]]
    out["solids"] = B.solids(robot)
    out["sweep"] = {}
    for bump in (-3, 3, 6, 9, 12, 15):
        print(f"drive: bump {bump}", flush=True)
        items = [it for it in A.build(0, bump, "pods").items if it.group in ("pod0", "pod0u")]
        out["sweep"][str(bump)] = B.clashes(items, only=lambda a, b: a.group != b.group)
    out["axle_height_error_mm"] = max(abs(P.arm_motion(b).position.Z - b) for b in (-3, 0, 15))
    out["mount_depth_mm"] = {
        "output": 6 - (D.X_DRIVE_PLATE[1] - D.X_DRIVE_PLATE[0]),
        "rear": 8 - (D.X_REAR_BRIDGE[1] - D.X_REAR_BRIDGE[0]),
    }
    out["dimensions_mm"] = {"wheel_diameter": 2 * D.WHEEL_R, "row_pitch": D.ROW_X[1] - D.ROW_X[0],
                            "base_bumper_diameter": 2 * (D.DECK_R + D.BUMPER_T),
                            "lidar_scan_z": D.LIDAR_SCAN_Z, "payload_top": D.PLATE_TOP}
    out["speed_ratio_vs_127"] = 2 * D.WHEEL_R / 127
    return out


if __name__ == "__main__":
    result = report()
    path = Path(__file__).with_name("out") / "drive_checks.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
