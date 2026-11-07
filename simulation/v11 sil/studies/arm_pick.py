"""The tidy task, scored (design.md 11, Oct 24): sil_pick.SilPick over seeds and object sets: the robot's own code
(omni_arm's driver, floor and wrist vision, the tidy task; the mode mux; the C++ firmware) against the world.

Each run: n things from a set scattered within 1.2 m of a point 0.9 m ahead of the robot, the servos off their
calibration by 1 deg rms, the left camera by 0.3 deg; the task runs until it finds nothing more (or t_max). Scored on
the truth: how many ended in the bin, the time per object, what the task said went wrong.

  python arm_pick.py                 seeds 0-7 x {toys, clutter} x {arena, house}, 5 objects each (~20 min, 12 cores)
  python arm_pick.py --seeds 4 --n 3 --worlds house
  python arm_pick.py --camera stereo        the parked catch robot's stereo pair (the Oct results: arm_pick.json)
Writes results/arm_pick_d435.json (the D435, the default) or results/arm_pick.json (stereo).
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import save  # noqa: E402


def one(args):
    n, objects, seed, t_max, world, camera = args
    from sil_pick import SilPick
    g = SilPick(n, objects, seed, world=world, **(dict(depth_camera="d435") if camera == "d435" else {}))
    s = g.run(t_max)
    s.update(seed=seed, set=objects, names=g.names, world=world)
    g.cams.close()
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--t-max", type=float, default=420.0)
    ap.add_argument("--sets", default="toys,clutter")
    ap.add_argument("--worlds", default="arena,house", help="arena: a bare wooden floor; house: the v7 house's "
                                                            "hallway and living room (rugs, a runner)")
    ap.add_argument("--camera", default="d435", choices=("d435", "stereo"),
                    help="d435: the arm robot's only camera, on the riser (design.md 4.11; results arm_pick_d435.json); "
                         "stereo: the parked catch robot's pair (the earlier results, arm_pick.json)")
    ap.add_argument("--workers", type=int, default=0, help="0: all cores but one")
    a = ap.parse_args()
    jobs = [(a.n, s, k, a.t_max, w, a.camera) for w in a.worlds.split(",") for s in a.sets.split(",")
            for k in range(a.seeds)]
    import sil_robot  # noqa: F401  (builds the firmware's SIL library here, once, before the workers load it)
    from omni_fw_sil import Firmware
    Firmware().close()
    with ProcessPoolExecutor(max_workers=min(len(jobs), a.workers or max(1, (os.cpu_count() or 2) - 1))) as ex:
        runs = list(ex.map(one, jobs))
    total = sum(r["objects"] for r in runs)
    binned = sum(r["in_bin"] for r in runs)
    why = Counter()
    for r in runs:
        for _, state, msg in r["events"]:
            for key in ("missed", "dropped", "no grasp", "not there", "too big", "no clear path", "no straight way"):
                if key in msg:
                    why[key] += 1
    t_obj = sum(r["t"] for r in runs) / max(1, binned)
    print(f"{binned}/{total} in the bin ({100 * binned / max(1, total):.0f}%), {t_obj:.0f} s of sim per object picked")
    for w in sorted({r["world"] for r in runs}):
        for s in sorted({r["set"] for r in runs}):
            rr = [r for r in runs if r["world"] == w and r["set"] == s]
            print(f"  {w:6s} {s:8s}: {sum(r['in_bin'] for r in rr)}/{sum(r['objects'] for r in rr)}")
    for r in runs:
        print(f"  {r['world']:6s} {r['set']:8s} seed {r['seed']}: {r['in_bin']}/{r['objects']} in {r['t']:.0f} s "
              f"(task: {r['task_picked']} picked, {r['task_failed']} failed, {r['task_skipped']} skipped; {r['reason']})")
    print("what went wrong (events):", dict(why))
    save("arm_pick_d435" if a.camera == "d435" else "arm_pick", dict(camera=a.camera, runs=runs, total=total, in_bin=binned, rate=binned / max(1, total), t_per_object=t_obj,
                          why=dict(why), n=a.n, seeds=a.seeds))


if __name__ == "__main__":
    main()
