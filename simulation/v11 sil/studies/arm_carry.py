"""Carry, put down and stack (the console's Fetch and Block tower), scored on the truth: sil_pick.SilPick with the
robot's own code (omni_arm's tidy task with keep / start_place / tower, its floor and wrist vision, the C++ arm driver
on the servo bus, the mode mux, the C++ firmware), the D435 on the riser.

  fetch   one thing picked and kept (outcome held), then put down "ahead": scored on where it ended (on the floor, not
          in the bin, how far from the sweet spot ahead of the robot) and that it stayed upright
  tower   blocks (40 mm cubes) scattered 0.6 m round a point ahead: the first block found is the base, each other one
          goes on top: scored on how many stand stacked (each within 25 mm of the one below, its bottom on its top)

  python studies/arm_carry.py                     both, seeds 0-1 (MUJOCO_GL=osmesa without a display)
  python studies/arm_carry.py --case tower --seeds 3 --blocks 3
Writes results/<out>.json (--out; arm_carry by default): the Nov 7 runs are arm_carry_fetch.json and arm_carry_tower.json.
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import save  # noqa: E402


def _run(g, until, t_max):
    while not until() and g.d.time < t_max:
        g.step()


def world_objects(g):
    """Each object's world position, and how upright it stands (its z axis's z: 1 upright, a cube on any face too)."""
    import arm_objects
    P = arm_objects.positions(g.m, g.d)
    up = []
    for k in range(len(P)):
        Rk = g.d.xmat[g.m.body(f"obj{k}").id].reshape(3, 3)
        up.append(float(np.max(np.abs(Rk[2]))))             # a cube on any of its faces counts as standing
    return np.asarray(P), up


def fetch(seed: int, t_max: float = 300.0) -> dict:
    from sil_pick import SilPick
    g = SilPick(1, "blocks", seed, spread=0.5, depth_camera="d435")
    task = g.task
    task.start(g.d.time, keep=True)
    _run(g, lambda: task.state == "DONE", t_max)
    held = task.outcome
    r = dict(case="fetch", seed=seed, held=held, t_pick=round(g.d.time, 1))
    if held != "held":
        r.update(ok=False, why=task.done_reason, events=task.events[-6:])
        return r
    # where the robot stands: the sweet spot ahead (world), to compare with where it ends
    b = g.m.body("base").id
    R, pb = g.d.xmat[b].reshape(3, 3), g.d.xpos[b].copy()
    sx, sy = task.p.sweet
    want = pb + R @ np.array([sx, sy, 0.0])
    t0 = g.d.time
    task.start_place(g.d.time, ahead=True)
    _run(g, lambda: task.state == "DONE", t0 + 120.0)
    _run(g, lambda: False, g.d.time + 2.0)                    # let it settle
    P, up = world_objects(g)
    in_bin = g.in_bin()[0]
    err = float(np.hypot(*(P[0][:2] - want[:2])))
    r.update(outcome=task.outcome, in_bin=in_bin, on_floor=bool(P[0][2] < 0.03), upright=bool(up[0] > 0.95),
             place_err_mm=round(1000 * err, 1), t=round(g.d.time - t0, 1),
             ok=bool(task.outcome == "placed" and not in_bin and P[0][2] < 0.03 and err < 0.05))
    return r


def tower(seed: int, n: int = 3, t_max: float = 900.0) -> dict:
    from sil_pick import SilPick
    g = SilPick(n, "blocks", seed, spread=0.6, depth_camera="d435")
    task = g.task
    task.start(g.d.time, tower=True, max_blocks=n)
    _run(g, lambda: task.state == "DONE", t_max)
    _run(g, lambda: False, g.d.time + 2.0)
    P, up = world_objects(g)
    # the stack on the truth: the tallest column, from any block on the floor up, each on the one below
    side = 0.04

    def column(k0):
        st = [k0]
        for _ in range(len(P)):
            top = P[st[-1]]
            nxt = [k for k in range(len(P)) if k not in st and abs(P[k][2] - top[2] - side) < 0.012
                   and np.hypot(*(P[k][:2] - top[:2])) < 0.025]
            if not nxt:
                break
            st.append(nxt[0])
        return st

    stack = max((column(k) for k in range(len(P)) if P[k][2] < 0.03), key=len, default=[])
    offs = [round(1000 * float(np.hypot(*(P[stack[i + 1]][:2] - P[stack[i]][:2]))), 1) for i in range(len(stack) - 1)]
    return dict(case="tower", seed=seed, blocks=n, stacked=len(stack), task_n=None if task.tower is None else task.tower["n"],
                outcome=task.outcome, reason=task.done_reason, offsets_mm=offs, upright=[round(u, 3) for u in up],
                in_bin=int(sum(g.in_bin())), t=round(g.d.time, 1), ok=len(stack) == n,
                blocks_odom=[[round(float(v), 3) for v in (*g.odom_xy(P[k:k + 1])[0], P[k][2])] for k in range(len(P))],
                events=[e[2] for e in task.events if e[1] in ("P_APPROACH", "P_CREEP", "STOW", "DONE", "LOOK")][-12:])


def one(args):
    """One run; tried again once when the renderer fails (osmesa returned a garbage segmentation image once under
    memory pressure: a crash of the sim's camera, not of the robot's code), else its error as the result."""
    case, seed, n = args
    err = None
    for _ in range(2):
        try:
            return fetch(seed) if case == "fetch" else tower(seed, n)
        except Exception as e:                       # noqa: BLE001
            err = f"{type(e).__name__}: {e}"
    return dict(case=case, seed=seed, ok=False, error=err)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", choices=("fetch", "tower", "both"), default="both")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--blocks", type=int, default=3)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--first", type=int, default=0, help="the first seed")
    ap.add_argument("--out", default="arm_carry", help="the results' name (studies/results/<out>.json)")
    a = ap.parse_args()
    cases = ["fetch", "tower"] if a.case == "both" else [a.case]
    jobs = [(c, s, a.blocks) for c in cases for s in range(a.first, a.first + a.seeds)]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(one, jobs))
    for r in res:
        print({k: v for k, v in r.items() if k != "events"})
        if not r.get("ok") and r.get("events"):
            print("   ", r["events"])
    save(a.out, dict(runs=res, ok=sum(r.get("ok", False) for r in res), n=len(res)))


if __name__ == "__main__":
    main()
