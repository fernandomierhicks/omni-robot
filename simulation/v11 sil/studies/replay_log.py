"""Replay a session log's throws in the sim (design.md 5: "the gap between simulated and real catch rate for the same
settings" is the trust metric once hardware exists; design.md 7: one log format for the robot and the sim).

A session log (omni-log/1, ros/src/omni_console/omni_console/session_log.py) from the robot, or from the sim
(sil_game.py --log), lists each throw as the robot tracked it: the ball's state when the catch plan froze (statef:
position and velocity, odom frame, the filter having seen most of the flight; the track's first state, state0, is too
noisy in depth at 6 m to start from) and whether it caught it. Each throw is integrated back to the thrower's hand
(release height, with the
ball's drag), placed in the sim's field (the robot's odom frame starts at catch_world's HOME, facing the thrower), and
thrown again at the robot with its real code in the loop (sil_game.SilCatchGame). Spin isn't measured: the replay
throws without it (and again with random spin with --spin).

    python replay_log.py <log.jsonl> [--seed 9000] [--spin] [--out results/replay_<name>.json]
    python replay_log.py --demo              a sim session (6 throws), then its replay: the machinery checked end to end

Prints, per throw, caught on the robot (the log) and in the sim, and the totals: the sim-vs-real gap.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from catch_world import CatchWorld, WorldConfig  # noqa: E402
from sil_game import SilCatchGame  # noqa: E402
from omni_console import session_log  # noqa: E402

RELEASE_HEIGHT = 1.0


def back_to_release(flight, pos, vel, z_release: float = RELEASE_HEIGHT, dt: float = 0.002, t_max: float = 3.0):
    """Integrate the ball backward (drag included) until it last rose through z_release: (pos, vel) there, or the
    state as given if it never was that low (a throw tracked from very early)."""
    p, v = np.array(pos, dtype=float), np.array(vel, dtype=float)
    if p[2] <= z_release:
        return p, v
    t = 0.0
    while t < t_max:
        a = flight.accel(v)
        v_prev = v - a * dt
        p_prev = p - 0.5 * (v + v_prev) * dt
        if p_prev[2] <= z_release:
            f = (p[2] - z_release) / max(p[2] - p_prev[2], 1e-9)
            return p - f * (p - p_prev), v - f * (v - v_prev)
        p, v, t = p_prev, v_prev, t + dt
    return p, v


def scripted_throws(throws, world_from_odom, flight, rng=None, spin_sigma=0.0) -> list:
    """The log's throws as catch_world scripted throws (world frame)."""
    x0, y0, a = world_from_odom
    c, s = math.cos(a), math.sin(a)
    out = []
    for r in throws:
        st = r.get("statef") or r.get("state0")      # the frozen plan's state: the filter had the whole flight
        if not st or any(v is None for v in st):
            continue
        p = np.array([x0 + c * st[0] - s * st[1], y0 + s * st[0] + c * st[1], st[2]])
        v = np.array([c * st[3] - s * st[4], s * st[3] + c * st[4], st[5]])
        p, v = back_to_release(flight, p, v)
        spin = rng.normal(0, spin_sigma, 3) if rng is not None and spin_sigma > 0 else np.zeros(3)
        out.append((p.tolist(), v.tolist(), spin.tolist(), r))
    return out


def replay(path, seed: int = 9000, spin: float = 0.0, verbose: bool = False) -> dict:
    head, recs = session_log.read(path)
    throws = [r for r in recs if r.get("type") == "throw"]
    w = head.get("world") or {}
    probe = CatchWorld(WorldConfig(dist=w.get("dist", 6.5), seed=seed))        # its geometry and ball model
    flight, pose0 = probe.flight, probe.pose0.copy()
    probe.close()
    sc = scripted_throws(throws, pose0, flight, np.random.default_rng(seed + 31), spin)
    if not sc:
        raise SystemExit(f"{path}: no throws with a tracked state (state0) to replay")
    cfg = WorldConfig(dist=w.get("dist", 6.5), seed=seed, return_throws=False,
                      scripted=[(p, v, sp) for p, v, sp, _ in sc])
    g = SilCatchGame(cfg, verbose=verbose)
    res = g.run(len(sc), t_max=12.0 * len(sc) + 10.0)
    g.world.close()
    rows = []
    for (p, v, _, r), out in zip(sc, res):
        rows.append(dict(toss=r.get("toss"), real=bool(r.get("caught")), sim=bool(out["caught"]),
                         speed=round(float(np.linalg.norm(v)), 2), sim_pos_error=round(out["pos_error"], 3)
                         if out["pos_error"] == out["pos_error"] else None,
                         logged_truth=(r.get("truth") or {}).get("caught")))
    n = len(rows)
    real, sim = sum(r["real"] for r in rows), sum(r["sim"] for r in rows)
    agree = sum(r["real"] == r["sim"] for r in rows)
    return dict(log=Path(path).name, source=head.get("source"), throws=n, real_caught=real, sim_caught=sim,
                real_rate=round(100 * real / n, 1), sim_rate=round(100 * sim / n, 1),
                gap_points=round(100 * (sim - real) / n, 1), agree=round(100 * agree / n, 1), rows=rows,
                unreplayed=len(throws) - n, seed=seed, spin=spin)


def demo(folder: Path, tosses: int = 6) -> tuple:
    g = SilCatchGame(WorldConfig(dist=6.5, apex_range=(1.0, 1.8), seed=9100))
    g.open_log(folder)
    g.run(tosses, t_max=15.0 * tosses)
    s = g.close_log()
    g.world.close()
    return folder / s["name"], s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?")
    ap.add_argument("--seed", type=int, default=9000)
    ap.add_argument("--spin", action="store_true", help="random spin (6 rad/s per axis, catch_world's default)")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.demo:
        out = HERE / "results" / "replay_demo"
        out.mkdir(parents=True, exist_ok=True)
        path, s = demo(out)
        print(f"sim session: {path.name}, {s['throws']} throws, {s['caught']} caught")
    elif a.log:
        path = Path(a.log)
    else:
        ap.error("a log, or --demo")
    r = replay(path, a.seed, 6.0 if a.spin else 0.0)
    print(f"\n{r['log']} ({r['source']}): {r['throws']} throws replayed ({r['unreplayed']} without a track)")
    print(f"  {'toss':>4} {'speed':>6}  logged  sim")
    for row in r["rows"]:
        print(f"  {row['toss']:>4} {row['speed']:>5.1f}   {'caught' if row['real'] else 'missed':6}  "
              f"{'caught' if row['sim'] else 'missed'}")
    print(f"  logged {r['real_rate']}%, sim {r['sim_rate']}%: gap {r['gap_points']:+} points; "
          f"same outcome on {r['agree']}% of throws")
    dest = Path(a.out) if a.out else HERE / "results" / f"replay_{Path(r['log']).stem}.json"
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(r, indent=1))
    print(f"wrote {dest}")


if __name__ == "__main__":
    main()
