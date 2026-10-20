"""Does standing farther away help? Catch rate vs thrower distance, 1.5-6 m.

Flight time is set by the apex, not by the distance: release at 1.0 m, rim at ~0.39 m, so a 1 m apex flies
~1.0 s whether the thrower stands at 1.5 m or 6 m. What distance does change:
  - the thrower's scatter: a 3-4% release-speed error is a ~7% range error (range ~ v^2), and ~3 deg of aim is
    ~5% sideways, so the robot must move farther for the same accuracy of throw (est, adult casual underhand);
  - the arrival: at a fixed apex a longer throw is faster and flatter (harder for the funnel);
  - perception: the ball starts smaller and farther (stereo depth error grows with range^2), but high lobs from
    close in leave the top of the cameras' view.

Two toss sets, each with the expected camera pipeline of camera_estimate.py (Argus, 45 ms, 0.1 deg, spin):
  "fixed":  landing exactly 0.3 / 0.6 / 1.0 m from the waiting spot (random direction), apex 0.5-3 m: what
            distance does on its own;
  "human":  landing scatter grows with distance, sigma 7% of it along the throw and 5% across, apex 1 / 2 / 3 m.
Robots: physics limits + moving catches (v5), and firmware adult mode (v3b, 2 m/s, 20 cm gate).
A 10 x 6 m arena so a 6 m throw fits; the robot waits 2.5 m from the back netting.
Writes results/thrower_distance.json.
"""

from __future__ import annotations

import itertools
import math
import os
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import SimParams, save  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from camera_estimate import EXPECTED, SPIN  # noqa: E402

DISTANCES = (1.5, 2.0, 3.0, 4.0, 5.0, 6.0)
FIXED_APEX = (0.5, 1.0, 2.0, 3.0)
FIXED_OFF = (0.3, 0.6, 1.0)
HUMAN_APEX = (1.0, 2.0, 3.0)
SIG_RANGE, SIG_LAT = 0.07, 0.05      # of the distance (est)
TOSSES = 6
HOME = (2.5, 0.0)
AREA = (10.0, 6.0)
ROBOTS = {
    "physics, moving": (dict(), dict(catch_mode="moving")),
    "firmware adult mode": (dict(limits="firmware", base_max_speed=2.0), dict(limits="firmware", catch_mode="moving")),
}


class HumanTossGame(CatchGame):
    """Landing points scattered around the waiting spot like a person's throws: sigma grows with distance."""

    def __init__(self, *a, dist=3.0, **kw):
        super().__init__(*a, **kw)
        self.dist = dist

    def _toss(self, pose):
        rng_state = self.rng.bit_generator.state
        super()._toss(pose)                                  # sets everything up; then re-aim from the same rng state
        self.rng.bit_generator.state = rng_state
        e = self.rng.normal(0, 1, 2) * np.array([SIG_RANGE, SIG_LAT]) * self.dist
        u = self.home[:2] - self.hand[:2]
        u /= np.linalg.norm(u)
        aim = self.home[:2] + e[0] * u + e[1] * np.array([-u[1], u[0]])
        self.apex = self.rng.uniform(*self.cfg.apex_range)
        z_aim = self.robot.g.rim_center_robot(0.0)[1]
        v0 = self.flight.solve_toss(self.hand, [*aim, z_aim], self.apex)
        self.d.qvel[self.bv:self.bv + 3] = v0
        self.zone_dist = float(np.linalg.norm(aim - self.home[:2]))


def run_cell(args):
    mode, robot, dist, apex, off, seed = args
    pkw, gkw = ROBOTS[robot]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off if mode == "fixed" else 0.0, zone_mode="ring", apex_range=(apex, apex),
                     return_throws=False, wait_between=1.2, perception="vision", vision_cfg=EXPECTED, toss_spin=SPIN,
                     **gkw)
    game = (HumanTossGame(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist) if mode == "human"
            else CatchGame(m, d, p, cfg=cfg, seed=seed, verbose=False))
    first, out = None, []
    while len(game.results) < TOSSES:
        game.step()
        if game.state == "flight" and first is None and game.perc.ready():
            first = d.time - game.t_release
        if len(game.results) > len(out):
            r = game.results[-1]
            rim_margin = p.funnel_rim_radius - p.ball_spec.radius
            why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin
                                             else "bounce")
            out.append(dict(caught=bool(r.caught), why=why, offset=r.zone_dist, flight_time=r.flight_time,
                            arrival_deg=r.arrival_deg, arrival_speed=r.arrival_speed, first=first, moved=r.moved))
            first = None
    return dict(mode=mode, robot=robot, distance=dist, apex=apex, off=off, tosses=out)


def _sel(rows, **kw):
    return [t for r in rows if all(r[k] == v for k, v in kw.items()) for t in r["tosses"]]


def report(rows):
    for robot in ROBOTS:
        print(f"\n=== {robot} ===")
        print("fixed landing offset: catch rate by distance (rows) and apex (columns), all offsets pooled; then by offset")
        print("  dist   " + "".join(f"apex {a:<4g}" for a in FIXED_APEX) + "  | " +
              "".join(f"off {o:<5g}" for o in FIXED_OFF) + "| all   flight(1m apex)  arrival(1m)  first est")
        for dd in DISTANCES:
            ca = [np.mean([t["caught"] for t in _sel(rows, mode="fixed", robot=robot, distance=dd, apex=a)]) for a in FIXED_APEX]
            co = [np.mean([t["caught"] for t in _sel(rows, mode="fixed", robot=robot, distance=dd, off=o)]) for o in FIXED_OFF]
            al = _sel(rows, mode="fixed", robot=robot, distance=dd)
            a1 = _sel(rows, mode="fixed", robot=robot, distance=dd, apex=1.0)
            print(f"  {dd:4.1f}   " + "".join(f"{100 * v:6.0f}%   " for v in ca) + "  | " +
                  "".join(f"{100 * v:6.0f}%   " for v in co) +
                  f"| {100 * np.mean([t['caught'] for t in al]):3.0f}%   "
                  f"{np.nanmedian([t['flight_time'] for t in a1]):.2f} s        "
                  f"{np.nanmedian([t['arrival_deg'] for t in a1]):3.0f} deg      "
                  f"{np.nanmedian([t['first'] or np.nan for t in al]):.3f} s")
        print(f"human scatter (sigma {100 * SIG_RANGE:.0f}% x {100 * SIG_LAT:.0f}% of distance): catch rate by distance and apex")
        print("  dist   " + "".join(f"apex {a:<4g}" for a in HUMAN_APEX) + "| all   median offset  misses reach/bounce")
        for dd in DISTANCES:
            ca = [np.mean([t["caught"] for t in _sel(rows, mode="human", robot=robot, distance=dd, apex=a)]) for a in HUMAN_APEX]
            al = _sel(rows, mode="human", robot=robot, distance=dd)
            print(f"  {dd:4.1f}   " + "".join(f"{100 * v:6.0f}%   " for v in ca) +
                  f"| {100 * np.mean([t['caught'] for t in al]):3.0f}%   {100 * np.median([t['offset'] for t in al]):5.0f} cm"
                  f"        {sum(t['why'] == 'reach' for t in al)}/{sum(t['why'] == 'bounce' for t in al)}")


def main():
    cells = []
    for robot in ROBOTS:
        cells += [("fixed", robot, dd, a, o, j) for j, (dd, a, o) in
                  enumerate(itertools.product(DISTANCES, FIXED_APEX, FIXED_OFF))]
        cells += [("human", robot, dd, a, 0.0, 1000 + j) for j, (dd, a, _) in
                  enumerate(itertools.product(DISTANCES, HUMAN_APEX, range(4)))]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=1))
    save("thrower_distance", dict(rows=rows, sig=(SIG_RANGE, SIG_LAT)))
    report(rows)


if __name__ == "__main__":
    main()
