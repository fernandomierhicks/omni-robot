"""Catch rate for long, shallow throws: a thrower 6-7 m away, apex 0.75-2 m above a 1.0 m release.

v5 (design.md 21.4) found that a thrower beyond ~4 m catches less, not more: flight time is set by the apex, the
landing scatter grows with distance, and the ball arrives flatter (45 deg from vertical at 6 m with a 1 m apex,
beyond v3b's 15 deg catch-tilt cap). v6 tries to win those throws back.

Throws (the same for every robot): human-like scatter around the waiting spot, sigma 7% of the distance along the
throw and 5% across (v5's estimate for a casual adult throw), at 6, 6.5 and 7 m, apex 0.75 / 1 / 1.5 / 2 m.
3 distances x 4 apexes x 3 seeds x 6 tosses = 216 throws per robot. The expected camera pipeline (Argus, MCU
trigger, printed stereo bracket recalibrated per session; camera_estimate.EXPECTED), 6 rad/s of spin per axis.
A 12 x 6 m arena; the robot waits 2.5 m from the back netting.

  python shallow_sweep.py                  # every robot in ROBOTS
  python shallow_sweep.py "v5 robot, firmware adult" ...   # only these
Writes results/shallow_sweep.json (merged with earlier runs of other robots) and prints the table.
"""

from __future__ import annotations

import itertools
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import mujoco
import numpy as np

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from camera_estimate import EXPECTED, SPIN  # noqa: E402

DISTANCES = (6.0, 6.5, 7.0)
APEXES = (0.75, 1.0, 1.5, 2.0)
SEEDS = 3
TOSSES = 6
SIG_RANGE, SIG_LAT = 0.07, 0.05      # of the distance (est, as v5's thrower_distance.py)
HOME = (3.5, 0.0)
AREA = (12.0, 6.0)

FW = dict(limits="firmware", base_max_speed=2.0)       # v3b firmware, adult mode
FWG = dict(limits="firmware", catch_mode="moving")
PH = dict(catch_mode="moving")
# rounds 1-2 rows keep their ideal feed motor and elastic contacts, so re-running them reproduces the history
LEGACY = dict(feed_motor="ideal", contact_model="elastic")
V5 = dict(funnel="cone55", feed="kicker", ball="foam-pickleball", pocket_pad=False, **LEGACY)      # the v5 robot (v3b module)
V6M = dict(funnel="cone55", feed="roller", ball="foam-pickleball", pocket_pad=True, **LEGACY)
R3 = dict(funnel="cone55", feed="chute", pocket_pad=True)      # round 3: GM3506 feed motors, viscoelastic contacts (defaults)
V6G = dict(max_catch_tilt_deg=35.0, tilt_gain=0.6)

V5V = dict(vision_cfg=replace(EXPECTED, honest_init=False))          # v5's track start (5 cm, 2 m/s)
PRIOR = dict(landing_prior=0.4)

# name: (SimParams kwargs, GameConfig kwargs)
ROBOTS = {
    "v5 robot, firmware adult": ({**V5, **FW}, dict(FWG, **V5V, max_catch_tilt_deg=15.0, tilt_gain=0.6)),
    "v5 robot, physics": ({**V5}, dict(PH, **V5V, max_catch_tilt_deg=15.0, tilt_gain=0.6)),
    # diagnostics: exact ball state after 0.12 s (the vision model's first estimate) instead of the cameras
    "v5 robot, firmware adult, exact ball state": ({**V5, **FW}, dict(FWG, max_catch_tilt_deg=15.0, tilt_gain=0.6,
                                                                       perception="truth", perception_delay=0.12)),
    # v6 catch module: roller feed (holds the ball at any tilt), padded pocket, catch tilt up to 35 deg
    "v6 module, firmware adult": ({**V6M, **FW}, dict(FWG, **V6G, **V5V)),
    "v6 module, physics": ({**V6M}, dict(PH, **V6G, **V5V)),
    "v6 module, firmware adult, exact ball state": ({**V6M, **FW}, dict(FWG, **V6G, perception="truth",
                                                                          perception_delay=0.12)),
    # + v6 perception: honest track start
    "v6 module + track start, firmware adult": ({**V6M, **FW}, dict(FWG, **V6G)),
    "v6 module + track start, physics": ({**V6M}, dict(PH, **V6G)),
    # + v6 planning: landing prior
    "v6 module + track start + prior, firmware adult": ({**V6M, **FW}, dict(FWG, **V6G, **PRIOR)),
    "v6 module + track start + prior, physics": ({**V6M}, dict(PH, **V6G, **PRIOR)),
    # round 2: the chute pocket (size-tolerant feed)
    "v6 chute + track start, firmware adult": ({**V6M, "feed": "chute", **FW}, dict(FWG, **V6G)),
    "v6 chute + track start, physics": ({**V6M, "feed": "chute"}, dict(PH, **V6G)),
    # round 3: real feed motors (GM3506 gimbal, FOC hold) and viscoelastic contacts
    "v6 round 3 (GM3506 feed, viscoelastic), firmware adult": ({**R3, **FW}, dict(FWG, **V6G)),
    "v6 round 3 (GM3506 feed, viscoelastic), physics": ({**R3}, dict(PH, **V6G)),
}


class HumanToss(CatchGame):
    """Landing points scattered around the waiting spot like a person's throws (v5 thrower_distance.py)."""

    def __init__(self, *a, dist=6.0, **kw):
        super().__init__(*a, **kw)
        self.dist = dist

    def _toss(self, pose):
        rng_state = self.rng.bit_generator.state
        super()._toss(pose)
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
        self.release_speed = float(np.linalg.norm(v0))


def run_cell(args):
    robot, dist, apex, seed = args
    pkw, gkw = ROBOTS[robot]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ckw = dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
               wait_between=1.2, perception="vision", vision_cfg=EXPECTED, toss_spin=SPIN)
    cfg = GameConfig(**{**ckw, **gkw})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    first, out = None, []
    rim_margin = p.funnel_catch_radius - p.ball_spec.radius
    while len(game.results) < TOSSES:
        game.step()
        if game.state == "flight" and first is None and (game.perc.ready() if game.perc else d.time - game.t_release >= cfg.perception_delay):
            first = d.time - game.t_release
        if len(game.results) > len(out):
            r = game.results[-1]
            why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin
                                             else "bounce")
            out.append(dict(caught=bool(r.caught), why=why, offset=r.zone_dist, pos_error=r.pos_error,
                            flight_time=r.flight_time, arrival_deg=r.arrival_deg, arrival_speed=r.arrival_speed,
                            tilt_deg=r.tilt_deg, catch_speed=r.catch_speed, first=first, moved=r.moved,
                            moving=r.moving, release_speed=getattr(game, "release_speed", float("nan")),
                            pred_err=r.pred_err, ctrl_err=r.ctrl_err, exit=getattr(r, "exit", "")))
            first = None
    return dict(robot=robot, distance=dist, apex=apex, seed=seed, tosses=out)


def _sel(rows, **kw):
    return [t for r in rows if all(r[k] == v for k, v in kw.items()) for t in r["tosses"]]


def summarize(rows, robot):
    ts = _sel(rows, robot=robot)
    if not ts:
        return None
    k = float(np.mean([t["caught"] for t in ts]))
    by_apex = {a: float(np.mean([t["caught"] for t in _sel(rows, robot=robot, apex=a)])) for a in APEXES}
    by_dist = {dd: float(np.mean([t["caught"] for t in _sel(rows, robot=robot, distance=dd)])) for dd in DISTANCES}
    caught = [t for t in ts if t["caught"]]
    return dict(n=len(ts), caught=k, reach=sum(t["why"] == "reach" for t in ts),
                bounce=sum(t["why"] == "bounce" for t in ts), by_apex=by_apex, by_dist=by_dist,
                offset_med=float(np.nanmedian([t["pos_error"] for t in caught])) if caught else float("nan"),
                arrival_med=float(np.nanmedian([t["arrival_deg"] for t in ts])),
                speed_med=float(np.nanmedian([t["arrival_speed"] for t in ts])),
                land_med=float(np.median([t["offset"] for t in ts])),
                first_med=float(np.nanmedian([t["first"] if t["first"] is not None else np.nan for t in ts])))


def report(rows, robots):
    print(f"{'robot':52s} caught  reach/bounce | " + " ".join(f"apex {a:<4g}" for a in APEXES) + " | " +
          " ".join(f"{dd:g} m " for dd in DISTANCES) + "| offset  first est")
    out = {}
    for name in robots:
        s = summarize(rows, name)
        if s is None:
            continue
        out[name] = s
        print(f"{name:52s} {100 * s['caught']:4.0f}%   {s['reach']:3d}/{s['bounce']:<3d}    | " +
              " ".join(f"{100 * v:6.0f}%  " for v in s["by_apex"].values()) + " | " +
              " ".join(f"{100 * v:4.0f}% " for v in s["by_dist"].values()) +
              f"| {100 * s['offset_med']:4.1f} cm  {s['first_med']:.3f} s")
    ts = _sel(rows, robot=next(iter(out))) if out else []
    if ts:
        print(f"throws: arrival median {np.nanmedian([t['arrival_deg'] for t in ts]):.0f} deg from vertical "
              f"(range {np.nanmin([t['arrival_deg'] for t in ts]):.0f}-{np.nanmax([t['arrival_deg'] for t in ts]):.0f}), "
              f"{np.nanmedian([t['arrival_speed'] for t in ts]):.1f} m/s, release "
              f"{np.nanmedian([t['release_speed'] for t in ts]):.1f} m/s, flight "
              f"{np.nanmedian([t['flight_time'] for t in ts]):.2f} s; landing {100 * np.median([t['offset'] for t in ts]):.0f} cm "
              f"from the waiting spot (median)")
    return out


def main(names=None):
    todo = names or list(ROBOTS)
    cells = [(rb, dd, a, s) for rb in todo for j, (dd, a, s) in
             enumerate(itertools.product(DISTANCES, APEXES, range(SEEDS)))]
    cells = [(rb, dd, a, 100 * s + 10 * DISTANCES.index(dd) + APEXES.index(a)) for rb, dd, a, s in cells]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=1))
    path = RESULTS / "shallow_sweep.json"
    old = json.loads(path.read_text())["rows"] if path.exists() else []
    rows_all = [r for r in old if r["robot"] not in todo] + rows
    summary = report(rows_all, [n for n in ROBOTS if any(r["robot"] == n for r in rows_all)])
    save("shallow_sweep", dict(rows=rows_all, summary=summary, sig=(SIG_RANGE, SIG_LAT)))


if __name__ == "__main__":
    main(sys.argv[1:] or None)
