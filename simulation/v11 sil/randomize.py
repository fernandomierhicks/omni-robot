"""Randomized worlds (v6 round 3, design.md 23.3): draw the guessed parameters instead of fixing them.

Every value below is an engineering estimate (est) of the range a real ball batch, net, floor and camera pipeline
could fall in. A "world" is one draw of all of them; studies/randomized.py runs the same throws in many worlds and
reports catch-rate ranges and which parameters move the result.

What changes in a world, and what doesn't:
  - the physical ball (diameter, mass, drag, foam stiffness, restitution, grip), the net, the pads, the tires and the
    floor change in the built model (apply());
  - the robot does NOT know: its hardware stays fitted to the nominal 74 mm ball, the trajectory predictor and the
    vision filter keep assuming the catalog ball (74 mm, 22 g, Cd 0.5), the launcher table is the nominal ball's and
    the base's acceleration table assumes a 0.95 floor. That is the situation before anything is measured;
  - the camera pipeline's noise and the throws' spin are drawn too (vision_cfg(), spin).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import mujoco
import numpy as np


@dataclass(frozen=True)
class Param:
    name: str
    lo: float
    hi: float
    unit: str
    why: str
    log: bool = False             # draw uniformly in log space

    def value(self, u: float) -> float:
        if self.log:
            return float(math.exp(math.log(self.lo) + u * (math.log(self.hi) - math.log(self.lo))))
        return float(self.lo + u * (self.hi - self.lo))


PARAMS = (
    Param("ball_diameter", 0.072, 0.076, "m", "a batch's spread; the chute takes 72-76 mm"),
    Param("ball_mass", 0.018, 0.026, "kg", "foam density varies (22 g assumed)"),
    Param("ball_cd", 0.42, 0.58, "", "drag coefficient (0.5 assumed; film throws to measure)"),
    Param("ball_stiffness", 1000.0, 2500.0, "N/m", "foam contact stiffness (1500 assumed)"),
    Param("ball_e_scale", 0.75, 1.25, "x", "restitution on hard surfaces, x (0.55 at 1 m/s, 0.38 at 7 m/s)"),
    Param("ball_mu_scale", 0.75, 1.25, "x", "ball grip on the pads, chute and structure"),
    Param("pad_e_scale", 0.5, 2.0, "x", "catch-pad restitution, x (0.10 / 0.04)", log=True),
    Param("net_mu", 0.7, 1.2, "", "ball on the net (1.0 assumed)"),
    Param("net_roll", 0.004, 0.020, "m", "net rolling resistance (12 mm assumed)"),
    Param("tire_mu", 0.8, 1.3, "", "TPU tires on foam: flywheels and feed wheels (1.1 assumed)"),
    Param("floor_mu", 0.70, 1.10, "", "omni rollers on the floor (0.95 assumed)"),
    Param("cam_latency", 0.030, 0.070, "s", "capture to filter (45 ms expected)"),
    Param("cam_calib_deg", 0.04, 0.30, "deg", "stereo calibration error (0.1 expected)", log=True),
    Param("cam_edge_px", 0.4, 0.9, "px", "segmentation edge noise (0.6 expected)"),
    Param("cam_drop", 0.02, 0.10, "", "dropped detections (5% expected)"),
    Param("cam_clutter", 0.0, 0.03, "", "clutter blobs picked instead of the ball (1% expected)"),
    Param("spin", 3.0, 10.0, "rad/s", "thrown ball's spin, 1-sigma per axis (6 assumed)"),
)
NAMES = [q.name for q in PARAMS]


def nominal() -> dict:
    """The fixed values v6 used (the centre of nothing in particular: the guesses)."""
    return dict(ball_diameter=0.074, ball_mass=0.022, ball_cd=0.50, ball_stiffness=1500.0, ball_e_scale=1.0,
                ball_mu_scale=1.0, pad_e_scale=1.0, net_mu=1.0, net_roll=0.012, tire_mu=1.1, floor_mu=0.95,
                cam_latency=0.045, cam_calib_deg=0.10, cam_edge_px=0.6, cam_drop=0.05, cam_clutter=0.01, spin=6.0)


def draw_worlds(n: int, seed: int = 0) -> list[dict]:
    """n worlds on a Latin hypercube (every parameter's range evenly covered)."""
    from scipy.stats import qmc
    u = qmc.LatinHypercube(d=len(PARAMS), seed=seed).random(n)
    return [{q.name: q.value(float(u[i, k])) for k, q in enumerate(PARAMS)} for i in range(n)]


def vision_cfg(base, w: dict):
    """The camera pipeline of world w (the filter is still tuned for base's noise and the catalog ball)."""
    sig_size = 1.2 * math.hypot(base.ball_size_rel, base.calib_f_rel) + 0.002
    return replace(base, latency_mean=w["cam_latency"], latency_jitter=0.22 * w["cam_latency"],
                   calib_rot_deg=w["cam_calib_deg"], edge_sigma=w["cam_edge_px"], drop_prob=w["cam_drop"],
                   false_prob=w["cam_clutter"], ball_size_rel=0.0, sigma_size_rel=sig_size)


def apply(robot, w: dict, perc=None):
    """Make the built model world w: the ball, the net, the pads, the tires and the floor. robot: robot.Robot
    (its contact model is refitted); perc: the game's VisionPerception (renders the real ball size)."""
    m, d, p = robot.m, robot.d, robot.p
    from robot_model import ball_names
    b = p.ball_spec
    r = w["ball_diameter"] / 2
    names = {m.geom(i).name: i for i in range(m.ngeom)}
    ball_geoms = [m.geom(n).id for n in ball_names(p)]
    for g in ball_geoms:
        bid = m.geom_bodyid[g]
        m.geom_size[g][0] = m.geom_rbound[g] = r
        m.body_mass[bid] = w["ball_mass"]
        m.body_inertia[bid] = 0.4 * w["ball_mass"] * r * r
        m.geom_fluid[g][1] = w["ball_cd"] / 2              # blunt drag (fluidcoef[0] = Cd / 2)
        m.geom_friction[g][0] = b.mu * w["ball_mu_scale"]
    is_ball = set(ball_geoms)
    for k in range(m.npair):
        g1, g2 = int(m.pair_geom1[k]), int(m.pair_geom2[k])
        other = g2 if g1 in is_ball else g1 if g2 in is_ball else None
        if other is None:
            if names.get("floor") in (g1, g2):                # the omni wheels on the floor
                m.pair_friction[k][:2] = w["floor_mu"]
            continue
        on = m.geom(other).name
        if on.startswith("funnel"):
            m.pair_friction[k][:2] = w["net_mu"]
            m.pair_friction[k][3:5] = w["net_roll"]
        elif on.startswith(("flywheel", "feed_")):
            m.pair_friction[k][:2] = w["tire_mu"]
        else:
            m.pair_friction[k][:2] = m.pair_friction[k][:2] * w["ball_mu_scale"]
    mujoco.mj_setConst(m, d)                                  # mass-dependent solver constants
    if robot.contacts is not None:
        robot.contacts.set_ball(mass=w["ball_mass"], stiffness=w["ball_stiffness"],
                                e_slow=b.e_slow * w["ball_e_scale"], e_fast=b.e_fast * w["ball_e_scale"],
                                pad_scale=w["pad_e_scale"])
    if perc is not None and hasattr(perc, "R_ball"):
        perc.R_ball = r                                       # the cameras see the real ball; the filter assumes 74 mm
