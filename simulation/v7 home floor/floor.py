"""Real floors (v6 round 3, design.md 23.4): rolling resistance, friction that varies, door thresholds and carpet.

The sim's floor was one flat plane with one friction for the omni rollers (0.95) and free-rolling wheels. A home
floor isn't:
  - rolling resistance: the rubber rollers deform and lose energy as they roll (Crr ~0.02 on a hard floor, ~0.06 on
    carpet, est). RollingResistance applies it every step as joint drag that follows each wheel's measured load and
    the surface under it: Crr N R on the drive hub (rolling) and Crr N r on the roller freedom (sideways). Not as
    MuJoCo rolling friction: in the elliptic cone a wheel that always rolls holds its rolling torque at the limit,
    which takes its sliding grip away (the base then ran 2-7% slower than its wheels);
  - friction varies: dust, a polished strip, a tile next to wood. FloorFriction sets the friction of each wheel's
    floor contact every step from where that contact is: a smooth random field (floor_mu_field = (sigma,
    correlation length, seed)) plus rectangular patches (floor_patches = ((x0, y0, x1, y1, mu), ...));
  - thresholds: a beveled strip across a doorway (Threshold), a convex prism on the floor;
  - carpet / rugs: a slab with its own grip, compliance and rolling resistance (Carpet); its edge is a step the
    wheels must climb, square (a rug) or beveled (carpet edge trim).
The wheels touch these only through explicit contact pairs (robot_model), so each feature has its own friction and
softness. When any feature exists, the robot gets collision geoms for its battery and motor undersides ("belly",
~21 mm up), so it can ground out on an obstacle instead of passing through it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from assets import _f


@dataclass(frozen=True)
class Threshold:
    """A door threshold strip: height h, flat top `top` wide, a ramp `ramp` wide on each side, `length` long, its
    long axis at yaw_deg (0 = along world y, so driving along x crosses it head-on)."""
    x: float
    y: float
    yaw_deg: float = 0.0
    length: float = 1.2
    height: float = 0.012
    top: float = 0.030
    ramp: float = 0.020
    mu: float = 0.8              # rollers on the (wood/aluminium) strip (est)


@dataclass(frozen=True)
class Carpet:
    """A rug or carpet area from (x0, y0) to (x1, y1), `thickness` high. Its x edges are square (a rug's cut edge)
    or ramped over `bevel` (carpet edge trim). On it the rollers grip `mu`, roll with `crr` and sink into a soft
    contact (solref timeconst `softness`)."""
    x0: float
    y0: float
    x1: float
    y1: float
    thickness: float = 0.008
    bevel: float = 0.0
    mu: float = 0.85             # rubber rollers on low-pile carpet (est)
    crr: float = 0.06            # rolling resistance on carpet (est; hard floor: SimParams.wheel_crr)
    softness: float = 0.012      # s: the wheel-carpet contact sinks ~2 mm under a wheel (est)
    yaw_deg: float = 0.0         # v7: turned about its centre (the corners above are before the turn)
    rgba: tuple | None = None    # v7: colour (the house scene)
    bevel_y: float = 0.0         # v7: ramp on the y edges too (a raised tile floor or wall-to-wall carpet with trim)

    @property
    def center(self):
        return (self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2

    def local(self, xy) -> np.ndarray:
        """World points in the rug's own (unturned) frame."""
        xy = np.atleast_2d(xy) - np.array(self.center)
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return np.column_stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]]) + np.array(self.center)

    def corner(self, x, y):
        """A point given in the unturned frame, in the world."""
        cx, cy = self.center
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return cx + c * (x - cx) - s * (y - cy), cy + s * (x - cx) + c * (y - cy)

    def contains(self, xy) -> np.ndarray:
        xy = self.local(xy)
        return ((xy[:, 0] >= min(self.x0, self.x1)) & (xy[:, 0] <= max(self.x0, self.x1)) &
                (xy[:, 1] >= min(self.y0, self.y1)) & (xy[:, 1] <= max(self.y0, self.y1)))


@dataclass(frozen=True)
class Wall:
    """v7, the house scene: a partition wall from (x0, y0) to (x1, y1) (the robot's deck and bumper hit it)."""
    x0: float
    y0: float
    x1: float
    y1: float
    height: float = 0.5
    thickness: float = 0.1


# v7 (design.md 24): the other transitions of a one-storey house, each with its edge at x = 0.6 like
# studies/floor_wheels.OBSTACLES (heights typical of the parts sold for them, est)
HOUSE_OBSTACLES = {
    "T-molding 5 mm": Threshold(0.6, 0.0, height=0.005, top=0.012, ramp=0.012, mu=0.8),    # wood / laminate rooms
    "reducer 8 mm": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.008, bevel=0.045, mu=0.9, crr=0.02, softness=0.004),
    "bath mat 6 mm": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.006, mu=0.85, crr=0.05),       # square edge, thin
    "plush rug 16 mm": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.016, mu=0.85, crr=0.08, softness=0.015),
}


def _prism(name: str, profile, length: float) -> str:
    """Mesh extruded along local y from a convex (x, z) profile."""
    v = [c for x, z in profile for c in (x, -length / 2, z)] + [c for x, z in profile for c in (x, length / 2, z)]
    return f'<mesh name="{name}" vertex="{_f(*v)}"/>'


def features(p) -> list:
    return list(p.floor_features)


def edges(p) -> list:
    """v7: the floor's edges as a map would hold them: (x0, y0, x1, y1, height, square). A threshold is one ramped
    edge along its centre line; a rug or carpet has four edges, its x edges square unless beveled, its y edges square."""
    out = []
    for f in features(p):
        if isinstance(f, Threshold):
            c, s = math.cos(math.radians(f.yaw_deg)), math.sin(math.radians(f.yaw_deg))
            h = f.length / 2
            out.append((f.x + s * h, f.y - c * h, f.x - s * h, f.y + c * h, f.height, f.ramp < 0.002))
        elif isinstance(f, Carpet):
            sx, sy = f.bevel < 0.002, f.bevel_y < 0.002
            for a, b, sq in (((f.x0, f.y0), (f.x0, f.y1), sx), ((f.x1, f.y0), (f.x1, f.y1), sx),
                             ((f.x0, f.y0), (f.x1, f.y0), sy), ((f.x0, f.y1), (f.x1, f.y1), sy)):
                out.append((*f.corner(*a), *f.corner(*b), f.thickness, sq))
        elif isinstance(f, Wall):
            continue
    return out


def seg_dist(pts: np.ndarray, e) -> np.ndarray:
    """Distance from each point (n, 2) to the edge segment e = (x0, y0, x1, y1, ...)."""
    a, b = np.array(e[:2]), np.array(e[2:4])
    ab = b - a
    t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(pts - (a + np.outer(t, ab)), axis=1)


def assets_xml(p) -> str:
    out = []
    for k, f in enumerate(features(p)):
        if isinstance(f, Threshold):
            a, b = f.top / 2, f.top / 2 + f.ramp
            out.append(_prism(f"floorfeat{k}", [(-b, -0.001), (b, -0.001), (a, f.height), (-a, f.height)], f.length))
        elif isinstance(f, Carpet):
            w, ln, t = abs(f.x1 - f.x0) / 2, abs(f.y1 - f.y0) / 2, f.thickness
            bx, by = max(f.bevel, 1e-4), max(f.bevel_y, 1e-4)
            v = [c for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1)) for c in (x * (w + bx), y * (ln + by), -0.001)] +                 [c for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1)) for c in (x * w, y * ln, t)]
            out.append(f'<mesh name="floorfeat{k}" vertex="{_f(*v)}"/>')
    return "".join(out)


def world_xml(p) -> str:
    """Feature geoms in the world body. The ball (bit 1) and the robot's belly (bit 3) hit them by contype; the
    wheels only through their pairs."""
    out = []
    for k, f in enumerate(features(p)):
        if isinstance(f, Threshold):
            q = (math.cos(math.radians(f.yaw_deg) / 2), 0, 0, math.sin(math.radians(f.yaw_deg) / 2))
            out.append(f'<geom name="floorfeat{k}" type="mesh" mesh="floorfeat{k}" pos="{_f(f.x, f.y, 0)}" quat="{_f(*q)}" '
                       f'contype="1" conaffinity="5" material="post" friction="{f.mu} 0.005 0.0001"/>')
        elif isinstance(f, Carpet):
            q = (math.cos(math.radians(f.yaw_deg) / 2), 0, 0, math.sin(math.radians(f.yaw_deg) / 2))
            rgba = getattr(f, "rgba", None) or (0.45, 0.2, 0.18, 1)
            out.append(f'<geom name="floorfeat{k}" type="mesh" mesh="floorfeat{k}" quat="{_f(*q)}" '
                       f'pos="{_f((f.x0 + f.x1) / 2, (f.y0 + f.y1) / 2, 0)}" contype="1" conaffinity="5" '
                       f'rgba="{_f(*rgba)}" friction="{f.mu} 0.005 0.0001"/>')
        elif isinstance(f, Wall):
            L = math.hypot(f.x1 - f.x0, f.y1 - f.y0) / 2
            q = (math.cos(math.atan2(f.y1 - f.y0, f.x1 - f.x0) / 2), 0, 0, math.sin(math.atan2(f.y1 - f.y0, f.x1 - f.x0) / 2))
            out.append(f'<geom name="floorfeat{k}" type="box" size="{_f(L, f.thickness / 2, f.height / 2)}" quat="{_f(*q)}" '
                       f'pos="{_f((f.x0 + f.x1) / 2, (f.y0 + f.y1) / 2, f.height / 2)}" contype="1" conaffinity="1" '
                       f'rgba="0.93 0.91 0.86 1" friction="0.5 0.005 0.0001"/>')
    return "".join(out)


def wheel_pairs_xml(p, wheel_geoms) -> str:
    out = []
    for k, f in enumerate(features(p)):
        if isinstance(f, Wall):
            continue
        if isinstance(f, Threshold):
            sol = 'solref="0.004 1" solreffriction="0.002 1" solimp="0.95 0.99 0.001"'
        else:
            sol = f'solref="{_f(f.softness)} 1" solreffriction="0.002 1" solimp="0.9 0.99 0.003"'
        out += [f'<pair geom1="floorfeat{k}" geom2="{g}" condim="3" friction="{_f(f.mu, f.mu)} 0.005 0.0001 0.0001" {sol}/>'
                for g in wheel_geoms]
    return "".join(out)


def belly_xml(p, g) -> str:
    """Collision stand-ins for the battery and the motors' undersides (only obstacles and the floor see them)."""
    if not features(p):
        return ""
    dz = g.deck_z
    out = [f'<geom name="belly_battery" type="box" size="0.075 0.036 0.0115" pos="0 0 {_f(dz - 0.0115)}" '
           f'contype="4" conaffinity="0" group="3" mass="0" friction="0.4 0.005 0.0001"/>']
    mot = p.motor
    L = p.wheel_base_radius
    hw = p.omni.width / 2
    for i, a in enumerate(p.wheel_angles_deg):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        r0 = L - hw - 0.004
        r1 = r0 - mot.length
        out.append(f'<geom name="belly_motor{i + 1}" type="cylinder" size="{_f(mot.diameter / 2)}" '
                   f'fromto="{_f(r0 * c, r0 * s, g.R, r1 * c, r1 * s, g.R)}" contype="4" conaffinity="0" group="3" '
                   f'mass="0" friction="0.4 0.005 0.0001"/>')
    return "".join(out)


class FloorFriction:
    """Per-step wheel-floor friction from where each wheel's floor contact is (field + patches)."""

    def __init__(self, m, d, p):
        self.m, self.d, self.p = m, d, p
        floor = m.geom("floor").id
        self.pairs, self.geoms = [], []
        for k in range(m.npair):
            g1, g2 = int(m.pair_geom1[k]), int(m.pair_geom2[k])
            if floor in (g1, g2):
                self.pairs.append(k)
                self.geoms.append(g2 if g1 == floor else g1)
        self.pairs, self.geoms = np.array(self.pairs), np.array(self.geoms)
        self.mu0 = p.mu
        self.modes = None
        if p.floor_mu_field is not None:
            sigma, corr, seed = p.floor_mu_field
            rng = np.random.default_rng(int(seed))
            n = 64                       # random Fourier modes: a Gaussian field with a Gaussian covariance
            self.k = rng.normal(0, 1 / corr, (n, 2))
            self.phi = rng.uniform(0, 2 * math.pi, n)
            self.amp = sigma * math.sqrt(2.0 / n)
            self.modes = n
        self.patches = np.array(p.floor_patches, dtype=float).reshape(-1, 5)

    def mu_at(self, xy: np.ndarray) -> np.ndarray:
        xy = np.atleast_2d(xy)
        mu = np.full(len(xy), self.mu0)
        if self.modes:
            mu = mu + self.amp * np.cos(xy @ self.k.T + self.phi).sum(axis=1)
        for x0, y0, x1, y1, mp in self.patches:
            inside = (xy[:, 0] >= min(x0, x1)) & (xy[:, 0] <= max(x0, x1)) & (xy[:, 1] >= min(y0, y1)) & \
                (xy[:, 1] <= max(y0, y1))
            mu = np.where(inside, mp, mu)
        return np.clip(mu, 0.2, 1.4)

    def pre_step(self):
        xy = self.d.geom_xpos[self.geoms][:, :2]
        mu = self.mu_at(xy)
        self.m.pair_friction[self.pairs, 0] = mu
        self.m.pair_friction[self.pairs, 1] = mu


class RollingResistance:
    """Load- and surface-dependent rolling resistance of each omni wheel, as joint drag (frictionloss) set every step:
    Crr N R on the drive hub, Crr N r_roller on the roller freedom (the sphere model's ball joint, or each explicit
    roller), on top of the gearbox and roller-bearing drag already in the model."""

    def __init__(self, m, d, p):
        self.m, self.d, self.p = m, d, p
        self.carpets = [f for f in features(p) if isinstance(f, Carpet)]
        n = p.n_wheels
        self.drive = np.array([m.jnt_dofadr[m.joint(f"drive{i}").id] for i in range(1, n + 1)])
        self.drive0 = m.dof_frictionloss[self.drive].copy()
        self.hub = [m.body(f"hub{i}").id for i in range(1, n + 1)]
        self.R, self.r = p.wheel_radius, p.omni.roller_diameter / 2
        self._cache = {}
        self.roller_dofs = []
        for i in range(1, n + 1):
            if p.wheel_model == "sphere":
                j = m.joint(f"roller{i}").id
                self.roller_dofs.append(np.arange(m.jnt_dofadr[j], m.jnt_dofadr[j] + 3))
            else:
                hub = self.hub[i - 1]
                js = [j for j in range(m.njnt) if m.body_parentid[m.jnt_bodyid[j]] == hub and m.jnt_type[j] == 3]
                self.roller_dofs.append(np.array([m.jnt_dofadr[j] for j in js]))
        self.roller0 = [m.dof_frictionloss[ix].copy() for ix in self.roller_dofs]

    def pre_step(self):
        from robot_model import wheel_loads     # (robot_model imports this module)
        m, d = self.m, self.d
        load = wheel_loads(m, d, self.p, self._cache)
        self._tick = getattr(self, "_tick", -1) + 1
        if self._tick % 10 == 0 or not hasattr(self, "_crr"):      # the surface under a wheel: every 5 ms is plenty
            crr = np.full(len(load), self.p.wheel_crr)
            if self.carpets:
                xy = d.xpos[self.hub][:, :2]
                for c in self.carpets:
                    crr = np.where(c.contains(xy), c.crr, crr)
            self._crr = crr
        f = self._crr * load
        m.dof_frictionloss[self.drive] = self.drive0 + f * self.R
        for k, ix in enumerate(self.roller_dofs):
            m.dof_frictionloss[ix] = self.roller0[k] + f[k] * self.r


def attach(m, d, p) -> list:
    """The floor's per-step hooks for this model (an empty list for v6's uniform, free-rolling floor)."""
    hooks = []
    if p.floor_mu_field is not None or len(p.floor_patches):
        hooks.append(FloorFriction(m, d, p))
    if p.wheel_crr > 0 or any(isinstance(f, Carpet) for f in features(p)):
        hooks.append(RollingResistance(m, d, p))
    return hooks
