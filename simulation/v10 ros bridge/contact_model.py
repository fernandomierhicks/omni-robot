"""Viscoelastic contacts for the foam ball and the tires (v6 round 3, design.md 23.2).

MuJoCo's soft contact is a linear spring-damper with a fixed damping ratio (solref = timeconst, dampratio), so a
ball bounces with the same restitution at every speed, the damping force jumps on at touch-down and, near release,
the contact would have to pull (MuJoCo clips it to zero instead). Foam and rubber aren't like that:

Ball contacts: Hunt-Crossley. The damping grows with the compression, c = 2 m wn (zeta0 + lam * delta), so a hard
hit (deep compression) loses a bigger share of its energy than a soft one, and the force starts and ends at zero.
Each material is fitted, in MuJoCo itself, to two drop-test numbers: the restitution at 1 m/s and at 7 m/s
(catalog Ball.e_slow / e_fast for the ball on hard surfaces; SimParams.pad_e_slow / pad_e_fast for the catch pad).
The spring stays the ball's foam stiffness. Once per physics step the damping ratio of every touching pair (and of
each ball's own solref, which its default contacts with the floor, walls and structure use) is set from that pair's
compression in the previous step: one step (0.5 ms) of lag in a ~12 ms contact.

Squeeze slides (flywheels "squeeze_*", feed wheels "feed_slide_*"): a standard linear solid. The joint's spring is
the static stiffness; a Maxwell arm (spring k_m in series with a dashpot, relaxation time tau) adds stiffness at
high rates and dissipates energy while the ball is squeezed through. Applied as a joint force (qfrc_applied), with
the dashpot's state integrated exactly over each step.

The net and the tire treads keep their own contacts (the net is lossy cloth; tire grip comes from the slides).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import mujoco
import numpy as np

V_SLOW, V_FAST = 1.0, 7.0      # m/s, the two drop-test speeds a material is fitted to
SOLIMP = "0.95 0.99 0.001"


@dataclass(frozen=True)
class HuntCrossley:
    wn: float       # rad/s, sqrt(k / m)
    zeta0: float    # damping ratio at zero compression
    lam: float      # 1/m: added damping ratio per metre of compression

    def solref(self, pen: float) -> tuple[float, float]:
        z = self.zeta0 + self.lam * max(0.0, pen)
        return 1.0 / (self.wn * z), z


def _drop_xml(radius: float, mass: float) -> str:
    return f"""<mujoco><option timestep="0.0005" integrator="implicitfast"/>
      <worldbody><geom name="floor" type="plane" size="1 1 0.1"/>
        <body name="ball" pos="0 0 1"><freejoint/>
          <geom name="ball" type="sphere" size="{radius}" mass="{mass}" solimp="{SOLIMP}" priority="1"/></body>
      </worldbody></mujoco>"""


def drop_restitution(hc: HuntCrossley, v: float, radius: float = 0.037, mass: float = 0.022) -> float:
    """Restitution of a ball hitting a plane at v under this law (MuJoCo, same solver settings as the robot)."""
    m = _drop_model(radius, mass)
    d = mujoco.MjData(m)
    g = m.geom("ball").id
    d.qpos[2] = radius + 0.0005
    d.qvel[2] = -v
    m.geom_solref[g] = hc.solref(0.0)
    touched = False
    for _ in range(400):
        pen = max((-d.contact.dist[i] for i in range(d.ncon)), default=0.0)
        m.geom_solref[g] = hc.solref(pen)
        mujoco.mj_step(m, d)
        if d.ncon:
            touched = True
        elif touched and d.qvel[2] > 0:
            return float(d.qvel[2] / v)
    return float(max(d.qvel[2], 0.0) / v)


@lru_cache(maxsize=None)
def _drop_model(radius: float, mass: float):
    return mujoco.MjModel.from_xml_string(_drop_xml(radius, mass))


@lru_cache(maxsize=None)
def fit(stiffness: float, mass: float, e_slow: float, e_fast: float, radius: float = 0.037) -> HuntCrossley:
    """Hunt-Crossley law with this spring whose drop test gives e_slow at 1 m/s and e_fast at 7 m/s."""
    from scipy.optimize import least_squares
    wn = math.sqrt(stiffness / mass)

    def res(x):
        hc = HuntCrossley(wn, x[0], x[1] * 100.0)
        return [drop_restitution(hc, V_SLOW, radius, mass) - e_slow, drop_restitution(hc, V_FAST, radius, mass) - e_fast]

    lr = math.log(max(e_slow, 1e-3))
    z0 = -lr / math.sqrt(math.pi ** 2 + lr ** 2)
    sol = least_squares(res, [z0, 1.0], bounds=([0.0, 0.0], [3.0, 200.0]), diff_step=0.02, xtol=1e-6)
    return HuntCrossley(wn, float(sol.x[0]), float(sol.x[1] * 100.0))


class ViscoContacts:
    """Per-step Hunt-Crossley damping on the ball contacts and the standard-linear-solid squeeze slides."""

    def __init__(self, m: mujoco.MjModel, d: mujoco.MjData, p):
        from robot_model import ball_names, guard_geoms, pad_geoms, slick_geoms
        self.m, self.d, self.p = m, d, p
        self.ball_geoms = [m.geom(n).id for n in ball_names(p)]
        self.is_ball = np.zeros(m.ngeom, dtype=bool)
        self.is_ball[self.ball_geoms] = True
        names = {m.geom(i).name: i for i in range(m.ngeom)}
        self.pad = {names[n] for n in pad_geoms(p) if n in names}
        self.hard_pairs = {names[n] for n in slick_geoms(p) + guard_geoms(p) if n in names}
        self.set_ball()
        # Maxwell arms on the squeeze slides
        self.slides = []
        for jn in ("squeeze_L", "squeeze_R", "feed_slide_L", "feed_slide_R"):
            try:
                j = m.joint(jn)
            except KeyError:
                continue
            self.slides.append((j.qposadr[0], j.dofadr[0], float(m.jnt_stiffness[j.id]) * (p.squeeze_dyn_ratio - 1.0)))
        self.x_d = np.array([d.qpos[q] for q, _, _ in self.slides])
        self._decay = math.exp(-m.opt.timestep / p.squeeze_relax)
        self._touched = {}

    def set_ball(self, mass: float | None = None, stiffness: float | None = None, e_slow: float | None = None,
                 e_fast: float | None = None, pad_scale: float = 1.0):
        """(Re)fit the laws, e.g. after a randomized study changed the ball. Unset values = the catalog ball's."""
        m, p = self.m, self.p
        b = p.ball_spec
        mass = b.mass if mass is None else mass
        k = b.stiffness if stiffness is None else stiffness
        es, ef = (b.e_slow if e_slow is None else e_slow), (b.e_fast if e_fast is None else e_fast)
        self.hard = fit(round(k, 3), round(mass, 6), round(es, 4), round(ef, 4))
        self.padlaw = fit(round(k, 3), round(mass, 6), round(min(0.95, p.pad_e_slow * pad_scale), 4),
                          round(min(0.95, p.pad_e_fast * pad_scale), 4))
        # target of each (ball geom, other geom) contact: an explicit pair, or the ball's own solref
        self.target = {}
        for k_ in range(m.npair):
            g1, g2 = int(m.pair_geom1[k_]), int(m.pair_geom2[k_])
            ball, other = (g1, g2) if self.is_ball[g1] else (g2, g1)
            if not self.is_ball[ball]:
                continue
            law = self.padlaw if other in self.pad else self.hard if other in self.hard_pairs else None
            if law is not None:
                self.target[(g1, g2)] = self.target[(g2, g1)] = ("pair", k_, law)
                m.pair_solref[k_] = law.solref(0.0)
        for g in self.ball_geoms:
            m.geom_solref[g] = self.hard.solref(0.0)

    def _target(self, g1: int, g2: int):
        t = self.target.get((g1, g2))
        if t is not None:
            return t
        # a default (non-pair) contact uses the ball's solref; skip contacts that come from other explicit pairs
        if (g1, g2) in self._other_pairs:
            return None
        g = g1 if self.is_ball[g1] else g2
        return ("geom", g, self.hard)

    @property
    def _other_pairs(self):
        if not hasattr(self, "_op"):
            m = self.m
            self._op = {(int(m.pair_geom1[k]), int(m.pair_geom2[k])) for k in range(m.npair)}
            self._op |= {(b, a) for a, b in self._op}
        return self._op

    def pre_step(self):
        m, d = self.m, self.d
        touched = {}
        n = d.ncon
        if n:
            geom = d.contact.geom
            dist = d.contact.dist
            for i in np.nonzero(self.is_ball[geom[:, 0]] | self.is_ball[geom[:, 1]])[0]:
                t = self._target(int(geom[i, 0]), int(geom[i, 1]))
                if t is None:
                    continue
                key = t[:2]
                pen = -float(dist[i])
                if pen > touched.get(key, (None, -1.0))[1]:
                    touched[key] = (t[2], pen)
        for key, (law, pen) in touched.items():
            ref = law.solref(pen)
            if key[0] == "pair":
                m.pair_solref[key[1]] = ref
            else:
                m.geom_solref[key[1]] = ref
        for key, law in self._touched.items():
            if key not in touched:
                (m.pair_solref if key[0] == "pair" else m.geom_solref)[key[1]] = law.solref(0.0)
        self._touched = {k: v[0] for k, v in touched.items()}
        # Maxwell arms: F = -k_m (x - x_d), the dashpot end x_d relaxing toward x with time constant tau
        for j, (qa, va, km) in enumerate(self.slides):
            x = d.qpos[qa]
            self.x_d[j] = x + (self.x_d[j] - x) * self._decay
            d.qfrc_applied[va] = -km * (x - self.x_d[j])


def attach(m, d, p):
    """The contact hooks for this model, or None for the elastic (v6 rounds 1-2) contacts."""
    return ViscoContacts(m, d, p) if p.contact_model == "viscoelastic" else None
