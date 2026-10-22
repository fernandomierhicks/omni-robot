"""Quasi-static traction and tip-over model for an N-wheel omni base.

Used three ways: by the wheel-count / tip-over studies (compared against the
sim), by the firmware-side direction-aware acceleration limit, and by the
load-aware torque allocation of a 4-wheel base (robot.py).

Conventions: robot frame, x forward, y left, wheel i at angle th_i and radius L,
rolling direction t_i = (-sin th_i, cos th_i). A wheel can only push along t_i;
the roller direction is free. The CG sits at (cx, cy) and height h.

Normal loads. Accelerating at a in direction u is, for the normal loads, the
same as moving the CG to c - (h a / g) u (d'Alembert). Three wheels are
statically determinate. Four wheels on a rigid chassis are not: the loads then
depend on the wheels' vertical compliance, modeled here as equal springs under
a rigid chassis on a flat floor (min sum N^2). A "rocker" puts two wheels on a
beam pivoting midway between them, which makes the base determinate again.

Traction. The wheel forces f must produce m a u and the yaw torque the inertial
force makes about the base center when the CG is off-center:
    G f = (m a u_x, m a u_y, m (c x a u)_z),   G = [t_x; t_y; L]
 - "pinv": f = pinv(G) w, what equal speed loops on every wheel naturally do;
 - "optimal": any f in G f = w (N > 3 leaves a null space), chosen to keep
   every wheel inside its friction limit (load-aware allocation).
A wheel slips when |f_i| > mu N_i. The base tips when the shifted CG leaves the
support polygon.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

G0 = 9.81


@dataclass
class Base:
    angles_deg: tuple
    L: float                     # wheel radius from the base center
    h: float                     # CG height
    cx: float = 0.0
    cy: float = 0.0
    mu: float = 0.95
    rocker: tuple | None = None  # indices of the two wheels on a rocker beam

    def __post_init__(self):
        th = np.radians(np.asarray(self.angles_deg, dtype=float))
        self.th = th
        self.p = self.L * np.column_stack([np.cos(th), np.sin(th)])
        self.t = np.column_stack([-np.sin(th), np.cos(th)])
        self.G = np.vstack([self.t.T, np.full(len(th), self.L)])
        self.G_pinv = np.linalg.pinv(self.G)
        _, s, vt = np.linalg.svd(self.G)
        self.null = vt[3:].T if len(th) > 3 else np.zeros((len(th), 0))
        # support polygon (convex hull of the contact points, counter-clockwise)
        order = np.argsort(np.arctan2(self.p[:, 1], self.p[:, 0]))
        self.hull = self.p[order]

    @property
    def n(self) -> int:
        return len(self.th)

    # -- loads ----------------------------------------------------------------
    def _spring_loads(self, pts: np.ndarray, c_eff: np.ndarray) -> np.ndarray:
        """Min-norm loads (fractions of m g) on points pts that put the resultant at c_eff."""
        P = np.column_stack([np.ones(len(pts)), pts])
        return P @ np.linalg.lstsq(P.T @ P, np.array([1.0, *c_eff]), rcond=None)[0]

    def loads(self, a: float, u) -> np.ndarray:
        """Normal loads as fractions of m g; wheels that would pull are lifted (0)."""
        c_eff = np.array([self.cx, self.cy]) - self.h * a / G0 * np.asarray(u, dtype=float)
        if self.rocker is not None:
            i, j = self.rocker
            others = [k for k in range(self.n) if k not in (i, j)]
            pivot = 0.5 * (self.p[i] + self.p[j])
            pts = np.vstack([self.p[others], pivot])
            if _inside(c_eff, pts):            # rocker floats: equal split between its two wheels
                nl = self._spring_loads(pts, c_eff)
                out = np.zeros(self.n)
                out[others] = nl[:-1]
                out[[i, j]] = nl[-1] / 2
                return np.maximum(out, 0.0)
            # outside the floating triangle the rocker sits on its end stop: rigid base
        active = list(range(self.n))
        for _ in range(self.n):
            nl = self._spring_loads(self.p[active], c_eff)
            if np.all(nl >= -1e-12) or len(active) <= 3:
                break
            active.pop(int(np.argmin(nl)))
        out = np.zeros(self.n)
        out[active] = np.maximum(nl, 0.0)
        return out

    # -- limits ---------------------------------------------------------------
    def tip_accel(self, u) -> float:
        """Acceleration at which the shifted CG reaches the support-polygon edge."""
        u = np.asarray(u, dtype=float)
        c = np.array([self.cx, self.cy])
        reach = float(np.max(self.hull @ (-u)) + c @ u)     # support distance behind the CG
        return G0 * reach / self.h

    def wrench(self, a: float, u) -> np.ndarray:
        u = np.asarray(u, dtype=float)
        tau = self.cx * a * u[1] - self.cy * a * u[0]
        return np.array([a * u[0], a * u[1], tau])          # per unit mass

    def forces(self, a: float, u, allocation: str = "pinv", loads=None) -> np.ndarray:
        """Wheel forces per unit mass (N/kg) for acceleration a along u."""
        w = self.wrench(a, u)
        f = self.G_pinv @ w
        if allocation == "pinv" or self.null.shape[1] == 0:
            return f
        nl = self.loads(a, u) if loads is None else loads
        cap = np.maximum(self.mu * G0 * nl, 1e-9)
        return f + self.null @ _best_null(f, self.null, cap)

    def utilization(self, a: float, u, allocation: str = "pinv") -> float:
        """max |f_i| / (mu N_i): 1.0 means the first wheel is about to slip."""
        nl = self.loads(a, u)
        f = self.forces(a, u, allocation, nl)
        cap = self.mu * G0 * nl
        with np.errstate(divide="ignore"):
            r = np.where(cap > 1e-9, np.abs(f) / np.maximum(cap, 1e-12), np.where(np.abs(f) > 1e-9, np.inf, 0.0))
        return float(np.max(r))

    def slip_accel(self, u, allocation: str = "pinv", a_hi: float = 30.0) -> float:
        lo, hi = 0.0, a_hi
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.utilization(mid, u, allocation) <= 1.0 and mid < self.tip_accel(u):
                lo = mid
            else:
                hi = mid
        return lo

    def limit(self, direction_deg: float, allocation: str = "pinv") -> dict:
        u = np.array([math.cos(math.radians(direction_deg)), math.sin(math.radians(direction_deg))])
        tip = self.tip_accel(u)
        slip = self.slip_accel(u, allocation)
        # first wheel to lift (rigid springs: the support shrinks before the base tips)
        lo, hi = 0.0, tip
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if np.all(self.loads(mid, u) > 1e-9) else (lo, mid)
        return dict(direction=direction_deg, slip=slip, tip=tip, lift=lo,
                    limited_by="tip" if slip >= tip - 1e-6 else "slip")

    def polar(self, directions, allocation: str = "pinv") -> list[dict]:
        return [self.limit(d, allocation) for d in directions]


def _best_null(f0: np.ndarray, null: np.ndarray, cap: np.ndarray) -> np.ndarray:
    """Null-space coefficients minimizing max |f_i| / cap_i (1-D null space: ternary search)."""
    if null.shape[1] == 1:
        nv = null[:, 0]
        span = float(np.max(np.abs(f0))) / max(float(np.min(np.abs(nv[np.abs(nv) > 1e-9]))), 1e-9) + 1e-9

        def cost(c):
            return float(np.max(np.abs(f0 + c * nv) / cap))
        lo, hi = -span, span
        for _ in range(60):
            m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            if cost(m1) < cost(m2):
                hi = m2
            else:
                lo = m1
        return np.array([0.5 * (lo + hi)])
    from scipy.optimize import linprog          # general case: minimize s s.t. |f0 + N c| <= s cap
    k = null.shape[1]
    A = np.vstack([np.column_stack([null, -cap]), np.column_stack([-null, -cap])])
    b = np.concatenate([-f0, f0])
    res = linprog(np.r_[np.zeros(k), 1.0], A_ub=A, b_ub=b, bounds=[(None, None)] * k + [(0, None)])
    return res.x[:k] if res.success else np.zeros(k)


def _inside(pt, pts) -> bool:
    """Is pt inside the convex hull of pts (counter-clockwise ordering by angle)?"""
    c = pts.mean(axis=0)
    order = np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    q = pts[order]
    for k in range(len(q)):
        a, b = q[k], q[(k + 1) % len(q)]
        if (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0]) < 0:
            return False
    return True


LAYOUTS = {
    "kiwi3": (60.0, 180.0, 300.0),        # v2: a wheel at the back, the launcher between two wheels
    "x4": (45.0, 135.0, 225.0, 315.0),    # X drive: wheels on the diagonals
}


if __name__ == "__main__":
    dirs = list(range(0, 360, 15))
    for name, ang in LAYOUTS.items():
        for L in (0.13, 0.15, 0.18):
            for h in (0.13, 0.15):
                b = Base(ang, L, h)
                rows = b.polar(dirs)
                opt = b.polar(dirs, "optimal") if len(ang) > 3 else rows
                print(f"{name} L={L:.2f} h={h:.2f}: slip min {min(r['slip'] for r in rows):.2f} "
                      f"mean {np.mean([r['slip'] for r in rows]):.2f} | optimal min {min(r['slip'] for r in opt):.2f} "
                      f"mean {np.mean([r['slip'] for r in opt]):.2f} | tip min {min(r['tip'] for r in rows):.2f} "
                      f"| lift min {min(r['lift'] for r in rows):.2f} | tip-limited dirs "
                      f"{[r['direction'] for r in rows if r['limited_by'] == 'tip']}")
