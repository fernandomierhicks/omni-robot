"""Online refinement of calibration parameters, bounded (design.md 10).

Some calibration moves while the robot runs: a printed camera bar flexes with temperature and knocks, the camera's
pipeline delay changes with load, the lidar's with its USB adapter. A slow estimator can follow them, but only under
rules, or it drifts wherever the estimator's other errors push it:

  * it starts from the bench value (the prior) and can never leave prior +/- bound: a value at the bound is reported
    ("recalibrate"), not used beyond it;
  * it moves only when the data can see it: each update brings information (H^T R^-1 H); updates are applied only
    once the window's information passes min_info in every direction (a straight drive says nothing about a time
    offset, a spin does), otherwise the parameter holds;
  * it moves slowly: a random walk of q per sqrt(s) (est from the bench repeats), and an innovation past gate sigma
    is refused (an outlier, a bump), counted, and half the recent updates refused flags it as inconsistent.

Mechanical flex that changes within a session faster than the walk (a knock that bends the bar) shows as an
inconsistency, not as a quiet bad estimate: the robot then slows down (omni_slam's health) and asks for a recalibration.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


class BoundedParam:
    def __init__(self, name: str, prior, prior_sigma, bound, q_per_sqrt_s, min_info: float, gate: float = 3.0,
                 window_s: float = 10.0):
        self.name = name
        self.prior = np.atleast_1d(np.asarray(prior, float))
        self.x = self.prior.copy()
        self.P = np.diag(np.atleast_1d(np.asarray(prior_sigma, float)) ** 2 * np.ones(self.prior.size))
        self.bound = np.atleast_1d(np.asarray(bound, float)) * np.ones(self.prior.size)
        self.q = np.atleast_1d(np.asarray(q_per_sqrt_s, float)) * np.ones(self.prior.size)
        self.min_info, self.gate, self.window_s = min_info, gate, window_s
        self.info: deque = deque()                    # (t, H^T R^-1 H)
        self.recent: deque = deque(maxlen=40)         # the last updates' outcomes: True = refused
        self.t = None
        self.n_used = self.n_held = self.n_gated = 0
        self.at_bound = False

    def _predict(self, t: float):
        if self.t is not None and t > self.t:
            self.P = self.P + np.diag(self.q ** 2 * (t - self.t))
        self.t = t

    def observable(self, t: float) -> bool:
        while self.info and self.info[0][0] < t - self.window_s:
            self.info.popleft()
        if not self.info:
            return False
        I = sum(i for _, i in self.info)
        return float(np.min(np.linalg.eigvalsh(np.atleast_2d(I)))) >= self.min_info

    def update(self, t: float, z, h_of_x, H, R) -> str:
        """z: the measurement, h_of_x: its prediction at the current x, H: dz/dx, R: its covariance. Returns what
        happened: "used" | "held" (not observable yet) | "gated" (refused as an outlier)."""
        self._predict(t)
        z, h = np.atleast_1d(np.asarray(z, float)), np.atleast_1d(np.asarray(h_of_x, float))
        H, R = np.atleast_2d(np.asarray(H, float)), np.atleast_2d(np.asarray(R, float))
        self.info.append((t, H.T @ np.linalg.inv(R) @ H))
        if not self.observable(t):
            self.n_held += 1
            return "held"
        y = z - h
        S = H @ self.P @ H.T + R
        if float(y @ np.linalg.solve(S, y)) > self.gate ** 2 * len(y):
            self.n_gated += 1
            self.recent.append(True)
            return "gated"
        self.recent.append(False)
        K = self.P @ H.T @ np.linalg.inv(S)
        x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P
        lo, hi = self.prior - self.bound, self.prior + self.bound
        self.at_bound = bool(np.any(x <= lo) or np.any(x >= hi))
        self.x = np.clip(x, lo, hi)
        self.n_used += 1
        return "used"

    @property
    def inconsistent(self) -> bool:
        """Most of the recent updates refused: the data and the parameter disagree (a knock, a moved mount)."""
        return len(self.recent) >= 20 and sum(self.recent) >= 0.5 * len(self.recent)

    def status(self) -> dict:
        return dict(name=self.name, value=[float(v) for v in self.x], prior=[float(v) for v in self.prior],
                    sigma=[float(math.sqrt(max(v, 0.0))) for v in np.diag(self.P)], used=self.n_used,
                    held=self.n_held, gated=self.n_gated, at_bound=self.at_bound, inconsistent=self.inconsistent)
