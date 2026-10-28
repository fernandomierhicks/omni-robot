"""How sure the robot is of where it is, and what it does about it (design.md 10).

Inputs, each optional (whatever the running localizers give):
  cov         the pose covariance (x, y, yaw) of the global estimate (the EKF with slam_toolbox's fixes, or tag SLAM)
  scan        the lidar match's quality: (inlier fraction, residual RMS m), and when the last accepted fix came
  tags        the last tag-implied pose and how far it sits from the estimate, in sigmas (NIS) and metres
  clock       the MCU clock map's mode ("pulse" / "echo" / "none"): without it every sensor time is a guess
  slip        the wheels' disagreement (m/s): odometry is worth less while it lasts

Output (assess): level OK / DEGRADED / POOR / LOST, a speed scale for every mode (Nav2's /speed_limit, the console's
caps), the reasons, and whether to relocalize. The rules (est, to tune on the robot):
  speed by position sigma   <= 5 cm: 1.0    <= 12 cm: 0.6    <= 25 cm: 0.3    beyond: 0 (stop)
  no accepted absolute fix (lidar or tag) for > 5 s while moving: one level down; > 15 s: POOR
  the lidar match poor (inliers < 50% or RMS > 4 cm) for 5 scans: one level down (an open room, people, a moved sofa)
  tags that agree with each other but not with the estimate (> 4 sigma, > 0.3 m, 3 detections in a row): the
  estimate is wrong -> relocalize from the tags (their map is the independent reference; the lidar alone can lock on
  the wrong room in a house of look-alike rooms)
  LOST (sigma > 25 cm, or POOR for > 3 s): stop, relocalize (the tags if one is in view, else v9's matcher's search
  around the last estimate), resume at 0.3 once a fix agrees
  clock not in "pulse" mode: the catch is held back (the console says so); driving isn't
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

LEVELS = ("OK", "DEGRADED", "POOR", "LOST")
SPEED = {"OK": 1.0, "DEGRADED": 0.6, "POOR": 0.3, "LOST": 0.0}


@dataclass
class HealthConfig:
    sigma_ok: float = 0.05
    sigma_degraded: float = 0.12
    sigma_poor: float = 0.25
    fix_age_degrade: float = 5.0
    fix_age_poor: float = 15.0
    scan_inliers: float = 0.5
    scan_rms: float = 0.04
    scan_bad_n: int = 5
    tag_nis: float = 16.0             # 4 sigma (x, y)
    tag_dist: float = 0.3
    tag_bad_n: int = 3
    poor_to_lost_s: float = 3.0


@dataclass
class Health:
    level: str
    speed_scale: float
    sigma_xy: float
    reasons: list = field(default_factory=list)
    relocalize: str | None = None     # None | "tags" | "search"
    catch_ok: bool = True

    def as_dict(self) -> dict:
        return dict(level=self.level, speed_scale=self.speed_scale, sigma_xy=round(self.sigma_xy, 4),
                    reasons=self.reasons, relocalize=self.relocalize, catch_ok=self.catch_ok)


class HealthMonitor:
    def __init__(self, cfg: HealthConfig = HealthConfig()):
        self.c = cfg
        self.t_fix = None
        self.scan_bad: deque = deque(maxlen=cfg.scan_bad_n)
        self.tag_bad: deque = deque(maxlen=cfg.tag_bad_n)
        self.t_poor = None
        self.last: Health | None = None

    def on_fix(self, t: float):
        """An absolute fix was accepted (a lidar match the filter took, or a tag factor in the graph)."""
        self.t_fix = t

    def on_scan(self, inliers: float, rms: float):
        self.scan_bad.append(inliers < self.c.scan_inliers or rms > self.c.scan_rms)

    def on_tag_pose(self, est, cov_est, tag_pose, tag_cov):
        """A pose implied by one tag of the map vs the current estimate."""
        d = np.asarray(tag_pose[:2], float) - np.asarray(est[:2], float)
        S = np.asarray(cov_est, float)[:2, :2] + np.asarray(tag_cov, float)[:2, :2]
        nis = float(d @ np.linalg.solve(S, d))
        self.tag_bad.append(nis > self.c.tag_nis and float(np.hypot(*d)) > self.c.tag_dist)

    def assess(self, t: float, cov, moving: bool = True, clock_mode: str | None = "pulse",
               slip: float = 0.0) -> Health:
        c = self.c
        reasons = []
        if cov is None:
            sig = math.inf
            reasons.append("no pose estimate")
        else:
            sig = math.sqrt(max(np.linalg.eigvalsh(np.asarray(cov, float)[:2, :2]).max(), 0.0))
        k = 0 if sig <= c.sigma_ok else 1 if sig <= c.sigma_degraded else 2 if sig <= c.sigma_poor else 3
        if k:
            reasons.append(f"position sigma {100 * sig:.0f} cm")
        age = math.inf if self.t_fix is None else t - self.t_fix
        if moving and age > c.fix_age_poor:
            k = max(k, 2)
            reasons.append(f"no absolute fix for {age:.0f} s")
        elif moving and age > c.fix_age_degrade:
            k = max(k, 1) if k < 3 else k
            reasons.append(f"no absolute fix for {age:.0f} s")
        if len(self.scan_bad) == self.scan_bad.maxlen and all(self.scan_bad):
            k = min(3, k + 1)
            reasons.append("lidar match poor")
        if slip > 0.15:
            k = max(k, 1)
            reasons.append("wheels slipping")
        relocalize = None
        if len(self.tag_bad) == self.tag_bad.maxlen and all(self.tag_bad):
            k = max(k, 2)
            reasons.append("the tags disagree with the estimate")
            relocalize = "tags"
        if k >= 2:
            self.t_poor = t if self.t_poor is None else self.t_poor
            if t - self.t_poor > c.poor_to_lost_s:
                k = 3
        else:
            self.t_poor = None
        if k == 3 and relocalize is None:
            relocalize = "search"
        level = LEVELS[k]
        catch_ok = clock_mode == "pulse" and k <= 1
        if clock_mode != "pulse":
            reasons.append(f"MCU clock map: {clock_mode or 'none'}")
        h = Health(level, SPEED[level], sig, reasons, relocalize, catch_ok)
        self.last = h
        return h

    def relocalized(self, t: float):
        """A relocalization was accepted: start over from it."""
        self.tag_bad.clear()
        self.scan_bad.clear()
        self.t_poor = None
        self.t_fix = t
