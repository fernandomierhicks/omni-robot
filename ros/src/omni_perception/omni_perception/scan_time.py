"""LiDAR sample times and de-skew (design.md 28, 32). No ROS here; scan_prep and the tests use it.

What the RPLIDAR A2M12 gives: no clock. Its serial stream (256 kbaud through the USB adapter) carries each sample's
angle and range and a start-of-turn flag; the driver (sllidar_ros2) stamps a turn with the Jetson's clock when it
starts reading it, takes the turn's duration from its own clock, and sets time_increment = duration / (n - 1). So:

  * every sample's time is reconstructed, not measured: t_i = stamp + i * time_increment, assuming the head turns at
    a constant rate during the turn (its motor is speed-controlled; the turn-to-turn period varies < 1%, est);
  * the stamp carries the serial and USB delivery delay (est 1-3 ms, with jitter), and the driver's index order may run
    against time: it flips the array so angles increase counter-clockwise (REP-103) while the head turns clockwise,
    so time_order = -1 (VERIFY on the real driver: its "inverted" / "angle_compensate" settings decide it);
  * angle_compensate re-bins samples to fixed angles: the bin's time is still the head's time at that angle.

ScanClock removes the delivery jitter from the stamps: a turn starts every T (the motor's period), so the start
times lie on a line; the stamps are that line plus a delay that is never negative. Fitting the line through the
stamps' lower envelope and subtracting the bench-measured fixed delay (latency_s: omni_calib.temporal from a spin)
gives each turn's start to ~0.1 ms instead of the stamp's jitter.

deskew() moves each sample into the laser frame at one reference time with the robot's motion (odom -> base_link
at the sample's own time), then re-bins by angle (nearest return kept), as scan_prep did inline and v9's matcher does.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


def sample_times(n: int, stamp: float, time_increment: float, order: int = 1) -> np.ndarray:
    """Each index's time. order = -1: the array runs against time (the last index was measured first)."""
    k = np.arange(n, dtype=float)
    if order < 0:
        k = (n - 1) - k
    return stamp + k * time_increment


def deskew(a: np.ndarray, r: np.ndarray, ts: np.ndarray, pose_at, laser_xy=(0.0, 0.0), t_ref: float | None = None,
           angle_min: float = 0.0, angle_increment: float | None = None, n_out: int | None = None,
           n_key: int = 11) -> np.ndarray:
    """Ranges as seen from the laser at t_ref (default: the earliest sample's time). a, r, ts: angle (laser frame),
    range (inf / nan = none), time of each sample. pose_at(t) -> (x, y, yaw) of base_link in odom at time t (called
    n_key times across the scan, interpolated in between). The laser's axes are the base's (planar); laser_xy is its
    origin on the base."""
    n = len(r)
    n_out = n_out or n
    inc = angle_increment if angle_increment is not None else 2 * math.pi / n_out
    t0, t1 = float(np.min(ts)), float(np.max(ts))
    t_ref = t0 if t_ref is None else t_ref
    tk = np.linspace(min(t0, t_ref), max(t1, t_ref), n_key)
    K = np.array([pose_at(float(t)) for t in tk], dtype=float)
    K[:, 2] = np.unwrap(K[:, 2])
    px, py, pyaw = (np.interp(ts, tk, K[:, j]) for j in range(3))
    rx, ry, ryaw = (float(np.interp(t_ref, tk, K[:, j])) for j in range(3))
    ok = np.isfinite(r)
    lx, ly = laser_xy
    bx = lx + r[ok] * np.cos(a[ok])                      # base frame at the sample's time
    by = ly + r[ok] * np.sin(a[ok])
    c, s = np.cos(pyaw[ok]), np.sin(pyaw[ok])
    wx, wy = px[ok] + c * bx - s * by, py[ok] + s * bx + c * by       # odom frame
    c0, s0 = math.cos(ryaw), math.sin(ryaw)
    qx = c0 * (wx - rx) + s0 * (wy - ry) - lx                          # laser at t_ref
    qy = -s0 * (wx - rx) + c0 * (wy - ry) - ly
    rr = np.hypot(qx, qy)
    aa = np.mod(np.arctan2(qy, qx) - angle_min, 2 * math.pi)
    k = np.round(aa / inc).astype(int) % n_out
    out = np.full(n_out, np.inf)
    np.minimum.at(out, k, rr)
    return out


class ScanClock:
    """Turn-start times from the driver's stamps: the lower envelope of the stamps on a line t_k = t0 + k T, minus
    the fixed delivery delay latency_s. update(stamp, scan_time) -> the corrected stamp."""

    def __init__(self, latency_s: float = 0.0, window: int = 100, envelope_q: float = 0.03):
        self.latency = latency_s
        self.window, self.q = window, envelope_q
        self.hist: deque = deque(maxlen=window)        # (k, stamp)
        self.k = None
        self.t_last = None
        self.T = None
        self.line = None                               # (t0, T): stamp_k ~ t0 + k T + delay >= 0
        self.jitter = math.nan

    def update(self, stamp: float, scan_time: float) -> float:
        if self.t_last is None or scan_time <= 0:
            self.k = 0
        else:
            T = self.T or scan_time
            dk = int(round((stamp - self.t_last) / T))
            if dk < 1 or dk > 50:                      # out of order or a long gap: start a new line
                self.hist.clear()
                self.k = 0
                self.line = None
            else:
                self.k += dk
        self.t_last = stamp
        self.hist.append((self.k, stamp))
        if len(self.hist) >= 5:
            k = np.array([h[0] for h in self.hist], dtype=float)
            t = np.array([h[1] for h in self.hist])
            T, t0 = np.polyfit(k, t, 1)
            for _ in range(2):                         # the line under the stamps: shift down to their low quantile
                res = t - (t0 + T * k)
                lo = res <= np.quantile(res, 0.5)
                if lo.sum() >= 3:
                    T, t0 = np.polyfit(k[lo], t[lo], 1)
            res = t - (t0 + T * k)
            t0 += float(np.quantile(res, self.q))
            self.line, self.T = (float(t0), float(T)), float(T)
            self.jitter = float(np.std(res))
            return float(t0 + T * self.k) - self.latency
        self.T = self.T or (scan_time if scan_time > 0 else None)
        return stamp - self.latency
