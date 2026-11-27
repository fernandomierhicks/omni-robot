"""LiDAR sample times and de-skew (design.md 28, 32). No ROS here; scan_prep and the tests use it.

Nov 17: the robot's lidar is the A2M8 and its driver is omni_cpp's rplidar_node (rplidar.hpp), which stamps each
sample from the device's own sample clock: the stamp is bin n - 1's time, the array runs against time (time_order -1,
robot.yaml), and ScanClock isn't needed. The text below is about sllidar_ros2's stamps, which ScanClock was for (and
the sim reproduces with lidar_stamp:=sllidar).

What the RPLIDAR A2M12 gives: no clock. Its serial stream (256 kbaud through the USB adapter) carries each sample's
angle and range and a start-of-turn flag; the driver (sllidar_ros2) stamps a turn with the host's clock when it
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


def floor_mask(a: np.ndarray, r: np.ndarray, cal: tuple, rp: tuple, margin: float,
               laser_xy: tuple = (0.0, 0.0)) -> np.ndarray:
    """Returns at or beyond the range where each ray meets the floor, less margin (scan_prep's floor filter).
    cal = (tan tilt, azimuth, height): the unit's measured plane tilt, its height above the floor at rest; rp = the
    base's (roll, pitch), scalars or one per sample (Nov 25: the base's at each sample's time, TiltHistory; a ramp
    swings the pitch ~20 deg in 0.3 s). laser_xy: the laser's offset from base_link; the base tilts about the floor
    point under base_link, so a laser ahead of it comes down nose-down (0.14 m ahead, 20 deg: 0.163 -> 0.105 m)."""
    tz, psi, h = cal
    d0, d1, d2 = np.cos(a), np.sin(a), tz * np.cos(a - psi)              # laser frame (= base frame axes)
    roll, pitch = (np.asarray(v, dtype=float) for v in rp)
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    w0 = cp * d0 + sp * sr * d1 + sp * cr * d2                           # Ry(pitch) Rx(roll)
    w1 = cr * d1 - sr * d2
    w2 = -sp * d0 + cp * sr * d1 + cp * cr * d2
    hz = -sp * laser_xy[0] + cp * sr * laser_xy[1] + cp * cr * h        # the laser's height, tilted
    slope = w2 / np.hypot(w0, w1)
    r_floor = np.where(slope < -1e-4, np.maximum(hz, 0.0) / np.maximum(-slope, 1e-4), np.inf)
    return np.isfinite(r) & (r >= r_floor - margin)


class TiltHistory:
    """The IMU's roll and pitch over the last `keep` seconds, read at any time: linear between the samples around it,
    the first / last one outside them, (0, 0) while empty. Samples out of order are dropped; one more than 1 s before
    the last (a clock jump) starts it over. omni_cpp scan.hpp's twin (scan_prep)."""

    def __init__(self, keep: float = 2.0):
        self.keep = keep
        self.h: deque = deque()

    def add(self, t: float, roll: float, pitch: float) -> None:
        if self.h and t <= self.h[-1][0]:
            if t >= self.h[-1][0] - 1.0:
                return
            self.h.clear()
        self.h.append((t, roll, pitch))
        while self.h[0][0] < t - self.keep:
            self.h.popleft()

    def at(self, t: float) -> tuple:
        h = self.h
        if not h:
            return 0.0, 0.0
        if t <= h[0][0]:
            return h[0][1], h[0][2]
        if t >= h[-1][0]:
            return h[-1][1], h[-1][2]
        lo, hi = 0, len(h) - 1                   # the last sample at or before t
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        p, q = h[lo], h[lo + 1]
        f = (t - p[0]) / (q[0] - p[0])
        return p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2])


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


def _line(k: np.ndarray, t: np.ndarray) -> tuple[float, float]:
    """Least-squares t = t0 + T k about the means: np.polyfit on raw stamps (~1.7e9 s) loses ~1e-8 s of T."""
    km, tm = k.mean(), t.mean()
    T = float(np.sum((k - km) * (t - tm)) / np.sum((k - km) ** 2))
    return T, float(tm - T * km)


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
        if self.t_last is None:
            self.k = 0
        else:
            T = self.T or (scan_time if scan_time > 0 else None)
            dk = int(round((stamp - self.t_last) / T)) if T else 0
            if dk < 1 or dk > 50:                      # out of order, a long gap or no period: start a new line
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
            T, t0 = _line(k, t)
            for _ in range(2):                         # the line under the stamps: shift down to their low quantile
                res = t - (t0 + T * k)
                lo = res <= np.quantile(res, 0.5)
                if lo.sum() >= 3:
                    T, t0 = _line(k[lo], t[lo])
            res = t - (t0 + T * k)
            t0 += float(np.quantile(res, self.q))
            self.line, self.T = (float(t0), float(T)), float(T)
            self.jitter = float(np.std(res))
            return float(t0 + T * self.k) - self.latency
        self.T = self.T or (scan_time if scan_time > 0 else None)
        return stamp - self.latency
