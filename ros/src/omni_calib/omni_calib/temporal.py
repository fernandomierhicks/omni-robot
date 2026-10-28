"""Time offsets and rotations between sensors from how they see the robot turn (design.md 10).

Every sensor on the base turns with it. A gyro measures the rate; a camera watching a fixed target (or tags) gets it
from consecutive poses; the lidar from consecutive scans' alignment. The same motion stamped by two clocks (or two
pipelines) appears shifted by their time offset:

  time_offset(t_a, x_a, t_b, x_b)   the shift d with x_b(t + d) = s x_a(t) (s: a scale, also fitted), from the least
                                    squares of the two signals over a lag search; its 1-sigma from the fit's
                                    curvature. Signals: |rate| (no extrinsics needed), or one axis
  rotation_from_rates(w_a, w_b)     R with w_b = R w_a (Kabsch / Procrustes): e.g. the camera's axes in the IMU's
  rates_from_rotations(t, Rs)       a sensor's angular rate in its own frame from its orientations

Observability: the offset only shows when the rate changes (a constant spin hides it: every lag fits). The motion
for it: a few seconds of back-and-forth yaw (the Calibrate tab's "wiggle", +/-2 rad/s at ~1 Hz) and, for the camera's
other axes, the robot rocked by hand. The result says how well determined it was (sigma) and refuses a weak one.
"""

from __future__ import annotations

import math

import numpy as np

from omni_calib.geometry import log_rot


def rates_from_rotations(t, Rs):
    """t (n,), Rs (n, 3, 3) = the sensor's orientation in a fixed frame -> (t_mid (n-1,), w (n-1, 3) in the sensor's
    own frame, rad/s)."""
    t = np.asarray(t, float)
    w = np.array([log_rot(Rs[k].T @ Rs[k + 1]) / (t[k + 1] - t[k]) for k in range(len(t) - 1)])
    return (t[:-1] + t[1:]) / 2, w


def _sse(t_a, x_a, t_b, x_b, d, fit_scale=True):
    xb = np.interp(t_a + d, t_b, x_b, left=np.nan, right=np.nan)
    ok = np.isfinite(xb)
    if ok.sum() < 10:
        return math.inf, 1.0, 0
    a, b = x_a[ok], xb[ok]
    s = float(np.dot(a, b) / max(np.dot(a, a), 1e-12)) if fit_scale else 1.0
    r = b - s * a
    return float(np.dot(r, r)), s, int(ok.sum())


def time_offset(t_a, x_a, t_b, x_b, max_lag: float = 0.05, step: float | None = None, fit_scale: bool = True,
                max_sigma: float = 0.5e-3) -> dict:
    """d (s) such that the event stamped t in a is stamped t + d in b. Returns dict(offset, sigma, scale, rms, n,
    observable): observable = a minimum inside the search, a signal that changes, and sigma under max_sigma."""
    t_a, x_a, t_b, x_b = (np.asarray(v, float) for v in (t_a, x_a, t_b, x_b))
    step = step or max(1e-4, float(np.median(np.diff(t_a))) / 4)
    lags = np.arange(-max_lag, max_lag + step / 2, step)
    sse = np.array([_sse(t_a, x_a, t_b, x_b, d, fit_scale)[0] for d in lags])
    k = int(np.argmin(sse))
    f = lambda d: _sse(t_a, x_a, t_b, x_b, d, fit_scale)[0]       # noqa: E731
    a, b = lags[max(k - 1, 0)], lags[min(k + 1, len(lags) - 1)]
    g = (math.sqrt(5) - 1) / 2                                    # golden section between the grid's neighbours
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    while b - a > step * 1e-3:
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    d0 = (a + b) / 2
    e0, s, n = _sse(t_a, x_a, t_b, x_b, d0, fit_scale)
    h = step
    e_p, e_m = _sse(t_a, x_a, t_b, x_b, d0 + h, fit_scale)[0], _sse(t_a, x_a, t_b, x_b, d0 - h, fit_scale)[0]
    curv = (e_p - 2 * e0 + e_m) / h ** 2
    var_r = e0 / max(n - 2, 1)
    sigma = math.sqrt(2 * var_r / curv) if curv > 0 else math.inf
    # what the motion allows at best: residual / sqrt(sum of the rate's change^2), the change taken over 50 ms so the
    # sensor's noise doesn't pass for motion (a constant spin has none: no offset can be seen in it)
    k50 = max(1, int(round(0.05 / max(float(np.median(np.diff(t_b))), 1e-6))))
    sm = np.convolve(x_b, np.ones(k50) / k50, mode="same")
    rate = np.zeros_like(sm)
    rate[k50:-k50] = (sm[2 * k50:] - sm[:-2 * k50]) / (t_b[2 * k50:] - t_b[:-2 * k50])
    dx = np.interp(t_a + d0, t_b, rate)
    sigma = max(sigma, math.sqrt(var_r) / max(float(np.sqrt(np.sum((s * dx) ** 2))), 1e-12))
    edge = k in (0, len(lags) - 1)
    return dict(offset=float(d0), sigma=float(sigma), scale=float(s), rms=float(math.sqrt(var_r)), n=n,
                observable=bool(not edge and sigma < max_sigma))


def rotation_from_rates(w_a, w_b) -> tuple:
    """R with w_b ~ R w_a (rows are time-aligned samples). Returns (R, rms rad/s, conditioning): the conditioning
    (smallest / largest singular value of the rates' cross-covariance) says whether all three axes were excited."""
    A, B = np.asarray(w_a, float), np.asarray(w_b, float)
    M = B.T @ A
    U, S, Vt = np.linalg.svd(M)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    rms = float(np.sqrt(np.mean(np.sum((B - A @ R.T) ** 2, axis=1))))
    return R, rms, float(S[-1] / max(S[0], 1e-12))
