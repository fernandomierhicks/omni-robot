"""Will the throw make it into the bucket, and how early is that known? (offline, on a recorded RealSense throw)

The bucket is a rim: a circle of radius R around `rim_center`, in the plane with normal `up`, all in the camera's optical frame
(x right, y down, z forward; the depth / IR1 imager's origin). `bucket.json` (from your CAD, see Bucket.from_pose) holds it.

  reference   the arc through ALL of the throw's detections (gravity fixed to `up`) and where it crosses the rim plane going down:
              the radial miss distance d at the crossing, in (d < R), rim (|d - R| < ball radius: touches the rim) or out.
              This is the throw's own answer, no label: it carries the camera's and the mount's systematic errors identically, so it
              tests how soon the estimate settles, not absolute accuracy.
  causal      the same arc refitted after each detection using only the detections so far (the association is the offline fit's:
              an oracle, so wrong-blob rejection is not tested here), and its predicted crossing and verdict.
  time to correct   the time to go (reference crossing time - now) at the earliest detection from which the verdict never again
              differs from the reference. Near-zero margins can't be called either way: they're flagged, not counted as errors.

  python rs_analyze.py data/<session> --bucket bucket.json      (this is called from there)
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import rs_detect as D
from arcfit import G, integrate


@dataclass
class Bucket:
    rim_center: np.ndarray             # m, camera optical frame
    up: np.ndarray                     # unit, the rim plane's normal pointing up, camera frame
    radius: float = 0.165              # rim (inner) radius, m
    ball_radius: float = 0.037

    @classmethod
    def from_pose(cls, rim_dx: float, rim_dy: float, rim_dz: float, pitch_deg: float, radius: float = 0.165, ball_radius: float = 0.037) -> "Bucket":
        """From the CAD: the rim centre's offset from the camera (the depth / IR1 optical centre) in a LEVEL frame, x to the right,
        y forward (toward the thrower), z up, metres; the camera looks up by `pitch_deg` (no roll)."""
        s, c = math.sin(math.radians(pitch_deg)), math.cos(math.radians(pitch_deg))
        return cls(np.array([rim_dx, s * rim_dy - c * rim_dz, c * rim_dy + s * rim_dz]), np.array([0.0, -c, s]), radius, ball_radius)

    @classmethod
    def from_cad(cls, rim_diameter: float, rim_height: float, back_horizontal: float, back_height: float, pitch_deg: float,
                 origin_forward: float = 0.0215, origin_lateral: float = -0.025, origin_up: float = 0.0, ball_radius: float = 0.037) -> "Bucket":
        """From the bucket + camera mount as drawn. The camera hangs on the bucket's outside and looks away from it, pitched up:
        back_horizontal / back_height = the camera's back-face middle point from the bucket's axis (horizontal) and from the floor;
        rim_height = the inner rim's height above the floor. origin_* = where the depth origin (the LEFT IR imager's optical centre)
        sits relative to the back-face middle point, in the camera body's own axes (forward, to its right, up): (est) the lens is
        ~21.5 mm in front of the back face (25.05 mm body, origin ~3.5 mm behind the glass) and the imager 25 mm left of the housing's
        centre (the IR pair's 50 mm baseline, assumed centred): VERIFY on Intel's mechanical drawing; the lateral term moves the
        predicted miss one-for-one."""
        s, c = math.sin(math.radians(pitch_deg)), math.cos(math.radians(pitch_deg))
        # level frame: x right (of the camera), y toward the thrower (away from the bucket), z up; origin on the bucket's axis, at the floor
        back = np.array([0.0, back_horizontal, back_height])
        fwd, up_b = np.array([0.0, c, s]), np.array([0.0, -s, c])
        origin = back + origin_forward * fwd + origin_lateral * np.array([1.0, 0.0, 0.0]) + origin_up * up_b
        rim = np.array([0.0, 0.0, rim_height]) - origin
        return cls.from_pose(rim[0], rim[1], rim[2], pitch_deg, rim_diameter / 2, ball_radius)

    @classmethod
    def load(cls, path) -> "Bucket":
        """bucket.json: {"cad": {...}} (see bucket.json and from_cad), or {"rim_center_cam": [x, y, z], "up_cam": [x, y, z]}, or
        {"pose": {"dx":, "dy":, "dz":, "pitch_deg":}}; "rim_radius_m", "ball_radius_m" optional."""
        d = json.loads(Path(path).read_text())
        r, br = float(d.get("rim_radius_m", 0.165)), float(d.get("ball_radius_m", 0.037))
        if "cad" in d:
            c = d["cad"]
            o = c.get("origin_from_back_face_m", {})
            return cls.from_cad(c["rim_diameter_m"], c["rim_height_m"], c["camera_back_horizontal_m"], c["camera_back_height_m"], c["pitch_deg"],
                                o.get("forward", 0.0215), o.get("lateral", -0.025), o.get("up", 0.0), br)
        if "pose" in d:
            p = d["pose"]
            return cls.from_pose(p["dx"], p["dy"], p["dz"], p["pitch_deg"], r, br)
        up = np.array(d["up_cam"], float)
        return cls(np.array(d["rim_center_cam"], float), up / np.linalg.norm(up), r, br)

    def radial(self, p) -> float:
        """Distance of p from the rim's axis, m."""
        v = np.asarray(p, float) - self.rim_center
        return float(np.linalg.norm(v - (v @ self.up) * self.up))

    def verdict(self, d: float) -> str:
        return "in" if d < self.radius - self.ball_radius else ("rim" if d <= self.radius + self.ball_radius else "out")


def crossing(fit: D.ArcFit, b: Bucket, t_from: float, t_max: float = 1.5, dt: float = 0.002):
    """(time, position, radial miss d) where the arc next crosses the rim plane going down (from above it), or None."""
    ts = np.arange(t_from, t_from + t_max, dt)
    P = fit.at(ts)
    h = (P - b.rim_center) @ b.up
    for i in range(1, len(ts)):
        if h[i - 1] > 0 >= h[i]:
            f = h[i - 1] / (h[i - 1] - h[i])
            p = P[i - 1] + f * (P[i] - P[i - 1])
            return float(ts[i - 1] + f * dt), p, b.radial(p)
    return None


def fit_fixed_gravity(t: np.ndarray, P: np.ndarray, up: np.ndarray, ball, cfg: D.DetectCfg, x0=None, t_ref=None):
    """Drag-law arc through the points with gravity = -G up (direction known): 6 free numbers, warm-startable. Returns
    (ArcFit, x) or (None, None)."""
    from scipy.optimize import least_squares
    t, P = np.asarray(t, float), np.asarray(P, float)
    t_ref = float(t[0]) if t_ref is None else t_ref
    g = -G * np.asarray(up, float) / np.linalg.norm(up)
    sig = cfg.sigma0 + cfg.sigma_z2 * np.maximum(P[:, 2], 0.3) ** 2
    if x0 is None:
        dt = max(t[-1] - t[0], 1e-3)
        v0 = (P[-1] - P[0]) / dt - 0.5 * g * dt
        x0 = np.concatenate([P[0], v0])

    def resid(x):
        return ((integrate(x[0:3], x[3:6], g, t, ball, step=0.01) - P) / sig[:, None]).ravel()
    try:
        sol = least_squares(resid, x0, loss="soft_l1", f_scale=2.0, max_nfev=25)
    except Exception:
        return None, None
    res = np.linalg.norm(integrate(sol.x[0:3], sol.x[3:6], g, t, ball, step=0.01) - P, axis=1)
    return D.ArcFit(t_ref, sol.x[0:3], sol.x[3:6], g, float(np.sqrt(np.mean(res ** 2))), np.ones(len(t), bool), len(t), g=g, ball=ball), sol.x


def ray_fitter(obs: dict, up: np.ndarray, ball):
    """fit(k, x0) for predict_throw from a colour source's observations (pixels + ranges, rs_analyze.colour_source): the same
    fixed-gravity drag arc, fitted to the first k pixels and ranges (fit_ray_arc)."""
    def fit(k, x0=None):
        return D.fit_ray_arc(obs["t"][:k], obs["uv"][:k], obs["z"][:k], obs["sig_z"][:k], obs["ic"], obs["R"], obs["tr"], ball, up=up,
                             x0=x0, sig_px=obs["sig_px"][:k])
    return fit


def predict_throw(t: np.ndarray, P: np.ndarray, b: Bucket, cfg: D.DetectCfg, up: np.ndarray | None = None, min_pts: int = 6, stride: int = 1,
                  fitter=None) -> dict:
    """The reference and the causal series for one throw's detections (t ascending, P Nx3, the offline fit's inliers). fitter(k, x0) ->
    (ArcFit, x) fits the first k detections (default: fit_fixed_gravity on P; a colour source passes ray_fitter)."""
    up = b.up if up is None else up
    t, P = np.asarray(t, float), np.asarray(P, float)
    if fitter is None:
        def fitter(k, x0=None):
            return fit_fixed_gravity(t[:k], P[:k], up, cfg.ball, cfg, x0=x0, t_ref=float(t[0]))
    ref, x = fitter(len(t))
    if ref is None:
        return dict(ok=False, reason="reference fit failed")
    cr = crossing(ref, b, float(t[0]), t_max=float(t[-1] - t[0]) + 1.5)
    if cr is None:
        return dict(ok=False, reason="the reference arc never comes down through the rim plane")
    t_c, p_c, d_c = cr
    out = dict(ok=True, reference=dict(verdict=b.verdict(d_c), in_rim=bool(d_c < b.radius), radial_m=d_c, margin_m=d_c - b.radius,
                                       t_cross=t_c, last_detection=float(t[-1]), extrapolated_s=float(t_c - t[-1]),
                                       position_cam=p_c.tolist(), rms_m=ref.rms))
    series, warm = [], None
    for k in range(min_pts, len(t) + 1, stride):
        arc, warm = fitter(k, warm)
        if arc is None:
            continue
        c = crossing(arc, b, float(t[k - 1]))
        if c is None:
            continue
        series.append(dict(t=float(t[k - 1]), n=k, ttg=float(t_c - t[k - 1]), t_cross=c[0], radial_m=c[2], in_rim=bool(c[2] < b.radius),
                           verdict=b.verdict(c[2]), error_m=float(np.linalg.norm(c[1] - p_c))))
    out["series"] = series
    if series:
        want = out["reference"]["in_rim"]
        ok_ = [s["in_rim"] == want for s in series]
        k = len(ok_)
        while k > 0 and ok_[k - 1]:
            k -= 1                                                # k = the last wrong one: the verdict holds from series[k] on
        out["first_prediction_ttg"] = series[0]["ttg"]
        out["always_correct"] = k == 0
        out["settled"] = k < len(series)                           # false: still wrong at the last detection
        out["time_to_correct_ttg"] = series[k]["ttg"] if k < len(series) else None
        out["time_to_correct_t"] = series[k]["t"] if k < len(series) else None
        out["wrong_predictions"] = int(len(ok_) - sum(ok_))
        out["ambiguous"] = bool(abs(out["reference"]["margin_m"]) < b.ball_radius)      # touches the rim: the reference itself is a coin flip
    return out


def describe(p: dict) -> str:
    if not p.get("ok"):
        return "  prediction: " + p.get("reason", "none")
    r = p["reference"]
    ext = r["extrapolated_s"]
    where = (f"extrapolated {1000 * ext:.0f} ms past the last detection" if ext >= 0 else f"seen {-1000 * ext:.0f} ms before the last detection")
    s = f"  prediction: reference {r['verdict'].upper()} (miss {100 * r['margin_m']:+.1f} cm from the rim, crossing at {r['t_cross']:.2f} s, {where})"
    if "time_to_correct_ttg" in p:
        if p["time_to_correct_ttg"] is None:
            s += "; never settled"
        else:
            s += (f"; first call {1000 * p['first_prediction_ttg']:.0f} ms before the crossing, correct from {1000 * p['time_to_correct_ttg']:.0f} ms before it"
                  + (" (always right)" if p["always_correct"] else f" ({p['wrong_predictions']} wrong on the way)")
                  + ("; AMBIGUOUS: the reference touches the rim" if p["ambiguous"] else ""))
    return s
