"""Is this ball-coloured blob a thrown ball? Blob tracks judged against a ballistic arc in the image (numpy only: it runs in the Pi's
recorder as the trigger and offline in rs_analyze as the colour source's association, the same code).

A ball in free flight traces u(t), v(t) that a quadratic in time follows to a pixel or two over a few hundred ms (perspective bends it
slowly, so the judge looks at the last `window` points), it travels several ball diameters, it moves fast (the lateral speed comes from
its own size: R / r_px metres per pixel, no range or calibration needed), its image acceleration is at most about g x r_px / R (the
part of g across the line of sight) and its size changes smoothly. The decoys fail at least one:

  flowers, a pumpkin, a red box   in place, a few px of jitter: no travel (and offline they sit at one pixel all clip: `static_mask`)
  a hand carrying the ball        slow
  the arm's throwing motion       accelerations far above g (the track passes once the ball is free, a few frames after release)
  sensor noise, a sleeve          no chain of frames, or no quadratic through it

  python arc2d.py <session dir> [throw]   # replay a recorded RealSense session's colour frames through the trigger (PC; what fires, why not)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class ArcCfg:
    ball_radius: float = 0.08034       # m (the session's ball; the 160.7 mm orange foam test ball)
    n_min: int = 6                     # points on a track before it is judged
    window: int = 10                   # the judge fits the last this many points
    min_span_s: float = 0.06
    rms_px: float = 2.0                # quadratic residual allowed: max(rms_px, rms_frac x radius px)
    rms_frac: float = 0.15
    travel_r: float = 4.0              # travel over the window >= this many ball radii (2 diameters)
    min_speed: float = 2.0             # m/s, lateral (from the blob's size)
    acc_max: float = 2.5               # image acceleration / (g r_px / R): perspective adds some, the arm's swing adds far more
    size_jump: float = 1.6             # radius ratio allowed between consecutive points
    min_r_px: float = 4.0              # smaller blobs are noise (the test ball is 8 px in radius at 6 m on the colour lens, f 616), and
                                       # a 1-2 px blob's size makes the speed from size meaningless (measured: 9-15 m/s "throws")
    gate_px: float = 15.0              # association: a track takes the nearest blob within gate_px + gate_r x radius of its prediction
    gate_r: float = 3.0
    max_miss: int = 2                  # frames a track may go without a blob
    max_tracks: int = 40
    fly_speed: float = 1.0             # offline: an in-flight window moves at least this (m/s): the ball held in the hand before a throw is a
                                       # smooth, slow track (0.5-0.6 m/s on the first real throw) that the shape test alone lets in


def quad(t: np.ndarray, y: np.ndarray):
    """Least-squares y = c0 + c1 t + c2 t^2 / 2 (rows of coef: c0, c1, c2), and the design matrix."""
    A = np.column_stack((np.ones_like(t), t, 0.5 * t * t))
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return coef, A


def judge(t, uv, r, cfg: ArcCfg, motion: bool = True) -> tuple[bool, str, dict]:
    """(ok, reason, stats) for one track's points (t s, uv Nx2 px, r N px radii). motion=False checks the shape only (quadratic,
    acceleration, size): near a lob's apex the image barely moves for a few frames, so the offline pass checks travel and speed once,
    over the whole flight."""
    t, uv, r = np.asarray(t, float), np.asarray(uv, float).reshape(-1, 2), np.asarray(r, float)
    if len(t) < cfg.n_min:
        return False, "short", {}
    tt = t - t[0]
    if tt[-1] < cfg.min_span_s:
        return False, "short", {}
    rm = float(max(np.median(r), 0.5))
    coef, A = quad(tt, uv)
    res = np.linalg.norm(uv - A @ coef, axis=1)
    rms = float(np.sqrt(np.mean(res ** 2)))
    travel = float(np.linalg.norm(uv[-1] - uv[0]))
    v_mid = coef[1] + coef[2] * tt.mean()
    m_per_px = cfg.ball_radius / rm
    speed = float(np.linalg.norm(v_mid) * m_per_px)
    acc = float(np.linalg.norm(coef[2]) * m_per_px / G)                         # in g
    rr = np.maximum(r, 0.5)
    jump = float(np.max(np.maximum(rr[1:] / rr[:-1], rr[:-1] / rr[1:]))) if len(rr) > 1 else 1.0
    st = dict(n=len(t), rms_px=rms, travel_px=travel, r_px=rm, speed_mps=speed, acc_g=acc, size_jump=jump,
              u=float(uv[-1, 0]), v=float(uv[-1, 1]))
    if rms > max(cfg.rms_px, cfg.rms_frac * rm):
        return False, f"not an arc ({rms:.1f} px off a quadratic)", st
    if motion and travel < cfg.travel_r * rm:
        return False, f"still ({travel:.0f} px travel, radius {rm:.0f} px)", st
    if motion and speed < cfg.min_speed:
        return False, f"slow ({speed:.1f} m/s)", st
    if acc > cfg.acc_max:
        return False, f"jerky ({acc:.1f} g in the image)", st
    if jump > cfg.size_jump:
        return False, f"size jumps (x{jump:.1f})", st
    return True, "arc", st


@dataclass
class Track:
    t: list = field(default_factory=list)
    uv: list = field(default_factory=list)
    r: list = field(default_factory=list)
    idx: list = field(default_factory=list)          # caller's frame index of each point
    miss: int = 0
    id: int = 0

    def add(self, t, c, idx=None):
        self.t.append(float(t))
        self.uv.append((float(c[0]), float(c[1])))
        self.r.append(float(c[2]))
        self.idx.append(idx)
        self.miss = 0

    def predict(self, t: float, window: int) -> np.ndarray:
        n = len(self.t)
        if n == 1:
            return np.array(self.uv[-1])
        ts = np.array(self.t[-window:])
        P = np.array(self.uv[-window:])
        if n == 2:
            v = (P[1] - P[0]) / max(ts[1] - ts[0], 1e-6)
            return P[1] + v * (t - ts[1])
        coef, _ = quad(ts - ts[0], P)
        tau = t - ts[0]
        return coef[0] + coef[1] * tau + 0.5 * coef[2] * tau * tau

    def last(self, k: int):
        return np.array(self.t[-k:]), np.array(self.uv[-k:]), np.array(self.r[-k:])


class ArcTracker:
    """Blob tracks frame to frame (nearest to each track's own quadratic prediction, longest tracks first), each judged on its last
    `window` points. `update` returns the stats of a track that looks thrown, else None; `best_reject` is the most ball-like track that
    failed (for a "why didn't it fire" line)."""

    def __init__(self, cfg: ArcCfg | None = None):
        self.cfg = cfg or ArcCfg()
        self.tracks: list[Track] = []
        self.done: list[Track] = []                  # finished tracks (keep_done: offline use)
        self.keep_done = False
        self.best_reject: tuple | None = None
        self._next = 1

    def reset(self):
        self.tracks = []
        self.best_reject = None

    def update(self, t: float, cands, idx=None) -> dict | None:
        cfg = self.cfg
        cands = [tuple(c[:3]) for c in cands if c[2] >= cfg.min_r_px]
        used: set = set()
        for tr in sorted(self.tracks, key=lambda x: -len(x.t)):
            p = tr.predict(t, cfg.window)
            gate = cfg.gate_px + cfg.gate_r * tr.r[-1]
            best, bd = None, gate
            for j, c in enumerate(cands):
                if j in used:
                    continue
                d = math.hypot(c[0] - p[0], c[1] - p[1])
                if d < bd:
                    best, bd = j, d
            if best is None:
                tr.miss += 1
            else:
                used.add(best)
                tr.add(t, cands[best], idx)
        alive = []
        for tr in self.tracks:
            (alive if tr.miss <= cfg.max_miss else self.done).append(tr)
        self.tracks = alive
        for j, c in enumerate(cands):
            if j not in used and len(self.tracks) < cfg.max_tracks:
                tr = Track(id=self._next)
                self._next += 1
                tr.add(t, c, idx)
                self.tracks.append(tr)
        if not self.keep_done:
            self.done = []
        hit = None
        for tr in self.tracks:
            if tr.miss or len(tr.t) < cfg.n_min:
                continue
            ok, why, st = judge(*tr.last(cfg.window), cfg)
            if ok:
                if hit is None or st["n"] > hit["n"]:
                    hit = dict(st, track=tr.id)
            elif st and st["travel_px"] >= cfg.travel_r * st["r_px"]:                   # moved like a ball but failed: report the closest
                if self.best_reject is None or st["speed_mps"] > self.best_reject[1].get("speed_mps", 0):
                    self.best_reject = (why, st)
        return hit

    def all_tracks(self) -> list[Track]:
        return self.done + self.tracks


def depth_in_front(u: float, v: float, r: float, depth: np.ndarray, scale: float, ic: tuple, idp: tuple, R: np.ndarray, t: np.ndarray,
                   z_guess: float, near: float = 0.6, share: float = 0.5) -> bool:
    """True when most of the depth inside a colour blob's disc is much nearer (< near x z_guess) than the blob's own size range: the blob
    is a patch on a nearer, bigger thing (a hand, an arm: Oct 30, a walking person's hand made a 7-frame "arc" whose size said 10 m and
    whose depth said 1 m), not a free ball (behind a flying ball the scene is farther, not nearer). ic, idp: (fx, fy, ppx, ppy) of the
    colour and depth images; R, t: depth -> colour (p_c = R p_d + t). numpy only (the recorder's trigger uses it on the Pi)."""
    fx, fy, cx, cy = ic
    pc = np.array([(u - cx) / fx * z_guess, (v - cy) / fy * z_guess, z_guess])
    pd = R.T @ (pc - t)
    if pd[2] <= 0.1:
        return False
    dfx, dfy, dcx, dcy = idp
    ud, vd = pd[0] / pd[2] * dfx + dcx, pd[1] / pd[2] * dfy + dcy
    rd = max(1.5, 0.8 * r * dfx / fx)
    x0, x1 = int(max(0, ud - rd)), int(min(depth.shape[1], ud + rd + 1))
    y0, y1 = int(max(0, vd - rd)), int(min(depth.shape[0], vd + rd + 1))
    if x1 <= x0 or y1 <= y0:
        return False
    z = depth[y0:y1, x0:x1].astype(np.float32) * scale
    z = z[z > 0.15]
    return len(z) >= 5 and float(np.mean(z < near * z_guess)) >= share


# ------------------------------------------------------------------------------------------------- offline
def static_mask(cands_per_frame: list, px: float = 3.0, frac: float = 0.25) -> list:
    """Per frame, True for each blob that sits within `px` of a blob in at least `frac` of the clip's frames: the scene's ball-coloured
    things (a flower, a pumpkin). A thrown ball passes any pixel in a frame or two."""
    pts = [(i, c[0], c[1]) for i, cs in enumerate(cands_per_frame) for c in cs]
    if not pts:
        return [[] for _ in cands_per_frame]
    a = np.array(pts, float)
    n = len(cands_per_frame)
    out = [[False] * len(cs) for cs in cands_per_frame]
    k = 0
    for i, cs in enumerate(cands_per_frame):
        for j, c in enumerate(cs):
            near = (np.abs(a[:, 1] - c[0]) <= px) & (np.abs(a[:, 2] - c[1]) <= px)
            out[i][j] = len(np.unique(a[near, 0])) >= frac * n
            k += 1
    return out


def flight_track(times, cands_per_frame: list, cfg: ArcCfg | None = None) -> dict | None:
    """Offline: the thrown ball's track through a clip. Static blobs dropped, every remaining blob tracked forward (the trigger's own
    tracker), each track's points marked "in flight" when some window of n_min..window consecutive points containing them has an arc's
    shape (`judge(motion=False)`) and moves at least `fly_speed`, the longest run of them kept if it travels and moves like a throw (`judge` on the run's ends and its
    fastest window); the track with the longest such run wins. {frames: [frame index], t, uv, r, n_tracks, track} or None."""
    cfg = cfg or ArcCfg()
    still = static_mask(cands_per_frame)
    trk = ArcTracker(cfg)
    trk.keep_done = True
    for i, (t, cs) in enumerate(zip(times, cands_per_frame)):
        trk.update(float(t), [c for c, s in zip(cs, still[i]) if not s], idx=i)
    best = None
    w = cfg.window
    for tr in trk.all_tracks():
        n = len(tr.t)
        if n < cfg.n_min:
            continue
        T, UV, R = np.array(tr.t), np.array(tr.uv), np.array(tr.r)
        fly = np.zeros(n, bool)
        for s in range(0, n - cfg.n_min + 1):
            e = min(n, s + w)
            ok, _, st = judge(T[s:e], UV[s:e], R[s:e], cfg, motion=False)
            if ok and st["speed_mps"] >= cfg.fly_speed:
                fly[s:e] = True
        keep = np.nonzero(fly)[0]
        if len(keep) < cfg.n_min:
            continue
        runs, start = [], keep[0]                     # contiguous runs of in-flight points (a release and a bounce are two flights)
        for a_, b_ in zip(keep[:-1], keep[1:]):
            if b_ != a_ + 1:
                runs.append((start, a_))
                start = b_
        runs.append((start, keep[-1]))
        for s, e in runs:
            if e - s + 1 < cfg.n_min:
                continue
            rm = float(np.median(R[s:e + 1]))
            travel = float(np.max(np.linalg.norm(UV[s:e + 1] - UV[s], axis=1)))
            fast = any(judge(T[k:k + w], UV[k:k + w], R[k:k + w], cfg)[0] for k in range(s, e + 2 - cfg.n_min))
            if travel >= cfg.travel_r * rm and fast and (best is None or e - s > best[0]):
                best = (e - s, tr, s, e)
    if best is None:
        return None
    _, tr, s, e = best
    sel = list(range(s, e + 1))
    # the whole track too: the 2D test fails where the ball nears the camera (perspective bends its path, its size grows fast and the
    # blob fragments), so the caller extends the flight along it with a 3D fit (rs_analyze.colour_source)
    return dict(frames=[tr.idx[k] for k in sel], t=[tr.t[k] for k in sel], uv=[tr.uv[k] for k in sel], r=[tr.r[k] for k in sel],
                n_tracks=len(trk.all_tracks()), track=tr.id, core=(s, e),
                all=dict(frames=list(tr.idx), t=list(tr.t), uv=list(tr.uv), r=list(tr.r)),
                tracklets=[dict(frames=list(o.idx), uv=list(o.uv), r=list(o.r)) for o in trk.all_tracks() if o is not tr and len(o.t) >= 4])


# ------------------------------------------------------------------------------------------------- replay a recording
def replay(session_dir, throw: int | None = None, cfg: ArcCfg | None = None, n_cands: int = 6):
    """Feed a recorded session's colour frames through the trigger's tracker: when it would fire and on what (PC only)."""
    import sys
    from pathlib import Path
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    sys.path.insert(0, str(here.parents[1] / "ros" / "src" / "omni_catch"))
    import session as S
    import rs_analyze as A
    from omni_catch.frames import ArrayFrame
    d = Path(session_dir)
    sess = S.load_session(d)
    cfg = cfg or ArcCfg(ball_radius=float((sess.get("ball") or {}).get("radius", ArcCfg.ball_radius)))
    from record_throws import make_detector
    det = make_detector(A.detector_cfg(sess))
    for p in S.throw_dirs(d):
        if throw is not None and int(p.name[1:]) != throw:
            continue
        th = S.load_throw(p, sess)
        t = th.t_of("color")
        cands = [[(float(c[0]), float(c[1]), float(c[2])) for c in det.candidates(ArrayFrame(np.asarray(th.frames[i])), n_cands)]
                 for i in range(len(t))]
        trk = ArcTracker(cfg)
        fires = []
        for i in range(len(t)):
            h = trk.update(float(t[i]), cands[i])
            if h:
                fires.append((i, float(t[i] - t[0]), h))
        print(f"{p.name}: {len(t)} frames, blobs in {sum(1 for c in cands if c)}; the arc trigger fires in {len(fires)} frames"
              + (f", first at {fires[0][1]:.2f} s: {fmt(fires[0][2])}" if fires else ""))
        if not fires and trk.best_reject:
            print(f"   closest reject: {trk.best_reject[0]}  {fmt(trk.best_reject[1])}")
        f = flight_track(t, cands, cfg)
        print("   offline flight: " + ("none" if f is None else
              f"{len(f['frames'])} frames {f['t'][0] - t[0]:.2f}..{f['t'][-1] - t[0]:.2f} s, ({f['uv'][0][0]:.0f},{f['uv'][0][1]:.0f}) -> "
              f"({f['uv'][-1][0]:.0f},{f['uv'][-1][1]:.0f}) px, of {f['n_tracks']} tracks"))


def fmt(st: dict) -> str:
    return (f"n {st.get('n')}, {st.get('rms_px', 0):.1f} px rms, travel {st.get('travel_px', 0):.0f} px, r {st.get('r_px', 0):.0f} px, "
            f"{st.get('speed_mps', 0):.1f} m/s, {st.get('acc_g', 0):.1f} g, at ({st.get('u', 0):.0f},{st.get('v', 0):.0f})")


if __name__ == "__main__":
    import sys
    replay(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else None)
