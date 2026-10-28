"""Tag SLAM's front end (design.md 10): odometry, lidar fixes and tag detections -> keyframes and factors in a TagGraph.

  on_odom(t, pose)          the EKF's odom -> base_link (smooth, drifting): the motion between keyframes
  on_lidar(t, pose, cov)    the lidar localizer's map -> base_link (slam_toolbox, or v9's matcher): an absolute fix at
                            the next keyframe. With these the map frame is the lidar's map; without, the first keyframe
                            anchors it (the gauge: any rotation / translation of everything fits equally well otherwise)
  on_tags(t, dets)          detections at one exposure: a keyframe there, each good one a factor; a tag seen for the
                            first time (well: close, square-on, unambiguous) starts a landmark at its measured pose
  solve()                   the graph, warm-started; then views whose corners still miss by > reject_px are dropped
                            (a misdetection, a tag that moved), and a tag left inconsistent is reset
  pose(t)                   map -> base_link now: the last keyframe's estimate + the odometry since
  map_from_odom()           the correction map -> odom (what the global localizer publishes)

Tags are persistent by id: save_map() / load_map() keep their poses and covariances with the occupancy map
(<map>.tags.json); loaded ones carry their covariance as a prior (a known map; they still refine).
Keyframes: every 0.3 m or 20 deg, and at every exposure with a usable tag; old keyframes without tags are dropped
past max_keyframes, their odometry joined (the chain's motion and covariance add up).
"""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import numpy as np

from omni_slam.graph import TagGraph, se2_between, se2_compose, wrap
from omni_slam.tags import Pinhole, TagGate, log_rot, rodrigues, tag_pose


def odom_cov(d) -> np.ndarray:
    """The odometry's covariance over a motion d = (dx, dy, dyaw) (est: 2% of distance, 1% of turn + a floor, as the
    EKF's wheel model; the IMU holds the heading)."""
    dist = math.hypot(d[0], d[1])
    sxy = 0.003 + 0.02 * dist
    sy = math.radians(0.2) + 0.01 * abs(d[2]) + math.radians(0.3) * dist
    return np.diag([sxy ** 2, sxy ** 2, sy ** 2])


class TagSlam:
    def __init__(self, cam: Pinhole, T_base_cam, gate: TagGate = TagGate(), kf_dist: float = 0.3,
                 kf_yaw: float = math.radians(20), solve_every: int = 5, reject_px: float = 4.0,
                 max_keyframes: int = 600, init_max_range: float = 3.0, sigma_px: float = 0.7, anchor: str = "first",
                 refine_cam_deg: float | None = None):
        # the camera's mount rotation refined online only where the map is fixed by something else (the lidar)
        self.g = TagGraph(cam, T_base_cam, sigma_px=sigma_px,
                          cam_sigma_deg=refine_cam_deg if anchor == "lidar" else None)
        self.cam, self.gate = cam, gate
        self.kf_dist, self.kf_yaw = kf_dist, kf_yaw
        self.solve_every, self.reject_px, self.max_kf = solve_every, reject_px, max_keyframes
        self.init_max_range = init_max_range
        self.anchor = anchor                         # "first": map = odom at the start; "lidar": the lidar's map
        self.odom: deque = deque(maxlen=4000)        # (t, x, y, yaw)
        self.att: deque = deque(maxlen=4000)         # (t, roll, pitch)
        self.kf_odom: list = []                      # each keyframe's odom pose
        self.lidar = None                            # the last fix (t, pose, cov), used at the next keyframe
        self.use_lidar = False
        self.n_since_solve = 0
        self.sizes: dict = {}                        # tag id -> size (m), from the tag map or a default
        self.default_size = 0.16
        self.n_rejected = self.n_dropped = self.n_resets = 0
        self.last_solve = None
        self.seen: dict = {}                         # tag id -> number of good views
        self.votes: deque = deque(maxlen=32)         # relocalization: (t, tag id, pose, cov, odom at t)

    # -- inputs ----------------------------------------------------------------------------------------------------
    def on_odom(self, t: float, pose):
        self.odom.append((t, float(pose[0]), float(pose[1]), float(pose[2])))
        if not self.kf_odom:
            if self.anchor == "first" or (self.lidar is not None and abs(self.lidar[0] - t) < 0.3):
                self._keyframe(t)
            return
        d = se2_between(self.kf_odom[-1], self._odom_at(t))
        if math.hypot(d[0], d[1]) > self.kf_dist or abs(d[2]) > self.kf_yaw:
            self._keyframe(t)

    def on_attitude(self, t: float, roll: float, pitch: float):
        """The IMU's roll and pitch (the BNO085's game rotation vector): the camera's tilt at each keyframe."""
        self.att.append((t, roll, pitch))

    def _att_at(self, t: float):
        if not self.att:
            return (0.0, 0.0)
        best = min(self.att, key=lambda a: abs(a[0] - t)) if len(self.att) < 8 else \
            min(list(self.att)[-64:], key=lambda a: abs(a[0] - t))
        return best[1], best[2]

    def on_lidar(self, t: float, pose, cov):
        self.lidar = (t, np.asarray(pose, float), np.asarray(cov, float))
        self.use_lidar = True

    def _odom_at(self, t: float):
        o = self.odom
        if not o:
            return None
        if t >= o[-1][0]:
            return np.array(o[-1][1:])
        if t <= o[0][0]:
            return np.array(o[0][1:])
        ts = np.fromiter((p[0] for p in o), float, len(o))
        k = int(np.searchsorted(ts, t))
        a, b = np.array(o[k - 1][1:]), np.array(o[k][1:])
        f = (t - o[k - 1][0]) / max(o[k][0] - o[k - 1][0], 1e-9)
        out = a + f * (b - a)
        out[2] = a[2] + f * float(wrap(b[2] - a[2]))
        return out

    def _keyframe(self, t: float) -> int:
        g = self.g
        op = self._odom_at(t)
        if not self.kf_odom:
            fix = self.lidar is not None and abs(self.lidar[0] - t) < 0.3
            guess = self.lidar[1] if fix else op.copy()
            i = g.add_pose(t, guess, self._att_at(t))
            if not fix:
                g.add_prior(i, guess, np.diag([1e-6, 1e-6, 1e-8]))          # the anchor: map = odom at the start
        else:
            d = se2_between(self.kf_odom[-1], op)
            i = g.add_pose(t, se2_compose(g.X[-1], d), self._att_at(t))
            g.add_between(i - 1, i, d, odom_cov(d))
        self.kf_odom.append(op)
        if self.lidar is not None and abs(self.lidar[0] - t) < 0.3:
            lt, lp, lc = self.lidar
            # the fix moved to the keyframe's time with the odometry between them
            lp = se2_compose(lp, se2_between(self._odom_at(lt), op))
            g.add_prior(i, lp, lc + np.diag([1e-5, 1e-5, 1e-6]))
            self.lidar = None
        self.n_since_solve += 1
        if self.n_since_solve >= self.solve_every:
            self.solve()
        return i

    def on_tags(self, t: float, dets) -> list:
        """Returns, per detection, what became of it: "used", "new tag", or why it was refused."""
        if not self.odom or not self.kf_odom:
            return ["not started"] * len(dets)
        poses = [tag_pose(self.cam, d, self.gate) for d in dets]
        if not any(p.ok for p in poses):
            self.n_rejected += len(dets)
            return [",".join(p.reasons) for p in poses]
        g = self.g
        # the robot hasn't moved since the last keyframe and sees only tags it saw there: nothing new to add
        d = se2_between(self.kf_odom[-1], self._odom_at(t))
        ids = {dd.tag_id for dd, p in zip(dets, poses) if p.ok}
        last_ids = {g.lm_index_inv(j) for i_, j in zip(g.obs_i, g.obs_j) if i_ == len(g.X) - 1}
        if math.hypot(d[0], d[1]) < 0.05 and abs(d[2]) < math.radians(3) and ids <= last_ids:
            return ["redundant"] * len(dets)
        i = self._keyframe(t) if not g.t_pose or abs(g.t_pose[-1] - t) > 1e-6 else len(g.X) - 1
        out = []
        new = False
        for d, p in zip(dets, poses):
            if not p.ok:
                self.n_rejected += 1
                out.append(",".join(p.reasons))
                continue
            if not g.has_tag(d.tag_id):
                if p.ambiguous or p.range > self.init_max_range:
                    out.append("ambiguous" if p.ambiguous else "too far to start")
                    continue
                R, tt = g.tag_in_map(g.X[i], p.R, p.t, g.att[i])
                g.add_tag(d.tag_id, d.size, R, tt)
                self.sizes[d.tag_id] = d.size
                new = True
                out.append("new tag")
            else:
                out.append("used")
            g.add_obs(i, d.tag_id, d.corners, t)
            self.seen[d.tag_id] = self.seen.get(d.tag_id, 0) + 1
        if new:
            self.solve()
        return out

    # -- solving ---------------------------------------------------------------------------------------------------
    def solve(self, iters: int = 8):
        g = self.g
        if not len(g.X):
            return None
        self.n_since_solve = 0
        st = g.solve(iters)
        if g.obs_i:
            # drop views whose corners still miss (after the robust fit): a misdetection or a moved tag
            bad = [k for k in range(len(g.obs_i)) if g.obs_residual_px(k) > self.reject_px]
            if bad:
                self.n_dropped += len(bad)
                for name in ("obs_i", "obs_j", "obs_uv", "obs_t"):
                    setattr(g, name, [v for k, v in enumerate(getattr(g, name)) if k not in set(bad)])
                st = g.solve(iters)
        self.last_solve = st
        if len(g.X) > self.max_kf:
            self._prune()
        return st

    def _prune(self):
        """Drop the oldest keyframes that carry no tag view and no prior, joining their odometry."""
        g = self.g
        keep_obs = set(g.obs_i)
        keep_prior = {i for i, _, _ in g.priors}
        n_drop = len(g.X) - int(0.8 * self.max_kf)
        drop = [i for i in range(1, len(g.X) - 1) if i not in keep_obs and i not in keep_prior][:n_drop]
        if not drop:
            return
        dropset = set(drop)
        remap, kept = {}, []
        for i in range(len(g.X)):
            if i not in dropset:
                remap[i] = len(kept)
                kept.append(i)
        betweens = []
        k = 0
        while k < len(kept) - 1:
            a, b = kept[k], kept[k + 1]
            z = se2_between(self.kf_odom[a], self.kf_odom[b])
            betweens.append((remap[a], remap[b], z, np.linalg.cholesky(np.linalg.inv(odom_cov(z))).T))
            k += 1
        g.X = g.X[kept]
        g.att = g.att[kept]
        g.t_pose = [g.t_pose[i] for i in kept]
        self.kf_odom = [self.kf_odom[i] for i in kept]
        g.priors = [(remap[i], z, W) for i, z, W in g.priors]
        g.betweens = betweens
        g.obs_i = [remap[i] for i in g.obs_i]
        g._lu = None

    # -- outputs ---------------------------------------------------------------------------------------------------
    def pose(self, t: float | None = None):
        g = self.g
        if not len(g.X):
            return None
        o = self._odom_at(t if t is not None else self.odom[-1][0])
        return se2_compose(g.X[-1], se2_between(self.kf_odom[-1], o))

    def pose_cov(self) -> np.ndarray | None:
        if not len(self.g.X) or self.last_solve is None:
            return None
        return self.g.pose_cov(len(self.g.X) - 1)

    def map_from_odom(self):
        """map -> odom as (x, y, yaw): the last keyframe's map pose after its odom pose."""
        g = self.g
        if not len(g.X):
            return None
        Xm, Xo = g.X[-1], self.kf_odom[-1]
        yaw = float(wrap(Xm[2] - Xo[2]))
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([Xm[0] - (c * Xo[0] - s * Xo[1]), Xm[1] - (s * Xo[0] + c * Xo[1]), yaw])

    def tags(self) -> dict:
        out = {}
        for tid, j in self.g.lm_index.items():
            R, t = self.g.tag_pose(tid)
            out[tid] = dict(R=R, t=t, size=self.g.lm_size[j], views=self.seen.get(tid, 0))
        return out

    # -- relocalization from one tag ---------------------------------------------------------------------------------
    def pose_from_tag(self, det, max_tag_sigma: float = 0.03, max_range: float = 3.0):
        """Where the robot is, from one good detection of a tag the map knows well: (x, y, yaw, cov) or None. Only
        close, unambiguous views: beyond ~3 m a 16 cm tag's mirror pose fits nearly as well (one of them can be off
        by metres), so the health monitor also wants two that agree before it moves the robot's estimate."""
        g = self.g
        if not g.has_tag(det.tag_id):
            return None
        p = tag_pose(self.cam, det, self.gate)
        if not p.ok or p.ambiguous or p.range > max_range:
            return None
        try:
            C = g.tag_cov(det.tag_id)
            if math.sqrt(max(C[3, 3] + C[4, 4], 0.0)) > max_tag_sigma:
                return None
        except Exception:
            C = np.diag([1e-4] * 6)
        R_mt, t_mt = g.tag_pose(det.tag_id)
        T_mt = np.eye(4)
        T_mt[:3, :3], T_mt[:3, 3] = R_mt, t_mt
        T_ct = np.eye(4)
        T_ct[:3, :3], T_ct[:3, 3] = p.R, p.t
        T_mb = T_mt @ np.linalg.inv(T_ct) @ np.linalg.inv(g.base_cam())
        yaw = math.atan2(T_mb[1, 0], T_mb[0, 0])
        s_xy = 0.01 + 0.01 * p.range ** 2 + math.sqrt(max(C[3, 3], 0.0))     # (est: grows with range squared)
        s_yaw = math.radians(0.5) + math.radians(0.5) * p.range
        return np.array([T_mb[0, 3], T_mb[1, 3], yaw]), np.diag([s_xy ** 2, s_xy ** 2, s_yaw ** 2])

    def vote(self, t: float, det, window_s: float = 4.0, agree_m: float = 0.10, agree_deg: float = 3.0):
        """Relocalization by agreement: each close, unambiguous view of a known tag gives a pose; carried to time t
        by the odometry, two from different tags (or the same tag seen from 0.3 m apart) that agree within agree_m /
        agree_deg give the robot's pose (their mean, with the tighter covariance). A misdecode (a wrong id) can't
        agree with anything. Returns (pose, cov) or None."""
        r = self.pose_from_tag(det)
        if r is not None:
            self.votes.append((t, det.tag_id, r[0], r[1], self._odom_at(t)))
        while self.votes and self.votes[0][0] < t - window_s:
            self.votes.popleft()
        o_now = self._odom_at(t)
        cand = [(tid, o, se2_compose(p, se2_between(o, o_now)), C) for tv, tid, p, C, o in self.votes]
        for a in range(len(cand)):
            for b in range(a + 1, len(cand)):
                (ia, oa, pa, Ca), (ib, ob, pb, Cb) = cand[a], cand[b]
                # independent views only: two tags, or one tag from places 0.3 m apart (two frames of the same view
                # share its errors, a wrong mirror pose included)
                if ia == ib and math.hypot(oa[0] - ob[0], oa[1] - ob[1]) < 0.3:
                    continue
                if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < agree_m and \
                        abs(float(wrap(pa[2] - pb[2]))) < math.radians(agree_deg):
                    mean = (pa + pb) / 2
                    mean[2] = float(wrap(pa[2] + float(wrap(pb[2] - pa[2])) / 2))
                    self.votes.clear()
                    return mean, (Ca if np.trace(Ca) < np.trace(Cb) else Cb)
        return None

    # -- the tag map ---------------------------------------------------------------------------------------------
    def save_map(self, path: str | Path):
        out = []
        for tid, j in sorted(self.g.lm_index.items()):
            R, t = self.g.tag_pose(tid)
            try:
                C = self.g.tag_cov(tid)
            except Exception:
                C = np.diag([1e-4] * 6)
            out.append(dict(id=int(tid), size=self.g.lm_size[j], rvec=[float(c) for c in log_rot(R)],
                            t=[float(c) for c in t], cov=[[float(c) for c in r] for r in C],
                            views=self.seen.get(tid, 0)))
        Path(path).write_text(json.dumps(dict(format="omni-tags/1", tags=out), indent=1))

    def load_map(self, path: str | Path, inflate: float = 4.0):
        raw = json.loads(Path(path).read_text())
        for tg in raw["tags"]:
            C = np.array(tg["cov"]) * inflate + np.diag([1e-6] * 3 + [1e-6] * 3)
            self.g.add_tag(int(tg["id"]), tg["size"], rodrigues(tg["rvec"]), np.array(tg["t"]), prior_cov=C)
            self.sizes[int(tg["id"])] = tg["size"]
