"""The tidy with the D435 elsewhere (design.md 4.11, Nov 5): sil_pick's whole pick-and-bin, the robot's code given the
camera's pose (robot_export.arm_json of each mount: its depth camera, keep-outs, the close look's step back), and what
the cameras see while the arm moves. A study beside d435_mount.py (which scores the views with the arm stowed); the
baseline stays robot_export.ARM_D435 (the nose, 7.5 deg).

For each mount and seed (sil_pick's arena, 6 toys, realistic D435 noise):

  tidy     toys in the bin (the truth), the task's picked / failed / skipped, its time
  imaging  every 0.25 s of sim time, by the task's state: the share of the D435's image the arm fills; the target (the
           thing the task is after) in the D435's field (its centre projects into the image), seen (>= 10 of its pixels
           by a segmentation render) and so hidden by the arm or the robot when in the field but not seen; the share
           of samples whose state asked for a depth frame; the target in the wrist camera's field
  pictures the D435's colour view at the moment the arm fills most of it, per mount (results/d435_mount_pick/)

    python studies/d435_mount_pick.py --only nose,riser30 --seeds 0 1      a few
    python studies/d435_mount_pick.py --out d435_mount_pick.json            every mount below, seeds 0-3
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import arm_objects  # noqa: E402
from robot_export import ARM_D435, _rounded, arm_d435_params, arm_json  # noqa: E402
from sil_pick import SilPick  # noqa: E402

OUT = Path(__file__).resolve().parent / "results"
# (label, body centre m, tilt deg down, roll deg (90 portrait), the close look's step back m)
MOUNTS = [
    ("nose", (0.19, 0.0, 0.1115), 7.5, 0.0, 0.10),        # the baseline (robot_arm_d435.json)
    ("riser30", (0.20, 0.0, 0.215), 30.0, 0.0, 0.10),
    ("riser35", (0.20, 0.0, 0.215), 35.0, 0.0, 0.05),
    ("riser_p25", (0.20, 0.0, 0.20), 25.0, 90.0, 0.05),
    ("riser_p30", (0.20, 0.0, 0.20), 30.0, 90.0, 0.0),
]
DT = 0.25                    # s of sim time between imaging samples
SEEN_PX = 10


def mount_kw(pos, tilt, roll) -> dict:
    return dict(d435_pos=tuple(pos), d435_tilt_deg=-float(tilt), d435_roll_deg=float(roll))


def write_config(label, pos, tilt, roll, look_back, out_dir: Path) -> Path:
    """The robot's config for the camera at this mount (robot_export.arm_json), the close look's step back set."""
    kw = mount_kw(pos, tilt, roll)
    cfg = arm_json(arm_d435_params(**kw), dict(ARM_D435, **kw))
    cfg["reach"]["look_back"] = look_back
    path = out_dir / f"robot_arm_d435_{label}.json"
    path.write_text(json.dumps(_rounded(cfg), indent=1) + "\n")
    return path


class WatchPick(SilPick):
    """sil_pick with an observer: what the D435 and the wrist camera see in each of the task's states."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        m, c = self.m, self.cams
        W, H = c.DEPTH_WH
        self.seg_d = mujoco.Renderer(c.mr, H, W)
        self.seg_d.enable_segmentation_rendering()
        self.pic = mujoco.Renderer(c.mr, H, W)             # its own (cams.color() would draw on the depth noise's rng)
        names = [m.body(m.geom_bodyid[g]).name for g in range(m.ngeom)]
        root = np.array([m.body_rootid[m.geom_bodyid[g]] for g in range(m.ngeom)])
        self.is_arm = np.r_[[n.startswith("arm") for n in names], False]
        self.is_robot = np.r_[root == m.body_rootid[m.body("base").id], False]
        self.obj_body = [m.body(f"obj{k}").id for k in range(len(arm_objects.positions(m, self.d)))]
        self.geom_body = np.r_[m.geom_bodyid, -1]
        self.t_watch = 0.0
        self.samples: list = []
        self.worst = (0.0, None, "")                       # (arm share, colour image, state)
        self.seg_errors = 0                                # renders whose segment ids came back out of range

    def _seg(self, ren, cam) -> np.ndarray | None:
        ren.update_scene(self.cams.dr, camera=cam)
        try:
            s = ren.render()
        except IndexError:                                 # MuJoCo's segment id readout, now and then (Windows)
            self.seg_errors += 1
            return None
        return np.where((s[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (s[..., 0] >= 0), s[..., 0], -1)

    def _target(self):
        """The floor object the task is after (the one nearest its detection), or None."""
        tg = getattr(self.task, "target", None)
        if tg is None or self.task.state in ("IDLE", "LOOK", "TURN", "DONE"):
            return None
        od = self.odom_xy(arm_objects.positions(self.m, self.d))
        k = int(np.argmin(np.hypot(od[:, 0] - tg.det.x, od[:, 1] - tg.det.y)))
        return k

    def _in_field(self, k, wrist: bool = False) -> bool:
        """The thing's centre projects into the camera's image."""
        d, c = self.cams.dr, (self.cams.cw if wrist else self.cams.cl)
        P = self.d.xpos[self.obj_body[k]]
        R = d.cam_xmat[c].reshape(3, 3)
        v = R.T @ (P - d.cam_xpos[c])                       # camera frame: x right, y up, looks along -z
        if v[2] >= -1e-3:
            return False
        fx, fy, cx, cy = self.cams.Kw if wrist else self.cams.Kd
        u, w = cx + fx * v[0] / -v[2], cy - fy * v[1] / -v[2]
        W, H = self.cams.WRIST_WH if wrist else self.cams.DEPTH_WH
        return 0 <= u < W and 0 <= w < H

    def step(self):
        super().step()
        t = self.d.time
        if t < self.t_watch - 1e-9 or self.out is None:
            return
        self.t_watch = t + DT
        self.cams._sync()
        g = self._seg(self.seg_d, self.cams.cl)
        if g is None:
            return
        arm = float(self.is_arm[g].mean())
        k = self._target()
        s = dict(state=self.task.state, arm=arm, robot=float(self.is_robot[g].mean()), want=bool(self.out.want_depth))
        if k is not None:
            b = self.obj_body[k]
            s["field"] = self._in_field(k)
            s["seen"] = int((self.geom_body[g] == b).sum()) >= SEEN_PX
            s["wrist"] = self._in_field(k, wrist=True)
        self.samples.append(s)
        if arm > self.worst[0]:
            self.pic.update_scene(self.cams.dr, camera=self.cams.cl)
            self.worst = (arm, self.pic.render().copy(), self.task.state)

    def imaging(self) -> dict:
        by = defaultdict(list)
        for s in self.samples:
            by[s["state"]].append(s)
        out = {}
        for st, ss in by.items():
            tg = [s for s in ss if "field" in s]
            infield = [s for s in tg if s["field"]]
            out[st] = dict(n=len(ss), arm_mean=round(float(np.mean([s["arm"] for s in ss])), 3),
                           arm_max=round(float(max(s["arm"] for s in ss)), 3),
                           want_depth=round(float(np.mean([s["want"] for s in ss])), 2),
                           target_n=len(tg),
                           in_field=round(len(infield) / len(tg), 2) if tg else None,
                           hidden=round(sum(not s["seen"] for s in infield) / len(infield), 2) if infield else None,
                           wrist=round(float(np.mean([s["wrist"] for s in tg])), 2) if tg else None)
        return out

    def close(self):
        self.seg_d.close()
        self.pic.close()
        self.cams.close()


def run_one(label, pos, tilt, roll, look_back, seed, n, t_max, out_dir: Path) -> dict:
    cfg = write_config(label, pos, tilt, roll, look_back, out_dir)
    w0 = time.perf_counter()
    g = WatchPick(n, "toys", seed, depth_camera="d435", arm_config=str(cfg), **mount_kw(pos, tilt, roll))
    s = g.run(t_max)
    r = dict(mount=label, pos=list(pos), tilt=tilt, roll=roll, look_back=look_back, seed=seed,
             **{k: v for k, v in s.items() if k != "events"}, imaging=g.imaging(),
             worst_arm=dict(share=round(g.worst[0], 3), state=g.worst[2]), seg_errors=g.seg_errors, wall_s=round(time.perf_counter() - w0, 1))
    if g.worst[1] is not None and seed == 0:
        try:
            from PIL import Image
            Image.fromarray(g.worst[1]).save(out_dir / f"{label}_arm_in_view.png")
        except ImportError:
            pass
    g.close()
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--only", default="", help="labels, comma separated (e.g. nose,riser_p30)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--t-max", type=float, default=500.0)
    ap.add_argument("--out", default="d435_mount_pick.json")
    a = ap.parse_args()
    only = {x for x in a.only.split(",") if x}
    out_dir = OUT / "d435_mount_pick"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for label, pos, tilt, roll, lb in MOUNTS:
        if only and label not in only:
            continue
        for seed in a.seeds:
            r = run_one(label, pos, tilt, roll, lb, seed, a.n, a.t_max, out_dir)
            rows.append(r)
            print(json.dumps({k: v for k, v in r.items() if k != "imaging"}), flush=True)
            (out_dir / a.out).write_text(json.dumps(rows, indent=1))
    print("wrote", out_dir / a.out)


if __name__ == "__main__":
    main()
