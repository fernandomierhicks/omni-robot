"""Where the SO-101 goes for picking things off the floor (design.md 11, Oct 24; Fernando: "more forward and lowered,
it doesn't have to be on the plate; the stereo cameras can move, lowered to see the floor").

The arm on a nose bracket at (x, z) (params.arm_mount_z; z = the adapter's top, robot frame) against the plate mount
of the Oct 23 study. For each mount, over the arm's joint space in its vertical plane (pan 0 = straight ahead, the
wrist's roll free):
  floor      top-down grasps: the tool point (the fixed finger's inner face, 6 mm above its tip) 8-30 mm above the
             floor, the fingers within 15 deg of straight down: the nearest and farthest such grasp from the robot's
             centre (the bumper is at 0.217 m), and the same for any approach
  clear      the links clear the floor (2 mm), the bumper ring (r 0.19-0.235 m up to 85 mm), the camera pods and the
             plate behind the bracket
  bin        a drop over the bin (params.arm_bin on the plate behind the arm): the tool 4-10 cm over its centre, the
             fingers within 45 deg of down, every link above the plate and the bin's rim
  stow       a folded pose within 15 mm of the bumper's outer edge: its top (lower is better for the CG and the view)
Then the reach at the chosen mount with the pan turned (how far to the sides it picks without moving the base).

  python arm_mount.py           -> results/arm_mount.json, printed table
"""

from __future__ import annotations

import itertools
import math
import sys

import mujoco
import numpy as np

from common import save  # noqa: E402
import arm_so101 as A  # noqa: E402
from robot_export import v11_params  # noqa: E402
from robot_model import build_model, camera_positions, geometry  # noqa: E402

PLATE_GAP = 0.096                      # the CAD's plate (cad/v7/dims.py)
BUMPER_R = 0.217
STOW_R = 0.232                         # folded: inside the bumper's outer edge (+15 mm)
DOWN_TOL = math.radians(15)
BIN = (-0.02, 0.0, 0.15, 0.14, 0.07)   # (x, y, length, width, height): on the plate behind the arm (est)
POD_X, POD_Z, POD_TILT = 0.17, 0.125, -35.0


def params(x: float, z: float | None, pods: bool = True):
    kw = dict(camera_mount="nose_pods", camera_pod_xz=(POD_X, POD_Z), camera_tilt_deg=POD_TILT) if pods else {}
    return v11_params(payload="arm", plate_gap=PLATE_GAP, arm_mount=(x, 0.0, 0.0), arm_mount_z=z,
                      arm_bin=BIN if pods else None, lidar_mount="rear" if pods else "front", home_xy=(0.0, 0.0), **kw)


class Fk:
    def __init__(self, p):
        self.p = p
        m = self.m = build_model(p)
        d = self.d = mujoco.MjData(m)
        adr = m.joint("base").qposadr[0]
        d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
        self.qadr = np.array([m.joint(A.name(j)).qposadr[0] for j in A.JOINTS])
        self.tool = m.site("arm_tool").id
        self.caps = [g for g in range(m.ngeom) if m.geom(g).name.startswith("arm_") and "_link" in m.geom(g).name
                     and m.geom(g).name not in ("arm_base_link0",)]
        self.r = np.array([m.geom_size[g][0] for g in self.caps])
        self.h = np.array([m.geom_size[g][1] for g in self.caps])
        g = geometry(p)
        self.plate_top = float(g.plate_top)
        self.mx = p.arm_mount[0]
        self.mz = A.mount_z(p, self.plate_top)
        _, cams = camera_positions(p, g)
        self.pods = [np.array(c) for _, c in cams] if p.camera_mount == "nose_pods" else []
        self.ref = np.array([0.0, 0.0, -1.0])

    def set(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)

    def points(self, n=5):
        s = np.linspace(-1, 1, n)
        P, R = [], []
        for g, r, h in zip(self.caps, self.r, self.h):
            c = self.d.geom_xpos[g]
            ax = self.d.geom_xmat[g].reshape(3, 3)[:, 2]
            P.append(c + np.outer(s * h, ax))
            R.append(np.full(n, r))
        return np.vstack(P), np.concatenate(R)

    def tool_pos(self):
        return self.d.site_xpos[self.tool].copy()

    def approach(self):
        return self.d.site_xmat[self.tool].reshape(3, 3)[:, 0].copy()

    def clear(self, allow_bin=False) -> bool:
        P, r = self.points()
        x, y, z = P.T
        rho = np.hypot(x, y)
        if np.any(z - r < 0.002):
            return False
        if np.any((rho > 0.19 - r) & (rho < 0.235 + r) & (z - r < 0.085)):
            return False
        for c in self.pods:                                       # each pod a 4 x 4 x 4 cm block
            if np.any((np.abs(x - c[0]) < 0.02 + r) & (np.abs(y - c[1]) < 0.02 + r) & (z - r < c[2] + 0.02)):
                return False
        # behind the bracket: nothing below the plate's top (+5 mm), and outside the bin's walls unless dropping
        behind = x < self.mx - 0.07
        if np.any(behind & (rho < 0.2) & (z - r < self.plate_top + 0.005)):
            return False
        # the robot's inside (deck, the bracket under the adapter): nothing below the adapter within the bumper
        if np.any(~behind & (rho < 0.19) & (z - r < self.mz - 0.01)):
            return False
        if self.p.arm_bin is None:
            return True
        bx, by, L, W, H = self.p.arm_bin
        top = self.plate_top + H
        inside = (np.abs(x - bx) < L / 2 + r) & (np.abs(y - by) < W / 2 + r)
        if np.any(inside & (z - r < top + (0.0 if allow_bin else 0.005))):
            return False
        return True


def grid(n=34):
    lo, hi = A.joint_ranges()[:, 0], A.joint_ranges()[:, 1]
    lift = np.linspace(lo[1], hi[1], n)
    elbow = np.linspace(lo[2], hi[2], n)
    wrist = np.linspace(lo[3], hi[3], n)
    return lift, elbow, wrist


def sweep(f: Fk, n=34):
    """Floor grasps, bin drops and stow candidates over the arm's vertical plane (pan 0, roll 0)."""
    lift, elbow, wrist = grid(n)
    floor_td, floor_any, drops, stows = [], [], [], []
    bx, by, L, W, H = f.p.arm_bin or (0.0, 0.0, 0.0, 0.0, 10.0)        # no bin: no drop
    bin_top = np.array([bx, by, f.plate_top + H])
    for a, b, c in itertools.product(lift, elbow, wrist):
        f.set((0.0, a, b, c, 0.0, 0.0))
        t = f.tool_pos()
        ap = f.approach()
        if 0.008 <= t[2] <= 0.03:
            if math.hypot(t[0], t[1]) < BUMPER_R + 0.012 or not f.clear():      # the object is outside the bumper
                continue
            floor_any.append(t[0])
            if ap @ f.ref > math.cos(DOWN_TOL):
                floor_td.append((t[0], (a, b, c)))
        elif t[2] > bin_top[2] + 0.04 and abs(t[0] - bx) < 0.04 and t[2] < bin_top[2] + 0.10:
            if ap @ f.ref > math.cos(math.radians(45)) and f.clear(allow_bin=True):
                drops.append((float(t[0]), float(t[2]), float(math.degrees(math.acos(ap @ f.ref))), (a, b, c)))
        P, r = f.points()
        if np.all(np.hypot(P[:, 0], P[:, 1]) + r < STOW_R) and f.clear():
            stows.append((float((P[:, 2] + r).max()), (a, b, c)))
    td = sorted(x for x, _ in floor_td)
    return dict(
        floor_topdown=[float(td[0]), float(td[-1])] if td else None,
        floor_any=[float(min(floor_any)), float(max(floor_any))] if floor_any else None,
        n_topdown=len(td),
        bin_drop=min(drops, key=lambda d: d[2]) if drops else None,
        stow=min(stows, key=lambda s: s[0]) if stows else None,
        topdown_poses=floor_td,
    )


def side_reach(f: Fk, poses, pans=np.radians(np.arange(0, 111, 10))):
    """For each pan angle: the farthest and nearest top-down floor grasp still clear (pan turns the plane)."""
    out = []
    for pan in pans:
        best = []
        for x, (a, b, c) in poses:
            f.set((pan, a, b, c, 0.0, 0.0))
            if f.clear():
                t = f.tool_pos()
                best.append((float(np.hypot(t[0], t[1])), float(t[0]), float(t[1])))
        if best:
            near, far = min(best), max(best)
            out.append(dict(pan_deg=float(math.degrees(pan)), near=near, far=far))
    return out


def main():
    rows = []
    mounts = [("plate centre (Oct 23)", 0.0, None, False), ("plate, 60 mm ahead (Oct 23)", 0.06, None, False)]
    mounts += [(f"nose x {x * 1000:.0f} z {z * 1000:.0f}", x, z, True)
               for x in (0.12, 0.15, 0.17) for z in (0.085, 0.105, 0.125)]
    for label, x, z, pods in mounts:
        p = params(x, z, pods)
        f = Fk(p)
        s = sweep(f)
        s.update(label=label, mount=[x, A.mount_z(p, f.plate_top)])
        rows.append(s)
        td, an = s["floor_topdown"], s["floor_any"]
        print(f"{label:30s} top-down floor {('%.3f-%.3f' % tuple(td)) if td else '   none    '} "
              f"any {('%.3f-%.3f' % tuple(an)) if an else 'none'}  bin drop "
              f"{('%.0f deg' % s['bin_drop'][2]) if s['bin_drop'] else 'no'}  stow top "
              f"{('%.3f' % s['stow'][0]) if s['stow'] else 'none'}")
        sys.stdout.flush()
    nose = [r for r in rows if r["label"].startswith("nose") and r["floor_topdown"] and r["bin_drop"] and r["stow"]]
    best = max(nose, key=lambda r: (r["floor_topdown"][1] - max(r["floor_topdown"][0], BUMPER_R)))
    p = params(best["mount"][0], best["mount"][1])
    side = side_reach(Fk(p), best["topdown_poses"])
    print(f"chosen: {best['label']}")
    for s in side:
        print(f"  pan {s['pan_deg']:5.0f}: {s['near'][0]:.3f}-{s['far'][0]:.3f} m from the centre")
    for r in rows:
        r.pop("topdown_poses")
    save("arm_mount", dict(rows=rows, chosen=best["label"], chosen_mount=best["mount"], side=side, bin=BIN,
                           pods=dict(x=POD_X, z=POD_Z, tilt_deg=POD_TILT)))


if __name__ == "__main__":
    main()
