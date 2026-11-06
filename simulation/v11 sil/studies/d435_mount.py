"""Where the arm robot's D435 should sit (design.md 4.11): today's nose mount against mounts higher up, on the arm's
riser in front of the arm and at head height over the stowed arm, each at a few tilts. The camera is used with the arm
stowed, mostly to see obstacles for navigation, toys and tags; the trade against picking (the tidy's close look) is
scored too. A study: SimParams overrides (d435_pos, d435_tilt_deg, d435_roll_deg), not a change to
robot_export.ARM_D435. Portrait mounts (roll 90: the 87 deg field tall, 58 deg across) are scored the same way.

For each mount (camera body centre in the robot frame, its tilt below the horizontal), with the arm stowed:

  floor       the D435's depth (sil_pick.PickCameras, depth_noise's realistic errors) of a bare arena floor: the nearest
              floor seen from the robot's centre (the blind zone) and the share of floor pixels that come back valid in
              bands 0.5, 1.0, 1.5, 2.0 m ahead (grazing angles lose them); the height noise at 1.5 m (sd of the floor's
              heights over the fitted plane: what an obstacle must stand clear of)
  obstacle    a shoe (11 cm, under the lidar's 163 mm plane) straight ahead at 0.5-2.5 m: the farthest range at which
              >= 30 of its pixels stand >= 3 cm over the floor (height_map's OBS_H), and the nearest at which it is still
              in view (it leaves the view under the camera's lower edge)
  overhang    a table top (1.0 m across, 0.6 m deep) with its underside 0.30 m and 0.45 m up, its near edge straight ahead
              at 0.3-2.5 m: the ranges at which >= 30 of its pixels are seen (the lidar's plane passes under both; the
              robot stands 0.60 m tall)
  arm         the share of the depth image the robot itself fills (the stowed arm, the bumper, the deck), by a
              segmentation render: what the camera can't see past
  tags        d435_tilt.tag_test: the share of free-floor house poses (8 headings each) that see at least one wall tag
              at 0.9 m and 1.3 m in the colour image
  pick        d435_tilt's close look (the tidy's CHECK at the reach's sweet spot, 8 small things x 5 offsets, floor_vision)
              and the standoff the look needs short of the sweet spot (the base then creeps in)

    python studies/d435_mount.py --quick                        every mount: floor, shoe, arm -> results/d435_mount.json
    python studies/d435_mount.py --only nose:7.5,riser:30 --out d435_mount_full.json   + tags and the close look
    python studies/d435_mount.py --only riser:30,riser_p:30 --overhang --out x.json      + the table tops
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import arm_objects  # noqa: E402
import arm_so101  # noqa: E402
from robot_export import arm_d435_params  # noqa: E402
from robot_model import build_model  # noqa: E402
from sil_link import SimLink  # noqa: E402,F401  (puts ros/src on the path)
from sil_pick import HOME, PickCameras  # noqa: E402
import d435_tilt as dt  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402

OUT = Path(__file__).resolve().parent / "results"
# (name, body centre (x, y, z) m in the robot frame, tilts deg below the horizontal, roll deg: 90 = portrait)
MOUNTS = [
    ("nose", (0.19, 0.0, 0.1115), (7.5, 15.0, 20.0), 0.0),       # today's (cad/v12 round 14 draws 7.5)
    ("riser", (0.20, 0.0, 0.215), (5.0, 10.0, 20.0, 30.0, 35.0, 40.0), 0.0),   # the arm riser's front, under the arm's base
    # the same, on its side (portrait), 15 mm lower: its 90 mm body at 0.215 meets the arm (omni_arm keep-outs)
    ("riser_p", (0.20, 0.0, 0.20), (20.0, 25.0, 30.0, 35.0), 90.0),
    # Nov 5, baselined (cad/v13): landscape in a pocket in the riser's front under the adapter (robot_export.D435_RISER)
    ("riser_cad", (0.1908, 0.0, 0.193), (25.0, 30.0, 35.0, 40.0), 0.0),
    ("head", (0.08, 0.0, 0.42), (20.0, 30.0, 40.0), 0.0),        # head height, looking over the stowed arm
    ("head_fwd", (0.16, 0.0, 0.40), (20.0, 30.0, 40.0), 0.0),    # head height, further forward
]
BANDS = (0.5, 1.0, 1.5, 2.0)
SHOE = ("box", (0.10, 0.28, 0.11))                                # across the view: 28 cm wide, 11 cm tall
TABLE = (0.60, 1.00, 0.03)                                        # a table top: deep, across, thick (m)
UNDERSIDES = (0.30, 0.45)                                         # m: a low table or bed frame; a table or chair seat


def model_at(pos, tilt, things=(), at=None, roll=0.0):
    """The arm robot in the arena at HOME with the D435 at pos / tilt (deg below the horizontal) / roll, the arm
    stowed; things (catalog names) parked, or at (x, y) world places."""
    names = list(things)
    xy = at or [(arm_objects.PARKED_X + 2 * k, 0.0) for k in range(len(names))]
    objs = arm_objects.objects_for(names, xy, [30.0] * len(names)) if names else ()
    p = arm_d435_params(d435_pos=tuple(pos), d435_tilt_deg=-float(tilt), d435_roll_deg=float(roll), home_xy=HOME,
                        thrower_xy=(-2.0, 0.0), area_size=(9.0, 9.0), floor_objects=objs)
    m = build_model(p)
    d = mujoco.MjData(m)
    settle(m, d)
    return p, m, d


def settle(m, d, steps: int = 300):
    """The base settles on its wheels with the arm held at stow (only the views matter here, not the servos)."""
    arm = arm_so101.ArmServos(m, d, "stow")
    for _ in range(steps):
        mujoco.mj_step(m, d)
        d.qpos[arm.qadr] = arm.target
        d.qvel[arm.vadr] = 0.0
    mujoco.mj_forward(m, d)


def shoe_box(m, d, fwd, o, Rz):
    """A shoe as a free box: put the first floor object (box) fwd m straight ahead."""
    adr = m.joint("obj0").qposadr[0]
    w = o + Rz @ np.array([fwd, 0.0, 0.0])
    d.qpos[adr:adr + 7] = [w[0], w[1], SHOE[1][2] / 2 + 0.001, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)


def robot_share(m, d, cams) -> float:
    """The share of the depth image that is the robot itself (any body under the base)."""
    r = mujoco.Renderer(cams.mr, cams.DEPTH_WH[1], cams.DEPTH_WH[0])
    r.enable_segmentation_rendering()
    cams._sync()
    r.update_scene(cams.dr, camera=cams.cl)
    seg = r.render()
    r.close()
    base = m.body("base").id
    root = np.array([m.body_rootid[m.geom_bodyid[g]] for g in range(m.ngeom)])
    is_robot = np.r_[root == m.body_rootid[base], False]
    gid = np.where((seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (seg[..., 0] >= 0), seg[..., 0], -1)
    return float(is_robot[gid].mean())


def floor_and_shoe(pos, tilt, seed=0, roll=0.0) -> dict:
    p, m, d = model_at(pos, tilt, things=["block"], roll=roll)                # obj0 reshaped below into a shoe
    gi = m.geom("obj0").id
    m.geom_size[gi] = np.array(SHOE[1]) / 2
    m.geom_rgba[gi] = (0.25, 0.18, 0.12, 1)                       # dark brown leather (albedo ~0.18)
    cams = PickCameras(m, d, p, seed=seed)
    o, yaw = dt._yaw_frame(m, d)
    Rz = dt._rz(yaw)
    cpos, R = dt.mount(m, d)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cpos
    out = {"cam_xyz": [round(float(v), 3) for v in cpos], "robot_share": round(robot_share(m, d, cams), 3)}
    # the bare floor
    z = cams.depth()
    fx, fy, cx, cy = cams.Kd
    H, W = z.shape
    v, u = np.mgrid[0:H, 0:W]
    ray = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1) @ R.T
    down = -ray[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        s_floor = np.where(down > 1e-6, cpos[2] / down, np.inf)    # the true floor along each ray (scale on ray)
        ahead = cpos[0] + s_floor * ray[..., 0]
    floor_px = np.isfinite(s_floor) & (s_floor < 12)
    seen = floor_px & np.isfinite(z)
    out["blind_m"] = round(float(np.min(ahead[floor_px])), 3) if floor_px.any() else None
    out["valid"] = {}
    for b in BANDS:
        band = floor_px & (np.abs(ahead - b) < 0.1)
        out["valid"][str(b)] = round(float(seen[band].sum() / max(band.sum(), 1)), 2) if band.any() else 0.0
    P = fv.points(fv.DepthFrame(0.0, z, cams.Kd, T))
    n_, d_ = fv._floor_plane(P, np.random.default_rng(0))
    h = P @ n_ + d_
    band = np.abs(P[:, 0] - 1.5) < 0.1
    out["h_sd_1.5"] = round(float(np.std(h[band])) * 1000, 1) if band.sum() > 20 else None
    # a shoe ahead
    far_ok, near_seen = None, None
    for fwd in np.arange(0.40, 2.61, 0.1):
        shoe_box(m, d, fwd, o, Rz)
        z = cams.depth()
        P = fv.points(fv.DepthFrame(0.0, z, cams.Kd, T))
        hh = P @ n_ + d_
        on = (np.abs(P[:, 0] - fwd) < 0.08) & (np.abs(P[:, 1]) < 0.16) & (hh > 0.03)
        if on.sum() >= 30:
            far_ok = round(float(fwd), 2)
            if near_seen is None:
                near_seen = round(float(fwd), 2)
    out["shoe_seen_m"] = [near_seen, far_ok]
    cams.close()
    return out


def overhang(pos, tilt, seed=0, roll=0.0) -> dict:
    """For each table top's underside height: the near edge's ranges ahead (0.3-2.5 m) at which the camera sees >= 30
    of its pixels, as [nearest, farthest] (None: never)."""
    p, m, d = model_at(pos, tilt, things=["block"], roll=roll)
    gi = m.geom("obj0").id
    m.geom_size[gi] = np.array(TABLE) / 2
    m.geom_rgba[gi] = (0.55, 0.40, 0.25, 1)                       # wood
    cams = PickCameras(m, d, p, seed=seed)
    o, yaw = dt._yaw_frame(m, d)
    Rz = dt._rz(yaw)
    cpos, R = dt.mount(m, d)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cpos
    adr = m.joint("obj0").qposadr[0]
    out = {}
    for under in UNDERSIDES:
        seen = []
        for edge in np.arange(0.30, 2.51, 0.1):
            w = o + Rz @ np.array([edge + TABLE[0] / 2, 0.0, 0.0])
            d.qpos[adr:adr + 7] = [w[0], w[1], under + TABLE[2] / 2, 1, 0, 0, 0]
            mujoco.mj_forward(m, d)
            P = fv.points(fv.DepthFrame(0.0, cams.depth(), cams.Kd, T))
            on = (P[:, 0] > edge - 0.05) & (P[:, 0] < edge + TABLE[0] + 0.05) & (np.abs(P[:, 1]) < TABLE[1] / 2) & \
                 (P[:, 2] > under - 0.05)
            if on.sum() >= 30:
                seen.append(round(float(edge), 2))
        out[f"{under:.2f}"] = [seen[0], seen[-1]] if seen else None
    cams.close()
    return out


def pick(pos, tilt, roll=0.0) -> dict:
    """d435_tilt's close look with the camera here (its arena, but at this mount)."""
    orig = dt.arena
    settle_arm = settle

    def arena_here(t, things=dt.THINGS, settle: int = 200):
        objs = arm_objects.objects_for(things, [(arm_objects.PARKED_X + 2 * k, 0.0) for k in range(len(things))],
                                       [30.0] * len(things))
        p = arm_d435_params(d435_pos=tuple(pos), d435_tilt_deg=-float(tilt), d435_roll_deg=float(roll), home_xy=HOME,
                            thrower_xy=(-2.0, 0.0), area_size=(9.0, 9.0), floor_objects=objs)
        m = build_model(p)
        d = mujoco.MjData(m)
        settle_arm(m, d, settle)
        return p, m, d
    dt.arena = arena_here
    try:
        r = dt.look_test(-float(tilt))
    finally:
        dt.arena = orig
    c = sum(x["found"] for x in r["close"])
    return {"close": f"{c}/{len(r['close'])}", "standoff": r["standoff"],
            "far": f"{sum(x['found'] for x in r['far'])}/{len(r['far'])}"}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quick", action="store_true", help="no tags, no pick (floor, shoe, arm share only)")
    ap.add_argument("--only", default="", help="name:tilt,... (e.g. nose:7.5,riser:30): just these mounts")
    ap.add_argument("--overhang", action="store_true", help="also the table tops")
    ap.add_argument("--out", default="d435_mount.json")
    a = ap.parse_args()
    only = {(k.split(":")[0], float(k.split(":")[1])) for k in a.only.split(",") if k}
    if not a.quick:
        hm_, hd = dt.house_world()
        from tag_sim import TagWorld
        worlds = {hz: TagWorld(hm_, hd, heights=(hz,), dock=False) for hz in (0.9, 1.3)}
        poses = dt.free_poses(hm_, hd, 0.5)
    rows = []
    for name, pos, tilts, roll in MOUNTS:
        for tilt in tilts:
            if only and (name, float(tilt)) not in only:
                continue
            r = {"mount": name, "pos": list(pos), "tilt": tilt, "roll": roll}
            r.update(floor_and_shoe(pos, tilt, roll=roll))
            if a.overhang:
                r["overhang"] = overhang(pos, tilt, roll=roll)
            if not a.quick:
                _, m, d = model_at(pos, tilt, roll=roll)
                cp, R = dt.mount(m, d, "d435_color")
                portrait = roll % 180 >= 45
                tag_cams = [(n, *((h, w) if portrait else (w, h)), dt.wide_fovy(f, w, h) if portrait else f)
                            for n, w, h, f in dt.TAG_CAMS]
                tags = dt.tag_test(cp, R, worlds, poses, tag_cams)
                r["tags_rgb"] = {k.split("@")[1]: v["any"] for k, v in tags.items() if k.startswith("rgb")}
                r["pick"] = pick(pos, tilt, roll)
            rows.append(r)
            print(json.dumps(r), flush=True)
    OUT.mkdir(exist_ok=True)
    (OUT / a.out).write_text(json.dumps(rows, indent=1))
    print("wrote", OUT / a.out)


if __name__ == "__main__":
    main()
