"""The nose D435's tilt (design.md 4.11): one tilt for the arm's floor looks and for seeing the room ahead (AprilTags on
the walls). For each tilt (the view's centre below the horizontal, landscape as mounted):

  close look  the tidy's close look from the reach's sweet spot (pick.py CHECK: the object must be found there, or it is
              "gone"): each small catalog thing alone at the sweet spot and 2.5 cm off it (the approach's tolerance),
              the D435's depth rendered (sil_pick.PickCameras: its noise and min-Z, a 0.3 deg calibration error) and
              omni_arm's floor_vision.detect on it; a look passes when it finds the thing within 3 cm
  far look    the same at 0.6-1.5 m (the LOOK before the approach; the detector reaches 2 m)
  tags        robot poses over the house's free floor (8 headings each), tags every 2.4 m on every wall face as
              tag_sim.TagWorld puts them, all at one height; a tag counts when tag_sim.TagCamera's rules give it a
              detection chance >= 0.5 (whole in the image, <= 7 m, <= 75 deg off its face, not behind a wall, side
              >= 22 px) in the D435's left infrared image (1280 x 720, the depth's 87 x 58 deg; global shutter) or its
              colour image (1280 x 720, 69 x 42 deg). Reported: the poses that see at least one tag, and the range
              band in which a tag straight ahead stays whole in the image
  views       the furnished house (props): the depth camera's view and the colour camera's from one pose, a third-person
              view with both fields drawn, and a side elevation of the fields

    python studies/d435_tilt.py                       the sweep -> results/d435_tilt.json
    python studies/d435_tilt.py --views -30 -15       views of these tilts -> results/d435_tilt/
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

import arm_objects  # noqa: E402
import house  # noqa: E402
from robot_export import arm_d435_params  # noqa: E402
from robot_model import D435, build_model  # noqa: E402
from sil_link import SimLink  # noqa: E402,F401  (puts ros/src on the path)
from sil_pick import HOME, PickCameras  # noqa: E402
from tag_sim import TagWorld  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402

OUT = Path(__file__).resolve().parent / "results"
SWEET = (0.31, 0.0)
TOL = 0.025                                         # m: pick.py's approach tolerance
CLOSE = [(SWEET[0], SWEET[1]), (SWEET[0] - TOL, 0.0), (SWEET[0] + TOL, 0.0), (SWEET[0], TOL), (SWEET[0], -TOL)]
FAR = [(0.6, 0.0), (1.0, 0.25), (1.5, -0.3)]
THINGS = ("cube30", "cube40", "brick", "crayon", "marker", "ball40", "cap", "block")
TILTS = tuple(np.arange(-35.0, 0.1, 2.5))
TAG_HEIGHTS = (0.3, 0.6, 0.9, 1.3)                  # m: the sim's tags are at 0.9 and 1.3 (tag_sim), the dock's at 0.25
# the D435's tag images: (name, width, height, vertical fov deg) -- the left IR imager shares the depth's field
TAG_CAMS = (("ir", 1280, 720, D435["depth_fovy_deg"]), ("rgb", 1280, 720, D435["color_fovy_deg"]))
MJ_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])


def _yaw_frame(m, d):
    b = m.body("base").id
    R = d.xmat[b].reshape(3, 3)
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([d.xpos[b][0], d.xpos[b][1], 0.0]), yaw


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def arena(tilt: float, things=THINGS, settle: int = 200):
    """The arm robot with the D435 at tilt in the arena at HOME, every test thing parked (floating, out of the world)."""
    objs = arm_objects.objects_for(things, [(arm_objects.PARKED_X + 2 * k, 0.0) for k in range(len(things))],
                                   [30.0] * len(things))
    p = arm_d435_params(d435_tilt_deg=tilt, home_xy=HOME, thrower_xy=(-2.0, 0.0), area_size=(9.0, 9.0),
                        floor_objects=objs)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(settle):
        mujoco.mj_step(m, d)
    return p, m, d


def mount(m, d, cam: str = "d435_depth"):
    """(pos, R optical) of a camera in the robot's level frame on the floor (x forward, z up), as the robot stands."""
    o, yaw = _yaw_frame(m, d)
    c = m.camera(cam).id
    Rz = _rz(yaw)
    return Rz.T @ (d.cam_xpos[c] - o), Rz.T @ d.cam_xmat[c].reshape(3, 3) @ MJ_TO_OPTICAL


def look_test(tilt: float, seed: int = 0) -> dict:
    """The close and far looks at this tilt: for each place, the things the detector found there; and the standoff,
    the least distance (0-0.2 m in 2.5 cm steps) the close look would have to be taken short of the sweet spot (the
    base then creeping in on odometry) for every thing at every offset to be found."""
    p, m, d = arena(tilt)
    cams = PickCameras(m, d, p, seed=seed)
    pos, R = mount(m, d)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, pos
    o, yaw = _yaw_frame(m, d)
    Rz = _rz(yaw)
    out = {}
    sets = [("close", CLOSE), ("far", FAR)] + [(f"s{s:.3f}", [(x + s, y) for x, y in CLOSE])
                                              for s in np.arange(0.025, 0.201, 0.025)]
    for label, places in sets:
        if label.startswith("s") and out.get("standoff") is not None:
            continue
        res = []
        for k, name in enumerate(THINGS):
            adr = m.joint(f"obj{k}").qposadr[0]
            park = d.qpos[adr:adr + 7].copy()
            h = arm_objects.height(arm_objects.objects_for([name], [(0, 0)], [0])[0])
            for fx, ly in places:
                w = o + Rz @ np.array([fx, ly, 0.0])
                a = math.radians(30.0) / 2
                d.qpos[adr:adr + 7] = [w[0], w[1], h / 2 + 0.0005, math.cos(a + yaw / 2), 0, 0, math.sin(a + yaw / 2)]
                z = cams.depth()
                dets = fv.detect(fv.DepthFrame(0.0, z, cams.Kd, T), rng=np.random.default_rng(7))
                hit = any(math.hypot(e.x - fx, e.y - ly) < 0.03 for e in dets)
                res.append(dict(thing=name, x=fx, y=ly, found=hit))
            d.qpos[adr:adr + 7] = park
        if label == "far":
            out[label] = res
            continue
        if label == "close":
            out[label] = res
        if all(x["found"] for x in res):
            out["standoff"] = 0.0 if label == "close" else float(label[1:])
        out.setdefault("standoff", None)
    cams.close()
    # where the depth field's lower and upper edges meet the floor, straight ahead
    fwd = R[:, 2]
    pitch = math.degrees(math.asin(-fwd[2]))                       # deg below the horizontal
    half = D435["depth_fovy_deg"] / 2
    floor_at = lambda dn: pos[0] + pos[2] / math.tan(math.radians(dn)) if dn > 0 else math.inf  # noqa: E731
    out["field"] = dict(cam_xyz=[round(float(v), 4) for v in pos], pitch_deg=round(pitch, 2),
                        depth_floor_near=round(floor_at(pitch + half), 3),
                        depth_top_deg=round(half - pitch, 1),
                        rgb_floor_near=round(floor_at(pitch + D435["color_fovy_deg"] / 2), 3),
                        rgb_top_deg=round(D435["color_fovy_deg"] / 2 - pitch, 1))
    return out


# -- tags ----------------------------------------------------------------------------------------------------------------

def house_world():
    """The house's walls (no robot needed) for TagWorld and the free-floor poses."""
    p = house.house_params("v8")
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def free_poses(m, d, spacing: float = 0.5, clear: float = 0.4, headings: int = 8) -> list:
    """(x, y, yaw) on a grid over the house: inside (rays at 0.3 m hit a wall every way within 12 m) and clear of
    every wall by clear."""
    walls = TagWorld(m, d, dock=False).walls
    xs = np.concatenate([w[0][None] for w in walls])
    lo, hi = xs.min(axis=0), xs.max(axis=0)
    gid = np.zeros(1, np.int32)
    out = []
    for x in np.arange(lo[0], hi[0], spacing):
        for y in np.arange(lo[1], hi[1], spacing):
            ok = True
            for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
                v = np.array([math.cos(a), math.sin(a), 0.0])
                r = mujoco.mj_ray(m, d, np.array([x, y, 0.3]), v, None, 1, -1, gid)
                if r < 0 or r > 12.0 or r < clear:
                    ok = False
                    break
            if ok:
                out += [(float(x), float(y), 2 * math.pi * k / headings) for k in range(headings)]
    return out


def _tag_p(world, tg, Rcw, pcw, cam) -> float:
    """tag_sim.TagCamera's detection chance for one tag, without the dice."""
    from omni_slam.tags import tag_corners
    corners_w = tg.t + tag_corners(tg.size) @ tg.R.T
    Pc = (corners_w - pcw) @ Rcw.T
    if np.any(Pc[:, 2] < 0.1):
        return 0.0
    rng_m = float(np.linalg.norm(tg.t - pcw))
    if rng_m > 7.0:
        return 0.0
    view = (pcw - tg.t) / rng_m
    if math.acos(max(-1.0, min(1.0, float(view @ tg.R[:, 2])))) > math.radians(75):
        return 0.0
    uv = cam.project(Pc)
    if np.any(uv[:, 0] < 5) or np.any(uv[:, 0] > cam.width - 5) or np.any(uv[:, 1] < 5) or \
            np.any(uv[:, 1] > cam.height - 5):
        return 0.0
    side = float(np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]))
    p = 0.97 * min(1.0, max(0.0, (side - 12.0) / 20.0))
    return 0.0 if p < 0.5 or world.occluded(pcw, tg.t) else p


def wide_fovy(fovy: float, w: int, h: int) -> float:
    """The tall field of a w x h camera (fovy tall) turned on its side."""
    return math.degrees(2 * math.atan(math.tan(math.radians(fovy) / 2) * w / h))


def tag_test(pos, R, worlds, poses, cams=TAG_CAMS) -> dict:
    """For each tag camera and tag height: the share of poses that see >= 1 tag, and the mean count seen."""
    from omni_slam.tags import Pinhole
    out = {}
    for name, w, h, fovy in cams:
        cam = Pinhole.from_fovy(fovy, w, h)
        for hz, world in worlds.items():
            seen = []
            for x, y, yaw in poses:
                Rz = _rz(yaw)
                pcw = np.array([x, y, 0.0]) + Rz @ pos
                Rcw = (Rz @ R).T
                seen.append(sum(_tag_p(world, tg, Rcw, pcw, cam) > 0 for tg in world.tags))
            seen = np.array(seen)
            out[f"{name}@{hz}"] = dict(any=round(float(np.mean(seen > 0)), 3), mean=round(float(seen.mean()), 2))
    return out


def ahead_band(pos, R, fovy: float, tag_z: float, size: float = 0.16, px_h: int = 480):
    """(near, far) m: a tag at tag_z straight ahead of the camera is whole in the image and >= 22 px tall."""
    pitch = math.asin(-R[2, 2])                                          # optical z (the view) below the horizontal
    half = math.radians(fovy) / 2
    top, bot = tag_z + size / 2 - pos[2], tag_z - size / 2 - pos[2]
    f = px_h / 2 / math.tan(half)
    near = 0.0
    if half - pitch <= 0 and top > 0:
        return None
    if top > 0:
        near = top / math.tan(half - pitch)
    if bot < 0 and pitch + half < math.pi / 2:
        near = max(near, -bot / math.tan(pitch + half))
    far = min(7.0, f * size / 22.0)
    if bot < 0 and pitch - half > 0:                     # the field's top edge is below the horizontal
        far = min(far, -bot / math.tan(pitch - half))
    if bot > 0 and pitch - half > -1e-9:
        return None
    return (round(near, 2), round(far, 2)) if near < far else None


def sweep(tilts=TILTS, spacing: float = 0.5):
    m, d = house_world()
    worlds = {hz: TagWorld(m, d, heights=(hz,), dock=False) for hz in TAG_HEIGHTS}
    poses = free_poses(m, d, spacing)
    print(f"{len(poses)} poses, {len(worlds[TAG_HEIGHTS[0]].tags)} tags per height")
    rows = []
    for t in tilts:
        r = look_test(float(t))
        _, ma, da = arena(float(t), settle=200)
        pos_ir, R_ir = mount(ma, da, "d435_depth")
        r["tags"] = tag_test(pos_ir, R_ir, worlds, poses)
        r["ahead"] = {f"{n}@{hz}": ahead_band(pos_ir, R_ir, fovy, hz, px_h=h)
                      for n, _, h, fovy in TAG_CAMS for hz in TAG_HEIGHTS}
        r["tilt"] = float(t)
        c = sum(x["found"] for x in r["close"])
        f = sum(x["found"] for x in r["far"])
        print(f"tilt {t:6.1f}: close {c:2d}/{len(r['close'])}  standoff {r['standoff']}  far {f:2d}/{len(r['far'])}  "
              f"floor from {r['field']['depth_floor_near']:.3f} m (rgb {r['field']['rgb_floor_near']:.3f}), "
              f"top {r['field']['depth_top_deg']:+.0f} deg (rgb {r['field']['rgb_top_deg']:+.0f})  "
              + "  ".join(f"{k} {v['any']:.2f}" for k, v in r["tags"].items()))
        rows.append(r)
    OUT.mkdir(exist_ok=True)
    (OUT / "d435_tilt.json").write_text(json.dumps(dict(poses=len(poses), rows=rows), indent=1))
    print(f"wrote {OUT / 'd435_tilt.json'}")
    return rows


# -- views ---------------------------------------------------------------------------------------------------------------

def _add_line(scn, a, b, rgba, width=3.0):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def _frustum(scn, m, d, cam: str, aspect: float, rgba, reach: float = 6.0):
    """The camera's field: its four edge rays to whatever they hit first (or reach m), and the outline joining them."""
    c = m.camera(cam).id
    p0, R = d.cam_xpos[c], d.cam_xmat[c].reshape(3, 3)
    ty = math.tan(math.radians(m.cam_fovy[c]) / 2)
    tx = ty * aspect
    ends = []
    gid = np.zeros(1, np.int32)
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        v = R @ np.array([sx * tx, sy * ty, -1.0])
        v /= np.linalg.norm(v)
        s = reach
        if v[2] < -1e-6:
            s = min(s, -p0[2] / v[2])
        hit = mujoco.mj_ray(m, d, p0 + 0.02 * v, v, None, 1, -1, gid)       # past its own box
        if hit >= 0:
            s = min(s, hit + 0.02)
        ends.append(p0 + v * s)
    for e in ends:
        _add_line(scn, p0, e, rgba)
    for k in range(4):
        _add_line(scn, ends[k], ends[(k + 1) % 4], rgba)


def view_model(tilt: float, pose, things=()):
    """The furnished house, the arm robot with the D435 at tilt at pose (x, y, yaw), things at (name, fwd, left)."""
    from bridge_core import CoreConfig, model_params
    x, y, yaw = pose
    Rz = _rz(yaw)
    names = [n for n, _, _ in things]
    xy = [tuple((np.array([x, y, 0]) + Rz @ np.array([f, l, 0]))[:2]) for _, f, l in things]
    p = model_params(CoreConfig(payload="arm", arm_camera="d435", props=True, n_objects=0))
    from dataclasses import replace
    p = replace(p, d435_tilt_deg=tilt, floor_objects=arm_objects.objects_for(names, xy, [25.0] * len(names)))
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 3] = [x, y, d.qpos[adr + 2]]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    for _ in range(300):
        mujoco.mj_step(m, d)
    return p, m, d


def views(tilts, pose, things, tag: str = "", out_dir: Path | None = None):
    from PIL import Image
    out_dir = out_dir or OUT / "d435_tilt"
    out_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for t in tilts:
        p, m, d = view_model(t, pose, things)
        m.vis.global_.offwidth, m.vis.global_.offheight = 1600, 900
        for cam, (W, H) in (("d435_depth", (848, 480)), ("d435_color", (1280, 720))):
            r = mujoco.Renderer(m, H, W)
            r.update_scene(d, camera=cam)
            f = out_dir / f"{tag}tilt{_tname(t)}_{'ir' if cam == 'd435_depth' else 'rgb'}.png"
            Image.fromarray(r.render()).save(f)
            r.close()
            files.append(f)
        # third person, from the robot's right and a little behind: the fields drawn to what they hit (green the depth /
        # IR, magenta the colour)
        r = mujoco.Renderer(m, 720, 1280)
        cam = mujoco.MjvCamera()
        b = m.body("base").id
        cam.lookat[:] = d.xpos[b] + _rz(pose[2]) @ np.array([1.1, 0.0, 0.0])
        cam.lookat[2] = 0.45
        cam.distance, cam.elevation = 3.2, -9.0
        cam.azimuth = math.degrees(pose[2]) + 80.0
        r.update_scene(d, camera=cam)
        r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        _frustum(r.scene, m, d, "d435_depth", 848 / 480, (0.1, 0.85, 0.2, 1.0))
        _frustum(r.scene, m, d, "d435_color", 1280 / 720, (0.9, 0.1, 0.8, 1.0))
        f = out_dir / f"{tag}tilt{_tname(t)}_3rd.png"
        Image.fromarray(r.render()).save(f)
        r.close()
        files.append(f)
    return files


def _tname(t: float) -> str:
    return f"{abs(t):04.1f}".replace(".", "p")


def side_elevation(tilts, path: Path, cam_xz=(0.1971, 0.1074), standoff: dict | None = None):
    """Side view of the depth/IR and colour fields for each tilt (left: the arm's floor, right: the room), with the
    reach, the close look's spot (standoff: tilt -> m short of the sweet spot), the tag heights and the lidar plane."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    standoff = standoff or {}
    fig, axes = plt.subplots(len(tilts), 2, figsize=(13, 3.0 * len(tilts)), gridspec_kw=dict(width_ratios=(1, 2.2)))
    axes = np.atleast_2d(axes)
    x0, z0 = cam_xz
    for (an, ar), t in zip(axes, tilts):
        for ax, xmax, zmax in ((an, 0.9, 0.32), (ar, 6.0, 1.6)):
            for fovy, col, lab in ((D435["depth_fovy_deg"], "#2a9d4b", "depth / IR (58° tall)"),
                                   (D435["color_fovy_deg"], "#c2187f", "colour (42° tall)")):
                pts = [(x0, z0)]
                for a in np.linspace(t - fovy / 2, t + fovy / 2, 90):
                    v = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
                    s = 8.0 if v[1] >= 0 else min(8.0, -z0 / v[1])
                    pts.append((x0 + v[0] * s, z0 + v[1] * s))
                ax.fill(*zip(*pts), color=col, alpha=0.18, lw=1.2, ec=col, label=lab)
            ax.axhline(0, color="k", lw=1)
            ax.axhline(0.163, color="#c00", lw=0.8, ls="--")
            ax.add_patch(plt.Rectangle((-0.217, 0.085), 0.434, 0.014, color="#888"))      # the deck (approx)
            ax.add_patch(plt.Rectangle((0.217, 0.03), 0.008, 0.06, color="#e07020"))      # the bumper's nose (approx)
            ax.plot([x0], [z0], "ks", ms=5)
            ax.set_xlim(0.0 if ax is an else -0.3, xmax)
            ax.set_ylim(-0.02, zmax)
        an.add_patch(plt.Rectangle((0.26, -0.012), 0.10, 0.012, color="#7b2cbf", label="arm reach"))
        s = standoff.get(t)
        if s:
            an.add_patch(plt.Rectangle((0.26 + s, -0.02), 0.10, 0.008, color="#1f6fd1", label=f"close look ({100 * s:.0f} cm back)"))
        an.text(0.01, 0.168, "lidar plane", color="#c00", fontsize=7)
        an.set_aspect("equal")
        an.set_ylabel("z (m)")
        an.set_title(f"{abs(t):g}° down — the arm's floor", fontsize=10, loc="left")
        an.legend(loc="upper left", bbox_to_anchor=(0.0, -0.32), fontsize=6.5, ncol=2, frameon=False)
        for hz in (0.9, 1.3):
            ar.axhspan(hz - 0.08, hz + 0.08, color="#555", alpha=0.18)
            ar.text(5.95, hz, f"tags at {hz} m", va="center", ha="right", fontsize=8)
        ar.set_title(f"{abs(t):g}° down — the room", fontsize=10, loc="left")
    axes[-1, 0].set_xlabel("x ahead of the robot's centre (m)")
    axes[-1, 1].set_xlabel("x ahead of the robot's centre (m)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilts", type=float, nargs="*", default=None)
    ap.add_argument("--views", type=float, nargs="*", default=None, help="render views of these tilts (deg, negative = down)")
    ap.add_argument("--pose", type=float, nargs=3, default=None, help="x y yaw(deg) in the house for the views")
    a = ap.parse_args()
    if a.views:
        pose = (a.pose[0], a.pose[1], math.radians(a.pose[2])) if a.pose else (house.START[0], house.START[1], 0.0)
        things = (("cube40", SWEET[0], SWEET[1]), ("ball40", 0.7, 0.2), ("block", 1.1, -0.3), ("cube30", 1.6, 0.1))
        for f in views(a.views, pose, things):
            print("wrote", f)
        rows = json.loads((OUT / "d435_tilt.json").read_text())["rows"] if (OUT / "d435_tilt.json").exists() else []
        s = {r["tilt"]: r["standoff"] for r in rows}
        print("wrote", side_elevation(a.views, OUT / "d435_tilt" / "side.png",
                                      # the standoff needed plus a step of margin, where today's close look fails
                                      standoff={t: s[t] + 0.025 for t in a.views if (s.get(t) or 0) > 0.025}))
        return
    sweep(a.tilts or TILTS)


if __name__ == "__main__":
    main()
