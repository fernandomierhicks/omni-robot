"""A forward IMX662 USB 2 camera at the arm robot's nose, under the lidar, tilted up (Nov 5 2026, study; imx662_cam.py, the CAD fit in
cad/v13/tools/imx662_fit.py). The D435 is on the riser at robot_export.D435_RISER (30 deg down): it sees the floor, not the walls. Does a
cheap rolling-shutter camera looking up give the robot tags (and people) while it drives, and does its optical flow help it go faster?

  tilt     geometry over the house's free floor (d435_tilt's poses, 8 headings): the share of poses that see >= 1 wall tag at 0.9 and
           1.3 m (tag_sim's rules, side >= 22 px), a standing adult (0..1.8 m) whole in the image, the nearest floor seen; the riser
           D435's colour camera for comparison; for tilts 0..30 deg up
  frames   rendered frames in the furnished house (props; the tags painted with their real 36h11 codes), each tag faced from 1.5 and
           3 m, the robot still or moving as it passes (driving 0.5-2 m/s, sideways 1 m/s, spinning 60-180 deg/s, driving with a
           pitch wobble), through imx662_cam's rolling shutter (readout 0 / 16 / 33 ms) and MJPEG -> DATA/frames/
  flow     12-frame sequences at 30 fps over the same motions -> DATA/flow/
           (DATA: $OMNI_IMX662_DATA, default ~/omni_sim_data/imx662, outside Dropbox: ~350 MB of JPEGs, regenerable)
  views    the camera's picture from a few poses (the person in the office, a hall) -> results/imx662/views/
The detections and the flow are scored by studies/imx662_detect.py in the realsense venv (pupil-apriltags, OpenCV), which reads the
saved JPEGs and truth.

    python studies/imx662_study.py tilt
    python studies/imx662_study.py frames [--quick]
    python studies/imx662_study.py flow
    python studies/imx662_study.py views
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import imx662_cam as IC  # noqa: E402
from robot_export import D435_RISER  # noqa: E402
from robot_model import D435, build_xml  # noqa: E402
from sil_link import SimLink  # noqa: E402,F401  (puts ros/src on the path)

OUT = Path(__file__).resolve().parent / "results" / "imx662"
DATA = Path(os.environ.get("OMNI_IMX662_DATA", Path.home() / "omni_sim_data" / "imx662"))   # the rendered frames (~350 MB), outside Dropbox
CACHE = DATA / "cache"
# the mount: cad/v13/tools/imx662_fit.py's default (lens front (214, 0, 105) mm, the pupil 4 mm behind it), tilt chosen by `tilt`
LENS_FRONT, PUPIL_BACK = (0.214, 0.0, 0.105), 0.004
TILT_UP = 20.0
TILTS = tuple(float(t) for t in range(0, 31, 5))
MOTIONS = {                                              # name: (forward m/s, left m/s, yaw deg/s, wobble (deg, Hz))
    "still": (0.0, 0.0, 0.0, None),
    "fwd0.5": (0.5, 0.0, 0.0, None),
    "fwd1": (1.0, 0.0, 0.0, None),
    "fwd2": (2.0, 0.0, 0.0, None),
    "side1": (0.0, 1.0, 0.0, None),
    "spin60": (0.0, 0.0, 60.0, None),
    "spin120": (0.0, 0.0, 120.0, None),
    "spin180": (0.0, 0.0, 180.0, None),
    "fwd1_wobble": (1.0, 0.0, 0.0, (0.5, 8.0)),          # pods on a seam / rug edge: 0.5 deg pitch at 8 Hz (est)
}
READOUTS = (0.0, 0.016, 0.033)
EXPOSURES = (0.004, 0.012)                               # bright room, dim room (est)


def mount(tilt=TILT_UP) -> IC.Mount:
    t = math.radians(tilt)
    fwd = np.array([math.cos(t), 0.0, math.sin(t)])
    return IC.Mount(pupil=tuple(np.asarray(LENS_FRONT) - fwd * PUPIL_BACK), tilt_up_deg=tilt)


# -- geometry -----------------------------------------------------------------------------------------------------------------------------

def _person_band(pos, R, fovy, top=1.8):
    """(near, far) m: a standing adult (feet 0, head top `top`) straight ahead is whole in the image"""
    pitch = math.asin(R[2, 2])                                         # the optical axis above the horizontal
    half = math.radians(fovy) / 2
    if pitch + half <= 0:
        return None
    h_up, h_dn = top - pos[2], pos[2]
    near_top = h_up / math.tan(pitch + half) if pitch + half < math.pi / 2 else 0.0
    near_bot = h_dn / math.tan(half - pitch) if half - pitch > 0 else math.inf
    return round(max(near_top, near_bot), 2)


def tilt_sweep():
    import d435_tilt as dt
    from tag_sim import TagWorld
    m, d = dt.house_world()
    heights = (0.9, 1.3)
    worlds = {hz: TagWorld(m, d, heights=(hz,), dock=False) for hz in heights}
    poses = dt.free_poses(m, d, 0.5)
    print(f"{len(poses)} poses", flush=True)
    rows = []
    cfg = IC.IMX662Config()
    cams = (("imx662", cfg.width, cfg.height, cfg.fovy_deg),)
    for t in TILTS:
        mt = mount(t)
        fwd, right, up = mt.axes()
        R = np.column_stack([right, -up, fwd])                          # optical: x right, y down, z forward
        pos = np.asarray(mt.pupil)
        r = dict(tilt_up=t, pupil=[round(float(v), 4) for v in pos])
        r["tags"] = dt.tag_test(pos, R, worlds, poses, cams)
        r["tag_ahead_band"] = {f"{hz}": dt.ahead_band(pos, R, cfg.fovy_deg, hz, px_h=cfg.height) for hz in heights}
        r["adult_whole_from_m"] = _person_band(pos, R, cfg.fovy_deg)
        r["face_in_view_from_m"] = _person_band(pos, R, cfg.fovy_deg, top=1.75)
        lo = math.radians(t - cfg.fovy_deg / 2)
        r["floor_from_m"] = round(pos[0] + pos[2] / math.tan(-lo), 2) if lo < 0 else None
        r["field_top_deg"] = round(t + cfg.fovy_deg / 2, 1)
        rows.append(r)
        print(t, r["tags"], r["adult_whole_from_m"], r["floor_from_m"], flush=True)
    # the riser D435's colour camera, as baselined (robot_export.D435_RISER)
    x, y, z = D435_RISER["d435_pos"]
    tl = math.radians(D435_RISER["d435_tilt_deg"])
    fwd = np.array([math.cos(tl), 0.0, math.sin(tl)])
    right = np.array([0.0, -1.0, 0.0])
    up = np.cross(right, fwd)
    cpos = np.array([x, y, z]) + fwd * D435["color_at"][0] + np.array([0, D435["color_at"][1], 0])
    Rc = np.column_stack([right, -up, fwd])
    ref = dict(camera="D435 colour (riser, robot_export.D435_RISER)", pos=[round(float(v), 4) for v in cpos],
               tags=dt.tag_test(cpos, Rc, worlds, poses, (("d435_rgb", 1280, 720, D435["color_fovy_deg"]),)),
               adult_whole_from_m=_person_band(cpos, Rc, D435["color_fovy_deg"]))
    print("D435 riser colour", ref, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "tilt.json").write_text(json.dumps(dict(poses=len(poses), rows=rows, d435_riser=ref), indent=1))
    print("wrote", OUT / "tilt.json")


# -- the rendered house -------------------------------------------------------------------------------------------------------------------

def house_xml() -> str:
    """the furnished house with the arm robot (D435 on the riser as baselined), cached: build_xml takes ~1 min"""
    from bridge_core import CoreConfig, model_params
    p = model_params(CoreConfig(payload="arm", arm_camera="d435", props=True, n_objects=0))
    key = hashlib.sha1(repr(p).encode()).hexdigest()[:12]
    f = CACHE / f"house_{key}.xml"
    if not f.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        f.write_text(build_xml(p), encoding="utf-8")
    return f.read_text(encoding="utf-8"), p


class Scene:
    """the house model with the IMX662, the robot settled at START, a kinematic pose setter"""

    def __init__(self, tilt=TILT_UP):
        xml, self.p = house_xml()
        self.mt = mount(tilt)
        self.m = mujoco.MjModel.from_xml_string(IC.with_camera(xml, self.mt))
        self.painted = IC.real_tags(self.m)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        for _ in range(300):
            mujoco.mj_step(self.m, self.d)
        self.adr = self.m.joint("base").qposadr[0]
        self.q0 = self.d.qpos.copy()
        self.z0 = float(self.q0[self.adr + 2])

    def set(self, x, y, yaw, pitch=0.0):
        q = self.d.qpos
        q[:] = self.q0
        q[self.adr:self.adr + 3] = [x, y, self.z0]
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        q[self.adr + 3:self.adr + 7] = [cy * cp, -sy * sp, cy * sp, sy * cp]          # Rz(yaw) Ry(pitch)
        self.d.qvel[:] = 0

    def pose_fn(self, x, y, yaw, motion, tc):
        """pose at time t for a robot passing (x, y, yaw) at time tc with this motion (robot-frame velocities)"""
        vx, vy, wz, wob = MOTIONS[motion] if isinstance(motion, str) else motion
        w = math.radians(wz)

        def f(t):
            s = t - tc
            if abs(w) > 1e-9:                                             # constant twist: an arc (or a spin in place)
                a = yaw + w * s
                dx = (vx * (math.sin(a) - math.sin(yaw)) + vy * (math.cos(a) - math.cos(yaw))) / w
                dy = (-vx * (math.cos(a) - math.cos(yaw)) + vy * (math.sin(a) - math.sin(yaw))) / w
            else:
                a = yaw
                dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * s
                dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * s
            pitch = math.radians(wob[0]) * math.sin(2 * math.pi * wob[1] * t) if wob else 0.0
            self.set(x + dx, y + dy, a, pitch)
        return f

    def clear(self, x, y, r=0.32) -> bool:
        """nothing within r of the robot's centre at bumper height and over it (the furniture included; the robot parked away)"""
        self.set(50.0, 50.0, 0.0)
        mujoco.mj_kinematics(self.m, self.d)
        gid = np.zeros(1, np.int32)
        for z in (0.06, 0.3):
            for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
                h = mujoco.mj_ray(self.m, self.d, np.array([x, y, z]), np.array([math.cos(a), math.sin(a), 0.0]), None, 1, -1, gid)
                if 0 <= h < r:
                    return False
        return True


def tag_list(sc: Scene):
    from house_props import tag_places
    return [(tid, t, R, size) for tid, t, R, size in tag_places(sc.p) if tid in sc.painted]


def test_poses(sc: Scene, dists=(1.5, 3.0), limit=None):
    """(name, x, y, yaw, tag id) facing each tag from each distance, where the robot fits (yaw: straight at the tag)"""
    out = []
    for tid, t, R, size in tag_list(sc):
        n = R[:, 2]
        for D in dists:
            c = t[:2] + n[:2] * D
            if sc.clear(c[0], c[1]):
                out.append((f"t{tid}_d{D:g}", float(c[0]), float(c[1]), math.atan2(-n[1], -n[0]), tid))
    return out[:limit] if limit else out


def truth(sc: Scene, tags):
    """the camera's world pose now and each tag's pose in the camera's optical frame (x right, y down, z forward)"""
    pc, Rc = IC.camera_pose(sc.m, sc.d)
    out = {}
    for tid, t, R, size in tags:
        out[str(tid)] = dict(t_cam=list(np.round(Rc.T @ (t - pc), 5)), normal_cam=list(np.round(Rc.T @ R[:, 2], 5)),
                             up_cam=list(np.round(Rc.T @ R[:, 1], 5)), size=size)
    return dict(cam_pos=list(np.round(pc, 5)), cam_R=np.round(Rc, 6).tolist(), tags=out)


def frames(quick=False):
    sc = Scene()
    tags = tag_list(sc)
    poses = test_poses(sc, limit=6 if quick else None)
    print(f"{len(tags)} tags painted, {len(poses)} test poses", flush=True)
    cfg0 = IC.IMX662Config()
    K = dict(fx=cfg0.fx(), fy=cfg0.fx(), cx=cfg0.width / 2, cy=cfg0.height / 2, width=cfg0.width, height=cfg0.height)
    root = DATA / "frames"
    root.mkdir(parents=True, exist_ok=True)
    cases = [(mo, ro, ex) for ex in EXPOSURES for ro in READOUTS for mo in MOTIONS
             if not (ex == 0.012 and mo in ("fwd0.5", "spin60"))]
    meta = dict(K=K, mount=dict(pupil=sc.mt.pupil, tilt_up_deg=sc.mt.tilt_up_deg), motions=MOTIONS, cases=[])
    cams = {}
    for mo, ro, ex in cases:
        key = (ro, ex)
        if key not in cams:
            cams[key] = IC.RollingShutterCamera(sc.m, sc.d, IC.IMX662Config(readout_s=ro, exposure_s=ex), seed=len(cams))
        cam = cams[key]
        name = f"{mo}_ro{1000 * ro:.0f}_ex{1000 * ex:.0f}"
        dd = root / name
        dd.mkdir(exist_ok=True)
        rows = []
        for pn, x, y, yaw, tid in poses:
            t0 = 1.0
            tc = t0 + ro / 2                                              # the middle row passes the test pose
            f = sc.pose_fn(x, y, yaw, mo, tc)
            img, info = cam.frame(f, t0)
            (dd / f"{pn}.jpg").write_bytes(cam.last_jpeg)
            f(info["t_mid"])
            mujoco.mj_kinematics(sc.m, sc.d)
            rows.append(dict(pose=pn, xyyaw=[x, y, yaw], target=tid, renders=info["renders"], **truth(sc, tags)))
        (dd / "truth.json").write_text(json.dumps(rows))
        meta["cases"].append(dict(name=name, motion=mo, readout=ro, exposure=ex, n=len(rows)))
        print(name, len(rows), "frames", f"{np.mean(cam.n_renders[-len(rows):]):.0f} renders/frame", flush=True)
    (root / "meta.json").write_text(json.dumps(meta, indent=1))
    for c in cams.values():
        c.close()


# -- optical flow sequences ---------------------------------------------------------------------------------------------------------------

FLOW_MOTIONS = ("fwd0.5", "fwd1", "fwd2", "side1", "spin60", "spin180", "fwd1_wobble")


def flow_starts(sc: Scene, n=4):
    """open floor with room ahead: (x, y, yaw) clear for 1.5 m along the heading"""
    out = []
    rng = np.random.default_rng(4)
    gid = np.zeros(1, np.int32)
    while len(out) < n:
        x, y, yaw = rng.uniform(-5.5, 5.5), rng.uniform(-4.0, 4.0), rng.uniform(-math.pi, math.pi)
        if not sc.clear(x, y, 0.45):
            continue
        h = mujoco.mj_ray(sc.m, sc.d, np.array([x, y, 0.1]), np.array([math.cos(yaw), math.sin(yaw), 0.0]), None, 1, -1, gid)
        if 1.8 < h < 6.0:
            out.append((float(x), float(y), float(yaw)))
    return out


def flow(n_frames=12):
    sc = Scene()
    starts = flow_starts(sc)
    root = DATA / "flow"
    root.mkdir(parents=True, exist_ok=True)
    cfg0 = IC.IMX662Config()
    meta = dict(K=dict(fx=cfg0.fx(), cx=cfg0.width / 2, cy=cfg0.height / 2, width=cfg0.width, height=cfg0.height),
                fps=cfg0.fps, starts=starts, cases=[])
    for ro in READOUTS:
        cam = IC.RollingShutterCamera(sc.m, sc.d, IC.IMX662Config(readout_s=ro), seed=7)
        for mo in FLOW_MOTIONS:
            for k, (x, y, yaw) in enumerate(starts):
                name = f"{mo}_ro{1000 * ro:.0f}_s{k}"
                dd = root / name
                dd.mkdir(exist_ok=True)
                f = sc.pose_fn(x, y, yaw, mo, 1.0)
                rows = []
                for j in range(n_frames):
                    t0 = 1.0 + j / cfg0.fps
                    img, info = cam.frame(f, t0)
                    (dd / f"{j:02d}.jpg").write_bytes(cam.last_jpeg)
                    f(info["t_mid"])
                    mujoco.mj_kinematics(sc.m, sc.d)
                    pc, Rc = IC.camera_pose(sc.m, sc.d)
                    rows.append(dict(t_mid=info["t_mid"], cam_pos=list(pc), cam_R=Rc.tolist()))
                (dd / "truth.json").write_text(json.dumps(rows))
                meta["cases"].append(dict(name=name, motion=mo, readout=ro, start=k))
            print("flow", mo, ro, flush=True)
        cam.close()
    (root / "meta.json").write_text(json.dumps(meta, indent=1))


# -- views ----------------------------------------------------------------------------------------------------------------------------------

VIEW_POSES = {"office_person": (1.1 + 2.2 * math.cos(math.radians(200)), 3.0 + 2.2 * math.sin(math.radians(200)), math.radians(20)),
              "hall": (-3.0, 1.25, 0.0), "kitchen": (1.6, -0.4, math.radians(-90))}


def views():
    from PIL import Image
    sc = Scene()
    dd = OUT / "views"
    dd.mkdir(parents=True, exist_ok=True)
    cams = {ro: IC.RollingShutterCamera(sc.m, sc.d, IC.IMX662Config(readout_s=ro, exposure_s=0.008), seed=1) for ro in (0.0, 0.033)}
    for name, (x, y, yaw) in VIEW_POSES.items():
        for mo, ro in (("still", 0.0), ("spin180", 0.0), ("spin180", 0.033), ("side1", 0.033), ("fwd2", 0.033)):
            img, _ = cams[ro].frame(sc.pose_fn(x, y, yaw, mo, 1.0 + ro / 2), 1.0)
            Image.fromarray(img).save(dd / f"{name}_{mo}_ro{1000 * ro:.0f}.jpg", quality=88)
        # the D435 colour (riser, 30 deg down) from the same pose, for comparison
        sc.set(x, y, yaw)
        mujoco.mj_forward(sc.m, sc.d)
        r = mujoco.Renderer(sc.m, 720, 1280)
        r.update_scene(sc.d, camera="d435_color")
        Image.fromarray(r.render()).save(dd / f"{name}_d435_color.jpg", quality=88)
        r.close()
        print("view", name, flush=True)
    for c in cams.values():
        c.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("tilt", "frames", "flow", "views"))
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    dict(tilt=tilt_sweep, frames=lambda: frames(a.quick), flow=flow, views=views)[a.stage]()


if __name__ == "__main__":
    main()
