"""People from the forward IMX662 (design.md 4.12; omni_perception people_vision / people_node, Nov 7 2026, study).

Does a COCO person detector (YOLOX nano / tiny, ONNX on the CPU) on the nose camera find people, how far, and does one
look reject the lidar's false leg candidates (chair and table legs)? Sim renders only: MuJoCo's primitive people
(house_scene.person) through imx662_cam's sensor model (noise, vignetting, MJPEG; a still robot, so no rolling-shutter
blur); not a real camera, real people, real light or real clutter.

  grid    one person on an open floor in front of a wall, a sofa and a plant: distance 1.2-7 m x bearing 0 / 25 / -38 deg
          x standing / seated (on a 0.45 m stool) x facing the robot / sideways / away, clothes varied. Per detector and
          image size (1920 x 1080; 960 x 540, the bridge's): found (a box whose bearing interval, +- 3 deg, holds the
          person's), the camera-only range error against its sigma, boxes not on the person, ms a frame
  house   the furnished house (props: furniture, signs, pictures) with three house_scene people added on free floor
          (two standing, one seated on a stool; house_props' own capsule figure in the office and the house's catch
          thrower figure by the front door are kept and counted as people too), the robot at free-floor poses, 4 headings: boxes on no person (false people), each person
          found when in view and not hidden (a ray from the camera to their chest or head reaches them first); the
          lidar's leg candidates (omni_console.people.detect on a 720-ray cast at the A2M12's plane, no map filter)
          and how many of the false ones one camera frame marks looked (people_vision.fuse: in view, no box). Frames
          with a false box are saved (results/people_detect/house_false/)

    python studies/people_detect.py grid  [--quick]
    python studies/people_detect.py house [--quick]
Models: python ros/tools/fetch_models.py yolox_nano yolox_tiny. Results: studies/results/people_detect/.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1] / "ros" / "src" / "omni_perception"))
sys.path.insert(0, str(ROOT.parents[1] / "ros" / "src" / "omni_console"))

import imx662_cam as IC  # noqa: E402
from omni_perception import people_vision as pv  # noqa: E402

OUT = HERE / "results" / "people_detect"
MODELS = ("yolox_nano", "yolox_tiny")
SIZES = ((1920, 1080), (960, 540))
LIDAR = (0.14, 0.0, 0.163)                  # robot_export.ARM: the A2M12 at the front, its scan plane
SHIRTS = ("h_shirt_red", "h_shirt_blue", "h_yellow_shirt")
PANTS = ("h_jeans", "h_khaki", "h_grey_pants")
SKINS = ("h_skin1", "h_skin2", "h_skin3")


def model_dir() -> Path:
    import os
    return Path(os.environ.get("OMNI_MODEL_DIR") or Path.home() / ".omni_robot" / "models")


def detectors(threads=4) -> dict:
    return {n: pv.PersonDetector(str(model_dir() / f"{n}.onnx"), threads=threads, min_score=0.35) for n in MODELS}


def camera_of(m, d, w, h, cam="imx662") -> pv.Camera:
    """people_vision's Camera for the MuJoCo camera, in the robot's base frame (the base body, or the world when no
    robot is in the model)"""
    pc, Rc = IC.camera_pose(m, d, cam)
    try:
        b = m.body("base").id
        Rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b].copy()
        pb[2] = 0.0                                              # base_link: the floor under the robot's centre
    except KeyError:
        Rb, pb = np.eye(3), np.zeros(3)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rb.T @ Rc, Rb.T @ (pc - pb)
    f = h / 2 / math.tan(math.radians(float(m.cam_fovy[m.camera(cam).id])) / 2)
    return pv.Camera((f, f, w / 2, h / 2), w, h, T)


def shrink(img, w, h):
    if img.shape[1] == w:
        return img
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


def hits(sights, bearing, margin=math.radians(3.0)):
    return [s for s in sights if s.az[1] - margin <= bearing <= s.az[0] + margin]


# -- grid: one person at a time --------------------------------------------------------------------------------------------

def grid_xml(person) -> str:
    import house_scene as H
    name, pos, yaw, pose, shirt, pants, skin = person
    bodies = [H.person(name, pos, yaw, pose, shirt=shirt, pants=pants, skin=skin, seat=0.45)]
    if pose == "sit":
        st = H._Body("stool", pos, yaw)
        st.box((0.15, 0.0, 0.225), (0.2, 0.2, 0.225), "h_cabinet")
        bodies.append(st)
    bodies += [H.sofa("sofa", (6.0, -3.0), 90), H.plant("plant", (7.0, 2.5))]
    m = IC.Mount()
    fwd, right, up = m.axes()
    return f"""<mujoco><compiler texturedir="{H.ASSET_DIR}"/>{H.VISUAL_XML}
  <default><default class="decor"><geom contype="0" conaffinity="0"/></default></default>
  <asset>{H.SKY_XML}{H.assets_xml()}</asset>
  <worldbody>{H.lights_xml()}
    <geom type="plane" size="30 30 0.1" material="h_floor"/>
    <geom class="decor" type="box" pos="9 0 1.3" size="0.05 10 1.3" material="h_wall"/>
    <camera name="imx662" pos="{' '.join(f'{v:.6g}' for v in m.pupil)}" xyaxes="{' '.join(f'{v:.6g}' for v in (*right, *up))}" fovy="58.4"/>
    {''.join(b.xml() for b in bodies)}
  </worldbody></mujoco>"""


def grid(quick=False):
    dets = detectors()
    dists = (1.5, 3.0, 5.0) if quick else (1.2, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    bearings = (0.0, 25.0) if quick else (0.0, 25.0, -38.0)
    poses = ("stand", "sit")
    facings = {"toward": 180.0, "side": 90.0, "away": 0.0}
    rows, k = [], 0
    cfg = IC.IMX662Config(readout_s=0.0, exposure_s=0.004, dt_s=0.004)
    for dist in dists:
        for bdeg in bearings:
            for pose in poses:
                for fname, fyaw in facings.items():
                    a = math.radians(bdeg)
                    x, y = IC.Mount().pupil[0] + dist * math.cos(a), dist * math.sin(a)
                    face = math.degrees(math.atan2(-y, -x)) + fyaw - 180.0      # "toward": facing the robot
                    person = ("p", (x, y), face, pose, SHIRTS[k % 3], PANTS[(k // 3) % 3], SKINS[(k // 9) % 3])
                    k += 1
                    m = mujoco.MjModel.from_xml_string(grid_xml(person))
                    d = mujoco.MjData(m)
                    mujoco.mj_forward(m, d)
                    rs = IC.RollingShutterCamera(m, d, cfg, seed=k)
                    img, _ = rs.frame(lambda t: None, 0.0)
                    rs.close()
                    truth_b = math.atan2(y, x)
                    r = dict(dist=dist, bearing=bdeg, pose=pose, facing=fname, x=round(x, 3), y=round(y, 3))
                    for (w, h) in SIZES:
                        cam = camera_of(m, d, w, h)
                        im = shrink(img, w, h)
                        for name, det in dets.items():
                            boxes = [b for b in det.detect(im) if pv.plausible(cam, b)]
                            sights = [pv.sight(cam, b) for b in boxes]
                            hit = hits(sights, truth_b)
                            key = f"{name}_{w}"
                            e = dict(found=bool(hit), extra=len(sights) - min(1, len(hit)), ms=round(det.ms, 1))
                            if hit:
                                s = max(hit, key=lambda s: s.box.score)
                                e.update(score=round(s.box.score, 3), how=s.how)
                                if s.xy is not None:
                                    e.update(range=round(math.hypot(*s.xy), 3), sigma=round(s.sigma, 3),
                                             range_err=round(math.hypot(*s.xy) - math.hypot(x, y), 3))
                            r[key] = e
                    rows.append(r)
                    print(json.dumps(r), flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"grid{'_quick' if quick else ''}.json").write_text(json.dumps(rows, indent=1))
    summary = summarize_grid(rows)
    (OUT / f"grid{'_quick' if quick else ''}_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))


def summarize_grid(rows) -> dict:
    out = {}
    keys = [k for k in rows[0] if k.startswith("yolox")]
    for key in keys:
        s = dict()
        for dist in sorted({r["dist"] for r in rows}):
            rr = [r for r in rows if r["dist"] == dist]
            f = [r[key]["found"] for r in rr]
            errs = [abs(r[key]["range_err"]) for r in rr if "range_err" in r[key]]
            within = [abs(r[key]["range_err"]) <= 2 * r[key]["sigma"] for r in rr if "range_err" in r[key]]
            s[f"{dist:g} m"] = dict(found=f"{sum(f)}/{len(f)}",
                                    found_stand=f"{sum(r[key]['found'] for r in rr if r['pose'] == 'stand')}/"
                                                f"{sum(r['pose'] == 'stand' for r in rr)}",
                                    found_sit=f"{sum(r[key]['found'] for r in rr if r['pose'] == 'sit')}/"
                                              f"{sum(r['pose'] == 'sit' for r in rr)}",
                                    range_err_median=round(float(np.median(errs)), 2) if errs else None,
                                    within_2_sigma=f"{sum(within)}/{len(within)}")
        s["extra_boxes"] = sum(r[key]["extra"] for r in rows)
        s["ms_median"] = round(float(np.median([r[key]["ms"] for r in rows])), 1)
        s["found_by_facing"] = {fc: f"{sum(r[key]['found'] for r in rows if r['facing'] == fc)}/"
                                    f"{sum(r['facing'] == fc for r in rows)}" for fc in ("toward", "side", "away")}
        out[key] = s
    return out


# -- house: the furnished house, the lidar's false candidates ------------------------------------------------------------

def lidar_cast(m, d, x, y, yaw, n=720, max_r=8.0):
    """A 360 deg cast at the A2M12's plane from the robot at (x, y, yaw) (the robot itself parked away first): ranges,
    angle_min, increment, as a LaserScan in the laser frame (= the base's axes)"""
    c, s = math.cos(yaw), math.sin(yaw)
    o = np.array([x + c * LIDAR[0] - s * LIDAR[1], y + s * LIDAR[0] + c * LIDAR[1], LIDAR[2]])
    gid = np.zeros(1, np.int32)
    inc = 2 * math.pi / n
    r = np.full(n, np.inf)
    for i in range(n):
        a = yaw - math.pi + i * inc
        v = np.array([math.cos(a), math.sin(a), 0.0])
        h = mujoco.mj_ray(m, d, o, v, None, 1, -1, gid)
        if 0 < h < max_r:
            r[i] = h
    return r, -math.pi, inc


PEOPLE_MATS = ("h_yellow_shirt", "h_skin1", "h_skin2", "h_skin3", "h_hair_dark", "h_hair_brown", "h_jeans", "h_khaki",
               "h_grey_pants", "h_shirt_blue", "h_shirt_red", "h_shoe")


def house_scene_xml(spots):
    """the furnished house (imx662_study's cache) with the IMX662 and house_scene people at spots [(x, y, yaw, pose)]"""
    import house_scene as H
    sys.path.insert(0, str(HERE))
    import imx662_study as S
    xml, p = S.house_xml()
    xml = IC.with_camera(xml, S.mount())
    mats = ['<material name="study_stool" rgba="0.45 0.32 0.22 1" specular="0.1"/>']
    for k in PEOPLE_MATS:
        rgba, spec, shin, em = H.COLOURS[k]
        mats.append(f'<material name="{k}" rgba="{" ".join(f"{v:.4g}" for v in rgba)}" specular="{spec}" '
                    f'shininess="{shin}" emission="{em}"/>')
    bodies = []
    for k, (x, y, yaw, pose) in enumerate(spots):
        bodies.append(H.person(f"study_person{k}", (x, y), yaw, pose, shirt=SHIRTS[k % 3], pants=PANTS[k % 3],
                               skin=SKINS[k % 3], seat=0.45))
        if pose == "sit":
            st = H._Body(f"study_stool{k}", (x, y), yaw)
            st.box((0.15, 0.0, 0.225), (0.2, 0.2, 0.225), "study_stool")
            bodies.append(st)
    if 'class="decor"' not in xml.split("<worldbody>")[0]:
        raise RuntimeError("the house model has no decor class")
    xml = xml.replace("</asset>", "".join(mats) + "</asset>", 1)
    k = xml.rfind("</worldbody>")
    return xml[:k] + "".join(b.xml() for b in bodies) + xml[k:], p


def visible(m, d, pc, target, body_prefix) -> bool:
    """a ray from the camera at pc to target meets a geom of a body named body_prefix* first"""
    v = np.asarray(target, float) - pc
    dist = float(np.linalg.norm(v))
    gid = np.zeros(1, np.int32)
    h = mujoco.mj_ray(m, d, pc, v / dist, None, 1, -1, gid)
    if h < 0 or gid[0] < 0:
        return False
    name = m.body(int(m.geom_bodyid[gid[0]])).name or ""
    return name.startswith(body_prefix) and h <= dist + 0.3


def house(quick=False):
    from PIL import Image
    sys.path.insert(0, str(HERE))
    import imx662_study as S
    import d435_tilt as dt
    from omni_console.people import detect
    dets = detectors()
    # three people on free floor, spread over the house (chosen from the free-floor grid, seeded)
    sc0 = S.Scene()
    grid0 = dt.free_poses(sc0.m, sc0.d, 1.0, headings=1)
    rng = np.random.default_rng(7)
    spots, cands = [], [g for g in grid0 if sc0.clear(g[0], g[1], 0.5)]
    for g in rng.permutation(len(cands)):
        x, y, _ = cands[g]
        if all(math.hypot(x - a, y - b) > 3.0 for a, b, _, _ in spots) and math.hypot(x - 1.1, y - 3.0) > 2.0:
            spots.append((float(x), float(y), float(rng.uniform(-180, 180)), "sit" if len(spots) == 2 else "stand"))
        if len(spots) == 3:
            break
    xml, _ = house_scene_xml(spots)
    m = mujoco.MjModel.from_xml_string(xml)
    IC.real_tags(m)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(300):
        mujoco.mj_step(m, d)
    sc0.m, sc0.d = m, d                                                # reuse its pose setter on this model
    sc0.adr = m.joint("base").qposadr[0]
    sc0.q0 = d.qpos.copy()
    sc0.z0 = float(sc0.q0[sc0.adr + 2])
    sc = sc0
    people = [(np.array([x, y]), f"study_person{k}", pose) for k, (x, y, _, pose) in enumerate(spots)]
    people.append((np.array([1.1, 3.0]), "", "props"))                # house_props' figure (geoms in the house body)
    th = d.xpos[m.body("thrower").id]                                   # the house's catch thrower, a figure too
    people.append((np.array([th[0], th[1]]), "thrower", "thrower"))
    poses = dt.free_poses(m, d, 2.0 if quick else 1.0, headings=4)
    if len(poses) > (40 if quick else 200):
        poses = [poses[i] for i in sorted(rng.choice(len(poses), 40 if quick else 200, replace=False))]
    cfg = IC.IMX662Config(readout_s=0.0, exposure_s=0.004, dt_s=0.004)
    rs = IC.RollingShutterCamera(m, d, cfg, seed=3)
    (OUT / "house_false").mkdir(parents=True, exist_ok=True)
    rows = []
    for i, (x, y, yaw) in enumerate(poses):
        if not sc.clear(x, y):
            continue
        sc.set(x, y, yaw)
        img, _ = rs.frame(lambda t: sc.set(x, y, yaw), 0.0)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_camlight(m, d)                                        # (the cameras' poses: not mj_kinematics')
        cam_full = camera_of(m, d, cfg.width, cfg.height)
        pc, _ = IC.camera_pose(m, d)
        c, s = math.cos(yaw), math.sin(yaw)
        seen = []                                                       # (base xy, pose) of each person in view
        for pw, body, pose in people:
            dx, dy = pw[0] - x, pw[1] - y
            pb = (c * dx + s * dy, -s * dx + c * dy)
            if not (cam_full.sees(*pb, z0=0.3, z1=1.0 if pose == "sit" else 1.2) and 0.8 < math.hypot(*pb) < 8.0):
                continue
            if body and not any(visible(m, d, pc, (pw[0], pw[1], z), body) for z in ((0.8, 1.1) if pose == "sit"
                                                                                       else (1.2, 1.55))):
                continue
            if not body and not any(visible(m, d, pc, (pw[0], pw[1], z), "house") for z in (1.2, 1.55)):
                continue
            seen.append((pb, pose))
        sc.set(50.0, 50.0, 0.0)                                         # the robot away: the lidar sees the house only
        mujoco.mj_kinematics(m, d)
        ranges, a0, inc = lidar_cast(m, d, x, y, yaw)
        sc.set(x, y, yaw)
        legs = detect(ranges, a0, inc, (LIDAR[0], LIDAR[1], 0.0))
        all_pb = []
        for pw, _, _ in people:
            dx, dy = pw[0] - x, pw[1] - y
            all_pb.append((c * dx + s * dy, -s * dx + c * dy))
        is_person = lambda ox, oy: any(math.hypot(ox - a, oy - b) < 0.45 for a, b in all_pb)   # noqa: E731
        r = dict(x=round(x, 2), y=round(y, 2), yaw_deg=round(math.degrees(yaw)), people_in_view=len(seen),
                 seen=[dict(range=round(math.hypot(*pb), 2), pose=pose) for pb, pose in seen],
                 lidar_candidates=len(legs), lidar_people=sum(is_person(*lg) for lg in legs))
        for (w, h) in SIZES:
            cam = camera_of_pose(cam_full, w, h)
            im = shrink(img, w, h)
            for name, det in dets.items():
                boxes = [b for b in det.detect(im) if pv.plausible(cam, b)]
                sights = [pv.sight(cam, b) for b in boxes]
                found = [bool(hits(sights, math.atan2(pb[1], pb[0]))) for pb, _ in seen]
                false = [sg for sg in sights
                         if not any(sg.az[1] - 0.05 <= math.atan2(b, a) <= sg.az[0] + 0.05 for a, b in all_pb)]
                obs = pv.fuse(sights, legs, cam)
                false_c = [o for o in obs if o.lidar and not o.camera and not is_person(o.x, o.y)]
                r[f"{name}_{w}"] = dict(boxes=len(sights), false_boxes=len(false), found=found, ms=round(det.ms, 1),
                                        false_candidates=len(false_c), false_looked=sum(o.looked for o in false_c),
                                        confirmed=sum(o.lidar and o.camera and is_person(o.x, o.y) for o in obs))
                if false and name == "yolox_tiny" and w == SIZES[0][0]:
                    Image.fromarray(shrink(img, 960, 540)).save(OUT / "house_false" / f"{i:03d}_{x:.1f}_{y:.1f}_"
                                                                f"{round(math.degrees(yaw))}.jpg", quality=80)
        rows.append(r)
        print(i, json.dumps(r), flush=True)
    rs.close()
    OUT.mkdir(parents=True, exist_ok=True)
    tag = "_quick" if quick else ""
    (OUT / f"house{tag}.json").write_text(json.dumps(dict(spots=spots, rows=rows), indent=1))
    summary = summarize_house(rows)
    th = d.xpos[m.body("thrower").id]
    summary["people"] = [dict(x=round(a, 2), y=round(b, 2), pose=p) for a, b, _, p in spots] + \
        [dict(x=1.1, y=3.0, pose="house_props capsule figure, standing"),
         dict(x=round(float(th[0]), 2), y=round(float(th[1]), 2), pose="the house's catch thrower, standing")]
    (OUT / f"house{tag}_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))


def camera_of_pose(cam: pv.Camera, w, h) -> pv.Camera:
    k = w / cam.width
    fx, fy, cx, cy = cam.K
    return pv.Camera((fx * k, fy * k, cx * k, cy * k), w, h, cam.T)


def summarize_house(rows) -> dict:
    out = dict(frames=len(rows), people_in_view=sum(r["people_in_view"] for r in rows),
               lidar_candidates_per_frame=round(float(np.mean([r["lidar_candidates"] for r in rows])), 2),
               lidar_people=sum(r["lidar_people"] for r in rows))
    for key in [k for k in rows[0] if k.startswith("yolox")]:
        found = [f for r in rows for f in r[key]["found"]]
        by_range = {}
        for r in rows:
            for sn, f in zip(r["seen"], r[key]["found"]):
                b = "<2 m" if sn["range"] < 2 else ("2-4 m" if sn["range"] < 4 else ">=4 m")
                by_range.setdefault(b, []).append(f)
        fc = sum(r[key]["false_candidates"] for r in rows)
        out[key] = dict(false_boxes=sum(r[key]["false_boxes"] for r in rows),
                        frames_with_a_false_box=sum(r[key]["false_boxes"] > 0 for r in rows),
                        people_found=f"{sum(found)}/{len(found)}",
                        found_by_range={k: f"{sum(v)}/{len(v)}" for k, v in sorted(by_range.items())},
                        lidar_people_confirmed=sum(r[key]["confirmed"] for r in rows),
                        false_lidar_candidates=fc, looked_at_once=sum(r[key]["false_looked"] for r in rows),
                        ms_median=round(float(np.median([r[key]["ms"] for r in rows])), 1))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("grid", "house"))
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    t = time.time()
    (grid if a.what == "grid" else house)(a.quick)
    print(f"{time.time() - t:.0f} s")


if __name__ == "__main__":
    main()
