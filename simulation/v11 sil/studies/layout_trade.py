"""Where the lidar, the stereo pair, the display and the arm go so that one base serves both payloads (design.md 12,
Oct 24, proposed). Fernando: the catch robot and the arm robot each have a layout that suits neither well; the swap is
rare (hours are fine), the base should stay the same; the stereo pair should sit on one stiff bar; a display at the
front; the arm with as much floor reach as it can get.

Each layout is the v11 model (the CAD's 96 mm plate gap) with the sim's own lidar, cameras, standoffs and display
replaced by the layout's (MjSpec), so every check runs on the sim's geometry of the catch module (at every tilt) and of
the SO-101 (at its poses):

  lidar     rays in the A2M12's scan plane (150 mm) from the layout's lidar: degrees blocked by the robot, the widest
            blocked wedge, and the travel directions where a lane of the robot's path 0.4-1.5 m ahead stays hidden
            (a thin shadow hides a thin thing only while the robot drives along it), per payload pose
  cameras   rays from both cameras: catch - the ball's last 1.5 s of a 6.5 m throw (apex 1.6 and 2.4 m) seen by both
            cameras with the module at the catch tilt (35 deg); arm - the floor ahead seen by both (nearest point
            straight ahead, near zone 0.25-0.6 m x |y| < 0.2, far zone 0.6-1.6 m x |y| < 0.6) with the arm stowed;
            the share of each image the robot itself fills
  display   rays from viewers' eyes to points on the panel: an adult at 1.5 m, a kid at 1.2 m (straight and 40 deg
            off), the thrower at 6.5 m: share of the panel seen, angle off the panel's axis
  clear     catch: the module (every geom, -30..+40 deg) against the head; the launch line (ball + 5 mm) against the
            head at every game elevation (>= 30 deg) and the bench's; arm: the head against the arm's sweep below
  arm       arm_mount.py's sweep with the layout's obstacles: top-down floor grasps (nearest, farthest from the
            centre), a drop into the bin, the lowest stow inside the bumper, the reach with the pan turned

C2 and A6 are the robots as cad/v7 draws them (head.py, display.py, arm_kit.py: Oct 24-25): the study loads the CAD's
own meshes of the head (cheeks, camera bar, holders), the face and, on the arm robot, the arm kit (riser, adapter, bin,
uprights, shelf, audio box) from cad/v7/out/head_stl (written by `build.py` or `build.py --head-only`) and casts its
rays at them; their module and launch-line clearances are the CAD's exact ones (out/checks.json "head"). On A6 every
pose of the arm is checked with the SO-101's own meshes against those parts (mesh_sdf.py, 3 mm): the capsules along
the links that the other layouts use miss the arm's base and shoulder bracket, which on the plate at x 0.10 (A5)
stand where the camera bar is.

  python layout_trade.py            -> results/layout_trade.json, printed table (~10 s)
  python layout_trade.py png        -> renders/layout_trade.png, renders/layout_lidar.png
  python layout_trade.py L          -> the lidar's value: the v9 house tour with each layout's blocked sectors (and
                                       bigger ones), results/layout_lidar.json
  python layout_trade.py report
  python layout_trade.py arm-check  -> the SO-101's meshes against the drawn arm robot at its fixed poses and grasps
                                       (mm, JSON on stdout; cad/v7/build.py runs it)
  python layout_trade.py arm-stl <dir> [pose]  -> the SO-101 at a pose (stow) as an STL in mm, for cad/v7's renders
  python layout_trade.py arm-poses [pose]       -> where each of its parts is (JSON on stdout; cad/v8 places the
                                                  SO-101's own STEP parts with it in the arm robot's STEP)
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import arm_mount as AM  # noqa: E402
import arm_so101 as A  # noqa: E402
from common import RESULTS, save  # noqa: E402
from robot_export import v11_params  # noqa: E402
from robot_model import build_xml, geometry, quat_axis  # noqa: E402

RENDERS = HERE.parent / "renders"
CAD_OUT = Path(os.environ.get("OMNI_CAD_OUT", HERE.parents[2] / "cad" / "v7" / "out"))   # cad/v8's build.py sets it
DEG = math.pi / 180

# --- the base (cad/v7/dims.py) --------------------------------------------------------------------------------------
PLATE_GAP = 0.096
DECK_TOP, PLATE_BOT, PLATE_TOP = 0.0665, 0.1625, 0.1685
LIDAR_Z = 0.150                        # A2M12 scan plane
BUMPER_R = 0.217
STANDOFFS = ((0.097, 0.0345), (0.097, -0.0345), (0.0, 0.112), (0.0, -0.112), (-0.104, 0.034), (-0.104, -0.034))
STANDOFF_R = 0.006
CAM_HALF = (0.012, 0.020, 0.014)       # the sim's camera body (half sizes, x along the view)
CAM_LENS = 0.017                       # optical centre ahead of the body's centre
HFOV = 2 * math.atan(math.tan(35 * DEG) * 1280 / 800)       # 70 deg vertical on 1280 x 800: 96.5 deg across
VFOV = 70 * DEG
# displays: (module w, h, housing depth) and the active area (w, h), m (VERIFY: Waveshare 5 in HDMI 800 x 480, the
# CAD's 7 in 1024 x 600)
PANELS = {"5in": ((0.129, 0.085, 0.016), (0.108, 0.065)), "7in": ((0.173, 0.108, 0.025), (0.154, 0.086)),
          "5in_cad": ((0.121, 0.077, 0.014), (0.108, 0.0648))}   # the CAD's panel + window, behind its housing's lip
GROUPS_LIDAR = np.array([1, 1, 0, 1, 1, 1], np.uint8)        # not the funnel's net (as sensors.py)
GROUPS_CAM = np.array([1, 1, 1, 1, 0, 1], np.uint8)          # not the camera bodies
GROUPS_EYE = np.array([1, 1, 1, 1, 1, 0], np.uint8)          # not the panel itself
G_CAM, G_PANEL = 4, 5


@dataclass
class Layout:
    key: str
    label: str
    payload: str                              # "catch" | "arm"
    lidar: tuple = (0.14, 0.0)                # scan centre (x, y)
    cams: tuple = (0.184, 0.095, 0.201, 30.0)  # (x, half baseline, z, pitch deg; + up)
    display: dict | None = None               # x, z (centre), y, panel, facing ("front" | "rear"), lean (deg back)
    boxes: tuple = ()                         # extra structure: (name, (x0, x1), (y0, y1), (z0, z1))
    arm: dict = field(default_factory=dict)   # mount (x, z or None = plate), bin (x, y, L, W, H)
    notes: str = ""
    standoff_r: tuple = (STANDOFF_R,) * 6     # per STANDOFFS
    meshes: str | None = None                 # the CAD's head at an index ("catch" | "arm"): cad/v7/out/head_stl/<index>

    def params(self):
        kw = dict(plate_gap=PLATE_GAP, thrower_xy=(30.0, 30.0), area_size=(80.0, 80.0), home_xy=(0.0, 0.0),
                  camera_mount="plate_front")
        if self.payload == "arm":
            a = self.arm
            kw.update(payload="arm", arm_mount=(a["mount"][0], 0.0, 0.0), arm_mount_z=a["mount"][1],
                      arm_riser=a.get("riser", 0.0), arm_bin=a.get("bin"), arm_pick=True)
        return v11_params(**kw)


# --- the layouts -----------------------------------------------------------------------------------------------------
# the head: one stiff bar (20 x 20 aluminium tube) across the plate's front edge, both cameras on it, the display in
# the middle; the bar turns about its own axis between two indexed stops (catch: 30 deg up, arm: 40 deg down)
HEAD_BAR = ("head_bar", (0.150, 0.170), (-0.118, 0.118), (0.172, 0.192))
HEAD_CAM = (0.184, 0.095, 0.201)
# the display hangs from the bar in front of the plate's edge between the cameras, leaning back 10 deg: its front face
# behind the lenses (x 0.194), its bottom 2.5 mm over the lidar's top (0.1605) and ahead of it (the lidar's head ends at
# x 0.175), so the scan passes under it; the 5 in fits between cameras 190 mm apart, the 7 in needs 230 mm
DISPLAY_BOT = 0.163
HEAD_5 = dict(x=0.194, y=0.0, z=DISPLAY_BOT + PANELS["5in"][0][1] / 2, panel="5in", facing="front", lean=10.0)
HEAD_7 = dict(x=0.194, y=0.0, z=DISPLAY_BOT + PANELS["7in"][0][1] / 2, panel="7in", facing="front", lean=10.0)
# the CAD's rear display (cad/v7/display.py: housing x -157..-122, z 180-299, facing the rear)
REAR_7 = dict(x=-0.150, y=0.0, z=0.2395, panel="7in", facing="rear", lean=0.0)
BIN = (-0.02, 0.0, 0.15, 0.14, 0.07)      # today's (robot_export.ARM)
BIN_SIZE = (0.12, 0.12, 0.07)             # the plate-mounted arms' bins (searched in arm_job)
# the arm robot's display when the arm sits behind the bar: just ahead of the rear bin's front wall (x -0.01), facing
# forward, leaning back 15 deg
BIN_FRONT_5 = dict(x=0.012, y=0.0, z=PLATE_TOP + 0.004 + PANELS["5in"][0][1] / 2, panel="5in", facing="front",
                   lean=15.0)
# or on a post at the plate's rear edge, behind the bin and above the arm's drops into it
POST_5 = dict(x=-0.132, y=0.0, z=0.40, panel="5in", facing="front", lean=10.0)
# the mast for the arm robot's high head: at the plate's rear edge, behind the bin
MAST_X = (-0.125, -0.105)
MAST_HEAD_Z = 0.43


# the head as drawn (cad/v7 head.py, display.py): the lenses' optical centres (x, z, pitch) at each index; the camera
# bodies here sit CAM_LENS behind them. The face: the panel and its window as one block (the housing is a mesh).
CAD_LENS = {"catch": (0.19196, 0.21098, 30.0), "arm": (0.18266, 0.16573, -40.0)}
CAD_FACE = dict(x=0.18824, y=0.0, z=0.20325, panel="5in_cad", facing="front", lean=10.0)
CAD_FACE_ARM = dict(x=-0.13386, y=0.0, z=0.39844, panel="5in_cad", facing="front", lean=10.0)    # over the bin
CAD_ARM = dict(mount=(0.14, None), riser=0.05, bin=(-0.07, 0.0, 0.12, 0.12, 0.07))             # cad/v7 arm_kit.py
CAD_STANDOFF_R = (0.004, 0.004) + (STANDOFF_R,) * 4                  # the front pair: Ø8 rods
MESH_KG = {"cheek": 0.034, "cam_holder": 0.036, "camera_bar": 0.068, "face_housing": 0.026, "face_back": 0.015,
           "arm_riser": 0.06, "arm_adapter": 0.10, "bin": 0.09, "face_upright": 0.05, "audio_shelf": 0.03,
           "audio_base": 0.05, "audio_lid": 0.015, "12_V_buck": 0.015,
           "crown": 0.11, "mast_foot": 0.018, "mast_post": 0.10}       # kg (est, from the CAD's prints; the last three:
                                                                        # cad/v8 round 2's face mast)


def _cad_cams(index):
    x, z, pitch = CAD_LENS[index]
    return (x - CAM_LENS * math.cos(pitch * DEG), 0.095, z - CAM_LENS * math.sin(pitch * DEG), pitch)


def cad_head_checks():
    """The CAD's exact clearances of the head (build.py: checks.json "head", or head_checks.json from --head-only)."""
    for f in ("checks.json", "head_checks.json"):
        path = CAD_OUT / f
        if path.exists():
            d = json.loads(path.read_text())
            d = d.get("head", d) if f == "checks.json" else d
            if "module" in d:
                return d
    return None


def _mast(z_top):
    return ("mast", MAST_X, (-0.012, 0.012), (PLATE_TOP, z_top))


LAYOUTS = [
    Layout("C0", "catch today: bar 30 deg up, 7 in at the rear facing back", "catch", display=REAR_7,
           boxes=(("cad_bar", (0.162, 0.168), (-0.122, 0.122), (0.181, 0.206)),),
           notes="cad/v7: stereo bar on two arms, display housing at the rear edge"),
    Layout("C1", "catch, front head: bar + 5 in between the cameras", "catch", display=HEAD_5, boxes=(HEAD_BAR,)),
    Layout("C2", "catch as drawn (cad/v7): bar 30 deg up, 5 in face, cheeks, Ø8 front rods", "catch",
           cams=_cad_cams("catch"), display=CAD_FACE, standoff_r=CAD_STANDOFF_R, meshes="catch",
           notes="cad/v7 head.py, display.py: the CAD's meshes; module and launch clearances from its exact check"),
    Layout("C1w", "catch, front head, 230 mm bar + 7 in", "catch", cams=(0.184, 0.115, 0.201, 30.0), display=HEAD_7,
           boxes=(("head_bar", (0.150, 0.170), (-0.138, 0.138), (0.172, 0.192)),)),
    Layout("A0", "arm today: nose arm, pods 35 deg down, lidar rear, 7 in on the bin facing back", "arm",
           lidar=(-0.14, 0.0), cams=(0.17, 0.095, 0.125, -35.0), display=None,
           boxes=(("pod_post_L", (0.148, 0.172), (0.083, 0.107), (DECK_TOP, 0.111)),
                  ("pod_post_R", (0.148, 0.172), (-0.107, -0.083), (DECK_TOP, 0.111)),
                  ("nose_bracket", (0.075, 0.155), (-0.035, 0.035), (DECK_TOP, 0.105))),
           arm=dict(mount=(0.12, 0.105), bin=BIN, rear_display=True),
           notes="robot_export.ARM; the lidar moved to the rear because the bracket fills the front of its layer"),
    Layout("A1", "arm on the plate behind the front head (C1's head, bar 40 deg down)", "arm",
           cams=(*HEAD_CAM, -40.0), display=HEAD_5, boxes=(HEAD_BAR,),
           arm=dict(mount=(0.05, None), bin=(-0.07, 0.0, *BIN_SIZE))),
    Layout("A1r", "as A1, the arm on a 60 mm riser", "arm",
           cams=(*HEAD_CAM, -40.0), display=HEAD_5, boxes=(HEAD_BAR,),
           arm=dict(mount=(0.05, PLATE_TOP + 0.06), bin=(-0.07, 0.0, *BIN_SIZE))),
    Layout("A3", "arm at the plate's front edge right behind C1's head, display still in the head", "arm",
           cams=(*HEAD_CAM, -40.0), display=HEAD_5, boxes=(HEAD_BAR,),
           arm=dict(mount=(0.10, None), bin=(-0.07, 0.0, *BIN_SIZE))),
    Layout("A4", "arm at the plate's front edge behind the bar; display moved to the bin's front, bin at the rear",
           "arm", cams=(*HEAD_CAM, -40.0), display=BIN_FRONT_5, boxes=(HEAD_BAR,),
           arm=dict(mount=(0.10, None), bin=(-0.07, 0.0, *BIN_SIZE))),
    Layout("A5", "as A4, the display on a post at the plate's rear, over the bin", "arm",
           cams=(*HEAD_CAM, -40.0), display=POST_5, boxes=(HEAD_BAR, ("display_post", (-0.172, -0.152), (-0.012, 0.012),
                                                                       (PLATE_TOP, POST_5["z"]))),
           arm=dict(mount=(0.10, None), bin=(-0.07, 0.0, *BIN_SIZE))),
    Layout("A6", "arm robot as drawn (cad/v7 arm_kit.py): SO-101 on a 50 mm riser at x 0.14, the bar 40 deg down, "
                 "bin and face at the rear", "arm",
           cams=_cad_cams("arm"), display=CAD_FACE_ARM, arm=CAD_ARM, standoff_r=CAD_STANDOFF_R, meshes="arm",
           notes="the CAD's meshes; the arm's own meshes checked against them at every pose"),
    Layout("A2", "arm on the plate's front edge, head on a rear mast 43 cm up", "arm",
           cams=(-0.095, 0.095, MAST_HEAD_Z, -45.0),
           display=dict(x=-0.090, y=0.0, z=MAST_HEAD_Z - 0.075, panel="5in", facing="front", lean=15.0),
           boxes=(_mast(MAST_HEAD_Z + 0.02), ("head_bar", (-0.115, -0.095), (-0.118, 0.118),
                                               (MAST_HEAD_Z - 0.01, MAST_HEAD_Z + 0.01))),
           arm=dict(mount=(0.12, None), bin=(-0.04, 0.0, *BIN_SIZE))),
]
BY_KEY = {lay.key: lay for lay in LAYOUTS}


# --- the model --------------------------------------------------------------------------------------------------------

def _rot_y(deg):
    """Quaternion turning +x up by deg (a camera's pitch)."""
    return quat_axis((0, 1, 0), -deg * DEG)


def build(lay: Layout):
    p = lay.params()
    spec = mujoco.MjSpec.from_string(build_xml(p))
    base = spec.body("base")
    drop = {"camera_body", "lens", "lidar", "lidar_top"}
    for g in list(base.geoms):
        if g.material in drop or (g.material == "aluminum" and g.size[0] < 0.005):     # the sim's thin standoffs
            spec.delete(g)
    for c in list(spec.cameras):
        if c.name.startswith("stereo_"):
            spec.delete(c)
    if lay.payload == "arm" and not lay.arm.get("rear_display"):
        g = spec.geom("arm_display")
        if g is not None:
            spec.delete(g)
    if lay.payload == "arm" and A.meshes_available():
        # sightlines see the arm's meshes; its capsules are fatter (the base's pokes 3 cm under its own mount)
        for g in list(spec.geoms):
            if g.name.startswith("arm_") and ("_link" in g.name or g.name.startswith("arm_tip")):
                spec.delete(g)
    box, cyl = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
    kw = dict(contype=0, conaffinity=0, density=0)
    for k, (x, y) in enumerate(STANDOFFS):
        base.add_geom(name=f"standoff{k}", type=cyl, size=[lay.standoff_r[k], 0, 0],
                      fromto=[x, y, DECK_TOP, x, y, PLATE_BOT], group=1, rgba=[0.7, 0.7, 0.7, 1], **kw)
    for f in mesh_files(lay):
        spec.add_mesh(name=f"cad_{f.stem}", file=str(f), scale=[0.001] * 3)
        base.add_geom(name=f"cad_{f.stem}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"cad_{f.stem}", group=1,
                      rgba=[0.95, 0.5, 0.15, 1] if f.stem.startswith("cheek") else [0.75, 0.78, 0.82, 1], **kw)
    for name, (x0, x1), (y0, y1), (z0, z1) in lay.boxes:
        base.add_geom(name=name, type=box, size=[(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2],
                      pos=[(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], group=1, rgba=[0.75, 0.78, 0.82, 1], **kw)
    cx, hb, cz, pitch = lay.cams
    for s, nm in ((1, "L"), (-1, "R")):
        base.add_geom(name=f"cam_{nm}", type=box, size=list(CAM_HALF), pos=[cx, s * hb, cz], quat=list(_rot_y(pitch)),
                      group=G_CAM, rgba=[0.1, 0.1, 0.12, 1], **kw)
    base.add_geom(name="lidar_head", type=cyl, size=[0.035, 0.006, 0], pos=[*lay.lidar, LIDAR_Z],
                  group=2, rgba=[0.15, 0.15, 0.15, 1], **kw)
    if lay.display:
        dsp = lay.display
        (w, h, t), _ = PANELS[dsp["panel"]]
        face = 1.0 if dsp["facing"] == "front" else -1.0
        q = quat_axis((0, 1, 0), -face * dsp["lean"] * DEG)          # top leans away from the viewer
        base.add_geom(name="display", type=box, size=[t / 2, w / 2, h / 2], pos=[dsp["x"] - face * t / 2, dsp["y"], dsp["z"]],
                      quat=list(q), group=G_PANEL, rgba=[0.2, 0.5, 0.9, 1], **kw)        # lit
    m = spec.compile()
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    return p, m, d


def mesh_files(lay: Layout):
    """The CAD's head meshes for a layout (none for the study's own layouts). The panel and its window are left out:
    the layout's display block stands for them."""
    if not lay.meshes:
        return []
    d = CAD_OUT / "head_stl" / lay.meshes
    files = sorted(f for f in d.glob("*.stl") if not f.stem.startswith(("5_in_display", "polycarbonate")))
    if not files:
        raise FileNotFoundError(f"{d}: run cad/v7/build.py --head-only first")
    return files


def stl_bounds(f):
    """(min xyz, max xyz) m of a binary STL in mm."""
    raw = Path(f).read_bytes()
    n = int(np.frombuffer(raw, np.uint32, 1, 80)[0])
    tri = np.frombuffer(raw, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
    v = tri["v"].reshape(-1, 3) / 1000.0
    return v.min(axis=0), v.max(axis=0)


class Scene:
    """A layout's model at a pose (module tilt or arm joints)."""

    def __init__(self, lay: Layout):
        self.lay = lay
        self.p, self.m, self.d = build(lay)
        m = self.m
        self.base = m.body("base").id
        self.self_geom = m.body_rootid[m.geom_bodyid] == self.base
        self.floor = m.geom("floor").id
        self.g = geometry(self.p)
        if lay.payload == "arm":
            self.qadr = np.array([m.joint(A.name(j)).qposadr[0] for j in A.JOINTS])

    def pose(self, tilt_deg=None, q=None):
        m, d = self.m, self.d
        if tilt_deg is not None:
            d.qpos[m.joint("tilt").qposadr[0]] = tilt_deg * DEG
        if q is not None:
            d.qpos[self.qadr] = q
        mujoco.mj_forward(m, d)

    def rays(self, pnt, vecs, groups, cutoff=20.0):
        """First hit along each unit vector: (geom id or -1, distance)."""
        n = len(vecs)
        gid = np.zeros(n, np.int32)
        dist = np.zeros(n)
        mujoco.mj_multiRay(self.m, self.d, np.asarray(pnt, float), np.ascontiguousarray(vecs, float).reshape(-1), groups,
                           1, -1, gid, dist, None, n, cutoff)
        return gid, dist

    def cameras(self):
        """[(optical centre, rotation camera -> robot: columns forward, left, up)] for L and R."""
        cx, hb, cz, pitch = self.lay.cams
        c, s = math.cos(pitch * DEG), math.sin(pitch * DEG)
        R = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        return [(np.array([cx, sgn * hb, cz]) + CAM_LENS * R[:, 0], R) for sgn in (1, -1)]


# --- checks ------------------------------------------------------------------------------------------------------------

def lidar_check(sc: Scene, n=1440, lane=(0.4, 1.5)):
    """Blocked degrees, the widest blocked wedge (deg), blocked sectors (deg, robot frame), and the travel directions
    (1 deg steps) with a hidden lane: a lateral offset within the bumper's width whose points 0.4-1.5 m ahead are all
    in the robot's shadow."""
    ang = np.arange(n) * 2 * math.pi / n
    vec = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(n)])
    o = np.array([*sc.lay.lidar, LIDAR_Z])
    gid, dist = sc.rays(o, vec, GROUPS_LIDAR, cutoff=3.0)
    blocked = (gid >= 0) & sc.self_geom[np.maximum(gid, 0)]
    step = 360.0 / n
    sectors, k = [], 0
    run, best = 0, 0
    b2 = np.concatenate([blocked, blocked])
    for i in range(2 * n):
        run = run + 1 if b2[i] else 0
        best = max(best, min(run, n))
    i = 0
    while i < n:
        if blocked[i] and not blocked[i - 1]:
            j = i
            while blocked[j % n]:
                j += 1
            sectors.append((round(i * step, 1), round((j - i) * step, 1)))
            i = j
        else:
            i += 1
    if blocked.all():
        sectors = [(0.0, 360.0)]
    # hidden lanes: the lidar's shadow is a set of wedges from its centre; a lane at lateral offset e along heading h
    # is hidden if every point at 0.4-1.5 m along it falls in a blocked wedge
    hidden = []
    offs = np.linspace(-BUMPER_R, BUMPER_R, 23)
    dd = np.linspace(BUMPER_R + lane[0], BUMPER_R + lane[1], 12)
    for hdeg in range(0, 360, 2):
        h = hdeg * DEG
        ux, uy = math.cos(h), math.sin(h)
        px = dd[None, :] * ux - offs[:, None] * uy - o[0]
        py = dd[None, :] * uy + offs[:, None] * ux - o[1]
        a = np.mod(np.arctan2(py, px), 2 * math.pi)
        idx = np.round(a / (2 * math.pi) * n).astype(int) % n
        if np.any(blocked[idx].all(axis=1)):
            hidden.append(hdeg)
    return dict(blocked_deg=round(float(blocked.sum() * step), 1), widest_deg=round(best * step, 1),
                sectors=sectors, hidden_lane_headings=hidden)


def _cam_dirs(R, nu=48, nv=30):
    """Unit rays over the image (pinhole, 1280 x 800, 70 deg vertical): forward, left, up in the robot frame."""
    tu, tv = math.tan(HFOV / 2), math.tan(VFOV / 2)
    u, v = np.meshgrid(np.linspace(-tu, tu, nu), np.linspace(-tv, tv, nv))
    loc = np.column_stack([np.ones(u.size), -u.ravel(), v.ravel()])
    loc /= np.linalg.norm(loc, axis=1, keepdims=True)
    return loc @ R.T


def _in_fov(R, vec):
    loc = vec @ R                          # forward, left, up
    f = loc[:, 0]
    return (f > 1e-6) & (np.abs(loc[:, 1] / np.maximum(f, 1e-6)) <= math.tan(HFOV / 2)) & \
        (np.abs(loc[:, 2] / np.maximum(f, 1e-6)) <= math.tan(VFOV / 2))


def seen_by_both(sc: Scene, pts):
    """Each point in both cameras' view and not hidden by the robot (or anything nearer)."""
    ok = np.ones(len(pts), bool)
    for c, R in sc.cameras():
        v = pts - c
        r = np.linalg.norm(v, axis=1)
        u = v / r[:, None]
        gid, dist = sc.rays(c, u, GROUPS_CAM, cutoff=30.0)
        clear = (gid < 0) | (dist >= r - 0.01) | (gid == sc.floor) & (dist >= r - 0.02)
        ok &= _in_fov(R, u) & clear
    return ok


def self_share(sc: Scene):
    """Share of each camera's image filled by the robot."""
    out = []
    for c, R in sc.cameras():
        gid, dist = sc.rays(c, _cam_dirs(R), GROUPS_CAM, cutoff=5.0)
        out.append(float(np.mean((gid >= 0) & sc.self_geom[np.maximum(gid, 0)])))
    return out


def ball_arc(apex_z, dist=6.5, h0=1.5, target=(0.04, 0.0, 0.414), dur=1.5, n=60):
    """The last dur s of a drag-free arc from (dist, 0, h0) to the funnel's rim centre at the catch tilt, apex at
    apex_z (drag shortens the real arc's far part: the ball is nearer the robot sooner, which this doesn't need)."""
    g = 9.81
    x0, x1 = dist, target[0]
    z1 = target[2]
    vz0 = math.sqrt(2 * g * (apex_z - h0))
    t_up = vz0 / g
    t_dn = math.sqrt(2 * (apex_z - z1) / g)
    T = t_up + t_dn
    t = np.linspace(max(0.0, T - dur), T, n)
    x = x0 + (x1 - x0) * t / T
    z = h0 + vz0 * t - 0.5 * g * t ** 2
    return np.column_stack([x, np.zeros(n), z]), T - t


def catch_view(sc: Scene, tilt=35.0):
    sc.pose(tilt_deg=tilt)
    out = {}
    for apex in (1.6, 2.4):
        pts, t_left = ball_arc(apex)
        ok = seen_by_both(sc, pts)
        last = float(t_left[ok].min()) if ok.any() else None
        out[f"apex_{apex}"] = dict(seen=float(ok.mean()), last_seen_s_before=last)
    out["self_share"] = self_share(sc)
    return out


def floor_view(sc: Scene, q):
    sc.pose(q=q)
    x = np.arange(0.20, 1.61, 0.01)
    ahead = np.column_stack([x, np.zeros_like(x), np.full_like(x, 0.002)])
    ok = seen_by_both(sc, ahead)
    nearest = float(x[ok].min()) if ok.any() else None
    gx, gy = np.meshgrid(np.arange(0.25, 0.601, 0.025), np.arange(-0.2, 0.201, 0.025))
    near = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.002)])
    gx, gy = np.meshgrid(np.arange(0.6, 1.601, 0.05), np.arange(-0.6, 0.601, 0.05))
    far = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.002)])
    cams = sc.cameras()
    rng_near = float(np.mean([np.linalg.norm(near - c, axis=1).mean() for c, _ in cams]))
    return dict(nearest_ahead=nearest, near_zone=float(seen_by_both(sc, near).mean()),
                far_zone=float(seen_by_both(sc, far).mean()), near_range_m=rng_near, self_share=self_share(sc))


VIEWERS = (("adult 1.5 m", 1.5, 1.55, 0.0), ("kid 1.2 m", 1.2, 1.0, 0.0), ("kid 1.2 m, 40 deg off", 1.2, 1.0, 40.0),
           ("thrower 6.5 m", 6.5, 1.5, 0.0))


def display_check(sc: Scene, **pose):
    """For each viewer: share of the panel's active area seen, and the angle off the panel's axis (deg)."""
    dsp = sc.lay.display
    if dsp is None:
        return None
    sc.pose(**pose)
    gi = sc.m.geom("display").id
    c = sc.d.geom_xpos[gi]
    R = sc.d.geom_xmat[gi].reshape(3, 3)
    face = 1.0 if dsp["facing"] == "front" else -1.0
    (w, h, t), (aw, ah) = PANELS[dsp["panel"]]
    yy, zz = np.meshgrid(np.linspace(-aw / 2, aw / 2, 9), np.linspace(-ah / 2, ah / 2, 6))
    pts = c + (np.column_stack([np.full(yy.size, face * (t / 2 + 0.001)), yy.ravel(), zz.ravel()]) @ R.T)
    normal = face * R[:, 0]
    out = {}
    for name, dist, eye, az in VIEWERS:
        e = np.array([dist * math.cos(az * DEG), dist * math.sin(az * DEG), eye])
        v = pts - e
        r = np.linalg.norm(v, axis=1)
        u = v / r[:, None]
        gid, dd = sc.rays(e, u, GROUPS_EYE, cutoff=20.0)
        facing = (-u) @ normal > 0.1
        vis = facing & ((gid < 0) | (dd >= r - 0.003))
        ang = math.degrees(math.acos(np.clip(((e - c) / np.linalg.norm(e - c)) @ normal, -1, 1)))
        out[name] = dict(seen=float(vis.mean()), off_axis_deg=round(ang, 1))
    return out


def head_boxes(sc: Scene):
    """The layout's added structure, cameras and display (geom ids)."""
    m = sc.m
    names = [b[0] for b in sc.lay.boxes] + ["cam_L", "cam_R"] + (["display"] if sc.lay.display else []) + \
        [f"cad_{f.stem}" for f in mesh_files(sc.lay)]
    return [m.geom(nm).id for nm in names]


def module_clearance(sc: Scene, tilts=np.arange(-30, 41, 2)):
    """Smallest distance from the catch module (every geom below the tilt joint) to the head over the tilt range."""
    m, d = sc.m, sc.d
    tilt_body = m.body("tilt").id
    mod = [g for g in range(m.ngeom) if _below(m, m.geom_bodyid[g], tilt_body)]
    head = head_boxes(sc)
    worst = (9.0, None, None, None)
    fromto = np.zeros(6)
    for t in tilts:
        sc.pose(tilt_deg=float(t))
        for h in head:
            for g in mod:
                dist = mujoco.mj_geomDistance(m, d, g, h, 0.05, fromto)
                if dist < worst[0]:
                    worst = (float(dist), float(t), m.geom(g).name or f"geom{g}", m.geom(h).name)
    return dict(min_m=round(worst[0], 4), at_tilt=worst[1], module_geom=worst[2], head_geom=worst[3])


def _below(m, b, root):
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


def launch_clearance(sc: Scene, rb=0.037, margin=0.005):
    """For launch elevations 0..60 deg (tilt 30..-30): the ball (+ margin) along the launch line from the nip out to
    0.6 m against the head; returns the lowest elevation that clears and the clearance at 30 deg (the game's flattest)."""
    g = sc.g
    head = head_boxes(sc)
    res = {}
    for el in range(0, 61, 2):
        tilt = math.degrees(g.theta) - el
        if not (-30 <= tilt <= 40):
            continue
        nip = g.module_to_robot(g.N, tilt * DEG)
        u = np.array([math.cos(el * DEG), math.sin(el * DEG)])
        s = np.linspace(0.0, 0.6, 61)
        cx = nip[0] + s * u[0]
        cz = nip[1] + s * u[1]
        dmin = 9.0
        for h in head:
            c = sc.d.geom_xpos[h]
            R = sc.d.geom_xmat[h].reshape(3, 3)
            half = sc.m.geom_size[h]
            P = np.column_stack([cx, np.zeros_like(cx), cz]) - c
            loc = P @ R
            q = np.abs(loc) - half
            dist = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
            dmin = min(dmin, float(dist.min()))
        res[el] = dmin - rb - margin
    ok = [el for el, c in res.items() if c >= 0]
    # the lowest elevation from which every steeper one clears
    lowest = None
    for el in sorted(res, reverse=True):
        if res[el] >= 0:
            lowest = el
        else:
            break
    return dict(lowest_clear_elev=lowest, clear_at_30=round(res.get(30, float("nan")), 4), n_ok=len(ok))


# --- the arm ---------------------------------------------------------------------------------------------------------

MESH_CLEAR = 0.003                    # the arm's meshes stay this far from the drawn head
_SDF = {}


def head_sdf(index):
    """The drawn head's parts at an index (cad/v7/out/head_stl) as a 1 mm distance field (mesh_sdf.py)."""
    if index not in _SDF:
        from mesh_sdf import MeshSDF, read_stl
        _SDF[index] = MeshSDF([read_stl(f) for f in sorted((CAD_OUT / "head_stl" / index).glob("*.stl"))],
                              (-0.20, -0.15, 0.15), (0.25, 0.15, 0.51), 0.0015)
    return _SDF[index]


def arm_mesh_points(m, per=400, seed=0, fixed=False):
    """[(geom, vertices)]: a sample of each SO-101 mesh's vertices, in its geom's frame; the base's (it stands on the
    adapter, fixed) only with fixed=True."""
    rng = np.random.default_rng(seed)
    out = []
    for g in range(m.ngeom):
        body = m.body(m.geom_bodyid[g]).name
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not body.startswith("arm_") or (body == "arm_base") != fixed:
            continue
        mid = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        out.append((g, V[rng.choice(len(V), min(per, len(V)), replace=False)].copy()))
    return out


def world_points(d, pts):
    return np.vstack([V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] for g, V in pts])


class LayoutFk(AM.Fk):
    """arm_mount.Fk with the layout's extra structure (bar, display, mast, cameras) as obstacles."""

    def __init__(self, lay: Layout):
        p = lay.params()
        super().__init__(p)
        self.pods = []
        self.obst = []
        for name, (x0, x1), (y0, y1), (z0, z1) in lay.boxes:
            self.obst.append(((x0, x1), (y0, y1), (z0, z1)))
        # the CAD's head: the arm's own meshes (every link's, the base's and the shoulder's too) against the drawn
        # parts' distance field (mesh_sdf.py); the capsules alone miss the base and the shoulder bracket
        self.sdf = head_sdf(lay.meshes) if lay.meshes else None
        self.arm_pts = arm_mesh_points(self.m) if lay.meshes else None
        cx, hb, cz, _ = lay.cams
        for s in (1, -1):
            self.obst.append(((cx - 0.02, cx + 0.03), (s * hb - 0.022, s * hb + 0.022), (cz - 0.022, cz + 0.022)))
        if lay.display:
            dsp = lay.display
            (w, h, t), _ = PANELS[dsp["panel"]]
            face = 1.0 if dsp["facing"] == "front" else -1.0
            x0, x1 = sorted((dsp["x"], dsp["x"] - face * (t + 0.004)))          # the housing and its mount
            self.obst.append(((x0, x1), (dsp["y"] - w / 2, dsp["y"] + w / 2), (dsp["z"] - h / 2, dsp["z"] + h / 2)))

        self.lo, self.hi = A.joint_ranges()[0]
        self.pan_j = self.m.joint(A.name("shoulder_pan")).id

    def mesh_clearance(self) -> float:
        """m from the arm's meshes to the drawn head (inf without one)."""
        if self.sdf is None:
            return math.inf
        return float(self.sdf.clearance(world_points(self.d, self.arm_pts)).min())

    def clear(self, allow_bin=False) -> bool:
        if not super().clear(allow_bin):
            return False
        if self.sdf is not None and self.mesh_clearance() < MESH_CLEAR:
            return False
        P, r = self.points()
        for (x0, x1), (y0, y1), (z0, z1) in self.obst:
            if np.any((P[:, 0] > x0 - r) & (P[:, 0] < x1 + r) & (P[:, 1] > y0 - r) & (P[:, 1] < y1 + r)
                      & (P[:, 2] > z0 - r) & (P[:, 2] < z1 + r)):
                return False
        return True

    def above_plate(self) -> bool:
        """Nothing of the arm below the plate's top: the lidar's layer and the cameras' view under the plate stay free."""
        P, r = self.points()
        return bool(np.all(P[:, 2] - r >= PLATE_TOP + 0.002))

    def plane_table(self, n):
        """The tool and its approach over the (lift, elbow, wrist) grid at pan 0, roll 0; and the pan axis (x, y)."""
        rows = []
        lift, elbow, wrist = AM.grid(n)
        for a in lift:
            for b in elbow:
                for c in wrist:
                    self.set((0.0, a, b, c, 0.0, 0.0))
                    rows.append((a, b, c, *self.tool_pos(), *self.approach()))
        return np.array(rows), self.d.xanchor[self.pan_j][:2].copy()

    def drops(self, table, axis, bin_):
        """Poses that drop into bin_ (x, y, L, W, H): the tool 3-12 cm over its rim at its centre (within 3 cm), the fingers
        within 45 deg of down, the arm clear of everything (the bin's rim too). Reached with the arm's plane turned
        toward the bin or away from it (folded back over itself). Best first (most nearly straight down)."""
        bx, by, L, W, H = bin_
        top = self.plate_top + H
        old = self.p
        self.p = self.p.with_(arm_bin=tuple(bin_))
        rho_t = np.hypot(table[:, 3] - axis[0], table[:, 4] - axis[1]) * np.sign(table[:, 3] - axis[0])
        c = np.array([bx, by]) - axis
        rc = float(np.hypot(*c))
        out = []
        for sgn in (1.0, -1.0):                         # tool ahead of the pan axis in its plane, or behind it
            pan = math.atan2(sgn * c[1], sgn * c[0])
            if not self.lo <= pan <= self.hi:
                continue
            ok = (np.abs(np.abs(rho_t) - rc) < 0.03) & (np.sign(rho_t) == sgn) & \
                 (table[:, 5] > top + 0.03) & (table[:, 5] < top + 0.12)
            for row in table[ok]:
                q = (pan, *row[:3], 0.0, 0.0)
                self.set(q)
                t, ap = self.tool_pos(), self.approach()
                if math.hypot(t[0] - bx, t[1] - by) > 0.03 or ap @ self.ref < math.cos(math.radians(45)):
                    continue
                if self.clear(allow_bin=True) and self.above_plate():
                    out.append((float(math.degrees(math.acos(np.clip(ap @ self.ref, -1, 1)))), [float(v) for v in q]))
        self.p = old
        return sorted(out)

    def stows(self, n=22, pans=(0.0, 90.0, -90.0)):
        """Folded poses inside the bumper's edge (+15 mm), clear, nothing below the plate's top: (top z, q), lowest
        first."""
        out = []
        lift, elbow, wrist = AM.grid(n)
        for pan in pans:
            for a in lift:
                for b in elbow:
                    for c in wrist:
                        q = (math.radians(pan), a, b, c, 0.0, 0.0)
                        self.set(q)
                        P, r = self.points()
                        if np.all(np.hypot(P[:, 0], P[:, 1]) + r < AM.STOW_R) and self.above_plate() and self.clear():
                            out.append((float((P[:, 2] + r).max()), [float(v) for v in q]))
        return sorted(out)


BIN_XY =[(x, y) for x in (-0.07, -0.05, -0.03, -0.01, 0.01, 0.03) for y in (0.0, 0.05, -0.05)]


def arm_job(key, n=30):
    lay = BY_KEY[key]
    f = LayoutFk(lay)
    s = AM.sweep(f, n=n)
    td = s.pop("topdown_poses")
    side = AM.side_reach(f, td, pans=np.radians([0, 30, 60, 90])) if td else []
    grasps = {}
    if td:
        x_far, q_far = max(td, key=lambda v: v[0])
        x_near, q_near = min(td, key=lambda v: v[0])
        grasps = dict(far=(x_far, [0.0, *q_far, 0.0, 0.0]), near=(x_near, [0.0, *q_near, 0.0, 0.0]))
    out = dict(floor_topdown=s["floor_topdown"], floor_any=s["floor_any"], side=side, grasps=grasps)
    if key == "A0":
        return key, out
    table, axis = f.plane_table(26)
    bins = []
    mx = lay.arm["mount"][0]
    for bx, by in BIN_XY:
        b = (bx, by, *BIN_SIZE)
        if math.hypot(abs(bx) + BIN_SIZE[0] / 2, abs(by) + BIN_SIZE[1] / 2) > 0.19 or bx + BIN_SIZE[0] / 2 > mx - 0.05:
            continue                                            # inside the bumper (it may overhang the plate), behind
                                                                # the arm's adapter
        if any(x0 < bx + BIN_SIZE[0] / 2 and x1 > bx - BIN_SIZE[0] / 2 and y0 < by + BIN_SIZE[1] / 2
               and y1 > by - BIN_SIZE[1] / 2 for (x0, x1), (y0, y1), _ in f.obst):
            continue
        dr = f.drops(table, axis, b)
        bins.append(dict(bin=b, n_drops=len(dr), best=dr[0] if dr else None))
    out["bins"] = bins
    out["stows"] = f.stows()[:400]
    return key, out


def arm_check() -> dict:
    """mm from the SO-101's own meshes to the drawn arm robot (cad/v7/out/head_stl/arm) at robot_export.ARM_POSES,
    the ready pose and the study's far and near grasps (every link but the base), and from the base (fixed on the
    adapter) to everything but the adapter."""
    from mesh_sdf import MeshSDF, read_stl
    from robot_export import ARM_POSES
    lay = BY_KEY["A6"]
    f = LayoutFk(lay)
    poses = {k: list(v) for k, v in ARM_POSES.items()}
    if (RESULTS / "layout_trade.json").exists():
        a6 = {r["key"]: r for r in json.loads((RESULTS / "layout_trade.json").read_text())["layouts"]}.get("A6", {})
        for k, (_, q) in a6.get("arm", {}).get("grasps", {}).items():
            poses[f"{k} grasp"] = q
    out = {}
    for name, q in poses.items():
        f.set(q)
        out[name] = round(f.mesh_clearance() * 1000, 1)
    files = [x for x in sorted((CAD_OUT / "head_stl" / "arm").glob("*.stl")) if not x.stem.startswith("arm_adapter")]
    sdf = MeshSDF([read_stl(x) for x in files], (-0.20, -0.15, 0.15), (0.25, 0.15, 0.51), 0.0015)
    f.set([0.0] * 6)
    out["base, to all but its adapter"] = round(float(sdf.clearance(world_points(f.d, arm_mesh_points(f.m, fixed=True))).min()) * 1000, 1)
    return out


def _arm_tris(pose: str):
    """[(body, mesh, triangles (n, 3, 3) mm, ROBOT frame)] of the SO-101's mesh geoms at robot_export.ARM_POSES[pose]."""
    from robot_export import ARM_POSES
    f = LayoutFk(BY_KEY["A6"])
    f.set(list(ARM_POSES[pose]))
    m, d = f.m, f.d
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not m.body(m.geom_bodyid[g]).name.startswith("arm_"):
            continue
        mid = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        F = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        W = (V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]) * 1000.0
        out.append((m.body(m.geom_bodyid[g]).name, m.mesh(mid).name, W[F]))
    return out


def _write_stl(path: Path, T, title: bytes):
    import struct
    with open(path, "wb") as fh:
        fh.write(title.ljust(80, b" "))
        fh.write(struct.pack("<I", len(T)))
        rec = np.zeros(len(T), np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        rec["v"] = T
        fh.write(rec.tobytes())


def write_arm_stl(path, pose: str = "stow"):
    """The SO-101's meshes at one of robot_export.ARM_POSES, on the arm kit, as one binary STL (mm, ROBOT frame)."""
    T = np.concatenate([t for *_n, t in _arm_tris(pose)]).astype("<f4")
    path = Path(path)
    for k in range(0, len(T), 150_000):                  # MuJoCo reads at most 200 000 faces per STL
        _write_stl(path.with_name(f"{path.stem}_{k // 150_000}{path.suffix}"), T[k:k + 150_000],
                   b"SO-101 (layout_trade.py arm-stl)")


def arm_poses(pose: str = "stow") -> list[dict]:
    """Where each of the SO-101's parts is at a pose: [{body, mesh (its file's name), R (3x3), t (mm)}] taking the
    mesh file's frame (mm) to the ROBOT frame (mm). MuJoCo re-centres meshes on load (mesh_pos, mesh_quat), so a file
    point p goes to R p + t with R = geom_xmat mesh_mat^T, t = 1000 (geom_xpos - R mesh_pos). TheRobotStudio's STEP of
    each part shares its STL's frame, so cad/v8 places the exact parts with these."""
    from robot_export import ARM_POSES
    f = LayoutFk(BY_KEY["A6"])
    f.set(list(ARM_POSES[pose]))
    m, d = f.m, f.d
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not m.body(m.geom_bodyid[g]).name.startswith("arm_"):
            continue
        mid = m.geom_dataid[g]
        rm = np.zeros(9)
        mujoco.mju_quat2Mat(rm, m.mesh_quat[mid])
        R = d.geom_xmat[g].reshape(3, 3) @ rm.reshape(3, 3).T
        t = 1000.0 * (d.geom_xpos[g] - R @ m.mesh_pos[mid])
        out.append(dict(body=m.body(m.geom_bodyid[g]).name, mesh=m.mesh(mid).name.removeprefix("so101_"),
                        R=R.round(9).tolist(), t=t.round(6).tolist()))
    return out


# --- run -------------------------------------------------------------------------------------------------------------

def geometry_job(key, arm_res=None):
    lay = BY_KEY[key]
    sc = Scene(lay)
    out = dict(key=key, label=lay.label, payload=lay.payload, notes=lay.notes)
    if lay.payload == "catch":
        sc.pose(tilt_deg=0.0)
        out["lidar"] = {"any tilt": lidar_check(sc)}          # the module stays above the plate
        out["camera"] = catch_view(sc)
        out["display"] = {f"tilt {t:.0f}": display_check(sc, tilt_deg=t) for t in (35.0, -10.0)} if lay.display else None
        if lay.meshes:                                             # the CAD's exact numbers (hulls and boxes can't)
            hc = cad_head_checks()
            if hc:
                m_, l_ = hc["module"], hc["launch"]
                out["module_clear"] = dict(min_m=m_["min_mm"] / 1000, at_tilt=m_["at_tilt"], module_geom=m_["module_part"],
                                           head_geom=m_["head_part"], source="cad/v7 build.py")
                out["launch"] = dict(lowest_clear_elev=l_["lowest_clear_elev_deg"], clear_at_30=l_["clear_at_30_mm"] / 1000,
                                     source="cad/v7 build.py")
        else:
            out["module_clear"] = module_clearance(sc)
            out["launch"] = launch_clearance(sc)
    else:
        poses = {}
        if key == "A0":
            poses["stow"] = A.POSES["pick_stow"]
        elif lay.meshes:                                   # the drawn robot: its own stow (robot_export.ARM_POSES,
            from robot_export import ARM_POSES             # reachable along clear paths, not just the best view)
            q = [float(v) for v in ARM_POSES["stow"]]
            fv = floor_view(sc, q)
            poses["stow"], out["stow_choice"] = q, dict(q=q, pan_deg=round(math.degrees(q[0])), near_zone=fv["near_zone"],
                                                         far_zone=fv["far_zone"], source="robot_export.ARM_POSES")
        elif arm_res and arm_res.get("stows"):
            poses["stow"], out["stow_choice"] = best_stow(sc, arm_res["stows"])
        if arm_res and arm_res.get("grasps"):
            poses["far grasp"] = arm_res["grasps"]["far"][1]
        out["lidar"], out["camera"], out["display"] = {}, {}, {}
        for nm, q in poses.items():
            sc.pose(q=q)
            out["lidar"][nm] = lidar_check(sc)
            if nm == "stow":
                out["camera"][nm] = floor_view(sc, q)
                out["display"][nm] = display_check(sc, q=q)
        out["mass"] = mass_cg(sc, poses.get("stow"))
    return key, out


def best_stow(sc: Scene, stows, n=60):
    """Of the lowest stows (distinct tops), the one that leaves the most of the near floor zone in view, then lowest."""
    seen, cands = set(), []
    for top, q in stows:
        k = round(top, 2)
        if len(cands) >= n:
            break
        if (k, round(q[0], 2)) in seen:
            continue
        seen.add((k, round(q[0], 2)))
        cands.append((top, q))
    scored = []
    for top, q in cands:
        fv = floor_view(sc, q)
        scored.append((-round(fv["near_zone"], 2), -round(fv["far_zone"], 2), top, q))
    nz, fz, top, q = min(scored)
    return q, dict(top=top, pan_deg=round(math.degrees(q[0])), near_zone=-nz, far_zone=-fz, n_candidates=len(cands), q=q)


def mass_cg(sc: Scene, q=None):
    """Mass and CG (robot frame) of the sim model with the layout's parts (the added parts at est masses: bar 0.08 kg,
    each camera 0.03, a 5 in panel and housing 0.16, a 7 in 0.30, the mast 0.10)."""
    m, d = sc.m, sc.d
    if q is not None:
        sc.pose(q=q)
    else:
        sc.pose(tilt_deg=0.0) if sc.lay.payload == "catch" else None
    b = m.body("base").id
    M = float(m.body_subtreemass[b])
    cg = d.subtree_com[b] * M
    extra = []
    for name, (x0, x1), (y0, y1), (z0, z1) in sc.lay.boxes:
        kg = 0.10 if name == "mast" else 0.08
        extra.append((kg, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))
    cx, hb, cz, _ = sc.lay.cams
    extra += [(0.03, (cx, hb, cz)), (0.03, (cx, -hb, cz))]
    for f in mesh_files(sc.lay):
        lo, hi = stl_bounds(f)
        extra.append((next(v for k, v in MESH_KG.items() if f.stem.startswith(k)), tuple((lo + hi) / 2)))
    if sc.lay.display:
        dsp = sc.lay.display
        extra.append(({"5in": 0.16, "5in_cad": 0.13}.get(dsp["panel"], 0.30), (dsp["x"], dsp["y"], dsp["z"])))
    for kg, pos in extra:
        M += kg
        cg = cg + kg * np.array(pos)
    return dict(kg=round(M, 3), cg=[round(float(v), 4) for v in cg / M])


def main():
    keys = [lay.key for lay in LAYOUTS]
    arm_keys = [k for k in keys if BY_KEY[k].payload == "arm" and k != "A0"]
    with ProcessPoolExecutor(max_workers=8) as ex:
        arm = dict(ex.map(arm_job, arm_keys))
    prev = json.loads((RESULTS / "arm_mount.json").read_text())
    a0 = next(r for r in prev["rows"] if r["label"] == "nose x 120 z 105")
    arm["A0"] = dict(floor_topdown=a0["floor_topdown"], floor_any=a0["floor_any"], bin_drop=a0["bin_drop"],
                     stow=a0["stow"], side=[s for s in prev["side"] if s["pan_deg"] in (0.0, 30.0, 60.0, 90.0)
                                            or abs(s["pan_deg"] - 30) < 1e-6 or abs(s["pan_deg"] - 60) < 1e-6],
                     source="results/arm_mount.json (the Oct 24 study)")
    with ProcessPoolExecutor(max_workers=8) as ex:
        geo = dict(ex.map(geometry_job, keys, [arm.get(k) for k in keys]))
    for k in arm:
        a = dict(arm[k])
        a.pop("stows", None)
        geo[k]["arm"] = a
    save("layout_trade", dict(layouts=[geo[k] for k in keys]))
    report()


# --- pictures --------------------------------------------------------------------------------------------------------

RENDER_SET = (("C0", dict(tilt_deg=35.0)), ("C1", dict(tilt_deg=35.0)), ("C2", dict(tilt_deg=35.0)), ("A0", "stow"),
              ("A5", "stow"), ("A6", "stow"), ("A6", "far grasp"), ("A6", "drop"))


def _render(sc: Scene, views, w=520, h=420):
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 1, 1]
    rnd = mujoco.Renderer(sc.m, h, w)
    out = []
    for lookat, dist, az, el in views:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        rnd.update_scene(sc.d, camera=cam, scene_option=opt)
        out.append(rnd.render().copy())
    rnd.close()
    return out


def pictures():
    """renders/layout_trade.png: today's catch and arm robots against the proposal (3/4 front and side views), and
    renders/layout_lidar.png: what each lidar sees in its scan plane."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = {r["key"]: r for r in json.loads((RESULTS / "layout_trade.json").read_text())["layouts"]}
    views = (((0.08, 0.0, 0.24), 1.0, 215.0, -14.0), ((0.05, 0.0, 0.24), 1.05, 90.0, -2.0))
    fig, axes = plt.subplots(len(RENDER_SET), 2, figsize=(9.2, 4.0 * len(RENDER_SET)))
    for row, (key, pose) in enumerate(RENDER_SET):
        sc = Scene(BY_KEY[key])
        title = f"{key}: {BY_KEY[key].label}"
        if isinstance(pose, dict):
            sc.pose(**pose)
            title += f" (tilt {pose['tilt_deg']:.0f} deg)"
        else:
            r = res[key]
            if pose == "stow":
                q = A.POSES["pick_stow"] if key == "A0" else r["stow_choice"]["q"]
            elif pose == "far grasp":
                q = r["arm"]["grasps"]["far"][1]
            else:
                b = next(b for b in r["arm"]["bins"] if b["n_drops"])
                q = b["best"][1]
            sc.pose(q=q)
            title += f" ({pose})"
        for col, img in enumerate(_render(sc, views)):
            ax = axes[row, col]
            ax.imshow(img)
            ax.axis("off")
            if col == 0:
                ax.set_title(title, fontsize=8, loc="left")
    fig.tight_layout()
    RENDERS.mkdir(exist_ok=True)
    fig.savefig(RENDERS / "layout_trade.png", dpi=110)
    plt.close(fig)
    # lidar shadows, top view
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.3), subplot_kw=dict(aspect="equal"))
    for ax, (key, pose) in zip(axes, (("C2", "any tilt"), ("A6", "far grasp"), ("A0", "stow"))):
        lay = BY_KEY[key]
        l = res[key]["lidar"][pose]
        ox, oy = lay.lidar
        for s0, wdt in l["sectors"]:
            a = np.radians(np.linspace(s0, s0 + wdt, 20))
            ax.fill(np.r_[ox, ox + 1.5 * np.cos(a)], np.r_[oy, oy + 1.5 * np.sin(a)], color="#c0392b", alpha=0.35, lw=0)
        t = np.linspace(0, 2 * np.pi, 200)
        ax.plot(BUMPER_R * np.cos(t), BUMPER_R * np.sin(t), color="#555", lw=1)
        ax.plot(0.12 * np.cos(t), 0.12 * np.sin(t), color="#999", lw=0.8, ls="--")
        for (x, y), r in zip(STANDOFFS, lay.standoff_r):
            ax.add_patch(plt.Circle((x, y), r, color="#333"))
        ax.plot(ox, oy, "o", color="#2471a3")
        ax.annotate("front", (0.32, 0.0), fontsize=8, ha="center")
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.6, 0.6)
        ax.set_title(f"{key} ({pose}): {l['blocked_deg']:.0f} deg blocked, widest {l['widest_deg']:.0f}", fontsize=9)
    fig.tight_layout()
    fig.savefig(RENDERS / "layout_lidar.png", dpi=110)
    plt.close(fig)
    print(f"wrote {RENDERS / 'layout_trade.png'} and {RENDERS / 'layout_lidar.png'}")


PROPOSAL = (("C2", "Catch robot (as drawn in cad/v7)", (("catching, module tilted 35 deg", dict(tilt_deg=35.0)),
                                                         ("returning a throw, launch 40 deg", dict(tilt_deg=-10.0)))),
            ("A6", "Arm robot (the drawn head at its arm index)", (("arm stowed", "stow"), ("grasping at full reach", "far grasp"),
                                                                   ("dropping into the bin", "drop"))))


def pictures_proposal():
    """renders/layout_proposal_catch.png and _arm.png: the proposal from three sides at its main poses."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = {r["key"]: r for r in json.loads((RESULTS / "layout_trade.json").read_text())["layouts"]}
    views = ((("front 3/4"), ((0.06, 0.0, 0.24), 0.95, 215.0, -16.0)), ("side", ((0.04, 0.0, 0.24), 1.0, 90.0, -3.0)),
             ("rear 3/4", ((0.0, 0.0, 0.24), 0.95, 315.0, -22.0)))
    for key, name, poses in PROPOSAL:
        sc = Scene(BY_KEY[key])
        r = res[key]
        fig, axes = plt.subplots(len(poses), 3, figsize=(15, 4.4 * len(poses)))
        for row, (label, pose) in enumerate(poses):
            if isinstance(pose, dict):
                sc.pose(**pose)
            else:
                q = {"stow": lambda: r["stow_choice"]["q"], "far grasp": lambda: r["arm"]["grasps"]["far"][1],
                     "drop": lambda: next(b for b in r["arm"]["bins"] if b["n_drops"])["best"][1]}[pose]()
                sc.pose(q=q)
            imgs = _render(sc, [v for _, v in views], w=720, h=560)
            for col, img in enumerate(imgs):
                ax = axes[row, col]
                ax.imshow(img)
                ax.axis("off")
                ax.set_title(f"{label} - {views[col][0]}", fontsize=10)
        fig.suptitle(f"{name}: {BY_KEY[key].label}", fontsize=12)
        fig.tight_layout()
        out = RENDERS / f"layout_proposal_{'catch' if key.startswith('C') else 'arm'}.png"
        fig.savefig(out, dpi=100)
        plt.close(fig)
        print(f"wrote {out}")


def _add(scn, kind, rgba, width=0.0, a=None, b=None, pos=None, size=None):
    """Append a decor geom to a rendered scene: a connector a -> b (line / capsule) or a sphere at pos."""
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([size or 0.01, 0, 0]),
                        np.asarray(pos if pos is not None else (0, 0, 0), float), np.eye(3).ravel(),
                        np.asarray(rgba, np.float32))
    if kind != "sphere":
        mujoco.mjv_connector(g, kind, width, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def _frustum_rays(sc: Scene, reach=2.0):
    """Per camera: (lens, [4 corner end points]) - the corner rays out to `reach` m, or to the floor."""
    out = []
    tu, tv = math.tan(HFOV / 2), math.tan(VFOV / 2)
    for c, R in sc.cameras():
        ends = []
        for u, v in ((-tu, -tv), (tu, -tv), (tu, tv), (-tu, tv)):
            dvec = R @ np.array([1.0, -u, v])
            dvec /= np.linalg.norm(dvec)
            t = reach
            if dvec[2] < -1e-6:
                t = min(t, -c[2] / dvec[2])
            ends.append(c + t * dvec)
        out.append((c, ends))
    return out


def _overlay(sc: Scene, scn, reach):
    """The lidar's scan fan, every 2 deg (green to its first hit or 0.9 m; red where the robot shadows it) and both
    camera frustums (left cyan, right magenta; on the floor where they reach it)."""
    LINE = mujoco.mjtGeom.mjGEOM_LINE
    n, L = 180, 0.9
    ang = np.arange(n) * 2 * math.pi / n
    vec = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(n)])
    o = np.array([*sc.lay.lidar, LIDAR_Z])
    gid, dist = sc.rays(o, vec, GROUPS_LIDAR, cutoff=L)
    for k in range(n):
        selfhit = gid[k] >= 0 and sc.self_geom[gid[k]]
        end = o + vec[k] * (dist[k] if gid[k] >= 0 else L)
        if selfhit:
            _add(scn, LINE, (0.95, 0.1, 0.05, 1.0), 2.5, o, end)
            _add(scn, LINE, (0.95, 0.1, 0.05, 0.6), 2.5, end, o + vec[k] * L)
        else:
            _add(scn, LINE, (0.1, 0.7, 0.2, 0.45), 1.0, o, end)
    for (c, ends), col in zip(_frustum_rays(sc, reach), ((0.0, 0.8, 0.95, 1.0), (0.95, 0.2, 0.85, 1.0))):
        for k in range(4):
            _add(scn, LINE, col, 3.0, c, ends[k])
            _add(scn, LINE, col, 3.0, ends[k], ends[(k + 1) % 4])


def _render_overlay(sc: Scene, views, reach, w=720, h=560):
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 1, 1]
    rnd = mujoco.Renderer(sc.m, h, w)
    out = []
    for lookat, dist, az, el in views:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        rnd.update_scene(sc.d, camera=cam, scene_option=opt)
        _overlay(sc, rnd.scene, reach)
        out.append(rnd.render().copy())
    rnd.close()
    return out


def _camera_eye(sc: Scene, extras=(), w=640, h=400):
    """What the left camera sees (pinhole, 70 deg vertical, 1280 x 800 aspect), with extra decor (ball arcs, floor
    marks) added to the scene."""
    c, R = sc.cameras()[0]
    sc.m.vis.global_.fovy = 70.0
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 1, 1]
    rnd = mujoco.Renderer(sc.m, h, w)
    cam = mujoco.MjvCamera()
    f = R[:, 0]
    cam.lookat[:] = c + 0.5 * f
    cam.distance = 0.5
    cam.azimuth = math.degrees(math.atan2(f[1], f[0]))
    cam.elevation = math.degrees(math.asin(f[2]))
    rnd.update_scene(sc.d, camera=cam, scene_option=opt)
    for e in extras:
        _add(rnd.scene, **e)
    img = rnd.render().copy()
    rnd.close()
    return img


def _ball_marks():
    out = []
    for apex, col in ((1.6, (1.0, 0.55, 0.1, 1.0)), (2.4, (1.0, 0.85, 0.1, 1.0))):
        pts, _ = ball_arc(apex, dur=1.5, n=16)
        out += [dict(kind="sphere", rgba=col, pos=p, size=0.037) for p in pts]
    return out


def _floor_marks():
    """White lines across the floor at 0.25 (just past the bumper), 0.33 (where the tidy stops), 0.6, 1.0, 1.6 m."""
    out = []
    for x in (0.25, 0.33, 0.6, 1.0, 1.6):
        half = 0.25 + 0.35 * x
        out.append(dict(kind=mujoco.mjtGeom.mjGEOM_CAPSULE, rgba=(1, 1, 1, 1), width=0.004 + 0.002 * x,
                        a=(x, -half, 0.003), b=(x, half, 0.003)))
    for x, y, col in ((0.33, 0.0, (0.9, 0.1, 0.1, 1)), (0.45, 0.15, (0.1, 0.4, 0.9, 1)), (0.8, -0.25, (0.1, 0.7, 0.2, 1)),
                      (1.2, 0.3, (0.9, 0.6, 0.1, 1))):
        out.append(dict(kind="sphere", rgba=col, pos=(x, y, 0.02), size=0.02))       # toys on the floor
    return out


def pictures_fov():
    """renders/layout_fov_catch.png and _arm.png: the cameras' frustums and the lidar's scan fan on the proposal,
    from above and from the side, and what the left camera sees."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = {r["key"]: r for r in json.loads((RESULTS / "layout_trade.json").read_text())["layouts"]}
    views = (("from above (front at the top)", ((0.25, 0.0, 0.0), 2.0, 0.0, -72.0)),
             ("side (front to the right)", ((0.35, 0.0, 0.35), 1.9, 90.0, -6.0)),
             ("front 3/4", ((0.25, 0.0, 0.2), 1.9, 215.0, -24.0)))
    sets = (("C2", "Catch robot (as drawn): cameras 30 deg up, module at the 35 deg catch tilt", dict(tilt_deg=35.0),
             1.2, "catch", _ball_marks(), "left camera's view: the last 1.5 s of 6.5 m throws, apex 1.6 and 2.4 m"),
            ("A6", "Arm robot (the drawn head): cameras 40 deg down, arm stowed", "stow", 1.6, "arm", _floor_marks(),
             "left camera's view: floor lines at 0.25 / 0.33 / 0.6 / 1.0 / 1.6 m, four toys"))
    for key, title, pose, reach, tag, extras, eye_title in sets:
        sc = Scene(BY_KEY[key])
        if isinstance(pose, dict):
            sc.pose(**pose)
        else:
            sc.pose(q=res[key]["stow_choice"]["q"])
        imgs = _render_overlay(sc, [v for _, v in views], reach)
        eye = _camera_eye(sc, extras)
        fig = plt.figure(figsize=(15, 9.6))
        gs = fig.add_gridspec(2, 3)
        for col, img in enumerate(imgs[:2]):
            ax = fig.add_subplot(gs[0, col])
            ax.imshow(img)
            ax.axis("off")
            ax.set_title(views[col][0], fontsize=10)
        ax = fig.add_subplot(gs[0, 2])
        ax.imshow(imgs[2])
        ax.axis("off")
        ax.set_title(views[2][0], fontsize=10)
        ax = fig.add_subplot(gs[1, 0:2])
        ax.imshow(eye)
        ax.axis("off")
        ax.set_title(eye_title, fontsize=10)
        l = res[key]["lidar"]["any tilt" if tag == "catch" else "stow"]
        ax = fig.add_subplot(gs[1, 2])
        ax.axis("off")
        cam = res[key]["camera"]
        if tag == "catch":
            cams = (f"Both cameras see {cam['apex_1.6']['seen']:.0%} / {cam['apex_2.4']['seen']:.0%} of the last 1.5 s\n"
                    f"of the two throws (lost ~{cam['apex_1.6']['last_seen_s_before']:.2f} s before the funnel).")
        else:
            c = cam["stow"]
            cams = (f"Both cameras see the floor from {c['nearest_ahead']:.2f} m ahead,\n{c['near_zone']:.0%} of the near "
                    f"zone (0.25-0.6 m), {c['far_zone']:.0%} of the far zone\n(0.6-1.6 m, |y| < 0.6 m).")
        ax.text(0.0, 0.95, f"Lidar (scan plane 150 mm, front of the deck):\n{l['blocked_deg']:.0f} deg in the robot's "
                f"shadow (red),\nwidest wedge {l['widest_deg']:.0f} deg; green = clear (drawn to 0.9 m).\n\n"
                f"Cameras (OV9782, 96 x 70 deg, 190 mm apart):\nleft = cyan, right = magenta.\n" + cams,
                va="top", fontsize=11, family="monospace")
        fig.suptitle(title, fontsize=12)
        fig.tight_layout()
        out = RENDERS / f"layout_fov_{tag}.png"
        fig.savefig(out, dpi=100)
        plt.close(fig)
        print(f"wrote {out}")


# --- L: what the lidar's blocked sectors cost the localizer ----------------------------------------------------------

def lidar_masks():
    """(name, [(start deg, width deg), ...]) in the lidar's frame: the layouts' own shadows and bigger synthetic ones."""
    res = json.loads((RESULTS / "layout_trade.json").read_text())
    by = {r["key"]: r for r in res["layouts"]}
    return [("full 360", []),
            ("front lidar, CAD standoffs (C1, A5 stowed)", by["C1"]["lidar"]["any tilt"]["sectors"]),
            ("front lidar, Ø8 front rods (C2, as drawn)", by["C2"]["lidar"]["any tilt"]["sectors"]),
            ("A5 during a far grasp", by["A5"]["lidar"]["far grasp"]["sectors"]),
            ("rear lidar, nose arm (A0 today)", by["A0"]["lidar"]["stow"]["sectors"]),
            ("rear 90 deg blocked", [(135.0, 90.0)]),
            ("rear half blocked (180 deg)", [(90.0, 180.0)]),
            ("front 90 deg blocked", [(315.0, 90.0)])]


def tour_job(args):
    """localization.py's house tour (v7 base, BNO085 + A2M12, wheels+IMU+lidar) with the scan's sectors blanked."""
    name, sectors, seed = args
    import house
    from estimator import Localizer
    from localization import TOUR, _stats
    from omni_control import wrap_angle
    from sensors import Sensors, SensorConfig
    drv = house.Driver(house.house_params("v7", assist=False))
    sn = Sensors(drv.r, SensorConfig.parts(imu="BNO085", lidar="A2M12", seed=seed))
    step = sn.step

    def masked():
        rd = step()
        if rd.scan is not None and sectors:
            a = np.degrees(rd.scan.ang) % 360.0
            bad = np.zeros(len(a), bool)
            for s0, w in sectors:
                bad |= (a - s0) % 360.0 < w
            rd.scan.rng[bad] = np.nan
        return rd

    sn.step = masked
    loc = Localizer(drv.r, sn, variants=["wheels+IMU+lidar"])
    r, d = drv.r, drv.d
    pe, ye = [], []
    nxt = 0.0
    for wp in TOUR:
        target = np.array([wp[0], wp[1], math.radians(wp[2])])
        t0 = d.time
        while d.time - t0 < 8.0:
            r.drive_to(r.pose(), target)
            r.step()
            loc.step()
            if d.time >= nxt:
                nxt = d.time + 0.1
                p_err, y_err = loc.errors()["wheels+IMU+lidar"]
                pe.append(p_err)
                ye.append(y_err)
            pose = r.pose()
            if np.linalg.norm(pose[:2] - target[:2]) < 0.03 and abs(wrap_angle(pose[2] - target[2])) < 0.05:
                break
    f = loc.f["wheels+IMU+lidar"]
    return dict(mask=name, seed=seed, pos=_stats(pe), yaw=_stats(ye), fixes=f.n_lidar, rejected=f.n_lidar_rej,
                relocs=f.n_reloc)


def lidar_value(seeds=(0, 1, 2)):
    jobs = [(name, sec, s) for name, sec in lidar_masks() for s in seeds]
    with ProcessPoolExecutor(max_workers=12) as ex:
        rows = list(ex.map(tour_job, jobs))
    masks = {name: dict(sectors=sec, blocked_deg=round(sum(w for _, w in sec), 1)) for name, sec in lidar_masks()}
    save("layout_lidar", dict(masks=masks, rows=rows))
    report_lidar()


def report_lidar():
    res = json.loads((RESULTS / "layout_lidar.json").read_text())
    print(f"\n{'mask':45s} {'blocked':>8s} {'pos med':>8s} {'p95':>6s} {'max':>6s} {'yaw p95':>8s} {'rejected':>9s}")
    for name, mk in res["masks"].items():
        rows = [r for r in res["rows"] if r["mask"] == name]
        med = np.mean([r["pos"]["med"] for r in rows]) * 1000
        p95 = np.mean([r["pos"]["p95"] for r in rows]) * 1000
        mx = max(r["pos"]["max"] for r in rows) * 1000
        yp = np.mean([r["yaw"]["p95"] for r in rows])
        rej = np.mean([r["rejected"] / max(1, r["fixes"] + r["rejected"]) for r in rows])
        print(f"{name:45s} {mk['blocked_deg']:7.0f}d {med:6.1f}mm {p95:5.1f} {mx:6.1f} {yp:7.2f}d {rej:8.1%}")


def report():
    res = json.loads((RESULTS / "layout_trade.json").read_text())
    for r in res["layouts"]:
        print(f"\n== {r['key']}: {r['label']}")
        for pose, l in r["lidar"].items():
            print(f"  lidar [{pose}]: blocked {l['blocked_deg']} deg, widest {l['widest_deg']} deg, "
                  f"hidden-lane headings {len(l['hidden_lane_headings'])}/180")
        cam = r["camera"]
        if r["payload"] == "catch":
            print(f"  cameras: apex 1.6 seen {cam['apex_1.6']['seen']:.2f} (last {cam['apex_1.6']['last_seen_s_before']} s), "
                  f"apex 2.4 seen {cam['apex_2.4']['seen']:.2f}; robot in image {cam['self_share']}")
            print(f"  module clearance to the head: {r['module_clear']}")
            print(f"  launch: {r['launch']}")
        else:
            for pose, c in cam.items():
                print(f"  floor [{pose}]: nearest ahead {c['nearest_ahead']}, near {c['near_zone']:.2f}, "
                      f"far {c['far_zone']:.2f}, range near {c['near_range_m']:.2f} m, robot in image "
                      f"{[round(x, 2) for x in c['self_share']]}")
            a = r.get("arm", {})
            td = a.get("floor_topdown")
            print(f"  arm: top-down floor {td[0]:.3f}-{td[1]:.3f}" if td else "  arm: no top-down floor grasp",
                  f"(pan 30/60/90: " + ", ".join(f"{s['far'][0]:.3f}" for s in a.get("side", [])[1:]) + ")")
            if "stow_choice" in r:
                print(f"  stow: {r['stow_choice']}")
            elif a.get("stow"):
                print(f"  stow top {a['stow'][0]:.3f} (arm_mount.json)")
            for b in a.get("bins", []):
                if b["n_drops"]:
                    print(f"     bin at ({b['bin'][0]:+.2f}, {b['bin'][1]:+.2f}): {b['n_drops']} drops, best "
                          f"{b['best'][0]:.0f} deg off vertical, pan {math.degrees(b['best'][1][0]):.0f}")
            if a.get("bins") is not None and not any(b["n_drops"] for b in a["bins"]):
                print("     no bin position on the plate gets a drop")
            if a.get("bin_drop"):
                print(f"     bin drop (arm_mount.json): {a['bin_drop'][2]:.0f} deg off vertical")
        if r.get("mass"):
            print(f"  mass {r['mass']['kg']} kg, CG {r['mass']['cg']}")
        if r.get("display"):
            for pose, dv in r["display"].items():
                if dv:
                    print(f"  display [{pose}]: " + ", ".join(f"{k} {v['seen']:.2f}@{v['off_axis_deg']:.0f}deg"
                                                          for k, v in dv.items()))


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else "G"
    if arg == "report":
        report()
        if (RESULTS / "layout_lidar.json").exists():
            report_lidar()
    elif arg == "L":
        lidar_value()
    elif arg == "arm-check":
        print(json.dumps(arm_check()))
    elif arg == "arm-stl":
        write_arm_stl(Path(sys.argv[2]) / "petg_grey__so101.stl", sys.argv[3] if len(sys.argv) > 3 else "stow")
    elif arg == "arm-poses":
        print(json.dumps(arm_poses(sys.argv[2] if len(sys.argv) > 2 else "stow")))
    elif arg == "png":
        pictures()
        pictures_proposal()
        pictures_fov()
    else:
        main()
