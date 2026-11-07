"""cad/v13 (Nov 5 2026, STUDY, not drawn into the robot): a generic IMX662 USB 2 camera (Arducam-sized) at the nose, under the lidar,
tilted up, as the arm robot's forward-looking camera (tags, people, the room). Fernando: no full CAD, a generic STEP to see it fits.

    python -B imx662_fit.py cloud      the SO-101 swept near the nose (d435_riser's motion; cached: validation/imx662/arm_cloud.npz)
    python -B imx662_fit.py check      the camera, its plug + lead and a placeholder stand at every candidate pose (fit.json, renders,
                                       imx662_module.step, imx662_fit_review.step)          (omni-robot-sim interpreter, from tools/)

The module is generic, nothing here is a vendor drawing (VERIFY on the board bought): a 38 x 38 mm board (Arducam's usual UVC size),
1.6 mm PCB, a 30 x 30 x 3.5 mm stack of parts on its back (the UVC bridge), an M12 lens holder 16 x 16 x 7 and a Ø14 x 16 mm M12 lens
(a 2.8 mm wide lens: 90 x 58 deg on the IMX662's 5.6 x 3.1 mm 1920 x 1080 array), a USB-C socket on the back in the board's upper half
with a straight plug (12.4 x 6.5 x 21, as the D435's bench cable) and its lead (Ø4.5) bent sharply down 4 mm past the plug. The stand is a
placeholder for whatever holds it (a back plate on four standoffs and a leg to node_front's top), there to show there is room for one.

Camera frame (as d435_riser.cam_R with a negative tilt): x along the view, y to its left, z its up; origin the PCB's front face centre.
Robot frame: x forward from the centre, z up from the floor, mm.
"""
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build123d as b                                                  # noqa: E402
import d435_riser as DR                                                # noqa: E402

VAL = HERE.parent / 'validation' / 'imx662'
SIM = HERE.parents[2] / 'simulation' / 'v11 sil'

# the module (generic, VERIFY)
BOARD, PCB_T = 38.0, 1.6
BACK = (30.0, 3.5)                                                     # parts on the back: side, height
HOLDER = (16.0, 7.0)                                                   # M12 lens holder: side, height off the PCB
LENS = (14.0, 16.0)                                                    # lens barrel: diameter, length past the holder
PUPIL_BACK = 4.0                                                       # the entrance pupil this far behind the lens's front (est): the sim's camera
PLUG = (21.0, 12.4, 6.5)                                               # straight USB-C: length, width, thickness
PLUG_Z = 9.0                                                           # socket centre above the board's middle (camera z)
LEAD_D, LEAD_STRAIGHT = 4.5, 4.0
LEAD_Z, LEAD_Y = 110.0, 26.0                                           # the lead's run back beside the pedestal tube, clipped to it: z 72 (on
                                                                       # the flange) was 1.6 mm from the FR hub at droop; at z 110, >= 5.2
FOV = (89.6, 58.4)                                                     # deg: 2.8 mm on 1920 x 2.9 um, 1080 x 2.9 um (pinhole, rectified)
# the robot (chassis.py / upper.py / d435_riser.py, read only)
Z_NODE_TOP = 62.0                                                      # chassis.Z1: node_front's top
SCAN_Z, LIDAR_MARGIN = DR.SCAN_Z, DR.LIDAR_MARGIN
R_BUMPER_IN = 222.15                                                   # the bumper's inside (upper.R_FLOOR + gap: 225.15 - 3)
STAND_T, STAND_GAP = 3.0, 1.0                                          # back plate, its gap behind the back parts
# candidates: (lens front centre x, z, tilt up deg)
CANDIDATES = {f'z{z:.0f}_up{t:.0f}': (214.0, z, t) for z in (95.0, 105.0, 115.0) for t in (10.0, 15.0, 20.0, 25.0, 30.0)}
DEFAULT = 'z105_up20'


def R_up(tilt_up):
    return DR.cam_R(-tilt_up, 0.0)


def frame(lens_x, lens_z, tilt_up):
    """(origin = PCB front face centre, R camera -> robot) for the lens's front centre at (lens_x, 0, lens_z)"""
    R = R_up(tilt_up)
    return np.array([lens_x, 0.0, lens_z]) - R @ np.array([HOLDER[1] + LENS[1], 0.0, 0.0]), R


def _mat_loc(M):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*[float(v) for v in M[0]], *[float(v) for v in M[1]], *[float(v) for v in M[2]])
    return b.Location(t)


def bx(x0, x1, y0, y1, z0, z1):
    return b.Box(x1 - x0, y1 - y0, z1 - z0).moved(b.Location(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def module_local():
    """the camera module in its own frame: {name: solid}"""
    h = BOARD / 2
    cam = bx(-PCB_T, 0, -h, h, -h, h)
    cam = cam + bx(-PCB_T - BACK[1], -PCB_T, -BACK[0] / 2, BACK[0] / 2, -BACK[0] / 2, BACK[0] / 2)
    cam = cam + bx(0, HOLDER[1], -HOLDER[0] / 2, HOLDER[0] / 2, -HOLDER[0] / 2, HOLDER[0] / 2)
    cam = cam + b.Cylinder(LENS[0] / 2, LENS[1]).moved(b.Location((HOLDER[1] + LENS[1] / 2, 0, 0), (0, 1, 0), 90))
    return dict(imx662=cam)


def plug_local(o, R):
    """the plug (camera frame) and its lead in the robot frame: straight back out of the socket, then sharply down to node_front's top"""
    x0 = -PCB_T - BACK[1]
    plug = bx(x0 - PLUG[0], x0, -PLUG[1] / 2, PLUG[1] / 2, PLUG_Z - PLUG[2] / 2, PLUG_Z + PLUG[2] / 2).moved(_mat_loc(_M(o, R)))
    a = o + R @ np.array([x0 - PLUG[0], 0.0, PLUG_Z])                 # the plug's end
    bend = a + R @ np.array([-LEAD_STRAIGHT, 0.0, 0.0])
    down = np.array([bend[0], 0.0, LEAD_Z])                            # under the pedestal's top flange (z 126.7), over the wheels' hubs
    side = down + np.array([0.0, -LEAD_Y, 0.0])                       # beside the pedestal's tube (r 20); y -45 met the FR wheel
    back = np.array([95.0, -LEAD_Y, LEAD_Z])                          # on past the tube, toward the computer (not drawn further)
    lead = None
    for p, q in ((a, bend), (bend, down), (down, side), (side, back)):
        seg = _cyl(p, q, LEAD_D / 2)
        lead = seg if lead is None else lead + seg
        lead = lead + b.Sphere(LEAD_D / 2).moved(b.Location(tuple(q)))
    return plug, lead


def _M(o, R):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, o
    return M


def _cyl(p, q, r):
    v = np.asarray(q, float) - np.asarray(p, float)
    L = float(np.linalg.norm(v))
    return b.Solid.make_cylinder(r, L, b.Plane(origin=tuple(p), z_dir=tuple(v / L)))


def stand(o, R):
    """placeholder stand: a plate behind the back parts (4 standoffs to the board's corners, a window for the plug) and a leg from its
    lower edge straight down to node_front's top"""
    h = BOARD / 2
    xb = -PCB_T - BACK[1] - STAND_GAP
    plate = bx(xb - STAND_T, xb, -h - 2, h + 2, -h - 2, h + 2)
    plate = plate - bx(xb - STAND_T - 1, xb + 1, -PLUG[1] / 2 - 1, PLUG[1] / 2 + 1, PLUG_Z - PLUG[2] / 2 - 1, h + 3)
    for sy in (-1, 1):
        for sz in (-1, 1):
            plate = plate + b.Cylinder(2.5, -PCB_T - xb).moved(b.Location(((xb - PCB_T) / 2, sy * (h - 3), sz * (h - 3)), (0, 1, 0), 90))
    M = _M(o, R)
    plate = plate.moved(_mat_loc(M))
    # leg: the plate's lower edge (camera z -h-2, x xb-STAND_T..xb) down to the node, +-14 wide (clear of nothing: the plug is up top)
    pts = [o + R @ np.array([x, 0.0, -h - 2]) for x in (xb - STAND_T, xb)]
    xs = sorted(p[0] for p in pts)
    ztop = min(p[2] for p in pts) + 1.0
    if ztop < Z_NODE_TOP + 2.0:
        return plate                                                  # the plate's edge is at (or under) the node: no leg
    return plate + bx(xs[0] - 6.0, xs[1] + 2.0, -14, 14, Z_NODE_TOP, ztop)


def place(cand):
    """{name: solid in the robot frame} for one candidate, and its optical centre / axis"""
    lx, lz, t = CANDIDATES[cand] if isinstance(cand, str) else cand
    o, R = frame(lx, lz, t)
    M = _mat_loc(_M(o, R))
    out = {k: s.moved(M) for k, s in module_local().items()}
    out['plug_imx662'], out['lead_imx662'] = plug_local(o, R)
    out['stand_imx662'] = stand(o, R)
    pupil = o + R @ np.array([HOLDER[1] + LENS[1] - PUPIL_BACK, 0, 0])
    return out, dict(lens_front=[lx, 0.0, lz], tilt_up_deg=t, pupil=[round(float(v), 2) for v in pupil], axis=list(np.round(R[:, 0], 4)),
                     pcb_centre=[round(float(v), 2) for v in o])


def frustum(pupil, R, L=500.0, fov=FOV):
    th, tv = math.tan(math.radians(fov[0] / 2)), math.tan(math.radians(fov[1] / 2))
    p = np.asarray(pupil, float)
    rects = [b.Wire.make_polygon([b.Vector(*(p + R @ np.array([d, sy * d * th, sz * d * tv]))) for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))],
                                 close=True) for d in (1.0, L)]
    return b.Solid.make_loft(rects)


# ---- the arm near the nose -------------------------------------------------------------------------------------------------------------
def arm_cloud():
    """d435_riser's swept cloud, but kept round the nose (its own is kept from z 130 up, round the riser)"""
    VAL.mkdir(parents=True, exist_ok=True)
    if not (VAL / 'arm_motion.json').exists():
        shutil.copy(DR.VAL / 'arm_motion.json', VAL / 'arm_motion.json')     # the same poses and moves, read-only from d435/
    DR.VAL, DR.ROI, DR.VOX = VAL, ((150.0, 330.0), (-80.0, 80.0), (0.0, 175.0)), 1.0
    return DR.load_cloud()


# ---- the robot round the nose ----------------------------------------------------------------------------------------------------------
def world():
    """the printed parts, bought and vendor models, chassis and fins, and the two front pods at droop, nominal and bump"""
    import base as B
    import chassis as C
    import first_floor as FF
    import pod as P
    import pod_lean as L
    import upper as U
    printed, bought, vendor, _, _ = U.build()
    vend = U.load_vendor(vendor)
    parts, _, env, _ = C.build(FF.pod_geometry())
    out = dict(printed, **bought, **vend, **parts)
    out.update({k: v for k, v in env.items() if k in ('env_lidar',)})
    sp, _ = L.build_static()
    for tag, a in (('droop', L.A_DROOP), ('nom', 0.0), ('bump', L.A_BUMP)):
        sh, _ = L.posed(sp, P.wheel_unit(), a)
        for i in (0, 3):                                              # FL, FR
            for k, s in sh.items():
                if not k.startswith(('bolt', 'spring', 'die_spring', 'washer', 'dowel', 'bearing')):
                    out[f'{B.NAMES[i]}_{tag}/{k}'] = B.place3d(s, i)
    return out


def check():
    import wheel_motor as W
    VAL.mkdir(parents=True, exist_ok=True)
    arm_cloud()
    print('building the robot ...', flush=True)
    wld = world()
    d435_fov = DR.fov_envelopes()
    region = bx(140, 240, -80, 80, 55, 200)
    near_w = {k: s for k, s in wld.items() if W.near(region, s, 0.0)}
    print(f'{len(near_w)} parts near the nose', flush=True)
    rows = {}
    for cand in CANDIDATES:
        mine, info = place(cand)
        R = R_up(info['tilt_up_deg'])
        r = dict(info)
        gaps = {}
        for mk, ms in mine.items():
            g = {}
            for k, s in near_w.items():
                if mk == 'stand_imx662' and k == 'node_front':
                    continue                                          # it stands on it
                if W.near(ms, s, 8.0):
                    dmin = W.min_dist(ms, s)
                    if dmin < 8.0:
                        g[k] = round(dmin, 2)
                    if dmin < 1e-6 and W.overlap(ms, s) > 0.01:
                        g[k] = -round(W.overlap(ms, s), 2)           # overlap volume (mm3), negative
            gaps[mk] = dict(sorted(g.items(), key=lambda kv: kv[1])[:6])
        r['gaps_under_8mm'] = gaps
        r['stand_on_node'] = round(W.min_dist(mine['stand_imx662'], wld['node_front']), 2)
        r['top_z'] = round(max(s.bounding_box().max.Z for s in mine.values()), 1)
        r['lidar_margin_mm'] = round(SCAN_Z - r['top_z'], 1)
        rmax = 0.0
        for s in mine.values():
            v, _ = s.tessellate(0.3, 0.3)
            rmax = max(rmax, max(math.hypot(q.X, q.Y) for q in v))
        r['r_max'] = round(rmax, 1)
        r['inside_bumper'] = rmax <= R_BUMPER_IN
        np.random.seed(5)
        r['arm_mm'] = {k: dict(zip(('min', 'by'), DR.arm_to_solid(s))) for k, s in mine.items() if k in ('imx662', 'stand_imx662')}
        r['in_d435_fields_mm3'] = {f: round(sum(W.overlap(F, s) for s in mine.values() if W.near(F, s, 0.0)), 2) for f, F in d435_fov.items()}
        F = frustum(info['pupil'], R)
        hits = {}
        for k, s in near_w.items():
            if W.near(F, s, 0.0):
                v = W.overlap(F, s)
                if v > 0.01:
                    hits[k] = round(v, 2)
        r['robot_in_its_field_mm3'] = hits
        ok = all(v > 0 for g in gaps.values() for v in g.values()) and r['lidar_margin_mm'] >= LIDAR_MARGIN and r['inside_bumper'] and \
            min(a['min'] for a in r['arm_mm'].values()) >= 5.0 and not any(r['in_d435_fields_mm3'].values()) and not hits
        r['fits'] = bool(ok)
        rows[cand] = r
        print(cand, 'fits' if ok else 'NO', 'arm', {k: v['min'] for k, v in r['arm_mm'].items()}, 'lidar', r['lidar_margin_mm'],
              'r', r['r_max'], 'gaps', {k: min(v.values()) if v else None for k, v in gaps.items()}, 'fov hits', hits, flush=True)
    rep = dict(note='generic IMX662 USB 2 module (VERIFY), placeholder stand; nominal CAD; front pods at droop / nominal / bump; arm: the '
                    'SO-101 swept through d435_riser\'s poses and moves (validation/d435/arm_motion.json), kept round the nose',
               module=dict(board=BOARD, pcb=PCB_T, back=BACK, holder=HOLDER, lens=LENS, plug=PLUG, lead_d=LEAD_D, fov_deg=FOV),
               rules=dict(arm='>= 5 mm', lidar=f'top <= scan {SCAN_Z} - {LIDAR_MARGIN}', bumper=f'r <= {R_BUMPER_IN}',
                          d435='nothing in its depth or RGB field (500 mm)', own_field='no robot part in the IMX662 field (500 mm)'),
               candidates=rows, default=DEFAULT)
    (VAL / 'fit.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print('wrote', VAL / 'fit.json')
    export(wld)
    render(wld)


def export(wld):
    from step_colour import write_part, write_step
    write_part(module_local()['imx662'], (0.12, 0.35, 0.18), VAL / 'imx662_module.step', 'imx662_usb_generic')
    mine, _ = place(DEFAULT)
    front = {k: v for k, v in wld.items() if v.bounding_box().max.X > 120.0 and v.bounding_box().min.Z < 260
             and not k.startswith(('arm_', 'FL_droop', 'FL_bump', 'FR_droop', 'FR_bump'))}
    items = {k: (v, (0.75, 0.77, 0.8)) for k, v in front.items()}
    col = dict(imx662=(0.12, 0.35, 0.18), plug_imx662=(0.15, 0.15, 0.15), lead_imx662=(0.1, 0.1, 0.1), stand_imx662=(0.95, 0.6, 0.2))
    items.update({k: (v, col[k]) for k, v in mine.items()})
    write_step(items, VAL / 'imx662_fit_review.step', 'imx662_fit_v13')
    print('wrote', VAL / 'imx662_fit_review.step', len(items), 'parts')


def render(wld):
    import pod_lean as L
    mine, info = place(DEFAULT)
    keep = {k: v for k, v in wld.items() if v.bounding_box().max.X > 90.0 and v.bounding_box().min.Z < 240
            and not k.startswith(('FL_droop', 'FL_bump', 'FR_droop', 'FR_bump', 'env_'))}
    items = dict(keep, **mine)
    hi = dict(imx662=(0.12, 0.6, 0.25), plug_imx662=(0.15, 0.15, 0.15), lead_imx662=(0.1, 0.1, 0.1), stand_imx662=(0.95, 0.55, 0.15))
    cf = lambda k: hi.get(k, (0.15, 0.15, 0.18) if k in ('d435',) or 'lidar' in k.lower() else (0.78, 0.8, 0.84))   # noqa: E731
    t = f'IMX662 (green) at the nose under the lidar, {info["tilt_up_deg"]:g} deg up, lens front {info["lens_front"]}; placeholder stand orange'
    R = R_up(info['tilt_up_deg'])
    p = np.asarray(info['pupil'])

    def fields(ax, Rm):
        """side view: the IMX662's vertical field (green), the riser D435's depth field (blue), the lidar's scan plane (red)"""
        def line(a, b_, **kw):
            A, B_ = Rm @ np.asarray(a, float), Rm @ np.asarray(b_, float)
            ax.plot([A[0], B_[0]], [A[2], B_[2]], **kw)
        for s in (-1, 1):
            v = R @ np.array([1.0, 0.0, s * math.tan(math.radians(FOV[1] / 2))])
            line(p, p + v / np.linalg.norm(v) * 260, color='g', lw=1.2)
        c, tilt, _ = DR.POSE
        Rd = DR.cam_R(tilt, 0.0)
        o = np.asarray(c) + Rd @ np.array([DR.DEPTH_LENS[0], 0.0, 0.0])
        for s in (-1, 1):
            v = Rd @ np.array([1.0, 0.0, s * math.tan(math.radians(29.0))])
            line(o, o + v / np.linalg.norm(v) * 200, color='b', lw=1.0)
        line((60, 0, SCAN_Z), (330, 0, SCAN_Z), color='r', lw=0.8, ls='--')
    # from the right with the right front pod, bumper and fin hidden, so the camera shows
    side = {k: v for k, v in items.items() if not k.startswith(('FR_', 'bumper_q3', 'support_front_R', 'node_right'))}
    L.iso(side, VAL / 'side_right.png', 0, 0, t + '\nfrom the right, FR pod / bumper / fin hidden; green its field, blue the riser D435\'s, red dashed the scan plane',
          cf, tol=0.5, overlay=fields)
    L.iso(items, VAL / 'iso_front.png', -60, 20, t + ' (front right)', cf, tol=0.5)


if __name__ == '__main__':
    stage = sys.argv[1] if len(sys.argv) > 1 else 'check'
    if stage == 'cloud':
        P, lab, labels = arm_cloud()
        print(P.shape, P.min(0), P.max(0))
    else:
        check()
