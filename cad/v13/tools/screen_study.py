"""cad/v11: can the 5 in face screen stand in front of the SO-101 without blocking it? (Nov 4 2026, layout F, STEP-only proposal)

    python -B screen_study.py            (omni-robot-sim interpreter; writes validation/screen/screen_study.json + png)

The arm's swept volume in front of its base, from omni_arm's own kinematics, capsules and planner (the ROS-free core, current ARM config:
arm base at (0.14, 0, 0.2185) as layout F). Swept: the fixed poses (stow, high, ready), and for floor targets over the fingers-down reach
(x 0.26..0.36 ahead, y -0.12..0.12; the base brings things there) the tidy's moves: high -> ready -> hover (8 cm over a 3 cm toy) -> grasp
-> hover -> high, every move as the planner gives it (joint-space lines sampled every 0.02 rad). The bin / face keep-outs of the old layout
are left out (layout F has neither in front). Each capsule is sampled densely; the screen must stay SCREEN_MARGIN under the lowest capsule
surface over its plan area. Then the wrist camera's view of each target and the D435's are checked against the screen box.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'ros' / 'src' / 'omni_arm'))
from omni_arm.kinematics import ArmConfig, ArmKinematics, DOWN       # noqa: E402
import omni_arm.motion as mo                                          # noqa: E402

VAL = HERE.parent / 'validation' / 'screen'
SCREEN_MARGIN = 0.010           # m: screen top below the arm's lowest capsule surface over it
FLOOR2_TOP = 0.1815             # second floor top (layout F)
ROBOT_R = 0.2253                # bumper outer radius (chassis.py, round 10: robot Ø450.6)
# candidate screen: Waveshare-class 5 in 800 x 480 HDMI panel, 121 x 76 x ~12 with its board (VERIFY on the bought unit), facing forward
SCREEN = dict(w=0.121, h=0.076, t=0.012)
TOY_H = 0.03


def old_layout_config():
    cfg = ArmConfig()
    cfg.keep_out = [k for k in cfg.keep_out if not any(w in k['note'] for w in ('bin', 'face', 'upright', 'audio', 'camera', 'cheek'))]
    return cfg


def samples(kin, q, n=9):
    """capsule surface samples: (points, radii)"""
    P, R = [], []
    for a, b, r in kin.capsules(q):
        s = np.linspace(0, 1, n)
        P.append(a + np.outer(s, b - a))
        R.append(np.full(n, r))
    return np.vstack(P), np.concatenate(R)


def path_qs(path, step=0.02):
    out = []
    for q0, q1 in zip(path[:-1], path[1:]):
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(2, int(math.ceil(np.max(np.abs(q1[:5] - q0[:5])) / step)) + 1)
        out += [q0 + s * (q1 - q0) for s in np.linspace(0, 1, n)]
    return out


def sweep():
    cfg = old_layout_config()
    kin = ArmKinematics(cfg)
    P = cfg.poses
    stow, high = P['stow'].copy(), P['high'].copy()
    ready = kin.ik([0.28, 0.0, 0.16], DOWN, yaw=math.pi / 2, seed=np.array([0, -0.56, 0.59, 1.54, 0, 0]))
    qs, failed, done = [stow, high, ready], [], 0
    p = mo.plan_checked(kin, stow, high)[0]
    qs += path_qs(p) if p else []
    for x in np.arange(0.26, 0.3601, 0.02):
        for y in np.arange(-0.12, 0.1201, 0.03):
            for yaw in (math.pi / 2, 0.0):
                tool = np.array([x, y, 0.012])
                hover = tool + np.array([0, 0, TOY_H + 0.08])
                qh = kin.ik(hover, DOWN, yaw=yaw, seed=ready, flip_ok=True)
                qg = kin.ik(tool, DOWN, yaw=yaw, seed=qh if qh is not None else ready, flip_ok=True) if qh is not None else None
                if qh is None or qg is None:
                    failed.append([round(x, 3), round(y, 3), round(yaw, 2), 'ik'])
                    continue
                legs = [(high, ready), (ready, qh), (qh, qg), (qg, qh), (qh, high)]
                ok = True
                for a, b in legs:
                    path, why = mo.plan_checked(kin, a, b, via=[high, ready])
                    if path is None:
                        failed.append([round(x, 3), round(y, 3), round(yaw, 2), why])
                        ok = False
                        break
                    qs += path_qs(path)
                done += ok
    return kin, qs, done, failed, dict(stow=stow, high=high, ready=ready)


def ceiling(kin, qs, xs, ys):
    """lowest capsule surface z over each (x, y) cell (cells 5 mm), over every swept q"""
    zc = np.full((len(xs), len(ys)), np.inf)
    h = 0.0025
    for q in qs:
        Pt, R = samples(kin, q)
        for (px, py, pz), r in zip(Pt, R):
            ix = np.where(np.abs(xs - px) <= r + h)[0]
            iy = np.where(np.abs(ys - py) <= r + h)[0]
            if ix.size and iy.size:
                sub = zc[np.ix_(ix, iy)]
                np.minimum(sub, pz - r, out=sub)
                zc[np.ix_(ix, iy)] = sub
    return zc


V10 = ROOT / 'cad' / 'v10 human editable'


def arm_parts():
    """the SO-101's official parts per kinematic body: [(body, product, 4 x 4 part-in-body (mm))] from cad/v10's link table (its body frames
    are the planner's: the same joint offsets; only the whole arm sits 13 mm lower in the sim, which this relative form removes)"""
    out = []
    for bd in json.loads((V10 / 'reference' / 'so101_links.json').read_text()):
        Fi = np.linalg.inv(np.array(bd['frame'], float))
        for p in bd['parts']:
            out.append((bd['body'], p['product'], Fi @ np.array(p['matrix'], float)))
    return out


def part_poses(kin, q, parts=None):
    """{index: 4 x 4 (mm) in the robot frame} of each official part at joint angles q"""
    fk = kin.fk(q)
    out = []
    for body, prod, M in parts or arm_parts():
        T = fk[body].copy()
        T[:3, 3] *= 1000.0
        out.append((prod, T @ M))
    return out


def mesh_points():
    """per part: its mesh vertices and triangle centroids (mm, part frame) from cad/v10's validation meshes"""
    pts = {}
    for _, prod, _ in arm_parts():
        if prod in pts:
            continue
        tri = np.load(V10 / 'validation' / 'fov_meshes' / f'{prod}.npz')['tri']
        pts[prod] = np.vstack([tri.reshape(-1, 3), tri.mean(axis=1)])
    return pts


def mesh_ceiling(kin, qs, xs, ys, skip_base=True):
    """as ceiling(), from the official meshes (cells 5 mm); the fixed base body (it stands on the riser behind x 0.195) is skipped"""
    parts = arm_parts()
    pts = mesh_points()
    zc = np.full((len(xs), len(ys)), np.inf)
    x0, y0, d = xs[0] * 1000, ys[0] * 1000, (xs[1] - xs[0]) * 1000
    for q in qs:
        for (body, prod, _), (_, T) in zip(parts, part_poses(kin, q, parts)):
            if skip_base and body == 'arm_base':
                continue
            P = pts[prod] @ T[:3, :3].T + T[:3, 3]
            ix = np.round((P[:, 0] - x0) / d).astype(int)
            iy = np.round((P[:, 1] - y0) / d).astype(int)
            m = (ix >= 0) & (ix < len(xs)) & (iy >= 0) & (iy < len(ys))
            np.minimum.at(zc, (ix[m], iy[m]), P[m, 2] / 1000.0)
    return zc


def main():
    VAL.mkdir(parents=True, exist_ok=True)
    kin, qs, done, failed, poses = sweep()
    xs = np.arange(0.10, ROBOT_R + 0.0001, 0.005)
    ys = np.arange(-0.10, 0.1001, 0.005)
    zc_caps = ceiling(kin, qs, xs, ys)
    near = []                                           # meshes only where the capsules come near the front of the second floor
    for q in qs:
        Pt, R = samples(kin, q, 5)
        if np.any((Pt[:, 0] > 0.16) & (Pt[:, 0] < 0.26) & (np.abs(Pt[:, 1]) < 0.11) & (Pt[:, 2] - R < 0.34)):
            near.append(q)
    zc_mesh = mesh_ceiling(kin, near, xs, ys)
    zc = np.minimum(zc_caps, zc_mesh)                   # the lower of the planner's capsules and the official meshes
    # the arm's own base and shoulder stand over x < ~0.19 (the base's footprint): the screen goes in front of whatever it reaches down to
    half = SCREEN['w'] / 2
    cols = np.abs(ys) <= half + 1e-9
    room = {}
    for i, x in enumerate(xs):
        z = float(np.min(zc[i, cols]))
        room[round(float(x), 3)] = round(z - SCREEN_MARGIN - FLOOR2_TOP, 4) if np.isfinite(z) else None
    # options: upright at the front edge, or leaning back (top toward the arm) at 20..60 deg from vertical: its front-bottom edge at the
    # bumper's inner face (x = ROBOT_R - 0.008), the top leaning in
    opts = []
    for lean in (0, 15, 30, 45, 60, 75):
        L = math.radians(lean)
        x_front = ROBOT_R - 0.008
        x_top = x_front - SCREEN['h'] * math.sin(L) - SCREEN['t'] * math.cos(L)
        z_top = FLOOR2_TOP + 0.004 + SCREEN['h'] * math.cos(L) + SCREEN['t'] * math.sin(L)
        # check every cell the panel covers in plan: its height there vs the ceiling
        worst = None
        for i, x in enumerate(xs):
            if x < x_top - 1e-9 or x > x_front + 1e-9:
                continue
            zp = FLOOR2_TOP + 0.004 + (x_front - x) / max(math.tan(L), 1e-9) if lean else z_top
            zp = min(zp, z_top)
            gap = float(np.min(zc[i, cols])) - zp
            worst = gap if worst is None else min(worst, gap)
        opts.append(dict(lean_deg=lean, plan_x=[round(x_top, 3), round(x_front, 3)], top_z=round(z_top, 3),
                         min_gap_to_arm_mm=None if worst is None else round(1000 * worst, 1)))
    lowest = {}
    for nm, z in (('capsules', zc_caps), ('meshes', zc_mesh)):
        sel = (xs >= ROBOT_R - 0.008 - SCREEN['t'] - 1e-9) & (xs <= ROBOT_R - 0.008 + 1e-9)
        lowest[nm] = round(float(np.min(z[np.ix_(sel, cols)])) * 1000, 1)
    rep = dict(arm_mount=list(kin.mount), swept_configurations=len(qs), floor_targets_done=done, floor_targets_failed=failed,
               screen=SCREEN, margin=SCREEN_MARGIN, room_above_floor2_by_x=room, options=opts,
               lowest_arm_z_over_upright_panel_mm=lowest,
               note='room: the screen top may reach this far above the second floor top (z 0.1815) over its full width (|y| <= 60.5 mm), at that x')
    (VAL / 'screen_study.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps({k: v for k, v in rep.items() if k != 'floor_targets_failed'}, indent=1))
    print('failed', len(failed), failed[:6])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(15, 6))
    im = axs[0].imshow(np.where(np.isfinite(zc), zc - FLOOR2_TOP, np.nan).T * 1000, origin='lower', extent=[xs[0] * 1000, xs[-1] * 1000, ys[0] * 1000, ys[-1] * 1000],
                       cmap='viridis', vmin=0, vmax=150, aspect='equal')
    fig.colorbar(im, ax=axs[0], label='lowest arm surface above the second floor (mm)')
    axs[0].axhline(-half * 1000, color='w', lw=0.8)
    axs[0].axhline(half * 1000, color='w', lw=0.8)
    axs[0].set_xlabel('x (mm, robot forward)')
    axs[0].set_ylabel('y (mm)')
    axs[0].set_title("arm's sweep over the front of the second floor (stow, high, ready, 18 x 2 floor grasps)")
    # side view: every swept capsule sample with |y| < 60, and the screen options
    for q in qs[::3]:
        Pt, R = samples(kin, q, 5)
        m = np.abs(Pt[:, 1]) < half
        axs[1].scatter(Pt[m, 0] * 1000, Pt[m, 2] * 1000, s=(R[m] * 1000) ** 2 / 6, color=(0.2, 0.4, 0.8, 0.03), linewidths=0)
    axs[1].axhline(FLOOR2_TOP * 1000, color='k', lw=1)
    axs[1].axvline(ROBOT_R * 1000, color='orange', lw=1)
    for o in opts:
        L = math.radians(o['lean_deg'])
        xf = (ROBOT_R - 0.008) * 1000
        pts = np.array([[xf, FLOOR2_TOP * 1000 + 4], [xf - SCREEN['h'] * 1000 * math.sin(L), FLOOR2_TOP * 1000 + 4 + SCREEN['h'] * 1000 * math.cos(L)]])
        axs[1].plot(pts[:, 0], pts[:, 1], lw=2, label=f"{o['lean_deg']} deg: gap {o['min_gap_to_arm_mm']} mm")
    axs[1].set_xlim(0, 400)
    axs[1].set_ylim(0, 450)
    axs[1].set_aspect('equal')
    axs[1].legend(fontsize=7)
    axs[1].set_title('side view, |y| < 60 mm: arm capsules swept (blue) and the screen leaning back from the front edge')
    fig.tight_layout()
    fig.savefig(VAL / 'screen_study.png', dpi=90)


if __name__ == '__main__':
    main()
