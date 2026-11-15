"""cad/v14 627 fit coupon (Nov 13 2026, Fernando: the 627s in the fork suspension sit too loose in their Ø22.0 seats). As coupon_687.py.
Nov 13 evening (Fernando: compact, like the 687 coupon): the solid 52.9 x 52.9 x 18.2 block (35 cm3) became a 7.6 mm base plate with a 45 deg
cone shell over each pocket (about half the plastic). build123d; bench print, not part of the robot.

    python -B coupon_627.py [build|check|export|render|all]        (omni-robot-sim interpreter)

One 2 x 2 block, each cell numbered on the block's side face next to it (engraved). Cell n has:
 * a 627 pocket, Ø C627_POCKET[n], through the 7.6 base plate from the bed side (as the fork's seats print: axis vertical), under a 45 deg cone
   roof (no support). Set the bearing on a flat bench and press the block down over it: it stops flush with the face. 2.5 mm of PLA between
   pockets and to the faces, as the fork's lobes leave round their 627s;
 * over the pocket only a cone shell: the roof inside and a 45 deg cone outside, ~3 mm thick, closing at the top on the shoulder's Ø10;
 * the pin on top for the inner race, as the inner pivot's printed pin: a Ø10 x 0.8 shoulder on the inner race, a Ø C627_PIN[n] x 8 pin
   with a 0.3 lead-in, and the Ø2.8 bore through (the pin's M3 pilot; the hole runs through the whole block). Press a bearing on to the shoulder;
 * two Ø1.6 push-out holes from the cone onto the pocket bearing's outer ring, wholly inside the pocket: push the bearing out with a pin.
Cell 1 is 0.05 under the fork's 22.00 seat; each cell after it is 0.05 tighter on the pocket and 0.05 larger on the pin (21.80 / 7.20 in cell 4).
Pick the tightest pocket that takes the bearing without cracking and the pin that needs a firm press and holds; put them into the fork's 627
seats (pod_lean: B627_OD seat) and pod_lean.PIN_D / SLEEVE_D. Print in the fork's filament and settings, cells' axes vertical.
Dimensions: dims.C627_*. Nominal CAD; not printed.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import palette as PAL                                                  # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'coupon_627'
VAL = W.ROOT / 'validation' / 'coupon_627'
RP_MAX = max(C627_POCKET) / 2
PITCH = 2 * RP_MAX + C627_WALL
HALF = PITCH / 2 + RP_MAX + C627_WALL
BASE = C627_POCKET_D                                                    # base plate top = pocket ceiling
TOP = BASE + RP_MAX - C627_BORE / 2 + C627_CAP                          # cone shells' top (the largest pocket's roof closes C627_CAP under it)
R_OUT = C627_SHOULDER_D / 2 + (TOP - BASE)                              # outer 45 deg cone: r at the base top (it ends at the shoulder's r at TOP)
CORNER_R = 4.0
CELLS = {1: (-PITCH / 2, PITCH / 2), 2: (PITCH / 2, PITCH / 2), 3: (-PITCH / 2, -PITCH / 2), 4: (PITCH / 2, -PITCH / 2)}
PIN_TOP = TOP + C627_SHOULDER + C627_PIN_LEN
B627_OD, B627_W, B627_BORE = 22.0, 7.0, 7.0
LABEL_Z = BASE / 2


def outward(n):
    cx, cy = CELLS[n]
    return np.array([math.copysign(1, cx), math.copysign(1, cy)]) / math.sqrt(2)


def cone(r0, r1, z0, z1):
    return b.Cone(r0, r1, z1 - z0, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN)).moved(b.Location((0, 0, z0)))


def label(n):
    """the cell number engraved in the side face (y = +-HALF) next to the cell, reading upright from outside"""
    cx, cy = CELLS[n]
    s = 1 if cy > 0 else -1
    pl = b.Plane(origin=(cx, s * (HALF + 0.1), LABEL_Z), x_dir=(-s, 0, 0), z_dir=(0, s, 0))
    txt = pl * b.Text(str(n), font_size=C687_LABEL[0], align=(b.Align.CENTER, b.Align.CENTER))
    return b.extrude(txt, amount=C687_LABEL[1] + 0.1, dir=(0, -s, 0))


def build():
    outline = b.Box(2 * HALF, 2 * HALF, PIN_TOP + 2, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN))
    outline = b.fillet([e for e in outline.edges() if e.geom_type.name == 'LINE' and abs(e.position_at(1).Z - e.position_at(0).Z) > 1], CORNER_R)
    blk = outline & b.Box(4 * HALF, 4 * HALF, BASE, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN))
    shells = None
    for cx, cy in CELLS.values():
        c = cone(R_OUT, C627_SHOULDER_D / 2, BASE - 0.01, TOP).moved(b.Location((cx, cy, 0)))
        shells = c if shells is None else shells + c
    blk = blk + (shells & outline)
    for n, (cx, cy) in CELLS.items():
        rp, rpin = C627_POCKET[n - 1] / 2, C627_PIN[n - 1] / 2
        L = b.Location((cx, cy, 0))
        pin = W.zcyl(C627_SHOULDER_D / 2, TOP - 0.01, TOP + C627_SHOULDER) + W.zcyl(rpin, TOP + C627_SHOULDER - 0.01, PIN_TOP)
        top = [e for e in pin.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().Z - PIN_TOP) < 1e-3]
        blk = blk + b.chamfer(top, 0.3).moved(L)
        roof = cone(rp, C627_BORE / 2, BASE, BASE + rp - C627_BORE / 2)
        blk = blk - (W.zcyl(rp, -1, BASE + 0.01) + roof + W.zcyl(C627_BORE / 2, -1, PIN_TOP + 1)).moved(L)
        u = outward(n)
        for s in (1, -1):
            px, py = np.array([cx, cy]) + s * C627_PUSH_R * np.array([-u[1], u[0]])
            blk = blk - W.zcyl(C627_PUSH_D / 2, B627_W, TOP).moved(b.Location((px, py, 0)))
        blk = blk - label(n)
    blk.label = 'coupon_627'
    return {'coupon_627': blk}


def bearing_627():
    return W.zcyl(B627_OD / 2, -B627_W / 2, B627_W / 2) - W.zcyl(B627_BORE / 2, -B627_W, B627_W)


def bearing_at(n, where):
    cx, cy = CELLS[n]
    z = B627_W / 2 if where == 'pocket' else TOP + C627_SHOULDER + B627_W / 2
    return bearing_627().moved(b.Location((cx, cy, z)))


def check(shapes):
    c = shapes['coupon_627']
    bb = c.bounding_box()
    rep = dict(part=dict(solids=len(c.solids()), valid=bool(c.is_valid), volume_cm3=round(c.volume / 1000, 2), bbox=[round(v, 2) for v in bb.size],
                         base_plate=BASE, shell_top=round(TOP, 2), over_pins=round(PIN_TOP, 2)), cells={})
    for n, (cx, cy) in CELLS.items():
        rp = C627_POCKET[n - 1] / 2
        rp_meas = W.min_dist(b.Vertex(cx, cy, 0.5), c)
        rpin_meas = 8.0 - W.min_dist(b.Vertex(cx + 8.0, cy, TOP + C627_SHOULDER + 4.0), c)
        u = outward(n)
        push_ok = []
        for s in (1, -1):
            px, py = np.array([cx, cy]) + s * C627_PUSH_R * np.array([-u[1], u[0]])
            rod = W.zcyl(0.65, B627_W - 0.1, TOP + 2).moved(b.Location((px, py, 0)))
            push_ok.append(round(W.overlap(rod, c), 4) == 0 and W.overlap(rod, bearing_at(n, 'pocket')) > 0)
        rep['cells'][n] = dict(
            pocket_d=C627_POCKET[n - 1], pocket_d_measured=round(2 * rp_meas, 3), interference_d_on_22=round(B627_OD - C627_POCKET[n - 1], 3),
            pocket_bearing_overlap_mm3=round(W.overlap(bearing_at(n, 'pocket'), c), 3),
            pin_d=C627_PIN[n - 1], pin_d_measured=round(2 * rpin_meas, 3), interference_d_on_7=round(C627_PIN[n - 1] - B627_BORE, 3),
            pin_bearing_overlap_mm3=round(W.overlap(bearing_at(n, 'pin'), c), 3),
            wall_to_face=round(HALF - (abs(cx) + rp), 2), web_to_neighbour_min=round(PITCH - (C627_POCKET[n - 1] + max(C627_POCKET)) / 2, 2),
            shell_thickness_normal=round((R_OUT - rp) / math.sqrt(2), 2), solid_over_roof_apex=round(TOP - (BASE + rp - C627_BORE / 2), 2),
            push_hole_inside_pocket_by=round(rp - C627_PUSH_R - C627_PUSH_D / 2, 3), pin_wall=round((C627_PIN[n - 1] - C627_BORE) / 2, 2),
            push_out_holes_reach_race=all(push_ok), bore_through=not any(W._inside_pt(c, (cx, cy, z)) for z in np.arange(0.2, PIN_TOP, 0.5)),
            label_engraved_mm3=round((label(n) & b.Box(2 * HALF, 2 * HALF, BASE, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN))).volume, 2),
            label_left_in_part_mm3=round(W.overlap(label(n), c), 3))
    rep['bearings_on_pins_clear_each_other'] = all(W.overlap(bearing_at(i, 'pin'), bearing_at(j, 'pin')) == 0 for i in CELLS for j in CELLS if i < j)
    rep['fits_bed'] = max(bb.size.X, bb.size.Y) <= 256
    return rep


def export(shapes):
    OUT.mkdir(parents=True, exist_ok=True)
    rgb = PAL.colour('pod_fork_outer') or (0.33, 0.33, 0.33)
    write_part(shapes['coupon_627'], rgb, OUT / 'coupon_627.step', 'coupon_627')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    c = shapes['coupon_627']
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.6))
    for ax, row in zip(axs, ((1, 2), (3, 4))):
        y = CELLS[row[0]][1]
        items = [(c, (0.45, 0.45, 0.48))] + [(bearing_at(n, w), (0.85, 0.55, 0.15)) for n in row for w in ('pocket', 'pin')]
        polys = []
        for s, col in items:
            x = s.moved(b.Location((0, -y, 0))).rotate(b.Axis.X, -90) & b.Box(200, 200, 0.4)
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.02, 0.2)
            Vv = np.array([[q.X, q.Y] for q in v])
            polys += [(Vv[tri], col) for tri in np.array(t)]
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.set_xlim(-HALF - 2, HALF + 2)
        ax.set_ylim(-2, PIN_TOP + 2)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title('cells ' + ' and '.join(f'{n} (pocket Ø{C627_POCKET[n - 1]:.2f}, pin Ø{C627_PIN[n - 1]:.2f})' for n in row), fontsize=9)
        ax.set_xlabel('x (mm); bed at z = 0, 627s orange')
    fig.tight_layout()
    VAL.mkdir(parents=True, exist_ok=True)
    fig.savefig(VAL / 'coupon_627_section.png', dpi=110)
    plt.close(fig)
    # a shaded view from above the +y / -x corner
    a, e = np.radians(215), np.radians(30)
    Rm = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]]) @ np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    v, t = c.tessellate(0.05, 0.3)
    P3 = np.array([[q.X, q.Y, q.Z] for q in v]) @ Rm.T
    polys = []
    for tri in np.array(t):
        Q = P3[tri]
        n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
        nn = np.linalg.norm(n)
        if nn == 0 or n[1] > 0:
            continue
        polys.append((Q[:, 1].mean(), Q[:, [0, 2]], tuple(np.array((0.5, 0.5, 0.55)) * (0.45 + 0.55 * abs(n[1] / nn)))))
    polys.sort(key=lambda p: -p[0])
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors='none'))
    ax.autoscale()
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(f'coupon_627: {2 * HALF:.1f} x {2 * HALF:.1f}, base {BASE:g}, {PIN_TOP:.1f} over the pins, {c.volume / 1000:.1f} cm3', fontsize=9)
    fig.savefig(VAL / 'coupon_627_iso.png', dpi=100)
    plt.close(fig)


def main(stage):
    shapes = build()
    c = shapes['coupon_627']
    print('coupon_627', len(c.solids()), c.is_valid, round(c.volume / 1000, 2), [round(v, 2) for v in c.bounding_box().size], flush=True)
    if stage in ('check', 'all'):
        VAL.mkdir(parents=True, exist_ok=True)
        rep = check(shapes)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')
