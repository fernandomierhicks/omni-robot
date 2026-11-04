"""cad/v11 compact robot base (PROPOSAL, Nov 3 2026): four lean pods (pod_lean.py) on a round printed plate with a slim bumper.
build123d, STEP review only. Nothing here is adopted; the second floor and the components come later.

    python -B base.py [study|build|check|export|render|all]        (omni-robot-sim interpreter)

Robot frame: origin on the floor under the centre, x forward, y left, z up; pod frame z + R_WHEEL = robot z.
 * X4 as the sim (robot_export.ARM): rolling lines tangent to r 146 at 45 / 135 / 225 / 315 deg, axles radial, motors inboard.
 * Opposite hands on opposite diagonals as the sim (pod_hands 'diagonal'): the drawn pod (pivot on hub +y) at 135 and 315 deg, its mirror
   image at 45 and 225. Every pivot points at the left or right side midline, so the pods meet in pairs there, pivot end to pivot end.
 * Each wheel slides SLIDE mm along its own rolling line, away from its pivot (toward the front / back). The rolling lines do not move, so
   the X4 kinematics (firmware, ROS) are unchanged; the support rectangle gets longer and narrower. The sim does not model the slide yet.
 * Plate: one flat 6 mm plate, top 62 mm above the floor (8 mm below the axles), cut where the pods pass through it (their swept volumes at
   droop / nominal / bump + 3 mm, and the drop-in path from above + 1 mm). Printed as four petals split on the diagonals: each pod's inner
   frame screws to both petals across the split, and an X spider under the centre ties the inner ends.
 * Bumper: a slim PETG ring, 4 quarters split on the midlines, each end on a tab screwed to the plate.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import MultiPoint, Point, Polygon, box as sbox
from shapely.geometry.polygon import orient

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

OUT = W.ROOT / 'step_export' / 'base'
VAL = W.ROOT / 'validation' / 'base'

# ---- layout ----------------------------------------------------------------------------------------------------------------------------
D_WHEEL = 146.0                    # rolling lines tangent to this circle (sim wheel_base_radius 0.146)
ANGLES = (45.0, 135.0, 225.0, 315.0)
NAMES = ('FL', 'RL', 'RR', 'FR')
SLIDE = -32.0                      # mm along each rolling line, away from the pivot (base.py study: R vs the side / front gaps)
CLR_MOVE, CLR_INSTALL = 3.0, 1.0   # plate cut round the swept moving parts / round the drop-in path from above
BUMPER_GAP, BUMPER_T = 3.0, 6.0    # bumper ring: inside face this far outside every moving part; wall
BUMPER_Z = (40.0, 90.0)            # robot z (v7's TPU bumper: 42..95)
TAB_T, TAB_REACH, TAB_DEG = 4.0, 16.0, 7.0   # bumper end tabs on the plate top: thickness, reach inward, angle
SPIDER_T, SPIDER_W = 8.0, 18.0     # X spider under the plate centre, along the petal splits
SPLIT_Y = sum(L.MOUNT_Y_L) / 2     # each split crosses its pod here (pod y), between the inner frame's two foot screws
PLATE_Z = (R_WHEEL + L.PLATE_BOTTOM_L, R_WHEEL + L.PLATE_TOP_L)
BED = 250.0
MOVING_SKIP = ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'bolt_pivot_outer', 'dowel_inner', 'washer_pivot_outer', 'spring_plunger', 'die_spring')
EXACT = ('pod_fork', 'pod_frame_inner', 'pod_keel_outer')     # non-convex parts: exact plan projection; the rest: convex hulls
COLOURS = {'plate': (0.80, 0.80, 0.78), 'spider': (0.55, 0.57, 0.60), 'bumper': (0.90, 0.45, 0.15)}


def colour(name):
    for k, v in COLOURS.items():
        if name.startswith(k):
            return v
    return L.colour(name.split('/')[-1])


def mirrored(i):
    """sim robot_model.fork_kinematics: the 45 and 225 deg pods are the mirror image (pivot on hub -y)"""
    return round(ANGLES[i]) % 180 == 45


def frame(i, slide=None):
    """pod frame -> robot frame: (axle xy, hub x (inward), hub y), and the mirror flag"""
    th = math.radians(ANGLES[i])
    c, s = math.cos(th), math.sin(th)
    sgn = -1.0 if mirrored(i) else 1.0
    sl = SLIDE if slide is None else slide
    A = np.array([D_WHEEL * c + sl * sgn * s, D_WHEEL * s - sl * sgn * c])
    return A, np.array([-c, -s]), np.array([s, -c]), mirrored(i)


def place2d(g, i, slide=None):
    A, xh, yh, m = frame(i, slide)
    if m:
        g = affinity.scale(g, 1, -1, origin=(0, 0))
    return affinity.affine_transform(g, [xh[0], yh[0], xh[1], yh[1], A[0], A[1]])


def place3d(shape, i, slide=None):
    A, xh, yh, m = frame(i, slide)
    if m:
        shape = shape.mirror(b.Plane.XZ)
    M = np.array([[xh[0], yh[0], 0, A[0]], [xh[1], yh[1], 0, A[1]], [0, 0, 1, R_WHEEL]])
    return shape.moved(W.loc(M))


# ---- plan projections (pod frame) ----------------------------------------------------------------------------------------------------
def proj(shape, exact, z0=None, z1=None, tol=0.4):
    s = shape if z0 is None else shape & P.box(-400, 400, -400, 400, z0, z1)
    if s is None or not s.faces():
        return None
    try:
        v, t = s.tessellate(tol, 0.3)
    except AttributeError:                       # a sliver the mesher cannot do: its bounding box (conservative)
        bb = s.bounding_box()
        return sbox(bb.min.X, bb.min.Y, bb.max.X, bb.max.Y)
    if not v:
        return None
    pts = np.array([[q.X, q.Y] for q in v])
    if not exact:
        h = MultiPoint(pts).convex_hull
        return h if h.area > 1e-3 else None
    tris = [Polygon(pts[tri]) for tri in np.array(t)]
    return union_all([q for q in tris if q.area > 1e-4]).buffer(0.02)


def footprints(parts, unit):
    """pod-frame plan polygons: moving (swept over the poses), static, the plate cutout and the plate-level hardware holes"""
    moving, static, motion, install = [], [], [], []
    for name, a in L.POSES.items():
        sh, _ = L.posed(parts, unit, a)
        for k, s in sh.items():
            ex = k.startswith(EXACT)
            mov = not k.startswith(MOVING_SKIP)
            if mov or name == 'nominal':
                g = proj(s, ex)
                if g is not None:
                    (moving if mov else static).append(g)
            if mov:
                g = proj(s, ex, L.PLATE_BOTTOM_L - 0.01, L.PLATE_TOP_L + 0.01)
                if g is not None:
                    motion.append(g.buffer(CLR_MOVE))
            if name == 'nominal':
                g = proj(s, ex, -200, L.PLATE_TOP_L - 0.05)
                if g is not None:
                    install.append(g.buffer(CLR_INSTALL))
    fp = dict(moving=union_all(moving), static=union_all(static), cutout=union_all(motion + install).simplify(0.2))
    fp['all'] = union_all([fp['moving'], fp['static']])
    return fp


def pod_screws():
    """pod-frame xy of the M3s that go into the plate (inner frame foot, outer keel foot)"""
    xc = (L.FI_X[0] + L.FI_X[1]) / 2
    return [(xc, y) for y in L.MOUNT_Y_L] + list(L.KEEL_SCREWS)


# ---- layout study ----------------------------------------------------------------------------------------------------------------------
def gaps(fp, slide):
    pl = [place2d(fp['all'], i, slide) for i in range(4)]
    mv = [place2d(fp['moving'], i, slide) for i in range(4)]
    allp = union_all(pl)
    r_all = max(math.hypot(*c) for g in allp.geoms for c in g.exterior.coords) if allp.geom_type == 'MultiPolygon' else \
        max(math.hypot(*c) for c in allp.exterior.coords)
    r_mov = max(math.hypot(*c) for g in mv for c in (g.exterior.coords if g.geom_type == 'Polygon' else [q for h in g.geoms for q in h.exterior.coords]))
    return dict(slide=slide, r_moving=round(r_mov, 1), r_all=round(r_all, 1), robot_dia=round(2 * (r_mov + BUMPER_GAP + BUMPER_T), 1),
                side_gap=round(pl[0].distance(pl[1]), 1), front_gap=round(pl[0].distance(pl[3]), 1),
                wheel_xy_FL=[round(v, 1) for v in frame(0, slide)[0]])


def study(fp):
    rows = [gaps(fp, s) for s in (0.0, -10.0, -15.0, -20.0, -24.0, -28.0, -32.0, -36.0, -40.0)]
    for r in rows:
        print(r, flush=True)
    return rows


# ---- plate, spider, bumper -----------------------------------------------------------------------------------------------------------
def r_bumper(fp):
    mv = union_all([place2d(fp['moving'], i) for i in range(4)])
    return max(math.hypot(*c) for g in getattr(mv, 'geoms', [mv]) for c in g.exterior.coords) + BUMPER_GAP


def face_of(poly):
    """shapely polygon (with holes) -> build123d face"""
    def wire(coords):
        pts = [b.Vector(x, y, 0) for x, y in list(coords)[:-1]]
        return b.Wire.make_polygon(pts, close=True)
    f = b.Face(wire(poly.exterior.coords), [wire(h.coords) for h in poly.interiors])
    return f


def solid_of(geom, z0, z1):
    out = []
    for g in getattr(geom, 'geoms', [geom]):
        if g.area > 1.0:
            g = orient(g.simplify(0.05), 1.0)        # counter-clockwise: the face looks up and extrudes z0 -> z1 (shapely's buffers are clockwise)
            s = b.extrude(face_of(g), amount=z1 - z0)
            bb = s.bounding_box()
            out.append(s.moved(b.Location((0, 0, z0 - bb.min.Z))))
    s = out[0]
    for o in out[1:]:
        s = s + o
    return s


def build_base(fp):
    rb = r_bumper(fp)
    r_plate = rb - 0.5
    cut = union_all([place2d(fp['cutout'], i) for i in range(4)])
    plate2d = Point(0, 0).buffer(r_plate, 256).difference(cut)
    # spider: along the diagonals from the centre to 6 mm short of the nearest cutout
    arms, r_sp = [], {}
    for a in ANGLES:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        r = next(r for r in np.arange(10, r_plate, 0.5) if cut.distance(Point(*(u * r))) < SPIDER_W / 2 + 6.0)
        r_sp[a] = float(r)
        arms.append(affinity.rotate(sbox(-SPIDER_W / 2, -SPIDER_W / 2, r, SPIDER_W / 2), a, origin=(0, 0)))
    spider2d = union_all(arms)
    # the petal splits: along each diagonal from the centre, then jogged sideways past the spider so that they cross the pod between
    # its inner frame's two foot screws (the diagonal itself crosses the pod at pod y = -SLIDE)
    split = {}
    for i, a in enumerate(ANGLES):
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        ypod = np.asarray(place2d(Point(0, 1), i).coords[0]) - np.asarray(place2d(Point(0, 0), i).coords[0])
        off = (-SLIDE - SPLIT_Y) * -ypod
        t = r_sp[a] - 2.0
        split[a] = [np.zeros(2), u * t, u * t + off, u * 1.5 * rb + off]
    holes = dict(pod_pilot=[], spider_clear=[], tab_pilot=[])
    for i in range(4):
        for p in pod_screws():
            holes['pod_pilot'].append(tuple(np.asarray(place2d(Point(*p), i).coords[0])))
    for a in ANGLES:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        n = np.array([-u[1], u[0]])
        for r in (16.0, r_sp[a] - 9.0):
            for s in (1, -1):
                holes['spider_clear'].append(tuple(u * r + s * 5.5 * n))
    tabs2d, tab_ok = [], {}
    for k in range(4):
        for side in (1, -1):
            a0 = 90.0 * k
            a1 = a0 + side * TAB_DEG
            sector = Polygon([(0, 0)] + [(1.5 * rb * math.cos(math.radians(t)), 1.5 * rb * math.sin(math.radians(t))) for t in np.linspace(min(a0, a1), max(a0, a1), 12)])
            tab = sector.intersection(Point(0, 0).buffer(rb + 0.01, 256).difference(Point(0, 0).buffer(r_plate - TAB_REACH, 256)))
            tabs2d.append((k, side, tab))
            am = math.radians(a0 + side * TAB_DEG / 2)
            p = ((r_plate - TAB_REACH / 2) * math.cos(am), (r_plate - TAB_REACH / 2) * math.sin(am))
            tab_ok[f'{int(a0)}{"+" if side > 0 else "-"}'] = round(plate2d.intersection(tab).area / tab.intersection(Point(0, 0).buffer(r_plate, 256)).area, 2)
            holes['tab_pilot'].append(p)
    # 3D
    plate = solid_of(plate2d, *PLATE_Z)
    for kind, d in (('pod_pilot', M3_PILOT), ('spider_clear', M3_CLR), ('tab_pilot', M3_PILOT)):
        for x, y in holes[kind]:
            plate = plate - cyl(d / 2, PLATE_Z[1] - PLATE_Z[0] + 0.2, (x, y, sum(PLATE_Z) / 2))
    petals = {}
    F = 1.5 * rb
    for k, nm in enumerate(('front', 'left', 'back', 'right')):
        lo, hi = split[(90.0 * k - 45) % 360], split[(90.0 * k + 45) % 360]
        arc_ = [(F * math.cos(math.radians(t)), F * math.sin(math.radians(t))) for t in np.linspace(90 * k - 40, 90 * k + 40, 14)]
        wedge2d = Polygon([tuple(p) for p in lo] + arc_ + [tuple(p) for p in hi[::-1]])
        assert wedge2d.is_valid, nm
        wedge = b.extrude(face_of(wedge2d), amount=200).moved(b.Location((0, 0, -50)))
        petals[f'plate_{nm}'] = plate & wedge
    spider = solid_of(spider2d, PLATE_Z[0] - SPIDER_T, PLATE_Z[0])
    for x, y in holes['spider_clear']:
        spider = spider - cyl(M3_PILOT / 2, SPIDER_T + 0.2, (x, y, PLATE_Z[0] - SPIDER_T / 2))
    ring = b.extrude(face_of(Point(0, 0).buffer(rb + BUMPER_T, 256).difference(Point(0, 0).buffer(rb, 256))), amount=BUMPER_Z[1] - BUMPER_Z[0]).moved(b.Location((0, 0, BUMPER_Z[0])))
    bumpers = {}
    for k in range(4):
        q = b.extrude(face_of(Polygon([(0, 0)] + [(1.5 * rb * math.cos(math.radians(t)), 1.5 * rb * math.sin(math.radians(t)))
                                                for t in np.linspace(90 * k, 90 * k + 90, 16)])), amount=400).moved(b.Location((0, 0, -100)))
        s = ring & q
        for kk, side, tab in tabs2d:
            if (kk == k and side > 0) or ((kk - 1) % 4 == k and side < 0) or (kk == 0 and k == 3 and side < 0):
                s = s + solid_of(tab, PLATE_Z[1], PLATE_Z[1] + TAB_T)
        for x, y in holes['tab_pilot']:
            s = s - cyl(M3_CLR / 2, 50, (x, y, PLATE_Z[1] + 10))
        bumpers[f'bumper_q{k}'] = s
    info = dict(r_bumper_inside=round(rb, 1), r_plate=round(r_plate, 1), robot_diameter=round(2 * (rb + BUMPER_T), 1),
                spider_reach=r_sp, tab_on_plate_fraction=tab_ok, holes={k: len(v) for k, v in holes.items()},
                plate_area_cm2=round(plate2d.area / 100, 1))
    return dict(petals=petals, spider=spider, bumpers=bumpers, plate2d=plate2d, cut=cut, spider2d=spider2d, holes=holes), info


cyl = W.cyl


# ---- checks ----------------------------------------------------------------------------------------------------------------------------------
def necks(base):
    """narrowest plate between neighbouring cutouts at the four midlines"""
    cuts = [place2d_cut for place2d_cut in base['cut'].geoms] if base['cut'].geom_type == 'MultiPolygon' else [base['cut']]
    out = {}
    pc = [g for g in cuts]
    for nm, ang in (('front', 0), ('left', 90), ('back', 180), ('right', 270)):
        u = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
        near_ = sorted(pc, key=lambda g: g.distance(Point(u[0] * 150, u[1] * 150)))[:2]
        out[nm] = round(near_[0].distance(near_[1]), 1) if len(near_) == 2 else None
    return out


def check(parts, unit, fp, base, info):
    rep = dict(layout=dict(d_wheel=D_WHEEL, slide=SLIDE, wheels={NAMES[i]: dict(angle=ANGLES[i], mirror_hand=mirrored(i),
                                                                          axle_xy=[round(v, 1) for v in frame(i)[0]]) for i in range(4)}),
               base=info, gaps_plan=gaps(fp, SLIDE), necks_between_cutouts=necks(base))
    pieces = dict(base['petals'], spider=base['spider'], **base['bumpers'])
    rep['pieces'] = {}
    for k, s in pieces.items():
        g = proj(s, False)
        rr = g.minimum_rotated_rectangle
        e = [math.dist(rr.exterior.coords[0], rr.exterior.coords[1]), math.dist(rr.exterior.coords[1], rr.exterior.coords[2])]
        bb = s.bounding_box()
        rep['pieces'][k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1),
                                footprint=[round(max(e), 1), round(min(e), 1)], height=round(bb.size.Z, 1),
                                bed_fit=bool(max(e) <= BED and min(e) <= BED) or bool(max(bb.size.X, bb.size.Y) <= BED))
    statics = list(base['petals'].values()) + [base['spider']] + list(base['bumpers'].values())
    res = {}
    for name, a in L.POSES.items():
        sh, _ = L.posed(parts, unit, a)
        ov, dmin = {}, {}
        for i in range(4):
            for k, s in sh.items():
                ps = place3d(s, i)
                for j, st in enumerate(statics):
                    if not W.near(ps, st, 6):
                        continue
                    v = W.overlap(ps, st)
                    if v > 0.01 and not (k.startswith(('pod_frame', 'pod_keel')) and j < 4):
                        ov[f'{NAMES[i]} {k} / piece {j}'] = round(v, 2)
                    if not k.startswith(MOVING_SKIP):
                        d = W.min_dist(ps, st)
                        key = 'plate' if j < 4 else ('spider' if j == 4 else 'bumper')
                        dmin[key] = min(dmin.get(key, 1e9), d)
        res[name] = dict(interference_mm3=ov, moving_to=({k: round(v, 2) for k, v in dmin.items()}))
        print(name, res[name], flush=True)
    rep['poses'] = res
    return rep


# ---- export / render ---------------------------------------------------------------------------------------------------------------------
def pod_items(parts, unit, a=0.0):
    sh, _ = L.posed(parts, unit, a)
    return {k: v for k, v in sh.items() if not k.startswith(L.STEP_SKIP)}


def export(parts, unit, base):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for k, s in dict(base['petals'], spider=base['spider'], **base['bumpers']).items():
        write_part(s, colour(k), OUT / 'parts' / f'{k}.step', k)
    items = {k: (s, colour(k)) for k, s in dict(base['petals'], spider=base['spider'], **base['bumpers']).items()}
    pod = pod_items(parts, unit)
    for i in range(4):
        for k, s in pod.items():
            items[f'{NAMES[i]}/{k}'] = (place3d(s, i), L.colour(k))
    write_step(items, OUT / 'base_assembly.step', 'robot_base')


def render(parts, unit, fp, base, info):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MPoly
    for f in VAL.glob('*.png'):
        f.unlink()

    def fill(ax, g, **kw):
        for q in getattr(g, 'geoms', [g]):
            ax.add_patch(MPoly(np.asarray(q.exterior.coords), closed=True, **kw))
            for h in q.interiors:
                ax.add_patch(MPoly(np.asarray(h.coords), closed=True, facecolor='white', edgecolor=kw.get('edgecolor', 'none'), lw=0.5))

    fig, ax = plt.subplots(figsize=(12, 12))
    rb = info['r_bumper_inside']
    ax.add_patch(Circle((0, 0), rb + BUMPER_T, facecolor=COLOURS['bumper'], edgecolor='none'))
    ax.add_patch(Circle((0, 0), rb, facecolor='white', edgecolor='none'))
    fill(ax, base['plate2d'], facecolor=COLOURS['plate'], edgecolor='0.4', lw=0.5)
    fill(ax, base['spider2d'], facecolor='none', edgecolor='0.3', lw=0.8, ls='--')
    for i in range(4):
        fill(ax, place2d(fp['moving'], i), facecolor=(0.95, 0.75, 0.15, 0.55), edgecolor=(0.6, 0.45, 0.0), lw=0.6)
        fill(ax, place2d(fp['static'], i), facecolor=(0.25, 0.68, 0.74, 0.85), edgecolor=(0.1, 0.35, 0.4), lw=0.6)
        A, xh, yh, m = frame(i)
        ax.plot(*A, 'k+', ms=12)
        piv = A + PIVOT_Y * yh * (-1 if m else 1)
        ax.plot(*piv, 'ko', ms=4)
        ax.annotate(f'{NAMES[i]} {"mirror" if m else "drawn"}', A + 0.35 * (A / np.linalg.norm(A)) * 60, ha='center', fontsize=10, weight='bold')
        t = np.array([-yh[1], yh[0]])
        L0 = A - yh * 230
        ax.plot(*zip(L0, A + yh * 230), color='0.6', lw=0.5, ls=':')
    for x, y in base['holes']['pod_pilot']:
        ax.plot(x, y, 'm.', ms=5)
    for x, y in base['holes']['tab_pilot']:
        ax.plot(x, y, 'm.', ms=5)
    for ang in (45, 135):
        ax.plot([-250 * math.cos(math.radians(ang)), 250 * math.cos(math.radians(ang))], [-250 * math.sin(math.radians(ang)), 250 * math.sin(math.radians(ang))], color='0.5', lw=0.6, ls='-.')
    ax.add_patch(Circle((0, 0), 225.0, facecolor='none', edgecolor='r', lw=0.8, ls='--'))
    ax.add_patch(Circle((0, 0), D_WHEEL, facecolor='none', edgecolor='0.6', lw=0.5, ls=':'))
    ax.annotate('front (+x)', (rb + 12, 0), rotation=-90, va='center', fontsize=10)
    ax.set_xlim(-250, 250)
    ax.set_ylim(-250, 250)
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    g = gaps(fp, SLIDE)
    ax.set_title(f'compact base (plan): bumper Ø{info["robot_diameter"]:.0f} (red dashed: v10 Ø450); slide {SLIDE:g} mm; side gap {g["side_gap"]}, front gap {g["front_gap"]} mm\n'
                 f'teal static pod parts, yellow moving (swept droop..bump), grey plate (4 petals split on the dash-dot diagonals, spider dashed), + axles, o pivots', fontsize=10)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=100)
    plt.close(fig)
    items = dict(base['petals'], spider=base['spider'], **base['bumpers'])
    pod = {k: v for k, v in pod_items(parts, unit).items() if not k.startswith('roller_core')}
    for i in range(4):
        for k, s in pod.items():
            items[f'{NAMES[i]}/{k}'] = place3d(s, i)
    L.iso(items, VAL / 'iso_front_left.png', 225, 28, 'compact base, front-left (nominal)', colour, tol=0.8)
    L.iso({k: v for k, v in items.items() if not k.startswith('bumper')}, VAL / 'iso_rear_right.png', 45, 35, 'compact base, rear-right, bumper hidden', colour, tol=0.8)
    L.iso(items, VAL / 'iso_below.png', 200, -30, 'compact base from below: plate petals, spider, fork base bars', colour, tol=0.8)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    fp = footprints(parts, unit)
    print('pod footprint bounds: moving', [round(v, 1) for v in fp['moving'].bounds], 'all', [round(v, 1) for v in fp['all'].bounds], flush=True)
    if stage in ('study', 'all'):
        rows = study(fp)
        (VAL / 'study.json').write_text(json.dumps(rows, indent=1), encoding='utf-8')
        if stage == 'study':
            return
    base, info = build_base(fp)
    print(json.dumps(info), flush=True)
    print('necks', necks(base), flush=True)
    if stage in ('check', 'all'):
        rep = check(parts, unit, fp, base, info)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k != 'poses'}, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, unit, base)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, unit, fp, base, info)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
