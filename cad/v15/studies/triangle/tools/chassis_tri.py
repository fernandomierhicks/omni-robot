"""cad/v15 chassis (PROPOSAL, Nov 14 2026, Fernando): each pod (pod_p4.py: cad/v14 Pod_reimagined_4) bolts onto ONE printed part, a hollow
triangle per quadrant: two straight sides on the robot's midlines (shared with the neighbouring triangles), a curved outer side that carries
that quadrant's bumper, and the pod in the hollow, held on both sides. build123d + shapely, STEP review only; nothing printed.

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B chassis_tri.py [study|build|check|export|render|all]

Robot frame as v14 (origin on the floor under the centre, x forward, y left, z up); pods placed by base.py (layout F's rolling lines, tangent
to r 154; the slide below). The members are v14's depth (z 40..62, the pods' feet at 62).
 * Each triangle = its quadrant's sector, cut to the rim radius, made of members: the rim (the curved side, the bumper's support), a leg on
   each midline, a rail under each of the pod's side frames (run on to the legs / rim), the cross member under the pod's beam (behind the
   fork) and the wedge behind it to the side midline, a pad for the lidar pedestal (front pair) and pads under the fins' feet; less the
   pod's hollow (everything of the pod below the deck top, all poses, + CLR). Inside corners rounded in plan.
 * The pod sits on it: 4 x M3 x 16 up from below into each side frame (two ahead of the pivot, two behind) and 2 into the beam (pod_p4).
 * The four triangles meet on the midlines (joints: dovetail half-laps as v14's frame, not drawn here); the fins, the lidar pedestal and the
   second floor are v14's, unchanged in position: this script checks where they land and what they hit.
"""
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import Point, Polygon, box as sbox

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import base as B                                                       # noqa: E402
import first_floor as FF                                               # noqa: E402  (sets layout F: D 154, slide -20)
import chassis as C                                                    # noqa: E402
import pod_p4 as P4                                                    # noqa: E402
import pod_rx as R                                                     # noqa: E402
import hardware as H                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import M3_CLR                                                # noqa: E402
import palette as PAL                                                  # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

VAL = W.ROOT / 'validation' / 'chassis_tri'
OUT = W.ROOT / 'step_export' / 'chassis_tri'
SLIDE = -25.0                      # v14: -20. The paired pods meet pivot end to pivot end at the side midlines; P4's static parts reach 9 mm
B.SLIDE = SLIDE                    # further behind the pivot than pod_lean's even shortened (v15 pod: y 112, pod_lean 103)
Z0, Z1 = C.Z0, C.Z1                # 40, 62
CLR = 1.0                          # triangle to anything of the pod below the deck top (pod_p4's own deck stand-in: 1.0)
LEG_W, JOINT_GAP = 9.0, 0.2        # each triangle's leg on a midline (two legs side by side = an 18 mm member)
RIM_W = 14.0                       # the curved side
FILLET = 8.0                       # inside corners in plan (v14: FRAME_FILLET 8)
RIM_MARGIN = 1.5                   # rim radius past the farthest pod part or rail
BUMPER_T, BUMPER_Z = C.BUMPER_T, (Z0, 90.0)
CB_D, SEAT = 7.2, P4.DECK_SEAT     # pod screws: counterbored from below, 6 mm of member under each head
PED_PAD_R = C.PED_FLANGE_R + 2.0
FIN_PAD_R = 9.0
QUAD = {'FL': (0.0, 90.0), 'RL': (90.0, 180.0), 'RR': (180.0, 270.0), 'FR': (270.0, 360.0)}
COLOURS = dict(tri=(0.86, 0.86, 0.82), bumper=(0.90, 0.45, 0.15), pedestal=(0.55, 0.57, 0.60), support=(0.85, 0.85, 0.88))


def U(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def sector(a0, a1, r):
    return Polygon([(0, 0)] + [tuple(r * U(t)) for t in np.linspace(a0, a1, 64)])


# ---- the pod's footprints (pod frame) ----------------------------------------------------------------------------------------------------
def pod_footprints():
    """plan polygons, pod frame: moving (all poses, all heights), low (everything of the pod below the deck top: moving parts in every pose and
    the beam's lug), static (nominal), the frames' and beam's seats on the deck; cached against the pod's sources"""
    srcs = [TOOLS / n for n in ('pod_p4.py', 'pod_rx.py', 'pod.py', 'pod_lean.py', 'wheel_motor.py', 'dims.py')]
    key = [round(s.stat().st_mtime) for s in srcs]
    cache = VAL / 'pod_fp.pkl'
    if cache.exists():
        got = pickle.loads(cache.read_bytes())
        if got[0] == key:
            return got[1]
    parts, rm, rs, unit = pod_build()
    zt = P4.DECK['z'][1]                                             # -8: the deck top (robot z 62)
    mov, low, sta, seats = [], [], [], []
    exact = ('pod_fork', 'rx_frame', 'rx_beam')
    for pn, a in R.POSES.items():
        sh, moving, _ = P4.assembly(parts, rm, rs, unit, a)
        for k, s in sh.items():
            if k.startswith(('rx_deck', 'bolt_deck')):
                continue
            ex = k.startswith(exact)
            if k in moving:
                for lst, z0, z1 in ((mov, None, None), (low, -200.0, zt + 0.5)):
                    g = B.proj(s, ex, z0, z1)
                    if g is not None:
                        lst.append(g)
            elif pn == 'nominal':
                g = B.proj(s, ex)
                if g is not None:
                    sta.append(g)
                g = B.proj(s, ex, -200.0, zt - 0.05)
                if g is not None:
                    low.append(g)
                if k.startswith(('rx_frame', 'rx_beam')):
                    g = B.proj(s, True, zt - 0.01, zt + 0.3)
                    if g is not None:
                        seats.append(g)
    fp = dict(moving=union_all(mov), low=union_all(low), static=union_all(sta), seats=union_all(seats),
              screws=P4.deck_screw_xy(), rails=[P4.deck_rails()[0], P4.deck_rails()[1]], cross=(P4.DECK['cross_y'], P4.DECK['y'][1]))
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, fp)))
    return fp


def rmax(g):
    return max(math.hypot(*c) for q in getattr(g, 'geoms', [g]) for c in q.exterior.coords)


# ---- the triangles (plan) ----------------------------------------------------------------------------------------------------------------
def pod_members(fp, i, r_rim):
    """pod i's rails (run on along the pod's y to the legs / rim), its cross member and the wedge behind it, placed (robot plan)"""
    (ox0, ox1), (ix0, ix1) = fp['rails']
    cy0, cy1 = fp['cross']
    g = [sbox(ox0, -400, ox1, cy1), sbox(ix0, -400, ix1, cy1), sbox(ox0, cy0, ix1, 400)]
    return union_all([B.place2d(q, i) for q in g])


def layout(fp):
    """the rim radius and the four triangles' plans"""
    hollow = union_all([B.place2d(fp['low'], i) for i in range(4)]).buffer(CLR, 32)
    r_parts = max(rmax(B.place2d(union_all([fp['static'], fp['moving']]), i)) for i in range(4))
    (ox0, ox1), (ix0, ix1) = fp['rails']
    cy0, cy1 = fp['cross']
    under = union_all([sbox(ox0, P4.DECK['y'][0], ox1, cy1), sbox(ix0, P4.DECK['y'][0], ix1, cy1), sbox(ox0, cy0, ix1, cy1)])   # P4's deck
    rails = union_all([B.place2d(under, i) for i in range(4)])
    r_rim = math.ceil(max(r_parts, rmax(rails)) + RIM_MARGIN)
    tris, info = {}, {}
    for i, n in enumerate(B.NAMES):
        a0, a1 = QUAD[n]
        sec = sector(a0, a1, r_rim)
        legs = union_all([affinity.rotate(sbox(0, JOINT_GAP / 2, r_rim + 5, LEG_W), a0, origin=(0, 0)),
                          affinity.rotate(sbox(0, -LEG_W, r_rim + 5, -JOINT_GAP / 2), a1, origin=(0, 0))])
        rim = sec.difference(Point(0, 0).buffer(r_rim - RIM_W, 128))
        mem = [legs, rim, pod_members(fp, i, r_rim)]
        if n in ('FL', 'FR'):                                          # the lidar pedestal straddles the front midline
            mem.append(Point(C.LIDAR_X, 0).buffer(PED_PAD_R, 64))
        mem += [Point(*xy).buffer(FIN_PAD_R, 32) for xy in fin_feet()]
        g = union_all(mem).intersection(sec.buffer(-JOINT_GAP / 2, join_style=2).union(rim))
        g = g.intersection(Point(0, 0).buffer(r_rim, 256))
        g = g.buffer(FILLET, 32).buffer(-FILLET, 32)                   # inside corners rounded
        g = g.intersection(sec).difference(hollow)
        sliver = g.buffer(-2.0).buffer(2.0)                            # nothing under 4 mm wide
        g = sliver.buffer(1.0, 16).buffer(-1.0, 16).intersection(sec).difference(hollow)
        g = max(getattr(g, 'geoms', [g]), key=lambda q: q.area) if g.geom_type == 'MultiPolygon' else g
        tris[n] = g
        info[n] = dict(area_cm2=round(g.area / 100, 1))
    return tris, hollow, r_rim, info


_FIN_FEET = None


def fin_feet():
    """the fins' deck screws (v14's fins, unchanged)"""
    global _FIN_FEET
    if _FIN_FEET is None:
        _FIN_FEET = C.build_supports()[3]
    return _FIN_FEET


# ---- solids ------------------------------------------------------------------------------------------------------------------------------
def build(fp):
    tris, hollow, r_rim, info = layout(fp)
    parts, hw = {}, {}
    screws = []
    for i, n in enumerate(B.NAMES):
        for p in fp['screws']:
            screws.append((n, tuple(B.place2d(Point(*p), i).coords[0])))
    for n, g in tris.items():
        s = C.prism(g, Z0, Z1)
        try:
            s = b.fillet([e for e in s.edges() if all(abs(e.position_at(t).Z - Z1) < 1e-3 for t in (0, 0.5, 1))], 1.5)
            info[n]['top_edges'] = 'R1.5'
        except Exception:                                                  # noqa: BLE001
            info[n]['top_edges'] = 'sharp (refused)'
        parts[f'tri_{n}'] = s
    for k, (n, (x, y)) in enumerate(screws):                           # the pod's screws: up from below, counterbored, 6 mm of member under
        hole = W.cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) + W.cyl(CB_D / 2, Z1 - SEAT - Z0 + 0.1, (x, y, (Z0 + Z1 - SEAT) / 2 - 0.05))
        parts[f'tri_{n}'] = parts[f'tri_{n}'] - hole
        hw[f'bolt_pod_{n}_{k}'] = H.solid(dict(kind='pan', size=3.0, L=P4.DECK_SCREW_L, p=(x, y, Z1 - SEAT), d=(0, 0, 1.0), name='bolt_pod'))
    for k, (x, y) in enumerate(fin_feet()):                            # the fins' feet: M3 x 30 up through the member (v14's)
        n = owner(tris, x, y)
        if n:
            parts[f'tri_{n}'] = parts[f'tri_{n}'] - W.cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) - W.cyl(CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z0 + FF.CB_DEPTH / 2 - 0.05))
        hw[f'bolt_fin_{k}'] = C.pan_screw((x, y, Z0 + FF.CB_DEPTH), (0, 0, 1), C.FIN_SCREW_L)
    for k, a in enumerate((45, 135, 225, 315)):                        # the lidar pedestal's screws into the front pair
        x, y = np.array([C.LIDAR_X, 0.0]) + U(a) * C.PED_SCREW_R
        n = owner(tris, x, y)
        if n:
            parts[f'tri_{n}'] = parts[f'tri_{n}'] - W.cyl(1.4, 16.0, (x, y, Z1 - 8.0))
        hw[f'bolt_pedestal_{k}'] = C.pan_screw((x, y, Z1 + 4.0), (0, 0, -1), 14.0)
    rb = r_rim + 0.5
    for n in B.NAMES:                                                  # the bumper quarters: one per triangle, on its curved side
        a0, a1 = QUAD[n]
        ring = sector(a0 + 0.3, a1 - 0.3, rb + BUMPER_T).difference(Point(0, 0).buffer(rb, 256))
        parts[f'bumper_{n}'] = C.prism(ring, *BUMPER_Z)
    parts['pedestal'] = C.build_pedestal()
    sup, shw, sections, _, _ = C.build_supports()
    parts.update(sup)
    hw.update({k: v for k, v in shw.items() if not k.startswith('bolt_pin_')})   # v14's studs on pod_lean's column tops: no column now
    g = dict(tris=tris, hollow=hollow, r_rim=r_rim, rb=rb, info=info, screws=screws, sections=sections)
    return parts, hw, g


def owner(tris, x, y):
    return next((n for n, g in tris.items() if g.buffer(0.3).contains(Point(x, y))), None)


_POD = None


def pod_build():
    global _POD
    if _POD is None:
        parts, rm, rs, _ = P4.build()
        _POD = (parts, rm, rs, R.wheel_unit())
    return _POD


def pods_placed(a=0.0):
    """the four pods at fork angle a in robot coordinates: {name: (static dict, moving dict)}"""
    parts, rm, rs, unit = pod_build()
    out = {}
    for i, n in enumerate(B.NAMES):
        sh, moving, _ = P4.assembly(parts, rm, rs, unit, a)
        sh = {k: v for k, v in sh.items() if not k.startswith(('rx_deck', 'bolt_deck'))}
        st = {k: B.place3d(v, i) for k, v in sh.items() if k not in moving}
        mv = {k: B.place3d(v, i) for k, v in sh.items() if k in moving}
        out[n] = (st, mv)
    return out


# ---- checks --------------------------------------------------------------------------------------------------------------------------------
def compound(d, skip=()):
    return b.Compound([v for k, v in d.items() if not k.startswith(skip)])


def check(parts, hw, g, fp):
    tris = g['tris']
    rep = dict(layout=dict(D=B.D_WHEEL, slide=SLIDE, slide_v14=-20.0, r_rim=g['r_rim'], bumper_inside_r=g['rb'],
                           robot_diameter=round(2 * (g['rb'] + BUMPER_T), 1), robot_diameter_v14=450.3,
                           wheels={n: [round(float(v), 1) for v in B.frame(i)[0]] for i, n in enumerate(B.NAMES)}))
    mv_plan = [B.place2d(fp['moving'], i) for i in range(4)]
    st_plan = [B.place2d(fp['static'], i) for i in range(4)]
    rep['layout']['r_moving'] = round(max(rmax(q) for q in mv_plan), 1)
    rep['layout']['r_static'] = round(max(rmax(q) for q in st_plan), 1)
    rep['layout']['bumper_to_moving_mm'] = round(g['rb'] - rep['layout']['r_moving'], 1)
    rep['pods_plan_gaps_mm'] = dict(side_FL_RL=round(union_all([mv_plan[0], st_plan[0]]).distance(union_all([mv_plan[1], st_plan[1]])), 1),
                                    side_FR_RR=round(union_all([mv_plan[3], st_plan[3]]).distance(union_all([mv_plan[2], st_plan[2]])), 1),
                                    front_FL_FR=round(union_all([mv_plan[0], st_plan[0]]).distance(union_all([mv_plan[3], st_plan[3]])), 1),
                                    back_RL_RR=round(union_all([mv_plan[1], st_plan[1]]).distance(union_all([mv_plan[2], st_plan[2]])), 1))
    # the triangles: size, bed, clearance in plan to the pods' hollow, the seats and screws they carry
    tri = {}
    for i, n in enumerate(B.NAMES):
        t = tris[n]
        frames_seat = B.place2d(fp['seats'].intersection(sbox(-80, -200, -45, 200).union(sbox(50, -200, 80, 200))), i)
        beam_seat = B.place2d(fp['seats'].intersection(sbox(-45, 0, 50, 200)), i)
        own = [xy for nn, xy in g['screws'] if nn == n]
        land = [round(t.exterior.distance(Point(xy)) if t.contains(Point(xy)) else -t.distance(Point(xy)), 2) for xy in own]
        best = min(((ang, affinity.rotate(t, ang, origin=(0, 0)).bounds) for ang in range(0, 180, 3)), key=lambda q: max(q[1][2] - q[1][0], q[1][3] - q[1][1]))
        bb = best[1]
        tri[n] = dict(area_cm2=round(t.area / 100, 1), volume_cm3=round(parts[f'tri_{n}'].volume / 1000, 1), solids=len(parts[f'tri_{n}'].solids()),
                      valid=bool(parts[f'tri_{n}'].is_valid), bed_fit_mm=[round(bb[2] - bb[0], 1), round(bb[3] - bb[1], 1)], bed_turn_deg=best[0],
                      fits_256=bool(max(bb[2] - bb[0], bb[3] - bb[1]) <= 256.0),
                      frame_feet_supported_pct=round(100 * frames_seat.intersection(t).area / frames_seat.area, 1),
                      beam_underside_supported_pct=round(100 * beam_seat.intersection(t).area / beam_seat.area, 1),
                      pod_screws_edge_margin_mm=land, plan_gap_to_pod_hollow_mm=round(t.distance(union_all([B.place2d(fp['low'], k) for k in range(4)])), 2),
                      top_edges=g['info'][n].get('top_edges'))
    rep['triangles'] = tri
    # the fins and the pedestal against the pods (3D, every pose), the fins' feet on the triangles
    fins = {k: v for k, v in parts.items() if k.startswith('support_')}
    clash, near_ = {}, {}
    for pn, a in R.POSES.items():
        pods = pods_placed(a)
        for pod, (st, mv) in pods.items():
            for nm, sh in list(fins.items()) + [('pedestal', parts['pedestal'])]:
                for grp, d in (('static', st), ('moving', mv)):
                    cp = compound(d, ('roller_core', 'bearing_687', 'bolt_cart'))
                    if not W.near(sh, cp, 6.0):
                        continue
                    v = (sh & cp).volume if W.near(sh, cp, 0.0) else 0.0
                    dist = 0.0 if v > 0.01 else round(sh.distance_to(cp), 2)
                    key = f'{nm} / {pod} {grp}'
                    if v > 0.01:
                        clash[f'{key} ({pn})'] = round(v, 1)
                    near_[key] = min(near_.get(key, 99.0), dist)
    rep['fins_pedestal_vs_pods'] = dict(overlap_mm3=clash, min_gap_mm=near_)
    rep['fin_feet_on_triangles'] = {f'{k}': owner(tris, *xy) or 'NOT ON A TRIANGLE' for k, xy in enumerate(fin_feet())}
    rep['pedestal_on'] = sorted({owner(tris, *(np.array([C.LIDAR_X, 0.0]) + U(a) * C.PED_SCREW_R)) or 'none' for a in (45, 135, 225, 315)})
    top = max(compound(st).bounding_box().max.Z for st, _ in pods_placed(0.0).values())
    rep['pod_tops_vs_lidar_plane'] = dict(pod_top_z=round(top, 1), scan_plane_z=FF.LIDAR['plane'], floor2_under=C.FLOOR2_UNDER)
    return rep


# ---- outputs ---------------------------------------------------------------------------------------------------------------------------------
def render(parts, hw, g, fp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    VAL.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, show_pods in ((axs[0], False), (axs[1], True)):
        for n, t in g['tris'].items():
            for q in getattr(t, 'geoms', [t]):
                ax.fill(*q.exterior.xy, color=COLOURS['tri'], ec='k', lw=0.6)
                for h in q.interiors:
                    ax.fill(*h.xy, color='w', ec='k', lw=0.4)
            c = t.representative_point()
            ax.annotate(f'tri_{n}', (c.x, c.y), fontsize=9, ha='center')
        if show_pods:
            for i in range(4):
                for key, col, al in (('moving', (0.95, 0.85, 0.5), 0.55), ('static', (0.40, 0.60, 0.70), 0.75)):
                    q = B.place2d(fp[key], i)
                    for p in getattr(q, 'geoms', [q]):
                        ax.fill(*p.exterior.xy, color=col, alpha=al)
        for n, xy in g['screws']:
            ax.plot(*xy, 'm.', ms=4)
        for xy in fin_feet():
            ax.plot(*xy, 'b.', ms=5)
        for _, sec in g['sections']:
            for q in getattr(sec, 'geoms', [sec]):
                ax.fill(*q.exterior.xy, color='k')
        ax.add_patch(plt.Circle((0, 0), g['rb'] + BUMPER_T, fill=False, color=COLOURS['bumper'], lw=3))
        ax.add_patch(plt.Circle((0, 0), 225.15, fill=False, color='grey', ls='--', lw=0.8))
        ax.add_patch(plt.Circle((C.LIDAR_X, 0), C.PED_FLANGE_R, fill=False, color='k'))
        ax.axhline(0, lw=0.4, color='k')
        ax.axvline(0, lw=0.4, color='k')
        ax.set_aspect('equal')
        lim = g['rb'] + 20
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.grid(lw=0.3)
    axs[0].set_title(f'v15 chassis: four hollow triangles (front = +x), rim r {g["r_rim"]}, bumper Ø{2 * (g["rb"] + BUMPER_T):.0f} '
                     f'(dashed: v14 Ø450); magenta: pod screws, blue: fin feet, black: fins at the scan plane', fontsize=9)
    axs[1].set_title('with the P4 pods: teal static, yellow moving (swept)', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=80)
    plt.close(fig)
    pods = pods_placed(0.0)
    view = {k: v for k, v in parts.items() if not k.startswith('support_')}
    hide = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')
    for n, (st, mv) in pods.items():
        view.update({f'{n}/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide)})
    R.iso(view, VAL / 'iso_base.png', 215, 30, 'v15 base: hollow triangles, P4 pods, bumper, lidar pedestal (fins and floor hidden)', size=(14, 10))
    tri_only = {k: v for k, v in parts.items() if k.startswith(('tri_', 'bumper_', 'pedestal'))}
    tri_only.update({k: v for k, v in hw.items() if k.startswith('bolt_pod')})
    R.iso(tri_only, VAL / 'iso_triangles.png', 215, 35, 'v15 chassis: the four triangles, their bumper quarters, the pedestal; the pods\' screws', size=(14, 10))
    one = {k: v for k, v in parts.items() if k in ('tri_FL', 'bumper_FL')}
    st, mv = pods['FL']
    one.update({f'FL/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide)})
    R.iso(one, VAL / 'iso_FL_module.png', 200, 30, 'tri_FL with its pod and bumper quarter', size=(12, 9))
    R.iso(dict(parts, **{f'{n}/{k}': v for n, (st, mv) in pods.items() for k, v in {**st, **mv}.items() if not k.startswith(hide)}),
          VAL / 'iso_with_fins.png', 215, 25, 'v15 base with v14\'s second-floor fins (unchanged positions)', size=(14, 10))


def export(parts, hw):
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        if k.startswith('tri_'):
            write_part(s, PAL.WHITE, OUT / f'{k}.step', k)
    pods = pods_placed(0.0)
    items = {k: (v, PAL.WHITE if k.startswith(('tri_', 'pedestal', 'support_')) else PAL.ORANGE if k.startswith('bumper') else PAL.DARK_GRAY)
             for k, v in parts.items()}
    items.update({k: (v, H.MAGENTA) for k, v in hw.items()})
    for n, (st, mv) in pods.items():
        for k, v in {**st, **mv}.items():
            items[f'{n}_{k}'] = (H.step_model(k, v), H.MAGENTA if k.startswith(H.STEP_HARDWARE) or k.startswith('plunger') else R.colour(k))
    write_step(items, OUT / 'v15_base_assembly.step')
    return len(items)


def main(stage):
    fp = pod_footprints()
    if stage == 'study':
        tris, hollow, r_rim, info = layout(fp)
        print('r_rim', r_rim, info)
        return
    parts, hw, g = build(fp)
    for k, v in parts.items():
        if k.startswith('tri_'):
            print(k, len(v.solids()), v.is_valid, round(v.volume / 1000, 1), 'cm3', flush=True)
    if stage in ('check', 'all'):
        rep = check(parts, hw, g, fp)
        VAL.mkdir(parents=True, exist_ok=True)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1, default=str))
        print(json.dumps(rep['layout']), flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, g, fp)
        print('rendered', flush=True)
    if stage in ('export', 'all'):
        print('exported', export(parts, hw), flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'study')
