"""cad/v15 MID-DECK chassis (PROPOSAL, Nov 14 2026, Fernando): the first floor moves up from z 40..62 to z 80..100, above the pods' pivots
(z 70) and under their spring towers. The pods (pod_mid.py) hang under it, bolted up from below; the towers stand on it. build123d + shapely,
STEP review only; nothing printed.

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B chassis_mid.py [study|build|check|render|export|all]

 * Layout F unchanged (rolling lines tangent to r 154, wheels slid 20 mm: v14's wheel positions, so the kinematics, the front gap and the
   arm's reach stay as they are). Robot frame as v14 (origin on the floor under the centre, x forward, y left, z up).
 * The deck: a disc, 20 mm deep, in four quadrant pieces split on the midlines (one pod per piece; joints: dovetail half-laps as v14's frame,
   not drawn), less each pod's opening (everything of the pod above the deck's underside, every pose, + 1 mm) and lightening pockets where
   nothing bears.
 * On it: the pods' towers, the lidar pedestal (now 32 mm: the lidar stays at its height), v14's second-floor fins with their feet on the deck
   (shorter: the floor above stays at z 176.5), and the bumper quarters hung from its rim.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import Point, Polygon, box as sbox

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import base as B                                                       # noqa: E402
import first_floor as FF                                               # noqa: E402  (layout F: D 154, slide -20)
import chassis as C                                                    # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import pod_rx as R                                                     # noqa: E402
import hardware as H                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import M3_CLR, M3_PILOT, R_WHEEL                             # noqa: E402
import palette as PAL                                                  # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

VAL = W.ROOT / 'validation' / 'chassis_mid'
OUT = W.ROOT / 'step_export' / 'chassis_mid'
Z0, Z1 = PM.DECK_Z[0] + R_WHEEL, PM.DECK_Z[1] + R_WHEEL               # 80, 100
C.Z1 = Z1                          # v14's fins and pedestal stand on the deck's top (were on the frame's, z 62)
C.SIDE['leg_shift'] = 0.0           # Nov 14 (Fernando): the side fins straight (v14 shifted their legs 4 mm sideways under z 120, off the rear
                                   # pod_lean's fork); their feet, and so the deck's holes for them, follow
C.LONG_BOLT_L = 90.0               # the front fins' long bolts end 8.5 mm into the deck (v14: M3 x 130 into the frame at z 62; an M3 x 90 is
                                   # longer than Fernando's longest M3 (50): to change when the fins are redrawn for the mid deck)
CLR = PM.CLR
JOINT_GAP, SEAM_KEEP = 0.2, 22.0   # quadrant seams on the midlines; no pocket within SEAM_KEEP of a seam (the laps go there)
# the joints (Nov 14): dovetail half-laps across each seam, as v14's frame. On every seam one piece is the LOWER one: its bottom half runs
# across the seam as a dovetail tab into a socket in the other piece's bottom half, whose top half covers it; 2 x M3 x 16 down through the
# top half into the tab. FL and RR are the lower pieces on all their seams, RL and FR drop onto them: the deck goes together from above
SEAMS = [(90.0, 'FL', 'RL'), (180.0, 'RR', 'RL'), (270.0, 'RR', 'FR'), (0.0, 'FL', 'FR')]    # (seam angle, lower piece, upper piece)
LAP_R = (48.0, 128.0, 205.0)      # nominal radii of the laps on each seam (moved up to +-30 to land on solid deck)
TAB = dict(neck=8.0, flare=11.0, depth=14.0, root=1.0, clr=0.1)     # half widths at the seam / at the far end, depth past the seam
ZL = 90.0                         # the half-lap line
LAP_SCREW_L, LAP_SCREW_DU = 16.0, 4.5
RIM_KEEP, OPEN_KEEP, SCREW_KEEP, SEAT_KEEP = 12.0, 10.0, 8.0, 3.0      # solid deck kept: rim band, round each opening, round screws, round seats
HUB_KEEP, POCKET_MIN, POCKET_R, FILLET = 30.0, 400.0, 6.0, 8.0
SLIVER_W = 14.0                    # Nov 14 (Fernando): strips narrower than this held at one end only (cantilevers that add no stiffness) go
BUMPER_T, BUMPER_Z = C.BUMPER_T, (40.0, 80.0)   # Nov 14 (Fernando): its top at the deck's underside (was 90, beside the rim), tabs under it
# the bumper's attachment (Nov 14, Fernando): tabs from the quarter's top inward under the deck's rim, each with an M3 x 16 pan head up through
# it (Ø3.4, counterbored in its underside) into a Ø2.8 pilot through the deck (vertical: along the deck's print axis). A 45 deg gusset under each
# tab; the quarter prints upside down, its top edge and the tabs on the bed (print_beds)
BTAB = dict(r_in=216.0, w=14.0, t=6.0, gusset=8.0, screw_r=222.5, screw_L=16.0, per_quarter=3, end_keep=4.0, min_sep=15.0, step=0.5)
BTAB_CLR_MOVING, BTAB_CLR_STATIC = 3.0, 1.5      # plan clearance of a tab (and its gusset) to the pods' swept moving parts / their static parts
CB_D = 7.2
PED_PAD_R = C.PED_FLANGE_R + 6.0
QUAD = {'FL': (0.0, 90.0), 'RL': (90.0, 180.0), 'RR': (180.0, 270.0), 'FR': (270.0, 360.0)}   # the bumper quarters (angles)
SEAM_O = (6.0, -10.0)              # Nov 14: the deck's seams cross here, not at the centre: the spine's feet (on y = 0) and the side fins' feet
                                   # (x -1.3) sat on the midline seams, half a hole in each piece (and SolidWorks imported one badly)


def piece_box(n, big=600.0):
    ox, oy = SEAM_O
    return dict(FL=sbox(ox, oy, big, big), RL=sbox(-big, oy, ox, big), RR=sbox(-big, -big, ox, oy), FR=sbox(ox, -big, big, oy))[n]
COLOURS = dict(deck=(0.86, 0.86, 0.82), bumper=(0.90, 0.45, 0.15))
UPPER = True                       # Nov 14 (Fernando): first the pod alone (False), then the fins and second floor's implications (True)


def U(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def sector(a0, a1, r):
    return Polygon([(0, 0)] + [tuple(r * U(t)) for t in np.linspace(a0, a1, 64)])


def rmax(g):
    return max(math.hypot(*c) for q in getattr(g, 'geoms', [g]) for c in q.exterior.coords)


_POD = None


def pod_build():
    """the pod (pod frame) and its footprints, once"""
    global _POD
    if _POD is None:
        parts, rm, rs, _ = PM.build()
        unit = R.wheel_unit()
        R.wheel_envelope(unit)                                         # primes the envelope footprints() uses when its cache is stale
        fp = PM.footprints(parts, rm, rs, unit)
        _POD = (parts, rm, rs, unit, fp)
    return _POD


def pods_placed(a=0.0):
    """robot coordinates at fork angle a: {pod: (static dict, moving dict)}; the towers and their screws count as the deck's"""
    parts, rm, rs, unit, _ = pod_build()
    out = {}
    for i, n in enumerate(B.NAMES):
        sh, moving, _ = PM.assembly(parts, rm, rs, unit, a, deck=False)
        st = {k: B.place3d(v, i) for k, v in sh.items() if k not in moving}
        mv = {k: B.place3d(v, i) for k, v in sh.items() if k in moving}
        out[n] = (st, mv)
    return out


_FINS = None


def fins():
    global _FINS
    if _FINS is None:
        _FINS = C.build_supports() if UPPER else ({}, {}, [], [], [])
    return _FINS


# ---- the deck (plan) -------------------------------------------------------------------------------------------------------------------------
def layout(fp):
    openings = union_all([B.place2d(PM.opening(fp), i) for i in range(4)])     # Nov 14: + the space between the arms (PM.between_arms)
    hold = union_all([B.place2d(union_all([fp['pod_static'], fp['towers']]), i) for i in range(4)])
    r_hold, r_open = rmax(hold), rmax(openings)
    r_deck = math.ceil(max(r_hold + 1.5, r_open + 6.0))
    disc = Point(0, 0).buffer(r_deck, 256)
    up = [tuple(B.place2d(Point(*p), i).coords[0]) for i in range(4) for p in fp['up_screws']]
    tw = [tuple(B.place2d(Point(*p), i).coords[0]) for i in range(4) for p in fp['tower_screws']]
    fix = [tuple(B.place2d(Point(*p), i).coords[0]) for i in range(4) for p in fp['tower_fix']]
    feet = fins()[3]
    seats = union_all([B.place2d(union_all([fp['seats'], fp['towers']]), i) for i in range(4)])
    keep = union_all([openings.buffer(OPEN_KEEP), disc.difference(Point(0, 0).buffer(r_deck - RIM_KEEP, 256)),
                      sbox(-r_deck, SEAM_O[1] - SEAM_KEEP, r_deck, SEAM_O[1] + SEAM_KEEP), sbox(SEAM_O[0] - SEAM_KEEP, -r_deck, SEAM_O[0] + SEAM_KEEP, r_deck),
                      Point(*SEAM_O).buffer(HUB_KEEP),
                      seats.buffer(SEAT_KEEP)] + ([Point(C.LIDAR_X, 0).buffer(PED_PAD_R)] if UPPER else [])
                     + [Point(p).buffer(SCREW_KEEP) for p in up + tw + fix + list(feet)])
    pockets = disc.difference(keep).buffer(-POCKET_R, 32).buffer(POCKET_R, 32)
    pockets = union_all([q for q in getattr(pockets, 'geoms', [pockets]) if q.area >= POCKET_MIN])
    decks, pruned = {}, {}
    protect = union_all([seats.buffer(SEAT_KEEP)] + [Point(p).buffer(SCREW_KEEP) for p in up + tw + fix + list(feet) + pedestal_xy()])
    for n in B.NAMES:
        sec = piece_box(n).buffer(-JOINT_GAP / 2, join_style=2)
        g = disc.intersection(sec).difference(openings).difference(pockets)
        g = g.buffer(FILLET, 32).buffer(-FILLET, 32).intersection(sec).intersection(disc).difference(openings)
        g = g.buffer(-1.5).buffer(1.5)                                 # nothing under 3 mm
        if g.geom_type == 'MultiPolygon':
            g = max(g.geoms, key=lambda q: q.area)
        g, cut = prune_cantilevers(g, protect, sec)
        pruned[n] = cut
        decks[n] = g
    laps, lower = make_laps(decks, up + tw + fix + list(feet) + pedestal_xy())
    return dict(decks=decks, lower=lower, laps=laps, openings=openings, pockets=pockets, pruned=pruned, r_deck=r_deck, r_hold=round(r_hold, 1), r_open=round(r_open, 1),
                up=up, tw=tw, fix=fix, feet=list(feet))


def prune_cantilevers(g, protect, sec, w=SLIVER_W):
    """remove the strips of g narrower than w that hang from the rest at one end only (they stiffen nothing), unless they carry a seat or a
    screw; the strips that join two wider regions (the rings round the openings, the rim) stay. Returns the plan and what was cut (mm2)"""
    cut = 0.0
    for _ in range(3):                                                 # a removed finger can leave another one behind
        core = g.buffer(-w / 2, 16).buffer(w / 2, 16).intersection(g)
        thin = g.difference(core.buffer(0.05))
        drop = []
        for c in getattr(thin, 'geoms', [thin]):
            if c.area < 1.0 or c.intersects(protect):
                continue
            touch = c.buffer(0.3).intersection(core)
            n_touch = len(getattr(touch, 'geoms', [touch])) if not touch.is_empty else 0
            if n_touch <= 1:
                drop.append(c)
        if not drop:
            break
        dd = union_all(drop)
        cut += dd.area
        g = g.difference(dd.buffer(0.1)).buffer(-0.5).buffer(0.5)
        if g.geom_type == 'MultiPolygon':
            g = max(g.geoms, key=lambda q: q.area)
    return g, round(cut, 1)


def pedestal_xy():
    return [tuple(np.array([C.LIDAR_X, 0.0]) + U(a) * C.PED_SCREW_R) for a in (45, 135, 225, 315)] if UPPER else []


def tab_poly(theta, side, u):
    """the dovetail tab at radius u on the seam at angle theta, reaching `side` (+-1: across the seam toward +-normal)"""
    T = TAB
    loc = Polygon([(u - T['neck'], -T['root']), (u + T['neck'], -T['root']), (u + T['flare'], T['depth']), (u - T['flare'], T['depth'])])
    loc = affinity.scale(loc, 1.0, side, origin=(0, 0))
    return affinity.translate(affinity.rotate(loc, theta, origin=(0, 0)), *SEAM_O)   # u from the seams' crossing


def make_laps(decks, avoid):
    """place the laps (first radius near each nominal where the tab and a margin round it are solid deck on both sides, clear of other
    screws); returns the laps and each piece's bottom-half plan"""
    lower = dict(decks)
    laps = []
    for theta, lo, hi in SEAMS:
        nrm, O = U(theta + 90.0), np.array(SEAM_O)
        rp = decks[hi].representative_point()
        side = 1.0 if rp.distance(Point(*(O + nrm * 50))) < rp.distance(Point(*(O - nrm * 50))) else -1.0
        both = decks[lo].union(decks[hi]).buffer(JOINT_GAP, join_style=2)
        for u0 in LAP_R:
            for du in (0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30):
                tab = tab_poly(theta, side, u0 + du)
                if not both.buffer(-3.0).contains(tab):
                    continue
                d = U(theta)
                screws = [tuple(O + d * (u0 + du + k * LAP_SCREW_DU) + nrm * side * TAB['depth'] * 0.5) for k in (-1, 1)]
                if any(Point(p).distance(Point(q)) < 9.0 for p in screws for q in avoid):
                    continue
                laps.append(dict(seam=theta, lower=lo, upper=hi, u=u0 + du, tab=tab, screws=screws))
                lower[lo] = lower[lo].union(tab)
                lower[hi] = lower[hi].difference(tab.buffer(TAB['clr'], join_style=2))
                break
    return laps, lower


def owner(decks, x, y):
    return next((n for n, g in decks.items() if g.buffer(0.3).contains(Point(x, y))), None)


# ---- accessory holes (Nov 14, Fernando): zip-tie anchors and spare M3 pilots through every deck piece -----------------------------------------
ACC_PITCH = 15.0                   # M3 pilots on a 15 mm square lattice (robot x / y) wherever the deck has room (the deck is mostly bands
ACC_WALL, ACC_KEEP = 2.6, 3.0      # 14-20 wide between its pockets: a sparse 30 mm grid as the second floor's found almost nothing). Walls:
ACC_SCREW_KEEP = 6.5               # ACC_WALL from a pilot or slot to the deck's edge (both halves), ACC_KEEP off seats, laps, fin feet and
ACC_TIE_STEP, ACC_TIE_GAP = 5.0, 22.0   # the pedestal, ACC_SCREW_KEEP round the screws. Tie pairs first: tried on a 5 mm lattice at 0/45/90/135
                                   # deg (cable direction), kept >= ACC_TIE_GAP apart (centres); pilots then skip anything within 4 mm of a pair


def accessory_holes(g, fp):
    """plan positions and cutters per piece: zip-tie pairs (two TIE_SLOT_L x TIE_SLOT_W slots TIE_PITCH apart across the cable: the tie goes
    down one, under the bridge, up the other) and Ø M3_PILOT pilots, all straight through the 20 mm deck (along the print axis)"""
    from dims import TIE_SLOT_L, TIE_SLOT_W, TIE_PITCH
    seats = union_all([B.place2d(union_all([fp['seats'], fp['towers']]), i) for i in range(4)])
    pts = g['up'] + g['tw'] + g['fix'] + g['feet'] + pedestal_xy() + [p for lap in g['laps'] for p in lap['screws']] + list(fins()[4])
    avoid = union_all([seats.buffer(ACC_KEEP), Point(C.LIDAR_X, 0).buffer(PED_PAD_R + ACC_KEEP)]
                      + [lap['tab'].buffer(ACC_KEEP + TAB['clr']) for lap in g['laps']] + [Point(p).buffer(ACC_SCREW_KEEP) for p in pts]
                      + [f.buffer(ACC_KEEP) for f in (B.proj(v, True, Z1 - 0.01, Z1 + 1.0) for v in fins()[0].values()) if f is not None]
                      + [tab_plan(ang, g['r_deck'] + 0.5).buffer(ACC_KEEP) for _, ang, _, _ in g.get('btabs', [])])   # under the bumper's tabs
    cutters, count = {}, {}
    slot = sbox(-TIE_SLOT_L / 2, -TIE_SLOT_W / 2, TIE_SLOT_L / 2, TIE_SLOT_W / 2)       # cable along local x; the slots across it
    for n, d in g['decks'].items():
        ok = d.intersection(g['lower'][n]).buffer(-ACC_WALL).difference(avoid)
        x0, y0, x1, y1 = d.bounds
        cut, ties, pils, taken = [], [], 0, []
        for x in np.arange(math.floor(x0 / ACC_TIE_STEP) * ACC_TIE_STEP, x1, ACC_TIE_STEP):    # tie pairs
            for y in np.arange(math.floor(y0 / ACC_TIE_STEP) * ACC_TIE_STEP, y1, ACC_TIE_STEP):
                if any(math.hypot(x - tx, y - ty) < ACC_TIE_GAP for tx, ty in ties):
                    continue
                for deg in (0.0, 90.0, 45.0, 135.0):
                    sl = [affinity.translate(affinity.rotate(affinity.translate(slot, 0.0, k * TIE_PITCH), deg, origin=(0, 0)), x, y) for k in (-0.5, 0.5)]
                    if all(ok.contains(s) for s in sl):
                        for s in sl:
                            cut.append(B.solid_of(s, Z0 - 1.0, Z1 + 1.0))
                        taken.append(union_all(sl).convex_hull)
                        ties.append((x, y))
                        break
        pil_pts = []
        for x in np.arange(math.floor(x0 / ACC_PITCH) * ACC_PITCH, x1, ACC_PITCH):              # pilots
            for y in np.arange(math.floor(y0 / ACC_PITCH) * ACC_PITCH, y1, ACC_PITCH):
                p = Point(x, y)
                if ok.contains(p.buffer(M3_PILOT / 2, 16)) and not any(t.distance(p) < 4.0 for t in taken):
                    cut.append(W.cyl(M3_PILOT / 2, Z1 - Z0 + 2.0, (float(x), float(y), (Z0 + Z1) / 2)))
                    pil_pts.append((float(x), float(y)))
        cutters[n], count[n] = cut, dict(tie_pairs=len(ties), m3_pilots=len(pil_pts))
    return dict(cutters=cutters, count=count)


# ---- the bumper's tabs (Nov 14) ------------------------------------------------------------------------------------------------------------
def tab_plan(ang, rb):
    """a tab's plan (with its gusset: the same footprint) at angle ang: r_in .. the bumper's inside, w wide"""
    T = BTAB
    p = sbox(T['r_in'], -T['w'] / 2, rb + 0.5, T['w'] / 2)
    return affinity.rotate(p, ang, origin=(0, 0))


def bumper_tabs(g, fp):
    """per quarter, up to BTAB['per_quarter'] tab angles where the tab's plan keeps clear of the pods (moving: swept, all heights; static), the
    fin feet's counterbores and the front fins' long bolts, and its screw lands on deck ACC_WALL + 1.5 inside a piece; near the quarter's
    1/6, 1/2, 5/6 points, >= min_sep deg apart. Returns [(quarter, angle, (x, y) screw, deck piece)]"""
    T, rb = BTAB, g['r_deck'] + 0.5
    obst = union_all([B.place2d(fp['moving'], i).buffer(BTAB_CLR_MOVING) for i in range(4)]
                     + [B.place2d(fp['pod_static'], i).buffer(BTAB_CLR_STATIC) for i in range(4)]
                     + [Point(p).buffer(CB_D / 2 + 2.0) for p in g['feet']] + [Point(p).buffer(3.0) for p in fins()[4]])
    out = []
    for n in B.NAMES:
        a0, a1 = QUAD[n]
        ok = []
        for ang in np.arange(a0 + T['end_keep'], a1 - T['end_keep'] + 1e-6, T['step']):
            if tab_plan(ang, rb).intersects(obst):
                continue
            sx, sy = T['screw_r'] * U(ang)
            piece = next((m for m, d in g['decks'].items() if d.buffer(-(M3_PILOT / 2 + ACC_WALL + 1.5)).contains(Point(sx, sy))), None)
            if piece:
                ok.append((float(ang), (float(sx), float(sy)), piece))
        chosen = []
        for f in (1 / 6, 1 / 2, 5 / 6):
            tgt = a0 + f * (a1 - a0)
            cand = [c for c in ok if all(abs(c[0] - q[0]) >= T['min_sep'] for q in chosen)]
            if cand and len(chosen) < T['per_quarter']:
                chosen.append(min(cand, key=lambda c: abs(c[0] - tgt)))
        out += [(n, a, xy, pc) for a, xy, pc in sorted(chosen)]
    return out


def bumper_quarter(n, tabs, rb):
    """a bumper quarter: the 3 mm wall (z BUMPER_Z) and its tabs (t thick under the deck, a 45 deg gusset under each), screw holes"""
    T = BTAB
    a0, a1 = QUAD[n]
    ring = sector(a0 + 0.3, a1 - 0.3, rb + BUMPER_T).difference(Point(0, 0).buffer(rb, 256))
    q = C.prism(ring, *BUMPER_Z)
    zt = BUMPER_Z[1]
    zb = zt - T['t']
    for qn, ang, (sx, sy), _ in tabs:
        if qn != n:
            continue
        sec = [(rb + 1.0, zb - T['gusset']), (rb + 1.0, zt), (T['r_in'], zt), (T['r_in'], zb), (rb - T['gusset'], zb)]   # radial section (r, z)
        tab = b.extrude(b.Plane.XZ * b.Polygon(*sec, align=None), amount=T['w'] / 2, both=True)
        tab = tab.rotate(b.Axis.Z, ang)
        try:
            tab = b.fillet([e for e in tab.edges() if e.center().Z > zb - 0.01 and math.hypot(e.center().X, e.center().Y) < rb - 0.5
                            and abs(e.position_at(1).Z - e.position_at(0).Z) > 1.0], 1.5)    # the tab's two inner vertical corners
        except Exception:                                              # noqa: BLE001
            pass
        q = q + tab
        q = q - W.cyl(M3_CLR / 2, T['t'] + 2.0, (sx, sy, zb + T['t'] / 2)) - W.cyl(CB_D / 2, 2.9 + 0.1, (sx, sy, zb + 2.9 / 2 - 0.05))
    return q.clean()


# ---- solids ------------------------------------------------------------------------------------------------------------------------------
def build(fp):
    g = layout(fp)
    decks = g['decks']
    parts, hw, info = {}, {}, {}
    for n, pl in decks.items():
        s = (C.prism(g['lower'][n], Z0, ZL) + C.prism(pl, ZL, Z1)).clean()
        top = [e for e in s.edges() if all(abs(e.position_at(t).Z - Z1) < 1e-3 for t in (0, 0.5, 1))]
        info[n] = 'top edges sharp (refused)'
        for r in (1.5, 1.0, 0.6):
            try:
                out = b.fillet(top, r)
                if out.is_valid and len(out.solids()) == 1:
                    s, info[n] = out, f'top edges R{r:g}'
                    break
            except Exception:                                              # noqa: BLE001
                pass
        else:
            try:
                out = b.chamfer(top, 0.8)
                if out.is_valid:
                    s, info[n] = out, 'top edges chamfer 0.8'
            except Exception:                                              # noqa: BLE001
                pass
        parts[f'deck_{n}'] = s
    for k, lap in enumerate(g['laps']):                                # the laps: down through the upper piece's top half into the tab
        for j, (x, y) in enumerate(lap['screws']):
            lo, hi = f'deck_{lap["lower"]}', f'deck_{lap["upper"]}'
            parts[hi] = parts[hi] - W.cyl(M3_CLR / 2, Z1 - ZL + 0.2, (x, y, (Z1 + ZL) / 2)) - W.cyl(CB_D / 2, 3.0, (x, y, Z1 - 1.45))
            parts[lo] = parts[lo] - W.cyl(M3_PILOT / 2, ZL - Z0 + 0.2, (x, y, (ZL + Z0) / 2))
            hw[f'bolt_lap_{k}_{j}'] = C.pan_screw((x, y, Z1 - 2.9), (0, 0, -1), LAP_SCREW_L)
    for k, (x, y) in enumerate(g['up']):                               # hangers and beams: the threads are in the deck (Ø2.8 through)
        n = owner(decks, x, y)
        if n:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - W.cyl(M3_PILOT / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))
    for x, y in g['fix']:                                              # the towers' short screws: Ø2.8 through the deck
        n = owner(decks, x, y)
        if n:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - W.cyl(M3_PILOT / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))
    for k, (x, y) in enumerate(g['tw']):                               # the hangers' long bolts pass up through the deck into the towers
        n = owner(decks, x, y)
        if n:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - W.cyl(M3_CLR / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))
    for k, (x, y) in enumerate(g['feet']):                             # the fins' feet: M3 up through the deck (counterbored below)
        hole = W.cyl(M3_CLR / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2)) + W.cyl(CB_D / 2, 3.0, (x, y, Z0 + 1.45))
        for n in decks:                                                # Nov 14 (SolidWorks interference): a foot on a seam needs its hole in
            if W.near(parts[f'deck_{n}'], hole, 0.0):                  # every piece it crosses (only the 'owner' was drilled)
                parts[f'deck_{n}'] = parts[f'deck_{n}'] - hole
        hw[f'bolt_fin_{k}'] = C.pan_screw((x, y, Z0 + 2.9), (0, 0, 1), 30.0)
    for x, y in fins()[4]:                                             # the front fins' long bolts down from the second floor: a pilot through
        for n in decks:                                                # the deck (Nov 14: they ran 28 mm out under it; now M3 x LONG_BOLT_L)
            pil = W.cyl(M3_PILOT / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))
            if W.near(parts[f'deck_{n}'], pil, 0.0):
                parts[f'deck_{n}'] = parts[f'deck_{n}'] - pil
    for k, a in enumerate((45, 135, 225, 315) if UPPER else ()):       # the pedestal's screws into the front pair
        x, y = np.array([C.LIDAR_X, 0.0]) + U(a) * C.PED_SCREW_R
        n = owner(decks, x, y)
        if n:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - W.cyl(M3_PILOT / 2, 16.0, (x, y, Z1 - 8.0))
        hw[f'bolt_pedestal_{k}'] = C.pan_screw((x, y, Z1 + 4.0), (0, 0, -1), 14.0)
    g['btabs'] = bumper_tabs(g, fp)                                   # Nov 14: the bumper's tabs under the rim, an M3 up into the deck each
    for k, (qn, ang, (x, y), piece) in enumerate(g['btabs']):
        parts[f'deck_{piece}'] = parts[f'deck_{piece}'] - W.cyl(M3_PILOT / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))
        hw[f'bolt_bumper_{k}'] = C.pan_screw((x, y, BUMPER_Z[1] - BTAB['t'] + 2.9), (0, 0, 1), BTAB['screw_L'])
    acc = accessory_holes(g, fp)                                       # Nov 14: zip-tie pairs and spare M3 pilots on a grid (as the second floor)
    for n, cutters in acc['cutters'].items():
        if cutters:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - b.Compound(cutters)
    g['accessory'] = acc['count']
    rb = g['r_deck'] + 0.5
    for n in B.NAMES:                                                  # bumper quarters hung on the deck's rim (one per quarter)
        a0, a1 = QUAD[n]
        parts[f'bumper_{n}'] = bumper_quarter(n, g['btabs'], rb)
    if UPPER:
        parts['pedestal'] = C.build_pedestal()
    sup, shw, sections, _, _ = fins()
    parts.update(sup)
    hw.update({k: v for k, v in shw.items() if not k.startswith('bolt_pin_')})
    g.update(rb=rb, info=info, sections=sections)
    return parts, hw, g


# ---- checks --------------------------------------------------------------------------------------------------------------------------------
def compound(d, skip=()):
    return b.Compound([v for k, v in d.items() if not k.startswith(skip)])


def check(parts, hw, g, fp):
    decks = g['decks']
    mv_plan = [B.place2d(fp['moving'], i) for i in range(4)]
    st_plan = [B.place2d(union_all([fp['pod_static'], fp['towers']]), i) for i in range(4)]
    r_m = max(rmax(q) for q in mv_plan)
    rep = dict(layout=dict(D=B.D_WHEEL, slide=B.SLIDE, deck_z=[Z0, Z1], deck_z_v14=[40.0, 62.0], r_deck=g['r_deck'], r_hold=g['r_hold'],
                           r_opening=g['r_open'], r_moving=round(r_m, 1), bumper_inside_r=g['rb'], robot_diameter=round(2 * (g['rb'] + BUMPER_T), 1),
                           robot_diameter_v14=450.3, bumper_to_moving_mm=round(g['rb'] - r_m, 1)))
    pair = lambda i, j: round(union_all([mv_plan[i], st_plan[i]]).distance(union_all([mv_plan[j], st_plan[j]])), 1)   # noqa: E731
    rep['pods_plan_gaps_mm'] = dict(side_FL_RL=pair(0, 1), side_FR_RR=pair(3, 2), front_FL_FR=pair(0, 3), back_RL_RR=pair(1, 2))
    q = {}
    for i, n in enumerate(B.NAMES):
        d = decks[n]
        seat_h = B.place2d(fp['seats'], i)
        seat_t = B.place2d(fp['towers'], i)
        mine = lambda pts: [p for p in pts if owner(decks, *p) == n]                                                   # noqa: E731
        margin = lambda pts: round(min(d.exterior.distance(Point(p)) if d.contains(Point(p)) else -d.distance(Point(p)) for p in pts), 2) if pts else None   # noqa: E731
        up_i = [tuple(B.place2d(Point(*p), i).coords[0]) for p in fp['up_screws']]
        tw_i = [tuple(B.place2d(Point(*p), i).coords[0]) for p in fp['tower_screws']]
        best = min(((ang, affinity.rotate(d, ang, origin=(0, 0)).bounds) for ang in range(0, 180, 3)), key=lambda t: max(t[1][2] - t[1][0], t[1][3] - t[1][1]))
        bb = best[1]
        q[n] = dict(volume_cm3=round(parts[f'deck_{n}'].volume / 1000, 1), solids=len(parts[f'deck_{n}'].solids()), valid=bool(parts[f'deck_{n}'].is_valid),
                    bed_fit_mm=[round(bb[2] - bb[0], 1), round(bb[3] - bb[1], 1)], fits_256=bool(max(bb[2] - bb[0], bb[3] - bb[1]) <= 256.0),
                    hanger_beam_tops_on_deck_pct=round(100 * seat_h.intersection(d).area / seat_h.area, 1),
                    tower_bases_on_deck_pct=round(100 * seat_t.intersection(d).buffer(0).area / max(seat_t.area, 1e-6), 1),
                    pod_up_screws_on_own_piece=f'{len(mine(up_i))} of {len(up_i)}', pod_up_screws_edge_margin_mm=margin(mine(up_i)),
                    tower_screws_edge_margin_mm=margin(mine(tw_i)), plan_gap_to_openings_mm=round(d.distance(g['openings'].buffer(-CLR)), 2),
                    top_edges=g['info'][n])
    rep['deck_pieces'] = q
    rep['cantilever_strips_removed_mm2'] = g['pruned']
    rep['joints'] = dict(kind='dovetail half-laps across the midline seams (lower piece: bottom half tab; upper piece: socket + its top half over it), '
                              f'2 x M3 x {LAP_SCREW_L:g} each, down from the top into the tab', order='FL and RR first, RL and FR dropped onto them',
                         laps=[dict(seam_deg=l['seam'], lower=l['lower'], upper=l['upper'], r=l['u']) for l in g['laps']],
                         per_seam={str(t): sum(1 for l in g['laps'] if l['seam'] == t) for t, _, _ in SEAMS})
    rep['accessory_holes'] = dict(per_piece=g.get('accessory'), pitch=ACC_PITCH, note='zip-tie pairs on multiples of the pitch (cable along x), '
                                  'M3 pilots (Ø2.8, through) half a pitch off, as the second floor; clear of seats, screws, laps, fin feet, pedestal')
    rep['deck_volume_cm3'] = dict(total=round(sum(v['volume_cm3'] for v in q.values()), 1), v14_frame=1141.3)
    clash, near_ = {}, {}
    fin_items = {k: v for k, v in parts.items() if k.startswith('support_')}
    for pn, a in R.POSES.items():
        for pod, (st, mv) in pods_placed(a).items():
            for nm, sh in (list(fin_items.items()) + ([('pedestal', parts['pedestal'])] if 'pedestal' in parts else [])
                           + [(k, v) for k, v in parts.items() if k.startswith('bumper_')]):        # Nov 14: the bumper's tabs under the rim
                for grp, dct in (('static', st), ('moving', mv)):
                    cp = compound(dct, ('roller_core', 'bearing_687', 'bolt_cart'))
                    if not W.near(sh, cp, 6.0):
                        continue
                    v = (sh & cp).volume if W.near(sh, cp, 0.0) else 0.0
                    key = f'{nm} / {pod} {grp}'
                    if v > 0.01:
                        clash[f'{key} ({pn})'] = round(v, 1)
                    near_[key] = min(near_.get(key, 99.0), 0.0 if v > 0.01 else round(sh.distance_to(cp), 2))
    rep['fins_pedestal_vs_pods'] = dict(overlap_mm3=clash, min_gap_mm=near_)
    rep['bumper'] = dict(z=BUMPER_Z, tabs=[dict(quarter=q, deg=round(a, 1), screw_xy=[round(v, 1) for v in xy], into=f'deck_{pc}') for q, a, xy, pc in g['btabs']],
                         per_quarter={n: sum(1 for q, *_ in g['btabs'] if q == n) for n in B.NAMES},
                         solids={k: (len(v.solids()), bool(v.is_valid)) for k, v in parts.items() if k.startswith('bumper_')},
                         screw=f"M3 x {BTAB['screw_L']:g} pan up through the tab (counterbored) into a Ø2.8 pilot through the deck: {BTAB['screw_L'] - BTAB['t'] + 2.9:.1f} mm of thread",
                         note='quarters print upside down: the top edge and the tabs on the bed, the gussets 45 deg')
    rep['fin_feet_on_deck'] = {str(k): owner(decks, *xy) or 'NOT ON THE DECK' for k, xy in enumerate(g['feet'])}
    rep['fins_vs_openings_plan_mm2'] = {k: round(B.proj(v, True).intersection(g['openings']).area, 1) for k, v in fin_items.items()}
    rep['pedestal'] = None if not UPPER else dict(on=sorted({owner(decks, *(np.array([C.LIDAR_X, 0.0]) + U(a) * C.PED_SCREW_R)) or 'none' for a in (45, 135, 225, 315)}),
                           height_mm=round(FF.LIDAR_BASE_Z - Z1, 1), height_v14_mm=round(FF.LIDAR_BASE_Z - 62.0, 1))
    top = max(compound(st).bounding_box().max.Z for st, _ in pods_placed(0.0).values())
    rep['heights'] = dict(pod_and_tower_top_z=round(top, 1), wheel_top_bump_z=round(R_WHEEL + float(R.P.rot_pt((0, 0), R.A_BUMP)[1]) + R_WHEEL, 1),
                          scan_plane_z=FF.LIDAR['plane'], floor2_under=C.FLOOR2_UNDER, lowest_chassis_z=dict(deck=Z0, bumper=BUMPER_Z[0]))
    rep['fins_volume_cm3'] = {k: round(v.volume / 1000, 1) for k, v in fin_items.items()}
    return rep


# ---- outputs ---------------------------------------------------------------------------------------------------------------------------------
def render(parts, hw, g, fp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    VAL.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, show in ((axs[0], False), (axs[1], True)):
        for n, d in g['decks'].items():
            for q in getattr(d, 'geoms', [d]):
                ax.fill(*q.exterior.xy, color=COLOURS['deck'], ec='k', lw=0.6)
                for h in q.interiors:
                    ax.fill(*h.xy, color='w', ec='k', lw=0.4)
            c = d.representative_point()
            ax.annotate(f'deck_{n}', (c.x, c.y), fontsize=9, ha='center')
        if show:
            for i in range(4):
                for key, col, al in (('moving', (0.95, 0.85, 0.5), 0.5), ('pod_static', (0.40, 0.60, 0.70), 0.8), ('towers', (0.25, 0.45, 0.60), 0.9)):
                    p = B.place2d(fp[key], i)
                    for qq in getattr(p, 'geoms', [p]):
                        ax.fill(*qq.exterior.xy, color=col, alpha=al)
        for lap in g['laps']:
            ax.plot(*lap['tab'].exterior.xy, color='r', lw=0.8)
            for x, y in lap['screws']:
                ax.plot(x, y, 'r.', ms=4)
        for x, y in g['up']:
            ax.plot(x, y, 'm.', ms=4)
        for x, y in g['tw']:
            ax.plot(x, y, 'c.', ms=4)
        for x, y in g['feet']:
            ax.plot(x, y, 'b.', ms=5)
        for _, sec in g['sections']:
            for qq in getattr(sec, 'geoms', [sec]):
                ax.fill(*qq.exterior.xy, color='k')
        ax.add_patch(plt.Circle((0, 0), g['rb'] + BUMPER_T, fill=False, color=COLOURS['bumper'], lw=3))
        ax.add_patch(plt.Circle((0, 0), 225.15, fill=False, color='grey', ls='--', lw=0.8))
        ax.axhline(0, lw=0.4, color='k')
        ax.axvline(0, lw=0.4, color='k')
        ax.set_aspect('equal')
        lim = g['rb'] + 20
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.grid(lw=0.3)
    axs[0].set_title(f'v15 mid deck (z {Z0:.0f}-{Z1:.0f}), four pieces split on the midlines; bumper Ø{2 * (g["rb"] + BUMPER_T):.0f} (dashed: v14 Ø450); '
                     'magenta: pod screws up into the deck, cyan: tower screws, blue: fin feet', fontsize=9)
    axs[1].set_title('with the pods: yellow moving (swept), teal hangers + beam (under the deck), dark: towers (on it)', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=80)
    plt.close(fig)
    pods = pods_placed(0.0)
    hide = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')
    allpods = {f'{n}/{k}': v for n, (st, mv) in pods.items() for k, v in {**st, **mv}.items() if not k.startswith(hide)}
    base = {k: v for k, v in parts.items() if not k.startswith('support_')}
    R.iso(dict(base, **allpods), VAL / 'iso_base.png', 215, 30, 'v15 mid deck with the pods (fins and second floor hidden)', size=(14, 10))
    if UPPER:
        R.iso(dict(parts, **allpods), VAL / 'iso_with_fins.png', 215, 25, 'v15 mid deck, pods, pedestal and v14\'s fins (shortened to the deck)', size=(14, 10))
    below = {k: v for k, v in dict(base, **allpods).items()}
    R.iso(below, VAL / 'iso_below.png', 200, -30, 'from below: pods hang under the deck, screwed up into it', size=(14, 10))
    one = {k: v for k, v in parts.items() if k in ('deck_FL', 'bumper_FL')}
    st, mv = pods['FL']
    one.update({f'FL/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide)})
    R.iso(one, VAL / 'iso_FL.png', 200, 30, 'deck_FL with its pod, towers and bumper quarter', size=(12, 9))
    mod = {f'FL/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide + ('rx_tower', 'plunger', 'spring', 'bolt_tower'))}
    R.iso(dict(mod, deck_FL=parts['deck_FL'].moved(b.Location((0, 0, 60)))), VAL / 'iso_FL_drop_out.png', 200, 20,
          'FL pod dropped out under its deck piece (the towers and springs stay on the deck: hidden here)', size=(12, 9))


def export(parts, hw):
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        if k.startswith(('deck_', 'pedestal', 'support_', 'bumper_')):
            write_part(s, PAL.ORANGE if k.startswith('bumper') else PAL.WHITE, OUT / f'{k}.step', k)
    items = {k: (v, PAL.ORANGE if k.startswith('bumper') else PAL.WHITE) for k, v in parts.items()}
    items.update({k: (v, H.MAGENTA) for k, v in hw.items()})
    for n, (st, mv) in pods_placed(0.0).items():
        for k, v in {**st, **mv}.items():
            items[f'{n}_{k}'] = (H.step_model(k, v), H.MAGENTA if k.startswith(H.STEP_HARDWARE) or k.startswith('plunger') else R.colour(k))
    write_step(items, OUT / 'v15_mid_base_assembly.step')
    return len(items)


def main(stage):
    fp = pod_build()[4]
    if stage == 'study':
        g = layout(fp)
        print({k: g[k] for k in ('r_deck', 'r_hold', 'r_open')}, {n: round(d.area / 100, 1) for n, d in g['decks'].items()})
        return
    parts, hw, g = build(fp)
    for k, v in parts.items():
        if k.startswith('deck_'):
            print(k, len(v.solids()), v.is_valid, round(v.volume / 1000, 1), 'cm3', flush=True)
    VAL.mkdir(parents=True, exist_ok=True)
    if stage in ('check', 'all'):
        rep = check(parts, hw, g, fp)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1, default=str))
        print(json.dumps(rep['layout']), flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, g, fp)
        print('rendered', flush=True)
    if stage in ('export', 'all'):
        print('exported', export(parts, hw), flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'study')
