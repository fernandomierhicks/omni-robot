"""v15 pod library, layer 3 of 5 (pod.py -> pod_lean.py -> pod_rx.py -> pod_p4.py -> pod_mid.py; tools/README.md): the fork's arms, the 627
pivot housings, rings, the static bosses, the wheel envelope, the pod checks (run_checks) and renders (iso, section). Its constants are v15's
own (the P4 arm outline, no shim, the motor plate's harness hole): no other module changes them. Library only, no entry point.

From cad/v14's blue-sky pod proposals Pod_reimagined_1..4 (Nov 13 2026, Fernando: "how the suspension mounts onto the fixed structure while
retaining a compact footprint"). Common to all of them (Fernando's asks, Nov 13):
 * both pivots built the same way: the 627's OUTER race lives in the fork arm, held both ways (in a seat from the arm's wheel face, on the
   seat's floor at the outer face side, and under a printed ring with 3 x M3 flat heads, flush in the wheel face); the static part's boss
   (Ø10, on the inner race only) and a printed cap from the other side, with Ø7.2 spigots into the bore, are clamped onto the INNER race by
   one M3 along the axis (Ø3.2 snug in the cap and spigots, thread in the static part or the cap). Printed outer face down, every housing
   surface faces up: no support;
 * the arms are one rounded outline each (an exact convex hull of circles: real arcs and tangent lines, no patches), 12 mm thick, filleted
   on the face away from the bed, chamfered on the bed face;
 * the outer arm hugs the wheel: 2 mm from the turning parts (x -37.4; the treads and petals reach x -35.32) wherever only the treads are,
   a 45 deg dish to x -41.5 inside r 38 where the hub's left end and cartridge are (x -39.5 out to r 36). Its outer face moves in from
   x -52.5 to -49.4;
 * the wheel, motor, hub, 6804 stub and its retainer are cad/v14's, unchanged; the pivot stays at y 80.6, level with the axle; travel -3 / +15.

Pod frame as dims.py: x = wheel axis (+ toward the motor = the robot centre), y toward the pivot, z up, axle at the origin, floor z -70.
"""
import json
import math
import pickle
import sys
import tempfile
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
import hardware as H                                                   # noqa: E402
import palette as PAL                                                  # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

ROOT = W.ROOT                                                          # this CAD version's folder
PIV = (PIVOT_Y, PIVOT_Z)
PIVOT_AXIS = P.PIVOT_AXIS
POSES = P.POSES
A_DROOP, A_BUMP = P.A_DROOP, P.A_BUMP
box, along_x, cyl, line, arc, V = P.box, W.along_x, W.cyl, W.line, W.arc, W.V
overlap, near, min_dist = W.overlap, W.near, W.min_dist

# ---- fork arms (shared) ------------------------------------------------------------------------------------------------------------------
ARM_T = 12.0                       # both arms: lip 2 + 627 7 + ring 3
OUTER_ARM = (-49.4, -37.4)         # outer arm x (outer face, wheel face); the wheel face 2.0 from the turning parts (the treads and petals reach
                                   # x -35.32 at r 44-52); its outer face 3.1 in from pod_lean (-52.5)
INNER_ARM = (54.7, 42.7)           # motor plate x (outer face, wheel face = 2 mm can pocket in front of the motor's rear face, 44.7)
DISH_X = -41.5                     # outer arm's wheel face inside r DISH_R (2 mm off the hub's left end / cartridge at x -39.5, r <= 36)
DISH_R = 38.0                      # the dish: r 38 at x -41.5, a 45 deg cone out to r 42.5 at the arm's wheel face
AXLE_R = 29.5                      # the motor plate round the axle (the can is r 26.5, its screws r 23.5)
AXLE_R_OUTER = 21.0                # v15 Nov 14 (Fernando): the outer arm round the axle (was 29.5 like the plate), so its 3 cartridge screws (flat
                                   # heads at r 26, Ø5.5) can be driven through past it: fork + retainer + 6804 + cartridge are assembled first,
                                   # then put onto the hub and screwed, turning the wheel to bring each screw clear of the arm (the stub
                                   # shoulder is r 11.5)
LOBE_R = 20.5                      # round the pivot: the ring (r 18) + 2.3 wall
EDGE_R, BED_CHAMFER = 2.0, 0.6     # arm edges: rounded on the face away from the bed, chamfered on the bed face
# pivot bearing housing (both arms the same, mirrored)
LIP_T = 2.0                        # lip on the outer face side: the seat floor, on the outer race only (r 9.4 .. seat), with a Ø18.8 hole
LIP_HOLE_R = 9.4                   # the 627 outer race land from ~Ø18.6 (VERIFY): the floor stays on the outer race
RING_T, RING_R0, RING_R1 = 3.0, 9.5, 18.0    # printed ring on the outer race (627 outer race land ~Ø18.6, VERIFY), flush in the wheel face
RING_CLR = 0.2                     # ring recess over the ring
RING_SCREW_R, RING_SCREW_AZ, RING_SCREW_L = 14.5, (90.0, 210.0, 330.0), 10.0   # 3 x M3 x 10 flat heads (7.5 mm of thread in the arm)
CAP_D = 10.0                       # cap on the inner race: Ø10 flange (the inner race only)
SPIGOT_D, SPIGOT_L = B627_PIN_D, 3.0   # Ø7.20 spigots (coupon_627 cell 4) 3 into the race's 7 mm bore from each side (1 mm apart)
BOSS_D = PIVOT_BOSS_D              # the static part's boss on the inner race (Ø10: inner race only)
STATIC_GAP = 1.0                   # static end face to the arm's wheel face
# Nov 14: each pivot is axially closed twice (the outer race held in its arm, the inner race clamped to the static part), so the static part's
# printed length must match the fork's printed spacing. The boss stops SHIM_T short of the race and a shim (a printed Ø10 / Ø7.4 washer on the
# spigot, picked at assembly from 0.4 / 0.6 / 0.8) fills the gap: the pivots turn free without relying on print accuracy (+-0.2 each side).
SHIM_T = 0.0                       # nominal gap = shim thickness. v15 (Nov 14, Fernando): 0, no shim washer: the beam's bosses are printed
                                   # to the exact length and bear on the 627s' inner races; a pivot that binds gets its boss face sanded, one
                                   # with play a printed shim (0.4 / 0.6 / 0.8, build_shim) added at assembly. (Proposals 1-4: 0.6)
SHIM_ID = SPIGOT_D + 0.2           # Ø7.4 on the Ø7.2 spigot
PIVOT_SCREW_L = 25.0               # M3 x 25 flat head: ~18.5 mm of Ø2.9 thread in the static part (a long thread)
CSK_D = 6.0                        # M3 flat head
# 6804 stub + retainer (cad/v14 pod_lean, unchanged; the arm's outer face moved in 3.5, so its screw is an M3 x 25)
RET_SCREW_L_RX = 25.0
CB_D, CB_DEPTH = 7.2, 2.9          # flush counterbore for an M3 pan head (6.5 x 2.5)
# motor plate (inner arm)
MOTOR_SCREW_L_RX = 10.0            # 4 x M2.5 x 10 pan heads (3.8 into the motor), heads 1.8 under the plate's outer face
TIE_PILOT, TIE_CB = 3.0, 6.8       # fork tie bolts: Ø3.0 pilot their whole length in the outer half, Ø3.4 through the inner half
TENON_INSET, TENON_L, TENON_CLR = 2.5, 7.0, 0.05

LEAD_TIE = False                   # the leads' zip-tie tunnel through the motor plate's rim (pod_lean): off in v15 (its support could not be
                                   # pushed out, Fernando Nov 14); HARNESS_HOLE instead
HARNESS_HOLE = (26.0, 18.0, 4.6)   # (y, z, Ø): a straight hole through the motor plate (x, its print axis: no support) to zip-tie the motor
                                   # leads to, ahead of the spring pads and the inner hanger's front (y 29.5), 8 mm under the arm's top edge;
                                   # 4.4 mm of wall to the nearest motor screw's counterbore, 2.3 to the can pocket (v15 Nov 14, Fernando)
CLR_WHEEL = 2.0                    # wanted: anything to the wheel's swept envelope (Fernando: 2 mm from the TPU is enough)
CLR_FORK = 0.8                     # wanted: static parts to the fork in every pose (pod_lean: keel to fork 0.8)
CLR_HARNESS = 4.0                  # wanted: static parts to the XT30 + leads (cad/solidworks.md 16)

EXTRA_COLOURS = {'spring': (0.15, 0.65, 0.25), 'tie_lead': (0.12, 0.12, 0.12), 'chassis_ghost': (0.85, 0.85, 0.82)}


VIEW = {'pod_fork': (0.62, 0.62, 0.65), 'rx_block': (0.40, 0.58, 0.70), 'rx_frame': (0.40, 0.58, 0.70), 'rx_tower': (0.40, 0.58, 0.70),
        'rx_tie': (0.40, 0.58, 0.70), 'rx_beam': (0.40, 0.58, 0.70), 'rx_hanger': (0.40, 0.58, 0.70), 'rx_deck': (0.86, 0.86, 0.82), 'rx_ring': (0.80, 0.80, 0.83), 'rx_cap': (0.55, 0.72, 0.82), 'pod_retainer': (0.80, 0.80, 0.83),
        'roller_tread': (0.77, 0.77, 0.77), 'ak40': (0.30, 0.30, 0.33), 'spring': (0.20, 0.70, 0.30)}


def view_colour(name):
    """display colours for the renders (the filament grey reads as black on screen): fork light grey, static parts blue-grey"""
    n = name.split('/')[-1]
    for k, v in VIEW.items():
        if n.startswith(k):
            return v
    return colour(name)


def colour(name):
    n = name.split('/')[-1]
    if n.startswith('rx_deck'):                                        # the chassis' stand-in: chassis parts are Jade White
        return PAL.WHITE
    if n.startswith(('pod_', 'rx_')):
        return PAL.DARK_GRAY
    if n.startswith(H.STEP_HARDWARE) or n.startswith('plunger'):
        return H.MAGENTA
    for k, v in EXTRA_COLOURS.items():
        if n.startswith(k):
            return v
    return L.colour(n)


# ---- 2D: exact convex hull of circles in the y-z plane ---------------------------------------------------------------------------------
def hull_edges(circles):
    """closed y-z outline (edges in the plane x = 0) of the convex hull of circles [(y, z, r), ...]: arcs of the hull circles joined by their
    outer tangent lines, counter-clockwise"""
    C = np.array([c[:2] for c in circles], float)
    R = np.array([c[2] for c in circles], float)
    th = np.linspace(0, 2 * math.pi, 7200, endpoint=False)
    U = np.stack([np.cos(th), np.sin(th)], 1)
    order = np.argmax(U @ C.T + R, axis=1)
    seq = [int(order[0])]
    for i in order[1:]:
        if int(i) != seq[-1]:
            seq.append(int(i))
    while len(seq) > 1 and seq[-1] == seq[0]:
        seq.pop()
    n = len(seq)
    tang = []                                                           # (normal angle, point on circle i, point on circle j) for i -> j
    for k in range(n):
        i, j = seq[k], seq[(k + 1) % n]
        d = C[j] - C[i]
        D = np.linalg.norm(d)
        u = math.atan2(d[1], d[0]) - math.acos((R[i] - R[j]) / D)
        uu = np.array([math.cos(u), math.sin(u)])
        tang.append((u, C[i] + R[i] * uu, C[j] + R[j] * uu))
    edges = []
    for k in range(n):
        u_in, _, p_in = tang[k - 1]                                    # arriving at circle seq[k]
        u_out, p_out, _ = tang[k]
        c, r = C[seq[k]], R[seq[k]]
        du = (u_out - u_in) % (2 * math.pi)
        if du > 1e-6:
            um = u_in + du / 2
            pm = c + r * np.array([math.cos(um), math.sin(um)])
            edges.append(arc(V(*p_in), V(*pm), V(*p_out)))
        q = tang[k][2]
        if np.linalg.norm(q - p_out) > 1e-6:
            edges.append(line(V(*p_out), V(*q)))
    return edges


def hull_prism(circles, x0, x1):
    return P.prism(hull_edges(circles), min(x0, x1), max(x0, x1))


def on_x(e, x):
    return all(abs(e.position_at(t).X - x) < 1e-3 for t in (0, 0.5, 1))


def round_bed(s, x_bed, x_top, r_top=EDGE_R, chamfer=BED_CHAMFER, info=None, key=''):
    """round every edge in the face x = x_top (falling back to smaller radii), chamfer those in the bed face x = x_bed"""
    for r in (r_top, 1.4, 1.2, 1.0, 0.6):
        try:
            out = b.fillet([e for e in s.edges() if on_x(e, x_top)], r)
            if out.is_valid and len(out.solids()) == 1:
                s = out
                if info is not None:
                    info[key + '_top_r'] = r
                break
        except Exception:                                              # noqa: BLE001
            pass
    try:
        out = b.chamfer([e for e in s.edges() if on_x(e, x_bed)], chamfer)
        if out.is_valid and len(out.solids()) == 1:
            s = out
            if info is not None:
                info[key + '_bed_chamfer'] = chamfer
    except Exception:                                                  # noqa: BLE001
        pass
    return s


def fillet_try(s, edges, radii, info=None, key=''):
    """fillet edges at the first radius OCC accepts; record what took"""
    if not edges:
        return s
    for r in radii:
        try:
            out = b.fillet(edges, r)
            if out.is_valid and len(out.solids()) == 1:
                if info is not None:
                    info[key] = f'R{r:g} on {len(edges)} edges'
                return out
        except Exception:                                              # noqa: BLE001
            pass
    if info is not None:
        info[key] = f'refused ({len(edges)} edges)'
    return s


# ---- the pivot housing (both arms, the same way) --------------------------------------------------------------------------------------------
# Seat and ring on the arm's WHEEL face, the lip on its OUTER face: printed outer face down, the seat, its floor (the lip's land) and the ring's
# recess all open upward, so the housing needs no support. The 627's inner race is clamped by the static part's boss from one side and a printed
# cap from the other: inboard static parts (proposals 1, 2) come through the ring's bore, outboard ones (3) through the lip's hole.
def frame_x(arm):
    """(x_outer_face, x_wheel_face, s): s = +1 if the outer face is toward +x (s points from the wheel face to the outer face)"""
    xo, xw = arm
    return xo, xw, (1.0 if xo > xw else -1.0)


def brg_x(arm):
    """the 627's faces: (b_ring: toward the wheel face / ring, b_lip: toward the lip / outer face)"""
    xo, xw, s = frame_x(arm)
    return xw + s * RING_T, xw + s * (RING_T + B627_W)


def ring_screw_yz():
    return [(PIVOT_Y + RING_SCREW_R * math.cos(math.radians(a)), PIVOT_Z + RING_SCREW_R * math.sin(math.radians(a))) for a in RING_SCREW_AZ]


def xs(a, c):
    return min(a, c), max(a, c)


def housing_cut(arm):
    """one arm's pivot housing: the ring's recess and the 627's seat from the wheel face, the seat's floor (the lip, LIP_T, on the outer race only)
    with its Ø18.8 hole out of the outer face, the ring's three Ø2.8 pilots through the arm"""
    xo, xw, s = frame_x(arm)
    b_ring, b_lip = brg_x(arm)
    cut = along_x(RING_R1 + RING_CLR, *xs(xw - s * 0.1, b_ring), *PIV) + along_x(B627_SEAT_D / 2, *xs(xw - s * 0.1, b_lip), *PIV)
    cut = cut + along_x(LIP_HOLE_R, *xs(b_lip - s * 0.1, xo + s * 0.1), *PIV)
    for y, z in ring_screw_yz():
        cut = cut + along_x(M3_PILOT / 2, *xs(xw - s * 0.1, xo + s * 0.1), y, z)
    return cut


def csk_cut(x_face, into, y, z, d_head=CSK_D, d_shank=M3_CLR):
    """countersink for an M3 flat head from the face x = x_face going `into` (+-1 along x): a Ø6 x 0.5 bore (cad/solidworks.md 18: 0.5 deeper)
    then the 90 deg cone"""
    bore = along_x(d_head / 2, *xs(x_face - into * 0.1, x_face + into * CSK_EXTRA), y, z)
    cone = b.Solid.make_cone(d_head / 2, d_shank / 2, (d_head - d_shank) / 2, b.Plane(origin=(x_face + into * CSK_EXTRA, y, z), z_dir=(into, 0, 0)))
    return bore + cone


def build_ring(arm):
    """the outer race's ring, flush in the arm's wheel face: Ø19 .. Ø36 x 3, three countersinks for M3 x 10 flat heads (0.5 deeper)"""
    xo, xw, s = frame_x(arm)
    b_ring, _ = brg_x(arm)
    x0, x1 = xs(xw, b_ring)
    r = along_x(RING_R1, x0, x1, *PIV) - along_x(RING_R0, x0 - 1, x1 + 1, *PIV)
    for y, z in ring_screw_yz():
        r = r - along_x(M3_CLR / 2, x0 - 0.1, x1 + 0.1, y, z) - csk_cut(xw, s, y, z)
    return r


def ring_screw_recs(arm, tag):
    xo, xw, s = frame_x(arm)
    return [dict(kind='flat', size=3.0, L=RING_SCREW_L, p=(xw + s * CSK_EXTRA, y, z), d=(s, 0.0, 0.0), name=f'bolt_ring_{tag}_{i + 1}')
            for i, (y, z) in enumerate(ring_screw_yz())]


def bearing_627(arm):
    x0, x1 = xs(*brg_x(arm))
    return along_x(B627_OD / 2, x0, x1, *PIV) - along_x(B627_BORE / 2, x0 - 1, x1 + 1, *PIV)


def static_end_x(arm, side='wheel'):
    """the static part's face next to the arm: on its wheel side (inboard static) or its outer side (outboard)"""
    xo, xw, s = frame_x(arm)
    return xw - s * STATIC_GAP if side == 'wheel' else xo + s * STATIC_GAP


def static_boss(arm, side='wheel', shim=True):
    """the static part's boss on the inner race (Ø10, from its face through the ring's bore (inboard) or the lip's hole (outboard)) and its Ø7.2
    spigot 3 into the bore; shim=False: the boss bears on the race itself (proposal 4's frames: the shim is on the beam's side)"""
    xo, xw, s = frame_x(arm)
    b_ring, b_lip = brg_x(arm)
    xe, bf, into = (static_end_x(arm, 'wheel'), b_ring, s) if side == 'wheel' else (static_end_x(arm, 'outer'), b_lip, -s)
    xb = bf - into * (SHIM_T if shim else 0.0)                         # the boss's face: SHIM_T short of the race (the shim's gap)
    return along_x(BOSS_D / 2, *xs(xe - into * 0.5, xb), *PIV) + along_x(SPIGOT_D / 2, *xs(xb - into * 0.01, bf + into * SPIGOT_L), *PIV)


def build_shim(arm, side='wheel'):
    """the shim between the static boss and the inner race (nominal SHIM_T; print 0.4 / 0.6 / 0.8 and fit the one that lets the pivot turn
    free with the pivot screw tight)"""
    xo, xw, s = frame_x(arm)
    b_ring, b_lip = brg_x(arm)
    bf, into = (b_ring, s) if side == 'wheel' else (b_lip, -s)
    x0, x1 = xs(bf - into * SHIM_T, bf)
    return along_x(BOSS_D / 2, x0, x1, *PIV) - along_x(SHIM_ID / 2, x0 - 0.1, x1 + 0.1, *PIV)


CAP_PROUD = 1.0                    # the outer cap stands this far out of the arm's outer face (3 mm for the flat head's countersink)


def build_cap(arm):
    """inboard static part: the inner race's cap on the OUTER side, in the lip's hole: Ø10 flange from the race to CAP_PROUD past the outer face,
    Ø7.2 spigot 3 into the bore, Ø3.2 snug, the pivot screw's flat head countersunk in it"""
    xo, xw, s = frame_x(arm)
    _, b_lip = brg_x(arm)
    xf = xo + s * CAP_PROUD
    c = along_x(CAP_D / 2, *xs(b_lip, xf), *PIV) + along_x(SPIGOT_D / 2, *xs(b_lip + s * 0.01, b_lip - s * SPIGOT_L), *PIV)
    c = c - along_x(M3_SNUG / 2, *xs(xf + s * 0.1, b_lip - s * (SPIGOT_L + 0.1)), *PIV)
    return c - csk_cut(xf, -s, *PIV, d_shank=M3_SNUG)


def pivot_screw_rec(arm, name):
    """inboard static part: the M3 x 25 flat head in the outer cap, along the axis into the static part"""
    xo, xw, s = frame_x(arm)
    return dict(kind='flat', size=3.0, L=PIVOT_SCREW_L, p=(xo + s * (CAP_PROUD - CSK_EXTRA), *PIV), d=(-s, 0.0, 0.0), name=name)


# ---- the arms ----------------------------------------------------------------------------------------------------------------------------
def arm_circles(extra=(), r_axle=AXLE_R):
    return [(0.0, 0.0, r_axle), (PIVOT_Y, PIVOT_Z, LOBE_R)] + list(extra)


WAISTS = [(38.0, -78.0, 50.0)]     # [(y, z, r)]: circles taken out of the arm outline: the waist between the axle and the pivot (P4's, v15)
WAIST_ROUND = 6.0                  # the waist's corners rounded in the outline (2D)
POLY_CUTS = [[(P.Y_S - 38.0, 29.8), (P.Y_S - 22.0, PAD_Z), (P.Y_S + 18.0, PAD_Z), (P.Y_S + 18.0, 90.0), (P.Y_S - 38.0, 90.0)]]
                                   # polygons taken out of the arm outline: the arm's top ramped down onto the springs' flat pad (P4's, v15)
POLY_ROUND = (10.0, 6.0, 3.0)      # their corners in the outline rounded (the first radius OCC accepts)


def _on_segment(v, a, c, tol=1e-3):
    p, a, c = np.array([v.Y, v.Z]), np.array(a, float), np.array(c, float)
    t = np.clip(np.dot(p - a, c - a) / max(np.dot(c - a, c - a), 1e-12), 0, 1)
    return np.linalg.norm(p - (a + t * (c - a))) < tol


def arm_face(extra=(), r_axle=AXLE_R):
    """the arm outline as a face in the y-z plane (x = 0): the hull, less the waist circles and cut polygons, their corners rounded"""
    f = b.Face(b.Wire(hull_edges(arm_circles(extra, r_axle))))
    if not WAISTS and not POLY_CUTS:
        return f
    for y, z, r in WAISTS:
        f = f - b.Face(b.Wire([b.Edge.make_circle(r, b.Plane(origin=(0, y, z), z_dir=(1, 0, 0)))]))
    for poly in POLY_CUTS:
        f = f - b.Face(b.Wire.make_polygon([b.Vector(0, y, z) for y, z in poly], close=True))
    f = f.faces()[0] if hasattr(f, 'faces') else f
    waist = [v for v in f.vertices() if any(abs(math.hypot(v.Y - y, v.Z - z) - r) < 1e-3 for y, z, r in WAISTS)]
    if waist:
        try:
            f = f.fillet_2d(WAIST_ROUND, waist)
        except Exception:                                              # noqa: BLE001
            pass
    cut = [v for v in f.vertices() if any(_on_segment(v, poly[i], poly[(i + 1) % len(poly)]) for poly in POLY_CUTS for i in range(len(poly)))]
    for r in POLY_ROUND if cut else ():
        try:
            f = f.fillet_2d(r, cut)
            break
        except Exception:                                              # noqa: BLE001
            pass
    return f


def arm_prism(extra, x0, x1, r_axle=AXLE_R):
    f = arm_face(extra, r_axle)
    x0, x1 = min(x0, x1), max(x0, x1)
    return b.extrude(f.moved(b.Location((x0, 0, 0))), amount=x1 - x0, dir=(1, 0, 0))


def member(circles, x0=None, x1=None):
    """a cross member between the arms whose section is the hull of `circles` (use the arm outline's own corner circles, so the member's faces
    run on flush and tangent from the arm's outline), from inside the outer arm to the split plane (the motor plate's wheel face)"""
    return hull_prism(circles, OUTER_ARM[0] + 2.0 if x0 is None else x0, INNER_ARM[1] if x1 is None else x1)


def outer_arm(extra_circles=(), info=None, members=()):
    """outer arm: the hull outline, 12 thick, with its cross members (joined before the edges are rounded, so the rounds run on round the
    joins), the wheel-side dish, the 6804 stub and the retainer's screw hole, the pivot housing. Prints on its outer face (on the bed)"""
    xo, xw = OUTER_ARM
    a = arm_prism(extra_circles, xo, xw, AXLE_R_OUTER)
    for m in members:
        a = a + m
    a = a.clean()                                                       # the flush faces become one face (no seam edges for the rounds)
    a = round_bed(a, xo, xw, info=info, key='outer_arm')
    h = xw - DISH_X + 0.1                                               # the dish: a 45 deg frustum, r 38 at its floor
    a = a - b.Solid.make_cone(DISH_R, DISH_R + h, h, b.Plane(origin=(DISH_X, 0, 0), z_dir=(1, 0, 0)))
    a = fillet_try(a, [e for e in a.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - DISH_X) < 1e-3 and abs(e.radius - DISH_R) < 0.01],
                   (1.5, 1.0), info, 'dish_floor')
    xs0 = X_PLATE_FLOOR - STUB_SHOULDER_GAP                              # the 6804 stub (pod_lean, unchanged)
    stub = along_x(STUB_SHOULDER_D / 2, DISH_X - 1.0, xs0) + along_x(STUB_D / 2, xs0 - 0.01, L.STUB_END)
    a = a + stub
    a = fillet_try(a, [e for e in a.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - DISH_X) < 1e-3
                       and abs(e.radius - STUB_SHOULDER_D / 2) < 1e-3], (1.5, 1.0), info, 'stub_root')
    a = a - along_x(M3_SNUG / 2, xo - 0.1, L.STUB_END + 0.1) - along_x(CB_D / 2, xo - 0.1, xo + CB_DEPTH)
    return a - housing_cut(OUTER_ARM)


def retainer_screw_rec():
    xo = OUTER_ARM[0]
    return dict(kind='pan', size=3.0, L=RET_SCREW_L_RX, p=(xo + CB_DEPTH, 0.0, 0.0), d=(1.0, 0.0, 0.0), name='bolt_retainer')


def motor_plate(extra_circles=(), info=None):
    """inner arm = the motor plate: the hull outline, 12 thick, the can pocket (2 deep, 0.5 lead-in), 4 x M2.5 x 10 in counterbores, the
    motor leads' zip-tie tunnel (pod_lean), the pivot housing. Prints on its outer face (x 54.7 on the bed)"""
    xo, xw = INNER_ARM
    a = arm_prism(extra_circles, xw, xo)
    a = round_bed(a, xo, xw, r_top=1.4, info=info, key="motor_plate")
    rp = MOTOR_R + MOTOR_POCKET_CLR
    a = a - along_x(rp, xw - 0.1, INNER_ARM_X[0]) - b.Cone(rp + 0.6, rp - 0.01, 0.6).rotate(b.Axis.Y, 90).moved(b.Location((xw + 0.2, 0, 0)))
    head_x = INNER_ARM_X[0] + MOTOR_SCREW_L_RX - REAR_ENGAGE
    for az in REAR_HOLES_AZ:
        y, z = W.at_angle(REAR_HOLES_R, az)
        a = a - along_x(REAR_CLR_D / 2, xw - 0.1, xo + 0.1, y, z) - along_x(REAR_CBORE_D / 2, head_x, xo + 0.1, y, z)
    if LEAD_TIE:
        a = a - L.lead_tie_cuts() - lead_tie_relief()
    if HARNESS_HOLE:                                                   # a plain through hole for a zip tie (along the plate's print axis)
        y, z, d = HARNESS_HOLE
        a = a - along_x(d / 2, xw - 0.1, xo + 0.1, y, z)
    return a - housing_cut(INNER_ARM)


def lead_tie_relief():
    """v15 Nov 14: with the XT30 facing up, the tie's leg crosses the plate's rim where the outline runs on toward the spring pads (r ~30.5,
    az ~67): a groove the tie's band + 0.4 thick and + 0.4 wide is cut into the rim wherever the band meets it (a seat for the tie)"""
    t0, w0 = L.LEAD_TIE_T, L.LEAD_TIE_W
    L.LEAD_TIE_T, L.LEAD_TIE_W = t0 + 0.8, w0 + 0.4
    try:
        return L.lead_tie()
    finally:
        L.LEAD_TIE_T, L.LEAD_TIE_W = t0, w0


# ---- droop stop: an M3 pan head on an arm face running in an arc slot in the facing static face ------------------------------------------
DROOP_SLOT_D, DROOP_SLOT_CLR, DROOP_SCREW_L = 2.0, 0.25, 8.0


def droop_slot(x_face, into, yz):
    """the arc slot (DROOP_SLOT_D deep into the static part from its face x = x_face, `into` +-1) that the head of a screw at yz on the facing arm
    runs in: swept from 4 deg past the bump pose to the droop pose, where the head (Ø6.5) meets the slot's round end"""
    dy, dz = yz[0] - PIVOT_Y, yz[1] - PIVOT_Z
    rd, th0 = math.hypot(dy, dz), math.degrees(math.atan2(dz, dy))
    hr = H.PAN_HEAD[3.0][0] / 2
    t_d, t_b = th0 + A_DROOP, th0 + A_BUMP - 4.0
    lo, hi = sorted((t_b, t_d))
    ts = np.linspace(lo, hi, 24)
    pts = [(PIVOT_Y + (rd + hr + DROOP_SLOT_CLR) * math.cos(math.radians(t)), PIVOT_Z + (rd + hr + DROOP_SLOT_CLR) * math.sin(math.radians(t))) for t in ts]
    pts += [(PIVOT_Y + (rd - hr - DROOP_SLOT_CLR) * math.cos(math.radians(t)), PIVOT_Z + (rd - hr - DROOP_SLOT_CLR) * math.sin(math.radians(t))) for t in ts[::-1]]
    x0, x1 = xs(x_face - into * 0.1, x_face + into * DROOP_SLOT_D)
    at = lambda t, r=rd: (PIVOT_Y + r * math.cos(math.radians(t)), PIVOT_Z + r * math.sin(math.radians(t)))   # noqa: E731
    return P.yz_poly(pts, x0, x1) + along_x(hr, x0, x1, *at(t_d)) + along_x(hr + DROOP_SLOT_CLR, x0, x1, *at(t_b))


def droop_screw(x_face, out, yz, name):
    """record + pilot: an M3 x 8 pan head on the arm face x = x_face, its head standing out of the face toward `out` (+-1), its shank into the
    arm (Ø2.8 pilot through the arm, length `through`)"""
    rec = dict(kind='pan', size=3.0, L=DROOP_SCREW_L, p=(x_face, *yz), d=(-out, 0.0, 0.0), name=name)
    return rec, along_x(M3_PILOT / 2, *xs(x_face + out * 0.1, x_face - out * ARM_T - out * 0.1), *yz)


# ---- front springs (ahead of the pivot, over each arm, pod.py's plungers: the dome down on a pad on the arm's top) --------------------------
PAD_CIRCLES = [(Y_S_ := PIVOT_Y - SPRING_ARM) - 14.0, Y_S_ + 14.0]   # y of the two corner circles that give each arm a flat top at PAD_Z
                                   # (v15: +-14, P4's; the proposals 1-3 had +-12)
PAD_CR = 4.0


def pad_circles():
    """arm outline circles whose tops make a flat land at z PAD_Z from y Y_S - 12 to Y_S + 12 (the plunger domes slide on it)"""
    return [(y, PAD_Z + 1.0 - PAD_CR, PAD_CR) for y in PAD_CIRCLES]   # tops 1 mm over the pad: the flat cut slices them cleanly (no slivers)


def rail_line(y0, y1):
    """the bump rails' underside under a tower: the pad's top at bump (a line in y-z from y0 to y1)"""
    p0 = P.rot_pt((y0, PAD_Z), A_BUMP)
    p1 = P.rot_pt((y1, PAD_Z), A_BUMP)
    m = (p1[1] - p0[1]) / (p1[0] - p0[0])
    return lambda y: float(p0[1] + m * (y - p0[0]))


def motor_screw_recs():
    head_x = INNER_ARM_X[0] + MOTOR_SCREW_L_RX - REAR_ENGAGE
    return [dict(kind='pan', size=2.5, L=MOTOR_SCREW_L_RX, p=(head_x, *W.at_angle(REAR_HOLES_R, az)), d=(-1.0, 0.0, 0.0), name=f'bolt_motor_{i + 1}')
            for i, az in enumerate(REAR_HOLES_AZ)]


# ---- fork tie bolts and tenons -------------------------------------------------------------------------------------------------------------
def tie_lines_holes(lines):
    """lines: [(y, z, L_outer, L_inner)]: each line has an outer bolt (head in the outer arm's face, threads into the outer half) and an inner
    bolt (head in the motor plate's face, passes the plate in Ø3.4, threads into the outer half). Returns (outer cuts, inner cuts, records)"""
    xo_o, xw_o = OUTER_ARM
    xo_i, xw_i = INNER_ARM
    oc, ic, recs = [], [], []
    for k, (y, z, lo, li) in enumerate(lines):
        so, si = xo_o + CB_DEPTH, xo_i - CB_DEPTH
        oc += [along_x(TIE_CB / 2, xo_o - 0.1, so, y, z), along_x(TIE_PILOT / 2, so, xw_i + TENON_L + 0.2, y, z)]   # on through the tenon
        ic += [along_x(TIE_CB / 2, si, xo_i + 0.1, y, z), along_x(M3_CLR / 2, xw_i - 0.1, si + 0.01, y, z)]
        recs += [dict(kind='pan', size=3.0, L=float(lo), p=(so, y, z), d=(1.0, 0, 0), name=f'bolt_tie_{k + 1}_outer'),
                 dict(kind='pan', size=3.0, L=float(li), p=(si, y, z), d=(-1.0, 0, 0), name=f'bolt_tie_{k + 1}_inner')]
    return oc, ic, recs


def tenon(y0, y1, z0, z1, grow=0.0):
    """square tenon on a cross member's end at the split plane (the motor plate's wheel face), or its socket (grow = TENON_CLR)"""
    xw = INNER_ARM[1]
    t = box(xw - 0.01, xw + TENON_L + grow, y0 + TENON_INSET - grow, y1 - TENON_INSET + grow, z0 + TENON_INSET - grow, z1 - TENON_INSET + grow)
    if grow:
        return t
    try:
        return b.chamfer([e for e in t.edges() if on_x(e, xw + TENON_L)], 0.5)
    except Exception:                                                  # noqa: BLE001
        return t


def cross_member(y0, y1, z0, z1, r=3.0):
    """a cross member between the arms (outer arm's wheel face .. the split plane), rounded rectangle section, its long edges rounded r"""
    xo_o, xw_o = OUTER_ARM
    m = box(xw_o - 1.0, INNER_ARM[1], y0, y1, z0, z1)
    return fillet_try(m, [e for e in m.edges() if abs(e.position_at(1).X - e.position_at(0).X) > 20], (r, r - 1, 1.0))


# ---- springs and plungers --------------------------------------------------------------------------------------------------------------
DOME_R = P.DOME_R
SPRING_K = SPRINGS[SPRING_DESIGN][0]


def plunger_solid(x, y, z_apex, up):
    """M4 button head (dome) + washer + shank + nyloc, dome apex at z_apex, the dome facing +z (up=True: it presses a part above) or -z"""
    s = 1.0 if up else -1.0
    hk = BOLT_HEAD_K
    cap = b.Sphere(DOME_R).moved(b.Location((0, 0, DOME_R))) & b.Cylinder(BOLT_HEAD_D / 2, hk).moved(b.Location((0, 0, hk / 2)))
    cap = cap - b.extrude(b.Plane.XY.offset(-0.1) * b.RegularPolygon(2.5 / math.sqrt(3), 6), amount=1.4)
    p = cap + cyl(SHANK_R, PL_BOLT_L, (0, 0, hk + PL_BOLT_L / 2))
    p = p + (cyl(WASHER_OD / 2, WASHER_T, (0, 0, hk + WASHER_T / 2)) - cyl(GUIDE_R, WASHER_T + 1, (0, 0, hk + WASHER_T / 2)))
    nut = b.extrude(b.RegularPolygon(NUT_AF / math.sqrt(3), 6), NUT_T).moved(b.Location((0, 0, PL_NUT_Z)))
    p = p + (nut - cyl(SHANK_R, NUT_T + 1, (0, 0, PL_NUT_Z + NUT_T / 2)))
    # built head down (apex at z 0, shank toward +z); flip for a dome facing up
    if up:
        p = p.mirror(b.Plane.XY)
    return p.moved(b.Location((x, y, z_apex)))


PL_BOLT_L = 40.0                   # set by the proposal (pl_setup)
PL_NUT_Z = 30.0


def pl_setup(bolt_l, nut_z):
    """the plunger's bolt length and its nut's position (from the apex, along the shank)"""
    global PL_BOLT_L, PL_NUT_Z
    PL_BOLT_L, PL_NUT_Z = bolt_l, nut_z


def spring_solid(x, y, z0, length):
    return cyl(SPRING_OD / 2, length, (x, y, z0 + length / 2)) - cyl(SPRING_ID / 2, length + 1, (x, y, z0 + length / 2))


def plunger_lift(a, y, z_pad, n_up):
    """how far a vertical plunger at y rises at fork angle a (deg) when its dome (radius DOME_R) stays tangent to the pad plane, which at
    nominal is z = z_pad with normal (0, n_up) pointing toward the plunger (+1: pad below the dome... the pad faces up)"""
    p0 = P.rot_pt((y, z_pad), a)
    n = P.rot_pt((y, z_pad + n_up), a) - p0
    zc0 = z_pad + n_up * DOME_R
    zc = p0[1] + (DOME_R - n[0] * (y - p0[0])) / n[1]
    return float(zc - zc0)


# ---- the wheel: unit (cached) and its swept envelope -----------------------------------------------------------------------------------
CACHE = Path(tempfile.gettempdir()) / f'pod_rx_cache_{ROOT.name}'   # per CAD version (v15 shares nothing with v14)
ROTATING = ('wheel_', 'roller_', 'bearing_687', 'bolt_', 'ak40_rotor')
WITH_FORK = ('ak40_stator', 'xt30_housing', 'xt30_pin', 'lead_', 'bearing_6804')
HARNESS = ('xt30_housing', 'lead_pos', 'lead_neg')


def wheel_unit():
    CACHE.mkdir(exist_ok=True)
    f = CACHE / 'unit.pkl'
    src = max(p.stat().st_mtime for p in (Path(W.__file__), Path(P.__file__), ROOT / 'tools' / 'dims.py'))
    if f.exists() and f.stat().st_mtime > src:
        try:
            return pickle.loads(f.read_bytes())
        except Exception:                                              # noqa: BLE001
            pass
    u = P.wheel_unit()
    try:
        f.write_bytes(pickle.dumps(u))
    except Exception:                                                  # noqa: BLE001
        pass
    return u


def surface_points(shape, step=0.6, tol=0.08):
    """points on a shape's surface, no more than ~step apart (triangles subdivided)"""
    v, t = shape.tessellate(tol, 0.2)
    Pv = np.array([[q.X, q.Y, q.Z] for q in v])
    T = np.array(t)
    if len(T) == 0:
        return Pv
    A, B_, C = Pv[T[:, 0]], Pv[T[:, 1]], Pv[T[:, 2]]
    e = np.max(np.stack([np.linalg.norm(B_ - A, axis=1), np.linalg.norm(C - B_, axis=1), np.linalg.norm(A - C, axis=1)]), axis=0)
    n = np.clip(np.ceil(e / step).astype(int), 1, 60)
    out = [Pv]
    for k in np.unique(n):
        m = n == k
        if k == 1:
            out.append((A[m] + B_[m] + C[m]) / 3)
            continue
        ij = np.array([(i, j) for i in range(k + 1) for j in range(k + 1 - i)], float) / k
        w = np.stack([1 - ij[:, 0] - ij[:, 1], ij[:, 0], ij[:, 1]], 1)
        pts = w[None, :, 0:1] * A[m][:, None, :] + w[None, :, 1:2] * B_[m][:, None, :] + w[None, :, 2:3] * C[m][:, None, :]
        out.append(pts.reshape(-1, 3))
    return np.vstack(out)


_ENV = None


def wheel_envelope(unit, dx=0.25):
    """the turning parts' swept envelope as a polygon in the (x, r) half plane (r about the axle): for each x bin the largest r of any
    turning part, and the smallest r of the turning parts' bores (the stub and motor are inside them)"""
    global _ENV
    if _ENV is not None:
        return _ENV
    f = CACHE / 'envelope.pkl'
    if f.exists() and (CACHE / 'unit.pkl').exists() and (unit is None or f.stat().st_mtime > (CACHE / 'unit.pkl').stat().st_mtime):
        _ENV = pickle.loads(f.read_bytes())
        return _ENV
    pts = np.vstack([surface_points(v, 0.5) for k, v in unit.items() if k.startswith(ROTATING)])
    x, r = pts[:, 0], np.hypot(pts[:, 1], pts[:, 2])
    xb = np.floor(x / dx).astype(int)
    k0 = xb.min()
    n = xb.max() - k0 + 1
    rmax, rmin, xmin, xmax = np.full(n, -1.0), np.full(n, 1e9), np.full(n, 1e9), np.full(n, -1e9)
    np.maximum.at(rmax, xb - k0, r)
    np.minimum.at(rmin, xb - k0, r)
    np.minimum.at(xmin, xb - k0, x)                                   # each bin spans its points' own x extent (a face on a bin edge
    np.maximum.at(xmax, xb - k0, x)                                   # must not widen the envelope by a bin), joined to its neighbours
    from shapely.geometry import box as sbox
    from shapely.ops import unary_union
    boxes = []
    for k in range(n):
        if rmax[k] < 0:
            continue
        x0 = xmin[k] if (k == 0 or rmax[k - 1] < 0) else (k + k0) * dx      # interior bins butt on the bin edges; the ends use the points
        x1 = xmax[k] if (k == n - 1 or rmax[k + 1] < 0) else (k + k0 + 1) * dx
        boxes.append(sbox(x0, rmin[k], max(x1, x0 + 1e-3), rmax[k]))
    poly = unary_union(boxes).buffer(0.01)
    f.write_bytes(pickle.dumps(poly))
    _ENV = poly
    return poly


def wheel_clearance(shape, env, a=0.0, about_pivot_moving=False, skip_r=0.0):
    """min distance from a shape's surface to the wheel's swept envelope with the fork at angle a (the shape is static unless
    about_pivot_moving: then it moves with the fork and the clearance does not depend on a)"""
    pts = surface_points(shape, 0.6)
    if a and not about_pivot_moving:                                    # static shape: put it in the fork's frame (turn it by -a)
        c, s = math.cos(math.radians(-a)), math.sin(math.radians(-a))
        y, z = pts[:, 1] - PIVOT_Y, pts[:, 2] - PIVOT_Z
        pts = np.stack([pts[:, 0], PIVOT_Y + y * c - z * s, PIVOT_Z + y * s + z * c], 1)
    xr = np.stack([pts[:, 0], np.hypot(pts[:, 1], pts[:, 2])], 1)
    xr = xr[xr[:, 1] >= skip_r]                                         # skip_r: leave out what is inside r (the 6804 stub and retainer)
    bb = env.bounds
    m = (xr[:, 0] > bb[0] - 10) & (xr[:, 0] < bb[2] + 10) & (xr[:, 1] < bb[3] + 10)
    if not m.any():
        return 99.0
    from shapely import points as spoints, distance
    d = distance(env, spoints(xr[m]))
    return float(d.min())


# ---- assembly at a pose ------------------------------------------------------------------------------------------------------------------
def pose(items, moving, a):
    """items {name: shape}: those in `moving` turned a deg about the pivot"""
    return {k: (v.rotate(PIVOT_AXIS, a) if (a and k in moving) else v) for k, v in items.items()}


def overhang_mm2(shape, up, lim=0.7072, tol=0.1):
    """area of faces pointing down more than 45 deg when the part prints with the unit vector `up` up, off the bed"""
    v, t = shape.tessellate(tol, 0.2)
    Pv, T = np.array([[q.X, q.Y, q.Z] for q in v]), np.array(t)
    A, B_, C = Pv[T[:, 0]], Pv[T[:, 1]], Pv[T[:, 2]]
    n = np.cross(B_ - A, C - A)
    ar = np.linalg.norm(n, axis=1) / 2
    u = np.asarray(up, float)
    nu = (n @ u) / np.maximum(2 * ar, 1e-12)
    h = ((A + B_ + C) / 3) @ u
    m = (nu < -lim) & (h > (Pv @ u).min() + 0.3)
    return round(float(ar[m].sum()), 1)


def bed_fit(shape, up):
    """the part on the 256 bed, printed with `up` up: print_beds.bed_fit (best rotation, 2 mm margin, as exported)"""
    import print_beds as PB
    return PB.bed_fit(shape, up)


# ---- hand loads ------------------------------------------------------------------------------------------------------------------------
def hand_loads(spring_lever, behind_pivot, n_springs=4, spring_z=0.0, bearing_span=None):
    """the forces the pod's members carry, by hand (statics of the rigid fork), for the design cases. Assumptions are listed in the result.
    spring_lever: horizontal distance pivot -> springs; behind_pivot: springs on the far side of the pivot (a tail) or ahead of it"""
    m = DESIGN_MASS
    F0 = m * 9.81 / 4                                                  # static wheel load
    Fb = 2.5 * F0                                                      # bump / drop design load (2.5 g on one wheel)
    Fx = 30.0                                                          # side hit at a threshold, at the roller (z -60)
    T = 4.1                                                            # AK40-10 peak torque (spec sheet figure, VERIFY)
    Fy = T / (R_WHEEL / 1000)
    arm = PIVOT_Y
    k = arm / spring_lever
    out = dict(assumptions=dict(robot_kg=m, static_wheel_N=round(F0, 1), bump_N=round(Fb, 1), side_hit_N=f'{Fx:g} along x at z -60 (a threshold edge)',
                                peak_traction_N=f'{Fy:.0f} (AK40-10 peak {T} N.m at R70, VERIFY)', motor_output_bearing_x='~+10 (VERIFY on the motor)'))
    for name, F in (('static', F0), ('bump', Fb)):
        S = F * k
        R = F + S if behind_pivot else S - F
        out[name] = dict(springs_total_N=round(S, 1), per_spring_N=round(S / n_springs, 1), pivot_total_N=round(R, 1), per_pivot_N=round(R / 2, 1))
    # hub load split between the 6804 (x -33) and the motor's output bearing (~x +10), contact on row 1 (x -17.25) or row 2 (+17.25)
    xa, xb = -33.0, 10.0
    split = {f'row_{i + 1}': dict(outer_6804=round((xb - xc) / (xb - xa), 2), motor=round((xc - xa) / (xb - xa), 2)) for i, xc in enumerate((-D_ROW, D_ROW))}
    out['hub_load_split'] = dict(**split, note='the motor side takes 0.4-1.2 of the wheel load, the 6804 -0.2..0.6: the fork is loaded unevenly '
                                 'whatever the springs do, so its cross members carry torsion every time the rows hand over')
    span = bearing_span or 100.0
    My, Mz = Fx * 60.0, Fx * PIVOT_Y                                    # side hit: roll and yaw moments about the pivot (N.mm)
    out['side_hit'] = dict(roll_Nm=round(My / 1000, 2), yaw_Nm=round(Mz / 1000, 2), pivot_bearing_couple_N=round(math.hypot(My, Mz) / span, 1),
                           axial_N=Fx, bearing_span_mm=span)
    out['traction'] = dict(pivot_horizontal_N=round(Fy, 1), note='pivot level with the axle: drive torque does not load the springs')
    return out


# ---- the checks every proposal runs ------------------------------------------------------------------------------------------------------
PRESS = ('bearing_627',)
FORK = ('pod_fork_outer', 'pod_fork_inner')


def run_checks(name, parts, recs_moving, recs_static, unit, info, assembly, static_printed, printed_up, pose_gaps, springs, loads):
    """assembly(parts, rm, rs, unit, a) -> (shapes, moving names, plunger lifts); static_printed: the static printed parts' names;
    pose_gaps(sh, a) -> extra {gap: mm} at a pose; springs: dict for the report; loads: hand_loads(...) result"""
    env = wheel_envelope(unit)
    rep = dict(proposal=name, poses_deg={k: round(v, 3) for k, v in POSES.items()}, build_info=info)
    for pn, a in POSES.items():
        sh, moving, lifts = assembly(parts, recs_moving, recs_static, unit, a)
        ov, press = {}, {}
        stat = [k for k in sh if k not in moving]
        plungers = [k for k in stat if k.startswith(('plunger_', 'spring_'))]
        for s in stat:
            for m in moving:
                if near(sh[s], sh[m]):
                    v = overlap(sh[s], sh[m])
                    if v > 0.01 and not (s.startswith('plunger_') and m in FORK and v < 0.05):
                        (press if (m.startswith(PRESS) or s.startswith(PRESS)) else ov)[f'{s} / {m}'] = round(v, 2)
        for p in [k for k in plungers if k.startswith('plunger_')]:
            for s in stat:
                if s in plungers or not near(sh[p], sh[s]):
                    continue
                v = overlap(sh[p], sh[s])
                if v > 0.05:
                    ov[f'{p} / {s}'] = round(v, 2)
        g = {}
        for s in static_printed:
            g[f'{s}_to_fork'] = min(min_dist(sh[s], sh[f]) for f in FORK)
            g[f'{s}_to_wheel_envelope'] = wheel_clearance(parts[s], env, a)
            g[f'{s}_to_harness'] = min(min_dist(sh[s], sh[h]) for h in HARNESS)
        g.update(pose_gaps(sh, a))
        rep[pn] = dict(axle=[round(float(v), 2) for v in P.rot_pt((0, 0), a)], interference_mm3=ov, press_fits_mm3=press,
                       gaps_mm={k: round(float(v), 2) for k, v in g.items()}, plunger_lift_mm={k: round(v, 2) for k, v in lifts.items()})
    sh, moving, _ = assembly(parts, recs_moving, recs_static, unit, 0.0)
    rep['fork_to_wheel_envelope_mm'] = {f: round(wheel_clearance(parts[f], env, 0.0, True), 2) for f in parts
                                        if f in FORK or f.startswith('rx_ring')}
    rep['fork_to_wheel_envelope_outside_r16_mm (the arms; the 6804 stub inside r 16 is unchanged from pod_lean)'] = {
        f: round(wheel_clearance(parts[f], env, 0.0, True, skip_r=16.0), 2) for f in FORK}
    rep['retainer_to_wheel_envelope_mm'] = round(wheel_clearance(parts['pod_retainer'], env, 0.0, True), 2)
    rep['fork_to_harness_mm'] = round(min(min_dist(sh[f], sh[h]) for f in FORK for h in HARNESS), 2)
    rep['fork_halves_overlap_mm3'] = round(overlap(parts[FORK[0]], parts[FORK[1]]), 3)
    tie = L.lead_tie()
    rep['lead_tie'] = None if not LEAD_TIE else dict(gap_to_motor_plate=round(min_dist(tie, parts[FORK[1]]), 2), overlap_motor_plate_mm3=round(overlap(tie, parts[FORK[1]]), 3),
                           overlap_leads_mm3=round(sum(overlap(tie, unit[k]) for k in ('lead_pos', 'lead_neg')), 3),
                           nearest_static=min((round(min_dist(tie.rotate(PIVOT_AXIS, a) if a else tie, parts[s]), 2), s, pn) for s in static_printed
                                              for pn, a in POSES.items()))
    rep['springs'] = springs
    rep['printed_parts'] = {k: dict(solids=len(v.solids()), valid=bool(v.is_valid), volume_cm3=round(v.volume / 1000, 2), print_up=printed_up[k],
                                    overhang_mm2=overhang_mm2(v, printed_up[k]), bed=bed_fit(v, printed_up[k])) for k, v in parts.items()}
    import print_beds as PB
    rep['bed_fit_failures'] = PB.bed_failures(rep['printed_parts'])
    for line in rep['bed_fit_failures']:
        print('BED FIT FAIL', line, flush=True)
    recs = recs_moving + recs_static
    up = {k: np.array(v, float) for k, v in printed_up.items()}
    rep['hole_audit'] = H.audit(recs, parts, up_of=lambda n: up.get(n))
    rep['blind_holes'] = H.blind_holes(recs, parts, up_of=lambda n: up.get(n))
    bbs = []
    for a in POSES.values():
        s2, _, _ = assembly(parts, recs_moving, recs_static, unit, a)
        bbs.append(b.Compound([v for k, v in s2.items() if k.startswith(('pod_', 'rx_', 'plunger', 'bolt_')) and not k.startswith(('bolt_1', 'bolt_2', 'bolt_cart'))]).bounding_box())
    rep['pod_envelope_swept'] = dict(x=[round(min(q.min.X for q in bbs), 1), round(max(q.max.X for q in bbs), 1)],
                                     y=[round(min(q.min.Y for q in bbs), 1), round(max(q.max.Y for q in bbs), 1)],
                                     z=[round(min(q.min.Z for q in bbs), 1), round(max(q.max.Z for q in bbs), 1)],
                                     pod_lean_for_comparison=dict(x=[-52.5, 78.6], y_back=103.0, z_top='56.9 (+ nuts 63.4)'))
    rep['hand_loads'] = loads
    return rep


def front_springs_report(n=4):
    """pod.py's front springs (two per arm, 26 mm ahead of the pivot)"""
    ns = SPRINGS_IN_SERIES                                             # v15 Nov 15: springs stacked in series in each bore
    # Nov 20: from the towers' geometry (pod.Z_CEIL, the M3 plunger's seat and lift), not from the old preload rule
    stack = {pn: round(ns * SPRING_L0 - (P.Z_CEIL - (P.Z_SEAT_M3 + max(P.LIFT_M3[pn]))), 2) for pn in POSES}
    comp = {pn: round(v / ns, 2) for pn, v in stack.items()}
    return dict(count=n, in_series_per_bore=ns, lever_mm=SPRING_ARM, where='ahead of the pivot, over each arm (pod.py plungers, dome down on the arm pad)',
                wheel_rate_N_per_mm=round(n * SPRING_K / ns * (SPRING_ARM / PIVOT_Y) ** 2, 2), compression_mm_each_spring_far_bore=comp,
                compression_mm_stack_far_bore=stack, stack_free_mm=ns * SPRING_L0, stack_solid_mm=round(ns * SPRING_SOLID, 2),
                bump_percent_of_free=round(100 * comp['bump'] / SPRING_L0, 1), long_life_limit_mm=SPRINGS[SPRING_DESIGN][1],
                stack_off_solid_at_bump_mm=round(ns * (SPRING_L0 - SPRING_SOLID) - stack['bump'], 2),
                plunger='the M3 plunger (plunger.py)', rate='ASSUMED (VERIFY: kitchen-scale test)')


# ---- outputs -----------------------------------------------------------------------------------------------------------------------------
def iso(items, out, az_deg, el, title, tol=0.35, size=(11, 9), xlim=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    a, e = np.radians(az_deg), np.radians(el)
    Rm = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]]) @ np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    light = np.array([0.35, -0.75, 0.55])
    light /= np.linalg.norm(light)
    polys = []
    for k, s in items.items():
        try:
            v, t = s.tessellate(tol, 0.3)
        except Exception:                                              # noqa: BLE001
            continue
        P3 = np.array([[q.X, q.Y, q.Z] for q in v]) @ Rm.T
        col = np.array(view_colour(k))
        for tri in np.array(t):
            Q = P3[tri]
            n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
            nn = np.linalg.norm(n)
            if nn == 0 or n[1] > 0:
                continue
            sh = 0.35 + 0.65 * max(0.0, float(-(n / nn) @ light))
            polys.append((Q[:, 1].mean(), Q[:, [0, 2]], tuple(np.clip(col * sh + 0.06, 0, 1))))
    polys.sort(key=lambda p: -p[0])
    fig, ax = plt.subplots(figsize=size)
    ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors=[p[2] for p in polys], linewidths=0.2))
    ax.autoscale()
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)


def section(items, out, plane, lims, title, size=(10, 7)):
    """a planar section: plane 'z0' (x-y at z = 0, the pivot plane), ('x', x0) (y-z at x0)"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    fig, ax = plt.subplots(figsize=size)
    for k, v in items.items():
        if plane == 'z0':
            sl = box(-150, 150, -80, 200, -0.15, 0.15)
            ij = (0, 1)
        else:
            sl = box(plane[1] - 0.15, plane[1] + 0.15, -100, 200, -100, 150)
            ij = (1, 2)
        try:
            x = v & sl
        except Exception:                                              # noqa: BLE001
            continue
        if x is None or not x.faces():
            continue
        vv, t = x.tessellate(0.05, 0.2)
        Pv = np.array([[q.X, q.Y, q.Z] for q in vv])[:, ij]
        ax.add_collection(PolyCollection([Pv[tri] for tri in np.array(t)], facecolors=[view_colour(k)], edgecolors='none'))
    ax.set_xlim(*lims[0])
    ax.set_ylim(*lims[1])
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)


def export(out_dir, printed, items_nominal, recs, asm_name):
    """parts/<name>.step (printed parts only, their filament) and <asm_name>_assembly.step (everything, bearings and fasteners magenta)"""
    (out_dir / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (out_dir / 'parts').glob('*.step'):
        f.unlink()
    for n, s in printed.items():
        write_part(s, colour(n), out_dir / 'parts' / (n + '.step'), n)
    items = {}
    for k, v in items_nominal.items():
        items[k] = (H.step_model(k, v), H.MAGENTA if k.startswith(H.STEP_HARDWARE) or k.startswith('plunger') else colour(k))
    for r in recs:
        items[r['name']] = (H.part_solid(r).moved(b.Location(b.Plane(origin=tuple(float(v) for v in r['p']), x_dir=tuple(H.pose(r)[:3, 0]),
                                                                    z_dir=tuple(H.pose(r)[:3, 2])))), H.MAGENTA)
    write_step(items, out_dir / f'{asm_name}_assembly.step', asm_name)
    return len(items)


def jdump(path, obj):
    def conv(o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        return str(o)
    Path(path).write_text(json.dumps(obj, indent=1, default=conv), encoding='utf-8')
