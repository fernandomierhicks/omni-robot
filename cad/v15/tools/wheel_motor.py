"""cad/v14 wheel + motor: the bearing roller wheel on the AK40-10, build123d, STEP review only. Dimensions in dims.py; the pod is pod_lean.py.

    python -B wheel_motor.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Baseline (Fernando, Nov 13; history in cad/v13's README: "Bearing roller wheel prototype" rounds 1-5 and "Smoother-rolling wheel"):
 * Hub: a drum round the motor's can, flange plate on the motor's output disc (3 x M2.5), a left end wall with the access bore and the cartridge
   counterbore, 45 deg cones so the cavity prints without support either end down, and a 45 deg collar round the drum between the spoke rows
   (CHANNEL_CHAMFER) so row 2's web roots grow out of the drum. Printed cartridge end down.
 * Spokes: per roller, two axle blocks (head side +z, pilot side -z) on webs from the drum. Each block is the R BLOCK_R round about its axle
   with 45 deg tangent flanks down to the row's x faces (spoke_unit): small, round, symmetric petals that print without support. The roller
   cut (band_tool) is the roller's revolved clearance envelope plus its radial drop-in channel.
 * Axle: an M3 x 50 pan head per roller, from its head counterbore (round shank hole, Ø7 counterbore, a 45 deg cone opening outward) through
   the roller into a Ø2.8 pilot in the other block.
 * Roller: a PLA core on two 687 bearings (pockets A and B) round a stationary printed sleeve, and a TPU tread keyed and snapped (V rib) onto
   the core. The sleeve's ends sit in click seats in the two thrust faces (SEAT_SLOT_*), on the seat floors (Nov 13 evening; was 0.1 oversize
   each end): the cartridge clicks into its slot and the M3 clamps the sleeve. No axial play: bearing B sits on its pocket floor, bearing A on
   the sleeve's shoulder (its pocket is deeper), so the two hold the roller between the shoulders from the inside; both 0.2 inside the core.
 * Nov 13 evening (Fernando: the M3 x 50 barely threaded; the bearings hard to get flush or inside): the sleeve 0.25 shorter each end (on the
   seat floors, and END_GAP 1.05 -> 0.9), 1.3 under the head seat (was 1.9): 6.9 mm of thread (was 6.0). B now stops on pocket B's floor
   (the core's top end as printed: that floor prints facing up) and A on the sleeve's shoulder (pocket A's floor is a ring printed over air and
   sags; A used to stop on it), both 0.2 inside the core ends (were 0.2 / 0.1, but A on a sagging floor and B with no margin); the lands 0.25 off them. The TPU tread is unchanged.
 * Tread tip relief: the last TREAD_RELIEF_LEN of each tread end falls TREAD_RELIEF_DEPTH under the R70 circle on a tangent arc.
 * Nov 13: pockets Ø14.05 and the sleeve Ø7.15 in the bearings (Fernando's printed coupon_687, cells 2 / 4); a thrust land on each thrust face
   stands LAND_GAP off its 687's static inner race (keeps the roller centred); axle and cartridge pilots run through; countersinks 0.5 deeper.
 * Cartridge: a flush plate whose boss carries the left 6804 on the pod's axle stub.
 * Motor: the AK40-10 as its own solid (stator + rotor) with its mated XT30(2+2) plug and leads (HARNESS_CLEAR to everything that turns).
Assembly of a roller (Nov 13 evening: B first): press bearing B into pocket B (the end that printed on top) down to its floor; push the sleeve in
from the A end, through B to its shoulder; press bearing A over the sleeve's other end until it stops on the shoulder; slide on the tread; press the
cartridge into its slot; fit the M3 x 50.
Everything is nominal CAD; nothing of this wheel has been printed (cad/v13's v13-hub wheel was).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from OCP.gp import gp_Trsf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dims import *                                                     # noqa: E402,F401,F403
import palette as PAL                                                  # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / 'reference'
MOTOR_STEP = REF / 'parts' / 'AK40-10 with connector.STEP'          # a copy of components/ak40-10's: this version reads only its own files
OUT = ROOT / 'step_export' / 'wheel_motor'
VAL = ROOT / 'validation' / 'wheel_motor'

COLOURS = {                         # STEP colours of the bought and mechanical parts; printed parts take their filament (palette.py)
    'wheel_hub': (0.93, 0.52, 0.20), 'wheel_cartridge': (0.66, 0.46, 0.86), 'roller_core': (0.38, 0.66, 0.38), 'roller_sleeve': (0.15, 0.55, 0.85),
    'roller_tread': (0.77, 0.77, 0.77), 'ak40_stator': (0.42, 0.43, 0.46), 'ak40_rotor': (0.78, 0.78, 0.80),
    'xt30_housing': (0.96, 0.82, 0.10), 'xt30_pin': (0.85, 0.62, 0.20), 'lead_pos': (0.85, 0.10, 0.10), 'lead_neg': (0.08, 0.08, 0.08),
    'bolt': (1.0, 0.0, 1.0), 'bearing': (0.30, 0.30, 0.30), 'motor_proxy': (0.62, 0.63, 0.66),
}


def colour(name):
    """the printed parts' filament (palette.py), else the role colours above (bought and mechanical parts)"""
    name = name.lstrip('_')
    if PAL.colour(name):
        return PAL.colour(name)
    for k, v in COLOURS.items():
        if name.startswith(k):
            return v
    return (0.7, 0.7, 0.7)


# ---- helpers --------------------------------------------------------------------------------------------------------------
def loc(M):
    t = gp_Trsf()
    t.SetValues(*[float(v) for r in np.asarray(M)[:3] for v in r])
    return b.Location(t)


def cyl(r, length, centre):
    """cylinder along local z, centred at `centre`"""
    return b.Cylinder(r, length).moved(b.Location(centre))


def zcyl(r, z0, z1):
    """cylinder along local z from z0 to z1"""
    return cyl(r, z1 - z0, (0, 0, (z0 + z1) / 2))


def along_x(r, x0, x1, y=0.0, z=0.0):
    return b.Cylinder(r, x1 - x0).rotate(b.Axis.Y, 90).moved(b.Location(((x0 + x1) / 2, y, z)))


def at_angle(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)             # (y, z)


def V(y, z):
    """point in a roller's local y-z plane"""
    return b.Vector(0, y, z)


def arc(p0, pm, p1):
    return b.Edge.make_three_point_arc(p0, pm, p1)


def line(p0, p1):
    return b.Edge.make_line(p0, p1)


def prism_x(edges, half):
    """closed y-z profile, extruded +-half along x"""
    return b.extrude(b.Face(b.Wire(edges)), amount=half, both=True)


def overlap(a, c):
    x = a & c
    return 0.0 if x is None else float(x.volume)


def near(a, c, gap=0.5):
    p, q = a.bounding_box(), c.bounding_box()
    return all(tuple(p.min)[i] - gap <= tuple(q.max)[i] and tuple(q.min)[i] - gap <= tuple(p.max)[i] for i in range(3))


def min_dist(a, c):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape(a.wrapped, c.wrapped)
    d.Perform()
    return d.Value()


def points(shape, tol=0.05):
    v, _ = shape.tessellate(tol, 0.1)
    return np.array([tuple(q) for q in v])


# ---- roller: 687 cartridge (roller-local z = its axis; side A = -z = pilot side, side B = +z = head side) ---------------------------
# Nov 13 evening: B (the core's top end as printed) stops on pocket B's floor, which prints facing up; A, pressed in last, stops on the sleeve's
# shoulder; pocket A runs POCKET_A_EXTRA past A, so A never meets its floor (a ring printed over air, which sags). Both 0.2 inside the core ends
ZB1 = HT - B_RECESS                                                         # bearing B's outer face 19.2
ZB0 = ZB1 - BRG_B                                                           # its inner face (= pocket B's floor and the sleeve's shoulder) 14.2
ZA0 = -(HT - A_RECESS)                                                      # bearing A's outer face -19.2
ZA1 = ZA0 + BRG_B                                                           # its inner face (= the sleeve's shoulder) -14.2
POCKET_B_FLOOR = ZB0
POCKET_A_FLOOR = ZA1 + POCKET_A_EXTRA                                       # -13.7: A never bottoms
SLEEVE_Z = ZF + SEAT_SLOT_D - SLEEVE_END_GAP                               # sleeve ends +-20.9, on each seat floor (evening: was 21.15, 0.1 past)
BRG_Z = ((ZA0 + ZA1) / 2, (ZB0 + ZB1) / 2)                                 # bearing centres A, B


def end_circles(s, r):
    return [e for e in s.edges() if e.geom_type.name == 'CIRCLE' and abs(abs(e.center().Z) - HT) < 1e-3 and abs(e.radius - r) < 1e-3]


def detent_ring(r_base, r_tip):
    """45 deg V ring round the roller axis at its middle, from radius r_base (the base, 2 x its depth wide) to r_tip"""
    h = abs(r_base - r_tip)
    edges = [line(b.Vector(r_base, -h, 0), b.Vector(r_tip, 0, 0)), line(b.Vector(r_tip, 0, 0), b.Vector(r_base, h, 0)),
             line(b.Vector(r_base, h, 0), b.Vector(r_base, -h, 0))]
    return b.revolve(b.Plane.XZ * b.Face(b.Wire(edges)), axis=b.Axis.Z)


def core_solid():
    """straight PLA core: keyways open at both ends (the tread slides on and off), the detent groove, end chamfers, a 687 pocket in each end
    and a clearance bore between them"""
    H = HT
    core = cyl(CORE_R, 2 * H, (0, 0, 0))
    core = b.chamfer(end_circles(core, CORE_R), CORE_CHAMFER) - zcyl(CORE_BORE_D / 2, -H - 1, H + 1)
    rp = BRG_DO / 2 + BRG_POCKET_FIT
    core = core - zcyl(rp, -H - 1, POCKET_A_FLOOR) - zcyl(rp, POCKET_B_FLOOR, H + 1)
    for a in (0, 120, 240):
        core = core - b.Box(KEY_DEPTH + 1, KEY_W, 2 * H + 2).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 1) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return core - detent_ring(CORE_R + 0.5, CORE_R - DETENT_DEPTH)


def sleeve_solid():
    """stepped sleeve: Ø SLEEVE_BRG_D (7.15, coupon_687 cell 4) through the bearings, a 45 deg step down to Ø7 just outside each bearing face for
    the hub's click seats, Ø8.6 between the inner races, Ø3.2 bore on the M3"""
    s = zcyl(BRG_D / 2, -SLEEVE_Z, SLEEVE_Z)
    s = b.chamfer([e for e in s.edges() if e.geom_type.name == 'CIRCLE' and abs(abs(e.center().Z) - SLEEVE_Z) < 1e-3], SLEEVE_LEAD)
    rb, ra = SLEEVE_BRG_D / 2, BRG_D / 2
    s = s + zcyl(rb, ZA0, ZB1)
    cone = b.Cone(rb, ra, SLEEVE_STEP_L, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN))   # z 0 (rb) .. +SLEEVE_STEP_L (ra)
    s = s + cone.moved(b.Location((0, 0, ZB1))) + cone.mirror(b.Plane.XY).moved(b.Location((0, 0, ZA0)))   # (B is pressed on over its cone)
    s = s + zcyl(BRG_IR_LAND / 2, ZA1, ZB0)
    return s - zcyl(SLEEVE_ID / 2, -SLEEVE_Z - 1, SLEEVE_Z + 1)


def bearing_687():
    """687 envelope (one ring), centred at the origin, axis z: for the assemblies and checks only (no bearing in any print STEP)"""
    return zcyl(BRG_DO / 2, -BRG_B / 2, BRG_B / 2) - zcyl(BRG_D / 2, -BRG_B / 2 - 1, BRG_B / 2 + 1)


# tread tip relief: the R_WHEEL circle (about the wheel axis) between -+RELIEF_Z0, a tangent arc of radius RELIEF_R to each end
RELIEF_R = 1.0 / (1.0 / R_WHEEL + 2.0 * TREAD_RELIEF_DEPTH / TREAD_RELIEF_LEN ** 2)   # the arc with that drop at the end (to second order)
RELIEF_Z0 = HT - TREAD_RELIEF_LEN
Z_HANDOVER = R_WHEEL * math.sin(math.radians(15.0))                       # where the two rows hand over, upright (30 deg pitch / 2)


def _relief_centre():
    """the relief arc's centre in the roller's (r, z) plane (r from the roller axis): on the R_WHEEL normal at RELIEF_Z0"""
    q = math.sqrt(R_WHEEL ** 2 - RELIEF_Z0 ** 2)
    return (q - R_AXIS) - RELIEF_R * q / R_WHEEL, RELIEF_Z0 - RELIEF_R * RELIEF_Z0 / R_WHEEL


def rt_relief(z):
    """relieved tread radius about the roller axis at local z (no end fillet)"""
    z = abs(z)
    if z <= RELIEF_Z0:
        return rt(z)
    cr, cz = _relief_centre()
    return cr + math.sqrt(RELIEF_R ** 2 - (z - cz) ** 2)


def tread_solid():
    """TPU tread: the R_WHEEL profile with the tip relief, R2 end fillets, straight bore on the core, keys along its whole length, the V rib
    that snaps into the core's groove"""
    H, Bv = HT, b.Vector
    zm = (RELIEF_Z0 + H) / 2
    edges = [arc(Bv(rt(RELIEF_Z0), -RELIEF_Z0, 0), Bv(rt(0), 0, 0), Bv(rt(RELIEF_Z0), RELIEF_Z0, 0)),
             arc(Bv(rt(RELIEF_Z0), RELIEF_Z0, 0), Bv(rt_relief(zm), zm, 0), Bv(rt_relief(H), H, 0)),
             line(Bv(rt_relief(H), H, 0), Bv(CORE_R, H, 0)), line(Bv(CORE_R, H, 0), Bv(CORE_R, -H, 0)),
             line(Bv(CORE_R, -H, 0), Bv(rt_relief(H), -H, 0)),
             arc(Bv(rt_relief(H), -H, 0), Bv(rt_relief(zm), -zm, 0), Bv(rt(RELIEF_Z0), -RELIEF_Z0, 0))]
    t = b.revolve(b.Plane.XZ * b.Face(b.Wire(edges)), axis=b.Axis.Z)
    t = b.fillet(end_circles(t, rt_relief(H)), FILLET_TREAD)
    t = b.chamfer(end_circles(t, CORE_R), CORE_CHAMFER)
    for a in (0, 120, 240):
        t = t + b.Box(KEY_DEPTH + 0.5, KEY_W, 2 * H).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 0.5) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return t + detent_ring(CORE_R + 0.01, CORE_R - DETENT_RIB)


def roller_frames():
    """[(row, k, 4x4 local -> pod)]; even rollers are turned 180 deg about local y so the two axles that share a spoke both enter from it"""
    out = []
    for row in (1, 2):
        x = -D_ROW if row == 1 else D_ROW
        for k in range(1, N_PER_ROW + 1):
            th = math.radians(60 * (k - 1) + (0 if row == 1 else 30))
            c, s = math.cos(th), math.sin(th)
            M = np.eye(4)
            M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = [1, 0, 0], [0, c, s], [0, -s, c], [x, R_AXIS * c, R_AXIS * s]
            if k % 2 == 0:
                M = M @ np.diag([-1.0, 1.0, -1.0, 1.0])
            out.append((row, k, M))
    return out


# ---- hub ---------------------------------------------------------------------------------------------------------------------
def band_tool():
    """the cut each roller makes in the spokes. Its profile in the local y-z plane: the thrust faces z = +-ZF run across the whole roller end and
    blend (BLEND_R) into the tread clearance arc (R_WHEEL + BAND_C about the wheel axis, mirrored: centre at local y = +R_AXIS), revolved about
    the roller axis (the roller's clearance envelope), plus an open channel above the axis (radial drop-in) as wide as the envelope, so the
    spoke outside the roller end is kept and wraps it"""
    big = R_WHEEL + BAND_C
    zc = ZF - BLEND_R
    yc = R_AXIS - math.sqrt((big - BLEND_R) ** 2 - zc ** 2)
    out = np.array([yc - R_AXIS, zc]) / math.hypot(yc - R_AXIS, zc)
    pts = {}
    for s in (1, -1):
        c = np.array([yc, s * zc])
        o = np.array([out[0], s * out[1]])
        tb = c + BLEND_R * o
        mid = c + BLEND_R * (o + np.array([0, s])) / np.linalg.norm(o + np.array([0, s]))
        pts[s] = (V(yc, s * ZF), V(*mid), V(*tb))
    edges = [line(V(0, -ZF), V(0, ZF)), line(V(0, ZF), pts[1][0]), arc(*pts[1]),
             arc(pts[1][2], V(R_AXIS - big, 0), pts[-1][2]), arc(pts[-1][2], pts[-1][1], pts[-1][0]), line(pts[-1][0], V(0, -ZF))]
    env = b.revolve(b.Face(b.Wire(edges)), axis=b.Axis.Z)
    half = big - R_AXIS
    top = 200.0
    return env + b.Box(2 * half, top, 2 * ZF).moved(b.Location((0, top / 2, 0)))


def pocket_tool():
    """axle head pocket: round Ø3.4 shank hole from the thrust face to the seat, round head counterbore, then a 45 deg cone opening outward
    along the axle (every face beyond the head slopes at 45 deg or less whichever way the hub prints), then the driver / head path on along
    the axle. The cone stops at FLARE_L: the two axles in a spoke cross just past their heads"""
    r = HEAD_POCKET_D / 2
    p = cyl(CLR_D / 2, SEAT - ZF + 1.0, (0, 0, (ZF - 1.0 + SEAT) / 2)) + cyl(r, B_OUT - SEAT + 0.01, (0, 0, (SEAT + B_OUT) / 2))
    p = p + b.Cone(r, r + FLARE_L, FLARE_L).moved(b.Location((0, 0, B_OUT + FLARE_L / 2)))
    return p + cyl(r, 60.0, (0, 0, B_OUT + FLARE_L + 30.0 - 0.01))


def seat_outline(neck_w, length=30.0, z=0.0):
    """the click seat's neck + channel outline (x-y, at height z), seat centre at the origin, open toward +y: neck_w wide from the centre to
    where the Ø SEAT_SLOT_W round narrows to it, then a 45 deg flare to SEAT_CHANNEL_W"""
    r = SEAT_SLOT_W / 2
    hn, hc = neck_w / 2, SEAT_CHANNEL_W / 2
    yn = math.sqrt(max(r * r - hn * hn, 0.0))
    yf = yn + (hc - hn)
    P = lambda x, y: b.Vector(x, y, z)                              # noqa: E731
    return b.Face(b.Wire.make_polygon([P(-hn, 0), P(hn, 0), P(hn, yn), P(hc, yf), P(hc, length), P(-hc, length), P(-hc, yf), P(-hn, yn)],
                                      close=True))


def seat_tool(neck_w=SEAT_NECK_W, depth=SEAT_SLOT_D, step=SEAT_STEP, length=30.0):
    """click seat + neck + channel, cut from z = 0 into +z: the Ø SEAT_SLOT_W round `depth` deep, the neck and channel `depth - step` deep"""
    return cyl(SEAT_SLOT_W / 2, depth + 0.01, (0, 0, (depth - 0.01) / 2)) + b.extrude(seat_outline(neck_w, length, -0.01), amount=depth - step + 0.01,
                                                                                       dir=(0, 0, 1))


def seat_slot_tools():
    """the click seats in both thrust faces of one roller (roller frame), open toward +y (outward, the drop-in path)"""
    t = seat_tool()
    return [t.moved(b.Location((0, 0, ZF))), t.mirror(b.Plane.XY).moved(b.Location((0, 0, -ZF)))]


LAND_TOP = (ZB1 + LAND_GAP, ZA0 - LAND_GAP)                                     # land faces (roller z): head side (B), pilot side (A)


def lands():
    """Nov 13: the thrust lands of one roller (roller frame): a ring Ø LAND_ID .. BRG_IR_LAND on each thrust face, up to LAND_GAP from its 687's
    face (on the static inner race only), cut open toward +y (the drop-in side) by the SEAT_CHANNEL_W channel"""
    out = []
    for zt, zf in ((LAND_TOP[0], ZF), (LAND_TOP[1], -ZF)):
        z0, z1 = sorted((zt, zf + math.copysign(0.3, zf)))                     # 0.3 into the block so it joins
        ring = zcyl(BRG_IR_LAND / 2, z0, z1) - zcyl(LAND_ID / 2, z0 - 1, z1 + 1)
        ring = ring - b.Box(SEAT_CHANNEL_W, 20.0, 10.0).moved(b.Location((0, 10.0, (z0 + z1) / 2)))
        out.append(ring)
    return out


def web_face(zc):
    """a spoke web's profile in the roller's y-z plane: the hull of the R BLOCK_R block circle at (0, zc) and two feet 1 mm inside the drum"""
    P = np.array([0.0, zc])
    feet = []
    for dz in (-WEB_REACH, WEB_REACH):
        z = max(-DRUM_OUT + 1, min(DRUM_OUT - 1, zc + dz))
        feet.append(np.array([-R_AXIS + math.sqrt(DRUM_OUT ** 2 - z * z) - 1.0, z]))

    def tangent(E, side):
        d = E - P
        phi, a = math.atan2(d[1], d[0]), math.acos(BLOCK_R / np.linalg.norm(d))
        return P + BLOCK_R * np.array([math.cos(phi + side * a), math.sin(phi + side * a)])
    t_lo, t_hi = tangent(feet[0], -1), tangent(feet[1], 1)
    if t_lo[1] > t_hi[1]:
        t_lo, t_hi = tangent(feet[0], 1), tangent(feet[1], -1)
    return b.Face(b.Wire([line(V(*feet[0]), V(*feet[1])), line(V(*feet[1]), V(*t_hi)), arc(V(*t_hi), V(BLOCK_R, zc), V(*t_lo)),
                          line(V(*t_lo), V(*feet[0]))]))


def side_face(xa, xb, z0):
    """a block seen along its axle (roller x-y): the R BLOCK_R round about the axle, 45 deg tangent flanks down to the row's x faces (xa < xb,
    roller x), R_FLANK_FACE into them; at z = z0"""
    br = BLOCK_R
    c = br / math.sqrt(2)
    roof = lambda x: br * math.sqrt(2) - abs(x)                     # noqa: E731
    ya, yb, y_low = roof(xa), roof(xb), -60.0
    P = lambda x, y: b.Vector(x, y, z0)                             # noqa: E731
    f = b.Face(b.Wire([line(P(xa, y_low), P(xb, y_low)), line(P(xb, y_low), P(xb, yb)), line(P(xb, yb), P(c, c)),
                       arc(P(c, c), P(0, br), P(-c, c)), line(P(-c, c), P(xa, ya)), line(P(xa, ya), P(xa, y_low))]))
    return f.fillet_2d(R_FLANK_FACE, [v for v in f.vertices() if abs(v.Y - y_low) > 1e-3 and min(abs(v.X - xa), abs(v.X - xb)) < 1e-3])


def spoke_unit(xa, xb):
    """both axle blocks of one roller (head +z, pilot -z), roller frame: the webs' wheel-plane outline run along x, cut to the side profile run
    along the axle"""
    zb, za = (ZF + B_OUT) / 2, -(ZF + A_OUT) / 2
    half = max(abs(xa), abs(xb)) + 2
    web = b.extrude(web_face(zb), amount=half, both=True) + b.extrude(web_face(za), amount=half, both=True)
    return web & b.extrude(side_face(xa, xb, -60.0), amount=120.0)


def web_x(row):
    """pod x range of a row's spokes"""
    return (-(D_ROW + WEB_OUT), -WEB_GAP / 2) if row == 1 else (WEB_GAP / 2, min(D_ROW + WEB_OUT, X_RIGHT))


def fillet_roots(hub, radii=(3.0, 2.5, 2.0, 1.5)):
    """fillet where the spoke webs' x faces meet the drum's outside cylinder"""
    planes = [x for row in (1, 2) for x in web_x(row) if abs(x - X_RIGHT) > 0.05       # a web face flush with the drum end has no root edge,
              and not (CHANNEL_CHAMFER > 0 and abs(x) < WEB_GAP / 2 + 0.05)]           # nor one standing on the collar
    arcs = [e for e in hub.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - DRUM_OUT) < 0.02
            and any(abs(e.position_at(0.5).X - p) < 0.05 for p in planes)]
    err = None
    for r in radii:
        try:
            out = b.fillet(arcs, r)
            if out.is_valid and len(out.solids()) == 1:
                return out, dict(edges=len(arcs), radius=r)
        except Exception as ex:             # noqa: BLE001
            err = str(ex)[:80]
    return hub, dict(edges=len(arcs), radius=None, error=err)


def build_hub():
    # drum, left end wall (cartridge counterbore + access bore) running out on a 45 deg cone, flange plate joined to the drum by a 45 deg cone,
    # open right end around the motor can: the cavity between the cones needs no support printed either end down; a 45 deg collar round the
    # drum under row 2's inboard web faces (CHANNEL_CHAMFER), so their roots need no support in the channel between the rows
    xc = WEB_GAP / 2
    collar = [(xc - CHANNEL_CHAMFER, DRUM_OUT), (xc, DRUM_OUT + CHANNEL_CHAMFER), (xc, DRUM_OUT)] if CHANNEL_CHAMFER > 0 else []
    prof = [(X_LEFT, DRUM_IN + 0.2), (X_LEFT, DRUM_OUT), *collar, (X_RIGHT, DRUM_OUT), (X_RIGHT, DRUM_IN), (FLANGE_BACK, DRUM_IN), (FLANGE_BACK, 0.0),
            (FLANGE_FRONT, 0.0), (FLANGE_FRONT, ACCESS_R), (FLANGE_FRONT - (DRUM_IN - ACCESS_R), DRUM_IN),
            (WALL_BACK + (DRUM_IN - ACCESS_R), DRUM_IN), (WALL_BACK, ACCESS_R), (X_PLATE_FLOOR, ACCESS_R), (X_PLATE_FLOOR, DRUM_IN + 0.2)]
    hub = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    keepout = b.Cylinder(DRUM_OUT - 0.5, 120.0).rotate(b.Axis.Y, 90)   # nothing of the spokes inside r 32.5 (the drum wall is r 30..33)
    info = dict(edge_round_failed=0)
    frames = roller_frames()
    for row, k, M in frames:
        x0, x1 = web_x(row)
        sx, xr = M[0, 0], M[0, 3]
        xa, xb = sorted(((x0 - xr) * sx, (x1 - xr) * sx))
        spoke = (spoke_unit(xa, xb).moved(loc(M)) - keepout) & b.Box(x1 - x0, 200, 200).moved(b.Location(((x0 + x1) / 2, 0, 0)))
        # round the spoke's x-end edges before it joins the drum (OCC will not fillet them on the finished hub); not the edges buried in the drum
        ends = [e for e in spoke.edges() if any(all(abs(e.position_at(t).X - p) < 1e-3 for t in (0, 0.5, 1)) for p in (x0, x1))
                and math.hypot(e.position_at(0.5).Y, e.position_at(0.5).Z) > DRUM_OUT]
        try:
            spoke = b.fillet(ends, HUB_EDGE_R)
        except Exception:                   # noqa: BLE001
            info['edge_round_failed'] += 1
        hub = hub + spoke
    hub, info['root_fillet'] = fillet_roots(hub)
    band = band_tool()
    for row, k, M in frames:
        hub = hub - band.moved(loc(M))
    solids = sorted(hub.solids(), key=lambda s: -s.volume)
    hub = solids[0]
    info['slivers_dropped_mm3'] = [round(s.volume, 2) for s in solids[1:]]
    try:                                                                    # drum's left end (outside and inside edges)
        hub = b.fillet([e for e in hub.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - X_LEFT) < 1e-3 and e.radius > DRUM_IN], HUB_EDGE_R)
    except Exception:                       # noqa: BLE001
        info['edge_round_failed'] += 1
    land = lands()
    for row, k, M in frames:                                                # Nov 13: the thrust lands (before the seats are cut)
        for t in land:
            hub = hub + t.moved(loc(M))
    z_out = (ZF + A_OUT) / 2 + BLOCK_R + 1.0                                 # past the pilot block's round end
    for row, k, M in frames:                                                # Nov 13: the pilot runs on out of the block (through)
        pilot = cyl(PILOT_D / 2, z_out - ZF, (0, 0, -(ZF + z_out) / 2))
        hub = hub - (pocket_tool() + pilot).moved(loc(M))
        for t in seat_slot_tools():
            hub = hub - t.moved(loc(M))
    for a in FLANGE_ANGLES:
        y, z = at_angle(FLANGE_R, a)
        hub = hub - along_x(M25_CLEAR / 2, FLANGE_FRONT - 0.1, FLANGE_BACK + 0.1, y, z)
    for a in CART_ANGLES:
        y, z = at_angle(CART_R, a)
        hub = hub - along_x(1.4, X_PLATE_FLOOR - 0.1, CART_PILOT_END, y, z)    # Nov 13: through the end wall's cone into the cavity
    return hub, info


CART_PILOT_END = WALL_BACK + (CART_R - ACCESS_R) + 1.0                      # the cone is at x -18.5 at r 26


# ---- cartridge (flush plate + boss carrying the left 6804) -------------------------------------------------------------------
def build_cartridge():
    """flush plate whose lip holds the left 6804's outer race, and a boss the bearing presses into from the open (+x) side: no floor, so no
    trapped cavity, and it prints plate-down without support. The 6804 floats axially (the motor's output bearing locates the wheel)."""
    x0, x1 = X_LEFT, X_PLATE_FLOOR
    brg_back = x1 + 7.0
    prof = [(x0, 13.0), (x0, DRUM_IN - 0.3), (x1, DRUM_IN - 0.3), (x1, 17.6), (brg_back, 17.6), (brg_back, R_BRG + BRG_FIT), (x1, R_BRG + BRG_FIT),
            (x1, 13.0)]
    c = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    for a in CART_ANGLES:
        y, z = at_angle(CART_R, a)
        c = c - along_x(1.7, x0 - 0.1, x1 + 0.1, y, z) - b.Cone(2.75, 1.7, 1.7).rotate(b.Axis.Y, 90).moved(b.Location((x0 + CSK_EXTRA + 0.85, y, z)))
        c = c - along_x(2.75, x0 - 0.1, x0 + CSK_EXTRA, y, z)                # Nov 13: the countersink 0.5 deeper (straight Ø5.5 over the cone)
    return c, (x1 + brg_back) / 2


# ---- motor, plug, leads ------------------------------------------------------------------------------------------------------
def motor_and_plug():
    """the real AK40-10 (surface model) and its XT30(2+2) plug: housing + the two power pins, turned MOTOR_ROT about the axis"""
    asm = b.import_step(MOTOR_STEP)
    T = b.Location(MOTOR_STEP_SHIFT)

    def place(s):
        return s.moved(T).rotate(b.Axis.X, MOTOR_ROT)
    motor = place(asm.children[0])
    solids = sorted(asm.children[1].solids(), key=lambda s: -s.volume)
    return motor, place(solids[0]), [place(s) for s in solids[1:]]


def build_motor(housing, pins):
    """own solid AK40-10: the maker's STEP is surfaces only (B-spline faces, no solid), which neither SolidWorks mates nor volume checks can use.
    Dimensions measured off that STEP: can Ø53 from the output side + 1 to the rear face, rear face 4 x M2.5 x 4.3 at r 23.5 (mounting; drawn Ø2.8 in
    the STEP) and 4 x Ø4 recessed case screws, front 6 x Ø2.6 at r 23.75; rotor = output disk Ø37 x 1 with 3 x Ø2.05 (M2.5) at r 13.5 and 3 x Ø2
    dowel holes at r 12.2, a spigot into the can. The XT30 socket is a pocket of the plug's own shape. Stator and rotor are separate so the wheel
    can turn."""
    xo, xr = MOTOR_X
    st = along_x(MOTOR_R, xo + 1.0, xr) - along_x(10.0, xo + 0.9, xo + 2.0)
    for a in REAR_HOLES_AZ:
        st = st - along_x(REAR_HOLE_D / 2, xr - REAR_HOLE_DEPTH, xr + 0.1, *at_angle(REAR_HOLES_R, a))
    for a in (-132, -42, 72, 162):
        st = st - along_x(2.0, xr - 0.5, xr + 0.1, *at_angle(23.5, a + MOTOR_ROT)) - along_x(1.1, xr - 1.4, xr, *at_angle(23.5, a + MOTOR_ROT))
    for a in range(-180, 180, 60):
        st = st - along_x(1.3, xo + 0.9, xo + 4.0, *at_angle(23.75, a + MOTOR_ROT))
    st = st - housing
    for p in pins:
        st = st - p
    ro = along_x(18.5, xo, xo + 1.0) + along_x(9.7, xo + 0.95, xo + 1.9)
    for a in FLANGE_ANGLES:
        ro = ro - along_x(1.025, xo - 0.1, xo + 2.0, *at_angle(FLANGE_R, a))
    for a in (90, -30, -150):
        ro = ro - along_x(1.0, xo - 0.1, xo + 1.6, *at_angle(12.2, a + MOTOR_ROT))
    return st, ro


def motor_proxy():
    """solid stand-in for the shell motor in volume checks and sections"""
    return along_x(MOTOR_R, MOTOR_X[0] + 1.0, MOTOR_X[1]) + along_x(18.5, MOTOR_X[0], MOTOR_X[0] + 1.1)


def build_leads(pins):
    """one lead per power pin: from inside the pin's outer end straight on along the pin, sharp 90 deg bend toward +x, short run"""
    out = []
    xh = np.array([1.0, 0, 0])
    for p in pins:
        P = points(p, 0.02)
        u, s, vt = np.linalg.svd(P - P.mean(0))
        d = vt[0] if vt[0] @ P.mean(0) > 0 else -vt[0]            # pin axis, pointing outward
        d[0] = 0.0
        d /= np.linalg.norm(d)
        proj = P @ d
        end = P[proj > proj.max() - 0.02].mean(0)
        p0, p1 = end - d * 1.0, end + d * LEAD_STRAIGHT
        cb = p1 + xh * LEAD_BEND_R
        p2 = cb + d * LEAD_BEND_R
        pm = cb + LEAD_BEND_R * (d - xh) / math.sqrt(2)
        path = b.Wire([line(b.Vector(*p0), b.Vector(*p1)), arc(b.Vector(*p1), b.Vector(*pm), b.Vector(*p2)),
                       line(b.Vector(*p2), b.Vector(LEAD_RUN_TO, p2[1], p2[2]))])
        prof = b.Plane(origin=path.position_at(0), z_dir=path.tangent_at(0)) * b.Circle(LEAD_D / 2)
        out.append(b.sweep(prof, path))
    out.sort(key=lambda s: s.center().Y)
    return out


# ---- assembly --------------------------------------------------------------------------------------------------------------------
ROLLER_PARTS = ('roller_core', 'roller_sleeve', 'roller_tread')      # the printed parts of one roller (+ two bearing_687)


def bearing_frames():
    """[(tag, 4x4 bearing -> roller frame)]: bearing A (pilot side) and B (head side)"""
    out = []
    for tag, z in zip('ab', BRG_Z):
        T = np.eye(4)
        T[2, 3] = z
        out.append((tag, T))
    return out


def build_all():
    shapes = {}
    hub, info = build_hub()
    shapes['wheel_hub'] = hub
    cart, x_brg_left = build_cartridge()
    shapes['wheel_cartridge'] = cart
    _, housing, pins = motor_and_plug()
    shapes['ak40_stator'], shapes['ak40_rotor'] = build_motor(housing, pins)
    shapes['xt30_housing'] = housing
    for i, p in enumerate(pins, 1):
        shapes[f'xt30_pin_{i}'] = p
    shapes['lead_pos'], shapes['lead_neg'] = build_leads(pins)
    roll = dict(roller_core=core_solid(), roller_sleeve=sleeve_solid(), roller_tread=tread_solid())
    brg = bearing_687()
    for k, s in roll.items():
        shapes['_' + k] = s                                               # unplaced, for the print exports
    shapes['_bearing_687'] = brg
    bolt = b.import_step(REF / 'bolt_M3_50_PAN.step')
    for row, k, M in roller_frames():
        for n, s in roll.items():
            shapes[f'{n}_{row}_{k}'] = s.moved(loc(M))
        for tag, T in bearing_frames():
            shapes[f'bearing_687_{row}_{k}_{tag}'] = brg.moved(loc(M @ T))
        shapes[f'bolt_{row}_{k}'] = bolt.moved(loc(M @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, SEAT], [0, 0, 0, 1.0]])))
    b6804 = b.import_step(REF / 'bearing_6804.step')
    shapes['bearing_6804_left'] = b6804.moved(loc(np.array([[0, 0, 1, x_brg_left], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]])))
    flat = b.import_step(REF / 'bolt_M3_12_FLAT.step')
    for i, a in enumerate(CART_ANGLES, 1):
        y, z = at_angle(CART_R, a)
        shapes[f'bolt_cartridge_{i}'] = flat.moved(loc(np.array([[0, 0, -1, X_LEFT + CSK_EXTRA], [0, 1, 0, y], [1, 0, 0, z], [0, 0, 0, 1.0]])))
    for k, s in shapes.items():
        s.label = k.lstrip('_')
        s.color = b.Color(*colour(k.lstrip('_')))
    return shapes, info


# ---- checks --------------------------------------------------------------------------------------------------------------------
def neck_widths(hub):
    """row-2 spoke at azimuth 0: material width along the tangent through the spoke centre, and x thickness, at several radii"""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN

    def inside(p):
        return BRepClass3d_SolidClassifier(hub.wrapped, gp_Pnt(*p), 1e-6).State() == TopAbs_IN

    def run(p, axis, step=0.1, lim=30):
        n = 0.0
        for sgn in (1, -1):
            d = 0.0
            while d < lim and inside(tuple(p[i] + (sgn * d if i == axis else 0) for i in range(3))):
                d += step
            n += d
        return n - step
    out = {}
    for rho in (34, 36, 38, 40, 42, 44, 46, 50, 54, 58):
        p = (D_ROW, rho, 0.0)
        out[rho] = dict(tangential=round(run(p, 2), 2), x=round(run(p, 0), 2)) if inside(p) else None
    return out


def overhang_mm2(hub):
    """downward faces steeper than 45 deg as printed cartridge end down (+x up), excluding the bed face: all, and on the spokes (r > 33.5)"""
    v, t = hub.tessellate(0.05, 0.15)
    P = np.array([tuple(q) for q in v])
    T = np.array(t)
    a, bb, c = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    n = np.cross(bb - a, c - a)
    area = 0.5 * np.linalg.norm(n, axis=1)
    n = n / np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    cen = (a + bb + c) / 3
    down = (n[:, 0] < -math.cos(math.radians(45))) & (cen[:, 0] > X_LEFT + 0.05)
    sp = down & (np.hypot(cen[:, 1], cen[:, 2]) > DRUM_OUT + 0.5)
    return round(float(area[down].sum())), round(float(area[sp].sum())), round(float(np.hypot(P[:, 1], P[:, 2]).max()), 2)


def section_radius(shape, z):
    """the solid's outer radius about the roller axis at local z, from a thin section"""
    sl = shape & b.Box(60, 60, 0.002).moved(b.Location((0, 0, z)))
    v, _ = sl.tessellate(0.002, 0.05)
    return max(math.hypot(q.X, q.Y) for q in v)


SHIELD_BORE = 9.2                    # 687ZZ shield bore, turning with the outer ring (VERIFY on the bearing): the land (Ø8.6) stays 0.3 inside it


def _inside_pt(s, p):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN
    return BRepClass3d_SolidClassifier(s.wrapped, gp_Pnt(*[float(v) for v in p]), 1e-6).State() == TopAbs_IN


def _inside_hub(hub, M, z):
    """is the roller-frame point (0, 0, z) of frame M inside the hub's plastic?"""
    return _inside_pt(hub, (M @ np.array([0, 0, z, 1.0]))[:3])


def check(shapes):
    rep = {}
    hub = shapes['wheel_hub']
    bb = hub.bounding_box()
    tp = points(shapes['roller_tread_1_1'], 0.05)
    rep['hub'] = dict(solids=len(hub.solids()), valid=bool(hub.is_valid), volume_mm3=round(hub.volume), x_range=[round(bb.min.X, 2), round(bb.max.X, 2)],
                      diameter_over_rollers=round(2 * max(np.hypot(tp[:, 1], tp[:, 2])), 2))
    rep['printed_parts'] = {k.lstrip('_'): dict(solids=len(shapes[k].solids()), valid=bool(shapes[k].is_valid), volume_mm3=round(shapes[k].volume, 1))
                            for k in ('wheel_hub', 'wheel_cartridge', '_roller_core', '_roller_sleeve', '_roller_tread')}
    # the hub against everything else: the axles thread into their pilots and the sleeves press into their seats by design, nothing else
    ov, squeeze, threads = {}, {}, {}
    for k in shapes:
        if k.startswith(('_', 'wheel_hub')) or not near(hub, shapes[k]):
            continue
        v = overlap(hub, shapes[k])
        if k.startswith('roller_sleeve_'):
            squeeze[k] = round(v, 2)
        elif k.startswith('bolt_'):
            threads[k] = round(v, 2)
        elif v > 0.01:
            ov[k] = round(v, 3)
    rep['hub_overlaps_mm3 (want none)'] = ov
    ring = math.pi * ((BRG_D / 2 - SLEEVE_LEAD) ** 2 - (SLEEVE_ID / 2) ** 2)
    rep['sleeve_in_seats_mm3 (by design: %.2f per end x its end face ~%.1f mm2 x 2 = %.1f; evening: 0, the sleeve sits on the seat floors)'
        % (-SLEEVE_END_GAP, ring, 2 * max(-SLEEVE_END_GAP, 0) * ring)] = \
        dict(min=min(squeeze.values()), max=max(squeeze.values()), sleeves=len(squeeze))
    rep['screws_overlapping_hub_mm3 (12 axles + 3 cartridge screws; want ~0)'] = dict(min=min(threads.values()), max=max(threads.values()), screws=len(threads))
    rep['seat_slots_mm'] = dict(depth=SEAT_SLOT_D, width=round(SEAT_SLOT_W, 2), side_clearance=round((SEAT_SLOT_W - BRG_D) / 2, 3),
                                neck=SEAT_NECK_W, neck_interference_per_side=round((BRG_D - SEAT_NECK_W) / 2, 3),
                                axial_click_step=SEAT_STEP, axial_pinch_per_side_entering=round(-SLEEVE_END_GAP + SEAT_STEP, 2),
                                axial_pinch_per_side_seated=round(-SLEEVE_END_GAP, 2),
                                m3x50_thread_in_pilot=round(50.0 - (SEAT + ZF + SEAT_SLOT_D), 2),
                                wall_under_head_seat=round(SEAT - ZF - SEAT_SLOT_D, 2))
    # Nov 13: the 687 envelope is one ring; its inner race (static, r < BRG_IR_LAND / 2) may meet the thrust lands, its outer race (turning) not
    # 687ZZ: inner ring OD ~Ø8.9, the shields' bore ~Ø9.2 (they turn with the outer ring, recessed ~0.2), outer ring ID ~Ø12 (all VERIFY)
    shield = zcyl(BRG_DO / 2, -BRG_B / 2, BRG_B / 2) - zcyl(SHIELD_BORE / 2, -BRG_B, BRG_B)
    outer_race = zcyl(BRG_DO / 2, -BRG_B / 2, BRG_B / 2) - zcyl(6.0, -BRG_B, BRG_B)
    fr = {(r, k): M for r, k, M in roller_frames()}
    gap = lambda rk, ring: round(min(min_dist(hub, ring.moved(loc(fr[tuple(map(int, rk.split('_')))] @ T))) for _, T in bearing_frames()), 3)   # noqa: E731
    rep['min_gaps_mm (turning parts to the hub)'] = {
        rk: dict(bearing_outer_race_r6=gap(rk, outer_race), bearing_shields_from_r4p6=gap(rk, shield),
                 core=round(min_dist(hub, shapes[f'roller_core_{rk}']), 3), tread=round(min_dist(hub, shapes[f'roller_tread_{rk}']), 3))
        for rk in ('1_1', '1_2', '2_1', '2_2')}
    rep['thrust_lands_mm (Nov 13: on the static inner race only)'] = {
        rk: dict(land_to_687_face=round(min(min_dist(hub, shapes[f'bearing_687_{rk}_{t}']) for t in 'ab'), 3),
                 id=LAND_ID, od=BRG_IR_LAND, height_head_side=round(ZF - LAND_TOP[0], 2), height_pilot_side=round(LAND_TOP[1] + ZF, 2),
                 width=round((BRG_IR_LAND - LAND_ID) / 2, 2))
        for rk in ('1_1', '2_2')}
    # one roller cartridge on its own
    core, sl, tr, brg = shapes['_roller_core'], shapes['_roller_sleeve'], shapes['_roller_tread'], [shapes['_bearing_687'].moved(loc(T)) for _, T in bearing_frames()]
    press = math.pi * ((SLEEVE_BRG_D / 2) ** 2 - (BRG_D / 2) ** 2) * BRG_B * 2
    rep['cartridge_overlaps_mm3 (core / sleeve-core want 0; sleeve / 687s = the Ø%.2f press fit on the 7.0 bores, ~%.1f by design)' % (SLEEVE_BRG_D, press)] = dict(
        core_bearings=round(sum(overlap(core, p) for p in brg), 3), sleeve_bearings=round(sum(overlap(sl, p) for p in brg), 3), sleeve_core=round(overlap(sl, core), 3))
    rep['fits'] = dict(pocket_d=round(BRG_DO + 2 * BRG_POCKET_FIT, 3), sleeve_d_in_bearings=SLEEVE_BRG_D, sleeve_d_at_seats=BRG_D, source='coupon_687 cells 2 (pocket) / 4 (pin), Nov 13')
    rep['tread_core_overlap_mm3 (keys in keyways + detent rib in groove: 0)'] = round(overlap(tr, core), 3)
    rep['bearing_stack_mm (Nov 13 evening)'] = dict(
        B_outer_face_inside_core_end=round(HT - ZB1, 2), B_on_pocket_B_floor=round(ZB0 - POCKET_B_FLOOR, 3) == 0,
        A_outer_face_inside_core_end=round(ZA0 + HT, 2), A_to_pocket_A_floor=round(POCKET_A_FLOOR - ZA1, 2),
        sleeve_shoulder_length=round(ZB0 - ZA1, 2), sleeve_length=round(2 * SLEEVE_Z, 2),
        order='B into pocket B to its floor; the sleeve in from the A end through B to its shoulder; A over the sleeve to the shoulder')
    rep['cartridge_clearances_mm'] = dict(sleeve_middle_to_core=round(min_dist(sl, core), 3), bearing_B_to_pocket_floor=round(ZB0 - POCKET_B_FLOOR, 2),
                                          sleeve_wall=round((BRG_D - SLEEVE_ID) / 2, 2),
                                          core_wall_at_pocket_to_key_slot=round(CORE_R - KEY_DEPTH - (BRG_DO / 2 + BRG_POCKET_FIT), 2))
    # tread tip relief: CAD sections against the formula
    prof = {}
    for z in (0.0, RELIEF_Z0, (RELIEF_Z0 + Z_HANDOVER) / 2, Z_HANDOVER, 18.5, 19.0):
        cad = section_radius(tr, z)
        prof[f'z {z:.2f}'] = dict(cad=round(cad, 3), formula=round(rt_relief(z), 3) if z < HT - 1.6 else None, relief=round(rt(z) - cad, 3))
    rep['tread_profile_mm (radius about the roller axis; relief = under the R70 circle)'] = prof
    rep['tread_relief'] = dict(len=TREAD_RELIEF_LEN, depth_at_end=TREAD_RELIEF_DEPTH, arc_radius=round(RELIEF_R, 2), starts_at_z=round(RELIEF_Z0, 2),
                               handover_z=round(Z_HANDOVER, 2), at_handover=round(rt(Z_HANDOVER) - rt_relief(Z_HANDOVER), 3))
    keys = [k for k in shapes if k.startswith(('roller_core_', 'roller_tread_'))]
    gaps = {}
    for i, k in enumerate(keys):
        for kk in keys[i + 1:]:
            if k.split('_')[-2:] != kk.split('_')[-2:] and near(shapes[k], shapes[kk], 1.6):
                d = min_dist(shapes[k], shapes[kk])
                if d < 2.0:
                    gaps[f'{k} / {kk}'] = round(d, 2)
    rep['roller_neighbour_gaps_mm_lt2 (closest 6)'] = dict(sorted(gaps.items(), key=lambda kv: kv[1])[:6])
    prox = motor_proxy()
    rep['rollers_vs_motor_proxy_overlap_mm3'] = round(sum(overlap(prox, shapes[k]) for k in keys if near(prox, shapes[k], 0)), 2)
    # harness: plug housing, pins and leads against everything that turns
    hz = ['xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg']
    rotating = [k for k in shapes if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_'))]
    worst = (1e9, '')
    for h in hz:
        for k in rotating:
            if near(shapes[h], shapes[k], HARNESS_CLEAR + 1):
                d = min_dist(shapes[h], shapes[k])
                worst = min(worst, (d, f'{h} - {k}'))
    rep['harness'] = {f'closest rotating part (want >= {HARNESS_CLEAR:g})': dict(gap=round(worst[0], 2) if worst[1] else 'none within 5 mm', pair=worst[1])}
    xs = [s.bounding_box() for k, s in shapes.items() if not k.startswith(('_', 'lead_'))]
    rep['x_range_hub_to_motor'] = [round(min(q.min.X for q in xs), 2), round(max(q.max.X for q in xs), 2)]
    rep['motor_x_output_rear'] = list(MOTOR_X)
    tun = {}
    for row, k, M in roller_frames():
        tool = cyl(AXLE_HEAD_D / 2 - 0.05, 70.0, (0, 0, SEAT + 35.0)).moved(loc(M))
        hits = {}
        for kk in shapes:
            if kk.startswith(('roller_', 'bolt_', 'wheel_cartridge', 'bearing_687')) and not kk.endswith(f'_{row}_{k}') and f'_{row}_{k}_' not in kk \
                    and near(tool, shapes[kk], 0):
                v = overlap(tool, shapes[kk])
                if v > 0.05:
                    hits[kk] = round(v, 2)
        tun[f'{row}_{k}'] = hits
    rep['axle_insertion_tunnel_hits_mm3 (want none)'] = {k: v for k, v in tun.items() if v}
    # radial press-in of the whole cartridge (core + tread + bearings + sleeve) along its slot
    drop = {}
    frames = {(r, k): M for r, k, M in roller_frames()}
    for rk in ((1, 1), (1, 2), (2, 1), (2, 2)):
        M = frames[rk]
        tag = f'{rk[0]}_{rk[1]}'
        for d in (0.0, 10.0, 20.0, 30.0):
            T = np.eye(4)
            T[:3, 3] = M[:3, 1] * d
            mv = loc(T)
            drop[f'{tag} +{d:g}'] = dict(
                core_tread_bearings=round(sum(overlap(hub, shapes[n].moved(mv)) for n in (f'roller_core_{tag}', f'roller_tread_{tag}',
                                                                                        f'bearing_687_{tag}_a', f'bearing_687_{tag}_b')), 2),
                sleeve=round(overlap(hub, shapes[f'roller_sleeve_{tag}'].moved(mv)), 2))
    rep['cartridge_radial_press_in_mm3 (core_tread_bearings want 0; sleeve = its pinch between the thrust faces along the slot)'] = drop
    zb, za = (ZF + B_OUT) / 2, (ZF + A_OUT) / 2
    rep['block_walls_mm'] = dict(
        head_counterbore_side_profile=round(BLOCK_R - HEAD_POCKET_D / 2, 2),
        head_counterbore_rim_wheel_plane=round(math.sqrt(BLOCK_R ** 2 - (B_OUT - zb) ** 2) - HEAD_POCKET_D / 2, 2),
        pilot_thread_side_profile=round(BLOCK_R - PILOT_D / 2, 2),
        pilot_thread_end_wheel_plane=round(math.sqrt(BLOCK_R ** 2 - (ZF + PILOT_DEPTH - za) ** 2) - PILOT_D / 2, 2),
        pilot='Nov 13: through the block (was a blind floor 2.95 beyond the thread)',
        pilot_open_past_block=not _inside_hub(hub, roller_frames()[0][2], -(za + BLOCK_R + 0.5)))
    rep['cartridge_screws'] = dict(countersink_extra=CSK_EXTRA, pilot_x=[X_PLATE_FLOOR, round(CART_PILOT_END, 2)], pilot='through the end wall into the cavity',
                                   pilot_open_at_end=not any(_inside_pt(hub, (CART_PILOT_END - 0.6, *at_angle(CART_R, a))) for a in CART_ANGLES))
    o = overhang_mm2(hub)
    rep['hub_print_overhang_mm2 (cartridge end down, faces under 45 deg)'] = dict(all=o[0], spokes=o[1])
    rep['hub_max_radius_mm'] = o[2]
    rep['spoke_neck_row2_az0_mm'] = neck_widths(hub)
    rep['roller_angular_overlap_deg'] = round(2 * math.degrees(math.asin(HT / R_WHEEL)) - 30, 2)
    return rep


# ---- export --------------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_', 'bearing_', '_')          # no fasteners or bearings in any STEP; '_' = unplaced print copies
PRINTED = ('wheel_hub', 'wheel_cartridge', '_roller_core', '_roller_sleeve', '_roller_tread')


def export(shapes):
    parts = OUT / 'parts'
    for f in list(parts.glob('*.step')) + list(OUT.glob('*.step')):
        f.unlink()
    for k in PRINTED:
        n = k.lstrip('_')
        write_part(shapes[k], colour(n), parts / (n + '.step'), n)
    write_step({k: (s, colour(k)) for k, s in shapes.items() if not k.startswith(STEP_SKIP)}, OUT / 'wheel_motor_assembly.step', 'wheel_motor')


# ---- render --------------------------------------------------------------------------------------------------------------------
def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    def role(k):
        """pictures use the role colours (core green, sleeve blue, 687 grey, tread dark, hub orange): the filament greys look alike"""
        n = k.lstrip('_')
        return next((v for p, v in COLOURS.items() if n.startswith(p)), colour(k))

    def section(items, transform, ax, xlim, ylim, title, slab_t=0.6, tol=0.08):
        slab = b.Box(400, 400, slab_t)
        polys = []
        for k, s in items.items():
            x = transform(s) & slab
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(tol, 0.2)
            Vv = np.array([[q.X, q.Y] for q in v])
            polys += [(Vv[tri], role(k)) for tri in np.array(t)]
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(title, fontsize=9)

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(VAL / (name + '.png'), dpi=95)
        plt.close(fig)

    solid = {k: s for k, s in shapes.items() if not k.startswith(('_', 'bolt_'))}
    fig, ax = plt.subplots(figsize=(13, 9))
    section({k: s for k, s in solid.items() if not k.startswith('roller_tread_')}, lambda s: s, ax, (-48, 72), (-68, 68),
            'axis plane z = 0 (x along the wheel axis): hub, cartridge, motor, roller cores, sleeves and 687s')
    save(fig, 'section_axis_plane_z0')
    fig, ax = plt.subplots(figsize=(10, 10))
    section({k: s for k, s in solid.items() if k == 'wheel_hub' or k.startswith(('roller_', 'bearing_687'))},
            lambda s: s.rotate(b.Axis.Y, 90).moved(b.Location((0, 0, D_ROW))), ax, (-75, 75), (-75, 75), 'across the row-2 spokes at x = +17.25')
    save(fig, 'section_row2_plane')
    # one roller in its slot, cut along its axle, and the click seat
    M = {(r, k): M for r, k, M in roller_frames()}[(1, 1)]
    inv = loc(M).inverse()
    items = {k: shapes[k] for k in ('wheel_hub', 'roller_core_1_1', 'roller_tread_1_1', 'roller_sleeve_1_1', 'bearing_687_1_1_a', 'bearing_687_1_1_b')}
    along = lambda s: s.moved(inv).rotate(b.Axis.Y, 90)            # noqa: E731  roller axis across the page (pilot side right)
    fig, axs = plt.subplots(1, 2, figsize=(16, 6))
    section(items, along, axs[0], (-34, 34), (-24, 24), 'roller 1_1 cut along its axle: 687s grey, sleeve blue, core green, tread dark, hub orange', tol=0.04)
    section(items, along, axs[1], (12, 23), (-9, 9), 'one end: sleeve shoulder on the inner race, sleeve end 0.1 into its click seat (press fit)', tol=0.02)
    save(fig, 'section_roller_cartridge')
    seat_items = {'wheel_hub': shapes['wheel_hub'], 'roller_sleeve_1_1': shapes['roller_sleeve_1_1']}
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    for ax, dz, ttl in ((axs[0], 0.2, 'across the axle 0.2 into the head block face: the Ø7.1 seat, 6.9 neck, channel'),
                        (axs[1], 0.45, '0.45 in: only the round seat (the channel is 0.4 deep)')):
        section(seat_items, lambda s, dz=dz: s.moved(inv).moved(b.Location((0, 0, -(ZF + dz)))), ax, (-9, 9), (-6, 12), ttl, slab_t=0.04, tol=0.02)
    save(fig, 'section_click_seat')
    # tread profile against the R70 circle
    z = np.linspace(-HT, HT, 400)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(z, [rt(q) for q in z], lw=1, label='R70 circle (v13 tread)')
    ax.plot(z, [rt_relief(q) for q in z], lw=1.5, label=f'v14 tread: tip relief {TREAD_RELIEF_DEPTH} over the last {TREAD_RELIEF_LEN:g} mm')
    ax.axvline(Z_HANDOVER, ls=':', c='k', lw=.8)
    ax.axvline(-Z_HANDOVER, ls=':', c='k', lw=.8)
    ax.set_xlabel('along the roller (mm)')
    ax.set_ylabel('radius about the roller axis (mm)')
    ax.set_ylim(rt(HT) - 0.4, rt(0) + 0.2)
    ax.legend(fontsize=8)
    ax.grid(True, lw=.3)
    ax.set_title('tread profile (before the R2 end fillets); dotted: the upright row handover')
    save(fig, 'tread_profile')


def main(stage):
    for d in (OUT / 'parts', VAL):
        d.mkdir(parents=True, exist_ok=True)
    shapes, info = build_all()
    print('hub build', info, flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes)
        rep['hub_build'] = info
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1))
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
