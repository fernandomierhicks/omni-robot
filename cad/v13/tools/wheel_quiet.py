"""Quieter wheel prototype (Nov 11 2026, Fernando). Proposal geometry for a print test, NOT the baseline: nothing here is
wired into the pod / robot assemblies. Generic wheel = the v13 wheel_motor hub and treads; no motor, cartridge or mounting
changes.

    python -B wheel_quiet.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Why (simulation/v11 sil/studies/wheel_noise.py, results/wheel_noise_2026-11-11.md): the clicks are the rollers' play on
their axles: radial (Ø3.4 bore on the M3) and axial (0.3 + 0.3 thrust gaps). Removing both is what counts.

Round 2 (Fernando: a bearing in each roller end from his kit -- 693 623 624 685 687 627 -- with a printed tube round the
M3 that fits the bearing bore, keep the M3 axle fitting and removal; 0.4 mm nozzle):
  * 687 (7 x 14 x 5): a 7 mm bore leaves a 1.8 mm sleeve wall round a Ø3.2 hole (four 0.45 perimeters); the 685's 5 mm
    leaves 0.9. Its Ø14 sits in the Ø25.4 core with 4.15 mm to the key slots.
  * roller cartridge = core + two 687 + a stepped printed sleeve. The sleeve is NOT part of the hub: a spigot from either
    block into a bearing would stop the roller dropping into its slot radially. Sleeve: Ø7 ends through the bearings, a
    Ø8.6 middle (inner races only) between them; its ends reach the block thrust faces, and the same M3 x 50 clamps it
    between the blocks, so it is the stationary axle and the screw only holds it. Round 4 (Fernando, 687ZZ bought): the
    sleeve is 0.1 too long at each end (SLEEVE_END_GAP -0.1, a 0.3 lead-in on its ends), so the cartridge presses into its
    slot and the blocks pinch the sleeve even before the screw is in; 1.0 radial clearance between the sleeve's middle and
    the core (CORE_BORE_D 10.6, was 0.5); the M3 thread pilots in the hub are Ø2.8 (PILOT_D_Q, v13 2.9).
  * no axial play: pocket A (pilot side) is bearing deep + 0.2 and bearing A sits on its floor; pocket B is 0.5 deeper
    than the bearing's place, so bearing B, pressed in last, stops on the sleeve's shoulder, not the pocket floor. The two
    bearings then hold the roller between the sleeve's shoulders from the inside (back to back); what is left is the
    687s' own internal play. Assembly: A into pocket A, sleeve through A until its shoulder meets the inner race, B over
    the sleeve's other end, pressed until it stops; slide the tread on; drop in; M3 through.
  * the core ends sit 1.05 off the block faces (the old journal + thrust gap), the bearings 0.15 / 0.1 further in.
  * fit_coupon_687: three pockets Ø14.0 / 14.1 / 14.2 and three pins Ø6.9 / 7.0 / 7.1, to pick the printed sizes first.
Round 3 (Fernando: the round-2 cut-outs looked ugly; remove the material round the screws that is not needed, gentle
rounded transitions, symmetric both sides of each petal, no supports; "go back to the feature that created the petals and
reduce its size"): the spokes are no longer cut after the fact. Each axle block -- wm.web()'s hull of a block circle and
two drum feet in the wheel plane, run along the wheel axis across the row -- is built smaller and shaped:
  * block radius BLOCK_R_Q 6 (v13: 8): the tips come in from r 65.6 to 63.3, the lobes stay round;
  * seen along its axle, each block is the R6 round about the axle with 45 deg tangent flanks down to both x faces
    (symmetric about the axle), R_FLANK_FACE into them: the corners that stood out past the round are gone, and 45 deg
    prints without support;
  * the separate R8 block cylinders along the axles are gone (the R6 round is now the side profile's crest);
  * walls: >= 2.5 round the Ø7 head counterbore (1.9 at its rim in the wheel plane), >= 4.5 round the pilot thread; the
    spokes' necks below r 54 are unchanged; the pockets (counterbore, 45 deg flare, driver tunnel, pilot) are v13's.
Round 2's cuts (r 62.5 tips and an outboard cone, same day) and round 1 (623s, hub bosses, TPU washer) are superseded.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_motor as wm                                                # noqa: E402
from dims import *                                                      # noqa: E402,F401,F403
from step_colour import write_part, write_step                          # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'step_export' / 'wheel_quiet'
VAL = ROOT / 'validation' / 'wheel_quiet'

# ---- round-2 dimensions (mm) --------------------------------------------------------------------------------------------------
BRG_D, BRG_DO, BRG_B = 7.0, 14.0, 5.0     # 687 (Fernando's kit)
BRG_IR_LAND = 8.6                          # sleeve shoulder Ø: on the inner race only (687 inner race land ~Ø8.8-9.1, VERIFY)
BRG_FIT_R = 0.05                           # pocket radius allowance (as dims.BRG_FIT); fit_coupon_687 picks the real one
POCKET_A_EXTRA = 0.2                       # pocket A: bearing A sits 0.2 below the core end, on the pocket floor
POCKET_B_EXTRA = 0.5                       # pocket B: 0.5 deeper than bearing B's place on the shoulder (it never bottoms)
B_RECESS = 0.1                             # bearing B's outer face this far below the core end
CORE_BORE_D = BRG_IR_LAND + 2.0            # core between the pockets: 1.0 radial clearance round the sleeve's middle (Fernando; was 0.5)
SLEEVE_ID = 3.2                            # on the M3 (prints ~3.05-3.1: snug; drill 3.2 if it binds)
SLEEVE_END_GAP = -0.1                      # each sleeve end 0.1 PAST its block face (Fernando: oversize, so it cannot rattle between the
                                           # blocks): the cartridge is pressed into its slot, the spokes spring 0.2 apart and pinch it
SLEEVE_LEAD = 0.2                          # 45 deg lead-in on the sleeve ends' outer edges, to start the press-in
SEAT_SLOT_D = 0.5                          # round 5 (Fernando): each thrust face has a seat this deep that the sleeve end sits in, so
SEAT_SLOT_W = BRG_D + 0.1                  # the sleeve cannot shift on the M3's play: a round Ø7.1 about the axle (0.05 a side), open
                                           # radially outward (the drop-in path) through a channel. 0.5 deep keeps 6.0 of M3 x 50 thread
                                           # in the pilot and 1.9 under the head seat
SEAT_NECK_W = BRG_D - 0.1                  # round 5 (Fernando: a pure U lets it rattle out): the round closes to this neck just past the
                                           # sleeve's centre (a C, 0.05 interference a side) before the channel opens out at 45 deg ...
SEAT_STEP = 0.1                            # ... and the channel and neck are this much shallower than the round, so the sleeve, 0.1
                                           # oversize, rides in pinched 0.2 a side and clicks down into the round (fit_coupon_687 tunes
                                           # the neck: 6.8 / 6.9 / 7.0)
SEAT_CHANNEL_W = BRG_D + 0.3               # channel width beyond the neck
PILOT_D_Q = 2.8                            # M3 thread pilot in the hub (Fernando: 2.8, the project's M3 rule; v13 roller axle: 2.9)
BLOCK_R_Q = 6.0                            # round 3: axle block radius (v13 dims.BLOCK_R 8)
R_FLANK_FACE = 2.5                         # round 3: 45 deg block flanks into the spoke's x faces
COUPON_POCKETS = (14.0, 14.1, 14.2)
COUPON_PINS = (6.9, 7.0, 7.1)

# roller-local z (axis), side A = -z (pilot side), side B = +z (head side)
ZA0, ZA1 = -(HT - POCKET_A_EXTRA), -(HT - POCKET_A_EXTRA - BRG_B)          # bearing A: -19.3 .. -14.3
ZB1 = HT - B_RECESS                                                         # bearing B outer face 19.4
ZB0 = ZB1 - BRG_B                                                           # its inner face (= the shoulder) 14.4
POCKET_A_FLOOR = ZA1                                                        # -14.3
POCKET_B_FLOOR = ZB0 - POCKET_B_EXTRA                                       # 13.9
SLEEVE_Z = ZF + SEAT_SLOT_D - SLEEVE_END_GAP                               # sleeve ends +-21.15: 0.1 past each slot floor

COLOURS = {'wheel_hub': (0.93, 0.52, 0.20), 'roller_core': (0.38, 0.66, 0.38), 'roller_tread': (0.20, 0.24, 0.32),
           'roller_sleeve': (0.15, 0.55, 0.85), 'bearing_687': (0.30, 0.30, 0.30), 'fit_coupon': (0.75, 0.75, 0.30)}


def colour(name):
    name = name.lstrip('_')
    for k, v in COLOURS.items():
        if name.startswith(k):
            return v
    return (0.7, 0.7, 0.7)


def zcyl(r, z0, z1):
    return wm.cyl(r, z1 - z0, (0, 0, (z0 + z1) / 2))


def core_687():
    """straight PLA core as wm.core_solid() (keys, detent groove, end chamfers) with 687 pockets and a clearance bore"""
    H = HT
    core = wm.cyl(CORE_R, 2 * H, (0, 0, 0))
    core = b.chamfer(wm.end_circles(core, CORE_R), CORE_CHAMFER) - zcyl(CORE_BORE_D / 2, -H - 1, H + 1)
    rp = BRG_DO / 2 + BRG_FIT_R
    core = core - zcyl(rp, -H - 1, POCKET_A_FLOOR) - zcyl(rp, POCKET_B_FLOOR, H + 1)
    for a in (0, 120, 240):
        core = core - b.Box(KEY_DEPTH + 1, KEY_W, 2 * H + 2).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 1) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return core - wm.detent_ring(CORE_R + 0.5, CORE_R - DETENT_DEPTH)


def sleeve_687():
    """stepped sleeve: Ø7 ends through the bearings to the block faces, Ø8.6 between the inner races"""
    s = zcyl(BRG_D / 2, -SLEEVE_Z, SLEEVE_Z)
    s = b.chamfer([e for e in s.edges() if e.geom_type.name == 'CIRCLE' and abs(abs(e.center().Z) - SLEEVE_Z) < 1e-3], SLEEVE_LEAD)
    s = s + zcyl(BRG_IR_LAND / 2, ZA1, ZB0)
    return s - zcyl(SLEEVE_ID / 2, -SLEEVE_Z - 1, SLEEVE_Z + 1)


def bearings_687():
    """687 envelopes in place, for checks and pictures only (never exported)"""
    return [zcyl(BRG_DO / 2, z0, z1) - zcyl(BRG_D / 2, z0 - 1, z1 + 1) for z0, z1 in ((ZA0, ZA1), (ZB0, ZB1))]


COUPON_NECKS = (6.8, 6.9, 7.0)


def seat_outline(neck_w, length=30.0, z=0.0):
    """the channel + neck outline (x-y, at height z), seat centre at the origin, open toward +y: SEAT_NECK_W wide from the
    centre to where the Ø SEAT_SLOT_W round narrows to it, then a 45 deg flare to SEAT_CHANNEL_W"""
    r = SEAT_SLOT_W / 2
    hn, hc = neck_w / 2, SEAT_CHANNEL_W / 2
    yn = math.sqrt(max(r * r - hn * hn, 0.0))
    yf = yn + (hc - hn)
    P = lambda x, y: b.Vector(x, y, z)                              # noqa: E731
    return b.Face(b.Wire.make_polygon([P(-hn, 0), P(hn, 0), P(hn, yn), P(hc, yf), P(hc, length), P(-hc, length), P(-hc, yf), P(-hn, yn)],
                                      close=True))


def seat_tool(neck_w, depth, step, length=30.0):
    """seat + click neck + channel, cut from z = 0 into +z: the Ø SEAT_SLOT_W round `depth` deep, the neck and channel `depth -
    step` deep"""
    return wm.cyl(SEAT_SLOT_W / 2, depth + 0.01, (0, 0, (depth - 0.01) / 2)) + b.extrude(seat_outline(neck_w, length, -0.01), amount=depth - step + 0.01,
                                                                                          dir=(0, 0, 1))


def fit_coupon():
    """bearing pockets (Ø14.0 / 14.1 / 14.2 x 5.6), sleeve pins (Ø6.9 / 7.0 / 7.1 x 6) and the sleeve-end click seats (the
    hub's Ø7.1 x 0.5 round with necks 6.8 / 6.9 / 7.0, 0.4 deep channels open to the plate's +y edge) on one 80 x 30 x 7 plate"""
    plate = b.Box(80, 30, 7).moved(b.Location((0, 0, 3.5)))
    for i, d in enumerate(COUPON_POCKETS):
        plate = plate - wm.cyl(d / 2, 5.6, (-29 + 11 * i, -6, 7 - 2.8 + 0.01))
    for i, d in enumerate(COUPON_PINS):
        plate = plate + wm.cyl(d / 2, 6.0, (8 + 11 * i, -6, 7 + 3.0))
    for i, w in enumerate(COUPON_NECKS):
        t = seat_tool(w, SEAT_SLOT_D, SEAT_STEP, 10.0).mirror(b.Plane.XY).moved(b.Location((-22 + 22 * i, 7, 7)))
        plate = plate - t
    return plate


def seat_slot_tools():
    """the click seats in both thrust faces of one roller (roller frame), open toward +y (outward)"""
    t = seat_tool(SEAT_NECK_W, SEAT_SLOT_D, SEAT_STEP)
    return [t.moved(b.Location((0, 0, ZF))), t.mirror(b.Plane.XY).moved(b.Location((0, 0, -ZF)))]


def web_face(zc, br=BLOCK_R_Q):
    """wm.web()'s profile (roller y-z: hull of the block circle at (0, zc) and two feet 1 mm inside the drum) with block radius br"""
    P = np.array([0.0, zc])
    feet = []
    for dz in (-WEB_REACH, WEB_REACH):
        z = max(-DRUM_OUT + 1, min(DRUM_OUT - 1, zc + dz))
        feet.append(np.array([-R_AXIS + math.sqrt(DRUM_OUT ** 2 - z * z) - 1.0, z]))

    def tangent(E, side):
        d = E - P
        phi, a = math.atan2(d[1], d[0]), math.acos(br / np.linalg.norm(d))
        return P + br * np.array([math.cos(phi + side * a), math.sin(phi + side * a)])
    t_lo, t_hi = tangent(feet[0], -1), tangent(feet[1], 1)
    if t_lo[1] > t_hi[1]:
        t_lo, t_hi = tangent(feet[0], 1), tangent(feet[1], -1)
    V = wm.V
    return b.Face(b.Wire([wm.line(V(*feet[0]), V(*feet[1])), wm.line(V(*feet[1]), V(*t_hi)), wm.arc(V(*t_hi), V(br, zc), V(*t_lo)),
                          wm.line(V(*t_lo), V(*feet[0]))]))


def side_face(xa, xb, z0, br=BLOCK_R_Q):
    """a block seen along its axle (roller x-y): the br round about the axle, 45 deg tangent flanks down to the row's x faces
    (xa < xb, roller x), R_FLANK_FACE into them; at z = z0"""
    c = br / math.sqrt(2)
    roof = lambda x: br * math.sqrt(2) - abs(x)                     # noqa: E731
    ya, yb, y_low = roof(xa), roof(xb), -60.0
    P = lambda x, y: b.Vector(x, y, z0)                             # noqa: E731
    f = b.Face(b.Wire([b.Edge.make_line(P(xa, y_low), P(xb, y_low)), b.Edge.make_line(P(xb, y_low), P(xb, yb)),
                       b.Edge.make_line(P(xb, yb), P(c, c)), b.Edge.make_three_point_arc(P(c, c), P(0, br), P(-c, c)),
                       b.Edge.make_line(P(-c, c), P(xa, ya)), b.Edge.make_line(P(xa, ya), P(xa, y_low))]))
    return f.fillet_2d(R_FLANK_FACE, [v for v in f.vertices() if abs(v.Y - y_low) > 1e-3 and min(abs(v.X - xa), abs(v.X - xb)) < 1e-3])


def spoke_unit(xa, xb):
    """both axle blocks of one roller (head +z, pilot -z), roller frame: the webs' wheel-plane outline run along x, cut to the
    side profile run along the axle. Replaces wm.build_hub()'s R8 block cylinders + webs"""
    zb, za = (ZF + B_OUT) / 2, -(ZF + A_OUT) / 2
    half = max(abs(xa), abs(xb)) + 2
    web = b.extrude(web_face(zb), amount=half, both=True) + b.extrude(web_face(za), amount=half, both=True)
    return web & b.extrude(side_face(xa, xb, -60.0), amount=120.0)


def build_hub_687(round3=True):
    """wm.build_hub() step for step, with spoke_unit() in place of its block cylinders + webs (round3=False: v13's hub)"""
    if not round3:
        return wm.build_hub()
    xc = WEB_GAP / 2
    collar = [(xc - CHANNEL_CHAMFER, DRUM_OUT), (xc, DRUM_OUT + CHANNEL_CHAMFER), (xc, DRUM_OUT)] if CHANNEL_CHAMFER > 0 else []
    prof = [(X_LEFT, DRUM_IN + 0.2), (X_LEFT, DRUM_OUT), *collar, (X_RIGHT, DRUM_OUT), (X_RIGHT, DRUM_IN), (FLANGE_BACK, DRUM_IN), (FLANGE_BACK, 0.0),
            (FLANGE_FRONT, 0.0), (FLANGE_FRONT, ACCESS_R), (FLANGE_FRONT - (DRUM_IN - ACCESS_R), DRUM_IN),
            (WALL_BACK + (DRUM_IN - ACCESS_R), DRUM_IN), (WALL_BACK, ACCESS_R), (X_PLATE_FLOOR, ACCESS_R), (X_PLATE_FLOOR, DRUM_IN + 0.2)]
    hub = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    keepout = b.Cylinder(DRUM_OUT - 0.5, 120.0).rotate(b.Axis.Y, 90)
    info = dict(edge_round_failed=0)
    frames = wm.roller_frames()
    for row, k, M in frames:
        x0, x1 = wm.web_x(row)
        sx, xr = M[0, 0], M[0, 3]
        xa, xb = sorted(((x0 - xr) * sx, (x1 - xr) * sx))
        spoke = (spoke_unit(xa, xb).moved(wm.loc(M)) - keepout) & b.Box(x1 - x0, 200, 200).moved(b.Location(((x0 + x1) / 2, 0, 0)))
        ends = [e for e in spoke.edges() if any(all(abs(e.position_at(t).X - p) < 1e-3 for t in (0, 0.5, 1)) for p in (x0, x1))
                and math.hypot(e.position_at(0.5).Y, e.position_at(0.5).Z) > DRUM_OUT]
        try:
            spoke = b.fillet(ends, HUB_EDGE_R)
        except Exception:                     # noqa: BLE001
            info['edge_round_failed'] += 1
        hub = hub + spoke
    hub, info['root_fillet'] = wm.fillet_roots(hub)
    band = wm.band_tool()
    for row, k, M in frames:
        hub = hub - band.moved(wm.loc(M))
    solids = sorted(hub.solids(), key=lambda s: -s.volume)
    hub = solids[0]
    info['slivers_dropped_mm3'] = [round(s.volume, 2) for s in solids[1:]]
    try:
        hub = b.fillet([e for e in hub.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - X_LEFT) < 1e-3 and e.radius > DRUM_IN], HUB_EDGE_R)
    except Exception:                         # noqa: BLE001
        info['edge_round_failed'] += 1
    for row, k, M in frames:
        pilot = wm.cyl(PILOT_D_Q / 2, PILOT_DEPTH, (0, 0, -(ZF + PILOT_DEPTH / 2)))
        hub = hub - (wm.pocket_tool() + pilot).moved(wm.loc(M))
        for t in seat_slot_tools():
            hub = hub - t.moved(wm.loc(M))
    for a in FLANGE_ANGLES:
        y, z = wm.at_angle(FLANGE_R, a)
        hub = hub - wm.along_x(M25_CLEAR / 2, FLANGE_FRONT - 0.1, FLANGE_BACK + 0.1, y, z)
    for a in CART_ANGLES:
        y, z = wm.at_angle(CART_R, a)
        hub = hub - wm.along_x(1.4, X_PLATE_FLOOR - 0.1, X_PLATE_FLOOR + 10.0, y, z)
    return hub, info


def overhang_mm2(hub):
    """downward faces steeper than 45 deg as printed cartridge end down (+x up), excluding the bed face: all, and on the
    spokes (r > 33.5)"""
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


def build_all():
    shapes = {}
    hub, info = build_hub_687(True)
    shapes['wheel_hub_687'] = hub
    shapes['_wheel_hub_v13'] = wm.build_hub()[0]                    # untrimmed, for the before / after picture only
    core, tread, sl = core_687(), wm.tread_solid(), sleeve_687()
    shapes['_roller_core_687'], shapes['_roller_tread'], shapes['_roller_sleeve_687'] = core, tread, sl
    shapes['_fit_coupon_687'] = fit_coupon()
    for row, k, M in wm.roller_frames():
        L = wm.loc(M)
        shapes[f'roller_core_687_{row}_{k}'] = core.moved(L)
        shapes[f'roller_tread_{row}_{k}'] = tread.moved(L)
        shapes[f'roller_sleeve_687_{row}_{k}'] = sl.moved(L)
        for i, p in enumerate(bearings_687()):
            shapes[f'bearing_687_{row}_{k}_{i}'] = p.moved(L)
    for k, s in shapes.items():
        s.label = k.lstrip('_')
        s.color = b.Color(*colour(k))
    return shapes, info


def check(shapes):
    rep = {}
    for name in ('wheel_hub_687', '_roller_core_687', '_roller_sleeve_687', '_fit_coupon_687'):
        s = shapes[name]
        rep[name.lstrip('_')] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_mm3=round(s.volume, 1))
    hub = shapes['wheel_hub_687']
    ov, squeeze = {}, {}
    for k, s in shapes.items():
        if k.startswith(('roller_', 'bearing_687_')) and wm.near(hub, s):
            v = wm.overlap(hub, s)
            if k.startswith('roller_sleeve_'):
                squeeze[k] = round(v, 2)
            elif v > 0.01:
                ov[k] = round(v, 3)
    rep['hub_overlaps_mm3 (want none; sleeves apart)'] = ov
    ring = math.pi * ((BRG_D / 2 - SLEEVE_LEAD) ** 2 - (SLEEVE_ID / 2) ** 2)
    rep['sleeve_squeeze_mm3 (by design: 0.1 per end x its end face ~%.1f mm2 x 2 = %.1f)' % (ring, 2 * 0.1 * ring)] = \
        dict(min=min(squeeze.values()), max=max(squeeze.values()))
    rep['seat_slots_mm'] = dict(depth=SEAT_SLOT_D, width=round(SEAT_SLOT_W, 2), side_clearance=round((SEAT_SLOT_W - BRG_D) / 2, 3),
                                neck=SEAT_NECK_W, neck_interference_per_side=round((BRG_D - SEAT_NECK_W) / 2, 3),
                                axial_click_step=SEAT_STEP, axial_pinch_per_side_entering=round(-SLEEVE_END_GAP + SEAT_STEP, 2),
                                axial_pinch_per_side_seated=round(-SLEEVE_END_GAP, 2),
                                m3x50_thread_in_pilot=round(50.0 - (SEAT + ZF + SEAT_SLOT_D), 2),
                                wall_under_head_seat=round(SEAT - ZF - SEAT_SLOT_D, 2))
    gaps = {}
    for rk in ('1_1', '1_2', '2_1', '2_2'):
        gaps[rk] = dict(bearing=round(min(wm.min_dist(hub, shapes[f'bearing_687_{rk}_{i}']) for i in (0, 1)), 3),
                        core=round(wm.min_dist(hub, shapes[f'roller_core_687_{rk}']), 3),
                        tread=round(wm.min_dist(hub, shapes[f'roller_tread_{rk}']), 3))
    rep['min_gaps_mm (turning parts to the hub)'] = gaps
    core, sl, brg = shapes['_roller_core_687'], shapes['_roller_sleeve_687'], bearings_687()
    rep['cartridge_overlaps_mm3 (want 0)'] = dict(core_bearings=round(sum(wm.overlap(core, p) for p in brg), 3),
                                                  sleeve_bearings=round(sum(wm.overlap(sl, p) for p in brg), 3),
                                                  sleeve_core=round(wm.overlap(sl, core), 3))
    rep['cartridge_clearances_mm'] = dict(sleeve_middle_to_core=round(wm.min_dist(sl, core), 3),
                                          bearing_B_to_pocket_floor=round(ZB0 - POCKET_B_FLOOR, 2),
                                          sleeve_wall=round((BRG_D - SLEEVE_ID) / 2, 2),
                                          core_wall_at_pocket_to_key_slot=round(CORE_R - KEY_DEPTH - (BRG_DO / 2 + BRG_FIT_R), 2))
    # radial drop-in of the whole cartridge (core + tread + sleeve) along its slot
    drop = {}
    frames = {(r, k): M for r, k, M in wm.roller_frames()}
    for rk in ((1, 1), (1, 2), (2, 1), (2, 2)):
        M = frames[rk]
        for d in (0.0, 10.0, 20.0, 30.0):
            T = np.eye(4)
            T[:3, 3] = M[:3, 1] * d
            drop[f'{rk[0]}_{rk[1]} +{d:g}'] = dict(
                core_tread=round(sum(wm.overlap(hub, shapes[f'roller_{p}_{rk[0]}_{rk[1]}'].moved(wm.loc(T))) for p in ('core_687', 'tread')), 2),
                sleeve=round(wm.overlap(hub, shapes[f'roller_sleeve_687_{rk[0]}_{rk[1]}'].moved(wm.loc(T))), 2))
    rep['cartridge_radial_press_in (core_tread want 0; sleeve = its squeeze between the thrust faces along the slot)'] = drop
    # round 3 block walls (analytic: the side profile's flanks are tangent to the R BLOCK_R_Q round about the axle; the
    # wheel-plane lobe is the R BLOCK_R_Q circle about the block centre)
    zb = (ZF + B_OUT) / 2
    za = (ZF + A_OUT) / 2
    rep['block_walls_mm'] = dict(
        head_counterbore_side_profile=round(BLOCK_R_Q - HEAD_POCKET_D / 2, 2),
        head_counterbore_rim_wheel_plane=round(math.sqrt(BLOCK_R_Q ** 2 - (B_OUT - zb) ** 2) - HEAD_POCKET_D / 2, 2),
        pilot_thread_side_profile=round(BLOCK_R_Q - PILOT_D_Q / 2, 2),
        pilot_thread_end_wheel_plane=round(math.sqrt(BLOCK_R_Q ** 2 - (ZF + PILOT_DEPTH - za) ** 2) - PILOT_D_Q / 2, 2),
        pilot_floor_beyond_thread=round(za + BLOCK_R_Q - (ZF + PILOT_DEPTH), 2))
    v13 = shapes['_wheel_hub_v13']
    o_new, o_old = overhang_mm2(hub), overhang_mm2(v13)
    rep['print_overhang_mm2 (cartridge end down, faces under 45 deg)'] = dict(round3=dict(all=o_new[0], spokes=o_new[1]), v13=dict(all=o_old[0], spokes=o_old[1]))
    rep['hub_max_radius_mm'] = dict(round3=o_new[2], v13=o_old[2])
    rep['volume_cm3'] = dict(round3=round(hub.volume / 1000, 1), v13=round(v13.volume / 1000, 1))
    rep['spoke_neck_row2_az0_mm (unchanged below r 54 = same load path)'] = dict(round3=wm.neck_widths(hub), v13=wm.neck_widths(v13))
    bb = hub.bounding_box()
    rep['hub_x_range'] = [round(bb.min.X, 2), round(bb.max.X, 2)]
    return rep


def export(shapes):
    parts = OUT / 'parts'
    parts.mkdir(parents=True, exist_ok=True)
    for f in list(parts.glob('*.step')) + list(OUT.glob('*.step')):
        f.unlink()
    for k in ('wheel_hub_687', '_roller_core_687', '_roller_tread', '_roller_sleeve_687', '_fit_coupon_687'):
        n = k.lstrip('_')
        write_part(shapes[k], colour(n), parts / (n + '.step'), n)
    skip = ('bearing_', '_')            # no bearings or fasteners in any STEP; '_' = unplaced copies
    write_step({k: (s, colour(k)) for k, s in shapes.items() if not k.startswith(skip)}, OUT / 'wheel_quiet_assembly.step', 'wheel_quiet')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    def section(items, transform, ax, xlim, ylim, title, cols=None, slab_t=0.6):
        slab = b.Box(400, 400, slab_t)
        polys = []
        for k, s in items.items():
            x = transform(s) & slab
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.06, 0.2)
            Vv = np.array([[q.X, q.Y] for q in v])
            c = (cols or {}).get(k, colour(k))
            polys += [(Vv[tri], c) for tri in np.array(t)]
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(title, fontsize=9)

    M = {(r, k): M for r, k, M in wm.roller_frames()}[(1, 1)]
    inv = wm.loc(M).inverse()
    items = {k: shapes[k] for k in ('wheel_hub_687', 'roller_core_687_1_1', 'roller_tread_1_1', 'roller_sleeve_687_1_1',
                                    'bearing_687_1_1_0', 'bearing_687_1_1_1')}
    along = lambda s: s.moved(inv).rotate(b.Axis.Y, 90)            # noqa: E731  roller axis across the page (pilot side right)
    fig, axs = plt.subplots(2, 2, figsize=(16, 12))
    section(items, along, axs[0, 0], (-34, 34), (-24, 24), 'roller 1_1 cut along its axle: 687s grey, sleeve blue, core green, tread dark')
    section(items, along, axs[0, 1], (12, 23), (-9, 9), 'one end: sleeve shoulder on the inner race, sleeve end 0.1 into the block face (press fit)')
    old = {'_wheel_hub_v13': shapes['_wheel_hub_v13'], 'wheel_hub_687': shapes['wheel_hub_687']}
    cols = {'_wheel_hub_v13': (0.85, 0.85, 0.85), 'wheel_hub_687': colour('wheel_hub')}
    for ax, az, ttl in ((axs[1, 0], 30, 'row-1 spoke (az 30) in the axis plane'), (axs[1, 1], 0, 'row-2 spoke (az 0) in the axis plane')):
        section(old, lambda s, az=az: s.rotate(b.Axis.X, -az), ax, (-45, 35), (25, 72),
                ttl + ': v13 grey, round 3 orange (R6 blocks, 45 deg flanks)', cols)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_sections.png', dpi=95)
    plt.close(fig)
    # the click seat: across the axle inside the head-side seat (0.2 into the block face), outward up the page
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    seat_items = {'wheel_hub_687': shapes['wheel_hub_687'], 'roller_sleeve_687_1_1': shapes['roller_sleeve_687_1_1']}
    for ax, dz, ttl in ((axs[0], 0.2, 'across the axle 0.2 into the head block face: the Ø7.1 seat, 6.9 neck, channel'),
                        (axs[1], 0.45, '0.45 in: only the round seat (the channel is 0.4 deep)')):
        section(seat_items, lambda s, dz=dz: s.moved(inv).moved(b.Location((0, 0, -(ZF + dz)))), ax, (-9, 9), (-6, 12), ttl, slab_t=0.04)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_seat.png', dpi=95)
    plt.close(fig)
    # end view of the spokes (along the wheel axis) through each row plane
    fig, axs = plt.subplots(1, 2, figsize=(16, 8))
    for ax, x, ttl in ((axs[0], -D_ROW, 'row 1 plane'), (axs[1], D_ROW, 'row 2 plane')):
        section({**old, 'roller_core': shapes['roller_core_687_1_1' if x < 0 else 'roller_core_687_2_1']},
                lambda s, x=x: s.rotate(b.Axis.Y, 90).moved(b.Location((0, 0, -x))), ax, (-75, 75), (-75, 75),
                f'{ttl}: v13 hub grey, round 3 orange', cols)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_end_view.png', dpi=80)
    plt.close(fig)
    render_tips(shapes)


def render_tips(shapes):
    """shaded close-ups of a head and a pilot spoke tip, v13 against round 3 (MuJoCo's offscreen renderer: depth-correct)"""
    import mujoco
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tmp = VAL / '_tmp'
    tmp.mkdir(exist_ok=True)
    hubs = (('v13', shapes['_wheel_hub_v13']), ('round 3', shapes['wheel_hub_687']))
    spokes_ = ((270.0, -D_ROW, 'row-1 head spoke'), (0.0, D_ROW, 'row-2 pilot spoke'))
    fig, axs = plt.subplots(len(hubs) * len(spokes_), 4, figsize=(17, 17))
    for si, (az, xrow, sname) in enumerate(spokes_):
        c, s = math.cos(math.radians(az)), math.sin(math.radians(az))
        ray, tang, xax = np.array([0, c, s]), np.array([0, -s, c]), np.array([1.0, 0, 0])
        sx = np.sign(xrow)
        tgt = xax * xrow / 1000 + ray * 0.058
        views = (('along the wheel axis (sketch 1)', xax * sx * 0.07), ('along the axle (sketch 2)', tang * 0.07),
                 ('oblique, outboard', (ray + tang + xax * sx) * 0.045), ('oblique, inboard', (ray - tang - xax * sx) * 0.045))
        meshes = []
        for hi, (hn, h) in enumerate(hubs):
            box = b.Box(90, 60, 50).moved(b.Location((0, 50 * c, 50 * s), (1, 0, 0), az - 270))
            stl = tmp / f'{si}_{hi}.stl'
            b.export_stl(h & box, str(stl), tolerance=0.01, angular_tolerance=0.1, ascii_format=False)
            meshes.append(f'<mesh name="m{hi}" file="{stl.as_posix()}" scale="0.001 0.001 0.001"/>')
        xml = (f'<mujoco><visual><global offwidth="700" offheight="700" fovy="35"/><headlight ambient="0.3 0.3 0.3" diffuse="0.55 0.55 0.55"/>'
               f'</visual><asset>{"".join(meshes)}</asset><worldbody><light pos="0 0.3 -0.3" dir="0 -1 1" diffuse="0.45 0.45 0.45"/>'
               + ''.join(f'<body mocap="true"><geom type="mesh" mesh="m{i}" rgba="0.93 0.55 0.25 1" contype="0" conaffinity="0"/></body>'
                         for i in range(len(hubs))) + '</worldbody></mujoco>')
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        r = mujoco.Renderer(m, 560, 560)
        for hi, (hn, _) in enumerate(hubs):
            for j in range(len(hubs)):
                d.mocap_pos[j] = (0, 0, 0) if j == hi else (10, 10, 10)
            mujoco.mj_forward(m, d)
            for j, (title, off) in enumerate(views):
                cam = mujoco.MjvCamera()
                cam.lookat[:] = tgt
                cam.distance = float(np.linalg.norm(off))
                cam.azimuth = math.degrees(math.atan2(off[1], off[0]))
                cam.elevation = -math.degrees(math.asin(np.clip(off[2] / np.linalg.norm(off), -0.999, 0.999)))
                r.update_scene(d, cam)
                ax = axs[si * len(hubs) + hi, j]
                ax.imshow(r.render())
                ax.set_title(f'{sname}, {hn}: {title}', fontsize=8)
                ax.axis('off')
        r.close()
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_tips.png', dpi=70)
    plt.close(fig)
    for f in tmp.glob('*.stl'):
        f.unlink()
    tmp.rmdir()


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    shapes, info = build_all()
    print('build', info, flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes)
        rep['build'] = info
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
