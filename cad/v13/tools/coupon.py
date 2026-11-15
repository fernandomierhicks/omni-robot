"""cad/v13 print-fit calibration coupons (Nov 11 2026, from the whole-CAD review). build123d, eight small printed parts; bench tooling, not part of the robot.

    python -B coupon.py [build|check|export|all]        (omni-robot-sim interpreter)

Every printed fit in v13 is a "first try": dovetail and tenon sockets 0.1 mm per face, bearing seats on or a tenth over the bearing, the motor's can pocket 0.1
radial, the pod's pocket 0.2, M3 pilots 2.8 / 2.9 / 3.0, a 0.3 mm roller thrust gap. Each was set by arithmetic, not by a print on this printer. These coupons
bracket every one with a short series, each member numbered on the part (raised 0.6 mm), so ONE print says which number to put into dims.py. Print them in
the filament and with the settings the robot's parts will use; a coupon in another material says nothing.
 * coupon_dt_tabs + coupon_dt_sockets: the chassis' two dovetail tails (ring, cross: rows R, X) and the fork's 9 x 24.5 x 7 tenon (row T), one column per
   clearance 0.10 / 0.15 / 0.20 / 0.30 mm per face (the number is hundredths of a mm). Every socket is blind from the top; turn the tab plate over like a page
   and set it on the block: each tab meets the socket of the same number. Pick the tightest that goes in by hand and comes out again.
 * coupon_seats: 4 seats for a 627 (Ø22: robot 22.00) and 4 for a 6804 (Ø32: robot 32.10), 7 deep, relieved through under; the number is the diameter in tenths of
   a mm. Press the real bearings by hand: which sit, which spin, which do not go.
 * coupon_pilots: M3 pilots Ø2.7 .. 3.1 (number = tenths): a row blind from the top (along the print axis, 12 deep) and a row through the block's thickness (in the
   layer planes, 24 deep). Drive real screws; the tie bolts, keel screws and nut-less pilots all come from this block.
 * coupon_slots + coupon_peg: 4 slide-in slots for a loose 12 mm peg (the pod frame in its pocket), 0.2 .. 0.5 per face (hundredths).
 * coupon_can: 4 pockets for the AK40-10's can (Ø53.0), 2 deep, 0.05 .. 0.20 radial (hundredths), 0.5 lead-in: the motor itself is the gauge.
 * coupon_gaps: 5 open slits 0.3 .. 0.8 wide (number = tenths), 10 deep: which stay open on this printer (a strip of paper or a feeler goes through).
Everything is nominal CAD; nothing printed. Dimensions in dims.py (COUPON_*).
"""
import json
import sys
from pathlib import Path

import build123d as b
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

sys.path.insert(0, str(Path(__file__).resolve().parent))
import print_beds as PB                                                # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

overlap, min_dist = W.overlap, W.min_dist
OUT = W.ROOT / 'step_export' / 'coupon'
VAL = W.ROOT / 'validation' / 'coupon'
BED = 256.0
LABEL_H, LABEL_SIZE = 0.6, 4.0
PRINTED = ('coupon_dt_tabs', 'coupon_dt_sockets', 'coupon_seats', 'coupon_pilots', 'coupon_slots', 'coupon_peg', 'coupon_can', 'coupon_gaps')
COLOURS = dict(zip(PRINTED, [(0.30, 0.65, 0.85), (0.20, 0.45, 0.70), (0.90, 0.60, 0.20), (0.35, 0.75, 0.45), (0.85, 0.35, 0.35), (0.85, 0.35, 0.35),
                             (0.60, 0.40, 0.80), (0.70, 0.70, 0.70)]))


def blk(x0, x1, y0, y1, z0, z1):
    return b.Box(x1 - x0, y1 - y0, z1 - z0).moved(b.Location(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def cyl(d, z0, z1, x, y):
    return b.Solid.make_cylinder(d / 2, z1 - z0, b.Plane(origin=(x, y, z0), z_dir=(0, 0, 1)))


def prism(poly, z0, z1):
    """a shapely polygon extruded from z0 to z1"""
    pts = list(orient(poly, 1.0).exterior.coords)[:-1]                        # counter-clockwise: a clockwise ring (shapely's buffer) extrudes the other way
    return b.extrude(b.Plane(origin=(0, 0, z0)) * b.Polygon(*pts, align=None), amount=z1 - z0)


def label(text, x, y, z):
    """raised text on a top face at (x, y, z)"""
    return b.extrude(b.Text(text, font_size=LABEL_SIZE, align=(b.Align.CENTER, b.Align.CENTER)), amount=LABEL_H).moved(b.Location((x, y, z)))


def engrave(text, x, y, z):
    """text cut LABEL_H deep into a top face at (x, y, z): for the faces that mate (the tab plate and the socket block), where raised numbers would hold them apart"""
    return b.extrude(b.Text(text, font_size=LABEL_SIZE, align=(b.Align.CENTER, b.Align.CENTER)), amount=LABEL_H + 0.1).moved(b.Location((x, y, z - LABEL_H)))


def tail(kind, x, y0):
    """a dovetail tail's plan: root (neck) on y = y0 at x, flaring to its end"""
    n, f, L = COUPON_DT[kind]
    return Polygon([(x - n, y0), (x + n, y0), (x + f, y0 + L), (x - f, y0 + L)])


def tag(c):
    return f'{round(c * 100):02d}'


# ---- tabs and sockets: one column per clearance, one row per feature (T tenon, R ring tail, X cross tail) --------------------------------------------------
PITCH_X, X0, PLATE_Y = 32.0, 20.0, 90.0
PLATE_X = X0 * 2 + PITCH_X * (len(COUPON_CLR) - 1)
ROW_Y = dict(tenon=(10.0, 34.5), ring=(48.0, 53.0), cross=(63.0, 70.0))       # tenon: y range; tails: root y .. end y
BASE_T = 4.0                                                                  # the tab plate's base
SOCKET_TOP = COUPON_DT_H + 3.0                                                 # the socket block's thickness: 2.7 mm of floor under an 11.3 deep socket
ROWS = (('T', 22.0), ('R', 50.0), ('X', 66.0))


def col_x(i):
    return X0 + PITCH_X * i


def sock_x(i):
    """the socket block's columns run the other way: the tab plate turns over like a page"""
    return PLATE_X - col_x(i)


def build_dt():
    tw, tl, td, lead = COUPON_TENON
    tabs = blk(0, PLATE_X, 0, PLATE_Y, 0, BASE_T)
    sock = blk(0, PLATE_X, 0, PLATE_Y, 0, SOCKET_TOP)
    mouths = []
    for i, c in enumerate(COUPON_CLR):
        x, xs = col_x(i), sock_x(i)
        y0 = ROW_Y['tenon'][0]
        tabs = tabs + blk(x - tw / 2, x + tw / 2, y0, y0 + tl, BASE_T - 0.01, BASE_T + td)
        sock = sock - blk(xs - tw / 2 - c, xs + tw / 2 + c, y0 - c, y0 + tl + c, SOCKET_TOP - td - c, SOCKET_TOP + 0.1)
        mouths.append((xs, y0 + tl / 2, tw / 2 + c, tl / 2 + c))
        for kind in ('ring', 'cross'):
            r0 = ROW_Y[kind][0]
            tabs = tabs + prism(tail(kind, x, r0), BASE_T - 0.01, BASE_T + COUPON_DT_H)
            sock = sock - prism(tail(kind, xs, r0).buffer(c, join_style=2), SOCKET_TOP - COUPON_DT_H - c, SOCKET_TOP + 0.1)
    mouth_edges = [e for e in sock.edges() if abs(e.center().Z - SOCKET_TOP) < 1e-3
                   and any(abs(e.center().X - mx) <= hw + 0.01 and abs(e.center().Y - my) <= hl + 0.01 for mx, my, hw, hl in mouths)]
    sock = b.chamfer(mouth_edges, lead)                                       # lead-in on the tenon sockets, as pod_lean.tenon's
    tabs = b.chamfer([e for e in tabs.edges() if abs(e.center().Z - (BASE_T + td)) < 1e-3], lead)   # and the tenon's own tip
    for i, c in enumerate(COUPON_CLR):                                        # engraved, not raised: these two faces meet
        tabs = tabs - engrave(tag(c), col_x(i), 5.0, BASE_T)
        sock = sock - engrave(tag(c), sock_x(i), 5.0, SOCKET_TOP)
    for name, y in ROWS:
        tabs = tabs - engrave(name, 6.0, y, BASE_T)
        sock = sock - engrave(name, PLATE_X - 6.0, y, SOCKET_TOP)
    return tabs, sock


# ---- bearing seats -------------------------------------------------------------------------------------------------------------------------------------------
SEAT_PITCH, SEAT_MARGIN = 44.0, 12.0


def build_seats():
    """4 + 4 seats, depth 7 on a 9 mm plate, relieved through under the seat; the number is the diameter in tenths"""
    width = SEAT_MARGIN * 2 + 32.0 + SEAT_PITCH * 3
    plate = blk(0, width, 0, 84.0, 0, COUPON_SEAT_PLATE)
    rows = (('b627', 17.0, 18.0), ('b6804', 52.0, 24.0))                       # key, centre y, label offset
    for key, yc, off in rows:
        for i, d in enumerate(COUPON_SEAT[key]):
            x = SEAT_MARGIN + 16.0 + SEAT_PITCH * i
            plate = plate - cyl(d, COUPON_SEAT_PLATE - COUPON_SEAT_DEPTH, COUPON_SEAT_PLATE + 0.1, x, yc) - cyl(COUPON_RELIEF[key], -0.1, COUPON_SEAT_PLATE, x, yc)
    for key, yc, off in rows:
        for i, d in enumerate(COUPON_SEAT[key]):
            plate = plate + label(f'{round(d * 10):d}', SEAT_MARGIN + 16.0 + SEAT_PITCH * i, yc + off, COUPON_SEAT_PLATE - 0.01)
    return plate


# ---- M3 pilots ------------------------------------------------------------------------------------------------------------------------------------------------
def build_pilots():
    """80 x 24 x 30: blind pilots from the top (along the print axis) and, 22 mm lower, through pilots in the layer planes; number = tenths of a mm"""
    block = blk(0, 80.0, 0, 24.0, 0, 30.0)
    for i, d in enumerate(COUPON_PILOT):
        x = 10.0 + 15.0 * i
        block = block - cyl(d, 30.0 - 12.0, 30.1, x, 7.0)
        block = block - b.Solid.make_cylinder(d / 2, 24.2, b.Plane(origin=(x, -0.1, 8.0), z_dir=(0, 1, 0)))
        block = block + label(f'{round(d * 10):d}', x, 17.0, 30.0 - 0.01)
    return block


# ---- slide-in slots -----------------------------------------------------------------------------------------------------------------------------------------
def build_slots():
    P = COUPON_SLOT_PEG
    pitch = P + 2 * max(COUPON_SLOT) + 8.0
    block = blk(0, pitch * len(COUPON_SLOT) + 4.0, 0, COUPON_SLOT_LEN + 6.0, 0, 20.0)
    for i, c in enumerate(COUPON_SLOT):
        x = 2.0 + pitch / 2 + pitch * i
        block = block - blk(x - P / 2 - c, x + P / 2 + c, -0.1, COUPON_SLOT_LEN, 4.0, 20.1)
        block = block + label(tag(c), x, COUPON_SLOT_LEN + 3.0, 20.0 - 0.01)
    return block, blk(0, P, 0, 30.0, 0, 12.0), pitch


# ---- the motor's can pocket ---------------------------------------------------------------------------------------------------------------------------------
def build_can():
    R = MOTOR_R
    pitch = 2 * (R + max(COUPON_CAN)) + 14.0
    size = 4.0 + pitch * 2 + 4.0
    plate = blk(0, size, 0, size + 6.0, 0, 4.0)
    for i, c in enumerate(COUPON_CAN):
        x, y = 4.0 + pitch / 2 + pitch * (i % 2), 10.0 + pitch / 2 + pitch * (i // 2)
        d = 2 * (R + c)
        plate = plate - cyl(d, 4.0 - MOTOR_POCKET_D, 4.1, x, y)
        plate = plate - b.Solid.make_cone(d / 2, d / 2 + 0.5, 0.5, b.Plane(origin=(x, y, 4.0 - 0.5), z_dir=(0, 0, 1)))
        plate = plate + label(tag(c), x, y - d / 2 - 4.0, 4.0 - 0.01)
    return plate


# ---- open slits ---------------------------------------------------------------------------------------------------------------------------------------------
def build_gaps():
    """a 10-deep slit per width, open through the block's thickness, 2 mm of floor; the number (tenths of a mm) is on the wall after each slit"""
    pitch = 14.0
    block = blk(0, pitch * len(COUPON_GAPS) + 6.0, 0, 14.0, 0, 12.0)
    for i, g in enumerate(COUPON_GAPS):
        x = 3.0 + pitch / 2 + pitch * i
        block = block - blk(x - g / 2, x + g / 2, -0.1, 14.1, 2.0, 12.1)
        block = block + label(f'{round(g * 10):d}', x + pitch / 2, 7.0, 12.0 - 0.01)
    return block


def build():
    tabs, sockets = build_dt()
    slots, peg, _ = build_slots()
    return dict(coupon_dt_tabs=tabs, coupon_dt_sockets=sockets, coupon_seats=build_seats(), coupon_pilots=build_pilots(), coupon_slots=slots, coupon_peg=peg,
                coupon_can=build_can(), coupon_gaps=build_gaps())


def check(shapes):
    rep = dict(parts={}, fits={}, series=dict(dovetail_tenon_clr=COUPON_CLR, b627=COUPON_SEAT['b627'], b6804=COUPON_SEAT['b6804'], pilots=COUPON_PILOT,
                                              slots=COUPON_SLOT, can=COUPON_CAN, gaps=COUPON_GAPS))
    for k, s in shapes.items():
        bb = s.bounding_box()
        rep['parts'][k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 2), bbox=[round(v, 1) for v in bb.size],
                               fits_bed=bool(max(bb.size.X, bb.size.Y) <= BED))
    # the tab plate turned over like a page (about y) onto the socket block: no overlap, and each column's closest approach is its clearance
    tabs, sock = shapes['coupon_dt_tabs'], shapes['coupon_dt_sockets']
    page = tabs.rotate(b.Axis.Y, 180).moved(b.Location((PLATE_X, 0, SOCKET_TOP + BASE_T)))
    page = page & blk(-1, PLATE_X + 1, -1, PLATE_Y + 1, -5, SOCKET_TOP - 0.01)  # the tabs only (the plate itself rests on the block's top)
    rep['fits']['tabs_into_sockets_overlap_mm3'] = round(float(overlap(page, sock)), 3)
    for i, c in enumerate(COUPON_CLR):
        col = blk(sock_x(i) - PITCH_X / 2 + 0.01, sock_x(i) + PITCH_X / 2 - 0.01, -1, PLATE_Y + 1, -1, SOCKET_TOP + 20)
        rep['fits'][f'closest_gap_at_{tag(c)}_mm'] = dict(want=c, got=round(float(min_dist(page & col, sock & col)), 3))
    # the loose peg in each slot
    slots, peg, pitch = shapes['coupon_slots'], shapes['coupon_peg'], build_slots()[2]
    P = COUPON_SLOT_PEG
    for i, c in enumerate(COUPON_SLOT):
        p = peg.moved(b.Location((2.0 + pitch / 2 + pitch * i - P / 2, 5.0, 6.0)))   # off the slot's floor, so the gap measured is the side wall's
        rep['fits'][f'peg_in_slot_{tag(c)}'] = dict(overlap_mm3=round(float(overlap(p, slots)), 3), want=c, got=round(float(min_dist(p, slots)), 3))
    rep['print'] = {k: dict(zip(('up', 'supports', 'why'), PB.bed(k))) for k in shapes if PB.bed(k)}
    return rep


def export(shapes):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for k, s in shapes.items():
        write_part(s, COLOURS[k], OUT / 'parts' / (k + '.step'), k)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    shapes = build()
    for k, s in shapes.items():
        print(k, len(s.solids()), s.is_valid, round(s.volume / 1000, 2), [round(v, 1) for v in s.bounding_box().size], flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
