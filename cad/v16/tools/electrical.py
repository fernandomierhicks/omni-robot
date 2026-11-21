"""v15 electrics, Nov 18 2026 (Fernando: "power stuff down there"; PROPOSAL, nothing bought or wired): where every electrical block sits on
both floors, its printed cradle and its screws. Library: upper.py's electronics() takes floor 2's items and both floors' cradles from here;
harness.py takes the ports. Envelopes, not detail CAD; sizes ASSUMED unless a model is named.

Floor 1 (the deck's top, z 100; nothing over z 148, the lidar's band): the deck is not modified (Fernando), so every cradle screws into the
deck's own accessory M3 pilots (chassis_mid.accessory_holes, g['accessory_pts']). The deck's free area (pods in every pose, towers, straps,
fins, openings: validation/electrical/deck_free.png) holds exactly two converter-sized blocks, one in each side lens outside the side fins,
and a few small blocks in the wedges by the spine:
  * 12 V 8 A (the arm) in the left lens, 5 V 5 A (face, hub) in the right lens;
  * the battery's path: the deck's lead hole (-128, -17) -> anti-spark switch (the main button's) -> inline 30 A fuse (on the lead) ->
    INA226 module (its shunt swapped for 2 mOhm: 41 A full scale) -> XT30 distribution block -> four drives, the two converters, the 19 V
    converter's feed (up) and the signal lead (CAN, the INA226's I2C; up). Nov 19: no pack sense lead, the INA226 reads the pack voltage.
Floor 2 (z 181.5): the 19 V 10 A converter (74 x 74: fits nowhere on the deck) by the computer; the computer under the bin; the carrier
(pcb/v1, KiCad STEP) beside the arm riser, plugs up; the INA3221 where the rails arrive; the USB hub. The ELRS receiver is on the carrier
(Nov 18, Fernando: plugged into J5 on headers, in the KiCad STEP with its antenna; no block, cradle or lead of its own).
"""
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import M3_PILOT, M3_CLR                                      # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
REF_E = ROOT / 'reference' / 'electronics'
Z_DECK, Z_FLOOR2 = 100.0, 181.5
PLATE, WALL, LIP = 2.5, 2.0, 7.0                                       # cradle floor, wall, wall height over the plate
PLATE_EARS = 7.5                                                       # Nov 18 (check): a converter's cradle floor: its ear screws (M3 x 10
                                                                       # through the 3 mm ear) get 7 mm of thread, the tip 0.5 over the deck
PLATE_DECK = 5.0                                                       # any other floor-1 cradle: room for its hold-down heads' counterbores
CB_HEAD = (7.0, 3.0)                                                   # floor-1 hold-down counterbore (Ø, depth): the head under the device
TAB, TAB_R = 3.0, 5.0                                                  # floor-2 hold-down tabs outside the walls: thickness, end radius
GAP = 0.5                                                              # device to cradle wall


def plate_of(blk):
    return PLATE_EARS if blk[5] else (PLATE_DECK if blk[0] == 1 else PLATE)


def plate(name):
    """a block's cradle floor thickness (its underside sits this far over the deck / floor)"""
    return plate_of(BLOCKS[name])

# ---- the blocks: (floor, centre x, y, angle of its long axis (deg), body L x W x H, ears (length, width) at both L ends or None, what) ------
BLOCKS = {
    # floor 1
    'conv_12v': (1, 22.0, 184.0, 90.0, (64.0, 57.0, 22.0), (9.0, 30.0),
                 '12 V 8 A potted buck, 15-90 V in (B0F3HQ6ND8 class; 64 x 57 x 22 per comparable 96 W units, VERIFY)'),
    'conv_5v': (1, 22.0, -184.0, 75.0, (66.0, 60.0, 22.0), (9.0, 30.0),
                '5 V 5 A potted buck, 8-40 V in (B0CM37DX4Q class; 66 x 60 x 22 per comparable 25 W units, VERIFY)'),
    'anti_spark': (1, -68.0, -40.0, 135.0, (50.0, 25.0, 15.0), None,
                   'anti-spark MOSFET switch, latching button input (60 A class, ASSUMED 50 x 25 x 15)'),
    'xt30_block': (1, 0.0, -62.0, 45.0, (70.0, 30.0, 16.0), None,
                   'printed distribution block: 7 x XT30 out on top (4 drives, 12 V, 5 V, 19 V feed), CAN junction and the signal lead on its +W '
                   'side; the INA226 module (shunt swapped to 2 mOhm 2512: 36 V, 41 A full scale) stands on its -W side, in series before it'),
    # floor 2
    'conv_19v': (2, 0.0, -165.0, 0.0, (74.0, 74.0, 32.0), (9.0, 40.0),
                 '19 V 10 A potted buck, 22-40 V in (B09KY6H4FZ; 74 x 74 x 32 ASSUMED from same-power units)'),
    'ina3221': (2, -105.0, -100.0, 0.0, (32.0, 25.0, 8.0), None, 'INA3221 module, 3 channels (components/ina3221), shunts swapped per pcb/v1 README'),
    'usb_hub': (2, -40.0, 120.0, 0.0, (90.0, 28.0, 14.0), None, 'powered USB 2 hub, 4 ports (ASSUMED 90 x 28 x 14)'),
}
CARRIER_AT = (26.0, -60.0)                                             # robot (x, y) of the board's top-left corner (board x = robot x,
CARRIER_STANDOFF = 6.0                                                 # board y down = robot -y); pcb/v1 placement study. Nov 18: x 30 -> 26
                                                                       # (the wider board: its top-left corner screw 7.9 off the side fin's
                                                                       # flange, was 5.1 at x 30; the 7 mm split 4 back / 3 forward)
CARRIER_SIZE = (102.0, 62.0)                                           # Nov 18 (Fernando): 102 x 62 (was 95) for the corner holes
CARRIER_HOLES = ((3.5, 3.5), (98.5, 3.5), (3.5, 58.5), (98.5, 58.5))   # board (x, y) of its M3 holes: the corners, 3.5 in (pcb/v1/tools/
                                                                       # gen_pcb.py HOLES, Nov 18; were inside the board)
CARRIER_SCREW_L = 18.0                                                 # one M3 per corner through the board, its standoff and the tray into
                                                                       # a floor pilot: 1.6 + 6 + 3 clamped, 7.4 into the floor (5 + boss 4)
TEENSY_PAD1 = (3.6, 30.0)                                              # board (x, y) of U1's pad 1 (rot 90: pads along +x, row 25-48 at y - 15.24)
SOCKET_H = 8.5                                                         # 0.1 in female header height
TEENSY_HEADER = 3.61                                                   # the Teensy model's male header plastic under its board (pcb/v1, Nov 19)


def deck_z(floor):
    return Z_DECK if floor == 1 else Z_FLOOR2


def frame(blk):
    """(centre x, y, angle, z0 of the device's underside)"""
    fl, x, y, ang = blk[:4]
    return x, y, ang, deck_z(fl) + plate_of(blk)


def placed(shape, x, y, ang, z):
    return shape.rotate(b.Axis.Z, ang).moved(b.Location((x, y, z)))


def body(name):
    """the device envelope: a box (L along local x) + its ears (with Ø4.5 holes) at both L ends, placed"""
    fl, x, y, ang, (L, Wd, Hh), ears, _ = BLOCKS[name]
    s = P.box(-L / 2, L / 2, -Wd / 2, Wd / 2, 0, Hh)
    if ears:
        el, ew = ears
        for sx in (-1, 1):
            e = P.box(sx * L / 2 if sx > 0 else -L / 2 - el, L / 2 + el if sx > 0 else -L / 2, -ew / 2, ew / 2, 0, 3.0)
            e = e - W.cyl(2.25, 3.4, (sx * (L / 2 + el / 2), 0, 1.5))
            s = s + e
    return placed(s, *frame(BLOCKS[name]))


def footprint(name, grow=0.0):
    fl, x, y, ang, (L, Wd, Hh), ears, _ = BLOCKS[name]
    Lx = L + (2 * ears[0] if ears else 0.0)
    return Lx + 2 * grow, Wd + 2 * grow


def _spread(cand, n, min_gap=25.0):
    """up to n of cand [(u, v, px, py)], spread: the farthest from the block's centre first, then each farthest from those chosen"""
    chosen = []
    while cand and len(chosen) < n:
        if not chosen:
            best = max(cand, key=lambda c: math.hypot(c[0], c[1]))
        else:
            best = max(cand, key=lambda c: min(math.hypot(c[2] - q[2], c[3] - q[3]) for q in chosen))
            if min(math.hypot(best[2] - q[2], best[3] - q[3]) for q in chosen) < min_gap:
                break
        chosen.append(best)
        cand = [c for c in cand if c is not best]
    return chosen


def _tab(u, v, hx, hy):
    """a TAB-thick tab from the cradle's outline (half sizes hx, hy) out to a screw at (u, v), beside one side only"""
    if abs(u) > hx:
        sg = 1.0 if u > 0 else -1.0
        bar = P.box(min(sg * (hx - 1.0), u), max(sg * (hx - 1.0), u), v - TAB_R, v + TAB_R, 0, TAB)
    else:
        sg = 1.0 if v > 0 else -1.0
        bar = P.box(u - TAB_R, u + TAB_R, min(sg * (hy - 1.0), v), max(sg * (hy - 1.0), v), 0, TAB)
    return bar + W.cyl(TAB_R, TAB, (u, v, TAB / 2))


TIE_DOWNS = {}                                                         # name: the deck tie pair (x, y, dir) its cradle's pad is tied to


def _tie_pad(name, acc_ties, hx, hy, pt, to_local, floor_ok, reach=20.0, rim=3.0):
    """a pt-thick pad from the cradle's outline (half sizes hx, hy) over the nearest deck tie pair whose slots lie outside it (within reach
    of the outline, floor_ok at its centre), the two slots cut through it: the tie goes down one slot through pad and deck, under the
    deck's bridge, up the other and over the pad's bridge. Returns the pad (local frame), or None when none fits"""
    from dims import TIE_SLOT_L, TIE_SLOT_W, TIE_PITCH
    best = None
    for tx, ty, dg in acc_ties:
        n = (-math.sin(math.radians(dg)), math.cos(math.radians(dg)))    # the slots' offset direction (across the cable)
        sl = [to_local(tx + k * TIE_PITCH / 2 * n[0], ty + k * TIE_PITCH / 2 * n[1]) for k in (-1, 1)]
        if any(abs(u) < hx + TIE_SLOT_L / 2 + 0.5 and abs(v) < hy + TIE_SLOT_L / 2 + 0.5 for u, v in sl):
            continue                                                       # a slot under the walls / body
        uc, vc = to_local(tx, ty)
        out = max(abs(uc) - hx, abs(vc) - hy)
        if out > reach or (floor_ok is not None and not floor_ok(tx, ty)):
            continue
        if best is None or out < best[0]:
            best = (out, tx, ty, dg, uc, vc, sl)
    if best is None:
        print(f'{name}: no deck tie pair within {reach} mm of its cradle: one screw only')
        return None
    _, tx, ty, dg, uc, vc, sl = best
    TIE_DOWNS[name] = (tx, ty, dg)
    (u1, v1), (u2, v2) = sl
    la = math.degrees(math.atan2(v2 - v1, u2 - u1))                       # local direction slot to slot
    pad = placed(P.box(-TIE_PITCH / 2 - TIE_SLOT_W / 2 - rim, TIE_PITCH / 2 + TIE_SLOT_W / 2 + rim,
                       -TIE_SLOT_L / 2 - rim, TIE_SLOT_L / 2 + rim, 0, pt), uc, vc, la, 0)
    if abs(uc) - hx > abs(vc) - hy:                                        # a bar back to the outline, TAB_R each side of the centre
        sg = 1.0 if uc > 0 else -1.0
        pad = pad + P.box(min(sg * (hx - 1.0), uc), max(sg * (hx - 1.0), uc), vc - TAB_R, vc + TAB_R, 0, pt)
    else:
        sg = 1.0 if vc > 0 else -1.0
        pad = pad + P.box(uc - TAB_R, uc + TAB_R, min(sg * (hy - 1.0), vc), max(sg * (hy - 1.0), vc), 0, pt)
    for u, v in sl:
        pad = pad - placed(P.box(-TIE_SLOT_W / 2, TIE_SLOT_W / 2, -TIE_SLOT_L / 2, TIE_SLOT_L / 2, -0.1, pt + 0.1), u, v, la, 0)
    return pad


def cradle(name, acc_pilots, keep_heads=(), floor_ok=None, acc_ties=()):
    """a printed tray under a block: a floor plate (PLATE; PLATE_EARS under a converter), WALL walls LIP high round the body (notched for
    the ears), the ears screwed into the plate (M3 x 10 through the ear, 7 mm of thread in a through pilot, the tip 0.5 over the surface).
    Nov 18 (check: the device stood on its hold-down heads; the ear screws had 3 mm of thread): floor 1 into the deck's own accessory
    pilots under the cradle (M3 x 12, the heads in CB_HEAD counterbores under the device; 7.5+ into the deck); floor 2 on TAB-thick tabs
    outside the walls into new floor pilots (M3 x 10: 7 into the floor and its boss) where floor_ok(x, y) allows. keep_heads: screw
    heads already on the surface (relieved Ø8).
    Returns (solid, hw records, floor-2 pilots)."""
    fl, x, y, ang, (L, Wd, Hh), ears, _ = BLOCKS[name]
    zs = deck_z(fl)
    pt = plate(name)
    Lx, Wy = footprint(name, GAP + WALL)
    hx, hy = Lx / 2, Wy / 2
    t = P.box(-hx, hx, -hy, hy, 0, pt)
    li, wi = L / 2 + GAP, Wd / 2 + GAP
    wall = P.box(-li - WALL, li + WALL, -wi - WALL, wi + WALL, pt, pt + LIP) - P.box(-li, li, -wi, wi, pt - 0.1, pt + LIP + 0.1)
    if ears:                                                               # the ears pass over the end walls: notch the walls there; the
        el, ew = ears                                                      # body rests on an 8 mm rim round a window (plastic, print time)
        wall = wall - P.box(-li - WALL - 0.1, li + WALL + 0.1, -ew / 2 - GAP, ew / 2 + GAP, pt - 0.01, pt + LIP + 0.1)
        t = t - P.box(-L / 2 + 8.0, L / 2 - 8.0, -Wd / 2 + 8.0, Wd / 2 - 8.0, -0.1, pt + 0.1)
    s = t + wall
    hw, pil = [], []
    c, sn = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    to_robot = lambda u, v: (x + c * u - sn * v, y + sn * u + c * v)    # noqa: E731
    to_local = lambda px, py: ((px - x) * c + (py - y) * sn, -(px - x) * sn + (py - y) * c)   # noqa: E731
    if ears:
        for sx in (-1, 1):
            u = sx * (L / 2 + ears[0] / 2)
            s = s - W.cyl(M3_PILOT / 2, pt + 0.2, (u, 0, pt / 2))
            px, py = to_robot(u, 0)
            hw.append(H.pan((px, py, zs + pt + 3.0), (0, 0, -1), 10.0, f'bolt_{name}_ear'))
    cand = []
    if fl == 1:                                                            # floor 1: the deck's pilots are sparse and nearly all under the
        ear_u = L / 2 + ears[0] / 2 if ears else None                     # cradles: inside, the heads counterbored under the device
        for px, py in acc_pilots:
            u, v = to_local(px, py)
            if abs(u) < hx - 5.0 and abs(v) < hy - 5.0 and (ear_u is None or abs(abs(u) - ear_u) > 7.0):
                cand.append((u, v, px, py))
        chosen = _spread(cand, 3)
        if not chosen:
            raise RuntimeError(f'{name}: no accessory pilot under its cradle')
        for u, v, px, py in chosen:
            if ears:                                                       # a pad back where the window took the plate
                s = s + W.cyl(6.0, pt, (u, v, pt / 2))
                wu, wv = L / 2 - 8.0, Wd / 2 - 8.0                         # Nov 19 (SolidWorks: bracket_conv_12v 2 bodies): a pad inside
                if wu - abs(u) < wv - abs(v):                              # the window floated; an 8 wide rib ties it to the nearer rim
                    e = math.copysign(wu + 1.0, u)
                    s = s + P.box(min(u, e), max(u, e), v - 4.0, v + 4.0, 0, pt)
                else:
                    e = math.copysign(wv + 1.0, v)
                    s = s + P.box(u - 4.0, u + 4.0, min(v, e), max(v, e), 0, pt)
            s = s - W.cyl(M3_CLR / 2, pt + 0.2, (u, v, pt / 2)) - W.cyl(CB_HEAD[0] / 2, CB_HEAD[1] + 0.1, (u, v, pt - CB_HEAD[1] / 2 + 0.05))
            hw.append(H.pan((px, py, zs + pt - CB_HEAD[1]), (0, 0, -1), 12.0, f'bolt_{name}_deck'))
        pad = _tie_pad(name, acc_ties, hx, hy, pt, to_local, floor_ok) if len(chosen) < 2 else None
        if pad is not None:                                                # Nov 18: one pilot can't stop it turning: a pad out to the
            s = s + pad                                                    # nearest deck tie pair outside, a zip tie through both
        return placed(s, x, y, ang, zs), hw, pil
    d = 6.5                                                                # floor 2: tab screws 6.5 outside the outline, along each side
    for f in np.linspace(-0.8, 0.8, 9):
        for u, v in ((f * (hx - TAB_R), hy + d), (f * (hx - TAB_R), -hy - d), (hx + d, f * (hy - TAB_R)), (-hx - d, f * (hy - TAB_R))):
            px, py = to_robot(u, v)
            if floor_ok is None or floor_ok(px, py):
                cand.append((u, v, px, py))
    chosen = _spread(cand, 4)
    if len(chosen) < 2:
        raise RuntimeError(f'{name}: {len(chosen)} hold-down screw places outside its cradle')
    for u, v, px, py in chosen:
        s = s + _tab(u, v, hx, hy)
        s = s - W.cyl(M3_CLR / 2, TAB + 0.2, (u, v, TAB / 2))
        hw.append(H.pan((px, py, zs + TAB), (0, 0, -1), 10.0, f'bolt_{name}_floor'))
        pil.append((px, py))
    for kx, ky in keep_heads:
        u, v = to_local(kx, ky)
        if abs(u) < hx + 12.0 + TAB_R and abs(v) < hy + 12.0 + TAB_R:
            s = s - W.cyl(4.0, max(pt, TAB) + 0.2, (u, v, max(pt, TAB) / 2))
    return placed(s, x, y, ang, zs), hw, pil


# ---- the carrier (pcb/v1) on floor 2 ---------------------------------------------------------------------------------------------------
def board_to_robot(bx, by, z=0.0):
    return CARRIER_AT[0] + bx, CARRIER_AT[1] - by, z


def carrier(keep_heads=()):
    """(printed tray, vendor {name: (path, 4x4)}, hw, floor pilots): the board on four Ø7 standoffs on a 3 mm tray. Nov 18 (pcb/v1's
    holes at its corners): one M3 x 18 per corner through the board, its standoff and the tray into a floor pilot, so the same four screws
    hold the board and the tray. keep_heads: screw heads already on the floor under the tray (two of the side fin's floor screws; relieved Ø8)"""
    x0, y0 = CARRIER_AT
    bw, bh = CARRIER_SIZE
    zt = Z_FLOOR2 + 3.0                                                    # tray top
    zb = zt + CARRIER_STANDOFF                                             # the board's underside
    tray = P.box(x0 - 2.0, x0 + bw + 2.0, y0 - bh - 2.0, y0 + 2.0, Z_FLOOR2, zt)
    hw, pil = [], []
    for hx, hy in CARRIER_HOLES:
        px, py, _ = board_to_robot(hx, hy)
        tray = tray + W.cyl(3.5, CARRIER_STANDOFF, (px, py, zt + CARRIER_STANDOFF / 2))
        tray = tray - W.cyl(M3_CLR / 2, CARRIER_STANDOFF + 3.4, (px, py, Z_FLOOR2 + (CARRIER_STANDOFF + 3.0) / 2))
        hw.append(H.pan((px, py, zb + 1.6), (0, 0, -1), CARRIER_SCREW_L, 'bolt_carrier'))
        pil.append((px, py))
    for kx, ky in keep_heads:
        if x0 - 6.0 < kx < x0 + bw + 6.0 and y0 - bh - 6.0 < ky < y0 + 6.0:
            tray = tray - W.cyl(4.0, 3.2, (kx, ky, Z_FLOOR2 + 1.5))
    T = np.eye(4)
    T[:3, 3] = (x0, y0, zb)                                                # the board STEP: x 0..102, y -62..0, board underside at z 0
    # Nov 19: pcb/v1/out/omni_carrier_colour.step (tools/colour_step.py), every face coloured; it carries the Teensy on its sockets
    # (seated right: its headers' plastic on the sockets), the CAN modules standing pin edge down, the buzzer and the ELRS receiver.
    # The separate Teensy and CAN models (the Teensy 3.61 low, the CAN upside down) are gone
    vendor = {'carrier_pcb': (REF_E / 'omni_carrier_v1.step', T)}
    return dict(tray_carrier=tray), vendor, hw, pil


def near_block(name, px, py, grow):
    """(px, py) within grow of the block's cradle outline"""
    fl, x, y, ang = BLOCKS[name][:4]
    hx, hy = (v / 2 for v in footprint(name, GAP + WALL))
    c, sn = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    u, v = (px - x) * c + (py - y) * sn, -(px - x) * sn + (py - y) * c
    return abs(u) < hx + grow and abs(v) < hy + grow


def carrier_near(px, py, grow):
    x0, y0 = CARRIER_AT
    return x0 - 2.0 - grow < px < x0 + CARRIER_SIZE[0] + 2.0 + grow and y0 - CARRIER_SIZE[1] - 2.0 - grow < py < y0 + 2.0 + grow


def build(acc_points, floor2_heads=(), floor2_ok=None):
    """(bought, printed, vendor, hw, floor-2 pilots). acc_points: chassis_mid g['accessory_pts'] (per deck piece: pilots, ties);
    floor2_ok(x, y): where floor 2 may take a hold-down screw (upper.py: off the seams, fins, slots, the lidar, the computer)"""
    pilots1 = [p for v in acc_points.values() for p in v['pilots']]
    ties1 = [t for v in acc_points.values() for t in v['ties']]
    bought, printed, vendor, hw, pil2 = {}, {}, {}, [], []
    for name, blk in BLOCKS.items():
        others = [k for k in BLOCKS if k != name and BLOCKS[k][0] == blk[0]]
        clear = TAB_R + 1.0

        def ok(px, py, others=others, fl=blk[0]):
            if any(near_block(k, px, py, clear) for k in others):
                return False
            if fl == 2:
                if carrier_near(px, py, clear) or any(math.hypot(px - hx, py - hy) < 8.0 for hx, hy in floor2_heads):
                    return False
                return floor2_ok is None or floor2_ok(px, py)
            return True
        bought[name] = body(name)
        cr, h, p = cradle(name, pilots1, floor2_heads if blk[0] == 2 else (), ok, ties1)
        printed[f'bracket_{name}'] = cr
        hw += h
        pil2 += p
    tr, v, h, p = carrier(floor2_heads)
    bad = [q for q in p if floor2_ok is not None and not floor2_ok(*q)]
    if bad:
        raise RuntimeError(f'carrier: its corner screws {bad} land on a seam, a fin flange, a slot, the lidar or a grid pilot')
    printed.update(tr)
    vendor.update(v)
    hw += h
    pil2 += p
    return bought, printed, vendor, hw, pil2


# ---- ports for harness.py: (block, where, conn, note). where: ('end', s, v, dz) on the L end s (+1 / -1), v across, dz over the body's
# mid-height, the plug out along the axis; ('top', u, v) on the top face, plug up; ('side', s, u, dz) on the W side s, plug out across.
# All ASSUMED (terminal layout per block class, VERIFY on the parts bought).
BLOCK_PORTS = {
    'as_in': ('anti_spark', ('end', 1, 0.0, 0.0), 'XT60', '6S in from the deck\'s lead hole'),
    'as_out': ('anti_spark', ('side', -1, -15.0, 0.0), 'XT60', '6S out (switched), on its side toward the spine (its -L end faces the '
               'block 6 mm off; its other side faces the rear-right pod\'s opening)'),
    'as_btn': ('anti_spark', ('top', 10.0, 0.0), 'XH2', 'the main button (latching): on = closed'),
    'ina226_in': ('xt30_block', ('end', 1, 0.0, 0.0), 'XT60', "the INA226's IN+ at the block's +L end (the INA226 stands on the block "
                  "beside it; the 30 A fuse is inline on this lead; its IN- joins the block's bus inside). Nov 18: the wedge between the "
                  "spine, the right side fin and the rear-right pod's opening leaves room for this lead only along the spine"),
    'ina226_i2c': ('xt30_block', ('top', -20.0, 11.0), 'XH4', 'the INA226: VS 3V3, GND, SDA, SCL (address 0x41)'),
    'blk_can': ('xt30_block', ('side', 1, 2.0, 0.0), 'XH2', "CAN1 junction: the carrier's lead meets the drives' chain here"),
    'blk_sig': ('xt30_block', ('side', 1, 20.0, 0.0), 'XH2', "the signal lead's block end (Nov 19: no pack sense, the INA226 reads it): its CAN and I2C "
                  'breakouts to the CAN junction and the INA226 beside it'),
}
for _k, _u in enumerate((-30.0, -21.5, -13.0, -4.5, 4.0, 12.5, 21.0)):
    _n = ('m1', 'm2', 'm3', 'm4', '12', '5', '19')[_k]
    BLOCK_PORTS[f'blk_{_n}'] = ('xt30_block', ('top', _u, 4.0), 'XT30', f'XT30 out: {_n}')
for _c, _v in (('12v', 8.0), ('5v', 8.0), ('19v', 8.0)):
    BLOCK_PORTS[f'c{_c}_in'] = (f'conv_{_c}', ('end', -1 if _c != '5v' else 1, _v, 0.0), 'XT30', 'input lead (6S)')
    BLOCK_PORTS[f'c{_c}_out'] = (f'conv_{_c}', ('end', -1 if _c != '5v' else 1, -_v, 0.0), 'XT30', 'output lead')
for _ch, _u in ((1, -10.0), (2, 0.0), (3, 10.0)):
    BLOCK_PORTS[f'ina3221_{_ch}_in'] = ('ina3221', ('side', -1, _u, 0.0), 'XH2', f'CH{_ch} IN+ (from the converter)')
    BLOCK_PORTS[f'ina3221_{_ch}_out'] = ('ina3221', ('side', 1, _u, 0.0), 'XH2', f'CH{_ch} IN- (to the load)')
BLOCK_PORTS['ina3221_i2c'] = ('ina3221', ('end', -1, 0.0, 0.0), 'XH4', 'VS 3V3, GND, SDA, SCL (address 0x40)')
BLOCK_PORTS['hub_up'] = ('usb_hub', ('side', -1, -20.0, 0.0), 'USB-C', "upstream, to the computer's USB-C")
BLOCK_PORTS['hub_d1'] = ('usb_hub', ('end', 1, 6.0, 0.0), 'USB-A', 'downstream 1 (the servo bus adapter)')
BLOCK_PORTS['hub_d2'] = ('usb_hub', ('end', 1, -6.0, 0.0), 'USB-A', 'downstream 2 (spare)')
BLOCK_PORTS['hub_pwr'] = ('usb_hub', ('end', -1, 0.0, 0.0), 'XH2', '5 V in (powered hub)')

# the carrier's connectors (pcb/v1 placement study, board coordinates of each housing's centre; vertical JST XH: plugs from the top).
# Nov 18: no J5 (the ELRS receiver plugs into it, on the board) and no J14 (the buzzer is soldered on); J20, J15 moved left. Then (the
# 102 x 62 board, corner holes) read from pcb/v1/omni_carrier.kicad_pcb: each footprint's courtyard centre
# Nov 19 (pcb/v1): no J1 (pack sense: the INA226 on the pack's high side reads it), the bottom row J6, J10, J12, J21, J13 from x 7,
# J19 an XH5
CARRIER_CONN = {'J3': (25.3, 3.9), 'J4': (11.2, 3.9), 'J6': (11.2, 58.1), 'J10': (20.3, 58.1),
                'J12': (31.9, 58.1), 'J13': (60.1, 58.1), 'J15': (53.5, 3.9), 'J20': (40.7, 3.9), 'J21': (46.0, 58.1),
                'J11': (91.4, 9.7), 'J16': (91.4, 29.4), 'J17': (91.4, 49.1), 'J18': (3.9, 42.2), 'J19': (66.0, 49.1)}
CARRIER_CONN_ALONG_Y = ('J11', 'J16', 'J17', 'J18')                   # their contacts run along board y (rot 90); the rest along x
XH_TOP = 7.0                                                           # a vertical XH header's top over the board


def board_top():
    return Z_FLOOR2 + 3.0 + CARRIER_STANDOFF + 1.6


def block_point(name, where):
    """(p, d) in the robot frame of a port on a block"""
    fl, x, y, ang, (L, Wd, Hh), ears, _ = BLOCKS[name]
    c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    ax, pp = np.array([c, s, 0.0]), np.array([-s, c, 0.0])
    z0 = deck_z(fl) + plate(name)
    o = np.array([x, y, z0 + Hh / 2])
    kind = where[0]
    if kind == 'end':
        _, sg, v, dz = where
        return o + ax * sg * L / 2 + pp * v + np.array([0, 0, dz]), ax * sg
    if kind == 'top':
        _, u, v = where
        return o + ax * u + pp * v + np.array([0, 0, Hh / 2]), np.array([0.0, 0.0, 1.0])
    _, sg, u, dz = where
    return o + ax * u + pp * sg * Wd / 2 + np.array([0, 0, dz]), pp * sg


def ports():
    """{name: (p, d, device, conn, note)} of every electrical.py port (blocks, carrier connectors, the Teensy's USB)"""
    out = {}
    for k, (blk, where, conn, note) in BLOCK_PORTS.items():
        p, d = block_point(blk, where)
        out[k] = (p, d, blk, conn, note)
    zt = board_top() + XH_TOP
    for ref, (bx, by) in CARRIER_CONN.items():
        px, py, _ = board_to_robot(bx, by)
        out[f'carrier_{ref}'] = (np.array([px, py, zt]), np.array([0.0, 0.0, 1.0]), 'carrier', 'XH', f'carrier {ref} (vertical JST XH)')
    tx, ty = TEENSY_PAD1                                                   # the Teensy's micro-B at its USB end (its model's x min -31.73)
    px, py, _ = board_to_robot(tx + 29.21 - 31.73, ty - 7.62)
    out['teensy_usb'] = (np.array([px, py, board_top() + SOCKET_H + TEENSY_HEADER + 1.6 + 1.5]), np.array([-1.0, 0.0, 0.0]), 'carrier',
                         'micro-B', 'Teensy 4.1 USB')
    return out


def report():
    out = {}
    for name, blk in BLOCKS.items():
        fl, x, y, ang, size, ears, what = blk
        out[name] = dict(floor=fl, centre=[x, y], angle=ang, size=size, ears=ears, what=what)
    out['carrier'] = dict(floor=2, corner=list(CARRIER_AT), size=list(CARRIER_SIZE), standoff=CARRIER_STANDOFF, model='pcb/v1 KiCad STEP')
    return out
