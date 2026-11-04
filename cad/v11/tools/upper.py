"""cad/v11 upper robot for layout F (PROPOSAL, Nov 4 2026): the second floor and everything on it. build123d, STEP review only.

    python -B upper.py [build|check|export|render|all]        (omni-robot-sim interpreter)

What it adds over chassis.py (first floor, pods, fins):
 * SECOND FLOOR: a 5 mm PETG plate (z 176.5..181.5, r 218) in four quadrant pieces (each fits the 256 bed). Under it, 6 x 6 lips: round
   the rim and along the two side seams (x = 0), where the quadrants bolt together through the lips (horizontal M3 x 12) on the bench
   before the floor goes on. The rear seam (y = 0) lies on the spine fin's flange: its five floor screws straddle the seam, each head
   clamping both quadrants. The front seam is tied by the arm riser and the screen housing. No lip crosses the fins' flanges or the
   lidar's top. 13 M3 x 12 down into the fins' flanges (chassis.py's floor pilots). Bosses under the floor give every device screw 6 mm.
 * ARM: cad/v10's reconciled riser and adapter (REC_arm_riser, REC_arm_adapter: they were drawn for the same 181.5 floor top and the sim's
   arm height), its 12 V buck on the riser's side, and the SO-101's official parts placed by omni_arm's kinematics at the stow pose.
 * ELECTRONICS BLOCK (layout_study.py's F): computer (BMAX B8A Pro envelope) in four corner clips, battery (OVONIC 6S 5200) in a tray with
   two strap slots, power board and Teensy carrier stacked on one hole pattern (40 mm posts, one M3 x 60 per corner into the floor), servo
   bus adapter by the arm. Labelled (raised text), with the plugs and leads of layout_study.py.
 * BIN DECK over the block on four posts: the v10 bin (120 x 120 x 70) on it, the arm drops into it from above (arm_sweep.py); the deck
   holds the computer and the battery down (3 mm over the computer). On its rear corners the e-stop and the power switch.
 * FACE: the 5 in 800 x 480 panel upright on a frame behind the bin (screen_study.py: in front of the arm the SO-101's own shoulder takes
   the room); its crown carries the USB mic array (on standoffs) and the speaker (in a cup under a grille cover).
Round 12 (Nov 4, after the review): the battery slides out to the left (+y) under the deck (tray open at both ends; strap opened and XT90
unplugged first); the SO-101's base is held by four printed L-clamps (its model has no screw holes); the riser's two floor screws over the
lidar moved off it (they had 5 mm of thread); ribs under the rear quadrants from the spine to the rear deck posts, the rim and the side fins.
Nothing here is printed, wired or load-tested. Purchased parts are envelopes unless a vendor model is named.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import LineString, Point, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chassis as C                                                    # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import layout_study as LS                                              # noqa: E402
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import M3_PILOT, M3_CLR                                      # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

OUT = W.ROOT / 'step_export' / 'upper'
VAL = W.ROOT / 'validation' / 'upper'
V10 = W.ROOT.parent / 'v10 human editable'
V10_STEP = V10 / 'step_parts'
V10_REC = V10 / 'step_export' / 'sim_reconciliation' / 'parts'
box, cyl = P.box, W.cyl

Z_UNDER, Z_TOP = C.FLOOR2_UNDER, C.FLOOR2_UNDER + 5.0                  # 176.5 / 181.5
R_FLOOR = 218.0                                                        # bumper inside r 222.2
LIP_W, LIP_D = 6.0, 6.0                                                # lips under the floor
SEAM = 0.1                                                             # each quadrant stops this far off its seam line
BOSS_D, BOSS_H = 8.0, 4.0                                              # under-floor bosses for device screws (5 + 4 >= 6 engagement + tip)
CB = (7.2, 2.9)
LID_Z = (239.5, 245.5)                                                 # bin deck: 3 mm over the computer (top 236.5)
LID_X, LID_Y = (-200.0, 16.0), 100.0                                   # round 12: front edge 16 (was 12) to carry the front posts
POSTS = ((8.0, 92.0), (8.0, -92.0), (-150.0, 97.0), (-150.0, -97.0))  # clear of the battery's XT90 end and the computer's plugs; round 12:
                                                                       # front posts 3 mm forward, off the battery's way out (+y)
RIBS = (((-150.0, 9.5), (-150.0, 91.0)), ((-150.0, 103.0), (-150.0, 152.0)), ((-145.0, 101.0), (-56.0, 150.0)))   # round 12: under the
RIB_W, RIB_D = 10.0, LIP_D                                             # rear quadrants (and mirrored): spine flange -> rear deck post ->
                                                                       # rim, post -> side fin's tail; as deep as the lips (above the
                                                                       # lidar's band)
ESTOP_XY, SWITCH_XY = (-183.0, 76.0), (-183.0, -76.0)                  # on the deck's rear corners, in printed pods above it (the
                                                                       # computer fills the space under the deck)
POST_W = 14.0
DZ_BIN = LID_Z[1] - Z_TOP                                              # the v10 bin (drawn on the 181.5 plate) raised onto the deck
STOW = 'stow'

COL = dict(floor=(0.80, 0.82, 0.86), riser=(0.55, 0.60, 0.66), deck=(0.80, 0.82, 0.86), post=(0.70, 0.72, 0.76), bracket=(0.40, 0.45, 0.52),
           housing=(0.20, 0.22, 0.25), display=(0.05, 0.05, 0.08), computer=(0.35, 0.45, 0.85), battery=(0.85, 0.25, 0.25),
           power=(0.95, 0.60, 0.10), mcu=(0.20, 0.65, 0.30), servo_adapter=(0.60, 0.30, 0.75), plug=(0.12, 0.12, 0.12),
           label=(0.97, 0.97, 0.97), arm=(0.92, 0.92, 0.92), servo=(0.10, 0.10, 0.12), bin=(0.95, 0.75, 0.20), audio=(0.30, 0.30, 0.34),
           estop=(0.85, 0.10, 0.10), switch=(0.15, 0.15, 0.15), buck=(0.10, 0.40, 0.70))


def colour(name):
    n = name.split('/')[-1]
    for k in sorted(COL, key=len, reverse=True):
        if n.startswith(k) or n.startswith('floor2') and k == 'floor':
            return COL[k]
    if n.startswith(('bolt', 'm3', 'dowel', 'nut')):
        return H.MAGENTA
    return (0.7, 0.7, 0.7)


def prism(g, z0, z1):
    return C.prism(g, z0, z1)


def label(text, at, size=5.0, up=0.6, yaw=0.0):
    """raised text on a top face at `at` (x, y, z), reading along robot x turned by yaw"""
    t = b.Text(text, font_size=size, align=(b.Align.CENTER, b.Align.CENTER))
    s = b.extrude(t, amount=up)
    return s.rotate(b.Axis.Z, yaw).moved(b.Location(at))


def step(path, loc=None):
    s = b.import_step(str(path))
    s = s if isinstance(s, b.Shape) and not isinstance(s, b.Compound) or len(s.solids()) != 1 else s.solids()[0]
    return s if loc is None else s.moved(loc)


def mat_loc(M):
    return W.loc(np.asarray(M, float)[:3, :])


# ---- the fins' flanges and screws (from chassis.py) -------------------------------------------------------------------------------------
def fin_flanges():
    """plan polygons of the three fins' flanges under the floor and their floor-screw points"""
    out, pts = [], []
    for kind, side in (('spine', 1), ('side', 1), ('side', -1)):
        S = C.SPINE if kind == 'spine' else C.SIDE
        u0, u1 = S['top']
        a, d = C.fin_xy(kind, u0, side), C.fin_xy(kind, u1, side)
        n = np.array([-(d - a)[1], (d - a)[0]]) / np.linalg.norm(d - a) * C.FIN_FLANGE / 2
        out.append(Polygon([a + n, d + n, d - n, a - n]))
        pts += [tuple(C.fin_xy(kind, u, side)) for u in S['floor']]
    return union_all(out), pts


# ---- second floor ----------------------------------------------------------------------------------------------------------------------
QUADS = dict(FL=(1, 1), RL=(-1, 1), RR=(-1, -1), FR=(1, -1))


def quad_region(sx, sy):
    q = affinity.translate(sbox(min(0, sx * 400), min(0, sy * 400), max(0, sx * 400), max(0, sy * 400)), sx * SEAM, sy * SEAM)
    return Point(0, 0).buffer(R_FLOOR, 360).intersection(q)


def lidar_keepout():
    return Point(C.LIDAR_X, 0).buffer(FF.LIDAR['d'] / 2 + 4.0, 64)


def build_floor(holes_thru, pilots, flange, fin_pts):
    """four quadrants; holes_thru: [(x, y)] Ø3.4 (screws down through the floor into something below); pilots: [(x, y)] Ø2.8 with a boss
    under the floor unless over the lidar or a fin flange (device screws from above)"""
    no_lip = union_all([flange.buffer(1.0), lidar_keepout(), Point(0, 0).buffer(1)])
    parts, hw, info = {}, [], {}
    seam_screw_y = (30.0, 75.0, 150.0, 190.0)
    ribs = union_all([LineString([(a[0], s_ * a[1]), (c[0], s_ * c[1])]).buffer(RIB_W / 2, 16, cap_style=2) for a, c in RIBS for s_ in (1, -1)])
    keep_off = union_all([Point(x, y).buffer(BOSS_D / 2 + 2.0) for x, y in pilots] + [Point(x, y).buffer(r + 2.0) for x, y, r in GROMMETS])
    ribs = ribs.difference(no_lip).difference(keep_off).intersection(Point(0, 0).buffer(R_FLOOR - 0.5, 360))
    for nm, (sx, sy) in QUADS.items():
        reg = quad_region(sx, sy)
        s = prism(reg, Z_UNDER, Z_TOP)
        rim = reg.difference(Point(0, 0).buffer(R_FLOOR - LIP_W, 360))
        (x0, x1), (y0, y1) = sorted((sx * SEAM, sx * (SEAM + LIP_W))), sorted((sy * 8.0, sy * R_FLOOR))
        seam = sbox(x0, y0, x1, y1)                                       # along the side seam x = 0
        lips = union_all([rim, seam.intersection(reg)]).difference(no_lip)
        if sx < 0:
            lips = union_all([lips, ribs.intersection(reg.buffer(-0.5))])
        if not lips.is_empty:
            s = s + prism(lips, Z_UNDER - LIP_D, Z_UNDER + 0.01)
        for (x, y) in pilots:
            if reg.buffer(-3.0).contains(Point(x, y)):
                if not no_lip.buffer(BOSS_D / 2).contains(Point(x, y)) and not lidar_keepout().buffer(BOSS_D / 2).contains(Point(x, y)):
                    s = s + cyl(BOSS_D / 2, BOSS_H + 0.01, (x, y, Z_UNDER - BOSS_H / 2))
                s = s - cyl(M3_PILOT / 2, 5.0 + BOSS_H + 0.2, (x, y, Z_TOP - (5.0 + BOSS_H) / 2))
        for (x, y) in holes_thru:
            if reg.buffer(4.0).contains(Point(x, y)):
                s = s - cyl(M3_CLR / 2, 5.4 + LIP_D, (x, y, Z_TOP - (5.4 + LIP_D) / 2 + 0.1))
        # side seams: horizontal M3 x 12 through both lips (the RL / RR piece takes the head, the FL / FR the 2.8 pilot)
        for y in seam_screw_y:
            yy = sy * y
            if no_lip.buffer(5).contains(Point(sx * 3, yy)):
                continue
            zc = Z_UNDER - LIP_D / 2
            if sx < 0:
                s = s - W.along_x(M3_CLR / 2, -SEAM - LIP_W - 0.1, -SEAM + 0.1, yy, zc)
                hw.append(H.pan((-SEAM - LIP_W, yy, zc), (1, 0, 0), 14.0, f'bolt_seam_{nm}_{y:g}'))   # 1.8 out past the far lip
            else:
                s = s - W.along_x(M3_PILOT / 2, SEAM - 0.1, SEAM + LIP_W + 0.1, yy, zc)
        # cable grommet holes (motor XT30 leads + CAN up to the power board; D435 / lidar USB up to the computer)
        for (x, y, r) in GROMMETS:
            if reg.contains(Point(x, y)):
                s = s - cyl(r, 30.0, (x, y, Z_TOP - 5.0))
        parts[f'floor2_{nm}'] = s
        info[nm] = dict(bbox=[round(v, 1) for v in tuple(s.bounding_box().size)])
    for x, y in fin_pts:                                                  # down into the fins' flanges (chassis.py's floor pilots)
        hw.append(H.pan((x, y, Z_TOP), (0, 0, -1), 12.0, 'bolt_floor_fin'))
    return parts, hw, info


GROMMETS = ((-105.0, -150.0, 11.0), (100.0, -128.0, 11.0), (-105.0, 150.0, 9.0))   # round 12: rear ones x -78 -> -105, off the ribs


# ---- arm ---------------------------------------------------------------------------------------------------------------------------------
def v10_instances():
    return json.loads((V10 / 'reference' / 'robot_sim_instances.json').read_text())


def arm_kit():
    """v10's riser + adapter (printed), the 12 V buck and its label, and their screws, as v10 placed them (same frame and heights)"""
    parts, vendor, hw = {}, {}, []
    riser = step(V10_REC / 'REC_arm_riser.step')
    parts['riser_adapter'] = step(V10_REC / 'REC_arm_adapter.step')
    # Nov 4: the adapter's four M3 (v10: x 18 into the riser's top) are made long so they clamp the riser's whole height (it prints z up and
    # the arm's moment pulls one side up across its layers): the front two (over the lidar) M3 x 35 into the riser's lowest 10 mm, the rear
    # two M3 x 45 on through the floor into bosses under it
    for x, y in ADAPTER_SCREWS:
        riser = riser + cyl(2.0, 212.0 - Z_TOP - 0.2, (x, y, (212.0 + Z_TOP) / 2))          # fill v10's pilot, then re-cut
        long_ = x < 100
        tip = 219.2 - (45.0 if long_ else 35.0)
        riser = riser - cyl(M3_CLR / 2, 212.2 - (Z_TOP if long_ else tip + 10.0), (x, y, (212.2 + (Z_TOP if long_ else tip + 10.0)) / 2))
        if not long_:
            riser = riser - cyl(M3_PILOT / 2, 10.8, (x, y, tip + 5.0 - 0.4))
    # round 12: v10's two front floor screws (105, +-25) stood over the lidar, where the floor gets no boss (5 mm of thread): moved to
    # (97, +-40.5) in the riser's 4 mm foot plate (head 0.25 off its side wall), over a boss 46.6 from the lidar's axis, M3 x 12
    for (xo, yo), (xn, yn) in RISER_MOVED.items():
        for s_ in (1, -1):
            riser = riser + cyl(M3_CLR / 2 + 0.1, RISER_PLATE[1] - Z_TOP - 0.1, (xo, s_ * yo, (RISER_PLATE[1] + Z_TOP) / 2))
            riser = riser - cyl(M3_CLR / 2, RISER_PLATE[1] - Z_TOP + 0.2, (xn, s_ * yn, (RISER_PLATE[1] + Z_TOP) / 2))
    parts['riser_arm'] = riser
    # round 12: the SO-101's base (its official model has no screw holes: v9's 4 x M3 at x 128 / 192, y +-45 fell just outside its walls)
    # is held at the front by two printed L-clamps: heel on the adapter, toe over the base's wall top (z 239.6), one M3 x 30 each through the
    # heel into a new Ø2.8 pilot through the adapter at y +-48.5 (v10's pilots at y +-45 filled; the rear pair at x 192 too)
    adapter = parts['riser_adapter']
    for s_ in (1, -1):
        adapter = adapter + cyl(1.5, 10.0, (192.0, s_ * 45.0, 217.1))
    for x, (y_wall, toe) in CLAMPS.items():
        for s_ in (1, -1):
            adapter = adapter + cyl(1.5, 10.0, (x, s_ * 45.0, 217.1)) - cyl(M3_PILOT / 2, 10.3, (x, s_ * CLAMP_Y, 217.1))
            y0, y1 = sorted((s_ * (y_wall + 0.6), s_ * (CLAMP_Y + 4.0)))
            yt0, yt1 = sorted((s_ * toe, s_ * (CLAMP_Y + 4.0)))
            c = box(x - 5.0, x + 5.0, y0, y1, ADAPTER_TOP, BASE_WALL_TOP + 0.1) + box(x - 5.0, x + 5.0, yt0, yt1, BASE_WALL_TOP + 0.1, BASE_WALL_TOP + 4.1)
            c = c - cyl(M3_CLR / 2, 22.2, (x, s_ * CLAMP_Y, (ADAPTER_TOP + BASE_WALL_TOP + 4.1) / 2))
            n = f'clamp_so101_{"l" if s_ > 0 else "r"}'
            parts[n] = c
            hw.append(H.pan((x, s_ * CLAMP_Y, BASE_WALL_TOP + 4.1), (0, 0, -1), 30.0, 'bolt_clamp_so101'))
    # the base's rear half lies under the shoulder's sweep (r 37.7 about the pan axis at x 178.8, down to z 240.7): no clamp there. Four
    # M2.5 x 14 come up from under the adapter (driven from the open side, z 181.5..212) into four of the base's own Ø2.4 holes in its
    # underside (their depth in the real base VERIFY)
    for x, y in BASE_SCREWS:
        for s_ in (1, -1):
            adapter = adapter - cyl(1.45, 10.4, (x, s_ * y, (212.0 + ADAPTER_TOP) / 2))
            hw.append(H.pan((x, s_ * y, 212.0), (0, 0, 1), 14.0, 'bolt_so101_base', size=2.5))
    parts['riser_adapter'] = adapter
    for r in v10_instances():
        if not r['group'].startswith('mounts'):
            continue
        M = np.array(r['matrix'], float)
        if r['product'] in ('12_V_buck_arm', 'label_12V_BUCK_ARM'):
            vendor['buck_12v_arm' if r['product'].startswith('12') else 'label_buck_12v_arm'] = (V10_STEP / f"{r['product']}.step", M)
        elif r['product'].startswith('M3_x_'):
            o, z = M[:3, 3], M[:3, 2]
            if np.allclose(np.abs(z), (0.64, 0, 0.77), atol=0.02) or abs(o[1]) > 58 and o[2] < 190:   # the cheeks' screws: no cheeks here
                continue
            L = float(r['product'].split('_')[2])
            if abs(o[2] - 219.2) < 0.5:                                    # the adapter's screws (see above)
                L = 45.0 if o[0] < 100 else 35.0
            moved = RISER_MOVED.get((round(float(o[0]), 1), round(abs(float(o[1])), 1)))
            if moved and abs(o[2] - RISER_PLATE[1]) < 0.5:
                o = np.array([moved[0], math.copysign(moved[1], o[1]), o[2]])
                L = 12.0
            hw.append(H.pan(tuple(o), tuple(-z), L, f"bolt_arm_kit_M3x{L:g}"))
    return parts, vendor, hw


ADAPTER_SCREWS = ((70.0, 40.0), (70.0, -40.0), (122.0, 40.0), (122.0, -40.0))
RISER_PLATE = (Z_TOP, 185.5)                                           # the v10 riser's foot plate
RISER_MOVED = {(105.0, 25.0): (97.0, 40.5)}
ADAPTER_TOP, BASE_WALL_TOP = 222.1, 239.6                             # the SO-101 base stands on the adapter; its side walls' top
CLAMP_Y = 48.5
CLAMPS = {128.0: (43.8, 40.0)}                                         # x: (the base wall's outer |y| there, the toe's inner |y|)
BASE_SCREWS = ((177.0, 47.0), (190.7, 16.5))                           # the base's Ø2.4 underside holes used (and mirrored)


def arm_parts_at(pose=STOW):
    import screen_study as SS
    kin = SS.ArmKinematics(SS.ArmConfig())
    q = kin.cfg.poses[pose]
    out = {}
    for n, (prod, T) in enumerate(SS.part_poses(kin, q)):
        out[f'arm_{n:02d}_{prod.replace("SO-101_", "")}'] = (V10_STEP / f'{prod}.step', T)
    return out, q


# ---- electronics block --------------------------------------------------------------------------------------------------------------
BLOCK = dict(computer=dict(xy=(-135.0, 1.0), yaw=90, z0=Z_TOP + 3.0),           # its corners on the four clips' 3 mm floors
             battery=dict(xy=(-26.0, 1.0), yaw=90, z0=Z_TOP + 2.0),             # in its tray (2 mm floor); 13 mm forward of layout F (clips)
             power=dict(xy=(9.0, -135.0), yaw=0, z0=Z_TOP + 6.0),               # on four 6 mm spacers
             mcu=dict(xy=(9.0, -135.0), yaw=0, z0=Z_TOP + 6.0 + 32.0 + 10.0),   # on four 10 mm standoffs on the power board's lid
             servo_adapter=dict(xy=(21.0, -7.0), yaw=90, z0=Z_TOP + 5.0))       # on two 5 mm spacers
LABELS = dict(computer='BMAX B8A PRO', battery='6S 5200 LIPO', power='POWER BOARD', mcu='TEENSY 4.1', servo_adapter='SERVO BUS')


def place(shape, xy, yaw, z0):
    return shape.rotate(b.Axis.Z, yaw).moved(b.Location((xy[0], xy[1], z0)))


def to_robot(local_xy, xy, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (xy[0] + c * local_xy[0] - s * local_xy[1], xy[1] + s * local_xy[0] + c * local_xy[1])


def electronics():
    """devices (purchased: envelopes), their plugs, labels; printed holders; screws; floor pilots"""
    devs = {d['name']: d for d in LS.devices()}
    c = devs['computer']                                                # layout_study stacked the two rear USB-A 2 plugs 17 mm apart: their
    devs['computer'] = LS.device('computer', c['size'], c['mass'], zones=[('+x', 12.0)], note=c['note'],   # leads clashed; side by side
                                 ports=[('usb_a3', '+x', -40, 20, '+z', 'D435 (USB 3)'), ('usb_a3', '+x', -22, 20, '+z', 'Teensy'),
                                        ('usb_a2', '-x', 18, 15, '+z', 'SO-101 bus adapter'), ('usb_a2', '-x', 38, 15, '+z', 'A2M12 adapter'),
                                        ('dc_barrel', '-x', -40, 22, '+z', '19 V in')])
    bought, printed, hw, pilots = {}, {}, [], []
    for name, at in BLOCK.items():
        d = devs[name]
        l, w, h = d['size']
        bought[name] = place(d['body'], at['xy'], at['yaw'], at['z0'])
        for n, (lab, pl) in enumerate(d['plugs']):
            bought[f'plug_{name}_{n}'] = place(pl, at['xy'], at['yaw'], at['z0'])
        bought[f'label_{name}'] = label(LABELS[name], (*at['xy'], at['z0'] + h), size=min(7.0, w / 9), yaw=at['yaw'] + 90)
    # computer: four corner clips (printed L, 20 tall, 2.5 walls), each one M3 x 12 into a floor boss
    d, at = devs['computer'], BLOCK['computer']
    l, w, h = d['size']
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        c = np.array([sx * l / 2, sy * w / 2])
        clip = box(-14, 2.5, -14, 2.5, 0, 20.0) - box(-14.1, 0.1, -14.1, 0.1, 3.0, 20.1)          # corner: walls outside the device
        tab = box(2.5, 14.0, -6, 6, 0, 3.0) - cyl(M3_CLR / 2, 3.2, (9.0, 0, 1.5))
        clip = clip + tab
        clip = clip.mirror(b.Plane.YZ) if sx < 0 else clip
        clip = clip.mirror(b.Plane.XZ) if sy < 0 else clip
        clip = clip.moved(b.Location((c[0] - sx * 0.0, c[1], 0)))
        printed[f'bracket_computer_{i}'] = place(clip, at['xy'], at['yaw'], Z_TOP)
        sp = to_robot((c[0] + sx * 9.0, c[1]), at['xy'], at['yaw'])
        pilots.append(sp)
        hw.append(H.pan((*sp, Z_TOP + 3.0), (0, 0, -1), 12.0, 'bolt_clip_computer'))
    # battery tray: 2 mm floor, 2.5 walls 30 tall, strap slots; four tabs with M3 x 12
    d, at = devs['battery'], BLOCK['battery']
    l, w, h = d['size']
    t = box(-l / 2 - 2.5, l / 2 + 2.5, -w / 2 - 2.5, w / 2 + 2.5, 0, 30.0) - box(-l / 2, l / 2, -w / 2, w / 2, 2.0, 30.1)
    t = t - box(-l / 2 - 0.1, -l / 2 + 12, -w / 2 - 3, w / 2 + 3, 2.0, 30.1).moved(b.Location((0, 0, 0)))   # the XT90 end stays open
    t = t - box(l / 2 - 0.1, l / 2 + 2.6, -w / 2 - 3, w / 2 + 3, 2.0, 30.1)      # round 12: the +u end (robot +y) open too: the pack slides
    for u in (-l / 4, l / 4):                                                     # out to the left under the deck (strap opened, XT90 unplugged)
        t = t - box(u - 11, u + 11, -w / 2 - 3, w / 2 + 3, 8.0, 11.0) - box(u - 11, u + 11, -w / 2 + 4, w / 2 - 4, -0.1, 2.1)   # strap slots
    for i, (u, v) in enumerate(((-l / 2 + 20, w / 2 + 8), (l / 2 - 20, w / 2 + 8), (-l / 2 + 20, -w / 2 - 8), (l / 2 - 20, -w / 2 - 8))):   # round 12: tabs
        # 5 mm further out (they touched the strap)
        t = t + (box(u - 6, u + 6, min(v, 0) * 0 + v - 6, v + 6, 0, 3.0) - cyl(M3_CLR / 2, 3.2, (u, v, 1.5)))
        sp = to_robot((u, v), at['xy'], at['yaw'])
        pilots.append(sp)
        hw.append(H.pan((*sp, Z_TOP + 3.0), (0, 0, -1), 12.0, 'bolt_tray_battery'))
    printed['bracket_battery_tray'] = place(t, at['xy'], at['yaw'], Z_TOP)
    bought['strap_battery'] = place(box(-l / 4 - 10, -l / 4 + 10, -w / 2 - 3.5, w / 2 + 3.5, 2.0, h + 3.0) - box(-l / 4 - 11, -l / 4 + 11, -w / 2 - 2.5, w / 2 + 2.5, 0, h + 2.0), at['xy'], at['yaw'], Z_TOP) \
        + place(box(l / 4 - 10, l / 4 + 10, -w / 2 - 3.5, w / 2 + 3.5, 2.0, h + 3.0) - box(l / 4 - 11, l / 4 + 11, -w / 2 - 2.5, w / 2 + 2.5, 0, h + 2.0), at['xy'], at['yaw'], Z_TOP)
    # power board and Teensy carrier stacked on one hole pattern (round 12; the carrier is our own board (est), laid out on the power
    # board's four corner holes; round 11 screwed its standoffs into the power board's lid): per corner a 6 mm spacer under the power
    # board's ear, a 40 mm printed post between the two boards, and one M3 x 60 from the carrier's ear down through both into a floor boss
    dp, ap = devs['power'], BLOCK['power']
    lp, wp, hp = dp['size']
    am = BLOCK['mcu']
    lm = devs['mcu']['size'][0]
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        u, v = sx * (lp / 2 + 4.0), sy * (wp / 2 - 6.0)
        bought[f'ear_power_{i}'] = place(box(u - 4, u + 4, v - 4, v + 4, 0, 2.0) - cyl(M3_CLR / 2, 2.2, (u, v, 1.0)), ap['xy'], ap['yaw'], ap['z0'])
        um = sorted((sx * lm / 2, u + sx * 4.0))                                                   # the carrier's tab out to the hole
        bought[f'ear_mcu_{i}'] = place(box(*um, v - 4, v + 4, 0, 2.0) - cyl(M3_CLR / 2, 2.2, (u, v, 1.0)), am['xy'], am['yaw'], am['z0'])
        sp = to_robot((u, v), ap['xy'], ap['yaw'])
        printed[f'post_spacer_power_{i}'] = cyl(3.5, 6.0, (*sp, ap['z0'] - 3.0)) - cyl(M3_CLR / 2, 6.2, (*sp, ap['z0'] - 3.0))
        hs = am['z0'] - ap['z0'] - 2.0
        printed[f'post_mcu_{i}'] = cyl(3.5, hs, (*sp, ap['z0'] + 2.0 + hs / 2)) - cyl(M3_CLR / 2, hs + 0.2, (*sp, ap['z0'] + 2.0 + hs / 2))
        hw.append(H.pan((*sp, am['z0'] + 2.0), (0, 0, -1), 60.0, 'bolt_mcu_stack'))
        pilots.append(sp)
    # servo adapter: two 5 mm spacers, M3 x 12
    d, at = devs['servo_adapter'], BLOCK['servo_adapter']
    l, w, h = d['size']
    for i, sx in enumerate((1, -1)):
        u, v = sx * (l / 2 + 4.0), 0.0
        bought[f'ear_servo_adapter_{i}'] = place(box(u - 4, u + 4, v - 4, v + 4, 0, 1.6) - cyl(M3_CLR / 2, 2.0, (u, v, 0.8)), at['xy'], at['yaw'], at['z0'])
        sp = to_robot((u, v), at['xy'], at['yaw'])
        printed[f'post_spacer_servo_adapter_{i}'] = cyl(3.5, 5.0, (*sp, Z_TOP + 2.5)) - cyl(M3_CLR / 2, 5.2, (*sp, Z_TOP + 2.5))
        hw.append(H.pan((*sp, at['z0'] + 1.6), (0, 0, -1), 14.0, 'bolt_servo_adapter'))
        pilots.append(sp)
    return bought, printed, hw, pilots


# ---- bin deck, bin, audio, e-stop ----------------------------------------------------------------------------------------------------------
def deck():
    printed, bought, vendor, hw, floor_pilots = {}, {}, {}, [], []
    reg = sbox(LID_X[0], -LID_Y, LID_X[1], LID_Y).intersection(Point(0, 0).buffer(R_FLOOR - 3.0, 360))
    lid = prism(reg, *LID_Z)
    for i, (x, y) in enumerate(POSTS):                                     # posts: M3 x 16 up from under the floor; M3 x 12 down through the deck
        p = box(x - POST_W / 2, x + POST_W / 2, y - POST_W / 2, y + POST_W / 2, Z_TOP, LID_Z[0])
        p = p - cyl(M3_PILOT / 2, 12.2, (x, y, Z_TOP + 6.0)) - cyl(M3_PILOT / 2, 8.2, (x, y, LID_Z[0] - 4.0))
        printed[f'post_deck_{i}'] = p
        hw.append(H.pan((x, y, Z_UNDER - BOSS_H), (0, 0, 1), 16.0, 'bolt_post_floor'))   # head under the floor's boss
        hw.append(H.pan((x, y, LID_Z[1]), (0, 0, -1), 12.0, 'bolt_post_deck'))
        lid = lid - cyl(M3_CLR / 2, 6.4, (x, y, sum(LID_Z) / 2))
        floor_pilots.append((x, y))
    # the v10 bin raised onto the deck: its four M3 x 10 into the deck (6 thick) -- from the v10 table
    vendor['bin'] = (V10_STEP / 'bin.step', W_T(0, 0, DZ_BIN))
    for r in v10_instances():
        if r['group'].startswith('mounts') and r['product'] == 'M3_x_10_PAN':
            M = np.array(r['matrix'], float)
            if M[0, 3] < 0:
                o = M[:3, 3] + np.array([0, 0, DZ_BIN])
                hw.append(H.pan(tuple(o), (0, 0, -1), 10.0, 'bolt_bin'))
                lid = lid - cyl(M3_PILOT / 2, 6.2, (o[0], o[1], sum(LID_Z) / 2))
    # e-stop (22 mm panel button, Ø40 mushroom) and the power switch (22 mm) in printed pods on the deck (their bodies above it)
    for nm, (x, y), cap in (('estop', ESTOP_XY, 20.0), ('switch', SWITCH_XY, 12.0)):
        pod = cyl(15.0, 36.0, (x, y, LID_Z[1] + 18.0)) - cyl(13.0, 34.0, (x, y, LID_Z[1] + 17.0 - 0.1)) - cyl(11.2, 2.2, (x, y, LID_Z[1] + 35.0))
        pod = pod + (box(x - 15, x + 15, y - 21, y + 21, LID_Z[1], LID_Z[1] + 3.0) - cyl(13.0, 3.2, (x, y, LID_Z[1] + 1.5)))
        for dy in (-17.0, 17.0):
            pod = pod - cyl(M3_CLR / 2, 3.2, (x, y + dy, LID_Z[1] + 1.5))
            hw.append(H.pan((x, y + dy, LID_Z[1] + 3.0), (0, 0, -1), 10.0, f'bolt_{nm}_pod'))
            lid = lid - cyl(M3_PILOT / 2, 6.2, (x, y + dy, sum(LID_Z) / 2))
        printed[f'housing_{nm}'] = pod
        top = LID_Z[1] + 36.0
        bought[nm] = cyl(cap, 14.0 if nm == 'estop' else 8.0, (x, y, top + (7.0 if nm == 'estop' else 4.0))) + cyl(11.0, 32.0, (x, y, top - 16.0 - 0.01))
    bought['label_estop'] = label('STOP', (ESTOP_XY[0], ESTOP_XY[1], LID_Z[1] + 36.0 + 14.0), 7.0, yaw=90)
    printed['deck_bin'] = lid
    return printed, bought, vendor, hw, floor_pilots


def W_T(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


# ---- face (behind the bin) -------------------------------------------------------------------------------------------------------------
FACE_X = (-160.0, -148.0)                                              # the panel (12 deep), facing forward over the bin (its back wall x -130)
FACE_Z0 = 322.0                                                        # panel bottom: over the bin's top (315.5)
FRAME_RAIL = 7.0                                                       # side rails (y) beside the panel


def face():
    """the v10 5 in panel (12 x 121 x 77; its frame: x depth, y width, z height from a corner) upright on a printed frame that stands on
    the deck behind the bin: two legs and the panel's surround in one part (prints on its back), the panel drops in from the top, a crown
    cap screws on and carries the USB mic array and the speaker (v10 models). screen_study.py: the 5 in panel does not fit in front of
    the arm (the SO-101's own shoulder stands over x 205..217 at z 241)"""
    x0, x1 = FACE_X
    hw, pilots = [], []
    vendor = dict(display=(V10_STEP / '5_in_display_800x480.step', W_T(x0, -60.5, FACE_Z0)))
    yi, yo = 60.7, 60.7 + FRAME_RAIL
    zt = FACE_Z0 + 77.0
    xb = x0 - 3.0                                                          # back plate behind the panel
    fr = box(xb, x1 + 1.0, -yo, yo, FACE_Z0 - 4.0, zt) - box(x0, x1 + 1.1, -yi, yi, FACE_Z0, zt + 0.1)   # back + rails + bottom
    fr = fr - box(x1 - 0.1, x1 + 1.1, -yi + 3.0, yi - 3.0, FACE_Z0 + 3.0, zt + 0.1)                      # bezel: a 3 mm lip round the glass
    fr = fr - box(x0 - 0.1, x1 - 2.0, -30.0, 30.0, FACE_Z0 - 4.1, FACE_Z0 + 0.1)                         # the panel's leads out at the bottom
    for s_ in (1, -1):                                                     # legs down to the deck, a foot each with 2 x M3 x 10
        leg = box(xb, xb + 14.0, s_ * yi if s_ > 0 else -yo, yo if s_ > 0 else -yi, LID_Z[1], FACE_Z0 - 4.0)
        foot = box(xb, xb + 31.0, min(s_ * yi, s_ * yo), max(s_ * yi, s_ * yo), LID_Z[1], LID_Z[1] + 4.0)   # forward of the leg (the bin
        fr = fr + leg + foot                                                                                  # wall is at x -130)
        for x in (xb + 18.0, xb + 26.0):
            y = s_ * (yi + yo) / 2
            fr = fr - cyl(M3_CLR / 2, 4.2, (x, y, LID_Z[1] + 2.0))
            hw.append(H.pan((x, y, LID_Z[1] + 4.0), (0, 0, -1), 10.0, 'bolt_face_deck'))
            pilots.append((x, y))
    # crown (round 12: fixings for the mics and the speaker): the mic board on four Ø7 x 3 standoffs, M3 x 10 through its Ø3.2 corner
    # holes (v10 model); the speaker in a cup (2 mm walls) under a grille cover held by two M3 x 10 into Ø8 bosses beside the cup. The
    # crown's two screws into the frame's rails move to x -158 (off the mic's standoffs); the crown grows back to x -204 for the speaker
    xc0, ys = -204.0, 71.0
    crown = box(xc0, x1 + 1.0, -yo, ys, zt, zt + 4.0)
    for y in (-(yi + yo) / 2, (yi + yo) / 2):
        fr = fr - cyl(M3_PILOT / 2, 8.2, (CROWN_SCREW_X, y, zt - 4.0))
        crown = crown - cyl(M3_CLR / 2, 4.2, (CROWN_SCREW_X, y, zt + 2.0))
        hw.append(H.pan((CROWN_SCREW_X, y, zt + 4.0), (0, 0, -1), 10.0, 'bolt_face_crown'))
    mic = step(V10_STEP / 'USB_mic_array.step')
    mb = mic.bounding_box()
    mx, my, mz = xb - 18.0 - mb.min.X, -64.0 - mb.min.Y, zt + 4.0 + MIC_STANDOFF - mb.min.Z           # front standoffs 1 mm in from the crown's edge
    vendor['mic_array'] = (V10_STEP / 'USB_mic_array.step', W_T(mx, my, mz))
    for hx, hy in MIC_HOLES:                                               # the model's hole axes (its own frame)
        x, y = mb.min.X + hx + mx, mb.min.Y + hy + my
        crown = crown + cyl(3.5, MIC_STANDOFF + 0.5, (x, y, zt + 3.5 + (MIC_STANDOFF + 0.5) / 2)) - cyl(M3_PILOT / 2, 4.0 + MIC_STANDOFF + 0.2, (x, y, zt + (4.0 + MIC_STANDOFF) / 2))
        hw.append(H.pan((x, y, zt + 4.0 + MIC_STANDOFF + 1.6), (0, 0, -1), 10.0, 'bolt_mic'))
    (sx0, sx1), (sy0, sy1) = SPEAKER_X, SPEAKER_Y
    vendor['speaker'] = (V10_STEP / 'micro_speaker_4_ohm_3_W.step', W_T(sx0, sy0, zt + 4.0))
    top = zt + 4.0 + 12.0
    crown = crown + (box(sx0 - 2.0, sx1 + 2.0, sy0 - 2.0, sy1 + 2.0, zt + 3.5, top) - box(sx0 - 0.2, sx1 + 0.2, sy0 - 0.2, sy1 + 0.2, zt + 4.0, top + 0.1))
    cover = box(sx0 - 10.0, sx1 + 10.0, sy0 - 2.0, sy1 + 2.0, top, top + 2.0)
    for i in range(5):                                                     # grille slots
        cover = cover - box(sx0 + 3.0, sx1 - 3.0, sy0 + 4.0 + 7.0 * i, sy0 + 7.0 + 7.0 * i, top - 0.1, top + 2.1)
    for x in (sx0 - 5.5, sx1 + 5.5):                                       # 0.5 into the cup's wall (tangent broke the solid in SolidWorks)
        y = (sy0 + sy1) / 2
        crown = crown + cyl(4.0, 12.5, (x, y, zt + 3.5 + 6.25)) - cyl(M3_PILOT / 2, 8.2, (x, y, top - 4.0))
        cover = cover - cyl(M3_CLR / 2, 2.2, (x, y, top + 1.0))
        hw.append(H.pan((x, y, top + 2.0), (0, 0, -1), 10.0, 'bolt_speaker_cover'))
    return dict(housing_face=fr, housing_face_crown=crown, housing_speaker_cover=cover), vendor, hw, pilots


CROWN_SCREW_X = -158.0
MIC_STANDOFF = 3.0
MIC_HOLES = ((2.4, 3.0), (29.4, 3.0), (2.4, 83.0), (29.4, 83.0))       # from the model's min corner (its Ø3.2 holes)
SPEAKER_X, SPEAKER_Y = (-192.0, -164.0), (26.0, 66.0)                 # the 28 x 40 x 12 speaker, behind the frame


# ---- build -------------------------------------------------------------------------------------------------------------------------------
def build():
    flange, fin_pts = fin_flanges()
    kit, kit_vendor, kit_hw = arm_kit()
    el_bought, el_printed, el_hw, el_pilots = electronics()
    dk_printed, dk_bought, dk_vendor, dk_hw, dk_pilots = deck()
    sc_printed, sc_vendor, sc_hw, sc_pilots = face()
    riser_pilots = [tuple(np.asarray(h['p'])[:2]) for h in kit_hw if abs(h['d'][2]) > 0.9 and (h['p'][2] < 190 or h['L'] > 40)]
    floor, fl_hw, fl_info = build_floor(fin_pts, el_pilots + dk_pilots + riser_pilots, flange, fin_pts)
    for x, y in sc_pilots:                                                # the face frame's feet: pilots in the deck (6 thick)
        dk_printed['deck_bin'] = dk_printed['deck_bin'] - cyl(M3_PILOT / 2, 6.2, (x, y, sum(LID_Z) / 2))
    # the floor pilots under the deck posts take M3 x 16 from below: a through Ø3.4 there instead of a pilot (the post is the nut)
    for k in floor:
        for x, y in dk_pilots:
            floor[k] = floor[k] - cyl(M3_CLR / 2, 12.0, (x, y, Z_TOP - 3.0))
    arm, q = arm_parts_at()
    printed = dict(floor, **kit, **el_printed, **dk_printed, **sc_printed)
    bought = dict(el_bought, **dk_bought)
    vendor = dict(kit_vendor, **dk_vendor, **sc_vendor, **arm)
    hw = fl_hw + kit_hw + el_hw + dk_hw + sc_hw
    return printed, bought, vendor, hw, dict(floor=fl_info, arm_pose=[round(float(v), 3) for v in q], fin_screws=len(fin_pts))


def load_vendor(vendor):
    out = {}
    for k, (path, T) in vendor.items():
        s = b.import_step(str(path))
        out[k] = s.moved(mat_loc(T))
    return out


def engagement(hw, solids):
    out = {}
    for n, r in enumerate(hw):
        p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
        rp = {3.0: (1.42, 1.68), 2.5: (1.05, 1.32), 4.0: (1.7, 2.15)}[r['size']]
        pl = b.Plane(origin=tuple(p), z_dir=tuple(d))
        tube = b.Solid.make_cylinder(rp[1], r['L'], pl) - b.Solid.make_cylinder(rp[0], r['L'] + 0.1, b.Plane(origin=tuple(p - 0.05 * d), z_dir=tuple(d)))
        v = sum(W.overlap(tube, s) for s in solids.values() if W.near(tube, s, 0.0))
        out[f"{r['name']}#{n}"] = round(v / (math.pi * (rp[1] ** 2 - rp[0] ** 2)), 1)
    return out


# ---- checks ----------------------------------------------------------------------------------------------------------------------------------
def check(printed, bought, vendor, hw, info):
    """every part one valid solid on the bed; overlaps between every pair of upper parts, and against the chassis, the fins and the pods'
    static parts (moving parts swept); fasteners: each screw's shank must end inside a printed part (thread) or a nut"""
    import pod_lean as L
    rep = dict(info=info)
    rep['printed'] = {k: dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1),
                              bbox=[round(v, 1) for v in tuple(s.bounding_box().size)],
                              bed_fit=bool(sorted(tuple(s.bounding_box().size))[1] <= 256)) for k, s in printed.items()}
    vend = load_vendor(vendor)
    allp = dict(printed, **bought, **vend)
    # pairwise overlaps (bounding boxes first)
    keys = list(allp)
    ov = {}
    for i, a in enumerate(keys):
        for c in keys[i + 1:]:
            if a.startswith('arm_') and c.startswith('arm_'):
                continue                                                   # the arm's own parts touch at their joints
            if W.near(allp[a], allp[c], 0.0):
                try:
                    v = W.overlap(allp[a], allp[c])
                except Exception:                                          # noqa: BLE001
                    v = -1
                if v > 0.05 or v < 0:
                    ov[f'{a} / {c}'] = round(v, 2)
    rep['overlaps_mm3'] = ov
    # against the chassis, fins, pods (static) and the moving pods' sweep (their bounding: wheel top 155)
    parts, chw, env, g = C.build(FF.pod_geometry())
    lower = {k: v for k, v in parts.items()}
    lower.update({k: v for k, v in env.items() if k != 'env_arm_riser'})
    ov2 = {}
    for a, sa in allp.items():
        for c, sc in lower.items():
            if W.near(sa, sc, 0.0):
                v = W.overlap(sa, sc)
                if v > 0.05:
                    ov2[f'{a} / {c}'] = round(v, 2)
    rep['overlaps_with_chassis_mm3'] = ov2
    # round 12: the battery's way out: the pack (and its label) slid along +y under the deck, against everything but its tray and strap
    out = {}
    pack = {k: allp[k] for k in ('battery', 'label_battery')}
    rest = {k: v for k, v in dict(allp, **lower).items() if k not in pack and k not in ('bracket_battery_tray', 'strap_battery')}
    for t in (5.0, 20.0, 40.0, 60.0, 90.0, 120.0, 150.0, 180.0):
        for k, s in pack.items():
            sm = s.moved(b.Location((0, t, 0)))
            for c, sc in rest.items():
                if W.near(sm, sc, 0.0) and W.overlap(sm, sc) > 0.05:
                    out.setdefault(str(t), []).append(f'{k} / {c}')
    rep['battery_slide_out_+y'] = dict(contacts=out, clear=not out, note='strap opened and the XT90 unplugged first (reached under the deck from the right side)')
    rep['lidar_top_to_floor_mm'] = round(Z_UNDER - (FF.LIDAR_BASE_Z + FF.LIDAR['h']), 2)
    # fasteners: thread engagement = shank length that has plastic between the pilot (r 1.4) and the clearance (r 1.7) radius: a tube
    # r 1.42..1.68 along the shank picks up material only where the hole is a pilot (or blind / solid), never in a Ø3.4 clearance hole
    rep['thread_engagement_mm'] = engagement(hw, dict(printed, **lower, **vend))
    vendor_thread = ('bolt_so101_base',)                                  # into the SO-101 base's own holes: its depth VERIFY, not our pilots
    rep['screws_short_of_6mm'] = {k: v for k, v in rep['thread_engagement_mm'].items() if v < 5.9 and not k.startswith(vendor_thread)}
    rep['screws_into_vendor_parts'] = {k: v for k, v in rep['thread_engagement_mm'].items() if k.startswith(vendor_thread)}
    rep['fastener_count'] = {}
    for r in hw:
        nm = H.part_name(r)
        rep['fastener_count'][nm] = rep['fastener_count'].get(nm, 0) + 1
    return rep


# ---- export / render ----------------------------------------------------------------------------------------------------------------------
def export(printed, bought, vendor, hw):
    """printed parts only, each alone and as one assembly STEP (no bought parts, vendor models or fasteners: Fernando's STEP rule)"""
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for k, s in printed.items():
        write_part(s, colour(k), OUT / 'parts' / f'{k}.step', k)
    write_step({k: (s, colour(k)) for k, s in printed.items()}, OUT / 'upper_assembly.step', 'upper')


def render(printed, bought, vendor, hw):
    import pod_lean as L
    vend = load_vendor(vendor)
    items = dict(printed, **bought, **vend)
    for n, r in enumerate(hw):
        items[f'bolt_{n}'] = H.solid(r)
    cf = lambda k: H.MAGENTA if k.startswith('bolt_') else (COL['servo'] if 'STS3215' in k else (COL['arm'] if k.startswith('arm_') else colour(k)))   # noqa: E731
    L.iso(items, VAL / 'iso_upper_front.png', 35, 25, 'upper robot (layout F proposal): floor, arm kit + SO-101 (stow), screen, electronics, bin deck', cf, tol=0.6)
    L.iso(items, VAL / 'iso_upper_rear.png', 215, 25, 'upper robot from the rear left: bin deck, audio, e-stop over the electronics block', cf, tol=0.6)
    under = {k: v for k, v in items.items() if not k.startswith(('arm_', 'deck', 'bin', 'housing_audio', 'mic', 'speaker', 'estop', 'switch', 'label_estop'))}
    L.iso(under, VAL / 'iso_block.png', 215, 35, 'the electronics block with the bin deck, the bin and the arm hidden', cf, tol=0.6)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    printed, bought, vendor, hw, info = build()
    for k, s in printed.items():
        print(k, len(s.solids()), s.is_valid, round(s.volume / 1000, 1), flush=True)
    print(len(hw), 'fasteners', json.dumps(info), flush=True)
    if stage in ('check', 'all'):
        rep = check(printed, bought, vendor, hw, info)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k not in ('printed', 'last_6mm_of_each_screw_in_plastic_mm')}, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(printed, bought, vendor, hw)
    if stage in ('render', 'all'):
        render(printed, bought, vendor, hw)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
