"""cad/v15 robot wiring harness (v13 Nov 11 2026; ported to v15's mid deck Nov 14: harness_route's world, the motor bundle's tie on the motor
plate; PROPOSAL; nothing bought, wired or built). Inventory, power budget, the leads as solids, checks.

    python -B harness.py inventory | power | build | check | render | all      (omni-robot-sim interpreter, from tools/)

 * inventory: every electrical connection (device port -> device port, signal, connector, lead OD, bend radius, what it must survive), from
   CONNECTIONS below -> validation/harness/inventory.json + inventory.md (the README table).
 * power: a load table per rail and what each converter, fuse and wire must carry -> validation/harness/power.json + power.md.
 * build / check / render: the leads as tubes along harness_route.py's routes (validation/harness/routes.json), bends at the allowed radius,
   the mating plugs on each device (envelopes: real catalogue sizes where known, else ASSUMED), checked against every robot part (chassis,
   pods in droop / nominal / bump, upper robot, arm at stow; the arm's sweep and the pods' slide-out by harness_route.py) -> checks.json,
   pictures, step_export/harness/harness_review.step (purchased items: for review only, not a print file).
Connector, lead OD and port positions are ASSUMED unless a vendor model or the CAD shows them (marked VERIFY); nothing here is measured.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
VAL = ROOT / 'validation' / 'harness'
OUT = ROOT / 'step_export' / 'harness'

# ---- wire: silicone hook-up wire, fine strand (typical catalogue ODs and copper resistance; current = continuous in a bundle, conservative) ---
WIRE = {10: (5.0, 0.00328, 40.0), 12: (4.0, 0.00521, 30.0), 14: (3.3, 0.00828, 22.0), 16: (2.7, 0.0132, 15.0), 18: (2.3, 0.0209, 10.0),
        20: (1.8, 0.0333, 7.0), 22: (1.6, 0.0530, 5.0), 24: (1.3, 0.0842, 3.5), 26: (1.1, 0.134, 2.2), 28: (0.9, 0.213, 1.4)}   # AWG: OD, ohm/m, A

# ---- leads (what is routed): OD of the bundle as drawn, tightest bend (centreline radius) allowed, what is in it. All ASSUMED (VERIFY)
LEADS = {
    'motor': (8.0, 16.0, '2 x 18 AWG silicone (bus +/-) + 2 twisted pairs 26 AWG (CAN in, CAN out) in braided sleeve'),
    'bat': (9.0, 12.0, "the pack's own XT90 pigtail, 2 x 10 AWG (HRB 6S 3300: VERIFY connector, gauge and length)"),
    'pair12': (7.0, 14.0, '2 x 12 AWG silicone, twisted'),
    'pair18': (5.0, 10.0, '2 x 18 AWG silicone, twisted'),
    'pair20': (3.8, 8.0, '2 x 20 AWG silicone, twisted'),
    'can': (3.0, 6.0, '2 twisted pairs 26 AWG (CAN in / out; the motor bundle\'s signal branch to the carrier)'),
    'usb3': (4.6, 23.0, 'USB 3 lead (the D435 lead is drawn Ø4.5, VERIFY)'),
    'usb2': (3.8, 19.0, 'USB 2 lead'),
    'usbc': (5.0, 25.0, 'USB-C lead'),
    'hdmi': (6.0, 6.0, 'flat FPC HDMI cable (a round HDMI lead is Ø7, R 35: too stiff here), ASSUMED'),
    'lidar': (3.5, 10.0, "the A2M12's own lead to its USB adapter (VERIFY)"),
    'imu': (5.0, 15.0, '10 x 26 AWG (SPI 4 + INT, RST, WAKE, BOOTN + 3V3 / GND), round jacket'),
    'multi4': (4.5, 18.0, '4 x 22 AWG in a jacket (button contacts + LED)'),
}

# ---- connectors: body across (w) x high (h) x how far the mated plug stands out of its device (L), the part, status ------------------------
def xh(n):
    return (2.5 * (n - 1) + 4.9, 5.75, 9.8)                              # JST XHP-n housing (catalogue A = 2.5 (n - 1) + 4.9), ASSUMED height / length


CONN = {
    'XT90': ((21.0, 11.0, 46.0), 'Amass XT90 pair, mated (the pack\'s plug + the board pigtail\'s socket)', 'ASSUMED size'),
    'XT30': ((10.2, 5.2, 16.0), 'Amass XT30U', 'ASSUMED size'),
    'XT30(2+2)': ((14.0, 9.0, 20.0), 'Amass XT30(2+2)-F, the CubeMars motor-cable plug (power + CAN_H / CAN_L)', 'motor side: maker STEP; plug VERIFY'),
    'XH2': (xh(2), 'JST XH 2P', 'ASSUMED size'), 'XH4': (xh(4), 'JST XH 4P', 'ASSUMED size'), 'XH6': (xh(6), 'JST XH 6P', 'ASSUMED size'),
    'XH10': (xh(10), 'JST XH 10P', 'ASSUMED size'),
    'USB-A': ((16.0, 8.5, 30.0), 'USB-A plug overmould', 'ASSUMED (layout_study.PLUGS)'),
    'USB-C': ((12.4, 6.5, 21.0), 'USB-C plug overmould', 'D435 bench cable size; others ASSUMED'),
    'micro-B': ((11.0, 7.0, 25.0), 'USB micro-B plug overmould', 'ASSUMED'),
    'HDMI-A': ((21.0, 9.0, 24.0), 'HDMI-A plug on an FPC adapter board', 'ASSUMED'),
    'DC5525': ((10.0, 10.0, 32.0), '5.5 / 2.5 mm DC barrel plug', 'BMAX: VERIFY the barrel size'),
    'Dupont1x8': ((20.3, 2.5, 18.0), 'Dupont 1 x 8 housing on the BNO085 header', 'drawn by upper.py'),
    'spade': ((7.0, 3.0, 8.0), '2.8 mm spade / screw terminals on the 22 mm contact block', 'ASSUMED'),
    'lidar': ((12.0, 6.0, 12.0), "A2M12's connector (on its base)", 'position ASSUMED (VERIFY on the unit)'),
}

# ---- ports (robot frame, mm): p = where the plug meets its device; d = the way the plug points out; dev; conn; drawn = an upper.py plug + lead
# already ends here (p is then that lead's end and the route starts there). New ports on our own boards are ASSUMED placements.
PB = dict(x=(-32.0, 38.0), y=(-174.0, -114.0), z=(220.5, 238.5))      # the distribution board on the power stack's shelf (upper.STACK, Nov 11)
MCU = dict(x=(-13.0, 87.0), y=(-176.0, -111.0), z=(252.5, 277.5))     # Teensy carrier (perfboard, Teensy on headers, JST XH round its edge)
CARRIER_Z = 256.0                                                      # side-entry XH headers on the perfboard (its top at ~254)
MOTOR_Y = (-127.0, -138.0, -149.0, -160.0)                             # the four motor XT30s on the board's -x face, between the stack's
                                                                       # posts at (-42, -117) and (-42, -175)
SLOT_ORDER = ('RR', 'FR', 'RL', 'FL')                                  # which socket each pod's lead takes (nearest the slot first)


def ports():
    P = {}

    def add(name, p, d, dev, conn, drawn=False, note=''):
        P[name] = dict(p=[round(float(v), 1) for v in p], d=list(d), dev=dev, conn=conn, drawn=drawn, note=note)
    # computer (BMAX B8A Pro envelope; port layout ASSUMED, upper.electronics draws the four USB-A and the DC plugs + leads)
    add('pc_usb3_1', (-95.0, 113.0, 242.5), (0, 0, 1), 'computer', 'USB-A', True, 'USB 3 A, the face toward +y (drawn lead end)')
    add('pc_usb3_2', (-113.0, 113.0, 242.5), (0, 0, 1), 'computer', 'USB-A', True, 'USB 3 A, +y face')
    add('pc_usb2_1', (-117.0, -105.0, 233.5), (0, 0, 1), 'computer', 'USB-A', True, 'USB 2 A, -y face')
    add('pc_usb2_2', (-97.0, -105.0, 233.5), (0, 0, 1), 'computer', 'USB-A', True, 'USB 2 A, -y face')
    add('pc_dc', (-175.0, -105.0, 236.5), (0, 0, 1), 'computer', 'DC5525', True, '19 V in, -y face')
    add('pc_usbc', (-135.0, 57.0, 204.5), (0, 1, 0), 'computer', 'USB-C', False, 'USB-C, +y face (new plug)')
    add('pc_hdmi', (-150.0, -55.0, 214.5), (0, -1, 0), 'computer', 'HDMI-A', False, 'HDMI, -y face (new plug)')
    # power stack (Nov 11): our distribution board (fuse, anti-spark + e-stop contactor, current sensor, XT30 / XH outputs) and the three
    # potted converters' output leads. ASSUMED ports
    zm, zj = 226.0, 234.0                                              # the board's XT30 row and its JST row
    add('pb_bat', (32.0, PB['y'][1], 229.0), (0, 1, 0), 'power', 'pigtail', False, "6S in: 12 AWG pigtail to the XT90 that mates the pack's")
    add('bat_deck', (-128.0, -17.0, 101.0), (0, 0, 1), 'battery', 'lead', True, "v15: the board's XT90 lead out of the deck's lead hole "
        '(chassis_mid.LEAD_HOLE), drawn from the XT90 pair under the deck (battery_items)')
    for k, y in enumerate(MOTOR_Y):
        add(f'pb_m{k + 1}', (PB['x'][0], y, zm), (-1, 0, 0), 'power', 'XT30', False, f'motor bus out {k + 1} (fused)')
    add('pb_19v', (-20.0, -110.0, 221.0), (0, 0, 1), 'conv_19v', 'pair18', False, "the 19 V module's output lead, up through the shelf's lead slot")
    add('pb_estop', (PB['x'][0], -128.0, zj), (-1, 0, 0), 'power', 'XH4', False, 'e-stop: NC loop (contactor) + sense')
    add('pb_switch', (PB['x'][0], -142.0, zj), (-1, 0, 0), 'power', 'XH4', False, 'power switch: NO (anti-spark enable) + LED')
    add('pb_5v_face', (PB['x'][0], -156.0, zj), (-1, 0, 0), 'power', 'XH2', False, '5 V out (from the 5 V module): display + audio amp')
    add('pb_arm', (106.0, -144.0, 231.0), (1, 0, 0), 'conv_12v', 'XT30', False, "the 12 V module's output to the arm (the e-stop cuts its input)")
    add('pb_spare_bat', (0.0, PB['y'][0], zm), (0, -1, 0), 'power', 'XT30', False, 'SPARE: switched 6S (future payload)')
    add('pb_spare_5v', (20.0, PB['y'][0], zj), (0, -1, 0), 'power', 'XH2', False, 'SPARE: 5 V')
    # Teensy carrier (perfboard + JST XH, Fernando Nov 11): CAN chain, IMU, spares; the Teensy's own micro-B is drawn by upper.py
    for k, x in enumerate((-2.0, 13.0, 28.0, 43.0)):
        add(f'mcu_can{k + 1}', (x, MCU['y'][0], CARRIER_Z), (0, -1, 0), 'mcu', 'XH4', False, f'CAN1 chain, motor {k + 1}: out pair + return pair')
    add('mcu_usb', (-60.0, -163.5, 298.5), (0, 0, 1), 'mcu', 'micro-B', True, "the Teensy 4.1's micro-B (drawn lead end)")
    add('mcu_imu', (30.0, MCU['y'][1], CARRIER_Z), (0, 1, 0), 'mcu', 'XH10', False, 'BNO085 (SPI)')
    add('mcu_spare_can3', (MCU['x'][1], -128.0, CARRIER_Z), (1, 0, 0), 'mcu', 'XH4', False, 'SPARE: CAN3 payload connector (firmware: reserved)')
    add('mcu_spare_uart', (MCU['x'][1], -142.0, CARRIER_Z), (1, 0, 0), 'mcu', 'XH4', False, 'SPARE: a UART (+5 V)')
    add('mcu_spare_i2c', (MCU['x'][1], -156.0, CARRIER_Z), (1, 0, 0), 'mcu', 'XH4', False, 'SPARE: I2C (+3V3)')
    add('mcu_spare_io', (60.0, MCU['y'][1], CARRIER_Z), (0, 1, 0), 'mcu', 'XH6', False, 'SPARE: GPIO / analog (bumpers, trigger, sync)')
    # devices
    add('imu', (25.6, -40.0, 211.0), (0, 0, 1), 'imu', 'Dupont1x8', True, 'BNO085 header housing, lead up (drawn)')
    add('d435', (59.8, -60.8, 197.6), (-1, 0, 0), 'd435', 'USB-C', True, "the D435's USB-C lead end at the riser's rear (d435_riser.py)")
    add('sa_usb', (41.0, 75.5, 243.0), (0, 0, 1), 'servo_adapter', 'USB-C', True, 'servo bus adapter USB-C (drawn lead end)')
    add('sa_dc', (41.5, 22.5, 214.0), (0, 1, 0), 'servo_adapter', 'DC5525', False, 'servo bus adapter DC in (ASSUMED on the +y end, over its USB-C)')
    add('lidar', (120.0 - 38.5 * 0.7071, -38.5 * 0.7071, 136.0), (-0.7071, -0.7071, 0), 'lidar', 'lidar', False, "A2M12 base, toward the right rear (its lead's exit: VERIFY)")
    add('lidar_ad_in', (-80.0, -146.0, 189.0), (0, -1, 0), 'lidar_adapter', 'lidar', False, 'A2M12 USB adapter board, lidar in')
    add('lidar_ad_usb', (-80.0, -124.0, 189.0), (0, 1, 0), 'lidar_adapter', 'micro-B', False, 'A2M12 USB adapter board, USB out')
    add('estop', (-183.0, 76.0, 249.0), (0, 0, -1), 'estop', 'spade', False, '22 mm e-stop contact block (NC + NC), lead down through the deck')
    add('switch', (-183.0, -76.0, 249.0), (0, 0, -1), 'switch', 'spade', False, '22 mm latching switch (NO + LED), lead down through the deck')
    add('disp_hdmi', (-154.8, -19.0, 322.0), (0, 0, -1), 'display', 'HDMI-A', False, '5 in panel HDMI, at its bottom edge (ASSUMED)')
    add('disp_pwr', (-154.8, 20.0, 322.0), (0, 0, -1), 'display', 'micro-B', False, '5 in panel 5 V (micro-USB power only, ASSUMED)')
    add('mic_usb', (-163.0, -64.0, 410.0), (0, -1, 0), 'mic_array', 'micro-B', False, 'USB mic array (ASSUMED micro-B at its -y end)')
    for name in ('FL', 'RL', 'RR', 'FR'):
        add(f'motor_{name}', (0, 0, 0), (0, 0, 0), f'pod {name}', 'XT30(2+2)', True, 'the CAD\'s motor bundle from its lead stubs to its tie on the motor plate (motor_leg_pod), nominal (v15)')
    return P


# ---- new purchased items the harness adds (envelopes, ASSUMED): plugs on the ports upper.py does not draw, the XT90 pair, the lidar's adapter --
def obox(p, d, L, w, h):
    """a box from p along unit d for L; w across (horizontal; y when d is vertical), h the third axis"""
    import pod as Pd
    import wheel_motor as W
    d = np.asarray(d, float) / np.linalg.norm(d)
    a = np.cross((0, 0, 1.0), d) if abs(d[2]) < 0.9 else np.array([0, 1.0, 0])
    a = a / np.linalg.norm(a)
    u = np.cross(d, a)
    M = np.zeros((3, 4))
    M[:, 0], M[:, 1], M[:, 2], M[:, 3] = d, a, u, p
    return Pd.box(0, L, -w / 2, w / 2, -h / 2, h / 2).moved(W.loc(M))


def corner_radii(pts, R):
    """the radius each corner of a polyline can take: R, unless the two corners sharing a leg need more than its length (then both are
    scaled down together; an end leg serves only its one corner). Returns [(index, turn angle, radius)]"""
    pts = [np.asarray(p, float) for p in pts]
    n = len(pts)
    th = [0.0] * n
    for i in range(1, n - 1):
        u, w = pts[i] - pts[i - 1], pts[i + 1] - pts[i]
        lu, lw = np.linalg.norm(u), np.linalg.norm(w)
        if lu > 1e-6 and lw > 1e-6:
            th[i] = math.acos(max(-1.0, min(1.0, float((u / lu) @ (w / lw)))))
    t = [R * math.tan(a / 2) if a >= math.radians(2) else 0.0 for a in th]
    for _ in range(3):
        for k in range(n - 1):                                           # leg k joins corners k and k + 1
            leg = float(np.linalg.norm(pts[k + 1] - pts[k])) * 0.98
            need = t[k] + t[k + 1]
            if need > leg > 0:
                t[k] *= leg / need
                t[k + 1] *= leg / need
    return [(i, th[i], t[i] / math.tan(th[i] / 2)) for i in range(1, n - 1) if th[i] >= math.radians(2)]


def fillet_path(pts, R, step_deg=15.0):
    """round each corner of a 3D polyline with an arc (radius from corner_radii: R where the legs allow)"""
    pts = [np.asarray(p, float) for p in pts]
    rad = {i: (a, r) for i, a, r in corner_radii(pts, R)}
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        if i not in rad:
            out.append(pts[i])
            continue
        th, r = rad[i]
        a, c, e = pts[i - 1], pts[i], pts[i + 1]
        u, w = (c - a) / np.linalg.norm(c - a), (e - c) / np.linalg.norm(e - c)
        t = r * math.tan(th / 2)
        p0, p1 = c - u * t, c + w * t
        bis = (w - u) / np.linalg.norm(w - u)
        ctr = c + bis * r / math.cos(th / 2)
        n = max(2, int(math.degrees(th) / step_deg) + 1)
        v0, v1 = p0 - ctr, p1 - ctr
        for k in range(n + 1):
            f = k / n
            out.append(ctr + (math.sin(th * (1 - f)) * v0 + math.sin(th * f) * v1) / math.sin(th))
    out.append(pts[-1])
    return np.array(out)


def bends(pts, R):
    """the radius each corner gets in fillet_path"""
    return [r for _, _, r in corner_radii(pts, R)]


def tube(pts, od, bend_r=None):
    """a lead along a polyline: cylinders with spheres at the joints; corners rounded to bend_r first"""
    import build123d as b
    pts = fillet_path(pts, bend_r) if bend_r else np.asarray(pts, float)
    segs = []
    for a, c in zip(pts[:-1], pts[1:]):
        v = c - a
        n = float(np.linalg.norm(v))
        if n > 1e-3:
            segs.append(b.Solid.make_cylinder(od / 2, n, b.Plane(origin=tuple(a), z_dir=tuple(v / n))))
    segs += [b.Solid.make_sphere(od / 2).moved(b.Location(tuple(q))) for q in pts[1:-1]]
    return b.Compound(segs) if len(segs) > 1 else segs[0]                  # a compound: fusing the chain gave shapes OCC could not mesh


LIDAR_AD = dict(x=(-95.0, -65.0), y=(-146.0, -124.0), z=(184.5, 192.5))   # the A2M12's USB adapter board (Slamtec, ~30 x 22: VERIFY), zip-tied
def battery_items():
    """v15 (Nov 14): the pack hangs under the deck's centre (chassis_mid.BATTERY), its XT90 end toward the rear. Its pigtail runs into the
    mated XT90 pair under the deck behind it; the board's 12 AWG lead runs from the pair to the deck's lead hole and up through it (drawn
    here to the hole's top: the port 'bat_deck', from where harness_route routes it to the power board, connection BAT_PB). The balance
    lead's XH7 is tucked against the pack's front end. All ASSUMED (pack lead length and exit, XT90 size)"""
    import pod as Pd
    import chassis_mid as CM
    Bt, x = CM.BATTERY, CM.BATTERY['xt90']
    (l, w, h), (cx, cy) = Bt['size'], Bt['xy']
    zc = (x['z'][0] + x['z'][1]) / 2
    hx, hy, _ = CM.LEAD_HOLE
    out = {}
    od_b, R_b = LEADS['bat'][:2]
    out['lead_battery_xt90'] = tube([(cx - l / 2 - 0.5, cy, zc), (x['x'][1] + 0.5, cy, zc)], od_b, R_b)
    out['plug_xt90_pair'] = Pd.box(*x['x'], *x['y'], *x['z'])
    od_p, R_p = LEADS['pair12'][:2]
    out['lead_pb_xt90'] = tube([(x['x'][0] - 0.5, cy, zc), (hx, cy, zc), (hx, hy, zc), (hx, hy, CM.Z1 + 1.0)], od_p, R_p)
    wx, hh, L_ = xh(7)
    out['plug_battery_balance'] = Pd.box(cx + l / 2 + 0.5, cx + l / 2 + 0.5 + L_, cy - wx / 2, cy + wx / 2, CM.Z0 - h + 4.0, CM.Z0 - h + 4.0 + hh)
    return out


def new_items():
    """{name: solid}: a plug on every port upper.py does not draw (the motors' are the CAD's), the lidar's adapter, the battery items"""
    import pod as Pd
    out = {}
    for k, pt in ports().items():
        if pt['drawn'] or pt['conn'] not in CONN or k.startswith(('motor_', 'estop', 'switch')):
            continue
        (w, h, L_), _, _ = CONN[pt['conn']]
        out[f'plug_{k}'] = obox(pt['p'], pt['d'], L_, w, h)
    out['lidar_adapter'] = Pd.box(*LIDAR_AD['x'], *LIDAR_AD['y'], *LIDAR_AD['z'])
    out.update(battery_items())
    return out


def lidar_supports():
    """the fins' lidar shadows (bearing, shadow_deg): chassis_mid.py's check of the generated fins (validation/chassis_mid/lidar_shadows.json);
    before that has run, v14's (reference/harness: the webs' rule in the scan band is unchanged)"""
    f = ROOT / 'validation' / 'chassis_mid' / 'lidar_shadows.json'
    f = f if f.exists() else ROOT / 'reference' / 'harness' / 'lidar_supports.json'
    return json.loads(f.read_text())['supports']


def motor_leg_pod():
    """v15 (Nov 14, Fernando): the motor bundle's fixed leg on the fork, pod frame, nominal. The XT30 faces up; its two lead stubs run outboard
    over the motor plate's top edge (wheel_motor.build_leads). The bundle starts on them over the plate's middle, runs back along the plate's
    top edge (MOTOR_LEG_ABOVE over it) to the zip tie through the plate's harness hole (pod_rx.HARNESS_HOLE: the tie goes through the hole and
    round the edge and the bundle), and from there the service loop goes up through the deck's opening (harness_route routes it from the tie).
    Returns [P0, ..., T]"""
    import build123d as b
    import pod_p4 as P4
    import pod_rx as R
    import wheel_motor as W
    r = LEADS['motor'][0] / 2
    xm = sum(R.INNER_ARM) / 2
    plate = R.arm_prism(P4.ARM_EXTRA, 0.0, 1.0)                        # the motor plate's outline (its top edge), as a 1 mm slice

    def top(y):
        zs = [z * 0.1 for z in range(0, 600) if plate.is_inside(b.Vector(0.5, y, z * 0.1))]
        return max(zs)
    _, _, pins = W.motor_and_plug()
    stubs = [s.bounding_box() for s in W.build_leads(pins)]
    y0 = sum((s.min.Y + s.max.Y) / 2 for s in stubs) / 2
    z0 = max(max(s.max.Z for s in stubs) - LEADS['motor'][0] / 2, top(y0) + r + MOTOR_LEG_ABOVE)
    yh = R.HARNESS_HOLE[0]
    out = [np.array([xm, y0, z0])]
    for y in list(np.arange(y0 + 5.0, yh - 2.0, 5.0)) + [yh]:            # along the edge every 5 mm (it curves down toward the pads)
        out.append(np.array([xm, float(y), max(top(y + dy) for dy in (-2.5, 0.0, 2.5)) + r + MOTOR_LEG_ABOVE]))
    return out


MOTOR_LEG_ABOVE = 1.0              # the bundle's underside this far over the plate's top edge (drawn; tied down, it lies on it)


MOTOR_LOOP_UP = 18.0               # the service loop drawn straight up from the tie through the deck's opening; the route starts at its top,
                                   # clear of the fork's travel (at bump the plate's edge at the tie rises ~10 mm)


def motor_legs():
    """{pod: [P0, ..., T, S]} robot frame (nominal): each pod's fixed leg to its tie T, then the service loop up to S, where the route starts
    (base.place3d's placement, both hands)"""
    import base as B
    import build123d as b
    leg = motor_leg_pod()
    leg = leg + [leg[-1] + np.array([0.0, 0.0, MOTOR_LOOP_UP])]
    out = {}
    for i, name in enumerate(B.NAMES):
        out[name] = []
        for p in leg:
            v = B.place3d(b.Vertex(*p), i)
            out[name].append(np.array([v.X, v.Y, v.Z]))
    return out


def motor_ends():
    """the motor bundle's route start per pod (robot frame, nominal): the top of its service loop, over the tie at the motor plate's hole"""
    return {k: [round(float(v), 1) for v in leg[-1]] for k, leg in motor_legs().items()}


# ---- connections: one row per lead. a / b: port names; lead: LEADS key; amps: peak current; survive: what it must allow ------------------
SVC_POD = ('pod removal (v15: the pod drops out downward): unplug the XT30 at the motor through the deck\'s opening and cut the tie at the '
           'motor plate; a service loop from that tie up onto the deck takes the fork\'s travel')
SVC_ARM = 'arm module removal: unplug at the module'
SVC_DECK = 'bin deck lift: unplug at the power board, or 150 mm slack'
CONNECTIONS = [
    # power
    dict(id='BAT', a='battery', b='bat_deck', sig='6S pack (18-25.2 V) in: the pack, its pigtail, the XT90 pair under the deck, the board '
         "lead up through the deck's lead hole", conn='XT90 pair', lead='bat', awg=10, amps=30.0, route=False,
         survive='battery removal (v15): robot on its stand, unplug the XT90 under the deck, open the straps, the pack drops out'),
    dict(id='BAT_PB', a='bat_deck', b='pb_bat', sig="6S in: the board's 12 AWG XT90 lead from the deck's lead hole to the power board",
         conn='pigtail', lead='pair12', awg=12, amps=30.0, route=True, survive='stays (unplugged at the XT90 under the deck)'),
    dict(id='BAL', a='battery', b=None, sig='6S balance lead (charging only)', conn='JST XH 7P', lead=None, awg=22, amps=0.0, route=False,
         survive="stays with the pack; its XH7 tucked against the pack's front end"),
] + [dict(id=f'M_{n}', a=f'motor_{n}', b=f'pb_m{SLOT_ORDER.index(n) + 1}', sig='motor bus 6S (after the e-stop contactor) + CAN1 in / out',
          conn='XT30(2+2)-F at the motor / XT30 at the board + XH4 at the carrier', lead='motor', awg=18, amps=7.3, route=True, survive=SVC_POD)
     for n in ('FR', 'FL', 'RR', 'RL')] + [
    dict(id=f'CAN_{n}', a=f'pb_m{SLOT_ORDER.index(n) + 1}', b=f'mcu_can{SLOT_ORDER.index(n) + 1}', sig='CAN1 chain branch of the motor bundle',
         conn='XH4', lead='can', awg=26, amps=0.1, route=True, survive='') for n in ('FR', 'FL', 'RR', 'RL')] + [
    dict(id='PC19', a='pb_19v', b='pc_dc', sig='19 V computer supply (the 19 V module, before the e-stop)', conn="module lead / DC 5.5x2.5",
         lead='pair18', awg=18, amps=4.5, route=True, survive=''),
    dict(id='ARM12', a='pb_arm', b='sa_dc', sig='12 V servo bus power from the stack\'s 12 V module (the e-stop cuts its input)',
         conn='XT30 at the module / DC 5.5x2.1 at the adapter (Waveshare: VERIFY)', lead='pair18', awg=18, amps=6.7, route=True,
         survive=SVC_ARM + ' (the DC plug at the servo adapter)'),
    dict(id='SBUS', a='servo_adapter', b='SO-101 base servo', sig='Feetech TTL half-duplex bus + 12 V', conn='5264 3P (SO-101 kit cable)',
         lead=None, awg=None, amps=6.6, route=False, survive='arm kit: inside the SO-101, routed by its own clips'),
    dict(id='5V', a='pb_5v_face', b='disp_pwr', sig='5 V: display + the speaker amp (branch at the crown)', conn='XH2 / micro-B', lead='pair20',
         awg=20, amps=2.0, route=True, survive=SVC_DECK),
    dict(id='PB-MCU', a='power board top', b='carrier underside', sig='5 V, GND, V_BUS, I_BUS, ESTOP, BUS_EN', conn='XH6 both ends',
         lead=None, awg=24, amps=0.5, route=False, survive='inside the stack (10 mm gap)'),
    # safety
    dict(id='ESTOP', a='estop', b='pb_estop', sig='e-stop: NC in the contactor / anti-spark loop + NC to Teensy pin 35', conn='spades / XH4',
         lead='multi4', awg=22, amps=0.5, route=True, survive=SVC_DECK),
    dict(id='SWITCH', a='switch', b='pb_switch', sig='power switch: NO to the anti-spark enable + LED ring', conn='spades / XH4', lead='multi4',
         awg=22, amps=0.5, route=True, survive=SVC_DECK),
    # data
    dict(id='D435', a='d435', b='pc_usb3_1', sig='USB 3 (5 Gb/s)', conn='USB-C / USB-A', lead='usb3', awg=None, amps=0.7, route=True,
         survive=SVC_ARM + ' (USB-C at the camera; the lead stays on the floor)'),
    dict(id='TEENSY', a='mcu_usb', b='pc_usb2_1', sig='USB 2 (Teensy native CDC)', conn='micro-B / USB-A', lead='usb2', awg=None, amps=0.1,
         route=True, survive=''),
    dict(id='SO101', a='sa_usb', b='pc_usb3_2', sig='USB (servo bus adapter)', conn='USB-C / USB-A', lead='usbc', awg=None, amps=0.1, route=True,
         survive=SVC_ARM + ' (USB-C at the adapter)'),
    dict(id='LIDAR', a='lidar', b='lidar_ad_in', sig="A2M12 UART + 5 V (its own lead)", conn='A2M12 connector both ends', lead='lidar',
         awg=None, amps=0.6, route=True, survive='second floor lift: unplug at the adapter'),
    dict(id='LIDARUSB', a='lidar_ad_usb', b='pc_usb2_2', sig='USB 2 (A2M12 adapter)', conn='micro-B / USB-A', lead='usb2', awg=None,
         amps=0.6, route=True, survive=''),
    dict(id='IMU', a='mcu_imu', b='imu', sig='BNO085 SPI + INT / RST / WAKE / BOOTN, 3V3', conn='XH10 / Dupont 1x8 (+2: VERIFY its header)',
         lead='imu', awg=26, amps=0.05, route=True, survive=''),
    dict(id='HDMI', a='pc_hdmi', b='disp_hdmi', sig='HDMI video to the 5 in face', conn='HDMI-A both ends (FPC cable)', lead='hdmi', awg=None,
         amps=0.0, route=True, survive=SVC_DECK.replace('unplug at the power board', 'unplug at the computer')),
    dict(id='MIC', a='mic_usb', b='pc_usbc', sig='USB audio: 4-mic array in, playback out to the amp (AEC reference)', conn='micro-B / USB-C',
         lead='usb2', awg=None, amps=0.3, route=True, survive=SVC_DECK.replace('unplug at the power board', 'unplug at the computer')),
    dict(id='SPK', a='mic array audio out', b='speaker', sig='3.5 mm line out -> class-D amp (PAM8302 class, ASSUMED) -> 4 ohm 3 W speaker',
         conn='3.5 mm / XH2', lead=None, awg=24, amps=0.9, route=False, survive='on the crown (amp beside the speaker, not drawn)'),
    dict(id='ELRS', a='ELRS receiver', b='mcu', sig='CRSF on Serial2 (pins 7 / 8) + 5 V', conn='soldered on the carrier', lead=None, awg=None,
         amps=0.15, route=False, survive='on the carrier; its antenna up past the carrier\'s edge'),
]


# ---- power budget ---------------------------------------------------------------------------------------------------------------------------
PACK = dict(cells=6, v_full=25.2, v_nom=22.2, v_stop=21.0, v_min=18.0, wh=73.3, usable=0.8)   # design.md 2 (21 V stop provisional); Nov 16: HRB 6S 3300 (was OVONIC 5200, 115 Wh)
EFF = dict(buckboost19=0.92, buck12=0.92, buck5=0.90)                  # converter efficiency (ASSUMED, typical synchronous modules)
# Fernando's picks (Nov 11): potted, waterproof Amazon modules. Values from the listings (and, where the listing gives none, a comparable unit's):
# VERIFY on the units bought. Sizes are the modules' bodies, leads extra.
CONVERTER_PARTS = {
    '19V': dict(listing='amazon.com/dp/B09KY6H4FZ', what='DC-DC 24 V to 19 V 10 A 190 W buck, IP67', vin=(22.0, 40.0), vout=19.0, amps=10.0,
                eff='up to 96 % (listing)', size_mm=(74.0, 74.0, 32.0), size_src='a comparable 24-19 V 10 A potted unit (VERIFY)',
                issue='input floor 22 V: a 6S pack is under 22 V for its last stretch (3.67 V / cell) and sags under it in a burst well before the '
                      '21 V stop, so the computer\'s supply would leave its spec: bench-test the dropout at 20-22 V and 3.4 A, or use a buck-boost'),
    '12V': dict(listing='amazon.com/dp/B0F3HQ6ND8', what='Proshopping DC-DC buck 15-90 V to 12 V 8 A 96 W, waterproof', vin=(15.0, 90.0), vout=12.0,
                amps=8.0, eff='not stated', size_mm=(64.0, 60.0, 22.0), size_src='ASSUMED: the potted 8 A class (no size in the listing)'),
    '5V': dict(listing='amazon.com/dp/B0CM37DX4Q', what='LeMotech DC 12/24 V to 5 V 5 A 25 W, IP68', vin=(8.0, 40.0), vout=5.0, amps=5.0,
               eff='not stated', size_mm=(66.0, 60.0, 22.0), size_src='ASSUMED: comparable 5 A 25 W potted units are 66 x 60 x 22-23'),
}


def ak40_bus_power(torque_nm, wheel_speed_mps, r_wheel=0.070):
    """one AK40-10 from the 6S bus: mechanical + copper (FOC: 0.75 I^2 R_ll) / driver 0.95; motors.csv: 4.1 N m at 7.3 A peak, R_ll 0.978"""
    i = torque_nm / 4.1 * 7.3
    p_cu = 0.75 * i * i * 0.978
    p_mech = torque_nm * wheel_speed_mps / r_wheel / 0.9                 # gear 0.9 (est)
    return (p_mech + p_cu) / 0.95, i


def power_budget():
    m, g, mu = 9.63, 9.81, 1.0                                          # design.md 2: arm model 9.63 kg (est); mu ~1 on the floor (est)
    trac_t = mu * m * g / 4 * 0.070                                      # traction-limited torque per wheel
    p_trac, i_trac = ak40_bus_power(trac_t, 2.0)
    p_peak, _ = ak40_bus_power(4.1, 2.0)
    loads = [   # (rail, load, typical W, peak W, provenance)
        ('motor', 'drive, 4 x AK40-10, cruising', 70.0, None, 'sim: cruise 68-76 W (design.md 3)'),
        ('motor', 'drive, accelerating at the traction limit at 2 m/s', None, round(4 * p_trac), f'est: mu 1, 9.63 kg, {trac_t:.2f} N m / wheel ({i_trac:.1f} A)'),
        ('motor', 'drive, all four at peak torque at 2 m/s (not reachable on a floor)', None, round(4 * p_peak), 'ceiling: 4.1 N m, 7.3 A each (motors.csv)'),
        ('12V', 'SO-101, 6 x STS3215 12 V: holding / moving', 12.0, 80.0, 'est: 2.7 A stall each (servos.csv); peak = two joints at stall + four at 0.3 A'),
        ('19V', 'computer (BMAX B8A Pro 8745HS) incl. its USB loads (D435 <= 3.5 W, A2M12 ~2.5 W, mic, adapter)', 30.0, 65.0,
         'est: typical robot load; peak = the maker\'s 19 V / 3.42 A adapter (VERIFY)'),
        ('5V', 'Teensy 4.1 + 2 CAN transceivers + BNO085 + ELRS + buzzer / LED', 1.5, 2.5, 'est (datasheet class values)'),
        ('5V', '5 in display (backlight)', 3.5, 5.0, 'est (VERIFY on the panel)'),
        ('5V', 'speaker amp, 3 W into 4 ohm', 0.3, 3.6, 'est'),
        ('bat', 'power board itself: contactor coil, current sensor, LEDs', 1.0, 1.5, 'est'),
    ]
    rails = {}
    for rail, what, typ, pk, _ in loads:
        r = rails.setdefault(rail, dict(typ=0.0, peak=0.0, ceiling=0.0))
        r['typ'] += typ or 0.0
        if 'not reachable' in what:
            r['ceiling'] += pk
        else:
            r['peak'] += pk if pk is not None else (typ or 0.0)
    rails['motor']['peak'] = round(4 * p_trac)                         # the traction-limited burst, not cruise + burst
    conv = {}
    for rail, v, eff, key in (('19V', 19.0, EFF['buckboost19'], '19 V buck-boost'), ('12V', 12.0, EFF['buck12'], '12 V buck (arm)'), ('5V', 5.0, EFF['buck5'], '5 V buck')):
        r = rails[rail]
        conv[key] = dict(out_typ_A=round(r['typ'] / v, 2), out_peak_A=round(r['peak'] / v, 2), in_peak_W=round(r['peak'] / eff, 1),
                         in_peak_A_at_v_stop=round(r['peak'] / eff / PACK['v_stop'], 2))
    rec = {'19 V buck-boost': '19 V, >= 5 A (95-100 W), input 16-26 V: a buck alone drops out near the 21 V stop under load; 3.4 A peak out',
           '12 V buck (arm)': '12 V, 8 A, in the power stack (the riser buck is gone); 6.7 A peak out',
           '5 V buck': '5 V, 5 A (25 W): 2.2 A peak out, room for a powered USB hub or a head camera later'}
    parts = CONVERTER_PARTS                                                # Fernando's picks (Nov 11), listing values: VERIFY on the units
    typ_bat = rails['motor']['typ'] + rails['19V']['typ'] / EFF['buckboost19'] + rails['12V']['typ'] / EFF['buck12'] + rails['5V']['typ'] / EFF['buck5'] + rails['bat']['typ']
    pk_bat = rails['motor']['peak'] + rails['19V']['peak'] / EFF['buckboost19'] + rails['12V']['peak'] / EFF['buck12'] + rails['5V']['peak'] / EFF['buck5'] + rails['bat']['peak']
    ceil_bat = pk_bat - rails['motor']['peak'] + rails['motor']['ceiling']
    bat = dict(typ_W=round(typ_bat), typ_A_at_nom=round(typ_bat / PACK['v_nom'], 1), peak_W=round(pk_bat), peak_A_at_stop=round(pk_bat / PACK['v_stop'], 1),
               ceiling_W=round(ceil_bat), ceiling_A_at_stop=round(ceil_bat / PACK['v_stop'], 1),
               runtime_min_typ=round(PACK['wh'] * PACK['usable'] / typ_bat * 60))
    fuses = {'main (battery +, at the board input)': '30 A slow (ATO / mini blade): above the 20 A burst, below the 12 AWG pigtail',
             'motor bus (after the contactor)': '20 A, or 4 x 7.5 A, one per motor XT30 (a shorted lead then stops one wheel)',
             '19 V module input': '7.5 A', '12 V module input (after the contactor)': '7.5 A', '12 V out to the arm': '10 A',
             '5 V module input': '3 A', 'spare XT30 out': '10 A'}
    wires = []
    for c in CONNECTIONS:
        if c.get('awg') and c['amps']:
            od, ohm, amp = WIRE[c['awg']]
            wires.append(dict(id=c['id'], awg=c['awg'], peak_A=c['amps'], rated_A=amp, ok=amp >= c['amps']))
    return dict(pack=PACK, loads=[dict(rail=a, load=b_, typ_W=c, peak_W=d, source=e) for a, b_, c, d, e in loads], rails=rails, converters=conv,
                converter_choice=rec, converter_parts=parts, battery=bat, fuses=fuses, wires=wires,
                computer_direct_from_battery=('no: the BMAX takes 19 V (maker: 19 V / 3.42 A); the pack is 25.2 V full, 22.2 nominal, 21 V at the stop. '
                                              'Feed it from the 19 V buck-boost on the battery side of the e-stop (an e-stop must not reboot the '
                                              'computer). Only a PC rated for a wide input (e.g. 12-28 V) could take the pack directly.'),
                notes=['e-stop cuts the motor bus and the arm (contactor / anti-spark); the computer, MCU, lidar and face stay up',
                       'regen: the AK40-10s brake into the bus; with the contactor open at speed the back-EMF at the 44.9 rad/s cap is ~25 V '
                       '(10:1, KV 170): a 30 V TVS on the motor bus (VERIFY)',
                       'all values est unless sourced; nothing measured'])


# ---- outputs -------------------------------------------------------------------------------------------------------------------------------
def md_table(rows, cols):
    out = ['| ' + ' | '.join(h for h, _ in cols) + ' |', '| ' + ' | '.join('---' for _ in cols) + ' |']
    for r in rows:
        out.append('| ' + ' | '.join(str(f(r)) for _, f in cols) + ' |')
    return '\n'.join(out)


def inventory(routes=None):
    P = ports()
    rows = []
    for c in CONNECTIONS:
        od, bend, what = LEADS[c['lead']] if c.get('lead') else (None, None, '')
        rt = (routes or {}).get(c['id'], {})
        rows.append(dict(c, od=od, bend_r=bend, what=what, length_mm=rt.get('length_mm'), a_port=P.get(c['a']), b_port=P.get(c['b'])))
    return rows


# ---- the drawn harness ------------------------------------------------------------------------------------------------------------------
def stub_len(port, lead):
    """straight lead behind a new plug before its first bend (the route starts there): one bend radius, at least 3 mm, so the first
    bend can take its full radius (bends() reports any that cannot)"""
    return max(3.0, LEADS[lead][1]) if lead else 3.0


# where each lead crosses the floor in the right slot: u along the side fin's ray (dims.HARNESS_SLOT_U 245..272, ends at 238.5 / 278.5),
# one behind another on the ray so each stays inside the fin's shadow (half width 7.7 mm at u 238.5 against a Ø8 bundle's 4)
SLOT_U_LEAD = {'M_RR': 243.0, 'M_FR': 251.5, 'M_RL': 260.0, 'M_FL': 268.5, 'LIDAR': 275.0}
SLOT_U_LEAD_L = {'BAT_PB': 258.5}    # v15: the battery's lead (from the deck's lead hole) up the spare LEFT slot: the right one is full
SLOT_Z = (138.0, 192.0)              # the lead runs straight up the slot's axis between these heights. v15 (Nov 14): was 146 / 188, but a
                                     # bundle's R16 bends at the run's ends then reached into the scan band (z 163 +- 3 + r) off the ray


def slot_of(rid):
    """(u, side) of a lead pinned to a slot (side -1: the right slot, +1: the left), or None"""
    if rid in SLOT_U_LEAD:
        return SLOT_U_LEAD[rid], -1
    if rid in SLOT_U_LEAD_L:
        return SLOT_U_LEAD_L[rid], 1
    return None


def slot_xy(u, side):
    import fins as F
    import upper as U
    return F.LIDAR_X + U.SIDE_RAY[0] * u, side * U.SIDE_RAY[1] * u


def snap_to_slot(rid, pts):
    """replace the route's cells in and near the slot by a vertical run on the slot's ray at this lead's u"""
    import upper as U
    us = slot_of(rid)
    if us is None:
        return pts
    x, y = slot_xy(*us)
    g = U.SLOTS['R' if us[1] < 0 else 'L'].buffer(14.0)
    from shapely.geometry import Point
    idx = [i for i, q in enumerate(pts) if SLOT_Z[0] - 8.0 <= q[2] <= SLOT_Z[1] + 8.0 and g.contains(Point(q[0], q[1]))]
    if not idx:
        return pts
    i0, i1 = min(idx), max(idx)
    up = pts[i0][2] < pts[i1][2]
    mid = [np.array([x, y, SLOT_Z[0] if up else SLOT_Z[1]]), np.array([x, y, SLOT_Z[1] if up else SLOT_Z[0]])]
    return pts[:i0] + mid + pts[i1 + 1:]


def lead_polyline(run, P, rid=None):
    """the lead's centreline: plug rear (or the drawn lead's end / the motor lead's end) -> route -> the far plug's rear"""
    pts = [np.array(q, float) for q in run['waypoints']]                # harness_route already runs each slot lead up its ray
    pinned = any(np.allclose(a[:2], c[:2], atol=0.05) and {round(a[2], 1), round(c[2], 1)} == {SLOT_Z[0], SLOT_Z[1]} for a, c in zip(pts[:-1], pts[1:]))
    if not pinned:
        pts = snap_to_slot(rid, pts)
    ends = []
    for key, s_ in (('a', pts[0]), ('b', pts[-1])):
        name = run[key]
        if name.startswith('motor_'):                                  # v15: the fixed leg on the fork, then the route from its tie
            leg = motor_legs()[name.split('_')[1]][:-1]
            pts = (leg + pts) if key == 'a' else (pts + leg[::-1])
            ends.append(None)
            continue
        pt = P[name]
        p, d = np.array(pt['p'], float), np.array(pt['d'], float)
        rear = p if pt['drawn'] else p + d * CONN[pt['conn']][0][2] if pt['conn'] in CONN else p
        ends.append(rear)
    out = ([ends[0]] if ends[0] is not None else []) + pts + ([ends[1]] if ends[1] is not None else [])
    keep = [out[0]]
    for q in out[1:]:
        if np.linalg.norm(q - keep[-1]) > 0.3:
            keep.append(q)
    return np.array(keep)


def build():
    """{name: solid} of the leads (tubes along the routes) and the new purchased items, plus each lead's bend record"""
    rf = VAL / 'routes.json'
    runs = json.loads(rf.read_text())['runs']
    P = ports()
    items = new_items()
    info, paths = {}, {}
    for k, r in runs.items():
        if not r.get('reachable'):
            continue
        od, R, _ = LEADS[r['lead']]
        pl = lead_polyline(r, P, k)
        items[f'lead_{k}'] = tube(pl, od, R)
        paths[k] = fillet_path(pl, R)
        got = bends(pl, R)
        info[k] = dict(od=od, bend_r_allowed=R, bends=len(got), tightest_bend_r=round(min(got), 1) if got else None,
                       bends_under_allowed=sum(1 for g in got if g < R - 0.05), length_drawn_mm=round(float(np.sum(np.linalg.norm(np.diff(fillet_path(pl, R), axis=0), axis=1)))))
    return items, info, paths


# ---- zip-tie anchors through the second floor ----------------------------------------------------------------------------------------------
def tie_candidates(runs, pitch=30.0):
    """(x, y, deg, why): pairs beside each slot (cable along the fin's ray, both sides), then a 30 mm grid over the whole floor: "zip-tie
    holes all around" (Fernando, Nov 11). A grid pair near a routed lead (<= 25 mm in plan, any height on the floor) takes that lead's
    direction, else it alternates 0 / 90 deg; tie_filter keeps those on plain plate"""
    import upper as U
    segs = []
    for k, r in runs.items():
        if r.get('reachable'):
            wp = np.array(r['waypoints'])
            for a, c in zip(wp[:-1], wp[1:]):
                if max(a[2], c[2]) > U.Z_TOP and min(a[2], c[2]) < U.LID_Z[0] and np.linalg.norm((c - a)[:2]) > 1.0:
                    segs.append((a[:2], c[:2], k))
    cand = []
    for side in (-1, 1):
        (x0, y0), (x1, y1) = U.slot_centreline(side)
        ray = math.degrees(math.atan2(y1 - y0, x1 - x0))
        u = np.array([x1 - x0, y1 - y0]) / math.hypot(x1 - x0, y1 - y0)
        n = np.array([-u[1], u[0]])
        for t in (0.15, 0.85):
            for off in (-17.0, 17.0):
                q = np.array([x0, y0]) + (np.array([x1, y1]) - np.array([x0, y0])) * t + n * off
                cand.append((round(float(q[0]), 1), round(float(q[1]), 1), round(ray, 0), f'slot_{"L" if side > 0 else "R"}'))
    k = 0
    for x in np.arange(-210.0, 211.0, pitch):
        for y in np.arange(-210.0, 211.0, pitch):
            if math.hypot(x, y) > U.R_FLOOR - 14.0:
                continue
            best, why = None, 'spare'
            for a, c, rid in segs:
                v = c - a
                t = float(np.clip(((np.array([x, y]) - a) @ v) / (v @ v), 0, 1))
                d = float(np.linalg.norm(a + v * t - np.array([x, y])))
                if d <= 25.0 and (best is None or d < best[0]):
                    best = (d, math.degrees(math.atan2(v[1], v[0])))
                    why = rid
            deg = round(best[1], 0) if best else (0.0 if k % 2 else 90.0)
            k += 1
            cand.append((round(float(x), 1), round(float(y), 1), deg, why))
    return cand


def tie_filter(cand, floor, others, lower):
    """keep a candidate when its tie's path is plain plate: the footprint (+TIE_KEEP) is all inside one quadrant at z 177..181 (no hole, notch
    or slot), nothing is under the plate there (no lip, rib, boss, fin flange: z 175.8) and nothing on it (z 182.2..190: devices, brackets,
    posts, plugs); and no two pairs closer than 18 mm"""
    import build123d as b
    from dims import TIE_KEEP
    import upper as U
    keep = []
    boxes = {id(s): s.bounding_box() for pool in (floor, others, lower) for s in pool.values()}

    def inside(s, v):
        bb = boxes[id(s)]
        return bb.min.X <= v.X <= bb.max.X and bb.min.Y <= v.Y <= bb.max.Y and bb.min.Z <= v.Z <= bb.max.Z and s.is_inside(v)
    for x, y, deg, why in cand:
        g = U.tie_footprint(x, y, deg, TIE_KEEP)
        if any(math.hypot(x - q[0], y - q[1]) < 18.0 for q in keep):
            continue
        if not g.within(__import__('shapely').geometry.Point(0, 0).buffer(U.R_FLOOR - U.LIP_W - 1.0, 128)):
            continue
        pts = [(px, py) for px, py in np.array(g.exterior.coords)[:-1]] + [(x, y)]
        pts += [tuple(np.array(g.exterior.coords[i]) * 0.5 + np.array([x, y]) * 0.5) for i in range(4)]
        ok = True
        for px, py in pts:
            v_in = b.Vector(px, py, (U.Z_UNDER + U.Z_TOP) / 2)
            if not any(inside(s, v_in) for s in floor.values()):
                ok = False
                break
            for z, pool in ((U.Z_UNDER - 0.7, lower), (U.Z_UNDER - 0.7, floor), (U.Z_TOP + 0.7, others), (U.Z_TOP + 6.0, others)):
                v_ = b.Vector(px, py, z)
                if any(inside(s, v_) for s in pool.values()):
                    ok = False
                    break
            if not ok:
                break
        if ok:
            keep.append((x, y, deg, why))
    return keep


def ties_stage():
    """place the floor's zip-tie anchors (validation/harness/ties.json); paste the list into upper.TIES_FLOOR"""
    import fins as F
    import upper as U
    runs = json.loads((VAL / 'routes.json').read_text())['runs']
    printed, bought, vendor, hw, _ = U.build()
    floor = {k: v for k, v in printed.items() if k.startswith('floor2_')}
    others = {k: v for k, v in dict(printed, **bought).items() if not k.startswith('floor2_')}
    others.update(new_items())
    import world as WD
    parts, chw, _ = WD.base()
    keep = tie_filter(tie_candidates(runs), floor, others, dict(parts, **chw))
    (VAL / 'ties.json').write_text(json.dumps(dict(ties=keep, n=len(keep)), indent=1), encoding='utf-8')
    print(len(keep), 'tie pairs')
    print('TIES_FLOOR = (' + ',\n              '.join(f'({x}, {y}, {d})' for x, y, d, _ in keep) + ')')
    return keep


# ---- M3 pilots on a square grid through the second floor (Nov 11, Fernando: "to attach things later") ---------------------------------------
def grid_stage():
    """place the floor's grid pilots (validation/harness/grid.json); paste the list into upper.GRID_PILOTS. A grid point (dims.GRID_PITCH, shifted
    GRID_OFFSET off the tie grid) is kept when:
     * the plate round it is plain: its boss's circle + GRID_KEEP lies inside one quadrant's plate at mid-thickness (no edge, seam, notch, slot,
       tie slot, hole or other pilot);
     * its boss fits under the plate: nothing within GRID_KEEP of the boss, from the boss's depth to the plate's underside (lips, ribs, seam lips,
       other bosses, fin flanges, the chassis, the lidar's keep-out) and no lead (centreline closer than boss + GRID_KEEP + the lead's radius);
     * nothing stands on it: no part, plug or lead within GRID_TOP_R, up to GRID_TOP_H above the floor;
     * no tie pair's path (footprint + TIE_KEEP) within the boss's circle."""
    from shapely import union_all
    from shapely.geometry import Point, Polygon
    import fins as F
    import upper as U
    from dims import GRID_PITCH, GRID_OFFSET, GRID_KEEP, GRID_TOP_R, GRID_TOP_H, TIE_KEEP
    U.GRID_PILOTS = ()                                                     # place from scratch
    printed, bought, vendor, hw, _ = U.build()
    floor = {k: v for k, v in printed.items() if k.startswith('floor2_')}
    others = {k: v for k, v in dict(printed, **bought, **U.load_vendor(vendor)).items() if not k.startswith('floor2_')}
    items, _, paths = build()
    others.update({k: v for k, v in items.items() if not k.startswith('lead_')})
    import world as WD
    parts, chw, _ = WD.base()
    lower = dict(parts, **chw)
    rb = U.BOSS_D / 2
    zm = (U.Z_UNDER + U.Z_TOP) / 2
    z_boss = U.Z_UNDER - U.BOSS_H - GRID_KEEP

    def plan(pool, z0, z1):
        gs = [B_proj(s, z0, z1) for s in pool.values()]
        return union_all([g for g in gs if g is not None] or [Polygon()])
    plates = {k: B_proj(s, zm - 0.2, zm + 0.2) for k, s in floor.items()}
    under = union_all([plan(floor, z_boss, U.Z_UNDER - 0.05), plan(lower, z_boss, U.Z_UNDER - 0.05), U.lidar_keepout()])
    on_top = plan(others, U.Z_TOP + 0.05, U.Z_TOP + GRID_TOP_H)
    ties = union_all([U.tie_footprint(x, y, d, TIE_KEEP) for x, y, d in U.TIES_FLOOR])
    lead_pts = []                                                          # (x, y, r) of every lead sample near the floor's height
    runs = json.loads((VAL / 'routes.json').read_text())['runs']
    for k, pth in paths.items():
        r = LEADS[runs[k]['lead']][0] / 2
        for q in samples(np.asarray(pth), 1.0)[0]:
            if z_boss - r <= q[2] <= U.Z_TOP + GRID_TOP_H + r:
                lead_pts.append((q[0], q[1], r))
    lead_pts = np.array(lead_pts) if lead_pts else np.zeros((0, 3))
    keep, why = [], {}
    n0 = math.ceil(U.R_FLOOR / GRID_PITCH)
    span = GRID_OFFSET + GRID_PITCH * np.arange(-n0, n0)                   # 15 + 30 k: half a pitch off the tie grid (multiples of 30)
    for x in span:
        for y in span:
            x, y = round(float(x), 1), round(float(y), 1)
            if math.hypot(x, y) > U.R_FLOOR - U.LIP_W - rb:
                continue
            c = Point(x, y)
            boss = c.buffer(rb + GRID_KEEP, 32)
            if not any(g is not None and boss.within(g) for g in plates.values()):
                why[f'{x}, {y}'] = 'plate'
            elif boss.intersects(under):
                why[f'{x}, {y}'] = 'under'
            elif c.buffer(GRID_TOP_R, 32).intersects(on_top):
                why[f'{x}, {y}'] = 'on top'
            elif boss.intersects(ties):
                why[f'{x}, {y}'] = 'tie'
            elif len(lead_pts) and (np.hypot(lead_pts[:, 0] - x, lead_pts[:, 1] - y) - lead_pts[:, 2] < rb + GRID_KEEP).any():
                why[f'{x}, {y}'] = 'lead'
            else:
                keep.append((x, y))
    counts = {k: sum(1 for v in why.values() if v == k) for k in ('plate', 'under', 'on top', 'tie', 'lead')}
    (VAL / 'grid.json').write_text(json.dumps(dict(pilots=keep, n=len(keep), rejected=counts, rejected_at=why), indent=1), encoding='utf-8')
    print(len(keep), 'grid pilots; rejected', counts)
    print('GRID_PILOTS = (' + ',\n               '.join(f'({x}, {y})' for x, y in keep) + ')')
    return keep


def B_proj(s, z0, z1):
    """exact plan of a solid between z0 and z1 (robot frame), or None"""
    import base as B_
    return B_.proj(s, True, z0, z1, tol=0.2)


# ---- checks -------------------------------------------------------------------------------------------------------------------------------
def surface_points(shapes, density=1.0, tol=0.2):
    """surface samples of each shape (mesh vertices + ~density per mm2) and the shape's index per point"""
    rng = np.random.default_rng(5)
    P, I = [], []
    for n, s in enumerate(shapes):
        try:
            v, t = s.tessellate(tol, 0.3)
        except Exception:                                                  # noqa: BLE001
            continue
        V = np.array([[q.X, q.Y, q.Z] for q in v])
        T = np.array(t, dtype=int).reshape(-1, 3)
        if not len(V) or not len(T):
            continue
        A, Bv, Cv = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(Bv - A, Cv - A), axis=1)
        idx = np.repeat(np.arange(len(T)), np.ceil(area * density).astype(int))
        r1, r2 = rng.random(len(idx)), rng.random(len(idx))
        sq = np.sqrt(r1)
        pts = np.vstack([V, (1 - sq)[:, None] * A[idx] + (sq * (1 - r2))[:, None] * Bv[idx] + (sq * r2)[:, None] * Cv[idx]])
        P.append(pts)
        I.append(np.full(len(pts), n))
    return np.vstack(P), np.concatenate(I)


def samples(path, step=0.5):
    """points every `step` along a polyline and their arc length"""
    out, sl = [path[0]], [0.0]
    for a, c in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(c - a) / step)))
        for k in range(1, n + 1):
            out.append(a + (c - a) * k / n)
            sl.append(sl[-1] + np.linalg.norm(c - a) / n)
    return np.array(out), np.array(sl)


def lead_r(rid):
    return LEADS[next(c for c in CONNECTIONS if c['id'] == rid)['lead']][0] / 2


DRAWN_PLUG = {'pc_usb3_1': 'plug_computer_0', 'pc_usb3_2': 'plug_computer_1', 'pc_usb2_1': 'plug_computer_2', 'pc_usb2_2': 'plug_computer_3',
              'pc_dc': 'plug_computer_4', 'mcu_usb': 'plug_mcu_0', 'imu': 'plug_imu', 'd435': 'plug_d435', 'sa_usb': 'plug_servo_adapter_0',
              'bat_deck': 'lead_pb_xt90'}   # v15: the routed BAT_PB continues the drawn lead out of the deck's hole


def check(items, info, paths):
    """the drawn harness against the robot. Leads are measured from their centrelines (samples every 0.5 mm) to every part's surface
    samples (~1 per mm2, +-0.3 mm) minus their radius, with a point-in-solid test every 3 mm (a lead inside a thick part); the new plugs and
    boards by exact overlap. Moving parts: the pods in droop / nominal / bump (fork, wheel, motor, rollers, plungers) and the arm's sweep
    (harness_route's grid). Scan band: each lead's width as the lidar sees it where its centreline is within 3 mm + r of the plane"""
    import build123d as b
    import fins as F
    import harness_route as HR
    import upper as U
    import wheel_motor as W
    import d435_riser as D
    from scipy.spatial import cKDTree
    rep = dict(leads=info)
    static, pods = HR.build_world(fov=False)                            # v15: the mid deck, the mid-deck pods, the upper robot (one world)
    new = set(new_items())
    world = {k: v for k, v in static.items() if k not in new}
    printed, bought, vendor, uhw, _ = U.build()
    world.update(U.load_vendor({k: v for k, v in vendor.items() if k.startswith('arm_')}))   # the arm at stow (its sweep: below)
    fov = D.fov_envelopes(500.0)
    plugs = {k: v for k, v in items.items() if not k.startswith('lead_') or k in ('lead_battery_xt90', 'lead_pb_xt90')}
    world_h = dict(world, **fov, **plugs)                                  # the leads also keep off the new plugs and boards and the D435's fields
    names = list(world_h)
    shapes = [world_h[k] for k in names]
    bbs = [s.bounding_box() for s in shapes]
    P, I = surface_points(shapes)
    tree = cKDTree(P)
    print('world samples', len(P), flush=True)
    ends = motor_ends()
    leg_len = {k: float(sum(np.linalg.norm(np.diff(np.array(v), axis=0), axis=1))) for k, v in motor_legs().items()}
    static, inside, fov_hits = {}, {}, {}
    for rid, path in paths.items():
        c = next(c for c in CONNECTIONS if c['id'] == rid)
        r = lead_r(rid)
        own = {f'plug_{c["a"]}', f'plug_{c["b"]}'} | {DRAWN_PLUG[q] for q in (c['a'], c['b']) if q in DRAWN_PLUG}
        if rid.startswith('CAN_'):
            own |= {f'plug_{c["a"]}'}
        q, sl = samples(path)
        keep = (sl > r + 1.5) & (sl < sl[-1] - r - 1.5)                    # its two ends sit against their plugs
        q = q[keep]
        d, ix = tree.query(q, k=12)
        best = (1e9, None)
        for col in range(d.shape[1]):
            for dd, j in zip(d[:, col], ix[:, col]):
                lb = names[I[j]]
                if lb in own:
                    continue
                cl = float(dd) - r
                if lb.startswith('env_d435_fov'):
                    if cl < 0:
                        fov_hits[rid] = lb
                    continue
                if cl < best[0]:
                    best = (cl, lb)
        static[rid] = dict(min_mm=round(best[0], 2), against=best[1])
        hits = []
        for p_ in q[::6]:
            v = b.Vector(*p_)
            for nm, s_, bb in zip(names, shapes, bbs):
                if nm in own or not (bb.min.X <= p_[0] <= bb.max.X and bb.min.Y <= p_[1] <= bb.max.Y and bb.min.Z <= p_[2] <= bb.max.Z):
                    continue
                if s_.is_inside(v):
                    hits.append(nm)
        if hits:
            inside[rid] = sorted(set(hits))
    rep['leads_static_clearance_mm'] = dict(per_lead=static, under_0=[k for k, v in static.items() if v['min_mm'] < -0.05],
                                            under_2=[k for k, v in static.items() if v['min_mm'] < 2.0],
                                            centreline_inside_a_part=inside, in_d435_fields=fov_hits,
                                            note='centreline samples every 0.5 mm to the nearest surface sample of any part (~1 per mm2, +-0.3 mm) minus '
                                                 "the lead's radius; its own two plugs and its last r + 1.5 mm at each end left out")
    devs = ('computer', 'power', 'mcu', 'lidar_adapter', 'conv_19v', 'conv_12v', 'conv_5v', 'servo_adapter', 'display', 'mic_array', 'env_lidar', 'battery')
    ov = {}
    for a, sa in plugs.items():
        for c_, sc in dict(world, **fov).items():
            if c_ in devs:
                continue
            if W.near(sa, sc, 0.0):
                v = W.overlap(sa, sc)
                if v > 0.05:
                    ov[f'{a} / {c_}'] = round(v, 2)
    pk = list(plugs)
    chain = {'lead_battery_xt90', 'plug_xt90_pair', 'lead_pb_xt90', 'plug_battery_balance', 'battery', 'strap_battery_0', 'strap_battery_1'}
    for n, a in enumerate(pk):
        for c_ in pk[n + 1:]:
            if {a, c_} <= chain:
                continue
            if W.near(plugs[a], plugs[c_], 0.0):
                v = W.overlap(plugs[a], plugs[c_])
                if v > 0.05:
                    ov[f'{a} / {c_}'] = round(v, 2)
    rep['plugs_overlap_mm3'] = ov
    ids = list(paths)
    trees = {k: cKDTree(samples(paths[k])[0]) for k in ids}
    shared = {}
    for n, a in enumerate(ids):
        for c_ in ids[n + 1:]:
            d, _ = trees[c_].query(trees[a].data)
            g = float(d.min()) - lead_r(a) - lead_r(c_)
            if g < 0:
                shared[f'{a} / {c_}'] = round(g, 1)
    rep['leads_sharing_a_path'] = dict(pairs=shared, note='negative = the two drawn tubes overlap by that much: built, they lie side by side in one bundle')
    mov = {}
    for pose in ('droop', 'nominal', 'bump'):                           # v15: the pods' moving parts (and what drops out with them) per pose
        mshapes, mnames = [], []
        for name, sh in pods.items():
            for kk, s_ in sh.items():
                if kk.startswith((pose + '/', 'static/')):
                    mshapes.append(s_)
                    mnames.append(f'{name}/{kk}')
        Pm, Im = surface_points(mshapes, 0.5)
        tm = cKDTree(Pm)
        for rid, path in paths.items():
            q, sl = samples(path)
            if rid.startswith('M_'):                                    # its fixed leg and loop ride with its own fork, then 20 mm more
                q = q[sl > leg_len[rid[2:]] + 20.0]
            d, ix = tm.query(q)
            m = int(np.argmin(d))
            cl = float(d[m]) - lead_r(rid)
            if rid not in mov or cl < mov[rid]['min_mm']:
                mov[rid] = dict(min_mm=round(cl, 2), against=mnames[Im[ix[m]]], pose=pose)
    rep['moving_pods_min_mm'] = dict(per_lead=mov, under_4mm={k: v for k, v in mov.items() if v['min_mm'] < 4.0},
                                     note='fork, wheel, motor, rings, retainer, plungers in droop / nominal / bump, and the hangers and beam (they drop '
                                          "out with the pod); a motor lead's fixed leg on its fork and its first 20 mm of service loop past the tie are left "
                                          'out; samples +-0.4 mm')
    occ = np.load(HR.arm_cache())['occ']
    ta = cKDTree(HR.ORG + (np.argwhere(occ) + 0.5) * HR.PITCH)
    arm = {}
    for rid, path in paths.items():
        d, _ = ta.query(samples(path)[0])
        arm[rid] = round(float(d.min()) - lead_r(rid) - HR.PITCH * math.sqrt(3) / 2, 1)
    rep['arm_sweep_min_mm'] = dict(per_lead=arm, min=min(arm.values()), under_4mm={k: v for k, v in arm.items() if v < 4.0},
                                   note='to the nearest swept-arm cell centre minus half its diagonal and the lead radius (a lower bound on the grid)')
    sup = lidar_supports()
    band = {}
    for rid, path in paths.items():
        r = lead_r(rid)
        q, _ = samples(path)
        q = q[np.abs(q[:, 2] - F.LIDAR['plane']) <= 3.0 + r]
        if not len(q):
            continue
        dist = np.hypot(q[:, 0] - F.LIDAR_X, q[:, 1])
        brg = np.degrees(np.arctan2(q[:, 1], q[:, 0] - F.LIDAR_X))
        half = np.degrees(np.arctan(r / dist))
        hid = np.zeros(len(q), bool)
        margin = None
        for s_ in sup:
            off = np.abs(((brg - s_['bearing'] + 180) % 360) - 180) + half
            hid |= off <= s_['shadow_deg'] / 2 + 1e-6
            if (off < 10).all():
                margin = round(float(s_['shadow_deg'] / 2 - off.max()), 2)
        lo, hi = brg[~hid] - half[~hid], brg[~hid] + half[~hid]
        band[rid] = dict(samples=int(len(q)), outside_fin_shadows=int((~hid).sum()),
                         blind_added_deg=round(float(hi.max() - lo.min()), 2) if (~hid).any() else 0.0, shadow_margin_deg=margin)
    rep['scan_band'] = dict(per_lead=band, blind_added_deg=round(sum(v['blind_added_deg'] for v in band.values()), 2),
                            note="a lead's full width, as the lidar sees it, wherever its centreline is within 3 mm + r of the scan plane; inside a fin's "
                                 "shadow it adds nothing (margin: deg of the shadow's half width left beside the lead)")
    rep['bends_under_allowed'] = {k: dict(tightest=v['tightest_bend_r'], allowed=v['bend_r_allowed'], n=v['bends_under_allowed'])
                                  for k, v in info.items() if v['bends_under_allowed']}
    return rep


def b_sphere(c, r):
    import build123d as b
    return b.Solid.make_sphere(r).moved(b.Location(tuple(float(v) for v in c)))


def colour(k):
    if k.startswith('lead_M_'):
        return (0.85, 0.15, 0.15)
    if k.startswith(('lead_CAN', 'lead_IMU', 'lead_TEENSY')):
        return (0.15, 0.55, 0.85)
    if k.startswith(('lead_PC19', 'lead_ARM12', 'lead_SERVO12', 'lead_5V', 'lead_battery', 'lead_pb')):
        return (0.9, 0.55, 0.1)
    if k.startswith('lead_'):
        return (0.2, 0.2, 0.22)
    return (0.95, 0.95, 0.3)


def render(items):
    """the harness with the second floor, the electronics and the chassis (pods nominal), from above front-right and rear-left; plan"""
    import pod_lean as L
    import upper as U
    printed, bought, vendor, hw, _ = U.build()
    ctx = {k: v for k, v in dict(printed, **bought).items() if k.startswith(('floor2', 'computer', 'battery', 'power', 'mcu', 'riser_arm', 'servo',
                                                                              'imu', 'bracket_imu', 'plug_', 'd435', 'post_', 'bracket_battery'))}
    allc = {}
    for k, s in dict(ctx, **items).items():
        try:
            s.tessellate(0.6, 0.4)
            allc[k] = s
        except Exception:                                                  # noqa: BLE001
            print('render: skipped (does not tessellate)', k, flush=True)
    cf = lambda k: colour(k) if k in items else (U.colour(k) if not k.startswith('floor2') else (0.82, 0.84, 0.88))   # noqa: E731
    L.iso(allc, VAL / 'harness_iso_rear_right.png', 225 + 90, 35, 'harness over the second floor (from the rear right): leads red motor, '
          'orange power, blue MCU, black USB / video', cf, tol=0.6)
    L.iso(allc, VAL / 'harness_iso_front_left.png', 45 + 90, 35, 'harness over the second floor (from the front left)', cf, tol=0.6)


def export(items):
    from step_colour import write_step
    OUT.mkdir(parents=True, exist_ok=True)
    write_step({k: (s, colour(k)) for k, s in items.items()}, OUT / 'harness_review.step', 'harness')


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    if stage == 'ties':
        ties_stage()
        return
    if stage == 'grid':
        grid_stage()
        return
    if stage in ('build', 'check', 'render', 'all'):
        items, info, paths = build()
        print(len(items), 'harness items', json.dumps(info, indent=0)[:3000], flush=True)
        if stage in ('check', 'all'):
            rep = check(items, info, paths)
            (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
            print(json.dumps({k: v for k, v in rep.items() if k != 'leads'}, indent=1)[:6000], flush=True)
        if stage in ('render', 'all'):
            render(items)
            export(items)
        if stage != 'all':
            return
    if stage in ('power', 'all', 'inventory'):
        pw = power_budget()
        (VAL / 'power.json').write_text(json.dumps(pw, indent=1), encoding='utf-8')
        print(json.dumps({k: pw[k] for k in ('rails', 'converters', 'battery')}, indent=1))
    if stage in ('inventory', 'all'):
        rf = VAL / 'routes.json'
        routes = json.loads(rf.read_text())['runs'] if rf.exists() else {}
        rows = inventory(routes)
        (VAL / 'inventory.json').write_text(json.dumps(dict(ports=ports(), connections=rows), indent=1), encoding='utf-8')
        print(md_table(rows, [('id', lambda r: r['id']), ('from', lambda r: r['a']), ('to', lambda r: r['b']), ('lead', lambda r: r.get('lead')),
                               ('mm', lambda r: r.get('length_mm'))]))


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'inventory')
