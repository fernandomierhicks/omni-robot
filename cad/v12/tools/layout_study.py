"""cad/v11 exploratory layout study (Nov 3 2026): where the electronics, computer, battery, lidar, D435 and the arm's second floor go
on the compact lean-pod base (base.py). Preliminary envelopes only (boxes, cylinders, real-size plugs and bent leads), not detail CAD.
The bin and the face / screen are not studied (their second-floor areas are only marked).

    python -B layout_study.py            (omni-robot-sim interpreter; writes cad/v12/validation/layout/)

Two hand patterns on the same base (D 146, wheels slid 32 mm along their rolling lines, plate top 62 mm above the floor):
  A 'sides': the sim's pattern (pivots toward the left / right side midlines); B 'front': the same robot turned 90 deg
  (pivots toward the front / back), i.e. sim forward = B's sideways.
Every device is placed by a small search in the free deck (plate minus each pod's plan footprint swept over droop..bump, + 3 mm),
devices 3 mm apart including their connector / lead keep-outs, each as close as it can get to its target point.
Devices, ports and plug sizes are ASSUMED where marked VERIFY; the BMAX B8A Pro port layout is a generic mini-PC guess.
"""
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import MultiPoint, Point, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL                                               # noqa: E402

VAL = W.ROOT / 'validation' / 'layout'
DECK = B.R_WHEEL + L.PLATE_TOP_L          # 62: deck top above the floor
FLOOR2 = (176.5, 181.5)                    # second floor (sim ARM payload plate top 181.5; arm base 218.5 on a 37 mm riser)
LIDAR_PLANE = 163.0                        # sim ARM lidar_scan_z
LIDAR = dict(d=76.0, h=41.0, plane_above_base=30.0, mass=0.19)   # RPLIDAR A2M12 envelope; plane height in its body VERIFY
BAY_TOP = LIDAR_PLANE - 5.0                # bay devices stay under the scan plane (it sees over every pod part: wheel top 154.8 at bump)
GAP = 3.0
STANDOFF = 3.0


# ---- connectors: real-size plug overmold + its lead leaving straight, a 90 deg bend at the allowed radius, then 30 mm -----------------
PLUGS = {   # overmold w x h x depth out of the panel, lead OD, bend radius (~5 x OD) -- all VERIFY against the bought cables
    'usb_a3': (16.0, 8.5, 30.0, 4.6, 23.0), 'usb_a2': (15.0, 8.0, 28.0, 3.8, 19.0), 'usb_c': (12.0, 7.0, 25.0, 5.0, 25.0),
    'micro_b': (11.0, 7.0, 25.0, 3.8, 19.0), 'dc_barrel': (10.0, 10.0, 32.0, 4.5, 15.0), 'hdmi': (21.0, 11.0, 38.0, 7.0, 35.0),
    'xt90': (21.0, 16.0, 30.0, 10.0, 30.0), 'xt30': (10.0, 6.0, 20.0, 5.0, 15.0)}


def plug(kind, turn):
    """local frame: panel at x = 0, plug out along +x, centre at y = z = 0; the lead bends toward `turn` ('+y', '-y', '+z', '-z')"""
    w, h, dpt, od, R = PLUGS[kind]
    s = P.box(0, dpt, -w / 2, w / 2, -h / 2, h / 2)
    x0 = dpt + 3.0
    lead = W.along_x(od / 2, dpt - 0.5, x0)
    ax = {'+y': (0, 1, 0), '-y': (0, -1, 0), '+z': (0, 0, 1), '-z': (0, 0, -1)}[turn]
    n = np.array(ax, float)
    centre = np.array([x0, 0, 0]) + R * n
    axis_dir = np.cross([1, 0, 0], n)                                   # bend axis
    circ = b.Face(b.Wire.make_circle(od / 2, b.Plane(origin=(x0, 0, 0), z_dir=(1, 0, 0))))
    bend = b.revolve(circ, b.Axis(tuple(centre), tuple(axis_dir)), 90)
    end = centre + R * np.array([1, 0, 0])
    tail = b.Solid.make_cylinder(od / 2, 15.0, b.Plane(origin=tuple(end), z_dir=tuple(n)))
    return s + lead + bend + tail


def on_face(shape, face, at):
    """move a plug built at the origin (out along +x) onto a device face: face in '+x','-x','+y','-y'; at = (u, z) along the face"""
    rot = {'+x': 0, '+y': 90, '-x': 180, '-y': 270}[face]
    return shape.rotate(b.Axis.Z, rot), rot


# ---- devices: local frame, centred on the footprint, z from the device's underside --------------------------------------------------
def dev_box(l, w, h):
    return P.box(-l / 2, l / 2, -w / 2, w / 2, 0, h)


def device(name, size, mass, ports=(), zones=(), note=''):
    """size (l along local x, w, h); ports: (kind, face, u, z, turn, label); zones: (face, depth) plain keep-outs"""
    l, w, h = size
    body = dev_box(l, w, h)
    plugs = []
    for kind, face, u, z, turn, label in ports:
        pl = plug(kind, turn)
        rot = {'+x': 0, '+y': 90, '-x': 180, '-y': 270}[face]
        off = {'+x': (l / 2, u), '+y': (-u, w / 2), '-x': (-l / 2, -u), '-y': (u, -w / 2)}[face]
        plugs.append((label, pl.rotate(b.Axis.Z, rot).moved(b.Location((off[0], off[1], z)))))
    fp = [MultiPoint(np.array([[q.X, q.Y] for q in s.tessellate(0.5, 0.5)[0]])).convex_hull for _, s in plugs]
    fp.append(sbox(-l / 2, -w / 2, l / 2, w / 2))
    for face, dpt in zones:
        fp.append({'+x': sbox(l / 2, -w / 2, l / 2 + dpt, w / 2), '-x': sbox(-l / 2 - dpt, -w / 2, -l / 2, w / 2),
                   '+y': sbox(-l / 2, w / 2, l / 2, w / 2 + dpt), '-y': sbox(-l / 2, -w / 2 - dpt, l / 2, -w / 2)}[face])
    return dict(name=name, size=size, mass=mass, body=body, plugs=plugs, fp=union_all(fp), note=note)


def devices():
    return [
        device('computer', (112.0, 126.0, 52.0), 0.43,
               ports=[('usb_a3', '+x', -40, 20, '+z', 'D435 (USB 3)'), ('usb_a3', '+x', -22, 20, '+z', 'Teensy'),
                      ('usb_a2', '-x', 32, 15, '+z', 'SO-101 bus adapter'), ('usb_a2', '-x', 32, 32, '+z', 'A2M12 adapter'),
                      ('dc_barrel', '-x', -50, 22, '+z', '19 V in')],
               zones=[('+x', 12.0)],
               note='BMAX B8A Pro 126 x 112 x 52, 430 g (maker); port layout ASSUMED: front 2 USB-A 3 / C / jack / power button, rear DC / HDMI / DP / LAN / 2 USB-A 2 (VERIFY)'),
        device('battery', (157.0, 47.0, 46.0), 0.706, zones=[('+x', 50.0), ('-x', 8.0)],
               note='OVONIC Roam 6S 5200 157 x 47 x 46, 706 g (maker); keep-outs: XT90-S pigtail bend + the mated pair + balance lead in line at one end (50)'),
        device('power', (130.0, 70.0, 32.0), 0.35,
               zones=[('-x', 40.0), ('+x', 30.0), ('+y', 10.0), ('-y', 10.0)],
               note='est (keep-outs: 6S lead in 40, motor XT30 leads out 30): fuse, e-stop contactor, current sensor, 19 V / 100 W buck (PC), 12 V / 8 A buck (arm), 5 V buck, motor distribution'),
        device('mcu', (100.0, 65.0, 25.0), 0.08, ports=[('micro_b', '-x', 20, 12, '+z', 'Teensy USB')], zones=[('+x', 25.0), ('+y', 20.0)],
               note='est: Teensy 4.1 carrier: CAN transceivers, BNO085, ELRS receiver, buzzer, bumper / e-stop inputs; CAN + 12 V headers'),
        device('servo_adapter', (45.0, 35.0, 15.0), 0.02, ports=[('usb_c', '-x', 0, 8, '+z', 'USB')], zones=[('+x', 25.0)],
               note='Waveshare Bus Servo Adapter (A) est; SO-101 bus + 12 V in'),
    ]


ALL = ('computer', 'battery', 'power', 'mcu', 'servo_adapter')
TOP_DEV = ('computer', 'power', 'mcu', 'servo_adapter')
CONFIGS = {   # pattern, where each device goes ('deck' between the pods / 'top' on the second floor), the lidar's place
    'A': dict(pattern='sides', surface={k: 'deck' for k in ALL}, lidar=(0.0, 0.0),
              title='A: sim hand pattern (pivots toward the sides), everything on the deck between the pods'),
    'B': dict(pattern='front', surface={k: 'deck' for k in ALL}, lidar=(140.0, 0.0),
              title='B: pivots toward the front / back (sim turned 90 deg), everything on the deck between the pods'),
    'C': dict(pattern='front', surface={k: 'top' for k in ALL}, lidar=(140.0, 0.0),
              title='C: pivots toward the front / back; battery + electronics in one block on the second floor rear; lidar and D435 in the front lobe'),
    'D': dict(pattern='sides', surface={k: 'top' for k in ALL}, lidar=(0.0, 0.0),
              title='D: sim hand pattern; battery + electronics block on the second floor rear; lidar on a pedestal at the deck centre'),
    'E': dict(pattern='front', surface={k: 'top' for k in ALL}, lidar=(140.0, 0.0), D=134.0, slide=-34.0,
              title='E: as C on the smallest base (pods bolted into a ring, D 134)'),
    'F': dict(pattern='sides', surface={k: 'top' for k in ALL}, lidar=(0.0, 0.0), D=154.0, slide=-20.0,
              title='F: sim hand pattern, base grown until the D435 nose clears; electronics block on the second floor; lidar on a pedestal at the centre'),
}
STACK = dict(mcu=('power', 10.0))           # on the second floor the MCU carrier stands on the power board (standoffs, mm)
TARGET = dict(deck=dict(computer=(0, 0), battery=(0, 0), power=(0, 0), mcu=(0, 0), servo_adapter=(0, 0)),
              top=dict(computer=(-135, 0), battery=(-40, 0), power=(-60, 120), mcu=(-60, -120), servo_adapter=(30, -150)))   # one block at the rear; the adapter by the arm
YAWS = (0, 90, 45, 135)
ORDERS = (ALL, ('battery', 'computer', 'power', 'mcu', 'servo_adapter'), ('computer', 'power', 'battery', 'mcu', 'servo_adapter'),
          ('power', 'computer', 'battery', 'mcu', 'servo_adapter'), ('computer', 'battery', 'mcu', 'power', 'servo_adapter'))
ARM_BASE = (140.0, 0.0)
ARM_KEEP = 100.0                           # second floor kept clear round the arm's base (its adapter + the folded arm's low links)


def pods_fp(pattern):
    B.mirrored = (lambda i: round(B.ANGLES[i]) % 180 == (45 if pattern == 'sides' else 135))
    cache = VAL / 'pod_fp.pkl'
    if cache.exists():
        fp = pickle.loads(cache.read_bytes())
    else:
        parts, _ = L.build_static()
        fp = B.footprints(parts, P.wheel_unit())
        cache.write_bytes(pickle.dumps(fp))
    return fp


def place(fp_dev, xy, yaw):
    return affinity.translate(affinity.rotate(fp_dev, yaw, origin=(0, 0)), *xy)


def search(free, taken, dev, target):
    from shapely.prepared import prep
    pf = prep(free)
    best = None
    for yaw in YAWS:
        g0 = affinity.rotate(dev['fp'], yaw, origin=(0, 0))
        for x in np.arange(-215, 216, 4.0):
            for y in np.arange(-215, 216, 4.0):
                d = math.hypot(x - target[0], y - target[1])
                if best and d >= best[0]:
                    continue
                g = affinity.translate(g0, x, y)
                if pf.contains(g) and (taken is None or not g.intersects(taken)):
                    best = (d, (float(x), float(y)), yaw, g)
    return best


def nose_band(pattern, z0=95.0, z1=130.0):
    """the pods' plan in a height band (robot z; default the D435's 95..130), placed, + 3 mm"""
    f = VAL / f'band_{int(z0)}_{int(z1)}.pkl'
    if f.exists():
        return union_all([B.place2d(pickle.loads(f.read_bytes()), i).buffer(3.0) for i in range(4)])
    pods_fp(pattern)
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    gs = []
    for a in L.POSES.values():
        sh, _ = L.posed(parts, unit, a)
        for k, s in sh.items():
            q = B.proj(s, k.startswith(B.EXACT), 95 - R_WHEEL, 130 - R_WHEEL)
            if q is not None:
                gs.append(q)
    g = union_all(gs)
    f.write_bytes(pickle.dumps(g))
    return union_all([B.place2d(g, i).buffer(3.0) for i in range(4)])


def study(name):
    cfg = CONFIGS[name]
    pattern = cfg['pattern']
    B.D_WHEEL, B.SLIDE = cfg.get('D', 146.0), cfg.get('slide', -32.0)
    pods_fp(pattern)                                                    # sets the hand pattern before anything is placed
    nose = nose_band(pattern)
    lidar_band = nose_band(pattern, LIDAR_PLANE - LIDAR['plane_above_base'] - 5, FLOOR2[0])
    fp = pods_fp(pattern)
    pods = union_all([B.place2d(fp['all'], i) for i in range(4)])
    r_in = B.r_bumper(fp) - 0.5
    deck_free = Point(0, 0).buffer(r_in, 256).difference(pods.buffer(GAP))
    top_free = Point(0, 0).buffer(r_in - 3, 256).difference(Point(*ARM_BASE).buffer(ARM_KEEP)).difference(sbox(52, -66, 230, 66))
    out = dict(config=name, title=cfg['title'], pattern=pattern, wheels={B.NAMES[i]: [round(v, 1) for v in B.frame(i)[0]] for i in range(4)},
               robot_dia=round(2 * (r_in + 0.5 + B.BUMPER_T)), free_deck_cm2=round(deck_free.area / 100), free_top_cm2=round(top_free.area / 100))
    d435 = sbox(177.5, -45, 202.5, 45)
    out['d435_nose'] = dict(clear_of_wheels_mm=round(d435.distance(nose), 1), overlap_mm2=round(d435.intersection(nose).area, 1))
    lid_xy = cfg['lidar']
    lidar_fp = Point(*lid_xy).buffer(LIDAR['d'] / 2 + GAP)
    out['lidar_clear_of_pods_mm'] = round(lidar_fp.distance(lidar_band) + GAP, 1)
    best = None
    for order in ORDERS:                                                # greedy packing; keep the first order that places everything
        taken = {'deck': (lidar_fp if lid_xy != (0.0, 0.0) else None), 'top': None}
        placed = {}
        for nm in order:
            dev = next(d for d in devices() if d['name'] == nm)
            surf = cfg['surface'][nm]
            if surf == 'top' and nm in STACK:
                continue
            free = deck_free if surf == 'deck' else top_free
            r = search(free, taken[surf], dev, TARGET[surf][nm])
            if r is None:
                placed[nm] = None
                continue
            d, xy, yaw, g = r
            placed[nm] = dict(xy=xy, yaw=yaw, footprint=g, dev=dev, surface=surf)
            taken[surf] = g.buffer(GAP) if taken[surf] is None else union_all([taken[surf], g.buffer(GAP)])
        n = sum(v is not None for v in placed.values())
        if best is None or n > best[0]:
            best = (n, placed, taken)
        if n == len(ALL) or all(cfg['surface'][k] == 'deck' for k in ALL):
            break
    _, placed, taken = best
    for nm, (on, gap) in STACK.items():
        if cfg['surface'][nm] == 'top' and placed.get(on):
            dev = next(d for d in devices() if d['name'] == nm)
            q = placed[on]
            placed[nm] = dict(xy=q['xy'], yaw=q['yaw'], footprint=place(dev['fp'], q['xy'], q['yaw']), dev=dev, surface='top',
                              lift=q['dev']['size'][2] + gap, stacked_on=on)
    placed = {k: placed.get(k) for k in ALL}
    base_z = dict(deck=DECK + STANDOFF, top=FLOOR2[1] + STANDOFF)
    out['placed'] = {k: (None if v is None else dict(surface=v['surface'], xy=[round(c, 1) for c in v['xy']], yaw=v['yaw'], on=v.get('stacked_on'),
                                                     top_z=round(base_z[v['surface']] + v.get('lift', 0) + v['dev']['size'][2], 1),
                                                     keep_out_area_cm2=round(v['footprint'].area / 100, 1))) for k, v in placed.items()}
    out['missing'] = [k for k, v in placed.items() if v is None]
    # the block's height on the second floor: devices + their leads bending up
    tops = []
    for k, v in placed.items():
        if v and v['surface'] == 'top':
            zmax = max([v['dev']['size'][2]] + [s.bounding_box().max.Z for _, s in v['dev']['plugs']])
            tops.append(base_z['top'] + v.get('lift', 0) + zmax)
    out['top_block_top_z'] = round(max(tops), 1) if tops else None
    # posts for the second floor, in the free deck, away from the lidar's front sector
    posts = []
    for ang in ((90, 150, 210, 270) if pattern == 'front' else (60, 120, 240, 300)):
        best = None
        for da in np.arange(-25, 26, 1.0):
            for r in np.arange(200, 80, -2.0):
                p = Point(r * math.cos(math.radians(ang + da)), r * math.sin(math.radians(ang + da)))
                if deck_free.contains(p.buffer(10)) and (taken['deck'] is None or not taken['deck'].intersects(p.buffer(10))):
                    if best is None or r - 2 * abs(da) > best[0]:
                        best = (r - 2 * abs(da), p)
                    break
        if best:
            posts.append((round(best[1].x, 1), round(best[1].y, 1)))
    out['posts'] = posts
    lx, ly = lid_xy
    shadows = [dict(what='post Ø16', bearing=round(math.degrees(math.atan2(py - ly, px - lx)), 1),
                    width_deg=round(2 * math.degrees(math.asin(8 / math.hypot(px - lx, py - ly))), 1)) for px, py in posts]
    out['lidar'] = dict(xy=lid_xy, scan_z=LIDAR_PLANE, shadows=shadows, blind_deg=round(sum(s['width_deg'] for s in shadows), 1),
                        note='no pod part reaches the plane (wheel top 154.8 at bump); the arm crosses it only while reaching down in front')
    # masses: pods 4 x 1.45 (est), deck 0.45, second floor 0.5, bumper 0.3, arm 0.85 at its stow, D435, lidar, devices, block enclosure 0.25
    mass = [(4 * 1.45, (0, 0, 72)), (0.45, (0, 0, DECK - 3)), (0.5, (0, 0, sum(FLOOR2) / 2)), (0.3, (0, 0, 65)), (0.85, (ARM_BASE[0], 0, 300)),
            (0.072, (190, 0, 111.5)), (LIDAR['mass'], (lx, ly, LIDAR_PLANE))]
    for k, v in placed.items():
        if v:
            mass.append((v['dev']['mass'], (v['xy'][0], v['xy'][1], base_z[v['surface']] + v.get('lift', 0) + v['dev']['size'][2] / 2)))
    if out['top_block_top_z']:
        mass.append((0.25, (-120, 0, (FLOOR2[1] + out['top_block_top_z']) / 2)))
    m = sum(a for a, _ in mass)
    cg = [sum(a * p[i] for a, p in mass) / m for i in range(3)]
    w = np.array(list(out['wheels'].values()))
    hx, hy = float(abs(w[:, 0]).min()), float(abs(w[:, 1]).min())
    out['mass_cg'] = dict(kg=round(m, 2), cg=[round(c, 1) for c in cg],
                          note='rough: pods 1.45 kg each (est), arm 0.85 kg at its stow, bin / face / head / wiring NOT included')
    out['stability'] = dict(half_wheelbase=round(hx, 1), half_track=round(hy, 1), tip_fwd_g=round((hx - cg[0]) / cg[2], 2), tip_back_g=round((hx + cg[0]) / cg[2], 2),
                            tip_side_g=round(hy / cg[2], 2), note='static rigid-body tipping about the wheel contacts at this CG (sim to confirm with pods and the full robot)')
    return out, dict(deck_free=deck_free, top_free=top_free, pods=pods, fp=fp, placed=placed, posts=posts, lid_xy=lid_xy, d435=d435, nose=nose)


# ---- renders ---------------------------------------------------------------------------------------------------------------------------
COL = dict(computer=(0.35, 0.45, 0.85), battery=(0.85, 0.25, 0.25), power=(0.95, 0.6, 0.1), mcu=(0.2, 0.65, 0.3), servo_adapter=(0.6, 0.3, 0.75))


def render(name, out, g):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MP, Rectangle

    def fill(ax, geom, **kw):
        for q in getattr(geom, 'geoms', [geom]):
            if q.is_empty:
                continue
            ax.add_patch(MP(np.asarray(q.exterior.coords), closed=True, **kw))

    def devs(ax, surf):
        for k, v in g['placed'].items():
            if v is None or v['surface'] != surf:
                continue
            c = COL[k]
            fill(ax, v['footprint'], facecolor=(*c, 0.2), edgecolor=c, lw=0.8, ls='--')
            sx, sy = v['dev']['size'][:2]
            fill(ax, place(sbox(-sx / 2, -sy / 2, sx / 2, sy / 2), v['xy'], v['yaw']), facecolor=(*c, 0.85), edgecolor='k', lw=0.8)
            for lab, s in v['dev']['plugs']:
                hp = place(MultiPoint(np.array([[q.X, q.Y] for q in s.tessellate(0.5, 0.5)[0]])).convex_hull, v['xy'], v['yaw'])
                fill(ax, hp, facecolor=(1, 0, 1, 0.8), edgecolor='m', lw=0.5)
            ax.text(*v['xy'], k.replace('_', '\n'), ha='center', va='center', fontsize=8, color='w', weight='bold')

    rb = B.r_bumper(g['fp'])
    fig, axs = plt.subplots(1, 3, figsize=(27, 9.5), gridspec_kw=dict(width_ratios=[1, 1, 1.25]))
    # deck plan
    ax = axs[0]
    ax.add_patch(Circle((0, 0), rb + B.BUMPER_T, facecolor=(0.9, 0.45, 0.15), edgecolor='none'))
    ax.add_patch(Circle((0, 0), rb, facecolor='white', edgecolor='none'))
    fill(ax, g['deck_free'], facecolor=(0.88, 0.95, 0.88), edgecolor='none')
    fill(ax, g['pods'], facecolor=(0.72, 0.72, 0.72), edgecolor='0.4', lw=0.5)
    for i in range(4):
        ax.text(*(B.frame(i)[0] * 1.05), B.NAMES[i], ha='center', va='center', fontsize=11, weight='bold')
    devs(ax, 'deck')
    lx, ly = g['lid_xy']
    ax.add_patch(Circle((lx, ly), LIDAR['d'] / 2, facecolor=(0.1, 0.1, 0.1, 0.35), edgecolor='k', lw=1.2))
    ax.text(lx, ly - 48 if lx == 0 else ly, 'A2M12 (z 133-174)', ha='center', va='center', fontsize=8)
    fill(ax, g['d435'], facecolor=(0.2, 0.2, 0.9, 0.5), edgecolor='b')
    fill(ax, g['nose'], facecolor='none', edgecolor='r', lw=0.7)
    for (px, py) in g['posts']:
        ax.add_patch(Circle((px, py), 8, facecolor='k'))
    for s in out['lidar']['shadows']:
        a0, wdt = math.radians(s['bearing']), math.radians(s['width_deg'])
        for a in (a0 - wdt / 2, a0 + wdt / 2):
            ax.plot([lx, lx + 300 * math.cos(a)], [ly, ly + 300 * math.sin(a)], color=(0.45, 0.45, 0.45), lw=0.5)
    miss = ', '.join(k for k in out['missing'] if CONFIGS[name]['surface'][k] == 'deck')
    for a_ in (ax, axs[1]):
        a_.set_xlim(-235, 235)
        a_.set_ylim(-235, 235)
        a_.set_aspect('equal')
        a_.grid(True, lw=0.3)
    ax.set_title(f'DECK (top z {DECK:g}): grey pods (swept droop..bump), green free deck {out["free_deck_cm2"]} cm2\n'
                 f'blue D435 at the nose (red: pods at its height); black posts; lines: lidar shadows' + (f'\nDOES NOT FIT: {miss}' if miss else ''), fontsize=9,
                 color='r' if miss else 'k')
    # second floor plan
    ax = axs[1]
    ax.add_patch(Circle((0, 0), rb + B.BUMPER_T, facecolor=(0.9, 0.45, 0.15, 0.4), edgecolor='none'))
    ax.add_patch(Circle((0, 0), rb - 0.5, facecolor=(0.9, 0.9, 0.92), edgecolor='0.5'))
    ax.add_patch(Circle(ARM_BASE, ARM_KEEP, facecolor='none', edgecolor='0.4', ls='--'))
    ax.add_patch(Rectangle((62, -56), 143, 112, facecolor=(0.8, 0.8, 0.85), edgecolor='k'))
    ax.add_patch(Circle(ARM_BASE, 25, facecolor='w', edgecolor='k'))
    ax.text(140, 0, 'SO-101\nbase', ha='center', va='center', fontsize=8)
    ax.add_patch(Rectangle((-130, -60), 120, 120, facecolor='none', edgecolor='0.5', hatch='//', lw=0.6))
    ax.text(-70, 68, 'bin / face: not studied\n(on the block)', ha='center', fontsize=8, color='0.35')
    devs(ax, 'top')
    for (px, py) in g['posts']:
        ax.add_patch(Circle((px, py), 8, facecolor='none', edgecolor='k'))
    t = out['top_block_top_z']
    miss = ', '.join(k for k in out['missing'] if CONFIGS[name]['surface'][k] == 'top')
    ax.set_title(f'SECOND FLOOR (top z {FLOOR2[1]:g}): arm adapter and its keep-out (dashed r {ARM_KEEP:g})'
                 + (f'\nelectronics block, devices + leads bent up: top z {t}' if t else '\n(no devices here)') + (f'\nDOES NOT FIT: {miss}' if miss else ''),
                 fontsize=9, color='r' if miss else 'k')
    # side elevation
    ax = axs[2]
    ax.axhline(0, color='k', lw=1)
    ax.add_patch(Rectangle((-rb + 0.5, DECK - 6), 2 * rb - 1, 6, facecolor=(0.75, 0.75, 0.72)))
    ax.add_patch(Rectangle((-rb + 0.5, FLOOR2[0]), 2 * rb - 1, FLOOR2[1] - FLOOR2[0], facecolor=(0.6, 0.62, 0.66)))
    for sx in (rb, -rb - B.BUMPER_T):
        ax.add_patch(Rectangle((sx, 40), B.BUMPER_T, 50, facecolor=(0.9, 0.45, 0.15)))
    for i in range(4):
        A = B.frame(i)[0]
        ax.add_patch(Circle((A[0], 70), 70, facecolor=(0.2, 0.24, 0.32, 0.2), edgecolor=(0.2, 0.24, 0.32)))
        ax.add_patch(Circle((A[0], 85), 70, facecolor='none', edgecolor=(0.2, 0.24, 0.32), ls=':'))
    for k, v in g['placed'].items():
        if v is None:
            continue
        sx, sy, sz = v['dev']['size']
        bb = place(sbox(-sx / 2, -sy / 2, sx / 2, sy / 2), v['xy'], v['yaw']).bounds
        z0 = (DECK + STANDOFF if v['surface'] == 'deck' else FLOOR2[1] + STANDOFF) + v.get('lift', 0)
        ax.add_patch(Rectangle((bb[0], z0), bb[2] - bb[0], sz, facecolor=(*COL[k], 0.6), edgecolor='k', lw=0.5))
    if t:
        ax.add_patch(Rectangle((-205, FLOOR2[1]), 175, t - FLOOR2[1], facecolor='none', edgecolor='k', ls='--', lw=0.8))
        ax.text(-200, t + 4, f'electronics block to z {t}', fontsize=8)
    ax.add_patch(Rectangle((lx - LIDAR['d'] / 2, LIDAR_PLANE - LIDAR['plane_above_base']), LIDAR['d'], LIDAR['h'], facecolor=(0.15, 0.15, 0.15)))
    ax.axhline(LIDAR_PLANE, color='g', lw=1, ls='--')
    ax.add_patch(Rectangle((177.5, 99), 25, 25, facecolor=(0.2, 0.2, 0.9, 0.6)))
    ax.add_patch(Rectangle((62, FLOOR2[1]), 143, 37, facecolor=(0.85, 0.85, 0.88), edgecolor='k', lw=0.5))
    ax.plot([140, 140, 300, 310], [218.5, 330, 120, 0], color='k', lw=3, alpha=0.35)
    ax.annotate('SO-101 base 218.5 (sim); fingers-down floor\nreach 260-360 mm from the centre', (150, 340), fontsize=8)
    for (px, py) in g['posts']:
        ax.add_patch(Rectangle((px - 8, DECK), 16, FLOOR2[0] - DECK, facecolor=(0, 0, 0, 0.2)))
    for z, lab, c in ((LIDAR_PLANE + 3, f'lidar scan z {LIDAR_PLANE:g}', 'g'), (DECK - 18, f'deck top {DECK:g}', 'k'), (FLOOR2[1] + 3, f'second floor {FLOOR2[1]:g}', 'k'),
                      (157, 'wheel top at bump 154.8', (0.2, 0.24, 0.32))):
        ax.text(-rb - 25, z, lab, color=c, fontsize=8, ha='right')
    ax.set_xlim(-330, 390)
    ax.set_ylim(-10, 440)
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    ax.set_title('side elevation (x-z, all y), front = +x; wheels nominal (dotted: bump)', fontsize=9)
    fig.suptitle(CONFIGS[name]['title'] + f'  -  D {B.D_WHEEL:g}, slide {B.SLIDE:g}, robot Ø{out["robot_dia"]}, wheels at ±{out["stability"]["half_wheelbase"]:g} x / ±{out["stability"]["half_track"]:g} y', fontsize=12)
    fig.tight_layout()
    fig.savefig(VAL / f'layout_{name}.png', dpi=80)
    plt.close(fig)


def iso3d(name, g, with_floor):
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    items = {}
    pod = {k: v for k, v in B.pod_items(parts, unit).items() if not k.startswith(('roller_core', 'die_spring'))}
    for i in range(4):
        for k, s in pod.items():
            items[f'{B.NAMES[i]}/{k}'] = B.place3d(s, i)
    items['plate'] = B.solid_of(Point(0, 0).buffer(B.r_bumper(g['fp']) - 0.5, 128).difference(union_all([B.place2d(g['fp']['cutout'], i) for i in range(4)])), DECK - 6, DECK)
    for k, v in g['placed'].items():
        if v is None or (v['surface'] == 'top' and not with_floor):
            continue
        z0 = (DECK + STANDOFF if v['surface'] == 'deck' else FLOOR2[1] + STANDOFF) + v.get('lift', 0)
        loc = b.Location((v['xy'][0], v['xy'][1], z0), (0, 0, 1), v['yaw'])
        items[f'dev_{k}'] = v['dev']['body'].moved(loc)
        for n, (lab, s) in enumerate(v['dev']['plugs']):
            items[f'bolt_plug_{k}_{n}'] = s.moved(loc)
    lx, ly = g['lid_xy']
    items['dev_lidar'] = W.cyl(LIDAR['d'] / 2, LIDAR['h'], (lx, ly, LIDAR_PLANE - LIDAR['plane_above_base'] + LIDAR['h'] / 2))
    z_ped = LIDAR_PLANE - LIDAR['plane_above_base']
    items['spider_lidar_pedestal'] = W.cyl(20, z_ped - DECK, (lx, ly, (z_ped + DECK) / 2))
    items['dev_d435'] = P.box(177.5, 202.5, -45, 45, 99, 124)
    for n, (px, py) in enumerate(g['posts']):
        items[f'spider_post{n}'] = W.cyl(8, FLOOR2[0] - DECK, (px, py, (FLOOR2[0] + DECK) / 2))
    if with_floor:
        fl = Point(0, 0).buffer(B.r_bumper(g['fp']) - 0.5, 128)
        items['spider_floor'] = B.solid_of(fl, *FLOOR2)
        items['dev_arm_adapter'] = P.box(62, 205, -56, 56, FLOOR2[1], 218.5) + W.cyl(25, 40, (140, 0, 238.5))
    for big in [k for k in items if k in ('plate', 'spider_floor')]:      # tiles, so the painter's sort of the iso view stays right
        s0 = items.pop(big)
        for i, x in enumerate(range(-240, 240, 40)):
            for j, y in enumerate(range(-240, 240, 40)):
                t = s0 & P.box(x, x + 40, y, y + 40, -10, 300)
                if t is not None and t.solids():
                    items[f'{big}_{i}_{j}'] = t
    cols = {**{f'dev_{k}': c for k, c in COL.items()}, 'dev_lidar': (0.15, 0.15, 0.15), 'dev_d435': (0.25, 0.25, 0.85), 'dev_arm': (0.85, 0.85, 0.88)}

    def colour(nm):
        for k, c in cols.items():
            if nm.startswith(k):
                return c
        return B.colour(nm)
    L.iso(items, VAL / f'iso_{name}_{"full" if with_floor else "deck"}.png', 225, 25 if not with_floor else 30,
          f'{name}: {"with the second floor and the electronics block" if with_floor else "deck level (second floor hidden)"}', colour, tol=0.8)


def main():
    VAL.mkdir(parents=True, exist_ok=True)
    for f in VAL.glob('*.png'):
        f.unlink()
    rep = {}
    for name in CONFIGS:
        out, g = study(name)
        rep[name] = out
        print(json.dumps({k: v for k, v in out.items() if k not in ('wheels',)}, indent=None), flush=True)
        render(name, out, g)
        if name in ('C', 'F'):
            iso3d(name, g, False)
            iso3d(name, g, True)
    rep['devices'] = {d['name']: dict(size=d['size'], mass_kg=d['mass'], ports=[lab for lab, _ in d['plugs']], note=d['note']) for d in devices()}
    rep['plugs'] = {k: dict(w=v[0], h=v[1], depth=v[2], lead_od=v[3], bend_r=v[4]) for k, v in PLUGS.items()}
    (VAL / 'layout.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
