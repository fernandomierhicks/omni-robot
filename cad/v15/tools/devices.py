"""Electronics device envelopes and real-size plugs for the upper robot (upper.py). Envelopes only (boxes, real-size plug overmolds and
their bent leads), not detail CAD. Every size, mass and port position marked est / ASSUMED / VERIFY is unconfirmed.
(The device inventory of cad/v11-v13's layout_study.py, without the layout search itself.)
"""
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import union_all
from shapely.geometry import MultiPoint, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402

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
        device('battery', (136.0, 46.0, 43.0), 0.558, zones=[('+x', 50.0), ('-x', 8.0)],
               note='HRB 6S 3300 mAh 136 x 46 x 43, 558 g (Fernando, Nov 16; components/battery); keep-outs: XT90 pigtail bend (connector VERIFY) + the mated pair + balance lead in line at one end (50)'),
        device('power', (130.0, 70.0, 32.0), 0.35,
               zones=[('-x', 40.0), ('+x', 30.0), ('+y', 10.0), ('-y', 10.0)],
               note='est (keep-outs: 6S lead in 40, motor XT30 leads out 30): fuse, e-stop contactor, current sensor, 19 V / 100 W buck (PC), 12 V / 8 A buck (arm), 5 V buck, motor distribution'),
        device('mcu', (100.0, 65.0, 25.0), 0.08, ports=[('micro_b', '-x', 20, 12, '+z', 'Teensy USB')], zones=[('+x', 25.0), ('+y', 20.0)],
               note='est: Teensy 4.1 carrier: CAN transceivers, BNO085, ELRS receiver, buzzer, bumper / e-stop inputs; CAN + 12 V headers'),
        device('servo_adapter', (45.0, 35.0, 15.0), 0.02, ports=[('usb_c', '-x', 0, 8, '+z', 'USB')], zones=[('+x', 25.0)],
               note='Waveshare Bus Servo Adapter (A) est; SO-101 bus + 12 V in'),
    ]
