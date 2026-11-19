"""v15 assembly marks (Nov 15, Fernando: "numbers and inner/outer labels on the large parts of pod + deck, for easier assembly"; "an up arrow on the
forks"; "a notch or label where the motor's XT30 lines up"). Library only.

Wheel / pod numbers, seen from the top with the robot's front up: 1 front left, 2 front right, 3 back left, 4 back right (base.NAMES order is
FL, RL, RR, FR). A mark is engraved MARK_DEPTH into a flat face, reading correctly when the face is looked at from outside.

Mirrored pods: base.place3d (and sw_robot_parts' _m files) mirror a pod part about the pod's x-z plane AFTER it is built. A mark cut into the
drawn part would come out back to front, so for a mirrored pod the mark is built readable in the mirrored part's frame and mirrored back
before it is cut: the part, mirrored later, then carries readable text.
"""
import math

import numpy as np
import build123d as b
from shapely import union_all
from shapely.geometry import Polygon, box as sbox

NUMBER = dict(FL=1, FR=2, RL=3, RR=4)          # Fernando's wheel numbers (top view, front up)
MARK_DEPTH = 0.6                                # engraving depth (3 layers at 0.2)
MARK_MARGIN = 1.5                               # a mark keeps this far inside the flat face's material (holes, edges, rounds)
MIRROR = np.diag([1.0, -1.0, 1.0])              # the mirror hand: pod y -> -y


def _v(p):
    return np.asarray(p, float)


def x_dir(normal, up):
    """reading direction of text on a face with outward `normal`, glyphs pointing `up` (seen from outside)"""
    d = np.cross(_v(up), _v(normal))
    return d / np.linalg.norm(d)


def arrow_poly(w=4.0, h=8.0):
    """an arrow pointing +v in the plane's (u, v) coordinates, centred on the origin"""
    sw, hh = w * 0.38, h * 0.45
    return [(-sw / 2, -h / 2), (sw / 2, -h / 2), (sw / 2, h / 2 - hh), (w / 2, h / 2 - hh), (0, h / 2), (-w / 2, h / 2 - hh), (-sw / 2, h / 2 - hh)]


def _text_size(text, size):
    t = b.Text(text, font_size=size, align=(b.Align.CENTER, b.Align.CENTER))
    bb = t.bounding_box()
    return bb.max.X - bb.min.X, bb.max.Y - bb.min.Y


def layout(text, size, arrow=False, gap=1.5, arrow_ang=0.0):
    """[(kind, payload, du)] pieces of one mark along u, centred, and its (w, h). arrow_ang: the arrow turned this far (rad, counter-clockwise
    in (u, v)) from pointing up; an arrow pointing toward +u goes after the text, so the mark reads like "OUT ->" """
    tw, th = _text_size(text, size) if text else (0.0, 0.0)
    a0, a1 = size * 0.75, size * 1.35
    c, s = abs(math.cos(arrow_ang)), abs(math.sin(arrow_ang))
    aw, ah = ((a0 * c + a1 * s), (a0 * s + a1 * c)) if arrow else (0.0, 0.0)
    w = tw + aw + (gap if (text and arrow) else 0.0)
    h = max(th, ah)
    out, u = [], -w / 2
    after = arrow and -math.sin(arrow_ang) > 0.5                       # pointing +u
    if arrow and not after:
        out.append(('arrow', (a0, a1, arrow_ang), u + aw / 2))
        u += aw + gap
    if text:
        out.append(('text', text, u + tw / 2))
        u += tw + gap
    if after:
        out.append(('arrow', (a0, a1, arrow_ang), u + aw / 2))
    return out, (w, h)


def _turn(poly, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(c * x - s * y, s * x + c * y) for x, y in poly]


def _mark_solid(pieces, size, centre, normal, up, depth):
    """the cutter of one mark: its centre (3D) on the face, outward normal, glyph-up direction"""
    n, up = _v(normal) / np.linalg.norm(normal), _v(up) / np.linalg.norm(up)
    xd = x_dir(n, up)
    out = None
    for kind, payload, du in pieces:
        pl = b.Plane(origin=tuple(_v(centre) + n * 0.1 + xd * du), x_dir=tuple(xd), z_dir=tuple(n))
        if kind == 'text':
            sk = pl * b.Text(payload, font_size=size, align=(b.Align.CENTER, b.Align.CENTER))
        else:
            w, h = payload[:2]
            sk = pl * b.Polygon(*_turn(arrow_poly(w, h), payload[2] if len(payload) > 2 else 0.0), align=None)
        s = b.extrude(sk, amount=depth + 0.1, dir=tuple(-n))
        out = s if out is None else out + s
    return out


def face_region(part, origin, normal, up, depth=0.4):
    """the part's material just under a flat face (plan in the face's (u, v) coordinates: u = reading direction, v = up), as a shapely
    polygon: where a mark can go"""
    n, up = _v(normal) / np.linalg.norm(normal), _v(up) / np.linalg.norm(up)
    xd = x_dir(n, up)
    pl = b.Plane(origin=tuple(_v(origin) - n * (depth / 2 + 0.15)), x_dir=tuple(xd), z_dir=tuple(n))
    slab = pl * b.Box(600, 600, depth)
    sec = part & slab
    if sec is None or not sec.faces():
        return None
    v, t = sec.tessellate(0.05, 0.2)
    P = np.array([[q.X, q.Y, q.Z] for q in v]) - _v(origin)
    uv = np.c_[P @ xd, P @ up]
    tris = [q for q in (Polygon(uv[tri]) for tri in np.array(t)) if q.area > 1e-5]
    return union_all(tris).buffer(0.01)


def fit(region, w, h, prefer=(0.0, 0.0), margin=MARK_MARGIN, step=0.5, reach=80.0):
    """centre (u, v) nearest `prefer` where a w x h box fits inside region less margin; None if nowhere"""
    if region is None:
        return None
    ok = region.buffer(-margin)
    if ok.is_empty:
        return None
    best = None
    for r in np.arange(0.0, reach, step):
        n = max(1, int(2 * math.pi * r / step))
        for k in range(n):
            a = 2 * math.pi * k / n
            u, v = prefer[0] + r * math.cos(a), prefer[1] + r * math.sin(a)
            if ok.contains(sbox(u - w / 2, v - h / 2, u + w / 2, v + h / 2)):
                best = (u, v)
                break
        if best:
            return best
    return None


def engrave(part, marks, mirrored=False, region_cache=None, key=''):
    """cut marks into a part (in its drawn frame). marks: [dict(text, size, origin, normal, up, prefer=(u, v), arrow=False, arrow_to=None)],
    origin a point on the face; arrow_to (Nov 18): the 3D direction the arrow points, in the drawn frame (default: up). mirrored: the part will
    be mirrored about y later (see the module docstring); arrow_to is mirrored with it, so it points the same way in the real part. Returns
    (part, report)"""
    rep = []
    for j, m in enumerate(marks):
        ang = 0.0
        if m.get('arrow_to') is not None:                                # the arrow's angle in the frame the mark is cut in
            M = MIRROR if mirrored else np.eye(3)
            n2, up2, d2 = M @ _v(m['normal']), M @ _v(m['up']), M @ _v(m['arrow_to'])
            up2 = up2 / np.linalg.norm(up2)
            ang = math.atan2(float(d2 @ up2), float(d2 @ x_dir(n2, up2))) - math.pi / 2
        pieces, (w, h) = layout(m.get('text', ''), m['size'], m.get('arrow', False) or m.get('arrow_to') is not None, arrow_ang=ang)
        ck = (key, j)
        if region_cache is not None and ck in region_cache:
            reg = region_cache[ck]
        else:
            reg = face_region(part, m['origin'], m['normal'], m['up'])
            if region_cache is not None:
                region_cache[ck] = reg
        uv = fit(reg, w, h, m.get('prefer', (0.0, 0.0)))
        if uv is None:
            rep.append(dict(mark=m.get('text', 'arrow'), placed=False))
            continue
        n, up = _v(m['normal']), _v(m['up'])
        xd = x_dir(n, up)
        c3 = _v(m['origin']) + uv[0] * xd + uv[1] * up / np.linalg.norm(up)
        if mirrored:                                                    # the part mirrored, the readable mark cut, mirrored back (a mirrored
            pm = part.mirror(b.Plane.XZ) - _mark_solid(pieces, m['size'], MIRROR @ c3, MIRROR @ n, MIRROR @ up, MARK_DEPTH)   # cutter left the
            part = pm.mirror(b.Plane.XZ)                                # part invalid once it was mirrored again, Nov 15)
        else:
            part = part - _mark_solid(pieces, m['size'], c3, n, up, MARK_DEPTH)
        rep.append(dict(mark=m.get('text', 'arrow') + (' + arrow' if m.get('arrow') and m.get('text') else ''), placed=True,
                        at=[round(float(q), 1) for q in c3], size=[round(w, 1), round(h, 1)]))
    return part, rep
