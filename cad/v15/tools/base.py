"""v15 layout: where the four pods sit (layout F) and the plan / solid helpers every v15 script uses to place them. Library only.

Robot frame: origin on the floor under the centre, x forward, y left, z up; pod frame z + R_WHEEL = robot z.
 * X4 as the sim (robot_export.ARM): rolling lines tangent to r D_WHEEL at 45 / 135 / 225 / 315 deg, axles radial, motors inboard.
 * Opposite hands on opposite diagonals as the sim (pod_hands 'diagonal'): the drawn pod (pivot on hub +y) at 135 and 315 deg, its mirror
   image at 45 and 225. Every pivot points at the left or right side midline, so the pods meet in pairs there, pivot end to pivot end.
 * Each wheel slides SLIDE mm along its own rolling line, away from its pivot (toward the front / back). The rolling lines do not move, so
   the X4 kinematics (firmware, ROS) are unchanged.
Layout F (cad/v12 on; D 154, slide -20) is this module's own default since the Nov 14 cleanup (it used to be set by importing first_floor.py).
(From cad/v11 base.py; its round-plate study is in cad/v14.)
"""
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import MultiPoint, Polygon, box as sbox
from shapely.geometry.polygon import orient

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL                                               # noqa: E402

# ---- layout ----------------------------------------------------------------------------------------------------------------------------
D_WHEEL = 154.0                    # layout F: the rolling lines are tangent to this circle
ANGLES = (45.0, 135.0, 225.0, 315.0)
NAMES = ('FL', 'RL', 'RR', 'FR')
SLIDE = -20.0                      # layout F: mm along each rolling line, away from the pivot

def mirrored(i):
    """sim robot_model.fork_kinematics: the 45 and 225 deg pods are the mirror image (pivot on hub -y)"""
    return round(ANGLES[i]) % 180 == 45


def frame(i, slide=None):
    """pod frame -> robot frame: (axle xy, hub x (inward), hub y), and the mirror flag"""
    th = math.radians(ANGLES[i])
    c, s = math.cos(th), math.sin(th)
    sgn = -1.0 if mirrored(i) else 1.0
    sl = SLIDE if slide is None else slide
    A = np.array([D_WHEEL * c + sl * sgn * s, D_WHEEL * s - sl * sgn * c])
    return A, np.array([-c, -s]), np.array([s, -c]), mirrored(i)


def place2d(g, i, slide=None):
    A, xh, yh, m = frame(i, slide)
    if m:
        g = affinity.scale(g, 1, -1, origin=(0, 0))
    return affinity.affine_transform(g, [xh[0], yh[0], xh[1], yh[1], A[0], A[1]])


def place3d(shape, i, slide=None):
    A, xh, yh, m = frame(i, slide)
    if m:
        shape = shape.mirror(b.Plane.XZ)
    M = np.array([[xh[0], yh[0], 0, A[0]], [xh[1], yh[1], 0, A[1]], [0, 0, 1, R_WHEEL]])
    return shape.moved(W.loc(M))


# ---- plan projections (pod frame) ----------------------------------------------------------------------------------------------------
def proj(shape, exact, z0=None, z1=None, tol=0.4):
    s = shape if z0 is None else shape & P.box(-400, 400, -400, 400, z0, z1)
    if s is None or not s.faces():
        return None
    try:
        v, t = s.tessellate(tol, 0.3)
    except AttributeError:                       # a sliver the mesher cannot do: its bounding box (conservative)
        bb = s.bounding_box()
        return sbox(bb.min.X, bb.min.Y, bb.max.X, bb.max.Y)
    if not v:
        return None
    pts = np.array([[q.X, q.Y] for q in v])
    if not exact:
        h = MultiPoint(pts).convex_hull
        return h if h.area > 1e-3 else None
    tris = [q for q in (Polygon(pts[tri]) for tri in np.array(t)) if q.area > 1e-4]
    try:
        return union_all(tris).buffer(0.02)
    except Exception:                            # noqa: BLE001  GEOS TopologyException on near-degenerate triangles (Nov 13 evening: a
        return union_all(tris, grid_size=1e-4).buffer(0.02)   # roller sleeve's slab sliver): snap to a 0.1 um grid, well under the buffer


def face_of(poly):
    """shapely polygon (with holes) -> build123d face"""
    def wire(coords):
        pts = [b.Vector(x, y, 0) for x, y in list(coords)[:-1]]
        return b.Wire.make_polygon(pts, close=True)
    f = b.Face(wire(poly.exterior.coords), [wire(h.coords) for h in poly.interiors])
    return f


def solid_of(geom, z0, z1):
    out = []
    for g in getattr(geom, 'geoms', [geom]):
        if g.area > 1.0:
            g = orient(g.simplify(0.05), 1.0)        # counter-clockwise: the face looks up and extrudes z0 -> z1 (shapely's buffers are clockwise)
            s = b.extrude(face_of(g), amount=z1 - z0)
            bb = s.bounding_box()
            out.append(s.moved(b.Location((0, 0, z0 - bb.min.Z))))
    s = out[0]
    for o in out[1:]:
        s = s + o
    return s


def prism(geom, z0, z1):
    """a plan outline extruded z0..z1, opened by 0.05 first (a valid shapely outline can make an invalid solid after clipping)"""
    return solid_of(geom.buffer(-0.05, join_style=2).buffer(0.05, join_style=2), z0, z1)
