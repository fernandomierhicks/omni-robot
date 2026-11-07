"""The last look before a grasp, through the wrist camera (design.md 11). No ROS.

The gripper hovers 8 cm over where the stereo pair put the object, fingers down; the wrist camera (on the back of the
fixed jaw, looking down the fingers) sees the object from above with the floor around it, and its own fingers in the
foreground. refine() finds the object again:
  0  the gripper's own pixels are masked out (self_mask: the gripper's and the jaw's boxes from robot_arm.json,
     projected with the jaw's angle; 4 px of margin)
  1  the stereo estimate projected into the image (the camera's pose from the arm's own joint angles: an error in the
     joints' calibration moves the camera and the fingers together, so it cancels in what this measures)
  2  the floor's colour from a ring around that spot (median and spread of the chromaticity r, g and the brightness),
     the object = the pixels whose chromaticity differs from it, or that are brighter (a shadow is the floor's colour,
     darker: not the object), the connected blob nearest the expected spot
  3  its pixels back onto the plane at half the object's height: seen at an angle, a box's silhouette there (its top
     face shifted one way, its bottom the other) is centred on the box; centre, long axis (yaw) and short side (width)
     in base_link
A floor with strong patterns of its own, an object the floor's colour, or a blob that doesn't fit what the stereo pair
saw (half to twice its size) returns None: the grasp then goes on the stereo estimate. A learned segmenter on the
robot computer can replace step 2 (VERIFY on real floors: rugs, wood grain, shadows of the arm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from omni_arm.floor_vision import Detection, _label


@dataclass
class WristFrame:
    t: float
    rgb: np.ndarray              # H x W x 3 uint8
    K: tuple                     # fx, fy, cx, cy
    T_base_cam: np.ndarray       # 4 x 4: the optical frame (z forward, x right, y down) in base_link
    mask: np.ndarray | None = None   # H x W bool: the robot's own pixels (self_mask)


def _hull(pts: np.ndarray) -> np.ndarray:
    """2D convex hull (Andrew's monotone chain), counter-clockwise."""
    p = sorted(map(tuple, pts))
    if len(p) < 3:
        return np.array(p)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for q in p:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in reversed(p):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], q) <= 0:
            hi.pop()
        hi.append(q)
    return np.array(lo[:-1] + hi[:-1])


def self_mask(shape, K, T_base_cam, boxes_base, grow: float = 4.0) -> np.ndarray:
    """Pixels covered by boxes (each 8 corners, base_link), grown by grow px."""
    H, W = shape
    m = np.zeros((H, W), bool)
    T = np.linalg.inv(T_base_cam)
    fx, fy, cx, cy = K
    near = 0.01
    for C in boxes_base:
        pc = C @ T[:3, :3].T + T[:3, 3]
        front = pc[:, 2] >= near
        if not front.any():
            continue
        pts = [pc[front]]
        if not front.all():                          # clip the box's edges at the near plane
            for a in range(8):
                for b in range(a + 1, 8):
                    if bin(a ^ b).count("1") == 1 and front[a] != front[b]:
                        s = (near - pc[a, 2]) / (pc[b, 2] - pc[a, 2])
                        pts.append((pc[a] + s * (pc[b] - pc[a]))[None])
        pc = np.vstack(pts)
        uv = np.stack([fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy], axis=1)
        h = _hull(uv)
        if len(h) < 3:
            continue
        u0, v0 = np.clip(np.floor(h.min(axis=0) - grow), 0, [W - 1, H - 1]).astype(int)
        u1, v1 = np.clip(np.ceil(h.max(axis=0) + grow), 0, [W - 1, H - 1]).astype(int)
        if u1 < u0 or v1 < v0:
            continue
        vv, uu = np.mgrid[v0:v1 + 1, u0:u1 + 1]
        inside = np.ones(uu.shape, bool)
        n = len(h)
        for k in range(n):                             # inside every edge, grown outward by grow px
            a, b = h[k], h[(k + 1) % n]
            e = b - a
            L = math.hypot(*e) or 1.0
            inside &= ((e[0] * (vv - a[1]) - e[1] * (uu - a[0])) / L) >= -grow
        m[v0:v1 + 1, u0:u1 + 1] |= inside
    return m


def box_corners(T, lo, hi) -> np.ndarray:
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return c @ T[:3, :3].T + T[:3, 3]


def project(frame: WristFrame, P) -> np.ndarray:
    T = np.linalg.inv(frame.T_base_cam)
    pc = np.asarray(P, float) @ T[:3, :3].T + T[:3, 3]
    fx, fy, cx, cy = frame.K
    return np.stack([fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy, pc[:, 2]], axis=1)


def to_plane(frame: WristFrame, u, v, z: float) -> np.ndarray:
    """Pixels (u, v) onto the horizontal plane at height z (base_link)."""
    fx, fy, cx, cy = frame.K
    rays = np.stack([(np.asarray(u) - cx) / fx, (np.asarray(v) - cy) / fy, np.ones(len(u))], axis=1)
    T = frame.T_base_cam
    d = rays @ T[:3, :3].T
    o = T[:3, 3]
    s = (z - o[2]) / d[:, 2]
    return o + d * s[:, None]


def _open(m: np.ndarray, r: int) -> np.ndarray:
    """Binary opening with a (2r + 1) square: erode, then dilate."""
    try:
        from scipy.ndimage import binary_opening
        return binary_opening(m, structure=np.ones((2 * r + 1, 2 * r + 1), bool))
    except ImportError:
        def shift_all(a, op):
            out = a.copy()
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    s = np.roll(np.roll(a, dy, 0), dx, 1)
                    out = op(out, s)
            return out
        return shift_all(shift_all(m, np.logical_and), np.logical_or)


def _chroma(img: np.ndarray) -> np.ndarray:
    """(r, g, brightness) per pixel: r, g = the channels over their sum (x 255), brightness = the sum / 3."""
    s = img.sum(axis=2) + 1.0
    return np.stack([255 * img[..., 0] / s, 255 * img[..., 1] / s, s / 3], axis=2)


def square_axis(xy: np.ndarray, step_deg: float = 1.0) -> np.ndarray:
    """The unit axis (within 0-90 deg) whose box round the points has the least area: a square's or a rectangle's
    face direction, where the covariance can't tell (a square's is round)."""
    best, ba = np.array([1.0, 0.0]), math.inf
    for th in np.radians(np.arange(0.0, 90.0, step_deg)):
        a = np.array([math.cos(th), math.sin(th)])
        b = np.array([-a[1], a[0]])
        pa, pb = xy @ a, xy @ b
        area = (np.percentile(pa, 98) - np.percentile(pa, 2)) * (np.percentile(pb, 98) - np.percentile(pb, 2))
        if area < ba:
            best, ba = a, area
    return best


def refine(frame: WristFrame, guess: Detection, min_contrast: float = 8.0, debug: dict | None = None):
    """A better Detection (base_link) of the object near guess, or None (see the module notes)."""
    img = _chroma(frame.rgb.astype(np.float32))
    H, W = img.shape[:2]
    c = project(frame, [[guess.x, guess.y, guess.top]])[0]
    if c[2] <= 0:
        return None
    fx = frame.K[0]
    r_obj = fx * 0.5 * max(guess.length, guess.width, 0.02) / c[2]          # px
    yy, xx = np.mgrid[0:H, 0:W]
    dist = np.hypot(xx - c[0], yy - c[1])
    own = frame.mask if frame.mask is not None else np.zeros((H, W), bool)
    ring = (dist > 2.2 * r_obj + 6) & (dist < 3.2 * r_obj + 16) & ~own
    if ring.sum() < 200:
        return None
    floor = img[ring]
    med = np.median(floor, axis=0)
    spread = np.median(np.abs(floor - med), axis=0) * 1.4826 + 1.0
    dc = np.abs(img[..., :2] - med[:2])
    chroma = (np.max(dc / spread[:2], axis=2) > 4.0) & (np.max(dc, axis=2) > min_contrast)
    brighter = (img[..., 2] - med[2]) > max(4.0 * spread[2], 25.0)
    mask = (chroma | brighter) & (dist < 2.2 * r_obj + 6) & ~own
    mask = _open(mask, 2)                         # thin lines (a floor's seams, a cable's shadow) out
    lab, n = _label(mask)
    if n == 0:
        return None
    best, bd = 0, math.inf
    for k in range(1, n + 1):
        sel = lab == k
        npx = int(sel.sum())
        if npx < 30:
            continue
        cy_, cx_ = np.mean(yy[sel]), np.mean(xx[sel])
        dd = math.hypot(cx_ - c[0], cy_ - c[1])
        if dd < bd:
            best, bd = k, dd
    if best == 0:
        return None
    sel = lab == best
    P = to_plane(frame, xx[sel].ravel(), yy[sel].ravel(), guess.top / 2)
    xy = P[:, :2]
    m = xy.mean(axis=0)
    w, V = np.linalg.eigh(np.cov((xy - m).T))
    ax = V[:, 1]
    perp = np.array([-ax[1], ax[0]])
    along, across = (xy - m) @ ax, (xy - m) @ perp
    a0, a1 = np.percentile(along, 2), np.percentile(along, 98)
    b0, b1 = np.percentile(across, 2), np.percentile(across, 98)
    L, Wd = float(a1 - a0), float(b1 - b0)
    if L - Wd < 0.25 * max(L, Wd):
        # near square: its covariance is round, so the eigenvectors say nothing about which way its faces point (a
        # tower's block must be closed across its faces). The rotation whose box round the points is smallest does
        ax = square_axis(xy - m)
        perp = np.array([-ax[1], ax[0]])
        along, across = (xy - m) @ ax, (xy - m) @ perp
        a0, a1 = np.percentile(along, 2), np.percentile(along, 98)
        b0, b1 = np.percentile(across, 2), np.percentile(across, 98)
        L, Wd = float(a1 - a0), float(b1 - b0)
    ctr = m + ax * (a0 + a1) / 2 + perp * (b0 + b1) / 2
    if debug is not None:
        debug.update(mask=sel, centre_px=c[:2], r_px=r_obj)
    size_g = max(guess.length, guess.width)
    if not (0.5 * size_g < max(L, Wd) < 2.0 * size_g + 0.01):
        return None
    if math.hypot(ctr[0] - guess.x, ctr[1] - guess.y) > 0.04:
        return None
    return Detection(float(ctr[0]), float(ctr[1]), guess.top, max(L, Wd), min(L, Wd),
                     float(math.atan2(ax[1], ax[0])) if L >= Wd else float(math.atan2(perp[1], perp[0])), int(sel.sum()))


@dataclass
class HeldBlock:
    """Where a held block sits in the gripper, in the tool frame (x along the fingers, z toward the moving jaw, y
    across the pads): its centre's z and y, its turn about x (skew: rad, + from z toward y), and its top face's x."""
    zc: float
    yc: float
    skew: float
    n: int
    top_x: float = 0.0


def _ray_box(o: np.ndarray, d: np.ndarray, lo, hi) -> np.ndarray:
    """Which rays (origin o, directions d: N x 3) pass through the axis-aligned box lo-hi in front of their origin."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (np.asarray(lo) - o) / d, (np.asarray(hi) - o) / d
    tn = np.nanmax(np.minimum(t1, t2), axis=1)
    tf = np.nanmin(np.maximum(t1, t2), axis=1)
    return (tn <= tf) & (tf > 0)


def held(frame: WristFrame, T_base_tool: np.ndarray, side: float, gap: float, top_x: float,
         min_contrast: float = 8.0, debug: dict | None = None) -> HeldBlock | None:
    """The block the gripper holds, seen from its own wrist camera (the arm still, the block lifted clear of the floor).

    The camera rides on the gripper, so what this measures doesn't depend on the joints' calibration. It sits beside the
    fingers (tool +y), just above the block's top face (about 1 cm for a 40 mm cube): it sees the block's near side and
    hardly its top, with the floor behind. Expected: the block (side x side, its top face at tool x = top_x) between the
    fixed finger's face (z = 0) and the jaw (z = gap). Its pixels are those whose colour differs from the floor round it
    (as refine). Back onto the plane of its bottom face, every ray that hit it lands in its bottom face or beyond it,
    away from the camera: the edge nearest the camera there (tool +y) is the bottom face's own. A line through that edge
    across the middle of the jaw's span (the fingers hide its ends) gives the skew and, with the side known, the centre
    across the pads, for each height it may sit at in the fingers (from top_x, 6 mm up to 4 mm down): the one whose
    block's silhouette best overlaps the pixels is taken; along the closing line the centre is half the side from the fixed finger (pressed against it; the
    jaw's gap reads wider than the block by the pads' give). None: no floor round it to compare, the block the floor's
    colour, or no clean edge. VERIFY on the robot: the real wrist camera (640 x 480, 60 deg), a block of each colour."""
    img = _chroma(frame.rgb.astype(np.float32))
    H, W = img.shape[:2]
    T_cam = np.linalg.inv(T_base_tool) @ frame.T_base_cam               # the optical frame in the tool frame
    own = frame.mask if frame.mask is not None else np.zeros((H, W), bool)
    fx, fy, cx, cy = frame.K
    vv, uu = np.mgrid[0:H, 0:W]
    rays = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones((H, W))], axis=-1).reshape(-1, 3) @ T_cam[:3, :3].T
    o = T_cam[:3, 3]
    # where the block should be: each pixel's ray through its box between the fingers (a little grown: it may sit off
    # centre or turned), and the floor: the rays that miss a box grown well beyond it
    lo, hi = np.array([top_x, -0.5 * side, 0.0]), np.array([top_x + side, 0.5 * side, gap])

    def through(m):
        return _ray_box(o, rays, lo - m, hi + m).reshape(H, W)
    inner, near = through(np.zeros(3)), through(np.array([0.005, 0.012, 0.005]))
    ring = ~through(np.array([0.02, 0.03, 0.02])) & ~own
    if ring.sum() < 500 or inner.sum() < 150:
        return None
    floor = img[ring]
    med = np.median(floor, axis=0)
    spread = np.median(np.abs(floor - med), axis=0) * 1.4826 + 1.0
    dc = np.abs(img[..., :2] - med[:2])
    chroma = (np.max(dc / spread[:2], axis=2) > 4.0) & (np.max(dc, axis=2) > min_contrast)
    brighter = (img[..., 2] - med[2]) > max(4.0 * spread[2], 25.0)
    mask = _open((chroma | brighter) & near & ~own, 1)
    lab, n = _label(mask)
    if n == 0:
        return None
    k = max(range(1, n + 1), key=lambda j: int((inner & (lab == j)).sum()))
    sel = lab == k
    if sel.sum() < 150:
        return None
    r_sel = rays[sel.ravel()]
    span = min(gap, side)
    region = (near | sel) & ~own

    def fit(tx):
        """The near edge on the bottom face's plane for the top face at tool x = tx: (yc, skew, edges) or None."""
        s_ = (tx + side - o[0]) / r_sel[:, 0]
        P = o + r_sel * s_[:, None]                                    # on the bottom face's plane (tool frame)
        zs, ys = P[s_ > 0, 2], P[s_ > 0, 1]
        edges = []                                     # the highest y in each slice across the middle of the block
        for z0 in np.linspace(0.15 * span, 0.85 * span, 15):
            m = np.abs(zs - z0) < 0.025 * span
            if m.sum() >= 4:
                edges.append((z0, float(np.percentile(ys[m], 99))))
        if len(edges) < 6:
            return None
        E = np.array(edges)
        for _ in range(6):                # a line; the slices a finger cuts into (they fall short) out, one at a time
            kk, b0 = np.polyfit(E[:, 0], E[:, 1], 1)
            res = E[:, 1] - (kk * E[:, 0] + b0)
            if len(E) <= 6 or np.abs(res).max() < 0.0005:
                break
            E = np.delete(E, int(np.argmax(np.abs(res))), axis=0)
        kk, b0 = np.polyfit(E[:, 0], E[:, 1], 1)
        sk = math.atan(kk)
        if abs(sk) > math.radians(30) or np.abs(E[:, 1] - (kk * E[:, 0] + b0)).max() > 0.004:
            return None
        return b0 + kk * span / 2 - 0.5 * side / math.cos(sk), sk, E

    def z_of(sk):
        """Its centre along the closing line: pressed against the fixed finger (z = 0), a corner first when turned."""
        return 0.5 * side * (math.cos(sk) + abs(math.sin(sk)))

    def overlap(tx, yc, sk):
        """How well a block there explains the pixels: intersection over union of its silhouette with them."""
        c, s_ = math.cos(sk), math.sin(sk)
        R = np.array([[1, 0, 0], [0, c, s_], [0, -s_, c]])            # its axes in the tool frame (z turned to y)
        ctr = np.array([tx + side / 2, yc, z_of(sk)])
        pred = _ray_box((o - ctr) @ R, rays @ R, -0.5 * side * np.ones(3), 0.5 * side * np.ones(3)).reshape(H, W)
        a, b = pred & region, sel & region
        return (a & b).sum() / max(1, (a | b).sum())

    # how high it sits in the fingers: gripping can push it up a few mm (2.5 in the sim): each height's edge, the one
    # whose block best explains the pixels
    # whose block best explains the pixels. The edge is fitted once, at top_x: on another plane parallel to it, the
    # camera's rays put the same edge at the same slope, its points scaled from the camera's foot
    f = fit(top_x)
    if f is None:
        return None
    E0 = f[2]
    h0 = top_x + side - o[0]
    def candidate(tx, dsk=0.0):
        """The block whose near edge is the one seen, its top face at tx, turned dsk more than the edge's slope says
        (the slope is the least sure: 1 deg is a third of a pixel over the edge): (score, tx, yc, skew, edge)."""
        k = (tx + side - o[0]) / h0
        E = np.column_stack([o[2] + (E0[:, 0] - o[2]) * k, o[1] + (E0[:, 1] - o[1]) * k])
        kk, b0 = np.polyfit(E[:, 0], E[:, 1], 1)
        sk = math.atan(kk) + dsk
        yc_ = b0 + kk * z_of(sk) - 0.5 * side / math.cos(sk)           # through the edge's middle
        return overlap(tx, yc_, sk), tx, yc_, sk, E

    best = max((candidate(tx) for tx in top_x + np.arange(-0.006, 0.00405, 0.0005)), key=lambda c: c[0])
    best = max([best] + [candidate(best[1] + dt, math.radians(dd)) for dt in (-0.0005, 0.0, 0.0005)
                         for dd in (-3, -2, -1, 1, 2, 3)], key=lambda c: c[0])
    if best[0] < 0.6:
        return None
    score, tx, yc, skew, E = best
    if abs(yc) > side:                                                 # half off the pads: not what it measured
        return None
    if debug is not None:
        debug.update(mask=sel, edges=E, iou=score)
    return HeldBlock(float(z_of(skew)), float(yc), float(skew), int(sel.sum()), float(tx))
