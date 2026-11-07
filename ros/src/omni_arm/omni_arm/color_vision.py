"""Colour and shape for the floor's candidates (ai_robot_architecture.md 5.2: "go get the purple square object"). No
ROS.

The depth detector (floor_vision.detect) finds small things on the floor with their footprint, height and position,
tracked over looks with ids (FloorMap, odom frame). find() labels each with a colour image registered to the depth
image (the same camera, the same pixels: the sim renders its depth camera in colour too; the D435 aligns its depth to
its colour stream):
  1  the candidate's pixels: the depth pixels whose point lies in its footprint (grown 6 mm) and above the floor plane
  2  colour: their hue, saturation and value (OpenCV's scale, H 0-179), the fraction inside the asked colour (COLORS,
     or {h: [lo, hi], s_min, v_min}), and the colour most of them have
  3  shape, from the candidate's points. Depth smears a footprint along the view (a 40 mm cube's comes out 1.1-1.85
     long / short) and the camera sees only a ball's front (its footprint lopsided), so the footprint's sides alone
     can't tell; with them: the longest side over the height (l/h), the flat top (the share of its 5 mm top-down cells
     whose highest point is near its top: a box's top face fills them, a ball's crest doesn't) and the width profile
     (max / min of the top-down points' width over 36 directions: a square's 1.41, a can's ~1.0, whatever way the
     footprint's axes came out):
       long    l/h >= 1.8 and long / short >= 1.6                       (a block, a crayon, a brick)
       round   flat top < 0.75 and l/h <= 1.15                          (a ball: rounded, as tall as wide)
       square  l/h > 1.15 and long / short < 1.75, or flat top >= 0.75 and the profile >= 1.12   (a box)
       round   flat top >= 0.85, the profile < 1.1, long / short <= 1.15   (a can standing; sure from 500 px, and
               maybe a square below that: a far cube looks like a can)
     A shape is sure from 300 pixels (est; ~0.85 m for a 40 mm cube in the D435's 480 x 270; a crayon's long from 60):
     below that it is a guess and a match scores under 0.5, and the result's hint says to move closer and find again.
     (thresholds est: tuned on the sim's "colours" set over 10 layouts per camera, studies/colour_scene.py; VERIFY on
     real objects and real depth)
  4  score = the colour fraction x (0.2 + 0.8 x the shape: 1 matched and sure, 0.35 matched as a guess, 0 not):
     near misses stay listed, ranked lower
Pixels of the asked colour that no candidate holds (too far, cut off by the image's edge, up on furniture) come back
as image_only blobs with their bearing: somewhere to turn to or drive closer to, never something to pick. A blob the
depth shows lying flat on the floor (flat: true, under FLAT_H) is a rug, a mat or a marking, not an object to seek.
The annotated image: the colour image, the asked colour's pixels tinted cyan, each candidate boxed (green: the best
match, yellow: the rest) with its id and score, the image-only blobs boxed red with "?".
Finder keeps one find at a time for arm_brain: request(), then step() with the newest frames.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from omni_arm import floor_vision as fv

# name: (hue ranges, OpenCV H 0-179 inclusive; s_min; v_min). Estimates: tune on renders and real captures.
COLORS = {
    "red": (((0, 8), (170, 179)), 80, 50),
    "orange": (((9, 20),), 80, 50),
    "yellow": (((21, 34),), 80, 50),
    "green": (((35, 85),), 60, 40),
    "blue": (((86, 125),), 60, 40),
    "purple": (((126, 160),), 60, 40),
    "pink": (((161, 169),), 60, 120),
}
GREY_S = 40                         # below this saturation a pixel has no hue: white, grey or black by its value
WHITE_V, BLACK_V = 180, 50
ALIASES = {"violet": "purple", "magenta": "pink", "gray": "grey", "cyan": "blue", "lime": "green"}
NAMES = tuple(COLORS) + ("white", "grey", "black")
SHAPES = ("square", "round", "long", "any")
GROW = 0.006                        # m: the footprint grown by this to take a candidate's pixels
MIN_PIXELS = 15                     # fewer: no colour for it
BLOB_MIN = 30                       # px: the smallest image-only blob reported
# m: an image-only blob whose depth stays this low (its 90th percentile) is flat on the floor: a rug, a mat, a marking
# (the sim house's rugs are 6-16 mm; a 30 mm cube's top pixels reach ~30). The D435 at 7.5 deg sees the red
# living-room rug across the room, and a find chased it for 12 looks without finding the red cube (Nov 5)
FLAT_H = 0.018
ROBOT_R = 0.25                      # m: pixels nearer the base's centre than this are the robot (its bumper is orange)
N_DIRS = 36


def hsv(rgb: np.ndarray):
    """(H 0-180, S 0-255, V 0-255) float32 arrays of an RGB uint8 image (OpenCV's convention)."""
    c = rgb.astype(np.float32)
    r, g, b = c[..., 0], c[..., 1], c[..., 2]
    mx, mn = c.max(axis=-1), c.min(axis=-1)
    d = mx - mn
    dd = np.where(d > 0, d, 1.0)
    h = np.where(mx == r, (60.0 * (g - b) / dd) % 360.0,
                 np.where(mx == g, 60.0 * (b - r) / dd + 120.0, 60.0 * (r - g) / dd + 240.0))
    h = np.where(d > 0, h, 0.0) / 2.0
    s = np.where(mx > 0, 255.0 * d / np.where(mx > 0, mx, 1.0), 0.0)
    return h.astype(np.float32), s.astype(np.float32), mx.astype(np.float32)


def color_spec(color) -> dict | None:
    """A colour asked for -> {name, ranges, s_min, v_min} (achromatic: {name, achromatic}); None = any colour.
    ValueError for one it doesn't know."""
    if color is None or (isinstance(color, str) and color.strip().lower() in ("", "any")):
        return None
    if isinstance(color, dict):
        lo, hi = (float(v) for v in color["h"])
        ranges = ((lo, hi),) if lo <= hi else ((lo, 179.0), (0.0, hi))          # hue wraps round past 179
        return dict(name=f"h {lo:g}-{hi:g}", ranges=ranges, s_min=float(color.get("s_min", 60)),
                    v_min=float(color.get("v_min", 40)))
    name = ALIASES.get(str(color).strip().lower(), str(color).strip().lower())
    if name in ("white", "grey", "black"):
        return dict(name=name, achromatic=True)
    if name not in COLORS:
        raise ValueError(f"colour '{color}': one of {', '.join(NAMES)}, or {{h: [lo, hi], s_min, v_min}}")
    ranges, s_min, v_min = COLORS[name]
    return dict(name=name, ranges=ranges, s_min=s_min, v_min=v_min)


def in_color(h, s, v, spec: dict) -> np.ndarray:
    if spec.get("achromatic"):
        n = spec["name"]
        grey = s < GREY_S
        return {"white": grey & (v >= WHITE_V), "grey": grey & (v >= BLACK_V) & (v < WHITE_V),
                "black": v < BLACK_V}[n]
    hue = np.zeros(h.shape, bool)
    for lo, hi in spec["ranges"]:
        hue |= (h >= lo - 0.5) & (h < hi + 0.5)
    return hue & (s >= spec["s_min"]) & (v >= spec["v_min"])


def dominant(h, s, v) -> tuple:
    """(name, fraction) of the colour most of these pixels have."""
    if len(h) == 0:
        return None, 0.0
    best = (None, 0.0)
    for n in NAMES:
        f = float(np.mean(in_color(h, s, v, color_spec(n))))
        if f > best[1]:
            best = (n, f)
    return best


def _hue_median(h: np.ndarray) -> float:
    """The circular median of hues (0-180 wraps): red's 178 and 2 are 4 apart, not 176."""
    a = h.astype(np.float64) * (2 * math.pi / 180.0)
    m = math.atan2(float(np.mean(np.sin(a))), float(np.mean(np.cos(a))))
    return (math.degrees(m) / 2.0) % 180.0


# -- shape ---------------------------------------------------------------------------------------------------------
def width_profile(xy: np.ndarray) -> tuple:
    """(min width, max width) of top-down points over N_DIRS directions (3rd to 97th percentile)."""
    th = np.arange(N_DIRS) * (math.pi / N_DIRS)
    proj = xy @ np.stack([np.cos(th), np.sin(th)])                      # n x N_DIRS
    w = np.percentile(proj, 97, axis=0) - np.percentile(proj, 3, axis=0)
    return float(w.min()), float(w.max())


SURE_PX, SURE_CAN_PX = 300, 500     # pixels for a sure shape (est)


def shape_of(det: fv.Detection, xy: np.ndarray | None, heights: np.ndarray | None = None) -> dict:
    lw = det.length / max(det.width, 1e-3)
    prof = flat = None
    if xy is not None and len(xy) >= MIN_PIXELS:
        wmin, wmax = width_profile(xy)
        prof = wmax / max(wmin, 1e-3)
    if heights is not None and xy is not None and len(heights) >= MIN_PIXELS:
        # top-down cells of 5 mm: the share whose highest point is near the top (a top face covers them all)
        cell = np.floor((xy - xy.min(axis=0)) / 0.005).astype(int)
        key = cell[:, 0] * 100_000 + cell[:, 1]
        order = np.argsort(key)
        k, h = key[order], heights[order]
        starts = np.r_[0, np.nonzero(np.diff(k))[0] + 1]
        cmax = np.maximum.reduceat(h, starts)
        flat = float(np.mean(cmax >= det.top - max(0.005, 0.15 * det.top)))
        n_cells = len(starts)
    else:
        n_cells = 0
    tall = det.length / max(det.top, 1e-3)                             # longest side over height
    npx = 0 if heights is None else len(heights)
    sure = npx >= SURE_PX
    cls, maybe = "other", []
    if tall >= 1.8 and lw >= 1.6:
        cls = "long"
        sure = sure or (tall >= 3.0 and lw >= 3.0 and npx >= 60)       # a crayon: long whatever the range
    elif flat is not None and flat < 0.75 and tall <= 1.15:
        cls = "round"                                                   # a ball
    elif flat is not None and flat >= 0.85 and prof is not None and prof < 1.1 and lw <= 1.15:
        cls, sure = "round", npx >= SURE_CAN_PX                         # a can standing (a far cube looks like one)
        maybe = [] if sure else ["square"]
    elif lw < 1.75 and tall > 1.15:
        cls = "square"
    elif flat is not None and flat >= 0.75 and prof is not None and prof >= 1.12:
        cls = "square"
    if not sure and cls in ("round", "square"):                         # too few pixels to tell a far cube from a ball
        maybe = [{"round": "square", "square": "round"}[cls]]
    return {"class": cls, "sure": bool(sure and cls != "other"), "maybe": maybe, "long_short": round(lw, 2),
            "profile": None if prof is None else round(prof, 2), "flat_top": None if flat is None else round(flat, 2),
            "long_height": round(tall, 2), "cells": n_cells}


def shape_match(shape: dict, want: str) -> float:
    """1 matched and sure, 0.35 matched as a guess (seen from too far: look closer), 0 not."""
    if want == "any":
        return 1.0
    if want in shape.get("maybe", ()):
        return 0.35
    if shape["class"] != want:
        return 0.0
    return 1.0 if shape["sure"] else 0.35


# -- the image's pixels in base_link ---------------------------------------------------------------------------------
def pixel_points(frame: fv.DepthFrame):
    """(H x W x 3 points in base_link, H x W valid)."""
    z = frame.depth
    H, W = z.shape
    fx, fy, cx, cy = frame.K
    v, u = np.mgrid[0:H, 0:W]
    ok = np.isfinite(z) & (z > 0.05) & (z < fv.RANGE)
    zz = np.where(ok, z, 0.0)
    pc = np.stack([(u - cx) / fx * zz, (v - cy) / fy * zz, zz], axis=-1)
    T = frame.T_base_cam
    return pc @ T[:3, :3].T + T[:3, 3], ok


def _resize_nearest(img: np.ndarray, hw: tuple) -> np.ndarray:
    H, W = hw
    if img.shape[:2] == (H, W):
        return img
    ys = (np.arange(H) * img.shape[0] / H).astype(int)
    xs = (np.arange(W) * img.shape[1] / W).astype(int)
    return img[ys][:, xs]


# -- the annotated image ---------------------------------------------------------------------------------------------
FONT = {"0": "111101101101111", "1": "010110010010111", "2": "111001111100111", "3": "111001111001111",
        "4": "101101111001001", "5": "111100111001111", "6": "111100111101111", "7": "111001001001001",
        "8": "111101111101111", "9": "111101111001111", ".": "000000000000010", "?": "111001011000010",
        " ": "000000000000000", "-": "000000111000000"}


def draw_text(img: np.ndarray, x: int, y: int, text: str, color, scale: int = 2):
    """A 3 x 5 pixel font (digits, '.', '?', '-'), on a dark box so it reads on any floor."""
    H, W = img.shape[:2]
    w, h = (4 * len(text) + 1) * scale, 7 * scale
    x, y = max(0, min(W - w, x)), max(0, min(H - h, y))
    img[y:y + h, x:x + w] = (img[y:y + h, x:x + w] * 0.25).astype(img.dtype)
    for k, ch in enumerate(text):
        bits = FONT.get(ch, FONT["?"])
        for r in range(5):
            for c in range(3):
                if bits[3 * r + c] == "1":
                    yy, xx = y + (r + 1) * scale, x + (4 * k + c + 1) * scale
                    img[yy:yy + scale, xx:xx + scale] = color


def draw_box(img: np.ndarray, u0, v0, u1, v1, color, t: int = 2):
    H, W = img.shape[:2]
    u0, u1 = max(0, int(u0) - 2), min(W - 1, int(u1) + 2)
    v0, v1 = max(0, int(v0) - 2), min(H - 1, int(v1) + 2)
    img[v0:v0 + t, u0:u1 + 1] = color
    img[max(0, v1 - t + 1):v1 + 1, u0:u1 + 1] = color
    img[v0:v1 + 1, u0:u0 + t] = color
    img[v0:v1 + 1, max(0, u1 - t + 1):u1 + 1] = color


# -- find --------------------------------------------------------------------------------------------------------------
@dataclass
class ColorFrame:
    t: float
    rgb: np.ndarray                 # H x W x 3 uint8, registered to the depth frame (the same K and pose)


def find(frame: fv.DepthFrame, rgb: np.ndarray, pose, fmap: fv.FloorMap, t: float, color=None, shape: str = "any",
         max_range: float = 1.6, rng: np.random.Generator | None = None):
    """(result, annotated image) for one look (see the module notes). pose: the base's (x, y, yaw) in odom when the
    frames were taken; fmap keeps the ids across finds (and the pick's look after)."""
    spec = color_spec(color)
    if shape not in SHAPES:
        raise ValueError(f"shape: one of {', '.join(SHAPES)}")
    rng = rng or np.random.default_rng(int(t * 1000) % 2 ** 31)
    rgb = _resize_nearest(np.asarray(rgb, np.uint8), frame.depth.shape)
    H, W = frame.depth.shape
    P, ok = pixel_points(frame)
    n, d = fv._floor_plane(P[ok], rng)
    hgt = P @ n + d
    hh, ss, vv = hsv(rgb)
    want = in_color(hh, ss, vv, spec) if spec is not None else np.zeros((H, W), bool)
    dets_all = fv.detect(frame, rng=rng)
    dets = [o for o in dets_all if math.hypot(o.x, o.y) < max_range and not o.big]
    fmap.update(t, fv.to_frame(dets, pose))
    taken = np.zeros((H, W), bool)
    cands = []
    for o, od in zip(dets, fv.to_frame(dets, pose)):
        tr = min((x for x in fmap.tracked if x.status not in ("picked", "gone")),
                 key=lambda x: math.hypot(x.det.x - od.x, x.det.y - od.y), default=None)
        if tr is None or math.hypot(tr.det.x - od.x, tr.det.y - od.y) > fmap.merge:
            continue
        c, s_ = math.cos(o.yaw), math.sin(o.yaw)
        dx, dy = P[..., 0] - o.x, P[..., 1] - o.y
        along, across = dx * c + dy * s_, -dx * s_ + dy * c
        m = ok & (np.abs(along) <= o.length / 2 + GROW) & (np.abs(across) <= o.width / 2 + GROW) & \
            (hgt > 0.004) & (hgt < o.top + 0.015)
        npx = int(m.sum())
        taken |= m
        xy = np.stack([P[..., 0][m], P[..., 1][m]], axis=1) if npx else None
        sh = shape_of(o, xy, hgt[m] if npx else None)
        if npx >= MIN_PIXELS:
            ph, ps, pv = hh[m], ss[m], vv[m]
            name, frac = dominant(ph, ps, pv)
            match = float(np.mean(want[m])) if spec is not None else 1.0
            col = dict(name=name, fraction=round(frac, 2), h=round(_hue_median(ph), 1),
                       s=round(float(np.median(ps)), 0), v=round(float(np.median(pv)), 0), match=round(match, 2))
        else:
            col, match = dict(name=None, fraction=0.0, match=0.0), 0.0 if spec is not None else 1.0
        score = match * (0.2 + 0.8 * shape_match(sh, shape))
        vs, us = np.nonzero(m)
        cands.append(dict(
            id=tr.id, x=round(tr.det.x, 3), y=round(tr.det.y, 3), range_m=round(math.hypot(o.x, o.y), 3),
            bearing_deg=round(math.degrees(math.atan2(o.y, o.x)), 1),
            size_mm=[round(1000 * o.length), round(1000 * o.width), round(1000 * o.top)],
            color=col, shape=sh, score=round(score, 2), pixels=npx, seen=tr.seen, status=tr.status,
            box=None if not npx else [int(us.min()), int(vs.min()), int(us.max()), int(vs.max())]))
    cands.sort(key=lambda c: -c["score"])
    robot = ok & (np.hypot(P[..., 0], P[..., 1]) < ROBOT_R)            # the robot's own bumper and arm
    blobs = _blobs(want & ~fv._dilate(taken, 3) & ~robot, frame, P, ok, hgt) if spec is not None else []
    img = rgb.copy()
    v_, u_ = np.mgrid[0:H, 0:W]
    hatch = want & ((u_ + v_) % 6 < 2)                                   # the asked colour, hatched: its own shows
    img[hatch] = (img[hatch] * 0.3 + np.array([0, 255, 255]) * 0.7).astype(np.uint8)
    for k, c in enumerate(cands):
        if c["box"] is None:
            continue
        col = (0, 255, 0) if k == 0 and c["score"] >= 0.5 else (255, 220, 0)
        draw_box(img, *c["box"], col)
        draw_text(img, c["box"][0], c["box"][1] - 16, f"{c['id']} {c['score']:.2f}".replace("0.", "."), col)
    for b in blobs:
        draw_box(img, *b["box"], (255, 40, 40))
        draw_text(img, b["box"][0], b["box"][1] - 16, "?", (255, 40, 40))
    guess = [c for c in cands if shape != "any" and c["color"]["match"] >= 0.5 and
             ((c["shape"]["class"] == shape and not c["shape"]["sure"]) or shape in c["shape"]["maybe"])]
    ids = ", ".join(str(c["id"]) for c in guess)
    dists = ", ".join("%.1f" % c["range_m"] for c in guess)
    hint = "" if not guess else (f"{ids}: the right colour, and {shape} only as a guess from {dists} m away: move to "
                                 f"~0.6 m of it and find again")
    res = dict(query=dict(color=None if spec is None else spec["name"], shape=shape, max_range_m=max_range),
               frame="odom", t=round(t, 2), candidates=cands, image_only=blobs, hint=hint,
               dropped=dict(big=sum(o.big for o in dets_all),
                            far=sum(math.hypot(o.x, o.y) >= max_range and not o.big for o in dets_all)),
               pose=[round(float(v), 3) for v in pose])
    return res, img


def _blobs(mask: np.ndarray, frame: fv.DepthFrame, P, ok, hgt) -> list:
    """The asked colour's pixels no candidate holds: their bearing (and range and height where the depth has them)."""
    if mask.sum() < BLOB_MIN:
        return []
    lab, nl = fv._label(mask)
    if nl == 0:
        return []
    sizes = np.bincount(lab.ravel(), minlength=nl + 1)
    out = []
    fx, fy, cx, cy = frame.K
    R, o = frame.T_base_cam[:3, :3], frame.T_base_cam[:3, 3]
    for k in np.argsort(-sizes[1:])[:5] + 1:
        if sizes[k] < BLOB_MIN:
            break
        vs, us = np.nonzero(lab == k)
        u, v = float(us.mean()), float(vs.mean())
        ray = R @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        b = dict(pixels=int(sizes[k]), u=round(u), v=round(v),
                 bearing_deg=round(math.degrees(math.atan2(ray[1], ray[0])), 1), image_only=True, pickable=False,
                 box=[int(us.min()), int(vs.min()), int(us.max()), int(vs.max())])
        sel = ok[vs, us]
        if sel.sum() >= 5:
            p = np.median(P[vs[sel], us[sel]], axis=0)
            h = hgt[vs[sel], us[sel]]
            b.update(range_m=round(float(math.hypot(p[0], p[1])), 2), height_m=round(float(np.median(h)), 3),
                     flat=bool(np.percentile(h, 90) < FLAT_H))
        else:
            b["flat"] = None                             # no depth: can't tell
        if "range_m" not in b and ray[2] < 0:            # no depth: where its ray meets the floor
            s = -o[2] / ray[2]
            b["range_m"] = round(float(math.hypot(o[0] + s * ray[0], o[1] + s * ray[1])), 2)
        out.append(b)
    return out


class Finder:
    """One find at a time, for arm_brain: request() with the console's args, then step() with the newest
    depth and colour frames (taken after the request) until it answers; result (for /arm/state's "found") and image
    (for /arm/find_image). The floor map keeps the ids for the pick that follows (pick.TidyTask's only)."""
    SETTLE = 0.3                    # s: frames from at least this long after the request
    TIMEOUT = 8.0                   # s: no frames by then: an error answer

    def __init__(self):
        self.map = fv.FloorMap()
        self.req = None
        self.result = None
        self.image = None
        self.n = 0                  # answers so far (a new one to publish)

    def request(self, a: dict, t: float) -> str:
        """'' when accepted, else why not (the answer then carries the error too)."""
        rid = str(a.get("id") or "")
        try:
            color_spec(a.get("color"))
            shape = str(a.get("shape") or "any")
            if shape not in SHAPES:
                raise ValueError(f"shape: one of {', '.join(SHAPES)}")
            rng = float(a.get("max_range", 1.6))
        except (ValueError, KeyError, TypeError) as e:
            self._answer(dict(id=rid, error=str(e)), None)
            return str(e)
        self.req = dict(id=rid, color=a.get("color"), shape=shape, max_range=rng, t_req=t + self.SETTLE, t0=t,
                        t_asked=None)
        return ""

    ASK_AGAIN = 2.0                 # s: no fresh frames by then: ask the cameras again

    def due(self, t: float) -> bool:
        """True when the node should ask the cameras for a frame now (the sim renders on request): once the settle
        time is up, and again every ASK_AGAIN while no fresh frames came."""
        r = self.req
        if r is None or t < r["t_req"]:
            return False
        if r["t_asked"] is None or t - r["t_asked"] >= self.ASK_AGAIN:
            r["t_asked"] = t
            return True
        return False

    def _answer(self, result: dict, image):
        self.result, self.image, self.req = result, image, None
        self.n += 1

    def step(self, t: float, depth: fv.DepthFrame | None, color: ColorFrame | None, pose) -> bool:
        """True when it answered this step. pose: the base's (x, y, yaw) in odom (the robot stands still)."""
        r = self.req
        if r is None:
            return False
        if t - r["t0"] > self.TIMEOUT:
            why = "no depth frame" if depth is None or depth.t < r["t_req"] else "no colour frame"
            self._answer(dict(id=r["id"], error=f"find: {why} from the camera in {self.TIMEOUT:g} s"), None)
            return True
        if depth is None or color is None or depth.t < r["t_req"] or color.t < r["t_req"]:
            return False
        res, img = find(depth, color.rgb, pose or (0.0, 0.0, 0.0), self.map, t, r["color"], r["shape"],
                        r["max_range"])
        res["id"] = r["id"]
        res["image"] = "/arm/find_image"
        self._answer(res, img)
        return True
