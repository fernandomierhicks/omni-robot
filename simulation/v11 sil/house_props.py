"""The v7 house furnished (props=True; the ROS bridge's props:=true): things for an AI to find and read
(ai_robot_architecture.md: "drive to the bedroom and find the sign that says Hello", "find the person in the office").

Decoration only, class "decor" (contype 0, group 1): the cameras render it and the lidar's rays see it (group 1), the
robot's physics doesn't (it can't bump into a sofa; Nav2 plans round it from the lidar). Every geom is named prop_*:
tag_sim skips those, so the AprilTags stay where they were without the props. The saved SLAM map was made without
them: furniture shows as new obstacles to the localizer and Nav2.

  walls     the partition walls raised to 2.4 m (they are 0.5 m lidar boxes), the outer walls made visible
  living    sofa, armchair, coffee table, TV on a stand, bookshelf, floor lamp, a picture
  kitchen   counters and wall cabinets, fridge, table with four chairs, a "KITCHEN" sign
  hallway   console table, coat stand, "WELCOME" (west wall), "EXIT" (east wall)
  bedroom   bed, nightstand, wardrobe, a "HELLO" sign on the north wall, a picture
  office    desk with a monitor, chair, bookshelf, an "OFFICE" sign, a person standing (legs at the lidar's height)
  bathroom  bath, toilet, sink and mirror, a "BATHROOM" sign
  tags      each AprilTag tag_sim places, drawn on its wall (a placeholder pattern with its id, not real 36h11 codes)
The rooms (house.py, world frame): living x -6..0, y -4.5..0.5; kitchen x 0..6; hallway y 0.5..2; bedroom x -6..-1,
office -1..2, bathroom 2..6, y 2..4.5. Positions are est (a plausible home), not a measured house.
Textures are generated once into assets/props/ (PIL; not in git).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from assets import ASSET_DIR, _f

PROPS_DIR = ASSET_DIR / "props"
WALL_H = 2.4
SIGNS = {   # name: (text, fill rgb, ink rgb)
    "hello": ("HELLO", (250, 250, 245), (20, 20, 120)),
    "welcome": ("WELCOME", (245, 235, 210), (90, 50, 20)),
    "kitchen": ("KITCHEN", (250, 250, 250), (20, 90, 30)),
    "office": ("OFFICE", (250, 250, 250), (30, 30, 30)),
    "bathroom": ("BATHROOM", (230, 240, 250), (20, 60, 120)),
    "exit": ("EXIT", (20, 140, 60), (255, 255, 255)),
}
PLAIN = {   # material: (rgba, specular, emission)
    "prop_wall": ((0.92, 0.9, 0.85, 1), 0.05, 0.0), "prop_wood": ((0.55, 0.38, 0.22, 1), 0.2, 0.0),
    "prop_dark": ((0.14, 0.14, 0.16, 1), 0.3, 0.0), "prop_white": ((0.95, 0.95, 0.94, 1), 0.4, 0.0),
    "prop_fabric": ((0.36, 0.42, 0.52, 1), 0.02, 0.0), "prop_fabric2": ((0.55, 0.45, 0.35, 1), 0.02, 0.0),
    "prop_metal": ((0.72, 0.74, 0.77, 1), 0.6, 0.0), "prop_cabinet": ((0.88, 0.86, 0.8, 1), 0.2, 0.0),
    "prop_counter": ((0.27, 0.27, 0.29, 1), 0.5, 0.0), "prop_blanket": ((0.72, 0.62, 0.45, 1), 0.02, 0.0),
    "prop_skin": ((0.82, 0.62, 0.48, 1), 0.1, 0.0), "prop_shirt": ((0.25, 0.5, 0.35, 1), 0.05, 0.0),
    "prop_trousers": ((0.2, 0.22, 0.3, 1), 0.05, 0.0), "prop_lamp": ((0.98, 0.94, 0.8, 1), 0.1, 0.4),
    "prop_glass": ((0.75, 0.85, 0.9, 1), 0.9, 0.0), "prop_plant": ((0.2, 0.45, 0.2, 1), 0.05, 0.0),
}


# -- textures ----------------------------------------------------------------------------------------------------------
def _font(px: int):
    for name in ("DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "arialbd.ttf",
                 "arial.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=px)
    except TypeError:                                 # an old Pillow: the tiny bitmap font
        return ImageFont.load_default()


def _sign(path, text, fill, ink, w=640, h=256):
    im = Image.new("RGB", (w, h), fill)
    d = ImageDraw.Draw(im)
    d.rectangle([6, 6, w - 7, h - 7], outline=ink, width=8)
    f = _font(int(h * 0.5))
    box = d.textbbox((0, 0), text, font=f)
    s = min(1.0, (w - 60) / max(1, box[2] - box[0]))
    if s < 1.0:
        f = _font(int(h * 0.5 * s))
        box = d.textbbox((0, 0), text, font=f)
    d.text(((w - (box[2] - box[0])) / 2 - box[0], (h - (box[3] - box[1])) / 2 - box[1]), text, font=f, fill=ink)
    im.save(path)


def _art(path, seed, w=384, h=256):
    rng = np.random.default_rng(seed)
    im = Image.new("RGB", (w, h), tuple(int(v) for v in rng.integers(150, 235, 3)))
    d = ImageDraw.Draw(im)
    for _ in range(9):
        x0, y0 = rng.integers(0, w), rng.integers(0, h)
        r = int(rng.integers(20, 90))
        d.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], fill=tuple(int(v) for v in rng.integers(30, 220, 3)))
    d.rectangle([0, 0, w - 1, h - 1], outline=(60, 45, 30), width=14)
    im.save(path)


def _tv(path, w=512, h=288):
    im = Image.new("RGB", (w, h), (20, 30, 60))
    d = ImageDraw.Draw(im)
    for k in range(h):
        d.line([(0, k), (w, k)], fill=(20 + k // 4, 40 + k // 5, 90 + k // 6))
    d.ellipse([w * 0.6, h * 0.15, w * 0.8, h * 0.5], fill=(250, 220, 120))
    d.polygon([(0, h), (w * 0.35, h * 0.45), (w * 0.7, h)], fill=(40, 90, 50))
    im.save(path)


def _tag(path, tag_id: int, px: int = 200):
    """A placeholder for tag tag_id: an AprilTag-like layout (white margin, black border, 6 x 6 bits from the id) with
    the id printed under it. Not a decodable 36h11 code: the sim's detections come from tag_sim, not from pixels."""
    im = Image.new("RGB", (px, int(px * 1.25)), (255, 255, 255))
    d = ImageDraw.Draw(im)
    c = px / 10
    d.rectangle([c, c, px - c, px - c], fill=(0, 0, 0))
    rng = np.random.default_rng(1000 + tag_id)
    bits = rng.integers(0, 2, (6, 6))
    for i in range(6):
        for j in range(6):
            if bits[i, j]:
                d.rectangle([c * (2 + j), c * (2 + i), c * (3 + j) - 1, c * (3 + i) - 1], fill=(255, 255, 255))
    f = _font(int(px * 0.16))
    d.text((px / 2, px * 1.12), f"tag {tag_id}", font=f, fill=(0, 0, 0), anchor="mm")
    im.save(path)


def textures(tag_ids=()) -> dict:
    """{name: png path}, made when missing."""
    PROPS_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for k, (text, fill, ink) in SIGNS.items():
        p = PROPS_DIR / f"sign_{k}.png"
        if not p.exists():
            _sign(p, text, fill, ink)
        out[f"prop_sign_{k}"] = p
    for k in (1, 2, 3):
        p = PROPS_DIR / f"art_{k}.png"
        if not p.exists():
            _art(p, k)
        out[f"prop_art_{k}"] = p
    p = PROPS_DIR / "tv.png"
    if not p.exists():
        _tv(p)
    out["prop_tv"] = p
    for i in tag_ids:
        p = PROPS_DIR / f"tag_{i}.png"
        if not p.exists():
            _tag(p, i)
        out[f"prop_tag_{i}"] = p
    return out


# -- the tags' places (tag_sim's, on the house without the props) -------------------------------------------------------
_TAGS: dict = {}


def tag_places(p) -> list:
    """[(id, centre xyz, R (columns: x along the wall, y up, z out of the face), size)] as tag_sim.TagWorld puts them."""
    key = (repr(p.floor_features), tuple(p.area_size))
    if key not in _TAGS:
        try:
            import mujoco
            from robot_model import build_model
            from tag_sim import TagWorld
            q = dataclasses.replace(p, props=False)
            m = build_model(q)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            w = TagWorld(m, d)
            tags = {t.tag_id: t for t in w.tags + ([w.dock] if w.dock is not None else [])}   # the dock may be listed
            _TAGS[key] = [(t.tag_id, t.t.copy(), t.R.copy(), t.size) for t in tags.values()]
        except Exception as e:                       # no tag faces is better than no house
            print(f"house_props: no tag faces ({e})")
            _TAGS[key] = []
    return _TAGS[key]


# -- geometry ------------------------------------------------------------------------------------------------------------
class _Geoms:
    def __init__(self):
        self.g: list[str] = []

    def _name(self):
        return f'name="prop_{len(self.g)}"'

    def box(self, lo, hi, mat):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        c, h = (lo + hi) / 2, np.maximum(np.abs(hi - lo) / 2, 0.001)
        self.g.append(f'<geom {self._name()} class="decor" type="box" material="{mat}" pos="{_f(*c)}" size="{_f(*h)}"/>')

    def panel(self, centre, normal_deg, w, h, mat, depth=0.004):
        """A flat picture facing normal_deg (its texture's right along the wall as seen from the room, up = up)."""
        n = (math.cos(math.radians(normal_deg)), math.sin(math.radians(normal_deg)))
        x = (-n[1], n[0], 0.0)                       # right, seen from the room (looking along -n)
        c = (centre[0] + n[0] * depth, centre[1] + n[1] * depth, centre[2])
        self.g.append(f'<geom {self._name()} class="decor" type="box" material="{mat}" pos="{_f(*c)}" '
                      f'size="{_f(w / 2, h / 2, depth)}" xyaxes="{_f(*x, 0, 0, 1)}"/>')

    def framed(self, centre, normal_deg, w, h, mat, frame="prop_dark"):
        self.panel(centre, normal_deg, w + 0.06, h + 0.06, frame, 0.006)
        n = (math.cos(math.radians(normal_deg)), math.sin(math.radians(normal_deg)))
        self.panel((centre[0] + n[0] * 0.006, centre[1] + n[1] * 0.006, centre[2]), normal_deg, w, h, mat)

    def cyl(self, a, b, r, mat):
        self.g.append(f'<geom {self._name()} class="decor" type="cylinder" material="{mat}" size="{_f(r)}" '
                      f'fromto="{_f(*a, *b)}"/>')

    def cap(self, a, b, r, mat):
        self.g.append(f'<geom {self._name()} class="decor" type="capsule" material="{mat}" size="{_f(r)}" '
                      f'fromto="{_f(*a, *b)}"/>')

    def sph(self, c, r, mat):
        self.g.append(f'<geom {self._name()} class="decor" type="sphere" material="{mat}" size="{_f(r)}" '
                      f'pos="{_f(*c)}"/>')

    def table(self, x0, y0, x1, y1, h, mat="prop_wood", leg=0.025, top=0.03):
        self.box((x0, y0, h - top), (x1, y1, h), mat)
        for x, y in ((x0 + 0.06, y0 + 0.06), (x1 - 0.06, y0 + 0.06), (x0 + 0.06, y1 - 0.06), (x1 - 0.06, y1 - 0.06)):
            self.cyl((x, y, 0.0), (x, y, h - top), leg, mat)

    def chair(self, x, y, back_deg, mat="prop_wood"):
        s = 0.21
        self.table(x - s, y - s, x + s, y + s, 0.46, mat, leg=0.018, top=0.04)
        bx, by = x + s * math.cos(math.radians(back_deg)), y + s * math.sin(math.radians(back_deg))
        ax, ay = -math.sin(math.radians(back_deg)) * s, math.cos(math.radians(back_deg)) * s
        self.box((min(bx - ax, bx + ax) - 0.01, min(by - ay, by + ay) - 0.01, 0.46),
                 (max(bx - ax, bx + ax) + 0.01, max(by - ay, by + ay) + 0.01, 0.92), mat)

    def person(self, x, y, facing_deg):
        c, s = math.cos(math.radians(facing_deg)), math.sin(math.radians(facing_deg))
        side = (-s * 0.1, c * 0.1)
        for k in (1, -1):
            lx, ly = x + k * side[0], y + k * side[1]
            self.cap((lx, ly, 0.08), (lx, ly, 0.85), 0.065, "prop_trousers")
            self.cap((x + k * side[0] * 2.1, y + k * side[1] * 2.1, 1.0), (x + k * side[0] * 2.3 + c * 0.05,
                                                                          y + k * side[1] * 2.3 + s * 0.05, 1.42),
                     0.045, "prop_shirt")
        self.cap((x, y, 0.95), (x, y, 1.42), 0.16, "prop_shirt")
        self.sph((x, y, 1.62), 0.11, "prop_skin")


def assets_xml(p) -> str:
    tags = tag_places(p)
    tex = textures([t[0] for t in tags])
    out = [f'<texture name="{k}" type="2d" file="{v.relative_to(ASSET_DIR).as_posix()}"/>' for k, v in tex.items()]
    out += [f'<material name="{k}" texture="{k}" specular="0.1"/>' for k in tex]
    out += [f'<material name="{k}" rgba="{_f(*rgba)}" specular="{sp}" emission="{em}"/>' for k, (rgba, sp, em) in
            PLAIN.items()]
    return "\n    ".join(out)


def worldbody_xml(p) -> str:
    from floor import Wall
    G = _Geoms()
    ax, ay = p.area_size[0] / 2, p.area_size[1] / 2
    # walls: the partitions up to WALL_H (a hair thinner, so a tag's face stays in front), the outer ones visible
    for w in p.floor_features:
        if isinstance(w, Wall):
            L = math.hypot(w.x1 - w.x0, w.y1 - w.y0)
            t = w.thickness / 2 - 0.002
            if abs(w.y1 - w.y0) < 1e-6:
                G.box((min(w.x0, w.x1), w.y0 - t, w.height), (max(w.x0, w.x1), w.y0 + t, WALL_H), "prop_wall")
            elif L > 0:
                G.box((w.x0 - t, min(w.y0, w.y1), w.height), (w.x0 + t, max(w.y0, w.y1), WALL_H), "prop_wall")
    for lo, hi in (((-ax - 0.06, -ay - 0.06, 0), (-ax - 0.012, ay + 0.06, WALL_H)),
                   ((ax + 0.012, -ay - 0.06, 0), (ax + 0.06, ay + 0.06, WALL_H)),
                   ((-ax, -ay - 0.06, 0), (ax, -ay - 0.012, WALL_H)),
                   ((-ax, ay + 0.012, 0), (ax, ay + 0.06, WALL_H))):
        G.box(lo, hi, "prop_wall")
    W, S, E, N = -ax + 0.012, -ay + 0.012, ax - 0.012, ay - 0.012     # the outer walls' inner faces
    # living room (x < 0, y < 0.5)
    G.box((-4.1, S, 0.0), (-2.1, -3.75, 0.45), "prop_fabric")         # sofa: seat, back, arms
    G.box((-4.1, S, 0.45), (-2.1, S + 0.2, 0.88), "prop_fabric")
    G.box((-4.25, S, 0.0), (-4.1, -3.75, 0.62), "prop_fabric")
    G.box((-2.1, S, 0.0), (-1.95, -3.75, 0.62), "prop_fabric")
    G.table(-3.6, -3.05, -2.6, -2.5, 0.42)                             # coffee table
    G.box((W, -3.2, 0.0), (W + 0.42, -1.6, 0.5), "prop_wood")          # TV stand and the TV
    G.box((W + 0.1, -3.0, 0.5), (W + 0.16, -1.8, 0.56), "prop_dark")
    G.panel((W + 0.17, -2.4, 0.95), 0.0, 1.2, 0.68, "prop_tv")
    G.box((-5.7, S, 0.0), (-4.9, S + 0.8, 0.45), "prop_fabric2")       # armchair
    G.box((-5.7, S, 0.45), (-4.9, S + 0.18, 0.9), "prop_fabric2")
    G.box((-1.05, S, 0.0), (-0.15, S + 0.36, 1.8), "prop_wood")        # bookshelf
    for z in (0.45, 0.9, 1.35):
        G.box((-1.0, S + 0.34, z), (-0.2, S + 0.36, z + 0.25), "prop_art_3")
    G.cyl((-1.4, -3.95, 0.0), (-1.4, -3.95, 0.03), 0.16, "prop_dark")  # floor lamp
    G.cyl((-1.4, -3.95, 0.03), (-1.4, -3.95, 1.5), 0.015, "prop_metal")
    G.cyl((-1.4, -3.95, 1.5), (-1.4, -3.95, 1.75), 0.18, "prop_lamp")
    G.framed((-3.1, S, 1.45), 90.0, 1.0, 0.66, "prop_art_1")           # a picture over the sofa
    # kitchen (x > 0, y < 0.5)
    G.box((0.3, S, 0.0), (2.9, S + 0.62, 0.88), "prop_cabinet")        # counters, worktop, wall cabinets
    G.box((0.3, S, 0.88), (2.9, S + 0.64, 0.92), "prop_counter")
    G.box((0.3, S, 1.45), (2.9, S + 0.35, 2.15), "prop_cabinet")
    G.box((3.0, S, 0.0), (3.7, S + 0.7, 1.8), "prop_metal")            # fridge
    G.panel((1.6, S + 0.35, 1.8), 90.0, 0.8, 0.32, "prop_sign_kitchen")
    G.table(1.2, -2.2, 2.4, -1.4, 0.74)                                # kitchen table and chairs
    for x, y, back in ((1.5, -2.55, 270), (2.1, -2.55, 270), (1.5, -1.05, 90), (2.1, -1.05, 90)):
        G.chair(x, y, back)
    # hallway (y 0.5..2)
    G.table(4.4, 1.62, 5.6, 1.94, 0.8)                                 # console table and a plant
    G.cyl((5.3, 1.78, 0.8), (5.3, 1.78, 1.0), 0.08, "prop_dark")
    G.sph((5.3, 1.78, 1.12), 0.15, "prop_plant")
    G.cyl((-5.75, 0.75, 0.0), (-5.75, 0.75, 1.75), 0.02, "prop_metal")  # coat stand
    G.sph((-5.75, 0.75, 1.78), 0.05, "prop_metal")
    G.panel((W, 1.25, 1.8), 0.0, 1.0, 0.4, "prop_sign_welcome")
    G.panel((E, 1.25, 2.05), 180.0, 0.5, 0.2, "prop_sign_exit")
    # bedroom (x -6..-1, y 2..4.5)
    G.box((-5.4, 2.6, 0.0), (-3.8, N, 0.35), "prop_wood")              # bed: frame, mattress, blanket, headboard
    G.box((-5.35, 2.65, 0.35), (-3.85, N - 0.05, 0.52), "prop_white")
    G.box((-5.38, 2.62, 0.45), (-3.82, 3.9, 0.56), "prop_blanket")
    G.box((-5.4, N - 0.08, 0.0), (-3.8, N, 1.1), "prop_wood")
    G.box((-3.6, N - 0.42, 0.0), (-3.15, N, 0.55), "prop_wood")       # nightstand and its lamp
    G.cyl((-3.38, N - 0.2, 0.55), (-3.38, N - 0.2, 0.8), 0.08, "prop_lamp")
    G.box((-1.65, 3.2, 0.0), (-1.06, 4.4, 2.0), "prop_wood")           # wardrobe against the office wall
    G.panel((-2.4, N, 1.75), -90.0, 0.6, 0.24, "prop_sign_hello")
    G.framed((-4.6, 2.06, 1.4), 90.0, 0.7, 0.46, "prop_art_2")         # a picture on the hall side of the bedroom
    # office (x -1..2, y 2..4.5)
    G.table(-0.6, N - 0.6, 0.8, N, 0.74)                               # desk, monitor, chair
    G.box((-0.05, N - 0.25, 0.74), (0.45, N - 0.2, 1.1), "prop_dark")
    G.chair(0.1, 3.55, 270)
    G.box((1.56, 3.4, 0.0), (1.94, 4.4, 1.8), "prop_wood")             # bookshelf on the bathroom wall
    G.panel((-0.94, 3.3, 1.75), 0.0, 0.6, 0.24, "prop_sign_office")
    G.person(1.1, 3.0, 200.0)                                          # someone standing, looking at the door
    # bathroom (x 2..6, y 2..4.5)
    G.box((4.3, N - 0.72, 0.0), (E, N, 0.55), "prop_white")            # bath
    G.cyl((2.35, N - 0.35, 0.0), (2.35, N - 0.35, 0.42), 0.18, "prop_white")    # toilet and its tank
    G.box((2.2, N - 0.18, 0.4), (2.5, N, 0.8), "prop_white")
    G.box((2.06, 2.6, 0.0), (2.55, 3.3, 0.85), "prop_cabinet")         # sink and mirror
    G.panel((2.06, 2.95, 1.45), 0.0, 0.5, 0.6, "prop_glass")
    G.panel((E, 3.0, 1.8), 180.0, 0.8, 0.26, "prop_sign_bathroom")
    # the tags' faces, where tag_sim puts them (on the raised walls)
    for tid, t, R, size in tag_places(p):
        x, z = R[:, 0], R[:, 2]
        w = size / 0.8                                       # the texture's black square is 0.8 of its width
        c = t + z * 0.003 - R[:, 1] * w * 0.125              # the id printed under the code: the image is 1.25 x tall
        G.g.append(f'<geom name="prop_tag{tid}" class="decor" type="box" material="prop_tag_{tid}" pos="{_f(*c)}" '
                   f'size="{_f(w / 2, w / 2 * 1.25, 0.002)}" xyaxes="{_f(*x, *R[:, 1])}"/>')
    return "".join(G.g)
