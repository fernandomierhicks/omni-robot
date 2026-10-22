"""v11 (design.md 31): a furnished house around the catch game's 12 x 6 m field, for the rendered cameras.

A great room (15.6 x 11 m, 3.4 m ceiling, est) with the field in the middle: living area (sofa, armchair, rug,
coffee table, floor lamp), the wall behind the thrower (built-in bookcase, TV on a console, a glass patio door), a
kitchen (counters, island, stools, fridge), a dining corner behind the robot, windows onto a garden with trees, and
people: the thrower (the ball leaves their hand), one bystander on the sofa and one standing by the bookcase.

Everything here is decoration: class "decor" geoms (contype = conaffinity = 0, no mass) placed outside the field, so
the physics of the throw and of the robot are those of the arena scene (test_v11 checks it). They still occlude the
cameras: the renderer draws them, and CameraSim's occlusion rays hit group 1.

Ball-coloured clutter on purpose (the colour detector's hue window is 45-100 deg): a tennis ball on a shelf, a
mustard cushion, lemons and limes in a bowl, a yellow toy truck, a lime toy ball, the standing bystander's mustard
shirt, sunlit foliage through the windows, and the next ball in the thrower's hand while they wait (CLUTTER).

Textures: CC0 photos from Poly Haven (POLYHAVEN), fetched once with `python house_scene.py fetch` into
assets/house/ (not in git; sources and md5 in assets/house/SOURCES.md). Without them the scene builds with flat
procedural stand-ins and says so.

    python house_scene.py fetch            download and convert the textures
    python house_scene.py overview         render renders/v11_house_overview.png
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image

from assets import ASSET_DIR, _f

HOUSE_DIR = ASSET_DIR / "house"

# material -> (Poly Haven asset, what it is). All CC0 (https://polyhaven.com/license)
POLYHAVEN = {
    "h_floor": ("wood_floor", "oak strip floor"),
    "h_wall": ("plastered_wall_04", "painted plaster walls"),
    "h_brick": ("painted_brick", "white painted brick, the wall behind the thrower"),
    "h_ceiling": ("white_plaster_02", "ceiling"),
    "h_sofa": ("velour_velvet", "sofa upholstery"),
    "h_leather": ("brown_leather", "armchair"),
    "h_rug": ("poly_wool_herringbone", "rug"),
    "h_linen": ("rough_linen", "curtains"),
    "h_table": ("wood_table_001", "coffee table, console, dining table"),
    "h_cabinet": ("kitchen_wood", "kitchen cabinets"),
    "h_marble": ("marble_01", "counter tops"),
    "h_grass": ("leafy_grass", "the garden outside"),
    "h_oak": ("white_oak_veneer", "bookcase, shelves"),
}
TEX_PX = 1024
# recoloured on conversion (the photo's luminance detail, times a contrast factor, on a new mean colour): white
# walls, ceiling and painted brick, off-white curtains, a slate sofa
TINT = {"h_wall": ((228, 224, 216), 0.7), "h_ceiling": ((240, 239, 236), 0.25), "h_brick": ((232, 230, 226), 1.0),
        "h_linen": ((222, 214, 200), 1.0), "h_sofa": ((92, 102, 118), 1.0)}

# flat stand-ins (mean colour, noise amplitude) when the photos aren't fetched
FALLBACK = {
    "h_floor": ((168, 124, 84), 18), "h_wall": ((222, 218, 210), 5), "h_brick": ((226, 224, 220), 10),
    "h_ceiling": ((236, 236, 234), 4), "h_sofa": ((96, 104, 116), 8), "h_leather": ((104, 66, 42), 10),
    "h_rug": ((170, 160, 146), 14), "h_linen": ((214, 206, 192), 8), "h_table": ((132, 92, 60), 14),
    "h_cabinet": ((196, 176, 146), 10), "h_marble": ((228, 228, 226), 8), "h_grass": ((76, 110, 46), 22),
    "h_oak": ((206, 180, 146), 10),
}

# the room (world frame: the field is x -6..6, y -3..3; the thrower on -x, the robot's home at x = 3.5)
X0, X1, Y0, Y1 = -8.0, 7.6, -5.8, 5.2
CEIL = 3.4                     # m, a great room's ceiling (est); the 2 m apex throws peak near 3.05 m
WALL_T = 0.12
FIELD = (6.0, 3.0)             # half sizes of the play field (catch_world.AREA / 2)
CEILING_GROUP = 2              # geom group of the ceiling (drawn by default; the cutaway overview hides it)


# ---------------------------------------------------------------------------
# textures

def _get(url: str, timeout: float = 60) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "omni-robot-sim/1.0 (house_scene.py)"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _api(asset: str) -> dict:
    return json.loads(_get(f"https://api.polyhaven.com/files/{asset}", 30).decode())


def fetch(force: bool = False):
    """Download each texture's 1k diffuse JPG (md5-checked against Poly Haven's API) into its own folder under
    assets/house/download/, convert it to a PNG MuJoCo reads, and write SOURCES.md."""
    HOUSE_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    for mat, (asset, what) in POLYHAVEN.items():
        png = HOUSE_DIR / f"{mat}.png"
        info = _api(asset)
        diff = info.get("Diffuse") or info.get("diffuse")
        f = diff["1k"]["jpg"]
        url, md5 = f["url"], f["md5"]
        rows.append((mat, asset, what, url, md5))
        if png.exists() and not force:
            continue
        sub = HOUSE_DIR / "download" / asset           # each download in its own folder
        sub.mkdir(parents=True, exist_ok=True)
        jpg = sub / Path(url).name
        data = jpg.read_bytes() if jpg.exists() else _get(url)
        got = hashlib.md5(data).hexdigest()
        if got != md5:
            raise RuntimeError(f"{asset}: md5 {got} != {md5} (Poly Haven API); not using it")
        jpg.write_bytes(data)
        img = Image.open(jpg)
        img.load()
        if img.format != "JPEG" or img.mode != "RGB":
            raise RuntimeError(f"{asset}: expected an RGB JPEG, got {img.format} {img.mode}")
        img = img.resize((TEX_PX, TEX_PX), Image.LANCZOS)
        if mat in TINT:
            rgb, contrast = TINT[mat]
            a = np.asarray(img, np.float32)
            lum = a @ np.array([0.299, 0.587, 0.114], np.float32)
            rel = 1 + contrast * (lum / lum.mean() - 1)
            img = Image.fromarray(np.clip(rel[..., None] * np.array(rgb, np.float32), 0, 255).astype(np.uint8))
        img.save(png)
        print(f"  {mat}: {asset} ({len(data) // 1024} KB) -> {png.name}")
    lines = ["# House scene textures", "",
             "Fetched by `python house_scene.py fetch` (simulation/v11 sil). All from Poly Haven, licence CC0 "
             "(https://polyhaven.com/license). 1k diffuse JPGs, md5-checked against the Poly Haven API, resized to "
             f"{TEX_PX} px PNGs. Recoloured (luminance detail times a contrast factor, on a new mean RGB): "
             + ", ".join(f"{m} {c} x{k}" for m, (c, k) in TINT.items()) + ".", "",
             "| Material | Poly Haven asset | Used for | URL | md5 |", "| --- | --- | --- | --- | --- |"]
    lines += [f"| {m} | {a} | {w} | {u} | {h} |" for m, a, w, u, h in rows]
    (HOUSE_DIR / "SOURCES.md").write_text("\n".join(lines) + "\n")


def _procedural(path: Path, rgb, amp, seed: int):
    rng = np.random.default_rng(seed)
    n = 256
    small = rng.normal(0, 1, (n // 8, n // 8))
    big = np.asarray(Image.fromarray(((small - small.min()) / np.ptp(small) * 255).astype(np.uint8))
                     .resize((n, n), Image.BICUBIC), dtype=np.float32) / 255 - 0.5
    img = np.array(rgb, np.float32)[None, None, :] + amp * (big[..., None] + 0.3 * rng.normal(0, 1, (n, n, 1)))
    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(path)


def _tv_screen(path: Path):
    """A dim landscape on the TV (est): dark blue-grey, a horizon, a pale sun."""
    h, w = 270, 480
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    img = np.zeros((h, w, 3), np.float32)
    img[:] = np.array([0.18, 0.24, 0.34])
    img[y > 0.62 * h] = np.array([0.12, 0.16, 0.14])
    img += 0.35 * np.exp(-(((x - 0.7 * w) ** 2 + (y - 0.35 * h) ** 2) / (2 * 22.0 ** 2)))[..., None] * \
        np.array([1.0, 0.9, 0.75])
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def _art(path: Path, seed: int):
    """An abstract print for a frame: soft colour fields (blue, terracotta, sand, one ochre band)."""
    rng = np.random.default_rng(seed)
    h, w = 256, 192
    img = np.ones((h, w, 3), np.float32) * np.array([0.92, 0.9, 0.85])
    cols = [(0.2, 0.32, 0.5), (0.72, 0.38, 0.26), (0.85, 0.78, 0.6), (0.8, 0.6, 0.18), (0.3, 0.35, 0.3)]
    for c in cols:
        y0, y1 = sorted(rng.integers(0, h, 2))
        x0, x1 = sorted(rng.integers(0, w, 2))
        img[y0:y1 + 20, x0:x1 + 20] = c
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def _book_spines(path: Path, seed: int = 5):
    """Rows of book spines (seeded colours; a few yellow ones), for the bookcase's shelves."""
    rng = np.random.default_rng(seed)
    h, w = 128, 1024
    img = np.zeros((h, w, 3), np.float32)
    palette = np.array([(0.55, 0.12, 0.1), (0.12, 0.22, 0.42), (0.85, 0.82, 0.74), (0.15, 0.3, 0.2),
                        (0.08, 0.08, 0.1), (0.62, 0.45, 0.28), (0.9, 0.75, 0.2), (0.35, 0.18, 0.35),
                        (0.75, 0.75, 0.72), (0.2, 0.45, 0.55)])
    x = 0
    while x < w:
        bw = int(rng.integers(8, 26))
        top = int(rng.integers(0, 30))
        c = palette[rng.integers(len(palette))] * rng.uniform(0.8, 1.05)
        img[top:, x:x + bw - 1] = c
        img[top + 10:top + 13, x:x + bw - 1] *= 0.6
        x += bw
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def ensure_house_textures() -> dict:
    """{material: png path} for every house texture, making stand-ins where a photo is missing."""
    HOUSE_DIR.mkdir(parents=True, exist_ok=True)
    out, missing = {}, []
    for k, mat in enumerate(POLYHAVEN):
        png = HOUSE_DIR / f"{mat}.png"
        if not png.exists():
            png = HOUSE_DIR / f"{mat}_stand_in.png"
            if not png.exists():
                _procedural(png, *FALLBACK[mat], seed=k)
            missing.append(mat)
        out[mat] = png
    for name, fn in (("h_tv", _tv_screen), ("h_books", _book_spines),
                     ("h_art1", lambda p: _art(p, 1)), ("h_art2", lambda p: _art(p, 4))):
        png = HOUSE_DIR / f"{name}.png"
        if not png.exists():
            fn(png)
        out[name] = png
    if missing and not getattr(ensure_house_textures, "_warned", False):
        print(f"house_scene: {len(missing)} photo textures missing ({', '.join(missing)}): using flat stand-ins; "
              f"run `python house_scene.py fetch` for the CC0 photos", file=sys.stderr)
        ensure_house_textures._warned = True
    return out


# ---------------------------------------------------------------------------
# materials

# photo materials: (texrepeat per metre, specular, shininess, reflectance)
TEXTURED = {
    "h_floor": (0.7, 0.35, 0.55, 0.06), "h_wall": (0.5, 0.05, 0.1, 0), "h_brick": (0.9, 0.05, 0.1, 0),
    "h_ceiling": (0.5, 0.0, 0.1, 0), "h_sofa": (2.0, 0.05, 0.1, 0), "h_leather": (2.0, 0.35, 0.5, 0),
    "h_rug": (1.2, 0.0, 0.05, 0), "h_linen": (1.5, 0.0, 0.05, 0), "h_table": (1.0, 0.4, 0.6, 0),
    "h_cabinet": (0.8, 0.25, 0.4, 0), "h_marble": (0.6, 0.6, 0.8, 0), "h_grass": (0.3, 0.0, 0.05, 0),
    "h_oak": (1.0, 0.25, 0.4, 0),
}
# plain materials: (rgba, specular, shininess, emission). (est) colours; the ones in the detector's hue window
# (45-100 deg, saturation >= 0.45, value >= 0.35) are marked "ball-coloured"
COLOURS = {
    "h_white": ((0.93, 0.93, 0.91, 1), 0.2, 0.3, 0),           # trim, frames, doors
    "h_offwhite": ((0.86, 0.84, 0.8, 1), 0.1, 0.2, 0),
    "h_black": ((0.04, 0.04, 0.045, 1), 0.6, 0.7, 0),          # TV, piano, stool frames
    "h_charcoal": ((0.16, 0.16, 0.17, 1), 0.3, 0.4, 0),
    "h_steel": ((0.72, 0.73, 0.75, 1), 0.9, 0.85, 0),
    "h_alu": ((0.3, 0.31, 0.33, 1), 0.6, 0.6, 0),              # window and door frames
    "h_glass": ((0.8, 0.88, 0.92, 0.12), 1.0, 1.0, 0),
    "h_tile": ((0.9, 0.9, 0.88, 1), 0.6, 0.7, 0),
    "h_teal": ((0.16, 0.42, 0.45, 1), 0.05, 0.1, 0),
    "h_rust": ((0.62, 0.28, 0.16, 1), 0.05, 0.1, 0),
    "h_terracotta": ((0.66, 0.36, 0.24, 1), 0.1, 0.2, 0),
    "h_ceramic": ((0.88, 0.88, 0.86, 1), 0.7, 0.8, 0),
    "h_wicker": ((0.6, 0.46, 0.3, 1), 0.05, 0.1, 0),
    "h_soil": ((0.2, 0.14, 0.1, 1), 0.0, 0.0, 0),
    "h_leaf_dark": ((0.1, 0.24, 0.08, 1), 0.3, 0.4, 0),       # indoor plants
    "h_leaf": ((0.2, 0.36, 0.12, 1), 0.3, 0.4, 0),
    "h_red": ((0.7, 0.1, 0.08, 1), 0.3, 0.4, 0),
    "h_blue": ((0.12, 0.25, 0.65, 1), 0.3, 0.4, 0),
    "h_mustard": ((0.82, 0.7, 0.1, 1), 0.05, 0.1, 0),         # ball-coloured: cushion (hue 50 deg)
    "h_yellow_shirt": ((0.8, 0.7, 0.12, 1), 0.05, 0.1, 0),    # ball-coloured: bystander's shirt (51 deg)
    "h_toy_yellow": ((0.95, 0.8, 0.08, 1), 0.5, 0.5, 0),      # ball-coloured: toy truck (50 deg)
    "h_lime_toy": ((0.62, 0.86, 0.2, 1), 0.4, 0.4, 0),        # ball-coloured: toy ball (82 deg)
    "h_lemon": ((0.96, 0.86, 0.2, 1), 0.4, 0.5, 0),           # ball-coloured (52 deg)
    "h_lime": ((0.48, 0.7, 0.14, 1), 0.4, 0.5, 0),            # ball-coloured (84 deg)
    "h_skin1": ((0.87, 0.68, 0.56, 1), 0.2, 0.3, 0),
    "h_skin2": ((0.55, 0.38, 0.27, 1), 0.2, 0.3, 0),
    "h_skin3": ((0.76, 0.56, 0.42, 1), 0.2, 0.3, 0),
    "h_hair_dark": ((0.1, 0.07, 0.05, 1), 0.2, 0.3, 0),
    "h_hair_brown": ((0.32, 0.2, 0.1, 1), 0.2, 0.3, 0),
    "h_jeans": ((0.14, 0.2, 0.34, 1), 0.05, 0.1, 0),
    "h_khaki": ((0.55, 0.5, 0.38, 1), 0.05, 0.1, 0),
    "h_grey_pants": ((0.3, 0.3, 0.32, 1), 0.05, 0.1, 0),
    "h_shirt_blue": ((0.2, 0.42, 0.7, 1), 0.05, 0.1, 0),       # the thrower
    "h_shirt_red": ((0.55, 0.12, 0.14, 1), 0.05, 0.1, 0),
    "h_shoe": ((0.9, 0.9, 0.92, 1), 0.3, 0.3, 0),
    "h_shoe_dark": ((0.15, 0.12, 0.1, 1), 0.3, 0.3, 0),
    # lights and screens (emissive)
    "h_downlight": ((1.0, 0.97, 0.9, 1), 0.0, 0.0, 1.0),
    "h_shade": ((1.0, 0.9, 0.72, 1), 0.0, 0.0, 0.85),         # lamp shades, lit from inside
    "h_bulb": ((1.0, 0.95, 0.85, 1), 0.0, 0.0, 1.0),
    # outside: lit by the sun and overexposed for a camera set for the room (emission stands in for the extra stops)
    "h_bark": ((0.3, 0.24, 0.18, 1), 0.0, 0.0, 0.3),
    "h_foliage1": ((0.2, 0.36, 0.11, 1), 0.1, 0.2, 0.35),     # 95 deg: ball-coloured once bright
    "h_foliage2": ((0.29, 0.45, 0.13, 1), 0.1, 0.2, 0.35),    # 90 deg
    "h_foliage3": ((0.4, 0.52, 0.15, 1), 0.1, 0.2, 0.35),     # 79 deg
    "h_foliage4": ((0.14, 0.28, 0.1, 1), 0.1, 0.2, 0.35),     # 107 deg: outside the hue window
    "h_hedge": ((0.12, 0.3, 0.1, 1), 0.05, 0.1, 0.3),
    "h_fence": ((0.62, 0.52, 0.4, 1), 0.05, 0.1, 0.35),
    "h_patio": ((0.62, 0.6, 0.56, 1), 0.1, 0.2, 0.3),
    "h_hall": ((0.42, 0.4, 0.37, 1), 0.0, 0.0, 0),            # the dim hallway behind the door
}


def assets_xml() -> str:
    tex = ensure_house_textures()
    out = [f'<texture name="{k}" type="2d" file="{v.relative_to(ASSET_DIR).as_posix()}"/>' for k, v in tex.items()]
    for k, (rep, spec, shin, refl) in TEXTURED.items():
        out.append(f'<material name="{k}" texture="{k}" texrepeat="{_f(rep, rep)}" texuniform="true" '
                   f'specular="{spec}" shininess="{shin}" reflectance="{refl}"/>')
    for k, (rgba, spec, shin, em) in COLOURS.items():
        out.append(f'<material name="{k}" rgba="{_f(*rgba)}" specular="{spec}" shininess="{shin}" emission="{em}"/>')
    out += ['<material name="h_tv" texture="h_tv" emission="0.9" specular="0.8" shininess="0.9"/>',
            '<material name="h_books" texture="h_books" specular="0.1"/>',
            '<material name="h_art1" texture="h_art1" specular="0.1"/>',
            '<material name="h_art2" texture="h_art2" specular="0.1"/>']
    return "\n    ".join(out)


# the rendering settings for the house (replace the arena's <visual> and skybox)
VISUAL_XML = """
  <visual>
    <global offwidth="2560" offheight="1440" azimuth="90" elevation="-15"/>
    <quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.24 0.24 0.23" diffuse="0 0 0" specular="0 0 0"/>
    <map znear="0.005" zfar="60" haze="0" shadowclip="4"/>
  </visual>"""
SKY_XML = '<texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.74 0.93" rgb2="0.96 0.97 0.99" width="512" height="3072"/>'


# ---------------------------------------------------------------------------
# geometry helpers

class _Body:
    """A static body of decor geoms, in its own frame (pos, yaw in degrees)."""

    def __init__(self, name: str, pos=(0.0, 0.0, 0.0), yaw: float = 0.0, group: int | None = None):
        self.name, self.pos, self.yaw = name, (tuple(pos) + (0.0,))[:3], yaw
        self.g: list[str] = []
        self.clutter: dict = {}            # ball-coloured parts: name -> local xyz
        self.group = f' group="{group}"' if group is not None else ""

    def world(self, local) -> tuple:
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        x, y, z = local
        return (self.pos[0] + c * x - s * y, self.pos[1] + s * x + c * y, self.pos[2] + z)

    def _add(self, typ, mat, size, pos=None, extra=""):
        p = f' pos="{_f(*pos)}"' if pos is not None else ""
        self.g.append(f'<geom class="decor" type="{typ}" material="{mat}" size="{_f(*size)}"{p}{extra}/>')
        return self

    def box(self, pos, half, mat, yaw: float = 0.0, euler=None):
        e = f' euler="{_f(*[math.radians(a) for a in euler])}"' if euler else \
            (f' euler="0 0 {_f(math.radians(yaw))}"' if yaw else "")
        return self._add("box", mat, half, pos, e)

    def span(self, lo, hi, mat):
        """A box from corner lo to corner hi, turned so its thinnest side is its local z: MuJoCo maps a 2D texture
        from the box's local x, y, so only the local +-z faces show it undistorted (walls, doors, cabinet fronts)."""
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        c, h = (lo + hi) / 2, np.abs(hi - lo) / 2
        k = int(np.argmin(h))
        if k == 2:
            return self.box(c, h, mat)
        size, axes = ((h[1], h[2], h[0]), "0 1 0 0 0 1") if k == 0 else ((h[0], h[2], h[1]), "1 0 0 0 0 1")
        return self._add("box", mat, size, c, f' xyaxes="{axes}"')

    def cyl(self, pos, r, half_h, mat, euler=None):
        e = f' euler="{_f(*[math.radians(a) for a in euler])}"' if euler else ""
        return self._add("cylinder", mat, (r, half_h), pos, e)

    def cap(self, a, b, r, mat):
        self.g.append(f'<geom class="decor" type="capsule" material="{mat}" size="{_f(r)}" fromto="{_f(*a, *b)}"/>')
        return self

    def rod(self, a, b, r, mat):
        self.g.append(f'<geom class="decor" type="cylinder" material="{mat}" size="{_f(r)}" fromto="{_f(*a, *b)}"/>')
        return self

    def sph(self, pos, r, mat):
        return self._add("sphere", mat, (r,), pos)

    def ell(self, pos, half, mat, euler=None):
        e = f' euler="{_f(*[math.radians(a) for a in euler])}"' if euler else ""
        return self._add("ellipsoid", mat, half, pos, e)

    def xml(self) -> str:
        geoms = "".join(g.replace('class="decor"', f'class="decor"{self.group}') for g in self.g) if self.group \
            else "".join(self.g)
        return (f'<body name="{self.name}" pos="{_f(*self.pos)}" euler="0 0 {_f(math.radians(self.yaw))}">'
                + geoms + "</body>")


def _wall(name, along: str, c: float, a0: float, a1: float, openings, mat) -> _Body:
    """A wall along x (at y = c) or along y (at x = c) from a0 to a1, full height, with rectangular openings
    (s0, s1, z0, z1)."""
    b = _Body(name)
    h = WALL_T / 2

    def piece(s0, s1, z0, z1):
        if s1 - s0 < 1e-3 or z1 - z0 < 1e-3:
            return
        if along == "x":
            b.span((s0, c - h, z0), (s1, c + h, z1), mat)
        else:
            b.span((c - h, s0, z0), (c + h, s1, z1), mat)

    s = a0
    for s0, s1, z0, z1 in sorted(openings):
        piece(s, s0, 0.0, CEIL)
        piece(s0, s1, 0.0, z0)
        piece(s0, s1, z1, CEIL)
        s = s1
    piece(s, a1, 0.0, CEIL)
    # skirting on the room side, broken only by doors
    sk = c - math.copysign(h + 0.008, c)

    def skirt(sa, sb):
        if sb - sa > 0.01:
            if along == "x":
                b.span((sa, sk - 0.008, 0.0), (sb, sk + 0.008, 0.1), "h_white")
            else:
                b.span((sk - 0.008, sa, 0.0), (sk + 0.008, sb, 0.1), "h_white")

    s = a0
    for s0, s1, z0, z1 in sorted(openings):
        if z0 < 0.05:
            skirt(s, s0)
            s = s1
    skirt(s, a1)
    return b


def _opening_frame(b: _Body, along: str, c: float, s0, s1, z0, z1, mullions: int = 1, door: bool = False,
                   inward: float = 1.0):
    """Frame, glazing bars, glass and (windows) a sill, in the wall's opening."""
    fw, fd = 0.05, WALL_T * 0.6

    def bar(sa, sb, za, zb, mat="h_alu", d=fd, off=0.0):
        if along == "x":
            b.span((sa, c - d / 2 + off, za), (sb, c + d / 2 + off, zb), mat)
        else:
            b.span((c - d / 2 + off, sa, za), (c + d / 2 + off, sb, zb), mat)

    bar(s0, s1, z1 - fw, z1)
    bar(s0, s1, z0, z0 + fw)
    bar(s0, s0 + fw, z0, z1)
    bar(s1 - fw, s1, z0, z1)
    for k in range(1, mullions + 1):
        sm = s0 + (s1 - s0) * k / (mullions + 1)
        bar(sm - fw / 2, sm + fw / 2, z0, z1)
    if not door:
        bar(s0, s1, (z0 + z1) / 2 + 0.25 - fw / 2, (z0 + z1) / 2 + 0.25 + fw / 2)        # transom
        bar(s0 - 0.04, s1 + 0.04, z0 - 0.03, z0, "h_white", d=WALL_T + 0.08, off=inward * 0.04)   # sill
    # no glass: MuJoCo draws a transparent pane into the shadow maps as if it were opaque, so it would keep the sun
    # out (and at 12% opacity it barely shows)


# ---------------------------------------------------------------------------
# furniture

def sofa(name, pos, yaw, length=2.4) -> _Body:
    """A three-seat sofa: front toward local +x."""
    b = _Body(name, pos, yaw)
    L, D = length / 2, 0.47
    b.span((-D, -L, 0.06), (D, L, 0.3), "h_sofa")                         # base
    for k in range(3):                                                    # seat cushions
        y0 = -L + 0.16 + k * (2 * L - 0.32) / 3
        b.span((-D + 0.2, y0 + 0.01, 0.3), (D - 0.01, y0 + (2 * L - 0.32) / 3 - 0.01, 0.45), "h_sofa")
        b.span((-D + 0.02, y0 + 0.02, 0.45), (-D + 0.24, y0 + (2 * L - 0.32) / 3 - 0.02, 0.86), "h_sofa",)
    b.span((-D, -L, 0.3), (-D + 0.18, L, 0.78), "h_sofa")                 # back frame
    for s in (-1, 1):                                                     # arms
        b.span((-D, s * L - 0.16 * (s > 0), 0.06), (D, s * L + 0.16 * (s < 0), 0.62), "h_sofa")
        for x in (-D + 0.06, D - 0.06):
            b.cyl((x, s * (L - 0.08), 0.03), 0.02, 0.03, "h_charcoal")   # feet
    return b


def armchair(name, pos, yaw) -> _Body:
    b = _Body(name, pos, yaw)
    D, W = 0.42, 0.42
    b.span((-D, -W, 0.18), (D, W, 0.42), "h_leather")
    b.span((-D, -W, 0.42), (-D + 0.18, W, 0.92), "h_leather")
    for s in (-1, 1):
        b.span((-D, s * W - 0.12 * (s > 0), 0.18), (D, s * W + 0.12 * (s < 0), 0.62), "h_leather")
    for x in (-D + 0.05, D - 0.05):
        for y in (-W + 0.05, W - 0.05):
            b.rod((x, y, 0.0), (x, y, 0.18), 0.018, "h_table")
    return b


def coffee_table(name, pos, yaw, items=True) -> _Body:
    b = _Body(name, pos, yaw)
    b.span((-0.28, -0.55, 0.38), (0.28, 0.55, 0.42), "h_table")
    b.span((-0.25, -0.52, 0.1), (0.25, 0.52, 0.12), "h_table")            # lower shelf
    for x in (-0.25, 0.25):
        for y in (-0.52, 0.52):
            b.span((x - 0.02, y - 0.02, 0.0), (x + 0.02, y + 0.02, 0.38), "h_table")
    if items:
        b.span((-0.12, -0.35, 0.42), (0.1, -0.08, 0.45), "h_blue")        # books
        b.span((-0.1, -0.32, 0.45), (0.09, -0.1, 0.47), "h_offwhite")
        b.cyl((0.05, 0.25, 0.47), 0.04, 0.05, "h_ceramic")               # mug / vase
        b.span((-0.2, 0.05, 0.12), (0.15, 0.4, 0.2), "h_wicker")          # basket on the shelf
    return b


def floor_lamp(name, pos) -> _Body:
    b = _Body(name, pos)
    b.cyl((0, 0, 0.012), 0.16, 0.012, "h_charcoal")
    b.rod((0, 0, 0.02), (0, 0, 1.45), 0.012, "h_charcoal")
    b.cyl((0, 0, 1.55), 0.2, 0.14, "h_shade")
    return b


def table_lamp(b: _Body, pos):
    x, y, z = pos
    b.cyl((x, y, z + 0.1), 0.06, 0.1, "h_ceramic")
    b.cyl((x, y, z + 0.32), 0.15, 0.12, "h_shade")


def side_table(name, pos, lamp=True) -> _Body:
    b = _Body(name, pos)
    b.cyl((0, 0, 0.55), 0.24, 0.02, "h_table")
    b.rod((0, 0, 0.02), (0, 0, 0.55), 0.03, "h_table")
    b.cyl((0, 0, 0.015), 0.18, 0.015, "h_table")
    if lamp:
        table_lamp(b, (0, 0, 0.57))
    return b


def plant(name, pos, scale=1.0, light=True, seed=0) -> _Body:
    """A potted fig: pot, soil, stems and leaf clusters (dark and mid greens; the mid green is in the hue window
    but darker than the detector's value floor indoors)."""
    rng = np.random.default_rng(seed)
    b = _Body(name, pos)
    s = scale
    b.cyl((0, 0, 0.18 * s), 0.18 * s, 0.18 * s, "h_terracotta" if seed % 2 else "h_ceramic")
    b.cyl((0, 0, 0.355 * s), 0.16 * s, 0.01, "h_soil")
    for k in range(7):
        a = 2 * math.pi * k / 7 + rng.uniform(-0.3, 0.3)
        r = rng.uniform(0.1, 0.32) * s
        z = rng.uniform(0.75, 1.6) * s
        top = (r * math.cos(a), r * math.sin(a), z)
        b.rod((0, 0, 0.35 * s), top, 0.01 * s, "h_bark")
        for j in range(3):
            c = (top[0] + rng.normal(0, 0.1 * s), top[1] + rng.normal(0, 0.1 * s), top[2] + rng.normal(0, 0.08 * s))
            b.ell(c, (0.14 * s, 0.1 * s, 0.03 * s), "h_leaf" if (light and rng.uniform() < 0.4) else "h_leaf_dark",
                  euler=(rng.uniform(-40, 40), rng.uniform(-40, 40), rng.uniform(0, 180)))
    return b


def bookcase(name, pos, yaw, width=2.6, height=2.3, depth=0.38, seed=1) -> _Body:
    """A built-in bookcase against the wall: front toward local +x. Books, baskets, a vase, a plant, and a
    tennis ball on one shelf (ball-coloured clutter at the real ball's size)."""
    rng = np.random.default_rng(seed)
    b = _Body(name, pos, yaw)
    W, D = width / 2, depth / 2
    t = 0.022
    b.span((-D, -W, 0), (-D + 0.012, W, height), "h_oak")                  # back panel
    shelves = np.linspace(0.08, height - t, 6)
    for z in shelves:
        b.span((-D, -W, z - t), (D, W, z), "h_oak")
    b.span((-D, -W, 0), (D, W, 0.08), "h_white")                           # plinth
    bays = np.linspace(-W, W, 4)
    for y in bays:
        b.span((-D, y - t / 2, 0), (D, y + t / 2, height), "h_oak")
    for i, z in enumerate(shelves[:-1]):
        gap = shelves[i + 1] - z - t
        for j in range(3):
            y0, y1 = bays[j] + t, bays[j + 1] - t
            fill = rng.uniform(0.35, 0.95)
            hb = gap * rng.uniform(0.6, 0.85)
            if (i, j) == (2, 1):                                           # the shelf with the tennis ball
                b.span((-D + 0.03, y0, z), (-D + 0.25, y0 + 0.35, z + hb), "h_books")
                b.sph((0.0, y0 + 0.5, z + 0.034), 0.034, "ball")
                b.clutter["tennis ball on a shelf"] = (0.0, y0 + 0.5, z + 0.034)
                b.cyl((0.0, y1 - 0.15, z + 0.1), 0.06, 0.1, "h_ceramic")
                continue
            if (i, j) in ((0, 0), (0, 2)):                                 # baskets on the bottom shelf
                b.span((-D + 0.03, y0 + 0.03, z), (D - 0.03, y1 - 0.03, z + gap * 0.75), "h_wicker")
                continue
            b.span((-D + 0.03, y0, z), (-D + 0.25, y0 + fill * (y1 - y0), z + hb), "h_books")
            if fill < 0.6:
                b.cyl((0.0, y1 - 0.12, z + 0.09), 0.05, 0.09, "h_ceramic" if rng.uniform() < 0.5 else "h_teal")
    return b


def tv_wall(name, pos) -> _Body:
    """Console, 65-inch TV on the wall (a dim landscape on screen), soundbar, a plant: front toward +x."""
    b = _Body(name, pos)
    b.span((-0.22, -1.0, 0.1), (0.22, 1.0, 0.52), "h_table")
    b.span((-0.2, -0.98, 0.0), (0.2, 0.98, 0.1), "h_charcoal")
    for y in (-0.5, 0.5):
        b.span((0.22, y - 0.47, 0.13), (0.225, y + 0.47, 0.49), "h_table")     # doors
    b.span((-0.18, -0.55, 0.52), (-0.08, 0.55, 0.59), "h_black")              # soundbar
    b.span((-0.215, -0.725, 0.93), (-0.185, 0.725, 1.76), "h_black")          # TV
    b.span((-0.186, -0.71, 0.945), (-0.184, 0.71, 1.745), "h_tv")
    b.cyl((0.0, 0.85, 0.62), 0.07, 0.1, "h_ceramic")
    for k in range(5):
        a = k * 1.25
        b.ell((0.03 * math.cos(a), 0.85 + 0.03 * math.sin(a), 0.82 + 0.03 * k), (0.06, 0.03, 0.1), "h_leaf_dark",
              euler=(20 * math.cos(a), 20 * math.sin(a), 0))
    return b


def kitchen(name) -> _Body:
    """Counters along the -y wall with a sink under the window, upper cabinets, fridge, backsplash."""
    b = _Body(name)
    yw = Y0 + WALL_T / 2
    x0, x1 = -6.8, -0.6
    d = 0.6
    b.span((x0, yw, 0.0), (x1, yw + d - 0.05, 0.1), "h_charcoal")                        # toe kick
    b.span((x0, yw, 0.1), (x1, yw + d - 0.02, 0.88), "h_cabinet")
    n = int(round((x1 - x0) / 0.6))
    for k in range(n):                                                                   # door seams + handles
        xa = x0 + k * (x1 - x0) / n
        b.span((xa - 0.002, yw + d - 0.021, 0.12), (xa + 0.002, yw + d - 0.015, 0.86), "h_charcoal")
        b.rod((xa + 0.1, yw + d - 0.0, 0.72), (xa + 0.3, yw + d - 0.0, 0.72), 0.006, "h_steel")
    b.span((x0 - 0.02, yw, 0.88), (x1 + 0.02, yw + d + 0.02, 0.92), "h_marble")         # worktop
    b.span((-4.2, yw + 0.12, 0.84), (-3.4, yw + 0.5, 0.921), "h_steel")                # sink
    b.span((-4.15, yw + 0.15, 0.85), (-3.45, yw + 0.47, 0.922), "h_charcoal")
    b.cap((-3.8, yw + 0.08, 0.92), (-3.8, yw + 0.08, 1.2), 0.012, "h_steel")             # tap
    b.cap((-3.8, yw + 0.08, 1.2), (-3.8, yw + 0.25, 1.15), 0.012, "h_steel")
    b.span((-2.1, yw + 0.05, 0.92), (-1.3, yw + 0.55, 0.926), "h_black")                # hob
    b.span((x0, yw, 0.92), (x1, yw + 0.015, 1.45), "h_tile")                            # backsplash
    for xa, xb in ((x0, -4.75), (-2.85, x1)):                                           # uppers
        b.span((xa, yw, 1.45), (xb, yw + 0.35, 2.3), "h_cabinet")
        m = max(1, int(round((xb - xa) / 0.6)))
        for k in range(1, m):
            xs = xa + k * (xb - xa) / m
            b.span((xs - 0.002, yw + 0.349, 1.47), (xs + 0.002, yw + 0.356, 2.28), "h_charcoal")
    b.span((-7.88, yw, 0.0), (-6.92, yw + 0.72, 1.9), "h_steel")                         # fridge
    b.span((-7.4 - 0.002, yw + 0.72, 0.05), (-7.4 + 0.002, yw + 0.727, 1.88), "h_charcoal")
    for xs in (-7.47, -7.33):
        b.rod((xs, yw + 0.76, 0.9), (xs, yw + 0.76, 1.5), 0.012, "h_steel")
    b.cyl((-1.0, yw + 0.3, 1.0), 0.09, 0.08, "h_terracotta")                            # herbs on the worktop
    for k in range(6):
        a = k * 1.05
        b.ell((-1.0 + 0.06 * math.cos(a), yw + 0.3 + 0.06 * math.sin(a), 1.16), (0.08, 0.03, 0.06), "h_leaf",
              euler=(30 * math.cos(a), 30 * math.sin(a), 0))
    return b


def island(name, pos) -> _Body:
    """The kitchen island (2.8 x 0.65 m), with a bowl of lemons and limes (ball-coloured), a vase, pendants
    above, and three stools on the field side."""
    b = _Body(name, pos)
    L, D = 1.4, 0.325
    b.span((-L + 0.05, -D + 0.05, 0), (L - 0.05, D - 0.05, 0.1), "h_charcoal")
    b.span((-L + 0.02, -D + 0.02, 0.1), (L - 0.02, D - 0.02, 0.88), "h_cabinet")
    b.span((-L, -D, 0.88), (L, D + 0.08, 0.92), "h_marble")              # overhang on the stools' side
    # bowl of fruit
    b.cyl((-0.5, 0.0, 0.95), 0.15, 0.03, "h_ceramic")
    b.clutter["lemons and limes"] = (-0.5, 0.0, 0.99)
    rng = np.random.default_rng(3)
    for k in range(7):
        a = 2 * math.pi * k / 7
        r = 0.08 if k < 6 else 0.0
        c = (-0.5 + r * math.cos(a), r * math.sin(a), 0.99 + (0.03 if k == 6 else 0.0))
        if k % 2 == 0:
            b.ell(c, (0.042, 0.032, 0.032), "h_lemon", euler=(0, 0, math.degrees(a) + rng.uniform(-20, 20)))
        else:
            b.sph(c, 0.03, "h_lime")
    # vase with stems
    b.cyl((0.6, 0.0, 1.02), 0.05, 0.1, "h_glass")
    for k in range(5):
        a = k * 1.3
        tip = (0.6 + 0.1 * math.cos(a), 0.1 * math.sin(a), 1.4 + 0.05 * k)
        b.rod((0.6, 0.0, 1.0), tip, 0.004, "h_leaf")
        b.sph(tip, 0.035, "h_offwhite")
    # stools on the +y side (outside the field: their edge stays 4 cm short of y = -3)
    for x in (-0.95, 0.0, 0.95):
        y = D + 0.27
        b.cyl((x, y, 0.66), 0.17, 0.025, "h_table")
        for a in (45, 135, 225, 315):
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            b.rod((x + 0.11 * ca, y + 0.11 * sa, 0.64), (x + 0.15 * ca, y + 0.15 * sa, 0.0), 0.012, "h_black")
        b.cyl((x, y, 0.25), 0.14, 0.006, "h_black")
    # pendants
    for x in (-0.7, 0.7):
        b.rod((x, 0.0, CEIL), (x, 0.0, 2.05), 0.004, "h_black")
        b.cyl((x, 0.0, 1.98), 0.14, 0.07, "h_black")
        b.cyl((x, 0.0, 1.905), 0.12, 0.006, "h_bulb")
    return b


def dining(name, pos) -> _Body:
    """Dining table along y with four chairs, a pendant above (behind the robot)."""
    b = _Body(name, pos)
    b.span((-0.45, -1.0, 0.72), (0.45, 1.0, 0.76), "h_table")
    for x in (-0.38, 0.38):
        for y in (-0.92, 0.92):
            b.span((x - 0.03, y - 0.03, 0), (x + 0.03, y + 0.03, 0.72), "h_table")
    for cx, cy, yaw in ((-0.62, -0.45, 0), (-0.62, 0.45, 0), (0.0, -1.28, 90), (0.0, 1.28, -90)):
        c = math.radians(yaw)
        R = np.array([[math.cos(c), -math.sin(c)], [math.sin(c), math.cos(c)]])

        def at(x, y, z):
            p = R @ np.array([x, y])
            return (cx + p[0], cy + p[1], z)
        b.box(at(0, 0, 0.45), (0.21, 0.21, 0.02), "h_oak", yaw=yaw)
        b.box(at(-0.19, 0, 0.75), (0.02, 0.2, 0.18), "h_oak", yaw=yaw)
        for x in (-0.18, 0.18):
            for y in (-0.18, 0.18):
                b.rod(at(x, y, 0.0), at(x, y, 0.44), 0.015, "h_oak")
    b.rod((0, 0, CEIL), (0, 0, 1.6), 0.004, "h_black")
    b.ell((0, 0, 1.52), (0.3, 0.3, 0.1), "h_shade")
    return b


def desk(name, pos) -> _Body:
    b = _Body(name, pos)
    b.span((-0.7, -0.3, 0.72), (0.7, 0.3, 0.75), "h_oak")
    for x in (-0.66, 0.66):
        b.span((x - 0.02, -0.28, 0), (x + 0.02, 0.28, 0.72), "h_white")
    b.span((-0.32, 0.12, 0.95), (0.32, 0.14, 1.32), "h_black")           # monitor
    b.rod((0, 0.16, 0.75), (0, 0.16, 0.95), 0.015, "h_charcoal")
    b.span((-0.2, -0.12, 0.75), (0.2, 0.0, 0.765), "h_charcoal")          # keyboard
    table_lamp(b, (0.55, 0.12, 0.75))
    # office chair, pushed back
    b.cyl((0, -0.6, 0.47), 0.24, 0.04, "h_charcoal")
    b.box((0, -0.82, 0.8), (0.22, 0.03, 0.26), "h_charcoal")
    b.rod((0, -0.6, 0.08), (0, -0.6, 0.45), 0.025, "h_steel")
    for k in range(5):
        a = 2 * math.pi * k / 5
        b.rod((0, -0.6, 0.08), (0.28 * math.cos(a), -0.6 + 0.28 * math.sin(a), 0.04), 0.015, "h_black")
    return b


def piano(name, pos, yaw) -> _Body:
    b = _Body(name, pos, yaw)
    b.span((-0.3, -0.75, 0.0), (0.0, 0.75, 1.25), "h_black")
    b.span((0.0, -0.72, 0.68), (0.25, 0.72, 0.76), "h_black")
    b.span((0.02, -0.7, 0.76), (0.2, 0.7, 0.785), "h_white")              # keys
    b.span((0.4, -0.4, 0.45), (0.75, 0.4, 0.5), "h_black")                # bench
    for x in (0.43, 0.72):
        for y in (-0.37, 0.37):
            b.span((x - 0.02, y - 0.02, 0), (x + 0.02, y + 0.02, 0.45), "h_black")
    b.span((-0.06, -0.2, 1.25), (-0.02, 0.2, 1.55), "h_art2")              # a print on top
    return b


def sideboard(name, pos, yaw) -> _Body:
    b = _Body(name, pos, yaw)
    b.span((-0.22, -0.9, 0.12), (0.22, 0.9, 0.8), "h_table")
    for y in (-0.8, 0.8):
        b.span((-0.18, y - 0.03, 0), (-0.14, y + 0.03, 0.12), "h_black")
        b.span((0.14, y - 0.03, 0), (0.18, y + 0.03, 0.12), "h_black")
    table_lamp(b, (0.0, -0.55, 0.8))
    b.cyl((0.0, 0.5, 0.92), 0.08, 0.12, "h_teal")
    return b


def toys(name, pos) -> _Body:
    """A toy basket with balls (one lime-green, ball-coloured but bigger), and a yellow dump truck nearby."""
    b = _Body(name, pos)
    b.span((-0.25, -0.18, 0), (0.25, 0.18, 0.3), "h_wicker")
    b.sph((-0.1, 0.0, 0.33), 0.07, "h_red")
    b.sph((0.08, 0.04, 0.34), 0.055, "h_lime_toy")
    b.clutter["lime toy ball"] = (0.08, 0.04, 0.34)
    b.sph((0.12, -0.08, 0.32), 0.05, "h_blue")
    return b


def toy_truck(name, pos, yaw) -> _Body:
    b = _Body(name, pos, yaw)
    b.clutter["yellow toy truck"] = (0.0, 0.0, 0.12)
    b.span((-0.16, -0.08, 0.05), (0.16, 0.08, 0.1), "h_toy_yellow")
    b.span((0.06, -0.075, 0.1), (0.16, 0.075, 0.2), "h_toy_yellow")        # cab
    b.box((-0.06, 0, 0.16), (0.1, 0.08, 0.05), "h_toy_yellow", euler=(0, -15, 0))   # tipper
    for x in (-0.11, 0.11):
        for y in (-0.085, 0.085):
            b.cyl((x, y, 0.045), 0.045, 0.015, "h_black", euler=(90, 0, 0))
    return b


def rug(name, pos, half) -> _Body:
    b = _Body(name, pos)
    b.box((0, 0, 0.004), (half[0], half[1], 0.004), "h_rug")
    return b


def art(b: _Body, along, c, s, z, w, h, mat, inward):
    """A framed print on a wall."""
    t = 0.025
    if along == "x":
        b.span((s - w / 2 - 0.03, c + inward * 0.005, z - h / 2 - 0.03),
               (s + w / 2 + 0.03, c + inward * (t + 0.005), z + h / 2 + 0.03), "h_black")
        b.span((s - w / 2, c + inward * (t + 0.006), z - h / 2), (s + w / 2, c + inward * (t + 0.008), z + h / 2), mat)
    else:
        b.span((c + inward * 0.005, s - w / 2 - 0.03, z - h / 2 - 0.03),
               (c + inward * (t + 0.005), s + w / 2 + 0.03, z + h / 2 + 0.03), "h_black")
        b.span((c + inward * (t + 0.006), s - w / 2, z - h / 2), (c + inward * (t + 0.008), s + w / 2, z + h / 2), mat)


def curtains(b: _Body, along, c, s0, s1, z1, inward):
    """Two linen panels drawn to the sides of a window."""
    for sa, sb in ((s0 - 0.35, s0 + 0.1), (s1 - 0.1, s1 + 0.35)):
        if along == "x":
            b.span((sa, c + inward * 0.1, 0.05), (sb, c + inward * 0.14, z1 + 0.25), "h_linen")
        else:
            b.span((c + inward * 0.1, sa, 0.05), (c + inward * 0.14, sb, z1 + 0.25), "h_linen")
    if along == "x":
        b.rod((s0 - 0.4, c + inward * 0.12, z1 + 0.28), (s1 + 0.4, c + inward * 0.12, z1 + 0.28), 0.012, "h_black")
    else:
        b.rod((c + inward * 0.12, s0 - 0.4, z1 + 0.28), (c + inward * 0.12, s1 + 0.4, z1 + 0.28), 0.012, "h_black")


# ---------------------------------------------------------------------------
# people

def person(name, pos, yaw, pose="stand", shirt="h_shirt_red", pants="h_jeans", skin="h_skin1",
           hair="h_hair_dark", shoe="h_shoe", seat=0.45, hand=(0.3, 0.0, 1.0)) -> _Body:
    """A person from primitives (~1.75 m), facing local +x. Poses: "stand", "sit" (on a seat of height `seat`),
    "throw" (an underhand toss: the right hand cupped under `hand`, where the ball leaves; left foot forward)."""
    b = _Body(name, pos, yaw)
    if pose == "sit":
        hz = seat + 0.1
        for s in (1, -1):
            b.cap((0.0, s * 0.1, hz), (0.42, s * 0.1, hz + 0.02), 0.075, pants)
            b.cap((0.45, s * 0.1, hz - 0.02), (0.5, s * 0.1, 0.1), 0.058, pants)
            b.box((0.55, s * 0.1, 0.035), (0.12, 0.048, 0.035), shoe)
        dz = hz - 0.95
        lean = 0.03
    else:
        dz, lean = 0.0, 0.0
        if pose == "throw":
            legs = {1: ((0.0, 0.1, 0.92), (0.13, 0.11, 0.5), (0.22, 0.12, 0.1)),
                    -1: ((0.0, -0.1, 0.92), (-0.06, -0.11, 0.5), (-0.14, -0.12, 0.1))}
        else:
            legs = {s: ((0.0, s * 0.1, 0.92), (0.01, s * 0.1, 0.5), (0.0, s * 0.1, 0.1)) for s in (1, -1)}
        for s, (hip, knee, ankle) in legs.items():
            b.cap(hip, knee, 0.075, pants)
            b.cap(knee, ankle, 0.058, pants)
            b.box((ankle[0] + 0.05, ankle[1], 0.035), (0.12, 0.048, 0.035), shoe)
    z = lambda v: v + dz                                       # noqa: E731
    b.ell((0.0, 0.0, z(0.96)), (0.12, 0.18, 0.12), pants)                         # hips
    b.ell((lean, 0.0, z(1.22)), (0.12, 0.19, 0.26), shirt)                        # torso
    b.cap((lean, 0.17, z(1.41)), (lean, -0.17, z(1.41)), 0.07, shirt)             # shoulders
    b.cap((lean, 0.0, z(1.46)), (lean, 0.0, z(1.55)), 0.05, skin)                 # neck
    b.ell((lean + 0.01, 0.0, z(1.65)), (0.095, 0.085, 0.115), skin)               # head
    b.ell((lean - 0.015, 0.0, z(1.7)), (0.1, 0.092, 0.085), hair)
    b.sph((lean + 0.1, 0.0, z(1.63)), 0.017, skin)                               # nose
    # arms: (shoulder, elbow, wrist, hand centre)
    if pose == "throw":
        hx, hy, hzz = hand
        arms = {1: ((0.0, 0.22, 1.41), (0.06, 0.3, 1.16), (0.17, 0.33, 0.97), (0.2, 0.34, 0.92)),
                -1: ((0.0, -0.22, 1.41), (0.1, -0.22, 1.15), (hx - 0.04, hy - 0.06, hzz - 0.06),
                     (hx, hy, hzz - 0.07))}
    elif pose == "sit":
        arms = {s: ((lean, s * 0.22, z(1.41)), (lean + 0.04, s * 0.25, z(1.15)), (lean + 0.3, s * 0.16, z(1.0)),
                    (lean + 0.36, s * 0.14, z(0.98))) for s in (1, -1)}
    else:
        arms = {s: ((0.0, s * 0.22, 1.41), (0.02, s * 0.25, 1.13), (0.06, s * 0.25, 0.88),
                    (0.07, s * 0.25, 0.82)) for s in (1, -1)}
    for s, (sh, el, wr, hd) in arms.items():
        b.cap(sh, el, 0.048, shirt)
        b.cap(el, wr, 0.04, skin)
        b.sph(hd, 0.045, skin)
    return b


# ---------------------------------------------------------------------------
# outside

def tree(name, pos, height=4.5, seed=0) -> _Body:
    rng = np.random.default_rng(seed)
    b = _Body(name, pos)
    b.rod((0, 0, 0), (0, 0, height * 0.55), 0.14, "h_bark")
    mats = ["h_foliage1", "h_foliage2", "h_foliage3", "h_foliage4"]
    for k in range(7):
        c = (rng.normal(0, 0.6), rng.normal(0, 0.6), height * rng.uniform(0.55, 0.85))
        r = rng.uniform(0.8, 1.3)
        b.ell(c, (r, r * rng.uniform(0.8, 1.1), r * 0.8), mats[int(rng.integers(4))])
    return b


def outside() -> list:
    out = []
    g = _Body("house_garden")
    m = 60.0                                     # to the far clipping plane (zfar): no floor plane past the garden
    for lo, hi in (((-m, -m), (X0 - WALL_T / 2, m)), ((X1 + WALL_T / 2, -m), (m, m)),
                   ((X0 - WALL_T / 2, Y1 + WALL_T / 2), (X1 + WALL_T / 2, m)),
                   ((X0 - WALL_T / 2, -m), (X1 + WALL_T / 2, Y0 - WALL_T / 2))):
        g.span((lo[0], lo[1], 0.0), (hi[0], hi[1], 0.01), "h_grass")
    g.span((X0 - 3.0, 1.6, 0.0), (X0 - WALL_T / 2, 5.0, 0.03), "h_patio")         # patio outside the door
    for k, x in enumerate(np.arange(-14.0, 12.0, 2.0)):                           # fence and hedges
        g.span((x, 12.4, 0), (x + 1.98, 12.45, 1.8), "h_fence")
        g.span((x, -12.45, 0), (x + 1.98, -12.4, 1.8), "h_fence")
    g.span((-15.0, -12.4, 0), (-14.95, 12.4, 1.8), "h_fence")
    g.span((-14.9, -9.0, 0), (-13.6, 9.0, 1.5), "h_hedge")
    g.span((-6.0, 11.0, 0), (6.0, 12.2, 1.4), "h_hedge")
    out.append(g)
    trees = [(-11.0, 3.8, 5.0), (-12.5, -0.8, 6.0), (-10.8, -5.5, 4.5), (-5.0, 8.8, 5.5), (-1.0, 9.8, 6.0),
             (2.8, 8.4, 4.8), (6.5, 9.0, 5.2), (-4.0, -9.5, 5.0), (1.5, -9.8, 5.5), (11.0, 1.0, 5.0),
             (-16.5, 6.0, 7.0), (-17.0, -4.0, 7.5)]
    out += [tree(f"house_tree{k}", (x, y), h, seed=k) for k, (x, y, h) in enumerate(trees)]
    return out


# ---------------------------------------------------------------------------
# the scene

# ball-coloured objects (world xyz, roughly their centre) for attributing false detections
CLUTTER: dict = {}


def lights_xml() -> str:
    """Seven lights (the fixed-function renderer draws at most eight, with the headlight's ambient): daylight through
    the +y windows (the only one whose shadows shape it: it gets in through the openings), four ceiling spots over the
    field (two cast shadows), and two fills that stand in for light bounced off the floor and the walls: MuJoCo has
    no global illumination, so without them a ball seen from below, or a wall facing away from the spots, is lit by
    the ambient term alone. The fills are dim and cast no shadows. (est)"""
    out = ['<light name="house_sun" directional="true" castshadow="true" pos="0 12 8" dir="-0.3 -0.75 -0.45" '
           'diffuse="0.8 0.77 0.7" specular="0.2 0.2 0.2"/>']
    for k, (x, y, sh) in enumerate(((-4.0, -1.2, True), (-4.0, 1.4, False), (0.5, -1.2, False), (0.5, 1.4, True))):
        out.append(f'<light name="house_spot{k}" pos="{_f(x, y, CEIL - 0.05)}" dir="0 0 -1" cutoff="70" exponent="3" '
                   f'attenuation="1 0.02 0.01" castshadow="{"true" if sh else "false"}" diffuse="0.42 0.4 0.36" '
                   f'specular="0.15 0.15 0.15"/>')
    out.append('<light name="house_bounce" directional="true" castshadow="false" pos="0 0 0.1" dir="0.1 0.2 1" '
               'diffuse="0.3 0.27 0.23" specular="0 0 0"/>')
    out.append('<light name="house_walls" directional="true" castshadow="false" pos="6 0 1.5" dir="-1 0.15 -0.25" '
               'diffuse="0.22 0.22 0.21" specular="0 0 0"/>')
    return "\n    ".join(out)


def worldbody_xml(p) -> str:
    """Room, furniture, people (the thrower at p.thrower_xy), outside: everything but the lights."""
    CLUTTER.clear()
    tx, ty = p.thrower_xy
    bodies: list[_Body] = []
    # --- shell: walls with windows and doors, ceiling with downlights
    win = [(-6.2, -4.4, 0.85, 2.35), (-2.4, -0.6, 0.85, 2.35), (1.6, 3.4, 0.85, 2.35)]
    patio = (2.3, 4.3, 0.0, 2.35)
    kwin = (-4.6, -3.0, 1.1, 2.25)
    door = (-2.9, -2.0, 0.0, 2.1)
    back = _wall("house_wall_back", "y", X0, Y0 - WALL_T / 2, Y1 + WALL_T / 2, [patio], "h_brick")
    _opening_frame(back, "y", X0, *patio, mullions=1, door=True)
    bodies.append(back)
    front = _wall("house_wall_front", "y", X1, Y0 - WALL_T / 2, Y1 + WALL_T / 2, [door], "h_wall")
    _opening_frame(front, "y", X1, *door, mullions=0, door=True)
    front.box((X1 + 0.28, -2.45 + 0.28, 1.04), (0.02, 0.44, 1.03), "h_white", yaw=55)    # the door, open
    bodies.append(front)
    left = _wall("house_wall_left", "x", Y1, X0, X1, win, "h_wall")
    for w in win:
        _opening_frame(left, "x", Y1, *w, mullions=1, inward=-1)
        curtains(left, "x", Y1, w[0], w[1], w[3], -1)
    bodies.append(left)
    right = _wall("house_wall_right", "x", Y0, X0, X1, [kwin], "h_wall")
    _opening_frame(right, "x", Y0, *kwin, mullions=1, inward=1)
    bodies.append(right)
    hall = _Body("house_hall")                                    # a dim hallway behind the open door
    hall.span((X1 + WALL_T / 2, -4.0, 0.0), (X1 + 2.0, -0.8, 0.02), "h_hall")
    hall.span((X1 + 2.0, -4.0, 0.0), (X1 + 2.1, -0.8, 2.6), "h_hall")
    hall.span((X1, -4.0, 2.6), (X1 + 2.1, -0.8, 2.7), "h_hall")
    hall.span((X1, -4.1, 0), (X1 + 2.1, -4.0, 2.7), "h_hall")
    hall.span((X1, -0.8, 0), (X1 + 2.1, -0.7, 2.7), "h_hall")
    bodies.append(hall)
    ceil = _Body("house_ceiling", group=CEILING_GROUP)        # its own group: the overview leaves it out
    ceil.span((X0 - WALL_T / 2, Y0 - WALL_T / 2, CEIL), (X1 + WALL_T / 2, Y1 + WALL_T / 2, CEIL + 0.1), "h_ceiling")
    for x in (-6.0, -3.5, -1.0, 1.5, 4.0, 6.5):
        for y in (-4.2, -1.2, 1.4, 4.0):
            ceil.cyl((x, y, CEIL - 0.003), 0.075, 0.003, "h_downlight")
            ceil.cyl((x, y, CEIL - 0.002), 0.09, 0.002, "h_white")
    ceil.cyl((-0.2, 0.1, CEIL - 0.01), 0.06, 0.01, "h_white")                            # smoke alarm
    for x in (-5.0, 2.5):
        ceil.span((x - 0.3, Y1 - 0.4, CEIL - 0.01), (x + 0.3, Y1 - 0.25, CEIL), "h_charcoal")   # vents
    bodies.append(ceil)
    # --- behind the thrower: bookcase, TV wall, plants, a bystander in a yellow shirt
    bodies.append(bookcase("house_bookcase", (X0 + WALL_T / 2 + 0.19, -2.9), 0))
    bodies.append(tv_wall("house_tv", (X0 + WALL_T / 2 + 0.22, 0.6)))
    bodies.append(plant("house_plant_patio", (X0 + 0.45, 1.95), 1.1, seed=1))
    art_b = _Body("house_art")
    art(art_b, "x", Y1 - WALL_T / 2, -3.4, 1.65, 0.9, 0.65, "h_art1", -1)
    art(art_b, "y", X1 - WALL_T / 2, 2.7, 1.6, 0.7, 0.9, "h_art2", -1)
    art(art_b, "x", Y0 + WALL_T / 2, 4.2, 1.5, 1.0, 0.7, "h_art1", 1)
    bodies.append(art_b)
    by2 = (X0 + 1.3, -0.95)
    bodies.append(person("house_bystander_standing", by2, 25, "stand", shirt="h_yellow_shirt", pants="h_khaki",
                         skin="h_skin3", hair="h_hair_brown", shoe="h_shoe_dark"))
    CLUTTER["bystander's yellow shirt"] = (by2[0], by2[1], 1.22)
    bodies.append(toy_truck("house_toy_truck", (X0 + 1.4, 3.45), 160))
    # --- living area (+y): rug, sofa with a bystander and a mustard cushion, coffee table, armchair, lamps
    bodies.append(rug("house_rug", (-3.6, 4.2), (1.9, 0.95)))
    sx, sy = -3.6, Y1 - WALL_T / 2 - 0.49
    bodies.append(sofa("house_sofa", (sx, sy), -90))
    cush = _Body("house_cushions", (sx, sy), -90)
    cush.box((-0.2, 0.9, 0.62), (0.07, 0.19, 0.19), "h_mustard", euler=(0, -12, 8))
    cush.box((-0.2, -0.95, 0.62), (0.07, 0.19, 0.19), "h_teal", euler=(0, -12, -6))
    cush.box((-0.25, -0.5, 0.6), (0.06, 0.17, 0.17), "h_rust", euler=(0, -14, 4))
    cush.clutter["mustard cushion"] = (-0.2, 0.9, 0.62)
    bodies.append(cush)
    bodies.append(person("house_bystander_sofa", (sx - 0.62, sy - 0.02), -90, "sit", shirt="h_shirt_red",
                         pants="h_grey_pants", skin="h_skin2", hair="h_hair_dark", shoe="h_shoe", seat=0.45))
    bodies.append(coffee_table("house_coffee_table", (sx, Y1 - 1.58), -90))
    bodies.append(armchair("house_armchair", (-0.9, 4.45), -125))
    bodies.append(side_table("house_side_table", (-5.3, 4.7)))
    bodies.append(floor_lamp("house_floor_lamp", (-1.75, 4.85)))
    bodies.append(plant("house_plant_living", (-6.75, 4.55), 1.25, seed=2))
    bodies.append(toys("house_toys", (0.4, 3.45)))
    bodies.append(desk("house_desk", (4.5, Y1 - WALL_T / 2 - 0.32)))
    bodies.append(plant("house_plant_desk", (6.0, 4.7), 1.0, seed=3))
    # --- kitchen (-y): counters, island with fruit (lemons and limes), fridge
    bodies.append(kitchen("house_kitchen"))
    bodies.append(island("house_island", (-4.0, -3.825)))
    bodies.append(piano("house_piano", (1.6, Y0 + WALL_T / 2 + 0.3), 90))
    bodies.append(plant("house_plant_piano", (3.2, Y0 + 0.5), 0.9, seed=4))
    # --- behind the robot: dining table, sideboard
    bodies.append(dining("house_dining", (X1 - 0.75, 0.0)))
    bodies.append(sideboard("house_sideboard", (X1 - WALL_T / 2 - 0.22, 2.6), 180))
    # --- the thrower: an underhand toss, the right hand under the release point (catch_world.Thrower.hand)
    bodies.append(person("thrower", (tx, ty), 0, "throw", shirt="h_shirt_blue", pants="h_jeans", skin="h_skin1",
                         hair="h_hair_brown", shoe="h_shoe", hand=(0.3, 0.0, 1.0)))
    bodies += outside()
    for b in bodies:
        for k, loc in b.clutter.items():
            CLUTTER[k] = b.world(loc)
    CLUTTER["bystander's yellow shirt"] = (by2[0], by2[1], 1.22)
    CLUTTER["trees through the patio door"] = (-11.0, 3.8, 3.0)
    CLUTTER["trees through the windows"] = (-5.0, 8.8, 3.0)
    held = (f'<body name="held_ball" mocap="true" pos="{_f(tx + 0.3, ty, -1.0)}">'
            f'<geom class="decor" type="sphere" size="{p.ball_spec.radius}" material="ball"/></body>')
    CLUTTER["the next ball in the thrower's hand"] = (tx + 0.3, ty, 1.0)
    return "\n    ".join([b.xml() for b in bodies] + [held])


def _look(pos, target) -> str:
    """xyaxes for a camera at pos looking at target, level horizon."""
    fwd = np.asarray(target, float) - np.asarray(pos, float)
    fwd /= np.linalg.norm(fwd)
    x = np.cross(fwd, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, fwd)
    return _f(*x, *y)


def cameras_xml() -> str:
    cams = {"house_overview": ((7.1, -5.3, 3.2), (-3.0, 1.2, 0.4), 75),
            "house_thrower": ((-1.2, -2.6, 1.7), (-5.5, 0.6, 1.0), 70),
            "house_behind_robot": ((6.2, 0.8, 1.6), (-3.0, 0.0, 1.2), 70)}
    return "".join(f'<camera name="{n}" pos="{_f(*pos)}" xyaxes="{_look(pos, tg)}" fovy="{fov}"/>'
                   for n, (pos, tg, fov) in cams.items())


def render_views(out_dir: Path, cams=("house_overview", "house_thrower", "house_behind_robot", "stereo_left"),
                 size=(1920, 1080), dist: float = 6.5, ball=None, cutaway: bool = False):
    """Render the house (the robot at home, the thrower at dist) from the named cameras into out_dir. cutaway: the
    overview camera goes up above the walls and the ceiling is left out."""
    import mujoco
    from catch_world import AREA, HOME
    from robot_export import v11_params
    from robot_model import build_model
    p = v11_params(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, scene="house")
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    if ball is not None:
        j = m.joint("ball")
        d.qpos[j.qposadr[0]:j.qposadr[0] + 3] = ball
    hb = m.body("held_ball").mocapid[0]
    d.mocap_pos[hb] = (p.thrower_xy[0] + 0.3, p.thrower_xy[1], 1.0)
    mujoco.mj_forward(m, d)
    out_dir.mkdir(parents=True, exist_ok=True)
    opt = mujoco.MjvOption()
    if cutaway:
        opt.geomgroup[CEILING_GROUP] = 0
        cid = m.camera("house_overview").id
        pos, tgt = np.array([9.5, -10.0, 9.0]), np.array([-1.5, 0.3, 0.0])
        m.cam_pos[cid] = pos
        fwd = (tgt - pos) / np.linalg.norm(tgt - pos)
        x = np.cross(fwd, [0, 0, 1.0])
        x /= np.linalg.norm(x)
        R = np.column_stack([x, np.cross(x, fwd), -fwd])
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.reshape(-1))
        m.cam_quat[cid] = q
        m.cam_fovy[cid] = 62
        mujoco.mj_forward(m, d)
    paths = []
    for cam in cams:
        w, h = (1280, 800) if cam.startswith("stereo") else size
        r = mujoco.Renderer(m, h, w)
        r.update_scene(d, camera=cam, scene_option=opt)
        path = out_dir / f"{cam}.png"
        Image.fromarray(r.render()).save(path)
        r.close()
        paths.append(path)
    return paths


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("fetch", "overview", "views"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.what == "fetch":
        fetch(force=True)
    elif a.what == "overview":
        out = Path(a.out) if a.out else Path(__file__).with_name("renders")
        src = render_views(out, cams=("house_overview",), size=(2560, 1440), cutaway=True)[0]
        src.replace(out / "v11_house_overview.png")
        print(f"wrote {out / 'v11_house_overview.png'}")
    else:
        for path in render_views(Path(a.out or "."), ball=(-1.0, 0.0, 2.2)):
            print(path)


if __name__ == "__main__":
    main()
