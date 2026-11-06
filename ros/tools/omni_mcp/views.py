"""Pictures of the robot's data for the model (ai_robot_architecture.md): the occupancy map with the rooms, places,
tags and the robot drawn on it, and a lidar scan as a plan view. PIL only; the numbers go beside the picture.

Map pictures are north (map +y) up, east (+x) right, a metre grid with its coordinates on the edges; a pixel (u, v)
of the picture is map x = x0 + (u + 0.5) * m_per_px, y = y1 - (v + 0.5) * m_per_px (both given in the reply).
"""

from __future__ import annotations

import io
import math

import numpy as np
from PIL import Image, ImageDraw

FREE, UNKNOWN, OCC = (250, 250, 250), (200, 200, 205), (30, 30, 35)
ROOM = [(220, 60, 60), (40, 120, 220), (40, 160, 70), (200, 120, 20), (150, 60, 190), (20, 160, 170), (180, 150, 20)]


def grid_of(msg: dict) -> dict:
    """A nav_msgs/OccupancyGrid (rosbridge JSON) -> {cells (H x W int8, row 0 at the map's origin y), res, x0, y0,
    frame}."""
    info = msg["info"]
    w, h = int(info["width"]), int(info["height"])
    cells = np.asarray(msg["data"], dtype=np.int16).reshape(h, w)
    o = info["origin"]["position"]
    return dict(cells=cells, res=float(info["resolution"]), x0=float(o["x"]), y0=float(o["y"]),
                frame=msg.get("header", {}).get("frame_id") or "map", width=w, height=h)


def map_picture(g: dict, pose=None, rooms=None, places=None, tags=None, people=None, crop=None, max_px: int = 900,
                marks=None) -> tuple:
    """(png bytes, info): the map (crop = (x0, y0, x1, y1) in map m, or the known part) scaled to at most max_px."""
    cells, res = g["cells"], g["res"]
    known = np.argwhere(cells >= 0)
    if crop is None:
        if len(known):
            (r0, c0), (r1, c1) = known.min(axis=0), known.max(axis=0)
            crop = (g["x0"] + c0 * res - 0.5, g["y0"] + r0 * res - 0.5, g["x0"] + (c1 + 1) * res + 0.5,
                    g["y0"] + (r1 + 1) * res + 0.5)
        else:
            crop = (g["x0"], g["y0"], g["x0"] + g["width"] * res, g["y0"] + g["height"] * res)
    x0, y0, x1, y1 = crop
    scale = min(max_px / max(1e-6, x1 - x0), max_px / max(1e-6, y1 - y0))       # px per m
    W, H = max(1, int((x1 - x0) * scale)), max(1, int((y1 - y0) * scale))
    # sample the grid at each picture pixel's centre
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    mx, my = x0 + (us + 0.5) / scale, y1 - (vs + 0.5) / scale
    ci, ri = ((mx - g["x0"]) / res).astype(int), ((my - g["y0"]) / res).astype(int)
    inside = (ci >= 0) & (ci < g["width"]) & (ri >= 0) & (ri < g["height"])
    val = np.full((H, W), -1, np.int16)
    val[inside] = cells[ri[inside], ci[inside]]
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = UNKNOWN
    img[(val >= 0) & (val < 50)] = FREE
    img[val >= 50] = OCC
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    px = lambda x, y: ((x - x0) * scale, (y1 - y) * scale)                    # noqa: E731
    for k in range(math.ceil(x0), math.floor(x1) + 1):                        # the metre grid
        d.line([px(k, y0), px(k, y1)], fill=(170, 190, 230) if k else (90, 120, 220), width=1)
        d.text((px(k, y1)[0] + 2, 2), f"{k}", fill=(40, 70, 170))
    for k in range(math.ceil(y0), math.floor(y1) + 1):
        d.line([px(x0, k), px(x1, k)], fill=(170, 190, 230) if k else (90, 120, 220), width=1)
        d.text((2, px(x0, k)[1] + 2), f"{k}", fill=(40, 70, 170))
    for i, (name, pts) in enumerate(sorted((rooms or {}).items())):
        col = ROOM[i % len(ROOM)]
        q = [px(*p) for p in pts]
        d.line(q + q[:1], fill=col, width=2)
        cx, cy = np.mean([p[0] for p in pts]), np.mean([p[1] for p in pts])
        d.text(px(cx, cy), name, fill=col, anchor="mm")
    for name, (x, y, yaw) in (places or {}).items():
        u, v = px(x, y)
        d.ellipse([u - 4, v - 4, u + 4, v + 4], outline=(200, 0, 160), width=2)
        d.text((u + 6, v - 6), name, fill=(200, 0, 160))
    for t in tags or ():
        u, v = px(t["x"], t["y"])
        d.rectangle([u - 4, v - 4, u + 4, v + 4], fill=(0, 0, 0))
        d.text((u + 6, v), f"tag {t['id']}", fill=(0, 0, 0))
    for x, y in people or ():
        u, v = px(x, y)
        d.ellipse([u - 7, v - 7, u + 7, v + 7], outline=(230, 120, 0), width=3)
    for m in marks or ():
        if not isinstance(m, dict):              # [x, y] or [x, y, label], as scripts write them
            m = dict(x=m[0], y=m[1], label=m[2] if len(m) > 2 else None)
        u, v = px(m["x"], m["y"])
        d.line([u - 6, v - 6, u + 6, v + 6], fill=(220, 0, 0), width=2)
        d.line([u - 6, v + 6, u + 6, v - 6], fill=(220, 0, 0), width=2)
        if m.get("label"):
            d.text((u + 7, v + 2), str(m["label"]), fill=(220, 0, 0))
    if pose is not None:
        x, y, yaw = pose
        u, v = px(x, y)
        r = max(8.0, 0.22 * scale)
        d.ellipse([u - r, v - r, u + r, v + r], outline=(0, 150, 0), width=3)
        d.line([u, v, u + 1.8 * r * math.cos(yaw), v - 1.8 * r * math.sin(yaw)], fill=(0, 150, 0), width=3)
    out = io.BytesIO()
    im.save(out, format="PNG")
    info = dict(frame=g["frame"], crop_m=[round(c, 2) for c in crop], width_px=W, height_px=H,
                m_per_px=round(1.0 / scale, 4), north_up=True,
                pixel_to_map="x = crop_m[0] + (u + 0.5) * m_per_px, y = crop_m[3] - (v + 0.5) * m_per_px",
                legend=dict(white="free", dark="occupied", grey="unknown", green_circle="the robot (line: its heading)",
                            colored_outlines="rooms", magenta="saved places", black_squares="AprilTags",
                            orange_circles="people (lidar)", red_x="marks"))
    return out.getvalue(), info


def scan_summary(scan: dict, laser=(0.0, 0.0, 0.0), sectors: int = 36) -> dict:
    """A sensor_msgs/LaserScan (rosbridge JSON) -> the nearest return per sector in the base frame (bearing deg, + left;
    range m from the base's centre), and the points (base frame) for a picture."""
    r = np.array([np.nan if v is None else v for v in scan["ranges"]], dtype=float)
    a = scan["angle_min"] + np.arange(len(r)) * scan["angle_increment"]
    ok = np.isfinite(r) & (r > scan["range_min"]) & (r < scan["range_max"])
    lx, ly, lyaw = laser
    bx = lx + r[ok] * np.cos(a[ok] + lyaw)
    by = ly + r[ok] * np.sin(a[ok] + lyaw)
    bear = np.degrees(np.arctan2(by, bx))
    dist = np.hypot(bx, by)
    width = 360.0 / sectors
    out = []
    for k in range(sectors):
        lo = -180.0 + k * width
        sel = (bear >= lo) & (bear < lo + width)
        out.append(dict(bearing_deg=round(lo + width / 2, 1),
                        nearest_m=None if not sel.any() else round(float(dist[sel].min()), 2)))
    return dict(sectors=out, points=np.stack([bx, by], axis=1), n=int(ok.sum()), frame="base_link")


def scan_picture(pts: np.ndarray, people=None, max_range: float = 6.0, px: int = 600) -> bytes:
    """A plan view of a scan around the robot (forward up, left to the left), rings every metre."""
    im = Image.new("RGB", (px, px), (252, 252, 252))
    d = ImageDraw.Draw(im)
    s = px / (2 * max_range)
    c = px / 2
    to = lambda x, y: (c - y * s, c - x * s)                                   # noqa: E731  forward = up
    for k in range(1, int(max_range) + 1):
        d.ellipse([c - k * s, c - k * s, c + k * s, c + k * s], outline=(210, 215, 230))
        d.text((c + 3, c - k * s + 2), f"{k} m", fill=(120, 130, 170))
    for x, y in pts:
        if abs(x) < max_range and abs(y) < max_range:
            u, v = to(x, y)
            d.point((u, v), fill=(20, 20, 30))
    for x, y in people or ():
        u, v = to(x, y)
        d.ellipse([u - 8, v - 8, u + 8, v + 8], outline=(230, 120, 0), width=3)
    d.polygon([to(0.25, 0), to(-0.15, 0.12), to(-0.15, -0.12)], outline=(0, 150, 0), width=2)
    out = io.BytesIO()
    im.save(out, format="PNG")
    return out.getvalue()
