"""v11 (design.md 31): the catch game with the robot's code seeing rendered images, against the sim's sparse frames.

The same throws as sil_catch.py (home_floor B's grid: 6 / 6.5 / 7 m x apex 0.75 / 1 / 1.5 / 2 m x seeds x 6 throws,
the same seeds), flat floor, the SIL game (C++ firmware, the Jetson's code), in the house scene (house_scene.py):
  sparse     catch_world.CameraSim's pixel lists (blur, edge noise, drops, random clutter blobs; the house only
             occludes)
  rendered   rendered_camera.RenderedCameraSim: MuJoCo renders of the house through the sensor model, full images
Both feed the same robot code (omni_catch: detector, filter, behaviour). --acquire sets the detector's
acquire_candidates (1: the largest blob starts a track, the code before design.md 31; >1: the moving one among the n
largest does); default: the code's own default.

Per throw: caught, why not (no track: the behaviour never had a ready track before the ball arrived; not in place;
bounced), when the track became ready. Per run: the detector against the truth (sil_game.PerceptionScore), render
time per frame, wall time per throw.

    python rendered_catch.py run --frames rendered [--seeds 1] [--acquire 40] [--workers 16]
    python rendered_catch.py run --frames sparse --seeds 3
    python rendered_catch.py report
    python rendered_catch.py pictures          frames from both cameras with the detector's blobs, the overview
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from sil_catch import APEXES, DISTANCES, TOSSES, jobs  # noqa: E402

RESULTS = HERE / "results"
OUT = RESULTS / "rendered_catch.json"
RENDERS = HERE.parent / "renders"


def variant(frames: str, scene: str, acquire: int, size_gain: float = 1.0) -> str:
    return f"{frames}/{scene}/acq{acquire}" + (f"/size{size_gain:g}" if size_gain != 1.0 else "")


def effective(acquire, size_gain):
    """The detector settings a run uses: the given ones, else the robot code's defaults."""
    import sil_game  # noqa: F401  (puts ros/src on the path)
    from omni_catch.detector import DetectorConfig
    d = DetectorConfig()
    return (acquire or d.acquire_candidates), (size_gain or d.size_gain)


def job(args):
    floor_name, dist, apex, seed, frames, scene, acquire, size_gain = args
    from catch_world import WorldConfig
    from home_floor import GAME_FLOORS
    from sil_game import SilCatchGame                  # (puts ros/src on the path)
    from omni_catch.detector import DetectorConfig
    acquire, size_gain = effective(acquire, size_gain)
    dcfg = DetectorConfig(acquire_candidates=acquire, size_gain=size_gain)
    w0 = time.perf_counter()
    g = SilCatchGame(WorldConfig(dist=dist, apex_range=(apex, apex), seed=seed, floor=GAME_FLOORS[floor_name],
                                 frames=frames, scene=scene), dcfg=dcfg)
    res = g.run(TOSSES, t_max=150)
    p = g.world.p
    rim = p.funnel_catch_radius - p.ball_spec.radius
    tosses = []
    for r in res:
        ft = r.get("flight_time", math.nan)
        tt = r.get("t_track", math.nan)
        if r["caught"]:
            why = "caught"
        elif not np.isfinite(tt) or (np.isfinite(ft) and tt > ft):
            why = "no track"
        else:
            why = "reach" if not np.isfinite(r["pos_error"]) or r["pos_error"] > rim else "bounce"
        pc = r.get("perc", {})
        tosses.append(dict(caught=bool(r["caught"]), why=why, pos_error=float(r["pos_error"]), t_track=float(tt),
                           flight_time=float(ft), flight_frames=pc.get("flight", 0), hits=pc.get("hits", 0),
                           false=pc.get("false", 0)))
    ps = g.score.summary()
    cams = g.world.cams
    out = dict(kind=variant(frames, scene, acquire, size_gain), frames=frames, scene=scene, acquire=acquire,
               size_gain=size_gain, floor=floor_name,
               distance=dist, apex=apex, seed=seed, tosses=tosses, perception=ps,
               err=[float(x) for x in g.score.err], rtf=g.world.d.time / max(g.wall, 1e-9),
               wall=time.perf_counter() - w0, sim_time=g.world.d.time,
               link_errors=g.link.host.stats.errors)
    if hasattr(cams, "ms_render"):
        out["render_ms"] = [float(np.median(cams.ms_render)), float(np.percentile(cams.ms_render, 95))]
        out["sub_renders"] = float(np.mean(cams.n_sub))
    g.world.close()
    return out


def load():
    return json.loads(OUT.read_text()) if OUT.exists() else []


def save(rows):
    RESULTS.mkdir(exist_ok=True)
    key = lambda r: (r["kind"], r["floor"], r["seed"], r["distance"], r["apex"])      # noqa: E731
    new = {key(r) for r in rows}
    OUT.write_text(json.dumps([r for r in load() if key(r) not in new] + rows, indent=1))


def report(rows=None):
    rows = rows or load()
    print("v11 rendered vs sparse frames: the 6-7 m catch game, the same throws, SIL (C++ firmware + the Jetson's code)")
    kinds = list(dict.fromkeys(r["kind"] for r in rows))
    for kind in kinds:
        rr = [r for r in rows if r["kind"] == kind]
        ts = [t for r in rr for t in r["tosses"]]
        n = len(ts)
        if not n:
            continue
        k = np.mean([t["caught"] for t in ts])
        se = math.sqrt(max(k * (1 - k), 1e-9) / n)
        why = {w: sum(t["why"] == w for t in ts) for w in ("no track", "reach", "bounce")}
        seeds = sorted({r["seed"] // 100 - 90 for r in rr})
        tt = np.array([t["t_track"] for t in ts])
        tt = tt[np.isfinite(tt)]
        line = (f"  {kind:28s} {100 * k:5.1f}% +/- {100 * se:.1f} of {n} (seeds {seeds}): no track {why['no track']}, "
                f"not in place {why['reach']}, bounced {why['bounce']}")
        if len(tt):
            line += (f"; track ready {1000 * np.median(tt):.0f} ms after release (median, 95% "
                     f"{1000 * np.percentile(tt, 95):.0f})")
        print(line)
        flight = sum(r["perception"]["flight_frames"] for r in rr)
        hits = sum(r["perception"]["hits"] for r in rr)
        cam = sum(r["perception"]["cam_frames"] for r in rr)
        false = sum(r["perception"]["false"] for r in rr)
        err = np.array([e for r in rr for e in r["err"]])
        by = {}
        for r in rr:
            for src, c in r["perception"]["false_by"].items():
                by[src] = by.get(src, 0) + c
        size = [r["perception"].get("size_ratio_med") for r in rr if r["perception"].get("size_ratio_med")]
        print(f"      detector: ball found in {100 * hits / max(flight, 1):.1f}% of {flight} in-flight camera frames, "
              f"centre error median {np.median(err):.2f} px, 95% {np.percentile(err, 95):.2f} px; "
              + (f"blob radius / true {np.median(size):.3f}; " if size else "")
              + (f"{sum(r['perception']['false_starts'] for r in rr)} of "
                 f"{sum(r['perception']['starts'] for r in rr)} track starts not on the ball; "
                 if "starts" in rr[0]["perception"] else "") + "false blobs "
              f"{1000 * false / max(cam, 1):.0f} per 1000 camera frames"
              + (" (" + ", ".join(f"{s} {c}" for s, c in sorted(by.items(), key=lambda x: -x[1])[:6]) + ")"
                 if by else ""))
        for a in APEXES:
            ta = [t["caught"] for r in rr if r["apex"] == a for t in r["tosses"]]
            if ta:
                print(f"      apex {a:4.2f} m: {100 * np.mean(ta):.0f}% of {len(ta)}", end="")
        print()
        wall = sum(r["wall"] for r in rr) / n
        line = f"      {wall:.1f} s of wall time per throw (one process), x{np.median([r['rtf'] for r in rr]):.3f} real time"
        if "render_ms" in rr[0]:
            line += (f"; render + sensor model {np.median([r['render_ms'][0] for r in rr]):.1f} ms per camera frame "
                     f"(median of runs; 95% {np.median([r['render_ms'][1] for r in rr]):.0f}), "
                     f"{np.mean([r['sub_renders'] for r in rr]):.2f} renders per frame (motion blur)")
        print(line)


def pictures():
    """A few rendered frames mid-flight from both cameras with the detector's blob (red) and the true ball (blue),
    cropped and enlarged, plus each camera's whole frame, and the house overview."""
    from PIL import Image, ImageDraw
    from catch_world import WorldConfig
    import house_scene
    from sil_game import SilCatchGame
    RENDERS.mkdir(exist_ok=True)
    g = SilCatchGame(WorldConfig(dist=6.5, apex_range=(1.0, 1.0), seed=9000, frames="rendered", scene="house"))
    want = [0.25, 0.55]
    saved = []
    w = g.world
    while len(w.results) < 1 and w.d.time < 10:
        g.step()
        lv = g.last_view
        cur = w.cur
        if cur is None or not want or not g._last_pair:
            continue
        pair, blobs = g._last_pair
        rel = pair[0].t_cap - cur["t_release"]
        if rel < want[0] or not all(f.truth and f.truth[3] for f in pair):
            continue
        want.pop(0)
        crops = []
        for f in pair:
            b = next((x for x in blobs if x.cam == f.cam), None)
            u, v, r, _ = f.truth
            full = Image.fromarray(f.img)
            full.save(RENDERS / f"v11_house_cam{'LR'[f.cam]}_{int(1000 * rel)}ms.png")
            x0, y0 = int(max(0, min(1280 - 160, u - 80))), int(max(0, min(800 - 100, v - 50)))
            crop = full.crop((x0, y0, x0 + 160, y0 + 100)).resize((640, 400), Image.NEAREST)
            dr = ImageDraw.Draw(crop)
            s = 4
            dr.ellipse([(u - x0 - r) * s, (v - y0 - r) * s, (u - x0 + r) * s, (v - y0 + r) * s], outline=(0, 120, 255),
                       width=2)
            if b is not None:
                bb = b.b or 2
                dr.ellipse([(b.u - x0 - bb) * s, (b.v - y0 - bb) * s, (b.u - x0 + bb) * s, (b.v - y0 + bb) * s],
                           outline=(255, 0, 0), width=2)
            dr.text((6, 4), f"{'left' if f.cam == 0 else 'right'} camera, {1000 * rel:.0f} ms after release",
                    fill=(255, 255, 255))
            crops.append(crop)
        both = Image.new("RGB", (1290, 400), (255, 255, 255))
        both.paste(crops[0], (0, 0))
        both.paste(crops[1], (650, 0))
        path = RENDERS / f"v11_house_ball_{int(1000 * rel)}ms.png"
        both.save(path)
        saved.append(path)
    w.close()
    out = house_scene.render_views(RENDERS, cams=("house_overview",), size=(2560, 1440), cutaway=True)[0]
    out.replace(RENDERS / "v11_house_overview.png")
    for p in saved + [RENDERS / "v11_house_overview.png"]:
        print(f"wrote {p}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("run", "report", "pictures"))
    ap.add_argument("--frames", choices=("sparse", "rendered"), default="rendered")
    ap.add_argument("--scene", choices=("arena", "house"), default="house")
    ap.add_argument("--acquire", type=int, default=None, help="DetectorConfig.acquire_candidates (default: the code's)")
    ap.add_argument("--size-gain", type=float, default=None, help="DetectorConfig.size_gain (default: the code's)")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--first-seed", type=int, default=0)
    ap.add_argument("--floor", default="flat")
    ap.add_argument("--workers", type=int, default=None)
    a = ap.parse_args()
    if a.what == "report":
        report()
        return
    if a.what == "pictures":
        pictures()
        return
    todo = [(*j, a.frames, a.scene, a.acquire, a.size_gain) for j in jobs(a.floor, a.first_seed + a.seeds)
            if (j[3] - 9000) // 100 >= a.first_seed]
    acq, sg = effective(a.acquire, a.size_gain)
    workers = a.workers or (16 if a.frames == "rendered" else min(27, os.cpu_count() or 4))
    w0 = time.perf_counter()
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(job, todo))
    save(rows)
    print(f"{len(rows)} jobs ({len(rows) * TOSSES} throws) in {time.perf_counter() - w0:.0f} s on {workers} workers")
    report([r for r in load() if r["kind"] == variant(a.frames, a.scene, acq, sg)])


if __name__ == "__main__":
    main()
