"""Tests 2-3 of the lidar bench (README.md): range noise, bias and dropouts on flat walls, from lidar_capture.py's raw
streams of a lidar standing still.

    python lidar_range.py C:/Users/ferna/lidar_data/SESSION [--out results] [--truth 1:2.315,3:0.842] [--skip 5]

The walls are found in the scene itself: the median scan of the longest run (median range per 0.5 deg) is split into
straight segments (split and merge, 2 cm), and each segment longer than 0.3 m, trimmed 10 % at both ends (corners,
mixed pixels), is a wall, its direction fitted there (total least squares). Then for each run and wall, every turn's
samples in the wall's bearings are pooled and only the wall's distance fitted (a few coarse samples can't tilt it):

  noise      a sample's offset from the wall's own mean shape (the median offset per 0.5 deg over the run), along its
             beam (perpendicular / cos incidence): std and the robust std (1.4826 MAD), per wall and in 0.5 m range
             bands pooled over the walls (incidence < 50 deg only); flat: the same about the straight line (it
             includes the wall's unevenness)
  per turn   the std of a turn's mean offset: what averaging over a turn leaves (temporal noise)
  dropouts   samples in the wall's bearings with no return
  bias       the line's distance from the lidar against Stability's (between modes) and, given --truth WALL:METRES
             (perpendicular, taped from the head's centre), against the tape
  drift      in a run of 60 s or more: the wall's distance in 10 s windows (warm-up, test 7)

Angles are the device's (clockwise, the encoder plus its parallax correction); the 0 deg mark needn't face anything.
Writes results/range_<session>.json and walls.png (the median scan, numbered walls) next to the raw data.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros", "src", "omni_console", "omni_console"))
import rplidar  # noqa: E402  (the robot's driver)

BIN_DEG = 0.5
SPLIT_M = 0.02
GAP_M = 0.15
MIN_LEN_M = 0.3
TRIM = 0.10
MAX_INC_DEG = 50.0
BAND_M = 0.5


def turns_of(run: dict, path: str, skip_s: float):
    """Each whole turn's samples as arrays (angle deg clockwise, range m; 0 = no return), the spin-up dropped."""
    z = np.load(path)
    turns = rplidar.decode_bytes(run["ans_type"], z["data"].tobytes())
    out, t = [], 0.0
    for tr in turns:
        a, r = np.asarray(tr.angle, float), np.asarray(tr.dist, float) / 1000.0
        t += len(a) * run["us_per_sample"] * 1e-6          # the lidar's own clock: time since the first whole turn
        if t >= skip_s:
            out.append((t, a % 360.0, r))
    return out


def xy(a_deg, r):
    th = -np.radians(a_deg)                                # clockwise -> counterclockwise
    return np.stack([r * np.cos(th), r * np.sin(th)], -1)


def median_scan(turns):
    nb = int(round(360 / BIN_DEG))
    a = np.concatenate([x[1] for x in turns])
    r = np.concatenate([x[2] for x in turns])
    b = (a / BIN_DEG).astype(int) % nb
    med = np.full(nb, np.nan)
    for k in range(nb):
        v = r[(b == k)]
        if v.size and np.mean(v > 0) >= 0.5:
            med[k] = np.median(v[v > 0])
    return (np.arange(nb) + 0.5) * BIN_DEG, med


def tls(p):
    """Line through points: (unit normal n, offset c) with n.p = c, c >= 0 (n points away from the lidar)."""
    m = p.mean(0)
    _, _, vt = np.linalg.svd(p - m, full_matrices=False)
    n = vt[1]
    c = float(n @ m)
    return (n, c) if c >= 0 else (-n, -c)


def split(p, idx):
    """Split and merge on ordered points: index ranges whose points stay within SPLIT_M of their chord."""
    if len(idx) < 3:
        return [idx]
    a, b = p[idx[0]], p[idx[-1]]
    d = b - a
    L = np.hypot(*d)
    if L < 1e-9:
        return [idx]
    q = p[idx] - a
    dist = np.abs(d[0] * q[:, 1] - d[1] * q[:, 0]) / L
    k = int(np.argmax(dist))
    if dist[k] <= SPLIT_M:
        return [idx]
    return split(p, idx[:k + 1]) + split(p, idx[k:])


def find_walls(ang, med):
    ok = np.isfinite(med)
    order = np.argsort(ang)
    ang, med, ok = ang[order], med[order], ok[order]
    p = xy(ang, np.where(ok, med, 0))
    runs, cur = [], []
    for i in range(len(ang)):                             # contiguous returns without a jump
        if ok[i] and (not cur or np.hypot(*(p[i] - p[cur[-1]])) < GAP_M):
            cur.append(i)
        else:
            if len(cur) >= 3:
                runs.append(cur)
            cur = [i] if ok[i] else []
    if len(cur) >= 3:
        runs.append(cur)
    walls = []
    for rn in runs:
        for seg in split(p, np.array(rn)):
            if len(seg) < 8:
                continue
            L = np.hypot(*(p[seg[-1]] - p[seg[0]]))
            if L < MIN_LEN_M:
                continue
            k = max(2, int(round(TRIM * len(seg))))
            seg = seg[k:-k]
            if len(seg) < 4:
                continue
            n, c = tls(p[seg])
            walls.append(dict(a0=float(ang[seg[0]] - BIN_DEG / 2), a1=float(ang[seg[-1]] + BIN_DEG / 2),
                              length_m=round(float(np.hypot(*(p[seg[-1]] - p[seg[0]]))), 3),
                              dist_m=round(c, 4), normal=[float(n[0]), float(n[1])]))
    walls.sort(key=lambda w: w["a0"])
    for i, w in enumerate(walls, 1):
        w["id"] = i
    return walls


def robust_std(x):
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if x.size else float("nan")


def wall_stats(turns, w):
    """Pool every turn's samples in the wall's bearings; noise along the beam; per-turn means; dropouts; drift."""
    sel_t, sel_a, sel_r = [], [], []
    tot = 0
    for t, a, r in turns:
        m = (a >= w["a0"]) & (a <= w["a1"])
        tot += int(m.sum())
        m &= r > 0
        sel_t.append(np.full(int(m.sum()), t))
        sel_a.append(a[m])
        sel_r.append(r[m])
    t, a, r = np.concatenate(sel_t), np.concatenate(sel_a), np.concatenate(sel_r)
    if r.size < 20:
        return None
    p = xy(a, r)
    n = np.array(w["normal"])                             # the wall's direction from the reference scan: only its
    d = p @ n                                             # distance is fitted per run (few samples can't tilt it)
    keep = np.abs(d - np.median(d)) < max(0.05, 6 * robust_std(d))   # the far side of an edge etc. out, counted
    c = float(d[keep].mean())
    perp = d - c
    u = p / np.linalg.norm(p, axis=1, keepdims=True)
    cosi = np.clip(np.abs(u @ n), 1e-3, 1)
    inc = np.degrees(np.arccos(cosi))
    along = perp / cosi                                   # the range error that puts the point off the line
    # temporal noise: off the wall's own mean shape (median perpendicular offset per 0.5 deg), so its unevenness
    # (skirting, frames, a slight bow) doesn't count
    b = np.floor(a / BIN_DEG).astype(int)
    shape = np.zeros_like(perp)
    for k in np.unique(b[keep]):
        m = keep & (b == k)
        shape[b == k] = np.median(perp[m])
    temp = (perp - shape) / cosi
    g = keep & (inc < MAX_INC_DEG)
    tt = np.unique(t)
    per_turn = np.array([perp[g & (t == x)].mean() for x in tt if np.any(g & (t == x))])
    out = dict(dist_m=round(c, 4), samples=int(r.size), samples_a_turn=round(r.size / max(len(turns), 1), 1),
               dropout=round(1 - r.size / max(tot, 1), 4), outliers=round(float(1 - keep.mean()), 4),
               range_m=[round(float(r[g].min()), 3), round(float(r[g].max()), 3)] if g.any() else None,
               incidence_deg=[round(float(inc[keep].min()), 1), round(float(inc[keep].max()), 1)],
               flat_mm=round(1000 * robust_std(along[g]), 2) if g.any() else None,
               noise_mm=round(1000 * float(temp[g].std()), 2) if g.any() else None,
               noise_robust_mm=round(1000 * robust_std(temp[g]), 2) if g.any() else None,
               turn_mean_std_mm=round(1000 * float(per_turn.std()), 3) if per_turn.size > 2 else None)
    if t.max() - t.min() >= 60:                           # drift: the line's distance in 10 s windows
        win = []
        for t0 in np.arange(t.min(), t.max(), 10.0):
            m = keep & (t >= t0) & (t < t0 + 10)
            if m.sum() > 20:
                win.append([round(float(t0), 1), round(1000 * (float(np.mean(p[m] @ n)) - c), 2)])
        out["drift_mm"] = win
    return out, r[g], temp[g]


def bands(rs, es):
    r, e = np.concatenate(rs), np.concatenate(es)
    out = []
    for lo in np.arange(0, np.ceil(r.max() / BAND_M) * BAND_M, BAND_M) if r.size else []:
        m = (r >= lo) & (r < lo + BAND_M)
        if m.sum() >= 50:
            out.append(dict(range_m=[round(float(lo), 2), round(float(lo + BAND_M), 2)], samples=int(m.sum()),
                            mean_mm=round(1000 * float(e[m].mean()), 2), std_mm=round(1000 * float(e[m].std()), 2),
                            robust_mm=round(1000 * robust_std(e[m]), 2)))
    return out


def plot(ang, med, walls, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ok = np.isfinite(med)
    p = xy(ang[ok], med[ok])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(p[:, 0], p[:, 1], s=3, c="0.5")
    ax.plot(0, 0, "r^", ms=10)
    ax.annotate("0°", (0.3, 0), color="r")
    for w in walls:
        aa = np.linspace(w["a0"], w["a1"], 30)
        n = np.array(w["normal"])
        th = -np.radians(aa)
        u = np.stack([np.cos(th), np.sin(th)], -1)
        q = u * (w["dist_m"] / (u @ n))[:, None]
        ax.plot(q[:, 0], q[:, 1], lw=3)
        m = q[len(q) // 2]
        ax.annotate(f"{w['id']}: {w['dist_m']:.2f} m", m, fontsize=11, weight="bold")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title("Median scan; walls with their perpendicular distance (lidar at the red triangle, x = 0° mark)")
    fig.savefig(path, dpi=110, bbox_inches="tight")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session_dir")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    ap.add_argument("--truth", default="", help="WALL:METRES,... taped perpendicular distances")
    ap.add_argument("--skip", type=float, default=5.0, help="s of each run dropped (spin-up)")
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.session_dir, "session.json")))
    runs = [r for r in meta["runs"] if not r.get("skipped")]
    data = {r["name"]: turns_of(r, os.path.join(a.session_dir, r["name"] + ".npz"), a.skip) for r in runs}
    ref = max(runs, key=lambda r: len(data[r["name"]]))
    ang, med = median_scan(data[ref["name"]])
    walls = find_walls(ang, med)
    plot(ang, med, walls, os.path.join(a.session_dir, "walls.png"))
    truth = {int(k): float(v) for k, v in (x.split(":") for x in a.truth.split(",") if x)}

    # modes are compared with Stability at the reference's PWM, the warm one (the latest such run) when there are two
    st_run = [r["name"] for r in runs if r["mode"] == "Stability" and r["pwm"] == ref["pwm"]]
    st_run = st_run[-1] if st_run else None
    res = dict(session=meta["session"], device=meta["info"], reference_run=ref["name"], stability_run=st_run,
               walls=walls, runs=[])
    stab = {}
    for r in runs:
        tr = data[r["name"]]
        row = dict(name=r["name"], mode=r["mode"], pwm=r["pwm"], turns=len(tr),
                   samples_a_turn=round(float(np.mean([len(x[1]) for x in tr])), 1) if tr else 0, walls={})
        rs, es = [], []
        for w in walls:
            s = wall_stats(tr, w)
            if s is None:
                continue
            st, rr, ee = s
            if w["id"] in truth:
                st["bias_mm"] = round(1000 * (st["dist_m"] - truth[w["id"]]), 1)
            row["walls"][w["id"]] = st
            rs.append(rr)
            es.append(ee)
        row["bands"] = bands(rs, es) if rs else []
        if r["name"] == st_run:
            stab = {k: v["dist_m"] for k, v in row["walls"].items()}
        res["runs"].append(row)
    for row in res["runs"]:                               # each mode's distance against Stability's (same minute)
        for k, v in row["walls"].items():
            if k in stab:
                v["vs_stability_mm"] = round(1000 * (v["dist_m"] - stab[k]), 1)

    os.makedirs(a.out, exist_ok=True)
    name = meta["session"] if meta["session"].startswith("range_") else "range_" + meta["session"]
    with open(os.path.join(a.out, name + ".json"), "w") as f:
        json.dump(res, f, indent=1)

    print(f"walls (from {ref['name']}):")
    for w in walls:
        print(f"  {w['id']:2d}: {w['dist_m']:.3f} m, {w['length_m']:.2f} m long, bearings {w['a0']:.1f}-{w['a1']:.1f} deg")
    for row in res["runs"]:
        print(f"\n{row['name']}: {row['turns']} turns, {row['samples_a_turn']} samples a turn")
        print("  wall  dist m   range m      inc deg  /turn  noise mm (robust)  flat mm  turn-mean mm  dropout  vs Stab mm"
              "  bias mm")
        for k, v in row["walls"].items():
            rg = v["range_m"] or [float("nan")] * 2
            print(f"  {k:4d}  {v['dist_m']:.3f}  {rg[0]:.2f}-{rg[1]:.2f}  {v['incidence_deg'][0]:4.0f}-{v['incidence_deg'][1]:3.0f}"
                  f"  {v['samples_a_turn']:5.1f}  {v['noise_mm']!s:>6} ({v['noise_robust_mm']!s:>5})  {v['flat_mm']!s:>7}"
                  f"     {v['turn_mean_std_mm']!s:>6}"
                  f"     {100 * v['dropout']:5.1f} %  {v.get('vs_stability_mm', ''):>8}  {v.get('bias_mm', ''):>7}")
        print("  bands: " + "; ".join(f"{b['range_m'][0]}-{b['range_m'][1]} m {b['robust_mm']} mm" for b in row["bands"]))
    print(f"\n-> {os.path.join(a.out, name + '.json')}, {os.path.join(a.session_dir, 'walls.png')}")


if __name__ == "__main__":
    main()
