"""Look at a recorded RealSense throw: all four imagers side by side at the moments the ball is in view, with each source's
detection drawn on (depth blob, IR blob, the colour blob the recorder's trigger saw). Also a stream health table.

  python rs_view.py data/<session> 3                 # throw t003: contact sheet PNG next to the results (rs_view_<session>_t003.png)
  python rs_view.py data/<session> 3 --scrub         # a window with a time slider (matplotlib)
  python rs_view.py data/<session> 3 --cols 9 --crop # more columns, each cropped around the ball
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rs_analyze as A          # noqa: E402
import rs_detect as D           # noqa: E402
import session as S             # noqa: E402


def health(th: S.Throw) -> str:
    rows = []
    t0 = th.t0_ns
    for n in th.names:
        s = th.meta.get("streams", {}).get(n) or S.stream_stats(th.stream(n).stamps, t0)
        rows.append(f"  {n:6s} {s['n']:4d} frames  {s.get('fps', 0):6.1f} fps  lost {s.get('lost', 0)}  max gap {s.get('dt_max_ms', 0):5.1f} ms  "
                    f"{s.get('first_s', 0):+.2f}..{s.get('last_s', 0):+.2f} s")
    return "\n".join(rows)


def frame_of(th: S.Throw, name: str, t: float):
    ts = th.t_of(name)
    i = int(A.nearest(ts, np.array([t]))[0])
    return i, th.stream(name).frames[i] if name != "color" else th.frames[i]


def draw_image(ax, name: str, img, scale: float):
    if name == "depth":
        z = np.asarray(img, np.float32) * scale
        ax.imshow(np.where(z > 0, z, np.nan), cmap="turbo", vmin=0.3, vmax=8.0)
    elif name == "color":
        ax.imshow(np.asarray(img))
    else:
        ax.imshow(np.asarray(img), cmap="gray", vmin=0, vmax=255)
    ax.set_xticks([])
    ax.set_yticks([])


def marks(res: dict, th: S.Throw, t: float):
    """{stream name: (u, v, r)} of the detections nearest time t."""
    out = {}
    keep = res.get("_keep", {})
    for src, name in (("depth", "depth"), ("ir_stereo", "ir1"), ("colour", "color")):
        if src in keep:
            ts, picks, _ = keep[src]
            i = int(A.nearest(ts, np.array([t]))[0])
            if picks[i] is not None:
                c = picks[i][1]
                out[name] = (c.u, c.v, max(c.r, 3.0))
    blobs = th.meta.get("live_blobs")
    if "colour" not in keep and blobs and th.frames is not None and len(blobs) == len(th.frames):     # no colour track at all: the trigger's blobs
        j = int(A.nearest(th.t_of("color"), np.array([t]))[0])
        moving = [b for b in blobs[j] if not still_blob(b, blobs)]
        if moving:
            out["color"] = moving[0]
    return out


def still_blob(b, blobs, px: float = 2.0, frac: float = 0.5) -> bool:
    """A blob that sits at the same pixel in at least half the window's frames: the scene, not the ball."""
    n = sum(1 for fr in blobs if any(abs(c[0] - b[0]) <= px and abs(c[1] - b[1]) <= px for c in fr))
    return n >= frac * len(blobs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session")
    ap.add_argument("throw", type=int, help="throw number")
    ap.add_argument("--cols", type=int, default=7)
    ap.add_argument("--crop", action="store_true", help="crop each tile around the ball (needs a detection)")
    ap.add_argument("--scrub", action="store_true")
    ap.add_argument("--set", nargs="*")
    ap.add_argument("--out", default=str(A.RESULTS))
    a = ap.parse_args(argv)

    d = Path(a.session)
    sess = S.load_session(d)
    th = S.load_throw(d / f"t{a.throw:03d}", sess)
    print(f"{th.path.name}: trigger at {th.meta.get('t_trigger_s', float('nan')):.2f} s")
    print(health(th))
    cfg = A.parse_set(a.set)
    bj = HERE / "bucket.json"
    up = __import__("rs_predict").Bucket.load(bj).up if bj.exists() else None          # the colour arc's gravity: the mount's
    res = A.analyze_throw(th, sess, cfg, colour=True, session_dir=d, up=up)
    print(A.describe(res))
    scale = float(sess["rs"].get("depth_scale", 0.001))
    names = th.names
    seen = [res["sources"][k] for k in ("colour", "depth", "ir_stereo") if res["sources"].get(k, {}).get("ok")]
    t_lo = min(s["first_t"] for s in seen) if seen else th.t_of(names[0])[0]
    t_hi = max(s["last_t"] for s in seen) if seen else th.t_of(names[0])[-1]

    import matplotlib
    if not a.scrub:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def render(axs, t_list):
        for r, name in enumerate(names):
            for c, t in enumerate(t_list):
                ax = axs[r][c]
                ax.clear()
                i, img = frame_of(th, name, t)
                draw_image(ax, name, img, scale)
                m = marks(res, th, t).get(name)
                if m is not None:
                    ax.add_patch(plt.Circle((m[0], m[1]), m[2] + 5, fill=False, color="white" if name != "depth" else "k", lw=1.0))
                    if a.crop:
                        h = 60
                        ax.set_xlim(m[0] - h, m[0] + h)
                        ax.set_ylim(m[1] + h, m[1] - h)
                if r == 0:
                    ax.set_title(f"{t:.2f} s", fontsize=8)
                if c == 0:
                    ax.set_ylabel(name, fontsize=9)

    if a.scrub:
        from matplotlib.widgets import Slider
        fig, axs = plt.subplots(len(names), 1, figsize=(6, 3 * len(names)), squeeze=False)
        fig.subplots_adjust(bottom=0.12)
        sl = Slider(fig.add_axes([0.15, 0.03, 0.7, 0.03]), "t (s)", float(t_lo), float(t_hi), valinit=float(t_lo))
        sl.on_changed(lambda v: (render(axs, [v]), fig.canvas.draw_idle()))
        render(axs, [t_lo])
        plt.show()
        return
    t_list = np.linspace(t_lo, t_hi, a.cols)
    fig, axs = plt.subplots(len(names), a.cols, figsize=(2.3 * a.cols, 1.5 * len(names) + 0.4), squeeze=False)
    render(axs, t_list)
    fig.suptitle(f"{d.name} {th.path.name}", fontsize=9)
    fig.tight_layout()
    Path(a.out).mkdir(parents=True, exist_ok=True)
    png = Path(a.out) / f"rs_view_{d.name}_{th.path.name}.png"
    fig.savefig(png, dpi=110)
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
