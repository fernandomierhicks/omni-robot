"""Scores studies/imx662_study.py's saved frames (run in the realsense venv: pupil-apriltags + OpenCV; no MuJoCo needed).

  tags   pupil-apriltags (the C library, proposed for tag_detector, design.md 4.6) on every frame, the min_margin 30 gate. A tag
         counts as visible at a pose when the still, global-shutter frame from that pose decodes it; per case: the share of those
         still decoded (recall), detections of tags not visible still or > 0.3 m off (wrong), the pose error against the truth at the
         frame's middle row (position, % of range, the face's normal)
  flow   pyramidal Lucas-Kanade on Shi-Tomasi corners between consecutive frames (forward-backward checked), the essential matrix
         (RANSAC) and its rotation and translation direction against the truth: the yaw-rate error (deg/s) and the direction of
         travel's error (deg, the omni base's slip check); features tracked, median image motion

    C:/Users/ferna/.venvs/realsense/Scripts/python.exe -I studies/imx662_detect.py tags|flow|all
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import pupil_apriltags as pa

OUT = Path(__file__).resolve().parent / "results" / "imx662"
DATA = Path(os.environ.get("OMNI_IMX662_DATA", Path.home() / "omni_sim_data" / "imx662"))   # imx662_study.py's rendered frames
MIN_MARGIN = 30.0


def _gray(p: Path) -> np.ndarray:
    return cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)


def _angle(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def tags():
    root = DATA / "frames"
    meta = json.loads((root / "meta.json").read_text())
    K = meta["K"]
    det = pa.Detector(families="tag36h11", nthreads=4, quad_decimate=1.0)
    cam = (K["fx"], K["fy"], K["cx"], K["cy"])
    found = {}
    for c in meta["cases"]:
        rows = json.loads((root / c["name"] / "truth.json").read_text())
        out = []
        for r in rows:
            im = _gray(root / c["name"] / f"{r['pose']}.jpg")
            dets = [d for d in det.detect(im, estimate_tag_pose=True, camera_params=cam, tag_size=0.16) if d.decision_margin >= MIN_MARGIN]
            out.append([dict(id=d.tag_id, margin=round(float(d.decision_margin), 1), t=d.pose_t.ravel().tolist(),
                             n=(-d.pose_R[:, 2]).tolist(), corners=d.corners.tolist()) for d in dets])
        found[c["name"]] = (c, rows, out)
        print("detected", c["name"], sum(len(o) for o in out), flush=True)
    still = {}                                                         # exposure ms -> pose -> ids the still global-shutter frame decodes
    for ex in {int(round(1000 * c["exposure"])) for c in meta["cases"]}:
        _, rows, out = found[f"still_ro0_ex{ex}"]
        still[ex] = {r["pose"]: {d["id"] for d in o} for r, o in zip(rows, out)}
    res = []
    for name, (c, rows, out) in found.items():
        ex = int(round(1000 * c["exposure"]))
        base = still[ex]
        n_vis = n_hit = n_wrong = n_flip = 0
        perr, prel, nerr, margins = [], [], [], []
        for r, o in zip(rows, out):
            vis = base[r["pose"]]
            n_vis += len(vis)
            for d in o:
                tr = r["tags"].get(str(d["id"]))
                if tr is None:
                    n_wrong += 1
                    continue
                t_est = np.array(d["t"]) * tr["size"] / 0.16                 # detected as 0.16 m; the dock's tag is 0.10
                e = float(np.linalg.norm(t_est - np.array(tr["t_cam"])))
                if e > 0.3:
                    n_wrong += 1
                    continue
                if d["id"] in vis:
                    n_hit += 1
                rng_ = float(np.linalg.norm(tr["t_cam"]))
                perr.append(e)
                prel.append(100 * e / rng_)
                a = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(d["n"], tr["normal_cam"]))))))
                if a > 30.0:
                    n_flip += 1                                           # the planar pose ambiguity's other solution
                else:
                    nerr.append(a)
                margins.append(d["margin"])
        q = lambda a, p: round(float(np.percentile(a, p)), 4) if a else None  # noqa: E731
        res.append(dict(case=name, motion=c["motion"], readout_ms=round(1000 * c["readout"]), exposure_ms=ex, frames=len(rows),
                        visible_still=n_vis, recall=round(n_hit / n_vis, 3) if n_vis else None, wrong=n_wrong, flipped=n_flip,
                        pos_err_m=dict(p50=q(perr, 50), p90=q(perr, 90)), pos_err_pct_range=dict(p50=q(prel, 50), p90=q(prel, 90)),
                        normal_err_deg=dict(p50=q(nerr, 50), p90=q(nerr, 90)), margin_p50=q(margins, 50)))
        print(res[-1], flush=True)
    (OUT / "tags_scored.json").write_text(json.dumps(res, indent=1))
    print("wrote", OUT / "tags_scored.json")


def _track(a, b):
    p0 = cv2.goodFeaturesToTrack(a, maxCorners=1000, qualityLevel=0.01, minDistance=12, blockSize=7)
    if p0 is None or len(p0) < 8:
        return None, None
    lk = dict(winSize=(21, 21), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p0, None, **lk)
    pb, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None, **lk)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((p0 - pb).reshape(-1, 2), axis=1) < 1.0)
    return p0.reshape(-1, 2)[ok], p1.reshape(-1, 2)[ok]


def flow():
    root = DATA / "flow"
    meta = json.loads((root / "meta.json").read_text())
    K = meta["K"]
    Km = np.array([[K["fx"], 0, K["cx"]], [0, K["fx"], K["cy"]], [0, 0, 1.0]])
    fps = meta["fps"]
    res = []
    for c in meta["cases"]:
        dd = root / c["name"]
        tr = json.loads((dd / "truth.json").read_text())
        frames = sorted(dd.glob("*.jpg"))
        rot_err, dir_err, n_feat, motion_px, rate_true = [], [], [], [], []
        fails = 0
        prev = _gray(frames[0])
        for j in range(1, len(frames)):
            cur = _gray(frames[j])
            a, b = _track(prev, cur)
            prev = cur
            R1, p1 = np.array(tr[j - 1]["cam_R"]), np.array(tr[j - 1]["cam_pos"])
            R2, p2 = np.array(tr[j]["cam_R"]), np.array(tr[j]["cam_pos"])
            Rt, tt = R2.T @ R1, R2.T @ (p1 - p2)
            rate_true.append(_angle(Rt) * fps)
            if a is None or len(a) < 15:
                fails += 1
                continue
            n_feat.append(len(a))
            motion_px.append(float(np.median(np.linalg.norm(b - a, axis=1))))
            E, inl = cv2.findEssentialMat(a, b, Km, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            if E is None or E.shape != (3, 3):
                fails += 1
                continue
            _, Re, te, _ = cv2.recoverPose(E, a, b, Km, mask=inl)
            if _angle(Re @ Rt.T) > 90.0:                                 # the decomposition's flipped solution: a failure, not an error
                fails += 1
                continue
            rot_err.append(_angle(Re @ Rt.T) * fps)                      # deg/s
            if np.linalg.norm(tt) > 0.005:
                u = tt / np.linalg.norm(tt)
                dir_err.append(math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(te.ravel(), u)))))))
        q = lambda a, p: round(float(np.percentile(a, p)), 2) if a else None  # noqa: E731
        res.append(dict(case=c["name"], motion=c["motion"], readout_ms=round(1000 * c["readout"]), pairs=len(frames) - 1, fails=fails,
                        true_rate_dps=q(rate_true, 50), rot_err_dps=dict(p50=q(rot_err, 50), p90=q(rot_err, 90)),
                        dir_err_deg=dict(p50=q(dir_err, 50), p90=q(dir_err, 90)), features_p50=q(n_feat, 50), motion_px_p50=q(motion_px, 50)))
        print(res[-1], flush=True)
    (OUT / "flow_scored.json").write_text(json.dumps(res, indent=1))
    print("wrote", OUT / "flow_scored.json")


def summary():
    """markdown tables from tags_scored.json and flow_scored.json -> results/imx662/summary.md"""
    lines = []
    if (OUT / "tags_scored.json").exists():
        T = json.loads((OUT / "tags_scored.json").read_text())
        lines += ["## Tags (pupil-apriltags, margin >= 30; recall against the still global-shutter frame from the same pose)", ""]
        for ex in sorted({r["exposure_ms"] for r in T}):
            lines += [f"Exposure {ex} ms. Each cell: recall / position error p50 (p90) in % of range / orientation flips (the pose "
                      f"ambiguity's other solution, normal > 30 deg off) / wrong ids.", "",
                      "| motion | " + " | ".join(f"readout {ro} ms" for ro in (0, 16, 33)) + " |", "| --- | --- | --- | --- |"]
            for mo in dict.fromkeys(r["motion"] for r in T):
                cells = []
                for ro in (0, 16, 33):
                    r = next((x for x in T if x["motion"] == mo and x["readout_ms"] == ro and x["exposure_ms"] == ex), None)
                    pe = r and r["pos_err_pct_range"]
                    cells.append("" if r is None else f"{r['recall']:.2f} / " + (f"{pe['p50']:.2f} ({pe['p90']:.2f})" if pe["p50"] is not None
                                                                                 else "-") + f" / {r['flipped']} / {r['wrong']}")
                if any(cells):
                    lines.append(f"| {mo} | " + " | ".join(cells) + " |")
            lines.append("")
    if (OUT / "flow_scored.json").exists():
        F = json.loads((OUT / "flow_scored.json").read_text())
        lines += ["## Optical flow (LK + essential matrix, 30 fps, 4 starts x 11 pairs)", "",
                  "Each cell: yaw-rate error deg/s p50 over the pairs of all starts (rotation flips past 90 deg/frame counted as failures) / "
                  "direction-of-travel error deg p50 / failed pairs.", "",
                  "| motion | " + " | ".join(f"readout {ro} ms" for ro in (0, 16, 33)) + " |", "| --- | --- | --- | --- |"]
        for mo in dict.fromkeys(r["motion"] for r in F):
            cells = []
            for ro in (0, 16, 33):
                rs = [x for x in F if x["motion"] == mo and x["readout_ms"] == ro]
                rot = [x["rot_err_dps"]["p50"] for x in rs if x["rot_err_dps"]["p50"] is not None]
                dr = [x["dir_err_deg"]["p50"] for x in rs if x["dir_err_deg"]["p50"] is not None]
                cells.append(f"{np.median(rot):.1f} / {np.median(dr):.0f} / {sum(x['fails'] for x in rs)}" if rot else "")
            lines.append(f"| {mo} | " + " | ".join(cells) + " |")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("tags", "all"):
        tags()
    if what in ("flow", "all"):
        flow()
    if what in ("summary", "all"):
        summary()
