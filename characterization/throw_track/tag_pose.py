"""The basket's AprilTag in the camera's frame: where the rim is (design.md 8, phase 1C).

record_throws.py saves a still at the start (tag.npy) and at the end (tag_end.npy) of a session. This finds the tag
(36h11, omni_calib.detect), solves its pose (solvePnP, the tag's known side and the camera's focal length) and writes
`basket.tag_cam` (the tag centre in the camera frame: x right, y up, the camera looks along -z, metres) into
session.json, where analyze_throws.py takes the rim from (rim = tag + basket.rim_center_from_tag, in the world frame
the camera's gravity direction defines). If the end still disagrees with the start, the camera or the basket moved
during the session: the shift is written as basket.tag_shift_m and printed.

Needs OpenCV (the Pi's venv and the sim's venv have none): the realsense venv, `tt.sh tag` runs it there.

  python tag_pose.py data/<session> [--size 0.10] [--id 0]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ros" / "src" / "omni_calib"))
sys.path.insert(0, str(HERE))

import session as S                                          # noqa: E402


def pose_of(img: np.ndarray, tag_id: int, size: float, f: float, cx: float, cy: float):
    import cv2
    from omni_calib.detect import apriltags
    from omni_calib.targets import tag_corners
    gray = (np.asarray(img, np.float32) @ np.array([0.299, 0.587, 0.114], np.float32)).astype(np.uint8)
    found = [d for d in apriltags(gray) if d[0] == tag_id]
    if not found:
        return None
    _, c, _ = found[0]
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
    ok, rvec, tvec = cv2.solvePnP(tag_corners(size), c.astype(np.float64), K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    flip = np.diag([1.0, -1.0, -1.0])                         # OpenCV (x right, y down, z forward) -> ours (y up, looks along -z)
    t = flip @ tvec.ravel()
    normal = flip @ R[:, 2]                                   # the tag face's outward normal, camera frame
    side_px = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    return dict(tag_cam=t.tolist(), normal_cam=normal.tolist(), range_m=float(np.linalg.norm(t)), side_px=side_px,
                corners_px=c.tolist())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--size", type=float, help="tag side, m (default: the session's basket.tag_size_m)")
    ap.add_argument("--id", type=int, help="tag id (default: the session's)")
    a = ap.parse_args()
    sess = S.load_session(a.session)
    b = sess.setdefault("basket", {})
    cam = sess["camera"]
    f = cam.get("f_px") or cam.get("f_px_est") or (cam["height"] / 2) / np.tan(np.radians(cam["fovy_deg"]) / 2)
    size, tid = a.size or b.get("tag_size_m", 0.10), a.id if a.id is not None else b.get("tag_id", 0)
    out = {}
    for name in ("tag", "tag_end"):
        f_img = a.session / f"{name}.npy"
        if not f_img.exists():
            continue
        r = pose_of(np.load(f_img), tid, size, f, cam["width"] / 2, cam["height"] / 2)
        out[name] = r
        print(f"{name}: " + ("tag not found" if r is None else
              f"range {r['range_m']:.2f} m, side {r['side_px']:.0f} px, centre (x {r['tag_cam'][0]:+.3f}, y {r['tag_cam'][1]:+.3f}, z {r['tag_cam'][2]:+.3f}) m"))
    if out.get("tag"):
        b["tag_cam"] = out["tag"]["tag_cam"]
        b["tag_normal_cam"] = out["tag"]["normal_cam"]
        b["tag_range_m"] = out["tag"]["range_m"]
    if out.get("tag") and out.get("tag_end"):
        shift = float(np.linalg.norm(np.array(out["tag"]["tag_cam"]) - np.array(out["tag_end"]["tag_cam"])))
        b["tag_shift_m"] = shift
        print(f"tag moved {100 * shift:.1f} cm in the camera frame between the start and the end of the session"
              + ("  <- the camera or the basket moved" if shift > 0.02 else ""))
    if not out.get("tag"):
        raise SystemExit("no tag in the start still: set basket.tag_cam (or basket.rim_center_world) in session.json by hand")
    S.save_session(a.session, sess)
    print("-> session.json basket.tag_cam")


if __name__ == "__main__":
    main()
