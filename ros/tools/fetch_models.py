"""Fetch the robot's learned models into ~/.omni_robot/models (or $OMNI_MODEL_DIR): not in git, checked by SHA-256.

  person detector (omni_perception people_node, design.md 4.12): YOLOX, Megvii's ONNX exports of release 0.1.1rc0,
  Apache-2.0, COCO-trained (class 0 = person). Input 416 px (nano, tiny) or 640 px (s); CPU times on a 4-core
  2.1 GHz Xeon VM (onnxruntime, 4 threads, Nov 7): nano ~9 ms, tiny ~23 ms, s ~75 ms a frame (est for the robot's
  computer)

    python ros/tools/fetch_models.py                # yolox_nano (the people node's default) and yolox_tiny
    python ros/tools/fetch_models.py yolox_s        # one by name
    python ros/tools/fetch_models.py --check        # verify what is there
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path

YOLOX = "https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/{}.onnx"
MODELS = {   # name: (url, sha256, bytes)
    "yolox_nano": (YOLOX.format("yolox_nano"), "c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d", 3659407),
    "yolox_tiny": (YOLOX.format("yolox_tiny"), "427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7", 20219662),
    "yolox_s": (YOLOX.format("yolox_s"), "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063", 35858002),
}
DEFAULT = ("yolox_nano", "yolox_tiny")


def model_dir() -> Path:
    return Path(os.environ.get("OMNI_MODEL_DIR") or Path.home() / ".omni_robot" / "models")


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(name: str, check_only: bool = False) -> bool:
    url, digest, size = MODELS[name]
    path = model_dir() / f"{name}.onnx"
    if path.exists() and sha256(path) == digest:
        print(f"{name}: ok ({path})")
        return True
    if check_only:
        print(f"{name}: {'wrong checksum' if path.exists() else 'missing'} ({path})")
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    print(f"{name}: fetching {url} ({size / 1e6:.1f} MB)")
    urllib.request.urlretrieve(url, tmp)
    got = sha256(tmp)
    if got != digest:
        tmp.unlink()
        print(f"{name}: checksum {got} is not {digest}; not kept", file=sys.stderr)
        return False
    tmp.replace(path)
    print(f"{name}: ok ({path})")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help=f"of {', '.join(MODELS)} (default: {', '.join(DEFAULT)})")
    ap.add_argument("--check", action="store_true", help="verify only")
    a = ap.parse_args()
    bad = [n for n in a.names if n not in MODELS]
    if bad:
        ap.error(f"unknown model(s) {', '.join(bad)}: one of {', '.join(MODELS)}")
    ok = all([fetch(n, a.check) for n in (a.names or DEFAULT)])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
