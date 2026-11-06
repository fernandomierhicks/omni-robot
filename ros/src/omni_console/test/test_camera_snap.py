"""camera_snap, the Cameras tab's picture server, on its test patterns (no cameras). Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test
"""

import io
import json
import os
import sys
import time
import urllib.error
import urllib.request

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console import camera_snap as cs                                # noqa: E402


@pytest.fixture
def server():
    cams = cs.cameras(fake=True, idle_s=0.5)
    httpd = cs.serve(cams, port=0, host="127.0.0.1")
    yield f"http://127.0.0.1:{httpd.server_address[1]}", cams
    httpd.shutdown()


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.headers, r.read()


def test_lists_both_cameras(server):
    base, _ = server
    _, h, body = get(base + "/cameras")
    assert h["Access-Control-Allow-Origin"] == "*"
    cams = {c["name"]: c for c in json.loads(body)["cameras"]}
    assert cams["arducam"]["streams"] == ["color"]
    assert cams["realsense"]["streams"] == ["color", "ir", "depth"]
    assert not cams["realsense"]["open"]


@pytest.mark.parametrize("cam, stream, mode", [("arducam", "color", "RGB"), ("realsense", "color", "RGB"),
                                               ("realsense", "ir", "L"), ("realsense", "depth", "RGB")])
def test_snap_is_a_scaled_jpeg(server, cam, stream, mode):
    base, _ = server
    _, h, body = get(f"{base}/snap?cam={cam}&stream={stream}&w=320&q=60")
    assert h["Content-Type"] == "image/jpeg" and "X-Frame-Info" in h["Access-Control-Expose-Headers"]
    img = Image.open(io.BytesIO(body))
    assert img.format == "JPEG" and img.mode == mode and img.size == (320, 240)
    info = json.loads(h["X-Frame-Info"])
    assert (info["width"], info["height"]) == (640, 480)           # what was captured, before scaling
    if stream == "depth":                                          # the fake's ramp: 1 m left to 3 m right
        assert info["valid"] == 1.0 and info["centre_m"] == pytest.approx(2.0, abs=0.02)
        assert info["p5_m"] < info["median_m"] < info["p95_m"]


def test_full_size_and_depth_probe(server):
    base, _ = server
    with pytest.raises(urllib.error.HTTPError) as e:              # nothing sent yet
        get(base + "/depth?u=0.5&v=0.5")
    assert e.value.code == 404
    _, _, body = get(f"{base}/snap?cam=realsense&stream=depth&w=0")
    assert Image.open(io.BytesIO(body)).size == (640, 480)
    for u, want in ((0.0, 1.0), (0.5, 2.0), (1.0, 3.0)):
        assert json.loads(get(f"{base}/depth?u={u}&v=0.5")[2])["range_m"] == pytest.approx(want, abs=0.02)


def test_unknown_stream_is_404(server):
    base, _ = server
    for q in ("cam=arducam&stream=depth", "cam=nope&stream=color"):
        with pytest.raises(urllib.error.HTTPError) as e:
            get(f"{base}/snap?{q}")
        assert e.value.code == 404


def test_opens_on_demand_and_closes_when_idle(server):
    base, cams = server
    get(f"{base}/snap?cam=realsense&stream=ir")
    assert cams["realsense"].src is not None and cams["arducam"].src is None
    t_end = time.time() + 5
    while cams["realsense"].src is not None and time.time() < t_end:
        time.sleep(0.1)
    assert cams["realsense"].src is None


def test_busy_camera_answers_503_and_backs_off():
    n = []

    def opener(*_):
        n.append(1)
        raise RuntimeError("Device or resource busy")

    cam = cs.Camera("arducam", "Arducam", ("color",), opener)
    httpd = cs.serve({"arducam": cam}, port=0, host="127.0.0.1")
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    try:
        for _ in range(3):
            with pytest.raises(urllib.error.HTTPError) as e:
                get(f"{base}/snap?cam=arducam&stream=color")
            assert e.value.code == 503 and "busy" in json.loads(e.value.read())["error"]
        assert len(n) == 1                                         # retried only after RETRY_S
        assert json.loads(get(base + "/cameras")[2])["cameras"][0]["error"].endswith("busy")
    finally:
        httpd.shutdown()


def test_missing_driver_is_said_plainly():
    def opener(*_):
        raise ModuleNotFoundError("No module named 'picamera2'", name="picamera2")

    cam = cs.Camera("arducam", "Arducam", ("color",), opener)
    with pytest.raises(cs.CameraError, match="picamera2 isn't installed"):
        cam.grab("color")


def read_parts(url, n):
    """The first n parts of a multipart stream: [(headers dict, body)]."""
    parts = []
    with urllib.request.urlopen(url, timeout=5) as r:
        assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace; boundary=")
        while len(parts) < n:
            line = r.readline()
            if not line.startswith(b"--"):
                continue
            head = {}
            while (line := r.readline().strip()):
                k, v = line.decode().split(":", 1)
                head[k.strip()] = v.strip()
            parts.append((head, r.read(int(head["Content-Length"]))))
    return parts


def test_cameras_list_profiles_and_the_detector(server):
    base, _ = server
    j = json.loads(get(base + "/cameras")[2])
    cams = {c["name"]: c for c in j["cameras"]}
    assert [p["name"] for p in cams["arducam"]["profiles"]] == list(cs.PI_PROFILES)
    assert cams["realsense"]["profile"] == "vga15" and cams["realsense"]["settings"]["emitter"] == 1
    assert "tag25h9" in j["tags"]["families"]


def test_stream_sends_newer_frames_each_with_its_info(server):
    base, cams = server
    parts = read_parts(f"{base}/stream?cam=realsense&stream=ir&w=320&q=50", 4)
    seqs = []
    for head, body in parts:
        assert head["Content-Type"] == "image/jpeg"
        assert Image.open(io.BytesIO(body)).size == (320, 240)
        seqs.append(json.loads(head["X-Frame-Info"])["seq"])
    assert seqs == sorted(seqs) and len(set(seqs)) == 4
    t_end = time.time() + 3                                        # the page went away: no longer streaming
    while cams["realsense"].streaming and time.time() < t_end:
        time.sleep(0.05)
    assert cams["realsense"].streaming == 0


def test_fps_cap_and_profile_switch(server):
    base, cams = server
    t0 = time.time()
    parts = read_parts(f"{base}/stream?cam=arducam&stream=color&fps=5&profile=crop100", 3)
    assert time.time() - t0 > 0.35                                 # 3 frames at 5 fps: at least 2 periods
    assert all(json.loads(h["X-Frame-Info"])["profile"] == "crop100" for h, _ in parts)
    j = json.loads(get(f"{base}/set?cam=arducam&profile=wide30&focus=1.5")[2])
    assert j["profile"] == "wide30" and j["settings"]["focus"] == 1.5


def test_finds_the_fake_tags_and_their_pose(server):
    pytest.importorskip("scipy")
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "omni_console"))
    from omni_console import apriltag_view as av
    if av.backend() is None:
        pytest.skip("no AprilTag detector here (pupil_apriltags or OpenCV aruco)")
    base, _ = server
    for fam, side, rng in (("tag25h9", 70, 0.2 * 600 / 70), ("tag36h11", 80, 0.2 * 600 / 80)):
        _, h, _ = get(f"{base}/snap?cam=realsense&stream=color&tags={fam}&size=0.2&dec=1")
        info = json.loads(h["X-Frame-Info"])
        assert [t["id"] for t in info["tags"]] == [0], info
        t = info["tags"][0]
        assert t["side_px"] == pytest.approx(side, abs=2)
        assert t["range"] == pytest.approx(rng, rel=0.04) and t["angle_deg"] < 15
        assert all(0 < u < 1 and 0 < v < 1 for u, v in t["corners"])


def test_depth_colormap_and_range():
    d = np.array([[0, 200, 4000]], np.uint16)
    rgb = cs.depth_colormap(d, 0.001)
    assert rgb[0, 0].tolist() == [0, 0, 0]                        # no depth: black
    assert rgb[0, 1].tolist() == [255, 40, 40]                    # near: red
    assert rgb[0, 2].tolist() == [60, 60, 255]                    # far: blue
    assert cs.range_at(np.zeros((10, 10), np.uint16), 0.001, 0.5, 0.5) is None
