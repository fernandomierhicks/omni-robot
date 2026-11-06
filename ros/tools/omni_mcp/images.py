"""Camera frames for robot_look and robot_find (ai_robot_architecture.md 3.1): a snapshot by HTTP from the robot's
web_video_server (/snapshot?topic=..., ROS images as JPEG; never through rosbridge) or camera_snap (/snap?cam=..., the
Pi cameras), downscaled to at most 640 px and re-encoded as JPEG quality 70 for the model.

web_video_server's snapshot waits for the topic's next message: a camera that streams answers at once; the sim's
depth camera renders its colour at 2 Hz while subscribed; /arm/find_image is sent again while anyone subscribes.
"""

from __future__ import annotations

import io
import time
import urllib.error
import urllib.parse
import urllib.request

from PIL import Image as PILImage

MAX_SIDE = 640
QUALITY = 70


class SnapshotError(RuntimeError):
    pass


def shrink(data: bytes, max_side: int = MAX_SIDE, quality: int = QUALITY) -> tuple:
    """(jpeg bytes, width, height): the image at most max_side on its longer side."""
    img = PILImage.open(io.BytesIO(data)).convert("RGB")
    img.thumbnail((max_side, max_side))
    out = io.BytesIO()
    img.save(out, format="JPEG", quality=quality)
    return out.getvalue(), img.width, img.height


def rotate180(jpeg: bytes, quality: int = QUALITY) -> bytes:
    img = PILImage.open(io.BytesIO(jpeg)).convert("RGB").rotate(180)
    out = io.BytesIO()
    img.save(out, format="JPEG", quality=quality)
    return out.getvalue()


def fetch(url: str, timeout: float) -> bytes:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read()
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        raise SnapshotError(f"{url}: {getattr(e, 'reason', e)}") from e


def snapshot(host: str, port: int, topic: str, timeout: float = 6.0, max_side: int = MAX_SIDE) -> dict:
    """{jpeg, width, height, topic, captured (wall time of the reply)} from web_video_server."""
    q = urllib.parse.urlencode(dict(topic=topic, quality=90), safe="/")   # it doesn't decode %2F in topic names
    t0 = time.time()
    raw = fetch(f"http://{host}:{port}/snapshot?{q}", timeout)
    if not raw:
        raise SnapshotError(f"{topic}: an empty snapshot")
    jpeg, w, h = shrink(raw, max_side)
    return dict(jpeg=jpeg, width=w, height=h, topic=topic, captured=round(time.time(), 2),
                waited_s=round(time.time() - t0, 2))


def camera_snap(host: str, port: int, cam: str, timeout: float = 6.0, max_side: int = MAX_SIDE) -> dict:
    """The same from omni_console's camera_snap (the Pi cameras, /snap?cam=...)."""
    raw = fetch(f"http://{host}:{port}/snap?{urllib.parse.urlencode(dict(cam=cam))}", timeout)
    jpeg, w, h = shrink(raw, max_side)
    return dict(jpeg=jpeg, width=w, height=h, topic=f"camera_snap:{cam}", captured=round(time.time(), 2))
