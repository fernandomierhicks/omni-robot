"""image_preview: a small, slow copy of a camera for the operator console (design.md 2), encoded once.

web_video_server encodes MJPEG on the CPU for every viewer at the camera's full rate, and the Orin Nano has no hardware
video encoder. The console doesn't need 120 fps at 1280 x 800 to see what the robot sees: this node keeps one frame in
every 1/rate s, shrinks it (every n-th pixel; on the robot the camera pipeline's VIC can make this copy for free) and
publishes it as JPEG. The page asks web_video_server for it with type=ros_compressed, which forwards the JPEG as it
is: no encoding per viewer. Nothing is done while nobody watches the preview.

Subscribes <input> sensor_msgs/Image (raw: only the kept frames are decoded)
Publishes  <input>/preview/compressed   sensor_msgs/CompressedImage (jpeg)
Parameters inputs (list of image topics), rate (Hz, 5), width (px, 320), quality (60)
"""

from __future__ import annotations

import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def parse_image(data: bytes):
    """sensor_msgs/Image CDR bytes -> (stamp sec, nanosec, frame_id, encoding, H x W x C uint8 view), or None."""
    e = "<" if data[1] & 1 else ">"
    p = 4
    sec, nsec, n = struct.unpack_from(e + "iII", data, p)
    p += 12
    frame = data[p:p + n - 1].decode(errors="replace")
    p += n
    p += (-(p - 4)) % 4
    h, w, n = struct.unpack_from(e + "III", data, p)
    p += 12
    enc = data[p:p + n - 1].decode(errors="replace")
    p += n + 1                                    # + is_bigendian
    p += (-(p - 4)) % 4
    step, size = struct.unpack_from(e + "II", data, p)
    p += 8
    ch = CHANNELS.get(enc)
    if ch is None or size < h * step:
        return None
    img = np.frombuffer(data, np.uint8, h * step, p).reshape(h, step)[:, :w * ch].reshape(h, w, ch)
    return sec, nsec, frame, enc, img


def shrink(img: np.ndarray, width: int) -> np.ndarray:
    k = max(1, int(np.ceil(img.shape[1] / width)))
    return img[::k, ::k]


def to_jpeg(img: np.ndarray, enc: str, quality: int) -> bytes:
    import cv2
    if enc in ("rgb8", "rgba8"):
        img = img[..., 2::-1] if img.shape[2] >= 3 else img           # cv2 wants BGR
    elif enc == "bgra8":
        img = img[..., :3]
    ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(img), [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else b""


class Preview(Node):
    def __init__(self):
        super().__init__("image_preview")
        P = self.declare_parameter
        inputs = P("inputs", ["/cam/left/image_raw"]).value
        self.period = 1.0 / max(0.2, P("rate", 5.0).value)
        self.width, self.quality = P("width", 320).value, P("quality", 60).value
        self.pubs = {}
        self.t_last = {}
        for topic in inputs:
            pub = self.create_publisher(CompressedImage, topic.rstrip("/") + "/preview/compressed", 2)
            self.pubs[topic], self.t_last[topic] = pub, 0.0
            self.create_subscription(Image, topic, lambda raw, t=topic: self.on_image(t, raw), qos_profile_sensor_data,
                                     raw=True)

    def on_image(self, topic: str, raw: bytes):
        now = time.monotonic()
        pub = self.pubs[topic]
        if now - self.t_last[topic] < self.period or pub.get_subscription_count() == 0:
            return
        self.t_last[topic] = now
        im = parse_image(raw)
        if im is None:
            return
        sec, nsec, frame, enc, img = im
        m = CompressedImage()
        m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id = sec, nsec, frame
        m.format = "jpeg"
        m.data = to_jpeg(shrink(img, self.width), enc, self.quality)
        pub.publish(m)


def main():
    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
