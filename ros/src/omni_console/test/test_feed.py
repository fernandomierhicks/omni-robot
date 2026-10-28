"""The console's raw-byte readers (cdr.py, tflite.py): what the mission node decodes instead of building Python
messages. Run from the repo root:  & $py -m pytest -q ros/src/omni_console/test   (in WSL, with ROS sourced, the
bytes also come from rclpy's own serializer)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console.cdr import CdrReader, CdrWriter, parse_ball_track   # noqa: E402
from omni_console.tflite import TfLite, parse_tf                        # noqa: E402


def tf_bytes(items) -> bytes:
    """TFMessage: [(parent, child, t, x, y, yaw)]."""
    w = CdrWriter().u32(len(items))
    for parent, child, t, x, y, yaw in items:
        w.header(t, parent).string(child).f64(x, y, 0.0).f64(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return w.bytes()


def test_parse_tf_reads_what_was_written():
    items = parse_tf(tf_bytes([("odom", "base_link", 12.5, 1.0, -2.0, 0.3), ("map", "odom", 12.4, 0.5, 0.0, -0.1)]))
    assert [i[:2] for i in items] == [("odom", "base_link"), ("map", "odom")]
    p, c, t, x, y, z, qx, qy, qz, qw = items[0]
    assert t == pytest.approx(12.5) and (x, y) == (1.0, -2.0) and qz == pytest.approx(math.sin(0.15))


def test_tflite_composes_map_odom_base_and_the_static_laser():
    tf = TfLite()
    tf.add(tf_bytes([("odom", "base_link", 10.0, 1.0, 0.0, math.pi / 2)]))
    assert tf.frame_pose("base_link")[:4] == ("odom", 1.0, 0.0, pytest.approx(math.pi / 2))     # no map yet
    tf.add(tf_bytes([("map", "odom", 9.9, 2.0, 1.0, math.pi / 2)]))
    tf.add(tf_bytes([("base_link", "laser", 0.0, 0.1, 0.0, 0.0)]), static=True)
    f, x, y, yaw, t = tf.frame_pose("base_link")
    assert f == "map" and (x, y) == (pytest.approx(2.0), pytest.approx(2.0)) and abs(yaw) == pytest.approx(math.pi)
    assert t == pytest.approx(9.9)                           # the oldest stamp on the chain
    f, x, y, yaw, t = tf.frame_pose("laser")                 # 0.1 m ahead of the base, which faces -x on the map
    assert (x, y) == (pytest.approx(1.9), pytest.approx(2.0)) and t == pytest.approx(9.9)   # static: no stamp
    assert tf.lookup("base_link", "laser")[:2] == (pytest.approx(0.1), 0.0)
    tf.add(b"\x00\x01\x00\x00\xff")                           # garbage is ignored
    assert tf.frame_pose("nowhere") is None


def test_ball_track_fields():
    w = CdrWriter().header(3.25, "odom").u8(1).u8(0).u32(17).f64(*range(6)).f64(*([0.5] * 36)).f64(1.0, 2.0, 0.4)
    d = parse_ball_track(w.f64(0.81, 0.07).bytes())
    assert d["tracking"] and not d["ready"] and d["frames"] == 17 and d["x"] == [0, 1, 2, 3, 4, 5]
    assert d["crossing"] == [1.0, 2.0, 0.4] and d["t_go"] == 0.81 and d["crossing_sigma"] == 0.07


def test_reader_alignment_after_odd_strings():
    b = CdrWriter().string("ab").f64(1.5).u8(7).f32(2.5).string("").u32(9).bytes()
    r = CdrReader(b)
    assert (r.string(), r.f64(), r.u8(), r.f32(), r.string(), r.u32()) == ("ab", 1.5, 7, 2.5, "", 9)


def test_against_rclpy_serializer():
    rclpy_ser = pytest.importorskip("rclpy.serialization")
    from omni_interfaces.msg import BallTrack
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    m = TFMessage()
    for parent, child, x in (("map", "odom", 0.25), ("odom", "base_link", -1.5)):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp.sec, t.header.stamp.nanosec = 41, 500_000_000
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        m.transforms.append(t)
    items = parse_tf(rclpy_ser.serialize_message(m))
    assert [(i[0], i[1], i[2], i[3]) for i in items] == [("map", "odom", 41.5, 0.25), ("odom", "base_link", 41.5, -1.5)]
    bt = BallTrack()
    bt.tracking, bt.frames, bt.t_go = True, 5, 0.5
    bt.x = [float(k) for k in range(6)]
    assert parse_ball_track(rclpy_ser.serialize_message(bt))["x"][5] == 5.0
