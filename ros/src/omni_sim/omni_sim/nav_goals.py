"""Nav2 goals through the house's doorways and across its rugs (design.md 28), scored against ground truth.

The robot starts in the hallway, localized by slam_toolbox on the saved map. Each goal is sent with NavigateToPose;
recorded: Nav2's result, the time it took (sim), the true path length, the true distance and heading error at the end,
and the recoveries Nav2 ran. The edges between rooms are under the lidar's plane, so Nav2 doesn't know they are there;
the firmware's terrain assist (design.md 25.4) still caps the speed at them.
Writes studies/results/<out>.json and exits (which ends the launch).
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile

from omni_sim import results_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# (label, x, y, yaw deg) in the house (world) frame, in order from the hallway start (house.START)
GOALS = [
    ("bedroom, over the 25 mm carpet trim onto 12 mm carpet", -3.0, 3.3, 90),
    ("office, over the 5 mm T-molding", 0.45, 3.2, 90),
    ("bathroom, over the 12 mm marble threshold, onto the 6 mm mat", 4.6, 3.25, 0),
    ("kitchen, up the 6 mm tile reducer", 2.5, -2.0, 0),
    ("living room, onto the 12 mm square-edged rug (turned 15 deg)", -3.1, -2.4, -135),
    ("onto the 8 mm runner (turned -30 deg)", -0.8, -0.4, 60),
    ("onto the 16 mm plush rug", -5.0, -0.1, 90),
    ("back to the hallway start", -3.0, 1.25, 0),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rclpy.init()
    nav = BasicNavigator(node_name="nav_goals")
    out_name = nav.declare_parameter("out", "ros_nav_goals").value
    timeout = nav.declare_parameter("goal_timeout", 150.0).value
    st = dict(origin=None, truth=None, path=0.0, last=None)

    def on_origin(m):
        st["origin"] = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_truth(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        if st["last"] is not None:
            st["path"] += math.hypot(p[0] - st["last"][0], p[1] - st["last"][1])
        st["last"] = st["truth"] = p

    nav.create_subscription(PoseStamped, "/sim/origin", on_origin, LATCHED)
    nav.create_subscription(Odometry, "/ground_truth/odom", on_truth, 20)
    while st["origin"] is None or st["truth"] is None:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav.waitUntilNav2Active(localizer="robot_localization")      # slam_toolbox localizes; nothing to wait for
    ox, oy, oyaw = st["origin"]
    c, s = math.cos(oyaw), math.sin(oyaw)
    rows = []
    for label, gx, gy, gyaw in GOALS:
        # world -> map frame
        dx, dy = gx - ox, gy - oy
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = nav.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = c * dx + s * dy, -s * dx + c * dy
        yaw = math.radians(gyaw) - oyaw
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t0 = nav.get_clock().now()
        st["path"] = 0.0
        nav.goToPose(g)
        rec = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                rec = max(rec, fb.number_of_recoveries)
            if nav.get_clock().now() - t0 > Duration(seconds=timeout):
                nav.cancelTask()
        res = nav.getResult()
        dt = (nav.get_clock().now() - t0).nanoseconds * 1e-9
        p = st["truth"]
        row = dict(goal=label, target=[gx, gy, gyaw], result=str(res).split(".")[-1], ok=res == TaskResult.SUCCEEDED,
                   time=dt, path=st["path"], err=math.hypot(p[0] - gx, p[1] - gy),
                   yaw_err=abs(math.degrees(wrap(p[2] - math.radians(gyaw)))), recoveries=rec, end=list(p))
        rows.append(row)
        nav.get_logger().info(f"{label}: {row['result']} in {dt:.1f} s, {row['path']:.1f} m, ends "
                              f"{100 * row['err']:.1f} cm / {row['yaw_err']:.1f} deg from the goal, {rec} recoveries")
    ok = [r for r in rows if r["ok"]]
    nav.get_logger().info(f"{len(ok)} of {len(rows)} goals reached; time {sum(r['time'] for r in ok):.0f} s")
    (results_dir() / f"{out_name}.json").write_text(json.dumps(dict(goals=rows), indent=1))
    nav.destroy_node()
    rclpy.shutdown()
