"""Nav2 goals through the house's doorways and across its rugs (design.md 28), scored against ground truth.

The robot starts in the hallway, localized by slam_toolbox on the saved map. Each goal is sent with NavigateToPose;
recorded: Nav2's result, the time it took (sim), the true path length, the true distance and heading error at the end,
the recoveries Nav2 ran, the true speed (peak and median while moving, over 0.25 s), how far the heading was from the
direction of travel while moving > 0.3 m/s (median, deg) and how much it turned (deg). The edges between rooms are
under the lidar's plane, so Nav2 doesn't know they are there; the firmware's terrain assist (design.md 25.4) still caps
the speed at them.
Parameters: speed (m/s, 0 = nav2.yaml's own) sets the velocity smoother's top speed as the console's Go To does;
heading ("" = the launch default: forward | agnostic | spinning) and spin_rate (deg/s) set the heading node's mode;
route (house = the eight goals | sprint = the hallway end to end, to see the top speed).
Writes studies/results/<out>.json and exits (which ends the launch).
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
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
# route "sprint": the hallway end to end (open to the rooms below, no edges): long enough to reach Nav2's top speed
SPRINT = [
    ("east end of the hallway, 8.2 m", 5.2, 1.25, 0),
    ("west end of the hallway, 10.4 m", -5.2, 1.25, 180),
    ("back to the hallway start", -3.0, 1.25, 0),
]
ROUTES = {"house": GOALS, "sprint": SPRINT}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def motion_stats(track: list) -> dict:
    """Speed and heading against travel from (t, x, y, yaw) samples: speeds over 0.25 s windows."""
    speeds, off, turned = [], [], 0.0
    j = 0
    for i in range(1, len(track)):
        turned += abs(wrap(track[i][3] - track[i - 1][3]))
        while track[i][0] - track[j][0] > 0.25:
            j += 1
        if j == 0 or track[i][0] - track[j - 1][0] < 0.2:
            continue
        t0, x0, y0, _ = track[j - 1]
        t1, x1, y1, yaw = track[i]
        v = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
        speeds.append(v)
        if v > 0.3:
            off.append(abs(math.degrees(wrap(math.atan2(y1 - y0, x1 - x0) - yaw))))
    moving = sorted(s for s in speeds if s > 0.05)
    off.sort()
    return dict(v_peak=max(speeds, default=0.0), v_median=moving[len(moving) // 2] if moving else 0.0,
                heading_off=off[len(off) // 2] if off else None, turned=math.degrees(turned))


def _pm(d) -> str:
    return "-" if d is None else f"{d['peak']:.2f} / {d['median']:.2f}"


def set_heading(nav, speed: float, heading: str, spin_rate: float):
    """The console's Go To settings: the smoother's top speed, the heading node's mode."""
    from rcl_interfaces.srv import SetParameters
    from rclpy.parameter import Parameter
    sets = []
    if speed > 0:
        sets.append(("velocity_smoother", [Parameter("max_velocity", value=[speed, speed, 2.0]),
                                           Parameter("min_velocity", value=[-speed, -speed, -2.0])]))
    if heading:
        sets.append(("heading_node", [Parameter("heading", value=heading),
                                      Parameter("spin_rate", value=math.radians(spin_rate))]))
    for node, params in sets:
        cli = nav.create_client(SetParameters, f"/{node}/set_parameters")
        if not cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"{node} has no parameter service")
        fut = cli.call_async(SetParameters.Request(parameters=[p.to_parameter_msg() for p in params]))
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=10.0)
        bad = [r.reason for r in (fut.result().results if fut.result() else []) if not r.successful]
        if fut.result() is None or bad:
            raise RuntimeError(f"{node} refused {params}: {bad}")
        nav.get_logger().info(f"{node}: {', '.join(f'{p.name}={p.value}' for p in params)}")


def main():
    rclpy.init()
    nav = BasicNavigator(node_name="nav_goals")
    out_name = nav.declare_parameter("out", "ros_nav_goals").value
    timeout = nav.declare_parameter("goal_timeout", 150.0).value
    speed = float(nav.declare_parameter("speed", 0.0).value)
    heading = str(nav.declare_parameter("heading", "").value)
    spin_rate = float(nav.declare_parameter("spin_rate", 120.0).value)
    route = str(nav.declare_parameter("route", "house").value)
    st = dict(origin=None, truth=None, path=0.0, last=None, track=[])

    def on_origin(m):
        st["origin"] = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_truth(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        if st["last"] is not None:
            st["path"] += math.hypot(p[0] - st["last"][0], p[1] - st["last"][1])
        st["last"] = st["truth"] = p
        st["track"].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, *p))

    # commanded speeds along Nav2's chain: MPPI's own (cmd_vel_nav), and what leaves the collision monitor (cmd_vel)
    cmds = {"/cmd_vel_nav": [], "/cmd_vel": []}

    def on_cmd(topic):
        return lambda m: cmds[topic].append(math.hypot(m.linear.x, m.linear.y))

    for topic in cmds:
        nav.create_subscription(Twist, topic, on_cmd(topic), 20)
    nav.create_subscription(PoseStamped, "/sim/origin", on_origin, LATCHED)
    nav.create_subscription(Odometry, "/ground_truth/odom", on_truth, 20)
    while st["origin"] is None or st["truth"] is None:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav.waitUntilNav2Active(localizer="robot_localization")      # slam_toolbox localizes; nothing to wait for
    set_heading(nav, speed, heading, spin_rate)
    ox, oy, oyaw = st["origin"]
    c, s = math.cos(oyaw), math.sin(oyaw)
    rows = []
    for label, gx, gy, gyaw in ROUTES[route]:
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
        st["track"] = []
        for v in cmds.values():
            v.clear()
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
                   yaw_err=abs(math.degrees(wrap(p[2] - math.radians(gyaw)))), recoveries=rec, end=list(p),
                   **motion_stats(st["track"]))
        for topic, key in (("/cmd_vel_nav", "cmd_nav"), ("/cmd_vel", "cmd_out")):
            v = sorted(s for s in cmds[topic] if s > 0.05)
            row[key] = dict(peak=v[-1], median=v[len(v) // 2]) if v else None
        rows.append(row)
        off = "-" if row["heading_off"] is None else f"{row['heading_off']:.0f}"
        nav.get_logger().info(f"{label}: {row['result']} in {dt:.1f} s, {row['path']:.1f} m, ends "
                              f"{100 * row['err']:.1f} cm / {row['yaw_err']:.1f} deg from the goal, {rec} recoveries; "
                              f"peak {row['v_peak']:.2f} m/s, median {row['v_median']:.2f}, heading off travel {off} "
                              f"deg, turned {row['turned']:.0f} deg; commanded peak / median: MPPI "
                              f"{_pm(row['cmd_nav'])}, to the robot {_pm(row['cmd_out'])}")
    ok = [r for r in rows if r["ok"]]
    nav.get_logger().info(f"{len(ok)} of {len(rows)} goals reached; time {sum(r['time'] for r in ok):.0f} s")
    (results_dir() / f"{out_name}.json").write_text(json.dumps(dict(goals=rows, route=route, speed=speed,
                                                                    heading=heading, spin_rate=spin_rate), indent=1))
    nav.destroy_node()
    rclpy.shutdown()
