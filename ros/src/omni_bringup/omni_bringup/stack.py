"""The launch files' common body (design.md 28): which nodes each step runs. Today every step runs against the MuJoCo
sim (omni_sim's bridge stands in for the robot's drivers); on the robot the same stack runs with the real drivers.

  sim           the bridge alone (wheel odometry publishes odom -> base_link), RViz, Foxglove, joystick teleop
  mapping       + robot_localization + scan prep + slam_toolbox mapping; drive it with the stick, then `omni.sh save-map`
  auto_map      the same, driven along v9's house tour on the sim's truth; saves the map when the tour ends, then exits
  localization  + slam_toolbox localization on maps/<map>; drive it with the stick
  navigation    localization + Nav2 (send goals from RViz's "2D Goal Pose" or Foxglove)
  tour          localization, driven along v9's tour with v9's estimators in shadow; the scorer writes the result
  goals         navigation + nav_goals (doorways, rugs); writes the result
  catch         v11 (design.md 30): the catch game. The catch bridge (field, thrower, cameras, the robot with its
                firmware) + the robot's catch code: mcu_link, catch_perception, catch_behavior, mode_manager (CATCH)
  catch_score   the same, no views, `tosses` throws; catch_scorer writes studies/results/<out>.json

v11, firmware (default true): the robot's C++ firmware runs in the loop (SIL) for every step but tour / auto_map
(they drive on the truth through the Python controller, as v10 scored them). Then mcu_link connects to its USB port
(TCP from the bridge) and publishes /odom, /imu and odom -> base_link, and mode_manager (TELEOP) forwards /cmd_vel.
firmware:=false is v10's bridge.

Arguments (name:=value): map (house), seed (0), rviz, foxglove, joy, viewer (MuJoCo's 3D window), console (the operator
console, http://localhost:8080) (true/false; defaults by step), fast (the sim's firmware at 1 kHz and 2 ms lidar
batches: ~1.0x real time; off for the scored steps, which keep v10's published settings), flatness (deg; -1 = the A2M12 model's drawn tilt), rtf (real-time factor the bridge
paces to; 0 = flat out), slam_range (m), floor_filter / deskew (scan prep), out (result name), firmware; catch steps:
dist (m, 6.5), floor (flat | home), returns (true), tosses (catch_score: 24), kid (false). mcu_port (5761): the sim's
firmware USB port on localhost TCP; another value lets a second run go alongside (with its own ROS_DOMAIN_ID).
"""

from __future__ import annotations

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from omni_bringup import maps_dir

PKG = "omni_bringup"
SIM = "omni_sim"
DEFAULTS = {               # step: rviz, foxglove, joy, viewer, fast, console
    "sim": ("false", "true", "true", "true", "true", "true"),
    "mapping": ("false", "true", "true", "true", "true", "true"),
    "auto_map": ("false", "false", "false", "false", "false", "false"),
    "localization": ("false", "true", "true", "true", "true", "true"),
    "navigation": ("false", "true", "false", "true", "true", "true"),
    "tour": ("false", "false", "false", "false", "false", "false"),
    "goals": ("false", "false", "false", "false", "false", "false"),
    "catch": ("false", "true", "false", "true", "true", "true"),
    "catch_score": ("false", "false", "false", "false", "false", "false"),
}
CATCH_HOME_X = 3.5             # catch_world.HOME / AREA: the field the catch bridge builds (odom frame below)
CATCH_AREA = (12.0, 6.0)


def cfg(name: str) -> str:
    return os.path.join(get_package_share_directory(PKG), "config", name)


def describe(kind: str) -> LaunchDescription:
    rv, fx, jy, vw, fa, co = DEFAULTS[kind]
    args = [DeclareLaunchArgument("map", default_value="house"), DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("rviz", default_value=rv), DeclareLaunchArgument("foxglove", default_value=fx),
            DeclareLaunchArgument("joy", default_value=jy), DeclareLaunchArgument("viewer", default_value=vw),
            DeclareLaunchArgument("fast", default_value=fa), DeclareLaunchArgument("console", default_value=co),
            DeclareLaunchArgument("flatness", default_value="-1.0"),
            DeclareLaunchArgument("rtf", default_value="1.0"),
            DeclareLaunchArgument("slam_range", default_value="6.0"),
            DeclareLaunchArgument("floor_filter", default_value="true"),
            DeclareLaunchArgument("deskew", default_value="true"),
            DeclareLaunchArgument("firmware", default_value="true"),
            DeclareLaunchArgument("mcu_port", default_value="5761"),
            DeclareLaunchArgument("dist", default_value="6.5"),
            DeclareLaunchArgument("floor", default_value="flat"),
            DeclareLaunchArgument("returns", default_value="true"),
            DeclareLaunchArgument("tosses", default_value="24" if kind == "catch_score" else "0"),
            DeclareLaunchArgument("kid", default_value="false"),
            DeclareLaunchArgument("out", default_value={"tour": "ros_tour", "goals": "ros_nav_goals",
                                                        "catch_score": "ros_catch"}.get(kind, ""))]
    return LaunchDescription(args + [OpaqueFunction(function=lambda ctx: build(ctx, kind))])


def catch_nodes(a: dict, on, kind: str):
    """v11 (design.md 30): the catch game. The bridge is the world and the MCU; the rest is the robot's own code."""
    sim_t = {"use_sim_time": True}
    dist = float(a["dist"])
    score = kind == "catch_score"
    bridge = Node(package=SIM, executable="catch_bridge", name="catch_bridge", output="screen",
                  parameters=[{"dist": dist, "seed": 9000 + int(a["seed"]), "floor": a["floor"],
                               "port": int(a["mcu_port"]), "returns": on("returns"), "tosses": int(a["tosses"]), "rtf": float(a["rtf"]),
                               "viewer": on("viewer")}])
    # the robot starts at home facing the thrower: its odom frame has the thrower dist m ahead and the field's centre
    # CATCH_HOME_X ahead (what a person detector and the map would give it)
    robot = [
        Node(package="omni_mcu", executable="mcu_link", name="mcu_link", output="screen",
             parameters=[{"port": f"tcp:127.0.0.1:{int(a['mcu_port'])}", "publish_tf": True, **sim_t}]),
        Node(package="omni_catch", executable="mode_manager", name="mode_manager", output="screen",
             parameters=[{"mode": "CATCH", "kid": on("kid"), **sim_t}]),
        Node(package="omni_catch", executable="catch_perception", name="catch_perception", output="screen",
             parameters=[{"source": "sparse", "arena_cx": CATCH_HOME_X, "arena_cy": 0.0, "arena_hx": CATCH_AREA[0] / 2,
                          "arena_hy": CATCH_AREA[1] / 2, **sim_t}]),
        Node(package="omni_catch", executable="catch_behavior", name="catch_behavior", output="screen",
             parameters=[{"thrower_x": dist, "thrower_y": 0.0, "arena_cx": CATCH_HOME_X, "arena_cy": 0.0,
                          "arena_hx": CATCH_AREA[0] / 2, "arena_hy": CATCH_AREA[1] / 2, "return_throws": on("returns"),
                          "kid": on("kid"), **sim_t}]),
    ]
    acts = [bridge] + robot
    if score:
        acts.append(Node(package=SIM, executable="catch_scorer", name="catch_scorer", output="screen",
                         parameters=[{"out": a["out"], "label": f"dist {dist}, seed {a['seed']}, {a['floor']}", **sim_t}]))
        acts.append(RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[EmitEvent(event=Shutdown())])))
    if on("foxglove"):
        acts.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
                         output="log", parameters=[{"port": 8765, "address": "0.0.0.0", **sim_t}]))
    if on("console"):
        acts += console_nodes(a, "/scan")
    return acts


def console_nodes(a: dict, scan: str):
    """The operator console (omni_console): http://localhost:8080. rosbridge carries the page's topics and its one
    command service, rosapi lists nodes and topics for its ROS tab, the mission node switches modes. None of them runs
    on sim time: that would subscribe each to the sim's 1 kHz /clock (in Python: ~60% of a core for the mission node
    alone). They need no stamps of their own (TF lookups take the latest, Nav2 goals carry none)."""
    return [
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket", output="log",
             parameters=[{"port": 9090, "address": ""}]),
        Node(package="rosapi", executable="rosapi_node", name="rosapi", output="log"),
        Node(package="omni_console", executable="mission_node", name="omni_console", output="screen",
             parameters=[{"map": a["map"], "places_dir": str(maps_dir()), "scan_topic": scan}]),
        Node(package="omni_console", executable="console_web", name="console_web", output="screen"),
    ]


def build(ctx, kind: str):
    a = {k: ctx.launch_configurations[k] for k in ("map", "seed", "rviz", "foxglove", "joy", "viewer", "fast", "flatness",
                                                   "console", "rtf", "out", "slam_range", "floor_filter", "deskew", "firmware",
                                                   "dist", "floor", "returns", "tosses", "kid", "mcu_port")}
    on = lambda k: a[k].lower() in ("1", "true", "yes")          # noqa: E731
    if kind in ("catch", "catch_score"):
        return catch_nodes(a, on, kind)
    sim_t = {"use_sim_time": True}
    acts = []
    tour = kind in ("tour", "auto_map")
    ekf = kind != "sim"
    fast = on("fast")
    fw = on("firmware") and not tour
    acts.append(Node(package=SIM, executable="bridge", name="mujoco_bridge", output="screen",
                     parameters=[{"mode": "tour" if tour else "cmd_vel", "seed": int(a["seed"]),
                                  "publish_odom_tf": not ekf, "shadow": kind == "tour",
                                  "lidar_flatness_deg": float(a["flatness"]), "real_time_factor": float(a["rtf"]),
                                  "viewer": on("viewer"), "markers": on("foxglove") or on("rviz"),
                                  "fw_div": 2 if fast else 1, "hook_div": 4 if fast else 1,
                                  "lidar_cast_dt": 0.002 if fast else 0.001, "firmware": fw,
                                  "clock_period": 0.001 if fw else 0.004, "port": int(a["mcu_port"])}]))
    if fw:
        # the robot's own MCU link and mode manager: /odom, /imu and odom -> base_link from the firmware (the EKF
        # takes the TF over when it runs), /cmd_vel to the firmware
        acts.append(Node(package="omni_mcu", executable="mcu_link", name="mcu_link", output="screen",
                         parameters=[{"port": f"tcp:127.0.0.1:{int(a['mcu_port'])}", "publish_tf": not ekf, **sim_t}]))
        acts.append(Node(package="omni_catch", executable="mode_manager", name="mode_manager", output="screen",
                         parameters=[{"mode": "TELEOP", "kid": on("kid"), **sim_t}]))
    if ekf:
        acts.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="log",
                         parameters=[cfg("ekf.yaml"), sim_t]))
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch")
    prep = (on("floor_filter") or on("deskew")) and kind != "sim"
    scan = "/scan_filtered" if prep else "/scan"
    if prep:
        acts.append(Node(package="omni_perception", executable="scan_prep", name="scan_prep", output="screen",
                         parameters=[{"floor": on("floor_filter"), "deskew": on("deskew"), **sim_t}]))

    def slam_yaml(name, extra=""):
        src = open(cfg(name)).read().replace("    max_laser_range: 12.0", f"    max_laser_range: {float(a['slam_range'])}")
        return tmp_yaml(src.replace("scan_topic: /scan", f"scan_topic: {scan}") + extra, name)

    if kind in ("mapping", "auto_map"):
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch, "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_yaml("slam_mapping.yaml"), "use_sim_time": "true"}.items()))
    if kind in ("localization", "navigation", "tour", "goals"):
        base = str(maps_dir() / a["map"])
        if not os.path.exists(base + ".posegraph"):
            raise RuntimeError(f"no serialized map at {base}.posegraph: run `omni.sh map` + `save-map` or `auto-map`")
        extra = f"\n    map_file_name: {base}\n    map_start_pose: [0.0, 0.0, 0.0]\n"
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch, "localization_launch.py")),
            launch_arguments={"slam_params_file": slam_yaml("slam_localization.yaml", extra),
                              "use_sim_time": "true"}.items()))
    if kind in ("navigation", "goals"):
        nav = open(cfg("nav2.yaml")).read().replace("topic: /scan\n", f"topic: {scan}\n")
        acts += nav2_nodes(tmp_yaml(nav, "nav2.yaml"))
    if on("rviz"):
        acts.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                         arguments=["-d", cfg("omni.rviz")], parameters=[sim_t]))
    if on("foxglove"):
        acts.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
                         output="log", parameters=[{"port": 8765, "address": "0.0.0.0", **sim_t}]))
    if on("console"):
        acts += console_nodes(a, scan)
    if on("joy"):
        acts.append(Node(package=SIM, executable="joy_relay", name="joy_relay", output="screen"))
        acts.append(Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
                         output="log", parameters=[cfg("teleop_extreme3d.yaml")]))
    # steps that end by themselves: when their last node exits, shut the launch down
    end = None
    if kind == "auto_map":
        end = Node(package=PKG, executable="save_map", name="map_saver_omni", output="screen",
                   parameters=[{"name": a["map"], "wait_for_done": True, **sim_t}])
    elif kind == "tour":
        end = Node(package=SIM, executable="scorer", name="loc_scorer", output="screen",
                   parameters=[{"out": a["out"], "label": f"seed {a['seed']}", "map": a["map"], **sim_t}])
    elif kind == "goals":
        end = Node(package=SIM, executable="nav_goals", name="nav_goals", output="screen",
                   parameters=[{"out": a["out"], **sim_t}])
    if end is not None:
        acts.append(end)
        acts.append(RegisterEventHandler(OnProcessExit(target_action=end, on_exit=[EmitEvent(event=Shutdown())])))
    return acts


def tmp_yaml(text: str, name: str) -> str:
    f = tempfile.NamedTemporaryFile("w", suffix="_" + name, delete=False)
    f.write(text)
    f.close()
    return f.name


def nav2_nodes(params_file: str):
    p = [params_file, {"use_sim_time": True, "autostart": True}]
    nav = "cmd_vel_nav"
    nodes = [
        Node(package="nav2_controller", executable="controller_server", output="log", parameters=p,
             remappings=[("cmd_vel", nav)]),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="log", parameters=p),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="log", parameters=p),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="log",
             parameters=p, remappings=[("cmd_vel", nav)]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="log", parameters=p),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="log", parameters=p, remappings=[("cmd_vel", nav)]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="log", parameters=p),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"use_sim_time": True, "autostart": True,
                                           "node_names": ["controller_server", "smoother_server", "planner_server",
                                                          "behavior_server", "velocity_smoother",
                                                          "collision_monitor", "bt_navigator"]}]),
    ]
    return nodes
