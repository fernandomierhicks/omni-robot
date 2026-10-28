"""The launch files' common body (design.md 28): which nodes each step runs. Today every step runs against the MuJoCo
sim (omni_sim's bridge stands in for the robot's drivers); on the robot the same stack runs with the real drivers.

  sim           the bridge alone (wheel odometry publishes odom -> base_link), RViz, Foxglove, joystick teleop
  mapping       + robot_localization + scan prep + slam_toolbox mapping; drive it with the stick, then `omni.sh save-map`
  auto_map      the same, driven along v9's house tour on the sim's truth; saves the map when the tour ends, then exits
  localization  + slam_toolbox localization on maps/<map>; drive it with the stick
  navigation    localization + Nav2 (send goals from RViz's "2D Goal Pose" or Foxglove)
  robot         the whole robot at once for the operator console: EKF, scan prep, Nav2, and slam_toolbox run by the
                console (mapping, or localizing on maps/<map>), so Manual, Go To, Explore, Patrol, the SLAM tab and the
                Thrower tab switch at runtime; Catch too (catch:=true, the default): when the mode turns CATCH a
                thrower steps in front of the robot in the house (the bridge's house_catch), and the robot's catch
                code (catch_perception, catch_behavior) plays there. 'catch' is still the open field
  tour          localization, driven along v9's tour with v9's estimators in shadow; the scorer writes the result
  goals         navigation + nav_goals (doorways, rugs); writes the result
  arm           design.md 11: the arm robot (the SO-101 on its nose instead of the catch module) in the house with small
                things on the floor: everything 'robot' runs, the bridge also serving the arm's servo bus and cameras,
                and omni_arm's arm_node (the console's Tidy tab starts its tidy task)
  catch         v11 (design.md 30): the catch game. The catch bridge (field, thrower, cameras, the robot with its
                firmware) + the robot's catch code: mcu_link, catch_perception, catch_behavior, mode_manager (CATCH)
  catch_score   the same, no views, `tosses` throws; catch_scorer writes studies/results/<out>.json

v11, firmware (default true): the robot's C++ firmware runs in the loop (SIL) for every step but tour / auto_map
(they drive on the truth through the Python controller, as v10 scored them). Then mcu_link connects to its USB port
(TCP from the bridge) and publishes /odom, /imu and odom -> base_link, and mode_manager (TELEOP) forwards /cmd_vel.
firmware:=false is v10's bridge.

Arguments (name:=value): map (house), seed (0), rviz, foxglove, joy, viewer (MuJoCo's 3D window), console (the operator
console, http://localhost:8080) (true/false; defaults by step), web (true: the console's own page server; false when
omni_console's supervisor serves the page and starts the steps: omni.sh console), fast (the sim's firmware at 1 kHz and 2 ms lidar
batches: ~1.0x real time; off for the scored steps, which keep v10's published settings), flatness (deg; -1 = the A2M12 model's drawn tilt), rtf (real-time factor the bridge
paces to; 0 = flat out), slam_range (m), floor_filter / deskew (scan prep), out (result name), firmware; goals step: speed, heading,
spin_rate (the console's Go To settings), route (house | sprint: the hallway end to end); catch steps:
dist (m, 6.5), floor (flat | home), returns (true), tosses (catch_score: 24), kid (false), pass_game (false: two
throwers 2.5 m apart taking turns, the robot returning each ball to the other one). person (false; house steps): a
person walking ahead of the robot, for the console's Follow / Tag. mcu_port (5761): the sim's
firmware USB port on localhost TCP; another value lets a second run go alongside (with its own ROS_DOMAIN_ID).
timing (ideal | real) and sync_gpio (none | hte | irq), house steps with the firmware (design.md 10): the MCU's clock
drifts and its CAN replies arrive as on the real bus; the Jetson stamps the firmware's 1 Hz SYNC_OUT on a modelled
GPIO and mcu_link maps the MCU's clock from those pulses (/mcu/clock says how well). tags (false; house steps):
AprilTags on the walls, the camera's detections simulated by the bridge, omni_slam's tag_slam alongside slam_toolbox
(/tag_slam/*, /localization/health: the console slows with it). arm step: objects (toys | clutter), n_objects (6),
servo_port (5763: the arm's servo bus on localhost TCP). robot (house steps): drawn (the interactive steps: the
robot as cad/v7 draws it, robot_export.drawn_params: the CAD's masses, its scan plane and lidar shadows; lidar_mask:=false
drops the shadows) | v7 (the scored steps tour / auto_map / goals, as published). catch (robot step, true): the catch game in the house.
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
    "robot": ("false", "true", "false", "true", "true", "true"),
    "arm": ("false", "true", "false", "true", "true", "true"),
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
            DeclareLaunchArgument("web", default_value="true"), DeclareLaunchArgument("person", default_value="false"),
            DeclareLaunchArgument("pass_game", default_value="false"),
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
            DeclareLaunchArgument("timing", default_value="ideal"),
            DeclareLaunchArgument("sync_gpio", default_value="none"),
            DeclareLaunchArgument("tags", default_value="false"),
            DeclareLaunchArgument("objects", default_value="toys"),
            DeclareLaunchArgument("n_objects", default_value="6"),
            DeclareLaunchArgument("servo_port", default_value="5763"),
            DeclareLaunchArgument("robot", default_value="v7" if kind in ("tour", "auto_map", "goals") else "drawn"),
            DeclareLaunchArgument("catch", default_value="true" if kind == "robot" else "false"),
            # the Launcher's live world (simulation/v11 sil/sim_world.py) in the robot and arm steps: clutter off |
            # toys | all (default: toys for the arm robot, off for the other), the house's thrower on / off
            DeclareLaunchArgument("clutter", default_value="toys" if kind == "arm" else "off"),
            DeclareLaunchArgument("thrower", default_value="true"),
            DeclareLaunchArgument("lidar_mask", default_value="true"),
            # goals step: the console's Go To settings (0 / "" = nav2.yaml's own)
            DeclareLaunchArgument("speed", default_value="0.0"),
            DeclareLaunchArgument("heading", default_value=""),
            DeclareLaunchArgument("spin_rate", default_value="120.0"),
            DeclareLaunchArgument("route", default_value="house"),      # goals step: house | sprint (nav_goals.py)
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
                               "viewer": on("viewer"), "pass_game": on("pass_game")}])
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
                          "kid": on("kid"), "return_to": "other" if on("pass_game") else "thrower", **sim_t}]),
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


def console_nodes(a: dict, scan: str, slam: tuple | None = None):
    """The operator console (omni_console): http://localhost:8080. rosbridge carries the page's topics and its one
    command service, rosapi lists nodes and topics for its ROS tab, the mission node switches modes (and runs
    slam_toolbox when given its parameter files: slam = (mapping, localization template)), web_video_server streams
    camera topics as MJPEG on 8081 (if installed). None of them runs on sim time: that would subscribe each to the
    sim's 1 kHz /clock (in Python: ~60% of a core for the mission node alone). They need no stamps of their own (TF
    lookups take the latest, Nav2 goals carry none); slam_toolbox, their child, does run on sim time."""
    mission = {"map": a["map"], "places_dir": str(maps_dir()), "scan_topic": scan,
               "log_source": "sim"}                          # session logs say the sim stood in for the robot
    if slam:
        mission.update(slam_mapping_params=slam[0], slam_localization_params=slam[1], slam_sim_time=True,
                       slam_start="localization")
    nodes = [
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket", output="log",
             parameters=[{"port": 9090, "address": ""}]),
        Node(package="rosapi", executable="rosapi_node", name="rosapi", output="log"),
        Node(package="omni_console", executable="mission_node", name="omni_console", output="screen",
             parameters=[mission]),
        # the cameras' small previews for the Vision tab (idle while nobody watches; no camera topics in the sim yet)
        Node(package="omni_perception", executable="image_preview", name="image_preview", output="log",
             parameters=[{"inputs": ["/cam/left/image_raw", "/cam/right/image_raw"]}]),
        # the robot's face and voice (omni_face): in the sim its window opens on Windows, with sound
        Node(package="omni_face", executable="face_node", name="omni_face", output="screen",
             parameters=[{"bind": "0.0.0.0", "start_display": "auto"}]),
    ]
    if a["web"].lower() in ("1", "true", "yes"):     # web:=false when the supervisor serves the page (omni.sh console)
        nodes.append(Node(package="omni_console", executable="console_web", name="console_web", output="screen"))
    try:
        get_package_share_directory("web_video_server")
        nodes.append(Node(package="web_video_server", executable="web_video_server", name="web_video_server",
                          output="log", parameters=[{"port": 8081, "address": "0.0.0.0"}]))
    except Exception:                                # not installed: the Vision tab says so
        pass
    return nodes


def build(ctx, kind: str):
    a = {k: ctx.launch_configurations[k] for k in ("map", "seed", "rviz", "foxglove", "joy", "viewer", "fast", "flatness",
                                                   "console", "rtf", "out", "slam_range", "floor_filter", "deskew", "firmware",
                                                   "dist", "floor", "returns", "tosses", "kid", "mcu_port", "web",
                                                   "person", "pass_game", "timing", "sync_gpio", "tags", "objects",
                                                   "n_objects", "servo_port", "robot", "catch", "lidar_mask", "clutter",
                                                   "thrower", "speed", "heading", "spin_rate", "route")}
    on = lambda k: a[k].lower() in ("1", "true", "yes")          # noqa: E731
    if kind in ("catch", "catch_score"):
        return catch_nodes(a, on, kind)
    sim_t = {"use_sim_time": True}
    acts = []
    tour = kind in ("tour", "auto_map")
    ekf = kind != "sim"
    arm = kind == "arm"
    robot = kind in ("robot", "arm")
    fast = on("fast")
    fw = (on("firmware") or arm) and not tour
    arm_kw = dict(payload="arm", objects=a["objects"], n_objects=int(a["n_objects"]),
                  servo_port=int(a["servo_port"])) if arm else {}
    acts.append(Node(package=SIM, executable="bridge", name="mujoco_bridge", output="screen",
                     additional_env={"MUJOCO_GL": "egl"} if arm else None,       # the arm's cameras render in WSL
                     parameters=[{"mode": "tour" if tour else "cmd_vel", "seed": int(a["seed"]),
                                  "publish_odom_tf": not ekf, "shadow": kind == "tour",
                                  "lidar_flatness_deg": float(a["flatness"]), "real_time_factor": float(a["rtf"]),
                                  "viewer": on("viewer"), "markers": on("foxglove") or on("rviz"),
                                  "fw_div": 2 if fast else 1, "hook_div": 4 if fast else 1,
                                  "lidar_cast_dt": 0.002 if fast else 0.001, "firmware": fw,
                                  "clock_period": 0.001 if fw else 0.004, "port": int(a["mcu_port"]),
                                  "person": on("person"), "timing": a["timing"], "sync_gpio": a["sync_gpio"],
                                  "tags": on("tags"), "base": a["robot"], "lidar_mask": on("lidar_mask"),
                                  "catch": on("catch") and fw and not arm, "catch_dist": float(a["dist"]),
                                  "catch_returns": on("returns"), "live_world": robot, "clutter": a["clutter"],
                                  "thrower": on("thrower"), "control_port": 9873 if robot else 0, **arm_kw}]))
    if arm:
        # design.md 11: the arm's own code, on the sim's servo bus (the bus board's USB port on the robot)
        acts.append(Node(package="omni_arm", executable="arm_node", name="arm_node", output="screen",
                         parameters=[{"port": f"tcp:127.0.0.1:{int(a['servo_port'])}", "odom_topic": "/odom",
                                      **sim_t}]))
    if fw:
        # the robot's own MCU link and mode manager: /odom, /imu and odom -> base_link from the firmware (the EKF
        # takes the TF over when it runs), /cmd_vel to the firmware
        sync = {"sync_source": "topic"} if a["sync_gpio"] != "none" else {}
        acts.append(Node(package="omni_mcu", executable="mcu_link", name="mcu_link", output="screen",
                         parameters=[{"port": f"tcp:127.0.0.1:{int(a['mcu_port'])}", "publish_tf": not ekf, **sync,
                                      **sim_t}]))
        acts.append(Node(package="omni_catch", executable="mode_manager", name="mode_manager", output="screen",
                         parameters=[{"mode": "TELEOP", "kid": on("kid"), **sim_t}]))
        if on("catch") and not arm:
            # the catch game in the house: the robot's catch code on the bridge's camera frames; the field comes on
            # /catch/setup when the mode turns CATCH (the parameters are the open field's, until then)
            acts.append(Node(package="omni_catch", executable="catch_perception", name="catch_perception",
                             output="screen", parameters=[{"source": "sparse", **sim_t}]))
            acts.append(Node(package="omni_catch", executable="catch_behavior", name="catch_behavior",
                             output="screen", parameters=[{"return_throws": on("returns"), "kid": on("kid"),
                                                           **sim_t}]))
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
    if kind in ("navigation", "goals", "robot", "arm"):
        nav = open(cfg("nav2.yaml")).read().replace("topic: /scan\n", f"topic: {scan}\n")
        acts += nav2_nodes(tmp_yaml(nav, "nav2.yaml"))
    if on("tags") and not tour:
        # design.md 10: AprilTags on the house's walls (the bridge simulates the camera's detections) as landmarks,
        # next to slam_toolbox (it keeps map -> odom); the localization health for the console
        # one map -> odom publisher only (REP-105): slam_toolbox's wherever it runs, tag_slam's only without it
        slam_running = kind in ("mapping", "localization", "navigation", "robot", "arm", "goals")
        acts.append(Node(package="omni_slam", executable="tag_slam", name="tag_slam", output="screen",
                         parameters=[{"map": a["map"], "odom_topic": "/odometry/filtered" if ekf else "/odom",
                                      "lidar": slam_running, "publish_map_odom": not slam_running, **sim_t}]))
    if on("rviz"):
        acts.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                         arguments=["-d", cfg("omni.rviz")], parameters=[sim_t]))
    if on("foxglove"):
        acts.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
                         output="log", parameters=[{"port": 8765, "address": "0.0.0.0", **sim_t}]))
    if on("console") or robot:
        acts += console_nodes(a, scan, (slam_yaml("slam_mapping.yaml"), slam_yaml("slam_localization.yaml"))
                              if robot else None)
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
                   parameters=[{"out": a["out"], "speed": float(a["speed"]), "spin_rate": float(a["spin_rate"]),
                                "route": a["route"],
                                **({"heading": a["heading"]} if a["heading"] else {}), **sim_t}])
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
             output="log", parameters=p, remappings=[("cmd_vel", nav), ("cmd_vel_smoothed", "cmd_vel_shape")]),
        # which way the robot faces on the way (the console's "Driving" choice, whose default it starts with)
        Node(package=PKG, executable="heading_node", name="heading_node", output="log",
             parameters=[{"heading": "forward"}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="log", parameters=p),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"use_sim_time": True, "autostart": True,
                                           "node_names": ["controller_server", "smoother_server", "planner_server",
                                                          "behavior_server", "velocity_smoother",
                                                          "collision_monitor", "bt_navigator"]}]),
    ]
    return nodes
