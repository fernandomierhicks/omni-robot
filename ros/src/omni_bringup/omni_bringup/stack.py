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
servo_port (5763: the arm's servo bus on localhost TCP), arm_camera (stereo | d435: the RealSense D435 on the deck's
nose gives the tidy's depth and Nav2's depth layer, design.md 4.11). robot (house steps): drawn (the interactive steps: the
robot as cad/v7 draws it, robot_export.drawn_params: the CAD's masses, its scan plane and lidar shadows; lidar_mask:=false
drops the shadows) | v7 (the scored steps tour / auto_map / goals, as published). catch (robot step, true): the catch game in the house.
terrain (false; house steps, design.md 16): a terrain course in the kitchen's east end (square steps, triangular bumps).
arm_impl (py): the arm robot's arm as one Python process (arm_node), or cpp: omni_cpp's arm_driver (the servo bus,
joint states and TF at 50 Hz) beside omni_arm's arm_brain (planning and the tidy, design.md 4.10). glue (cpp): mcu_link, mode_manager, scan_prep and heading_node from omni_cpp (C++, design.md 4.9); py: their Python
originals (omni_mcu, omni_catch, omni_perception, omni_bringup). nav_profile (default = full, pi on a Raspberry Pi | lean | pi): lean sets Nav2 and
slam_toolbox for a smaller computer (LEAN below), pi for a Pi 4 (PI below). catch_sleep (true): catch_perception and catch_behavior asleep
outside CATCH (they wake on /robot_mode; the catch steps start in CATCH). compose (true): Nav2's servers and omni_cpp's
heading_node in one component_container_isolated (nav2_container); all adds mcu_link, mode_manager and scan_prep (with
glue:=cpp); false: a process each. catch_impl (cpp): catch_perception and catch_behavior from omni_cpp (C++ ports of
omni_catch's, the same topics and parameters; test_cores.py checks the cores against the Python); py: omni_catch's
Python, the reference. side (both | sim | robot) and mcu_host / mcu_bind: the robot's computer in the loop: sim runs only the world (the
bridge, a step's scorer) with its MCU port open to the LAN, robot everything else, mcu_link on mcu_host:mcu_port
(ros/src/omni_bringup README; characterization/pi_camera/pi.sh robot). localizer (slam; localization, navigation, tour, goals): slam_toolbox's localization on
maps/<map>.posegraph | amcl: Nav2's AMCL (omni motion model) on maps/<map>.yaml, served by map_server (config/amcl.yaml).
"""

from __future__ import annotations

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from omni_bringup import maps_dir

PKG = "omni_bringup"
SIM_TIME = [True]            # clock:=wall sets it False (build()): the nodes run on the system clock
BOND_TIMEOUT = [4.0]         # Nav2's lifecycle bond, s (its default); a profile may set more (build(), PROFILE_EXTRAS)
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
            # the robot's computer in the loop (design.md 8, phase 1C step 2): side:=sim runs the bridge (the world and the
            # firmware in the loop) on the PC with its MCU port open to the LAN; side:=robot runs the robot's nodes on the
            # robot computer (a Pi), mcu_host:=<the PC> where mcu_link connects; both (default): one machine, as always
            DeclareLaunchArgument("side", default_value="both"),
            # clock: sim (the bridge publishes /clock at clock_period; the nodes run on sim time) | wall (no /clock: the
            # bridge stamps with the system time at rtf 1 and the nodes run on it, like the real robot; both computers
            # need synchronized clocks (chrony) when the sides run on two). clock_period: 0 = the step's default
            DeclareLaunchArgument("clock", default_value="sim"),
            # ghost (catch steps, on one computer): a real ball tracked by the Pi's camera head (pi.sh ghost, /ghost/track)
            # drives the simulated robot: the sim throws nothing and its own perception is off (omni_sim ghost_ball)
            DeclareLaunchArgument("ghost", default_value="false"),
            DeclareLaunchArgument("clock_period", default_value="0"),
            DeclareLaunchArgument("mcu_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("mcu_bind", default_value=""),
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
            # arm step (design.md 4.11): the depth camera, stereo (the head's pair) | d435 (the RealSense on the nose:
            # the tidy's looks and Nav2's depth layer)
            DeclareLaunchArgument("arm_camera", default_value="stereo"),
            DeclareLaunchArgument("robot", default_value="v7" if kind in ("tour", "auto_map", "goals") else "drawn"),
            DeclareLaunchArgument("catch", default_value="true" if kind == "robot" else "false"),
            # the Launcher's live world (simulation/v11 sil/sim_world.py) in the robot and arm steps: clutter off |
            # toys | all (default: toys for the arm robot, off for the other), the house's thrower on / off
            DeclareLaunchArgument("clutter", default_value="toys" if kind == "arm" else "off"),
            DeclareLaunchArgument("thrower", default_value="true"),
            # design.md 16: square steps and triangular bumps in the kitchen's east end (house.TERRAIN_COURSE)
            DeclareLaunchArgument("terrain", default_value="false"),
            DeclareLaunchArgument("lidar_mask", default_value="true"),
            # goals step: the console's Go To settings (0 / "" = nav2.yaml's own)
            DeclareLaunchArgument("speed", default_value="0.0"),
            DeclareLaunchArgument("heading", default_value=""),
            DeclareLaunchArgument("spin_rate", default_value="120.0"),
            DeclareLaunchArgument("route", default_value="house"),      # goals step: house | sprint (nav_goals.py)
            # the always-on glue nodes (mcu_link, mode_manager, scan_prep, heading_node): omni_cpp's C++ (design.md
            # 4.9: a fraction of the CPU) or their Python originals
            DeclareLaunchArgument("glue", default_value="cpp"),
            DeclareLaunchArgument("nav_profile", default_value="default"),   # | full (= default: a PC, an Orin) | lean | pi (a Pi 4: 0.6 m/s, see PI; the default on a Pi)
            # the catch nodes asleep outside CATCH (design.md 4.9); false: awake in every mode, as before Oct 28
            DeclareLaunchArgument("catch_sleep", default_value="true"),
            # Nav2 in one process (design.md 4.9): true (Nav2 + heading_node) | all (+ omni_cpp's glue: ~0.06 cores less,
            # but a Nav2 crash would take the MCU link and mode_manager with it) | false (a process per server)
            DeclareLaunchArgument("compose", default_value="true"),
            # the catch nodes (catch_perception, catch_behavior): omni_catch's Python | omni_cpp's C++ (design.md 4.9)
            DeclareLaunchArgument("catch_impl", default_value="cpp"),
            # localization on a saved map (localization, navigation, tour, goals; design.md 4.9): slam_toolbox's
            # localization mode on the pose graph | amcl (Nav2's particle filter on the occupancy grid)
            DeclareLaunchArgument("localizer", default_value="slam"),
            # scaling the robot down for a smaller computer (a Pi 4 in the loop, rpi.md): face (false: no face_node),
            # ekf_hz (the EKF's rate, 0 = ekf.yaml's 100 Hz), arm_hz (arm_node's cycle, 0 = its 50 Hz)
            DeclareLaunchArgument("face", default_value="true"),
            DeclareLaunchArgument("nav", default_value="true"),      # false: robot / arm steps without Nav2 and slam_toolbox
            DeclareLaunchArgument("ekf_hz", default_value="0"),
            DeclareLaunchArgument("arm_hz", default_value="0"),
            # the arm robot's arm (design.md 4.10): py = arm_node, one Python process; cpp = omni_cpp's arm_driver (the servo bus,
            # joints, TF, 50 Hz) with omni_arm's arm_brain (planning and the tidy, asleep between requests)
            DeclareLaunchArgument("arm_impl", default_value="py"),
            DeclareLaunchArgument("out", default_value={"tour": "ros_tour", "goals": "ros_nav_goals",
                                                        "catch_score": "ros_catch"}.get(kind, ""))]
    return LaunchDescription(args + [OpaqueFunction(function=lambda ctx: build(ctx, kind))])


def glue(a: dict, py_package: str) -> str:
    """The package a glue node comes from: omni_cpp (C++, the default) or its Python original's."""
    return "omni_cpp" if a["glue"] == "cpp" else py_package


def catch_pkg(a: dict) -> str:
    """The package the catch nodes come from: omni_catch (Python) or omni_cpp (catch_impl:=cpp, the same topics)."""
    if a["catch_impl"] not in ("py", "cpp"):
        raise RuntimeError(f"catch_impl: py or cpp, not {a['catch_impl']!r}")
    return "omni_cpp" if a["catch_impl"] == "cpp" else "omni_catch"


def catch_nodes(a: dict, on, kind: str):
    """v11 (design.md 30): the catch game. The bridge is the world and the MCU; the rest is the robot's own code."""
    sim_t = {"use_sim_time": a["clock"] != "wall"}
    dist = float(a["dist"])
    score = kind == "catch_score"
    bridge = Node(package=SIM, executable="catch_bridge", name="catch_bridge", output="screen",
                  parameters=[{"dist": dist, "seed": 9000 + int(a["seed"]), "floor": a["floor"],
                               "port": int(a["mcu_port"]), "mcu_bind": mcu_bind(a), "ghost": on("ghost"),
                               "clock_mode": a["clock"], "clock_period": clock_period(a, 0.001), "returns": on("returns"), "tosses": int(a["tosses"]), "rtf": float(a["rtf"]),
                               "viewer": on("viewer"), "pass_game": on("pass_game")}])
    # the robot starts at home facing the thrower: its odom frame has the thrower dist m ahead and the field's centre
    # CATCH_HOME_X ahead (what a person detector and the map would give it)
    robot = [
        Node(package=glue(a, "omni_mcu"), executable="mcu_link", name="mcu_link", output="screen",
             parameters=[{"port": f"tcp:{a['mcu_host']}:{int(a['mcu_port'])}", "publish_tf": True, **sim_t}]),
        Node(package=glue(a, "omni_catch"), executable="mode_manager", name="mode_manager", output="screen",
             parameters=[{"mode": "CATCH", "kid": on("kid"), **sim_t}]),
        *([] if on("ghost") else [
            Node(package=catch_pkg(a), executable="catch_perception", name="catch_perception", output="screen",
                 parameters=[{"source": "sparse", "arena_cx": CATCH_HOME_X, "arena_cy": 0.0, "arena_hx": CATCH_AREA[0] / 2,
                              "arena_hy": CATCH_AREA[1] / 2, **sim_t}])]),
        *([Node(package=SIM, executable="ghost_ball", name="ghost_ball", output="screen",
                parameters=[{"clock_mode": a["clock"], **sim_t}])] if on("ghost") else []),
        Node(package=catch_pkg(a), executable="catch_behavior", name="catch_behavior", output="screen",
             parameters=[{"thrower_x": dist, "thrower_y": 0.0, "arena_cx": CATCH_HOME_X, "arena_cy": 0.0,
                          "arena_hx": CATCH_AREA[0] / 2, "arena_hy": CATCH_AREA[1] / 2, "return_throws": on("returns"),
                          "kid": on("kid"), "return_to": "other" if on("pass_game") else "thrower", **sim_t}]),
    ]
    acts = [bridge] + robot
    sim_side = [bridge]
    if score:
        acts.append(Node(package=SIM, executable="catch_scorer", name="catch_scorer", output="screen",
                         parameters=[{"out": a["out"], "label": f"dist {dist}, seed {a['seed']}, {a['floor']}", **sim_t}]))
        acts.append(RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[EmitEvent(event=Shutdown())])))
        sim_side += acts[-2:]
    if on("foxglove"):
        acts.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
                         output="log", parameters=[{"port": 8765, "address": "0.0.0.0", **sim_t}]))
    if on("console"):
        acts += console_nodes(a, "/scan")
    return split_sides(a, acts, sim_side)


def mcu_bind(a: dict) -> str:
    """Where the bridge's MCU port listens: localhost, or every interface when the robot side is another computer."""
    return a["mcu_bind"] or ("0.0.0.0" if a["side"] == "sim" else "127.0.0.1")


def clock_period(a: dict, default: float) -> float:
    """The /clock period: the launch argument, or the step's default (1 ms with the firmware, 4 ms without)."""
    return float(a["clock_period"]) or default


def split_sides(a: dict, acts: list, sim_side: list) -> list:
    """side:=both (default) every node on this machine; sim: only the world (the bridge, and a step's scorer);
    robot: everything else, for the robot computer (mcu_host:=<the PC>). Design.md 8, phase 1C step 2."""
    side = a["side"]
    if side == "both":
        return acts
    if side not in ("sim", "robot"):
        raise RuntimeError(f"side: both, sim or robot, not {side!r}")
    keep = [x for x in acts if (x in sim_side) == (side == "sim")]
    return keep


def console_nodes(a: dict, scan: str, slam: tuple | None = None):
    """The operator console (omni_console): http://localhost:8080. rosbridge carries the page's topics and its one
    command service, rosapi lists nodes and topics for its ROS tab, the mission node switches modes (and runs
    slam_toolbox when given its parameter files: slam = (mapping, localization template)), web_video_server streams
    camera topics as MJPEG on 8081 (if installed). None of them runs on sim time: that would subscribe each to the
    sim's 1 kHz /clock (in Python: ~60% of a core for the mission node alone). They need no stamps of their own (TF
    lookups take the latest, Nav2 goals carry none); slam_toolbox, their child, does run on sim time."""
    mission = {"map": a["map"], "places_dir": str(maps_dir()), "scan_topic": scan,
               "log_source": "sim"}                          # session logs say the sim stood in for the robot
    if os.environ.get("OMNI_STATE_DIR"):                     # a parallel test run: its own settings, logs, slam log
        mission["state_dir"] = os.environ["OMNI_STATE_DIR"]
    cap =PROFILE_EXTRAS.get(a["nav_profile"], {}).get("speed_cap", 0.0)
    if a["nav_profile"].startswith("study"):          # the console's Go To / Patrol ask for the study's speed
        import re
        cap = float(re.match(r"study(\d+(?:\.\d+)?)", a["nav_profile"]).group(1))
    if cap:
        mission["speed_cap"] = cap                           # nav_profile pi: Go To / Patrol never ask Nav2 for more
    if slam:
        mission.update(slam_mapping_params=slam[0], slam_localization_params=slam[1], slam_sim_time=SIM_TIME[0],
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
    ]
    if a.get("face", "true").lower() in ("1", "true", "yes"):
        # the robot's face and voice (omni_face): in the sim its window opens on Windows, with sound
        nodes.append(Node(package="omni_face", executable="face_node", name="omni_face", output="screen",
                          parameters=[{"bind": "0.0.0.0", "start_display": "auto"}]))
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
                                                   "n_objects", "servo_port", "arm_camera", "robot", "catch", "lidar_mask", "clutter",
                                                   "thrower", "speed", "heading", "spin_rate", "route", "terrain",
                                                   "glue", "nav_profile", "catch_sleep", "compose", "catch_impl",
                                                   "localizer", "side", "mcu_host", "mcu_bind", "clock", "clock_period", "ghost",
                                                   "face", "ekf_hz", "arm_hz", "arm_impl", "nav")}
    on = lambda k: a[k].lower() in ("1", "true", "yes")          # noqa: E731
    a["nav_profile"] = resolved_profile(a["nav_profile"], a["side"])
    if a["clock"] not in ("sim", "wall"):
        raise RuntimeError(f"clock: sim or wall, not {a['clock']!r}")
    SIM_TIME[0] = a["clock"] != "wall"
    if kind in ("catch", "catch_score"):
        return catch_nodes(a, on, kind)
    sim_t = {"use_sim_time": a["clock"] != "wall"}
    acts = []
    tour = kind in ("tour", "auto_map")
    ekf = kind != "sim"
    if a["nav_profile"] not in PROFILES and not a["nav_profile"].startswith("study"):
        raise RuntimeError(f"nav_profile: one of {', '.join(PROFILES)}, not {a['nav_profile']!r}")
    # the EKF's rate: ekf_hz if given, else the profile's, else ekf.yaml's 100 Hz
    ekf_hz = float(a["ekf_hz"]) or PROFILE_EXTRAS.get(a["nav_profile"], {}).get("ekf_hz", 0.0)
    BOND_TIMEOUT[0] = PROFILE_EXTRAS.get(a["nav_profile"], {}).get("bond_timeout", 4.0)
    arm_hz = float(a["arm_hz"]) or PROFILE_EXTRAS.get(a["nav_profile"], {}).get("arm_hz", 0.0)     # 0: arm_node's 50 Hz
    arm = kind == "arm"
    robot = kind in ("robot", "arm")
    fast = on("fast")
    fw = (on("firmware") or arm) and not tour
    if a["arm_camera"] not in ("stereo", "d435"):
        raise RuntimeError(f"arm_camera: stereo or d435, not {a['arm_camera']!r}")
    d435 = arm and a["arm_camera"] == "d435"
    arm_kw = dict(payload="arm", objects=a["objects"], n_objects=int(a["n_objects"]),
                  servo_port=int(a["servo_port"]), arm_camera=a["arm_camera"]) if arm else {}
    acts.append(Node(package=SIM, executable="bridge", name="mujoco_bridge", output="screen",
                     additional_env={"MUJOCO_GL": "egl"} if arm else None,       # the arm's cameras render in WSL
                     parameters=[{"mode": "tour" if tour else "cmd_vel", "seed": int(a["seed"]),
                                  "publish_odom_tf": not ekf, "shadow": kind == "tour",
                                  "lidar_flatness_deg": float(a["flatness"]), "real_time_factor": float(a["rtf"]),
                                  "viewer": on("viewer"), "markers": on("foxglove") or on("rviz"),
                                  "fw_div": 2 if fast else 1, "hook_div": 4 if fast else 1,
                                  "lidar_cast_dt": 0.002 if fast else 0.001, "firmware": fw,
                                  "clock_period": clock_period(a, 0.001 if fw else 0.004), "port": int(a["mcu_port"]),
                                  "mcu_bind": mcu_bind(a), "clock_mode": a["clock"],
                                  "person": on("person"), "timing": a["timing"], "sync_gpio": a["sync_gpio"],
                                  "tags": on("tags"), "base": a["robot"], "lidar_mask": on("lidar_mask"),
                                  "catch": on("catch") and fw and not arm, "catch_dist": float(a["dist"]),
                                  "catch_returns": on("returns"), "live_world": robot, "clutter": a["clutter"],
                                  "thrower": on("thrower"), "terrain": on("terrain"),
                                  "control_port": 9873 if robot else 0, **arm_kw}]))
    sim_side = [acts[-1]]                                          # what side:=sim keeps (stack.split_sides)
    if arm:
        # design.md 11: the arm's own code, on the sim's servo bus (the bus board's USB port on the robot)
        arm_port = f"tcp:{a['mcu_host']}:{int(a['servo_port'])}"
        arm_cfg = "d435" if d435 else ""
        if a["arm_impl"] == "cpp":
            # the C++ driver owns the bus at 50 Hz; arm_hz only when asked (the pi profile's 20 Hz was the Python executor's)
            acts.append(Node(package="omni_cpp", executable="arm_driver", name="arm_driver", output="screen",
                             parameters=[{"port": arm_port, "config": arm_cfg, **sim_t,
                                          **({"rate": float(a["arm_hz"])} if float(a["arm_hz"]) > 0 else {})}]))
            acts.append(Node(package="omni_arm", executable="arm_brain", name="arm_node", output="screen",
                             parameters=[{"odom_topic": "/odom", "config": arm_cfg, **sim_t}]))
        elif a["arm_impl"] == "py":
            acts.append(Node(package="omni_arm", executable="arm_node", name="arm_node", output="screen",
                             parameters=[{"port": arm_port, "odom_topic": "/odom", "config": arm_cfg, **sim_t,
                                          **({"rate": arm_hz} if arm_hz > 0 else {})}]))
        else:
            raise RuntimeError(f"arm_impl: py or cpp, not {a['arm_impl']!r}")
    # compose (design.md 4.9): true puts Nav2 and heading_node into one process, all adds omni_cpp's glue nodes
    comps, nav_yaml = [], None
    glue_comp = a["compose"] == "all" and a["glue"] == "cpp"

    def glue_node(py_pkg: str, exe: str, plugin: str, params: dict):
        if glue_comp:
            comps.append(ComposableNode(package="omni_cpp", plugin=plugin, name=exe, parameters=[params]))
        else:
            acts.append(Node(package=glue(a, py_pkg), executable=exe, name=exe, output="screen", parameters=[params]))

    if fw:
        # the robot's own MCU link and mode manager: /odom, /imu and odom -> base_link from the firmware (the EKF
        # takes the TF over when it runs), /cmd_vel to the firmware
        sync = {"sync_source": "topic"} if a["sync_gpio"] != "none" else {}
        glue_node("omni_mcu", "mcu_link", "omni::McuLink",
                  {"port": f"tcp:{a['mcu_host']}:{int(a['mcu_port'])}", "publish_tf": not ekf, **sync, **sim_t})
        glue_node("omni_catch", "mode_manager", "omni::ModeManager", {"mode": "TELEOP", "kid": on("kid"), **sim_t})
        if on("catch") and not arm:
            # the catch game in the house: the robot's catch code on the bridge's camera frames; the field comes on
            # /catch/setup when the mode turns CATCH (the parameters are the open field's, until then)
            # asleep outside CATCH (catch_sleep): they wake on /robot_mode
            acts.append(Node(package=catch_pkg(a), executable="catch_perception", name="catch_perception",
                             output="screen", parameters=[{"source": "sparse", "sleep": on("catch_sleep"), **sim_t}]))
            acts.append(Node(package=catch_pkg(a), executable="catch_behavior", name="catch_behavior",
                             output="screen", parameters=[{"return_throws": on("returns"), "kid": on("kid"),
                                                           "sleep": on("catch_sleep"), **sim_t}]))
    if ekf:
        acts.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="log",
                         parameters=[cfg("ekf.yaml"), sim_t,
                                     *([{"frequency": ekf_hz}] if ekf_hz > 0 else [])]))
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch")
    prep = (on("floor_filter") or on("deskew")) and kind != "sim"
    scan = "/scan_filtered" if prep else "/scan"
    if prep:
        glue_node("omni_perception", "scan_prep", "omni::ScanPrep",
                  {"floor": on("floor_filter"), "deskew": on("deskew"), **sim_t})

    def slam_yaml(name, extra=""):
        src = open(cfg(name)).read().replace("    max_laser_range: 12.0", f"    max_laser_range: {float(a['slam_range'])}")
        src = profiled(src, name, a["nav_profile"])
        return tmp_yaml(src.replace("scan_topic: /scan", f"scan_topic: {scan}") + extra, name)

    if kind in ("mapping", "auto_map"):
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch, "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_yaml("slam_mapping.yaml"), "use_sim_time": str(SIM_TIME[0]).lower()}.items()))
    if a["localizer"] not in ("slam", "amcl"):
        raise RuntimeError(f"localizer: slam or amcl, not {a['localizer']!r}")
    nav_on = a["nav"].lower() in ("1", "true", "yes")             # nav:=false: no Nav2, no SLAM (a small computer's tiers)
    amcl = a["localizer"] == "amcl"
    if amcl and (kind in ("localization", "navigation", "tour", "goals") or (robot and nav_on)):
        # design.md 4.9: AMCL on the occupancy grid saved with the pose graph (the same map frame), map_server serving it;
        # in the robot and arm steps the console then runs no slam_toolbox (no Explore, no map switching)
        base = str(maps_dir() / a["map"])
        if not os.path.exists(base + ".yaml"):
            raise RuntimeError(f"no occupancy map at {base}.yaml: run `omni.sh map` + `save-map` or `auto-map`")
        src = open(cfg("amcl.yaml")).read().replace("    laser_max_range: 12.0", f"    laser_max_range: {float(a['slam_range'])}")
        src = profiled(src, "amcl.yaml", a["nav_profile"])
        amcl_yaml = tmp_yaml(src.replace("scan_topic: /scan", f"scan_topic: {scan}"), "amcl.yaml")
        st = {"use_sim_time": SIM_TIME[0]}                         # amcl.yaml says true; clock:=wall runs on the system's
        acts.append(Node(package="nav2_map_server", executable="map_server", name="map_server", output="log",
                         parameters=[amcl_yaml, {"yaml_filename": base + ".yaml"}, st]))
        acts.append(Node(package="nav2_amcl", executable="amcl", name="amcl", output="log", parameters=[amcl_yaml, st]))
        acts.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                         name="lifecycle_manager_localization", output="log",
                         parameters=[amcl_yaml, st, {"bond_timeout": BOND_TIMEOUT[0]}]))
    elif kind in ("localization", "navigation", "tour", "goals"):
        base = str(maps_dir() / a["map"])
        if not os.path.exists(base + ".posegraph"):
            raise RuntimeError(f"no serialized map at {base}.posegraph: run `omni.sh map` + `save-map` or `auto-map`")
        extra = f"\n    map_file_name: {base}\n    map_start_pose: [0.0, 0.0, 0.0]\n"
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch, "localization_launch.py")),
            launch_arguments={"slam_params_file": slam_yaml("slam_localization.yaml", extra),
                              "use_sim_time": str(SIM_TIME[0]).lower()}.items()))
    if kind in ("navigation", "goals", "robot", "arm") and nav_on:
        nav = open(cfg("nav2.yaml")).read().replace("topic: /scan\n", f"topic: {scan}\n")
        if d435:
            nav = with_depth_layer(nav)
        nav_yaml = tmp_yaml(profiled(nav, "nav2.yaml", a["nav_profile"]), "nav2.yaml")
        if a["compose"] in ("true", "all"):
            comps += nav2_components(nav_yaml, glue(a, PKG))
            if glue(a, PKG) != "omni_cpp":                  # the Python heading_node is no component
                acts.append(Node(package=PKG, executable="heading_node", name="heading_node", output="log",
                                 parameters=[{"heading": "forward"}]))
        else:
            acts += nav2_nodes(nav_yaml, glue(a, PKG))
    if comps:
        acts.append(composed(nav_yaml, comps))
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
                              if robot and nav_on and not amcl else None)
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
        if kind in ("tour", "goals"):                              # the scorers drive the test: they stay with the world
            sim_side += acts[-2:]
    return split_sides(a, acts, sim_side)


# nav_profile:=lean (design.md 4.9): Nav2 and slam_toolbox set for a smaller computer. MPPI half the samples over a
# 2.0 s horizon (4 m at 2 m/s) and a local costmap to match; the behavior tree ticked at 50 Hz; slam_toolbox corrects
# every 0.2 s / 0.2 m instead of 0.1. Each line must still be in the config: a changed config fails loudly here.
LEAN = {
    "nav2.yaml": [("    bt_loop_duration: 10\n", "    bt_loop_duration: 20\n"),
                  ("      time_steps: 56\n", "      time_steps: 40\n"),
                  ("      batch_size: 2000\n", "      batch_size: 1000\n"),
                  ("      prune_distance: 6.0 ", "      prune_distance: 4.5 "),
                  ("      width: 12 ", "      width: 9 "),
                  ("      height: 12\n", "      height: 9\n")],
    "slam_localization.yaml": [("    minimum_time_interval: 0.1 ", "    minimum_time_interval: 0.2 "),
                               ("    minimum_travel_distance: 0.1 ", "    minimum_travel_distance: 0.2 "),
                               ("    minimum_travel_heading: 0.1 ", "    minimum_travel_heading: 0.2 ")],
}


# nav_profile:=pi (Oct 31, design.md 4.10 / rpi.md; reshaped Nov 2 after runs P1-P4, C3-C7): navigation for a Pi 4
# class computer (4 slow cores), slow but working. It trades speed and reach for CPU:
#   speed    0.6 m/s top (MPPI, the velocity smoother, the console's Go To / Patrol via speed_cap): slam's corrections
#            per second, the horizon, the local costmap and the tolerated latency all shrink with it
#   control  MPPI 400 samples x 24 steps of 0.1 s at 10 Hz (the model step equals the controller period), the
#            smoother at 10 Hz, the tree at 20 Hz, replanning rate 5 Hz
#   maps     local costmap 5 m at 0.075 m (about 4,400 cells against 32,000 in the default), updated at 2 Hz, the lidar's
#            marking out to 2.8 m; the global costmap at 1 Hz as in full (at 0.5 Hz the planner timed out waiting for
#            it, and the controller for the 2 Hz local one: costmap_update_timeout 3 s / 1 s; C7's "Nav2 gave up")
#   slam     slam_toolbox corrects every 0.5 s / 0.25 m / 0.25 rad (~2 a second at 0.6 m/s, as P3's 0.4 m at 1 m/s),
#            loop closing kept: without it the robot slipped ~1 m at the same spots even on a PC (localization median
#            0.34 m, C3; 0.18 m with it, C7; full slam 0.08 m, C4, but 1.0-1.4 Pi cores while driving and lost, P4).
#            The 1 cm correlation grid kept: 2 cm killed slam_toolbox on activation on the Pi (P2)
#   guard    the collision monitor takes each scan as it is in base_link, not shifted to "now" through odom: on the
#            loaded Pi odom often came >0.2 s late, the lookup failed and the monitor stopped the robot ~100 times a
#            run ("invalid source", P5-P8). At 0.6 m/s a 0.1 s old scan is 6 cm off, inside its 1.2 s approach margin
# and the EKF at 30 Hz, arm_node at 20 Hz. Everything else (face, nav:=false) has its own flag. AMCL (localizer:=amcl) localized
# worse than either slam on the PC (median ~0.35 m, slips to 2 m; C6). The default profile ("full", for a PC or an
# Orin) is the one every scored result in design.md was run with.
PI = {
    "nav2.yaml": [("    bt_loop_duration: 10\n", "    bt_loop_duration: 50\n"),
                  ("    controller_frequency: 20.0\n", "    controller_frequency: 10.0\n"),
                  ("    costmap_update_timeout: 1.0\n", "    costmap_update_timeout: 3.0\n"),      # the planner's
                  ("    costmap_update_timeout: 0.30\n", "    costmap_update_timeout: 1.0\n"),     # the controller's
                  ("      time_steps: 56\n      model_dt: 0.05\n      batch_size: 2000\n",
                   "      time_steps: 24\n      model_dt: 0.1\n      batch_size: 400\n"),
                  ("      vx_std: 0.6 ", "      vx_std: 0.25 "),
                  ("      vy_std: 0.6\n", "      vy_std: 0.25\n"),
                  ("      vx_max: 2.0\n      vx_min: -2.0\n      vy_max: 2.0\n      wz_max: 1.9\n",
                   "      vx_max: 0.6\n      vx_min: -0.6\n      vy_max: 0.6\n      wz_max: 1.2\n"),
                  ("      prune_distance: 6.0 ", "      prune_distance: 3.0 "),
                  ("      update_frequency: 5.0\n      publish_frequency: 2.0\n",
                   "      update_frequency: 2.0\n      publish_frequency: 1.0\n"),
                  ("      width: 12 ", "      width: 5 "),
                  ("      height: 12\n      resolution: 0.05\n", "      height: 5\n      resolution: 0.075\n"),
                  ("          raytrace_max_range: 6.5 ", "          raytrace_max_range: 3.0 "),
                  ("          obstacle_max_range: 6.0 ", "          obstacle_max_range: 2.8 "),
                  ("    expected_planner_frequency: 20.0\n", "    expected_planner_frequency: 5.0\n"),
                  ("    smoothing_frequency: 20.0\n", "    smoothing_frequency: 10.0\n"),
                  ("max_velocity: [2.0, 2.0, 2.0]", "max_velocity: [0.6, 0.6, 1.2]"),
                  ("min_velocity: [-2.0, -2.0, -2.0]", "min_velocity: [-0.6, -0.6, -1.2]"),
                  ("    base_shift_correction: True\n", "    base_shift_correction: False\n")],
    "slam_localization.yaml": [("    minimum_time_interval: 0.1 ", "    minimum_time_interval: 0.5 "),
                               ("    minimum_travel_distance: 0.1 ", "    minimum_travel_distance: 0.25 "),
                               ("    minimum_travel_heading: 0.1 ", "    minimum_travel_heading: 0.25 ")],
}
PROFILES = {"default": {}, "full": {}, "lean": LEAN, "pi": PI}      # "full" = "default": a PC or an Orin
# what a profile sets outside the two YAMLs, unless the launch argument says otherwise: the EKF's rate (ekf_hz) and the
# console's navigation speed cap (m/s; Go To, Patrol, Explore, Follow ask for less or equal), and Nav2's lifecycle bond
# timeout (s): on a loaded Pi 4 bt_navigator took 25 s to activate and behavior_server's 4 s heartbeat lapsed meanwhile,
# so the lifecycle manager tore Nav2 down and never got it back (Nov 2, run P1); and arm_node's cycle (arm_hz): its
# rclpy executor at 50 Hz took ~0.67 of a Pi 4 core holding still, ~0.5 at 20 Hz, and the tidy still binned (Nov 2, P7)
PROFILE_EXTRAS = {"pi": {"ekf_hz": 30.0, "speed_cap": 0.6, "bond_timeout": 30.0, "arm_hz": 20.0}}


def on_a_pi(model: str = "/proc/device-tree/model") -> bool:
    try:
        return open(model).read().startswith("Raspberry Pi")
    except OSError:
        return False


def resolved_profile(profile: str, side: str, pi: bool = None) -> str:
    """nav_profile's default: pi when a Raspberry Pi runs the robot's nodes (baselined Nov 2, design.md 4.10), else
    full. A named profile is kept as it is; the sim's half (side:=sim) has no navigation and keeps the default."""
    if profile != "default" or side == "sim":
        return profile
    return "pi" if (on_a_pi() if pi is None else pi) else profile


def study_profile(profile: str) -> dict:
    """nav_profile:=study<v>[a<accel>] (the Nov 1 motor study, phase 6; never a robot default): Nav2 out of the way up to
    v m/s, so the firmware (OMNI_SIM_STUDY's build) and the physics set the robot's speed. MPPI's and the smoother's speed
    and acceleration limits (accel default 4 m/s^2), the yaw rate with them, the path MPPI scores and the local costmap
    long enough for its 2.8 s horizon at v. e.g. study3, study4a5."""
    import re
    mt = re.fullmatch(r"study(\d+(?:\.\d+)?)(?:a(\d+(?:\.\d+)?))?", profile)
    if not mt:
        raise RuntimeError(f"nav_profile {profile!r}: study<v>[a<accel>], e.g. study3 or study4a5")
    v, acc = float(mt.group(1)), float(mt.group(2) or 4.0)
    wz = round(1.9 * v / 2.0, 2)
    reach = round(max(6.0, 2.8 * v), 1)
    width = int(max(12, math.ceil(2 * reach)))
    return {"nav2.yaml": [
        ("      ax_max: 2.0\n      ax_min: -2.0\n      ay_max: 2.0\n      ay_min: -2.0\n",
         f"      ax_max: {acc}\n      ax_min: -{acc}\n      ay_max: {acc}\n      ay_min: -{acc}\n"),
        ("      vx_std: 0.6 ", f"      vx_std: {round(0.3 * v, 2)} "),
        ("      vy_std: 0.6\n", f"      vy_std: {round(0.3 * v, 2)}\n"),
        ("      vx_max: 2.0\n      vx_min: -2.0\n      vy_max: 2.0\n      wz_max: 1.9\n",
         f"      vx_max: {v}\n      vx_min: -{v}\n      vy_max: {v}\n      wz_max: {wz}\n"),
        ("      prune_distance: 6.0 ", f"      prune_distance: {reach} "),
        ("      width: 12 ", f"      width: {width} "),
        ("      height: 12\n", f"      height: {width}\n"),
        ("max_velocity: [2.0, 2.0, 2.0]", f"max_velocity: [{v}, {v}, {wz}]"),
        ("min_velocity: [-2.0, -2.0, -2.0]", f"min_velocity: [-{v}, -{v}, -{wz}]"),
        ("max_accel: [2.0, 2.0, 3.2]", f"max_accel: [{acc}, {acc}, 3.2]"),
        ("max_decel: [-2.0, -2.0, -3.2]", f"max_decel: [-{acc}, -{acc}, -3.2]")]}


def profiled(text: str, name: str, profile: str) -> str:
    if profile.startswith("study"):
        edits = study_profile(profile)
    elif profile not in PROFILES:
        raise RuntimeError(f"nav_profile: one of {', '.join(PROFILES)} or study<v>, not {profile!r}")
    else:
        edits = PROFILES[profile]
    for old, new in edits.get(name, []):
        if old not in text:
            raise RuntimeError(f"nav_profile {profile}: '{old.strip()}' is no longer in {name}")
        text = text.replace(old, new)
    return text


# design.md 4.11: the D435's depth in both costmaps, a layer of its own so the lidar's clearing (its plane is 13-15 cm
# up) doesn't wipe the low things only the camera sees. Two sources on the one cloud: marking above 4 cm (the floor's
# noise, plus the base's ~0.9 deg static pitch on its pods: ~1.6 cm at 1 m; odom -> base_link carries no pitch) out
# to 1.2 m, and clearing along every ray to the floor (min height below the floor) out to 2.0 m. The camera's own
# shadows and the arm reaching into view aren't filtered (est: fine while Nav2 waits in PICK; VERIFY on the robot)
DEPTH_LAYER = """      depth_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        footprint_clearing_enabled: True
        observation_sources: depth_mark depth_clear
        depth_mark:
          topic: /d435/depth/color/points
          data_type: "PointCloud2"
          marking: True
          clearing: False
          min_obstacle_height: 0.04
          max_obstacle_height: 0.60
          obstacle_max_range: 1.2
          obstacle_min_range: 0.0
          expected_update_rate: 0.0
        depth_clear:
          topic: /d435/depth/color/points
          data_type: "PointCloud2"
          marking: False
          clearing: True
          min_obstacle_height: -0.20
          max_obstacle_height: 0.60
          raytrace_max_range: 2.0
          raytrace_min_range: 0.0
          expected_update_rate: 0.0
"""
DEPTH_PLUGINS = [('      plugins: ["obstacle_layer", "inflation_layer"]\n',
                  '      plugins: ["obstacle_layer", "depth_layer", "inflation_layer"]\n'),
                 ('      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]\n',
                  '      plugins: ["static_layer", "obstacle_layer", "depth_layer", "inflation_layer"]\n')]


def with_depth_layer(nav: str) -> str:
    """nav2.yaml with DEPTH_LAYER in the local and the global costmap (each plugins line must be there once)."""
    for old, new in DEPTH_PLUGINS:
        if nav.count(old) != 1:
            raise RuntimeError(f"depth layer: '{old.strip()}' is not in nav2.yaml once")
        nav = nav.replace(old, new + DEPTH_LAYER)
    return nav


def tmp_yaml(text: str, name: str) -> str:
    f = tempfile.NamedTemporaryFile("w", suffix="_" + name, delete=False)
    f.write(text)
    f.close()
    return f.name


NAV2_LIFECYCLE = ["controller_server", "smoother_server", "planner_server", "behavior_server", "velocity_smoother",
                  "collision_monitor", "bt_navigator"]


def composed(params_file: str | None, comps: list) -> ComposableNodeContainer:
    """compose (design.md 4.9): one process for Nav2 (and heading_node, and with compose:=all omni_cpp's glue), as
    nav2_bringup's composed launch has it: component_container_isolated, each node on its own executor thread. The
    container's own parameters (nav2.yaml and use_sim_time) apply to every node in the process, the costmaps and
    Nav2's helper nodes included (as separate processes had them); a component's own parameters win over them, so a
    wall-time node says use_sim_time false."""
    p = [params_file, {"use_sim_time": SIM_TIME[0], "autostart": True}] if params_file else [{"use_sim_time": SIM_TIME[0]}]
    # it ignores SIGINT for a while at shutdown (launch waited 15 s before SIGKILL): escalate sooner
    return ComposableNodeContainer(name="nav2_container", namespace="", package="rclcpp_components",
                                   executable="component_container_isolated", output="log", parameters=p,
                                   composable_node_descriptions=comps, sigterm_timeout="3", sigkill_timeout="2")


def nav2_components(params_file: str, heading_pkg: str = PKG) -> list:
    """nav2_nodes as components (compose:=true); heading_node is omni_cpp's (the Python one is no component)."""
    p = [params_file, {"use_sim_time": SIM_TIME[0], "autostart": True}]
    nav = "cmd_vel_nav"
    C = ComposableNode
    comps = [
        C(package="nav2_controller", plugin="nav2_controller::ControllerServer", name="controller_server",
          parameters=p, remappings=[("cmd_vel", nav)]),
        C(package="nav2_smoother", plugin="nav2_smoother::SmootherServer", name="smoother_server", parameters=p),
        C(package="nav2_planner", plugin="nav2_planner::PlannerServer", name="planner_server", parameters=p),
        C(package="nav2_behaviors", plugin="behavior_server::BehaviorServer", name="behavior_server", parameters=p,
          remappings=[("cmd_vel", nav)]),
        C(package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator", name="bt_navigator", parameters=p),
        C(package="nav2_velocity_smoother", plugin="nav2_velocity_smoother::VelocitySmoother",
          name="velocity_smoother", parameters=p, remappings=[("cmd_vel", nav), ("cmd_vel_smoothed", "cmd_vel_shape")]),
        C(package="nav2_collision_monitor", plugin="nav2_collision_monitor::CollisionMonitor", name="collision_monitor",
          parameters=p),
        C(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
          name="lifecycle_manager_navigation",
          parameters=[{"use_sim_time": SIM_TIME[0], "autostart": True, "node_names": NAV2_LIFECYCLE,
                       "bond_timeout": BOND_TIMEOUT[0]}]),
    ]
    if heading_pkg == "omni_cpp":
        comps.append(C(package="omni_cpp", plugin="omni::HeadingNode", name="heading_node",
                       parameters=[{"heading": "forward", "use_sim_time": False}]))
    return comps


def nav2_nodes(params_file: str, heading_pkg: str = PKG):
    p = [params_file, {"use_sim_time": SIM_TIME[0], "autostart": True}]
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
        Node(package=heading_pkg, executable="heading_node", name="heading_node", output="log",
             parameters=[{"heading": "forward"}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="log", parameters=p),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"use_sim_time": SIM_TIME[0], "autostart": True, "node_names": NAV2_LIFECYCLE,
                                           "bond_timeout": BOND_TIMEOUT[0]}]),
    ]
    return nodes
