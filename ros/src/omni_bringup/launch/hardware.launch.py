"""The robot's software on its real hardware, as far as it exists (the robot computer's "robot" step: omni_console's
config/robot_steps.json, started from the console's Robot > System page). No sim: the MCU on its USB port.

    mcu_link       the MCU (the Teensy, or the UNO R4 standing in for it on the bench: firmware/bench) on its USB port;
                   /odom, /imu, odom -> base_link, the MCU's state
    mode_manager   DISARMED until the console arms it; TELEOP from /cmd_vel/joy > /cmd_vel/manual > /cmd_vel
    the console    rosbridge (the page's topics), rosapi (its ROS tab), omni_console's mission_node (modes, verbs,
                   settings, the session log). The page itself is the supervisor's (port 8080), never console_web here.

    arm:=true      the SO-101 (the console's Arm page switches it on): omni_cpp's arm_driver on the servo bus's USB
                   adapter and omni_arm's arm_brain (arm_node), plus mode_manager's arm-out cap. Only with the arm's
                   LeRobot calibration (arm_calibration, default <state dir>/so101_calibration.json): the sim's nominal
                   one would aim every pose off on a real arm. Without it, or without the adapter, the step runs
                   without the arm and its log says why. The arm nodes don't end the step if they die: the base and
                   the console stay up.

    lidar:=true    the RPLIDAR A2M8 (Nov 16): omni_cpp's rplidar_node on its CP210x adapter, /scan stamped from the
                   device's sample clock, /lidar/status. Off by default until it has run on the bench (design.md 4.6):
                   it holds the port exclusively, so the console's Test > Lidar (camera_snap) can't open it meanwhile.
                   Its frame "laser" needs base_link -> laser from the robot's frames (not yet in this file).

Not yet on the hardware, so not here: the EKF, scan prep, slam_toolbox, Nav2, cameras, face. Each joins this file as
its driver is brought up (design.md 8).

    ros2 launch omni_bringup hardware.launch.py [mcu:=auto|/dev/...] [arm:=false] [arm_port:=auto|/dev/...]
                                                [arm_calibration:=<json>] [kid:=false] [map:=house]
                                                [lidar:=false] [lidar_port:=auto|/dev/...]

mcu (auto): the MCU's serial device; auto takes a Teensy by id, else an Arduino (the bench's UNO R4), else /dev/ttyACM0.
arm_port (auto): the SO-101 bus adapter (a CH340/CH343 by id: SERVO_PATTERNS), never the MCU's device.
lidar_port (auto): the lidar's adapter (rplidar_node's own search: a CP210x by id, else /dev/ttyUSB0).
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

from omni_bringup import maps_dir
from omni_bringup.stack import robot_params

MCU_PATTERNS = ("/dev/serial/by-id/*Teensy*", "/dev/serial/by-id/*Arduino*", "/dev/ttyACM0")
# the SO-101's bus adapter (Waveshare/Feetech boards: WCH CH340 / CH343, vendor 1a86); VERIFY on the robot's own board
SERVO_PATTERNS = ("/dev/serial/by-id/*1a86*", "/dev/serial/by-id/*USB_Single_Serial*", "/dev/serial/by-id/*CH34*")


def find_mcu() -> str | None:
    for pat in MCU_PATTERNS:
        hit = sorted(glob.glob(pat))
        if hit:
            return hit[0]
    return None


def find_servo_bus(mcu: str | None) -> str | None:
    skip = os.path.realpath(mcu) if mcu else None
    for pat in SERVO_PATTERNS:
        for hit in sorted(glob.glob(pat)):
            if os.path.realpath(hit) != skip:
                return hit
    return None


def arm_nodes(a: dict, mcu: str | None) -> list:
    """The SO-101 on the robot (arm:=true): its driver and arm_node, or why not (LogInfo)."""
    state = os.path.expanduser(os.environ.get("OMNI_STATE_DIR", "~/.omni"))
    cal = os.path.expanduser(a["arm_calibration"]) if a["arm_calibration"] else os.path.join(state, "so101_calibration.json")
    port = find_servo_bus(mcu) if a["arm_port"] == "auto" else a["arm_port"]
    if not os.path.isfile(cal):
        return [LogInfo(msg=f"hardware: arm:=true but no arm calibration at {cal} (LeRobot's calibration json for this "
                            "SO-101): the arm stays off")]
    if not port:
        return [LogInfo(msg="hardware: arm:=true but no servo bus adapter found (" + ", ".join(SERVO_PATTERNS)
                            + "): plug it in (and its 12 V) and start the robot again, or set arm_port")]
    return [
        LogInfo(msg=f"hardware: the arm on {port}, calibration {cal}"),
        Node(package="omni_cpp", executable="arm_driver", name="arm_driver", output="screen",
             parameters=[robot_params(), {"port": f"serial:{port}", "calibration": cal, "config": "d435"}]),
        Node(package="omni_arm", executable="arm_brain", name="arm_node", output="screen",
             parameters=[robot_params(), {"odom_topic": "/odom", "config": "d435"}]),
    ]


def ends_step(name: str) -> Shutdown:
    """A node that dies ends the whole launch, so the supervisor starts the step again (keep_running: at most 3 times
    in 5 minutes, then it stays stopped and the page says so). Otherwise ros2 launch carries on without it: on Nov 12
    a killed mcu_link left the step "running" with no MCU link, and nothing restarted it."""
    return Shutdown(reason=f"{name} died: the robot step ends (the console starts it again)")


def build(ctx):
    a = {k: ctx.launch_configurations[k] for k in ("mcu", "arm", "arm_port", "arm_calibration", "kid", "map", "lidar",
                                                   "lidar_port")}
    on = lambda k: a[k].lower() in ("1", "true", "yes")          # noqa: E731
    dev = find_mcu() if a["mcu"] == "auto" else a["mcu"]
    acts = []
    if dev:
        acts += [
            LogInfo(msg=f"hardware: the MCU on {dev}"),
            Node(package="omni_cpp", executable="mcu_link", name="mcu_link", output="screen",
                 on_exit=ends_step("mcu_link"),
                 parameters=[robot_params(), {"port": f"serial:{dev}"}]),
            Node(package="omni_cpp", executable="mode_manager", name="mode_manager", output="screen",
                 on_exit=ends_step("mode_manager"),
                 parameters=[robot_params(), {"arm": on("arm"), "kid": on("kid")}]),
        ]
    else:
        acts.append(LogInfo(msg="hardware: no MCU found (no Teensy or Arduino in /dev/serial/by-id, no /dev/ttyACM0): "
                                "plug it in and start the robot again. The console runs without it."))
    mission = {"map": a["map"], "places_dir": str(maps_dir()), "log_source": "robot", "cmd_vel_topic": "/cmd_vel/manual"}
    if os.environ.get("OMNI_STATE_DIR"):
        mission["state_dir"] = os.environ["OMNI_STATE_DIR"]
    acts += [
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket", output="log",
             on_exit=ends_step("rosbridge"),
             parameters=[{"port": int(os.environ.get("OMNI_ROSBRIDGE_PORT", 9090)), "address": ""}]),
        Node(package="rosapi", executable="rosapi_node", name="rosapi", output="log", on_exit=ends_step("rosapi")),
        Node(package="omni_console", executable="mission_node", name="omni_console", output="screen",
             on_exit=ends_step("mission_node"),
             parameters=[mission]),
    ]
    if on("arm"):
        acts += arm_nodes(a, dev)
    if on("lidar"):                                               # it retries on its own: it doesn't end the step
        acts.append(Node(package="omni_cpp", executable="rplidar_node", name="rplidar", output="screen",
                         parameters=[robot_params(), {"port": a["lidar_port"]}]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mcu", default_value="auto", description="the MCU's serial device; auto: a Teensy by "
                              "id, else an Arduino (the bench's UNO R4), else /dev/ttyACM0"),
        DeclareLaunchArgument("arm", default_value="false", description="the SO-101: its driver, arm_node and "
                              "mode_manager's arm-out cap (needs arm_calibration)"),
        DeclareLaunchArgument("arm_port", default_value="auto", description="the servo bus adapter; auto: a CH340 / "
                              "CH343 by id"),
        DeclareLaunchArgument("arm_calibration", default_value="", description="the SO-101's LeRobot calibration "
                              "json (default <state dir>/so101_calibration.json)"),
        DeclareLaunchArgument("kid", default_value="false", description="the kid speed profile from the start"),
        DeclareLaunchArgument("map", default_value="house", description="the console's map (maps/<map>)"),
        DeclareLaunchArgument("lidar", default_value="false", description="the A2M8: omni_cpp's rplidar_node (/scan "
                              "from the sample clock)"),
        DeclareLaunchArgument("lidar_port", default_value="auto", description="the lidar's serial device; auto: a "
                              "CP210x by id, else /dev/ttyUSB0"),
        OpaqueFunction(function=build),
    ])
