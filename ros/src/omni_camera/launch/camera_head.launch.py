"""The camera head (design.md 4.9): libcamera_capture + omni_cpp's catch_perception in one container with
intra-process comms, so frames reach the detector uncopied and only /catch/track, /catch/ball and /catch/perception
leave the board.

  ros2 launch omni_camera camera_head.launch.py [robot_config:=...] [sleep:=true] [width:=768 height:=432 fps:=120]
                                                [exposure_us:=0] [perception:=true] [track_topic:=/catch/track]

robot_config: the cameras catch_perception projects through (default: omni_catch's robot_v11.json, the robot's
stereo pair); the bench's one-camera config is characterization/pi_camera/bench_config.py's. sleep: perception
sleeps outside CATCH (/robot_mode), as on the robot. perception:=false runs the capture alone. track_topic: where
the track is published (/ghost/track for the ghost-ball test, where a simulated robot reads it: omni_sim ghost_ball).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _nodes(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in
         ("robot_config", "sleep", "width", "height", "fps", "exposure_us", "perception", "track_topic")}
    ipc = [{"use_intra_process_comms": True}]
    nodes = [ComposableNode(
        package="omni_camera", plugin="omni::LibcameraCapture", name="libcamera_capture_left",
        parameters=[{"side": "left", "camera": 0, "width": int(a["width"]), "height": int(a["height"]),
                     "fps": float(a["fps"]), "exposure_us": int(a["exposure_us"])}],
        extra_arguments=ipc)]
    if a["perception"].lower() == "true":
        p = {"source": "image", "sleep": a["sleep"].lower() == "true"}
        if a["robot_config"]:
            p["robot_config"] = a["robot_config"]
        remap = [("/catch/track", a["track_topic"])] if a["track_topic"] != "/catch/track" else []
        nodes.append(ComposableNode(package="omni_cpp", plugin="omni::CatchPerception", name="catch_perception",
                                    parameters=[p], remappings=remap, extra_arguments=ipc))
    return [ComposableNodeContainer(name="camera_head", namespace="", package="rclcpp_components",
                                    executable="component_container", composable_node_descriptions=nodes,
                                    output="screen")]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in
            (("robot_config", ""), ("sleep", "true"), ("width", "768"), ("height", "432"), ("fps", "120"),
             ("exposure_us", "0"), ("perception", "true"), ("track_topic", "/catch/track"))]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
