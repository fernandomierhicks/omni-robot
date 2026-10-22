cmake_minimum_required(VERSION 3.8)
project(omni_interfaces)

# omni-robot messages (design.md 30): the MCU link, the catch path, the sim's camera frames, robot modes
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/McuState.msg"
  "msg/McuCommand.msg"
  "msg/BallTrack.msg"
  "msg/SparseImage.msg"
  "msg/CatchStatus.msg"
  "srv/SetMode.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
