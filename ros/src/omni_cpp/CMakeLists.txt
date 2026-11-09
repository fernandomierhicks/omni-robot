cmake_minimum_required(VERSION 3.8)
project(omni_cpp)

# The robot's always-on glue nodes in C++ (design.md 4.9): mcu_link, mode_manager, scan_prep, heading_node; arm_driver
# (the arm's servo bus and joints, design.md 4.10, reading omni_arm's config/robot_arm*.json); and the
# catch path, catch_perception and catch_behavior, reading omni_catch's config/robot_v11.json. The MCU
# link's frames come from the firmware's own source (firmware/lib/omni_fw/protocol.*): beside ros/ in the repo, or
# mirrored into the workspace by omni.sh ($OMNI_FIRMWARE_DIR).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(omni_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)

set(FW_CANDIDATES "$ENV{OMNI_FIRMWARE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../../firmware"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware")
foreach(d ${FW_CANDIDATES})
  if(d AND EXISTS "${d}/lib/omni_fw/protocol.cpp" AND NOT FW_DIR)
    set(FW_DIR "${d}/lib/omni_fw")
  endif()
endforeach()
if(NOT FW_DIR)
  message(FATAL_ERROR "omni_cpp: no firmware/lib/omni_fw (set OMNI_FIRMWARE_DIR)")
endif()
message(STATUS "omni_cpp: the MCU protocol from ${FW_DIR}")

# the ROS-free cores (test/test_cores.py builds the same files with its checker)
add_library(omni_cpp_cores STATIC src/clock_sync.cpp src/host_link.cpp src/mode_mux.cpp src/heading.cpp
            src/scan.cpp src/gpio_edges.cpp ${FW_DIR}/protocol.cpp
            src/json_value.cpp src/flight.cpp src/robot_config.cpp src/tracker.cpp src/detector.cpp src/perception.cpp
            src/intercept.cpp src/game.cpp src/feetech.cpp src/arm_kin.cpp src/arm_motion.cpp src/arm_driver_core.cpp)
target_include_directories(omni_cpp_cores PUBLIC include ${FW_DIR})
set_target_properties(omni_cpp_cores PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(DEPS rclcpp rclcpp_components rcl_interfaces builtin_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs tf2
    tf2_ros tf2_msgs omni_interfaces visualization_msgs ament_index_cpp)

# each node: a component library plus its own executable (the name the Python node had)
function(omni_node lib src plugin exe)
  add_library(${lib} SHARED ${src})
  target_link_libraries(${lib} omni_cpp_cores)
  ament_target_dependencies(${lib} ${DEPS})
  rclcpp_components_register_node(${lib} PLUGIN ${plugin} EXECUTABLE ${exe})
  install(TARGETS ${lib} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endfunction()

omni_node(mcu_link_component src/mcu_link_node.cpp "omni::McuLink" mcu_link)
omni_node(mode_manager_component src/mode_manager_node.cpp "omni::ModeManager" mode_manager)
omni_node(scan_prep_component src/scan_prep_node.cpp "omni::ScanPrep" scan_prep)
omni_node(heading_component src/heading_node.cpp "omni::HeadingNode" heading_node)
omni_node(catch_perception_component src/catch_perception_node.cpp "omni::CatchPerception" catch_perception)
omni_node(catch_behavior_component src/catch_behavior_node.cpp "omni::CatchBehaviorNode" catch_behavior)
omni_node(arm_driver_component src/arm_driver_node.cpp "omni::ArmDriver" arm_driver)

ament_package()
