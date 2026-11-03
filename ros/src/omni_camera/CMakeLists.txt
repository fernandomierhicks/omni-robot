cmake_minimum_required(VERSION 3.8)
project(omni_camera)

# The camera head's capture (design.md 4.9): libcamera_capture, one CSI camera as /cam/<side>/image_raw. It needs
# Raspberry Pi's libcamera fork (>= 0.5, built from source on the camera head:
# characterization/pi_camera/setup/build_libcamera.sh); elsewhere (WSL, the main computer: Ubuntu's 0.2, or none)
# the node is skipped and only the launch file installs, so `colcon build` stays green everywhere.
set(CMAKE_CXX_STANDARD 20)    # libcamera 0.7's controls are std::span
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA libcamera>=0.5)

if(LIBCAMERA_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  message(STATUS "omni_camera: libcamera ${LIBCAMERA_VERSION}")
  add_library(libcamera_capture_component SHARED src/libcamera_capture.cpp)
  target_include_directories(libcamera_capture_component PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
  target_link_directories(libcamera_capture_component PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
  target_link_libraries(libcamera_capture_component ${LIBCAMERA_LIBRARIES})
  ament_target_dependencies(libcamera_capture_component rclcpp rclcpp_components sensor_msgs std_msgs)
  rclcpp_components_register_node(libcamera_capture_component PLUGIN "omni::LibcameraCapture"
                                  EXECUTABLE libcamera_capture)
  install(TARGETS libcamera_capture_component ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
else()
  message(STATUS "omni_camera: no libcamera >= 0.5 (Raspberry Pi's fork): libcamera_capture skipped")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
