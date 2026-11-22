cmake_minimum_required(VERSION 3.16)
project(lo_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(STRIDE_WERROR "Treat warnings as errors" ON)
set(STRIDE_SANITIZER "" CACHE STRING "address | undefined | thread")

find_package(ament_cmake REQUIRED)
find_package(lo_core REQUIRED)          # + stride_target_flags()
find_package(lo_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# git SHA for the black box / diagnostics
execute_process(COMMAND git rev-parse --short=12 HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE STRIDE_GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT STRIDE_GIT_SHA)
  set(STRIDE_GIT_SHA "unknown")
endif()

# Conversions, parameters, URDF and black box: shared with lo_tools.
add_library(lo_ros_common SHARED
  src/black_box.cpp
  src/conversions.cpp
  src/params.cpp
  src/urdf_kinematics.cpp
)
target_include_directories(lo_ros_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lo_ros_common PUBLIC lo_core::lo_core
  ${lo_msgs_TARGETS} ${livox_ros_driver2_TARGETS}
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS}
  rosbag2_cpp::rosbag2_cpp urdf::urdf yaml-cpp)
stride_target_flags(lo_ros_common)

add_library(lo_node SHARED src/lo_node.cpp)
target_compile_definitions(lo_node PRIVATE STRIDE_GIT_SHA="${STRIDE_GIT_SHA}")
target_link_libraries(lo_node PUBLIC lo_ros_common
  rclcpp_lifecycle::rclcpp_lifecycle rclcpp_components::component
  diagnostic_updater::diagnostic_updater ${std_msgs_TARGETS} tf2_ros::tf2_ros)
stride_target_flags(lo_node)
rclcpp_components_register_nodes(lo_node "lo_ros::LoNode")

add_executable(lo_node_main src/lo_node_main.cpp)
set_target_properties(lo_node_main PROPERTIES OUTPUT_NAME lo_node)
target_link_libraries(lo_node_main PRIVATE lo_node)
stride_target_flags(lo_node_main)

add_executable(lo_supervisor src/supervisor.cpp)
target_link_libraries(lo_supervisor PRIVATE rclcpp::rclcpp ${lifecycle_msgs_TARGETS})
stride_target_flags(lo_supervisor)

install(TARGETS lo_ros_common lo_node EXPORT export_lo_ros
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS lo_node_main lo_supervisor RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/orin_setup.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions lo_ros_common)
  ament_add_gtest(test_black_box test/test_black_box.cpp TIMEOUT 60)
  target_link_libraries(test_black_box lo_ros_common)
  ament_add_gtest(test_params test/test_params.cpp)
  target_link_libraries(test_params lo_ros_common)
  target_compile_definitions(test_params PRIVATE
    LO_ROS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_export_targets(export_lo_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(lo_core lo_msgs livox_ros_driver2 rclcpp sensor_msgs nav_msgs
  geometry_msgs rosbag2_cpp urdf yaml-cpp)
ament_package()
