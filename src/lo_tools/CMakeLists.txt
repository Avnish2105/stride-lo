cmake_minimum_required(VERSION 3.16)
project(lo_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(STRIDE_WERROR "Treat warnings as errors" ON)
set(STRIDE_SANITIZER "" CACHE STRING "address | undefined | thread")

find_package(ament_cmake REQUIRED)
find_package(lo_core REQUIRED)
find_package(lo_ros REQUIRED)
find_package(lo_msgs REQUIRED)
find_package(lo_synth REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(lo_replay src/lo_replay.cpp)
target_link_libraries(lo_replay PRIVATE lo_ros::lo_ros_common rosbag2_cpp::rosbag2_cpp tf2::tf2
  ${tf2_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ament_index_cpp::ament_index_cpp)
stride_target_flags(lo_replay)

add_executable(lo_synth_bag src/lo_synth_bag.cpp)
target_link_libraries(lo_synth_bag PRIVATE lo_synth::lo_synth lo_ros::lo_ros_common
  rosbag2_cpp::rosbag2_cpp ${tf2_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
stride_target_flags(lo_synth_bag)

install(TARGETS lo_replay lo_synth_bag RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/bag_suite.py
  scripts/evaluate.py
  scripts/fault_inject.py
  scripts/live_check.sh
  scripts/run_fault_suite.sh
  scripts/soak.py
  scripts/time_offset.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
