# Out-of-tree consumer of stride-lo. Builds against an installed stride-lo (underlay) only:
#   source <stride-lo>/install/setup.bash && colcon build   (in your own workspace)
cmake_minimum_required(VERSION 3.16)
project(stride_minimal_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lo_msgs REQUIRED)
find_package(lo_gtsam_adapter REQUIRED)   # brings GTSAM and lo_msgs along

add_executable(minimal_backend src/minimal_backend.cpp)
target_link_libraries(minimal_backend lo_gtsam_adapter::lo_gtsam_adapter rclcpp::rclcpp
  ${lo_msgs_TARGETS})

install(TARGETS minimal_backend DESTINATION lib/${PROJECT_NAME})
ament_package()
