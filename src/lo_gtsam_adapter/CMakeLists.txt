cmake_minimum_required(VERSION 3.16)
project(lo_gtsam_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(STRIDE_WERROR "Treat warnings as errors" ON)
set(STRIDE_SANITIZER "" CACHE STRING "address | undefined | thread")

find_package(ament_cmake REQUIRED)
find_package(lo_core REQUIRED)          # stride_target_flags(); lo_core itself only in the tests
find_package(lo_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# GTSAM stays out of lo_core (CLAUDE.md rule 4): this package is the only place it is used.
add_library(lo_gtsam_adapter SHARED src/adapter.cpp)
target_include_directories(lo_gtsam_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lo_gtsam_adapter PUBLIC gtsam ${lo_msgs_TARGETS})
stride_target_flags(lo_gtsam_adapter)

add_executable(lo_graph_demo src/lo_graph_demo.cpp)
target_link_libraries(lo_graph_demo PRIVATE lo_gtsam_adapter rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
stride_target_flags(lo_graph_demo)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS lo_gtsam_adapter EXPORT export_lo_gtsam_adapter
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS lo_graph_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adapter test/test_adapter.cpp TIMEOUT 120)
  target_link_libraries(test_adapter lo_gtsam_adapter lo_core::lo_core)
  stride_target_flags(test_adapter)
endif()

ament_export_targets(export_lo_gtsam_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(lo_msgs GTSAM)
ament_package()
