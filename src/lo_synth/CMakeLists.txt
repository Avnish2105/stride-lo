cmake_minimum_required(VERSION 3.16)
project(lo_synth VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(BUILD_TESTING "Build the tests" ON)
option(STRIDE_WERROR "Treat warnings as errors" ON)
set(STRIDE_SANITIZER "" CACHE STRING "address | undefined | thread")

find_package(lo_core REQUIRED)   # also provides stride_target_flags()

add_library(lo_synth SHARED
  src/evaluation.cpp
  src/robot.cpp
  src/runner.cpp
  src/scene.cpp
  src/simulator.cpp
  src/trajectory.cpp
)
target_include_directories(lo_synth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lo_synth PUBLIC lo_core::lo_core)
stride_target_flags(lo_synth)

add_executable(lo_synth_run tools/lo_synth_run.cpp)
target_link_libraries(lo_synth_run PRIVATE lo_synth)
stride_target_flags(lo_synth_run)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS lo_synth EXPORT lo_synthTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS lo_synth_run RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
set(LO_SYNTH_CMAKE_DIR share/lo_synth/cmake)
install(EXPORT lo_synthTargets NAMESPACE lo_synth:: DESTINATION ${LO_SYNTH_CMAKE_DIR})
configure_package_config_file(cmake/lo_synthConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/lo_synthConfig.cmake INSTALL_DESTINATION ${LO_SYNTH_CMAKE_DIR})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/lo_synthConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lo_synthConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/lo_synthConfigVersion.cmake DESTINATION ${LO_SYNTH_CMAKE_DIR})
install(FILES package.xml DESTINATION share/lo_synth)

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  file(GLOB LO_SYNTH_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
  foreach(test_src ${LO_SYNTH_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE lo_synth GTest::gtest GTest::gtest_main)
    stride_target_flags(${test_name})
    gtest_discover_tests(${test_name} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 120 PROPERTIES TIMEOUT 3600)
  endforeach()
endif()
