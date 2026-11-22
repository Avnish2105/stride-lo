cmake_minimum_required(VERSION 3.16)
project(lo_core VERSION 0.1.0 LANGUAGES CXX)

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
include(cmake/stride_flags.cmake)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

add_library(lo_core SHARED
  src/bias_estimator.cpp
  src/config.cpp
  src/degeneracy.cpp
  src/deskew.cpp
  src/downsample.cpp
  src/emitter.cpp
  src/imu_buffer.cpp
  src/kinematics.cpp
  src/local_map.cpp
  src/pipeline.cpp
  src/point_covariance.cpp
  src/registration.cpp
  src/seed.cpp
  src/self_filter.cpp
  src/validation.cpp
)
target_include_directories(lo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lo_core PUBLIC Eigen3::Eigen Sophus::Sophus small_gicp::small_gicp
  OpenMP::OpenMP_CXX)
stride_target_flags(lo_core)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS lo_core EXPORT lo_coreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
set(LO_CORE_CMAKE_DIR share/lo_core/cmake)
install(EXPORT lo_coreTargets NAMESPACE lo_core:: DESTINATION ${LO_CORE_CMAKE_DIR})
configure_package_config_file(cmake/lo_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/lo_coreConfig.cmake INSTALL_DESTINATION ${LO_CORE_CMAKE_DIR})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/lo_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/lo_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/lo_coreConfigVersion.cmake
  cmake/stride_flags.cmake
  DESTINATION ${LO_CORE_CMAKE_DIR})
install(FILES package.xml DESTINATION share/lo_core)

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  file(GLOB LO_CORE_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
  foreach(test_src ${LO_CORE_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(${test_name} PRIVATE lo_core GTest::gtest GTest::gtest_main)
    stride_target_flags(${test_name})
    gtest_discover_tests(${test_name} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 60 PROPERTIES TIMEOUT 600)
  endforeach()
endif()
