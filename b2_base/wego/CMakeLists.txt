# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

cmake_minimum_required(VERSION 3.8)
project(b2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(b2_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

# Use the pinned Unitree SDK2 submodule when it is present.  The fallback keeps
# the ROS-only bridge buildable for users who intentionally omit submodules.
option(B2_USE_BUNDLED_UNITREE_SDK2 "Build the pinned Unitree SDK2 submodule" ON)
set(B2_UNITREE_SDK2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../upstream/unitree_sdk2")
set(B2_UNITREE_SDK2_AVAILABLE FALSE)
set(B2_UNITREE_SDK2_BUNDLED FALSE)
if(B2_USE_BUNDLED_UNITREE_SDK2 AND EXISTS "${B2_UNITREE_SDK2_DIR}/CMakeLists.txt")
  set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Unitree SDK2 examples in b2_base" FORCE)
  add_subdirectory("${B2_UNITREE_SDK2_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2" EXCLUDE_FROM_ALL)
  set(B2_UNITREE_SDK2_AVAILABLE TRUE)
  set(B2_UNITREE_SDK2_BUNDLED TRUE)
else()
  find_package(unitree_sdk2 QUIET)
  if(unitree_sdk2_FOUND)
    set(B2_UNITREE_SDK2_AVAILABLE TRUE)
  endif()
endif()

add_executable(b2_cmd_vel_bridge b2_cmd_vel_bridge.cpp)
ament_target_dependencies(b2_cmd_vel_bridge
  geometry_msgs
  rclcpp
  std_srvs
  unitree_api
)

add_executable(b2_state_bridge b2_state_bridge.cpp)
ament_target_dependencies(b2_state_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  unitree_go
)

if(B2_UNITREE_SDK2_AVAILABLE)
  add_executable(b2_sdk_driver b2_sdk_driver.cpp)
  ament_target_dependencies(b2_sdk_driver
    b2_interface
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
  )
  target_link_libraries(b2_sdk_driver unitree_sdk2)
  if(B2_UNITREE_SDK2_BUNDLED)
    set_target_properties(b2_sdk_driver PROPERTIES
      BUILD_RPATH "${B2_UNITREE_SDK2_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
      INSTALL_RPATH "$ORIGIN/.."
    )
  endif()
else()
  message(STATUS "Unitree SDK2 not found: b2_sdk_driver will not be built")
endif()

install(TARGETS
  b2_cmd_vel_bridge
  b2_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(TARGET b2_sdk_driver)
  install(TARGETS b2_sdk_driver DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/set_unitree_static_ip.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # ament_copyright only recognizes open-source license templates. Keep the
  # existing proprietary license and verify its SPDX headers explicitly.
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(copyright test/test_source_headers.py)
  string(CONCAT B2_TEST_LOOPBACK_DDS
    "<CycloneDDS><Domain><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/></Interfaces><AllowMulticast>false</AllowMulticast>"
    "</General><Discovery><Peers><Peer address=\"localhost\"/></Peers></Discovery>"
    "</Domain></CycloneDDS>")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_measurement_stamp test/test_measurement_stamp.cpp)
  target_include_directories(test_measurement_stamp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_gtest(test_state_bridge test/test_state_bridge.cpp TIMEOUT 10
    ENV "ROS_DOMAIN_ID=229" "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "CYCLONEDDS_URI=${B2_TEST_LOOPBACK_DDS}")
  ament_target_dependencies(test_state_bridge geometry_msgs nav_msgs rclcpp
    sensor_msgs std_msgs tf2_ros unitree_go)
  if(TARGET b2_sdk_driver)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_sdk_safety test/test_sdk_safety.cpp TIMEOUT 60
      ENV "ROS_DOMAIN_ID=154" "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" "CYCLONEDDS_URI=${B2_TEST_LOOPBACK_DDS}")
    ament_target_dependencies(test_sdk_safety b2_interface diagnostic_msgs geometry_msgs nav_msgs rclcpp
      sensor_msgs std_msgs std_srvs tf2_ros)
    target_link_libraries(test_sdk_safety unitree_sdk2)
    set_tests_properties(test_sdk_safety PROPERTIES RUN_SERIAL TRUE)
    # Exercise the hardware launch's ROS RMW as well as the legacy Cyclone path.
    # Both fixtures keep native SDK traffic on loopback; no robot is contacted.
    ament_add_gtest_test(test_sdk_safety TEST_NAME test_sdk_safety_fastrtps TIMEOUT 60
      ENV "ROS_DOMAIN_ID=174" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
    set_tests_properties(test_sdk_safety_fastrtps PROPERTIES RUN_SERIAL TRUE)
    add_executable(benchmark_state_messages test/benchmark_state_messages.cpp)
    target_include_directories(benchmark_state_messages PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    ament_target_dependencies(benchmark_state_messages geometry_msgs nav_msgs sensor_msgs std_msgs)
    target_link_libraries(benchmark_state_messages unitree_sdk2)
  endif()
  add_executable(test_velocity_limits test/test_velocity_limits.cpp)
  target_include_directories(test_velocity_limits PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME velocity_limits COMMAND test_velocity_limits)
  # Check production and test sources separately so Google Test macros use
  # cppcheck's library model without excluding tests from static analysis.
  find_package(ament_cmake_cppcheck REQUIRED)
  set(ament_cmake_cppcheck_ADDITIONAL_EXCLUDE "test/*")
  ament_cppcheck(TESTNAME cppcheck_tests test LIBRARIES googletest)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Lint hooks create their tests inside ament_package(), not while dependencies
# are discovered. Set this afterward so analysis is executed rather than skipped.
if(BUILD_TESTING)
  set_tests_properties(cppcheck cppcheck_tests PROPERTIES
    ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
endif()
