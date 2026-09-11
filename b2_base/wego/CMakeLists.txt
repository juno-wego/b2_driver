cmake_minimum_required(VERSION 3.8)
project(b2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(b2_interface REQUIRED)
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
