cmake_minimum_required(VERSION 3.8)
project(b2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(b2_cmd_vel_bridge src/b2_cmd_vel_bridge.cpp)
ament_target_dependencies(b2_cmd_vel_bridge
  geometry_msgs
  rclcpp
  std_srvs
  unitree_api
)

add_executable(b2_state_bridge src/b2_state_bridge.cpp)
ament_target_dependencies(b2_state_bridge
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  unitree_go
)

install(TARGETS
  b2_cmd_vel_bridge
  b2_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
