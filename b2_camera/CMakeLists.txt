cmake_minimum_required(VERSION 3.16)
project(b2_camera)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_library(b2_camera_image STATIC src/image_conversion.cpp)
target_compile_features(b2_camera_image PUBLIC cxx_std_17)
target_include_directories(b2_camera_image PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(b2_camera_image PUBLIC ${OpenCV_LIBRARIES}
  sensor_msgs::sensor_msgs__rosidl_generator_cpp std_msgs::std_msgs__rosidl_generator_cpp)

option(B2_CAMERA_BUILD_DRIVER "Build the Linux Unitree SDK hardware node" ON)
option(B2_CAMERA_USE_PINNED_SDK "Prefer the existing pinned SDK source over a system SDK" ON)
set(B2_CAMERA_UNITREE_SDK2_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../b2_base/upstream/unitree_sdk2"
  CACHE PATH "Existing pinned Unitree SDK2 source; never copied or patched by b2_camera")
if(B2_CAMERA_BUILD_DRIVER)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "B2 camera SDK binaries support Linux only; use -DB2_CAMERA_BUILD_DRIVER=OFF for image/launch tests without hardware")
  endif()
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(camera_calibration_parsers REQUIRED)
  find_package(camera_info_manager REQUIRED)
  if(NOT TARGET unitree_sdk2)
    if(B2_CAMERA_USE_PINNED_SDK AND EXISTS "${B2_CAMERA_UNITREE_SDK2_DIR}/CMakeLists.txt")
      set(BUILD_EXAMPLES OFF CACHE BOOL "Disable unused SDK examples for b2_camera" FORCE)
      add_subdirectory("${B2_CAMERA_UNITREE_SDK2_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2" EXCLUDE_FROM_ALL)
      install(FILES "${B2_CAMERA_UNITREE_SDK2_DIR}/LICENSE"
        DESTINATION share/${PROJECT_NAME}/licenses/unitree_sdk2)
      install(DIRECTORY "${B2_CAMERA_UNITREE_SDK2_DIR}/licenses/"
        DESTINATION share/${PROJECT_NAME}/licenses/third_party)
    else()
      find_package(unitree_sdk2 QUIET CONFIG)
    endif()
  endif()
  if(NOT TARGET unitree_sdk2)
    message(FATAL_ERROR "Unitree SDK2 is unavailable: initialize submodules or set B2_CAMERA_UNITREE_SDK2_DIR")
  endif()
  add_executable(b2_camera_node src/camera_node.cpp)
  target_compile_features(b2_camera_node PRIVATE cxx_std_17)
  target_link_libraries(b2_camera_node b2_camera_image unitree_sdk2 sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
  ament_target_dependencies(b2_camera_node rclcpp rcl_interfaces ament_index_cpp
    camera_calibration_parsers camera_info_manager)
  # SDK EXCLUDE_FROM_ALL does not install its shared DDS runtime. Install the
  # existing unmodified binaries and symlinks so release installs are relocatable.
  foreach(dds_target IN ITEMS ddsc ddscxx)
    if(NOT TARGET ${dds_target})
      message(FATAL_ERROR "Unitree SDK2 must provide ${dds_target} for runtime packaging")
    endif()
    get_target_property(dds_location ${dds_target} IMPORTED_LOCATION)
    if(NOT dds_location OR NOT EXISTS "${dds_location}")
      message(FATAL_ERROR "Cannot locate Unitree SDK2 ${dds_target} runtime")
    endif()
    get_filename_component(dds_directory "${dds_location}" DIRECTORY)
    file(GLOB dds_runtime "${dds_directory}/lib${dds_target}.so*")
    install(FILES ${dds_runtime} DESTINATION lib/${PROJECT_NAME}/sdk)
    set_property(TARGET b2_camera_node APPEND PROPERTY BUILD_RPATH "${dds_directory}")
  endforeach()
  set_target_properties(b2_camera_node PROPERTIES INSTALL_RPATH "$ORIGIN/sdk;$ORIGIN/..")
  # DT_RPATH is inherited by the private SDK's ddscxx -> ddsc dependency too;
  # no SDK directory is exported into the ROS installation's global library path.
  target_link_options(b2_camera_node PRIVATE "-Wl,--disable-new-dtags")
  install(TARGETS b2_camera_node DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS b2_camera_image ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(test_image_conversion test/test_image_conversion.cpp)
  target_link_libraries(test_image_conversion b2_camera_image)
  add_test(NAME image_conversion COMMAND test_image_conversion)
  ament_add_pytest_test(camera_launch test/test_launch.py)
  ament_add_pytest_test(calibration_profile test/test_calibration_profile.py)
  ament_add_pytest_test(dds_interop test/test_dds_interop.py TIMEOUT 25)
  if(B2_CAMERA_BUILD_DRIVER)
    add_executable(fake_video_peer test/fake_video_peer.cpp)
    target_compile_features(fake_video_peer PRIVATE cxx_std_17)
    target_link_libraries(fake_video_peer unitree_sdk2)
    ament_add_pytest_test(sdk_video_runtime test/test_sdk_video_runtime.py TIMEOUT 90
      ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONDONTWRITEBYTECODE=1
        "B2_CAMERA_FAKE_VIDEO_PEER=$<TARGET_FILE:fake_video_peer>"
        "B2_CAMERA_DRIVER_EXECUTABLE=$<TARGET_FILE:b2_camera_node>")
    set_tests_properties(sdk_video_runtime PROPERTIES RESOURCE_LOCK synthetic_sdk_video_112)
  endif()
endif()

ament_package()
