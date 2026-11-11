cmake_minimum_required(VERSION 3.16)
project(b2_camera_calibration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/cameracalibrator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch docs DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(calibration_launch test/test_launch.py)
  find_program(XVFB_RUN xvfb-run)
  if(XVFB_RUN)
    add_test(NAME calibration_runtime COMMAND ${XVFB_RUN} -a /usr/bin/python3
      -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime.py -q)
    set_tests_properties(calibration_runtime PROPERTIES TIMEOUT 60
      RESOURCE_LOCK synthetic_camera_calibration_115
      ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=226;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()
endif()
ament_package()
