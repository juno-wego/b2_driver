# b2_interface

This directory is the Wego-owned `b2_interface` ROS package.

- The sibling `../upstream/unitree_ros2/` directory is the pinned, unmodified
  official Unitree ROS 2 source
  submodule. Its `cyclonedds_ws/src/unitree/{unitree_go,unitree_api}` packages
  are built directly from that checkout.

`SetMotionMode` selects a B2 posture or locomotion mode. Velocity remains a
standard `geometry_msgs/Twist` on `cmd_vel`; emergency movement cancellation
remains the independent `std_srvs/Trigger` service `b2/motion/stop`.
