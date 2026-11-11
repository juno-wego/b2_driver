// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary
#pragma once

#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include "state_conversion.hpp"

namespace b2_base
{
// Startup-fixed frame names and covariance defaults are initialized once.
// The ROS state timer owns this storage; SDK receive callbacks never modify it.
// publish(const Message &) copies/serializes synchronously, so later updates do
// not mutate messages already delivered to an intra-/inter-process subscriber.
struct SportStateMessages
{
  SportStateMessages(const std::string &odom_frame, const std::string &base_frame,
    const std::string &imu_frame)
  {
    imu.header.frame_id = imu_frame;
    odom.header.frame_id = odom_frame; odom.child_frame_id = base_frame;
    transform.header.frame_id = odom_frame; transform.child_frame_id = base_frame;
  }

  template<class SportState>
  void update(const SportState &state, const builtin_interfaces::msg::Time &stamp,
    bool with_tf = true)
  {
    const auto &raw = state.imu_state();
    imu.header.stamp = stamp;
    imu.orientation.w = raw.quaternion()[0]; imu.orientation.x = raw.quaternion()[1];
    imu.orientation.y = raw.quaternion()[2]; imu.orientation.z = raw.quaternion()[3];
    imu.angular_velocity.x = raw.gyroscope()[0]; imu.angular_velocity.y = raw.gyroscope()[1];
    imu.angular_velocity.z = raw.gyroscope()[2];
    imu.linear_acceleration.x = raw.accelerometer()[0];
    imu.linear_acceleration.y = raw.accelerometer()[1];
    imu.linear_acceleration.z = raw.accelerometer()[2];
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = state.position()[0];
    odom.pose.pose.position.y = state.position()[1];
    odom.pose.pose.position.z = state.position()[2];
    odom.pose.pose.orientation = imu.orientation;
    const auto velocity = velocity_in_body(state.velocity(), raw.quaternion());
    odom.twist.twist.linear.x = velocity[0]; odom.twist.twist.linear.y = velocity[1];
    odom.twist.twist.linear.z = velocity[2]; odom.twist.twist.angular.z = state.yaw_speed();
    if (with_tf) {
      transform.header.stamp = stamp;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
    }
    mode.data = state.mode();
  }

  sensor_msgs::msg::Imu imu;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped transform;
  std_msgs::msg::Int32 mode;
};
}  // namespace b2_base
