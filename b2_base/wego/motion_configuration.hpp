// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <b2_interface/srv/configure_gait.hpp>
#include <b2_interface/srv/configure_height_mode.hpp>
#include <b2_interface/srv/set_body_height.hpp>
#include <b2_interface/srv/set_body_pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace b2_base
{
// These are deployment limits, not claimed Unitree firmware limits. Unknown
// height conventions must never be filled from another model's SDK examples.
struct MotionConfiguration
{
  explicit MotionConfiguration(rclcpp::Node & node)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    height_enabled = node.declare_parameter("body_height.enabled", false, descriptor);
    height_min = node.declare_parameter("body_height.minimum", 0.0, descriptor);
    height_max = node.declare_parameter("body_height.maximum", 0.0, descriptor);
    pose_enabled = node.declare_parameter("body_pose.enabled", false, descriptor);
    roll_limit = node.declare_parameter("body_pose.max_roll_rad", 0.0, descriptor);
    pitch_limit = node.declare_parameter("body_pose.max_pitch_rad", 0.0, descriptor);
    yaw_limit = node.declare_parameter("body_pose.max_yaw_rad", 0.0, descriptor);
    gait_enabled = node.declare_parameter("gait_configuration.enabled", false, descriptor);
    height_mode_enabled = node.declare_parameter("height_mode_configuration.enabled", false,
        descriptor);
    stop_timeout = node.declare_parameter("configuration.stop_confirm_timeout_s", 2.0, descriptor);
    allowed_gaits = node.declare_parameter<std::vector<int64_t>>("gait_configuration.allowed_ids",
        {}, descriptor);
    if (!std::isfinite(stop_timeout) || stop_timeout <= 0 || stop_timeout > 10 ||
      !finite_float(height_min) || !finite_float(height_max) ||
      (height_enabled && height_min >= height_max) ||
      !finite_float(roll_limit) || roll_limit < 0 ||
      !finite_float(pitch_limit) || pitch_limit < 0 ||
      !finite_float(yaw_limit) || yaw_limit < 0 ||
      (pose_enabled && roll_limit == 0 && pitch_limit == 0 && yaw_limit == 0) ||
      std::any_of(allowed_gaits.begin(), allowed_gaits.end(), [](int64_t id) {
        return id < std::numeric_limits<int32_t>::min() || id > std::numeric_limits<int32_t>::max();
      }))
    {
      throw std::invalid_argument(
          "Body height/pose settings need explicit finite deployment limits and valid gait IDs");
    }
  }

  static bool finite_float(double value)
  {return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();}

  std::string validate(const b2_interface::srv::SetBodyHeight::Request & request) const
  {
    if (!height_enabled) {
      return "BODY_HEIGHT_DISABLED: configure firmware-verified bounds before enabling";
    }
    if (!finite_float(request.height) || request.height < height_min ||
      request.height > height_max)
    {
      return "BODY_HEIGHT_OUT_OF_RANGE";
    }
    return {};
  }

  std::string validate(const b2_interface::srv::SetBodyPose::Request & request) const
  {
    if (!pose_enabled) {
      return "BODY_POSE_DISABLED: configure permitted body angles before enabling";
    }
    if (!finite_float(request.roll) || !finite_float(request.pitch) || !finite_float(request.yaw) ||
      std::abs(request.roll) > roll_limit || std::abs(request.pitch) > pitch_limit ||
      std::abs(request.yaw) > yaw_limit)
    {
      return "BODY_POSE_OUT_OF_RANGE";
    }
    return {};
  }

  std::string validate(const b2_interface::srv::ConfigureGait::Request & request) const
  {
    using Request = b2_interface::srv::ConfigureGait::Request;
    if (!gait_enabled) {return "GAIT_CONFIGURATION_DISABLED";}
    if (request.feature > Request::AUTO_RECOVERY) {return "UNSUPPORTED_GAIT_FEATURE";}
    if (request.feature == Request::SWITCH_GAIT &&
      std::find(allowed_gaits.begin(), allowed_gaits.end(), request.gait_id) == allowed_gaits.end())
    {
      return "GAIT_ID_NOT_CONFIGURED";
    }
    return {};
  }

  std::string validate(const b2_interface::srv::ConfigureHeightMode::Request & request) const
  {
    if (!height_mode_enabled) {return "HEIGHT_MODE_CONFIGURATION_DISABLED";}
    if (request.feature > b2_interface::srv::ConfigureHeightMode::Request::GAIT_HEIGHT) {
      return "UNSUPPORTED_HEIGHT_MODE_FEATURE";
    }
    return {};
  }

  bool height_enabled{false}, pose_enabled{false}, gait_enabled{false};
  bool height_mode_enabled{false};
  double height_min{0}, height_max{0}, roll_limit{0}, pitch_limit{0}, yaw_limit{0};
  double stop_timeout{2.0};
  std::vector<int64_t> allowed_gaits;
};
}  // namespace b2_base
