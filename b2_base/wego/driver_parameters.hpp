// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace b2_base
{
inline bool valid_sdk_timeout(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0 || seconds > std::numeric_limits<float>::max()) {
    return false;
  }
  const float sdk_seconds = static_cast<float>(seconds);
  const double microseconds = static_cast<double>(sdk_seconds) * 1e6;
  return std::isfinite(sdk_seconds) && microseconds >= 1.0 &&
         microseconds < static_cast<double>(std::numeric_limits<int64_t>::max()) / 2;
}

template<typename T>
T startup_parameter(rclcpp::Node & node, const std::string & name, const T & value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, value, descriptor);
}
}  // namespace b2_base
