// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace b2_base
{
inline void validate_motion_parameters(
  double forward, double reverse, double lateral,
  double angular, double deadband, double rate_hz, double command_timeout)
{
  const std::array<std::pair<const char *, double>, 5> magnitudes{{
    {"max_linear_x", forward}, {"max_reverse_linear_x", reverse},
    {"max_linear_y", lateral}, {"max_angular_z", angular}, {"deadband", deadband}}};
  for (const auto & item : magnitudes) {
    if (!std::isfinite(item.second) || item.second < 0 ||
      item.second > std::numeric_limits<float>::max())
    {
      throw std::invalid_argument(std::string(item.first) + " must be finite and nonnegative");
    }
  }
  if (!std::isfinite(rate_hz) || rate_hz <= 0 || rate_hz > 1e9) {
    throw std::invalid_argument(
        "command_rate_hz must be positive with a representable timer period");
  }
  if (!std::isfinite(command_timeout) || command_timeout <= 0) {
    throw std::invalid_argument("cmd_timeout must be finite and positive");
  }
}

template<typename Twist>
inline bool finite_twist(const Twist & command)
{
  return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
         std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
         std::isfinite(command.angular.y) && std::isfinite(command.angular.z);
}

inline double clamp_forward_speed(double value, double forward_limit, double reverse_limit)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  const double forward = std::abs(forward_limit);
  const double reverse = std::min(forward, std::abs(reverse_limit));
  return std::clamp(value, -reverse, forward);
}
}  // namespace b2_base
