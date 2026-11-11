// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary
#pragma once

#include <array>
#include <cmath>
#include <limits>

namespace b2_base
{
// SportModeState.velocity is in the Unitree odometry frame. ROS Odometry.twist
// must be in child_frame_id (the physical body), including roll and pitch.
inline std::array<double, 3> velocity_in_body(
  const std::array<float, 3> & velocity, const std::array<float, 4> & quaternion_wxyz)
{
  double norm = 0.0;
  for (const auto value : quaternion_wxyz) {
    norm += static_cast<double>(value) * value;
  }
  if (!std::isfinite(norm) || norm < 1e-12 ||
    !std::isfinite(velocity[0]) || !std::isfinite(velocity[1]) || !std::isfinite(velocity[2]))
  {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid};
  }
  const double w = quaternion_wxyz[0] / std::sqrt(norm);
  const double x = -quaternion_wxyz[1] / std::sqrt(norm);
  const double y = -quaternion_wxyz[2] / std::sqrt(norm);
  const double z = -quaternion_wxyz[3] / std::sqrt(norm);
  const double tx = 2.0 * (y * velocity[2] - z * velocity[1]);
  const double ty = 2.0 * (z * velocity[0] - x * velocity[2]);
  const double tz = 2.0 * (x * velocity[1] - y * velocity[0]);
  return {velocity[0] + w * tx + y * tz - z * ty,
    velocity[1] + w * ty + z * tx - x * tz,
    velocity[2] + w * tz + x * ty - y * tx};
}
}  // namespace b2_base
