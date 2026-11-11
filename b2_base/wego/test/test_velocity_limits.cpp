// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include <cmath>
#include <iostream>
#include <limits>

#include "velocity_limits.hpp"
#include "state_conversion.hpp"

int main()
{
  const auto check = [](double input, double expected) {
      const double actual = b2_base::clamp_forward_speed(input, 1.0, 0.8);
      if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12) {
        std::cerr << "velocity clamp: " << input << " -> " << actual
                  << ", expected " << expected << '\n';
        return false;
      }
      return true;
    };
  if (!check(0.0, 0.0) || !check(0.6, 0.6) || !check(1.0, 1.0) ||
    !check(1.01, 1.0) || !check(2.0, 1.0) || !check(-0.6, -0.6) ||
    !check(-0.8, -0.8) || !check(-1.0, -0.8) ||
    !check(std::numeric_limits<double>::quiet_NaN(), 0.0) ||
    !check(std::numeric_limits<double>::infinity(), 0.0))
  {
    return 1;
  }
  if (b2_base::clamp_forward_speed(1.0, 0.5, 0.3) != 0.5 ||
    b2_base::clamp_forward_speed(-1.0, 0.5, 0.3) != -0.3 ||
    b2_base::clamp_forward_speed(-1.0, 0.5, 0.8) != -0.5)
  {
    return 1;
  }
  const auto forward = b2_base::velocity_in_body({0.F, .1F, 0.F},
      {.70710678F, 0.F, 0.F, .70710678F});
  const auto pitch = b2_base::velocity_in_body({0.F, 0.F, -.1F},
      {.70710678F, 0.F, .70710678F, 0.F});
  if (std::abs(forward[0] - .1) > 1e-7 || std::abs(forward[1]) > 1e-7 ||
    std::abs(pitch[0] - .1) > 1e-7 || std::abs(pitch[2]) > 1e-7 ||
    std::isfinite(b2_base::velocity_in_body({0.F, 0.F, 0.F}, {0.F, 0.F, 0.F, 0.F})[0]))
  {
    std::cerr << "Unitree world velocity must become physical body-frame feedback\n";
    return 1;
  }
  return 0;
}
