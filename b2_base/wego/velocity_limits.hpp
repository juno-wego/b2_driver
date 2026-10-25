#pragma once

#include <algorithm>
#include <cmath>

namespace b2_base
{
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
