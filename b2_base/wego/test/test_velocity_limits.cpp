#include "velocity_limits.hpp"

#include <cmath>
#include <iostream>
#include <limits>

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
  return 0;
}
