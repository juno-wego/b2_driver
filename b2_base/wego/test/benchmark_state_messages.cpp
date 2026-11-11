// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

// Explicit host microbenchmark: no DDS initialization, ROS graph or SDK commands.
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <unitree/idl/go2/SportModeState_.hpp>
#include "sport_state_messages.hpp"

template<class T> void observe(const T &value)
{
  // Keep every output field observable without adding serialization or a
  // volatile write to the conversion kernel being measured.
  asm volatile("" : : "g"(&value) : "memory");
}

int main(int argc, char **argv)
{
  const int samples = argc > 1 ? std::stoi(argv[1]) : 200;
  if (samples < 10 || samples > 10000) return 2;
  constexpr int batch = 1000;
  using Clock = std::chrono::steady_clock;
  unitree_go::msg::dds_::SportModeState_ state;
  state.imu_state().quaternion()[0] = .70710678F;
  state.imu_state().quaternion()[3] = .70710678F;
  state.velocity()[0] = .1F; state.velocity()[1] = -.03F;
  builtin_interfaces::msg::Time stamp;
  std::cout << "frames,samples,batch,construction,p50_batch_mean_ns,p95_batch_mean_ns\n" <<
    std::fixed << std::setprecision(3);
  for (const std::string &prefix : {std::string{}, std::string("inspection_robot_001/")}) {
    const auto odom = prefix + "odom", base = prefix + "base_link", imu = prefix + "imu_link";
    b2_base::SportStateMessages reused(odom, base, imu);
    std::vector<double> fresh, cached;
    auto measure = [&](bool reuse) {
        const auto start = Clock::now();
        for (int i = 0; i < batch; ++i) {
          stamp.nanosec = i; state.position()[0] = static_cast<float>(i % 20) * .001F;
          if (reuse) {
            reused.update(state, stamp); observe(reused);
          } else {
            b2_base::SportStateMessages message(odom, base, imu);
            message.update(state, stamp); observe(message);
          }
        }
        return std::chrono::duration<double, std::nano>(Clock::now()-start).count() / batch;
      };
    for (int i = -20; i < samples; ++i) {
      double a, b;
      if (i % 2) {a = measure(false); b = measure(true);} else {
        b = measure(true); a = measure(false);
      }
      if (i >= 0) {fresh.push_back(a); cached.push_back(b);}
    }
    for (const auto &entry : std::vector<std::pair<std::string, std::vector<double>>>{
        {"fresh", fresh}, {"reuse", cached}})
    {
      auto elapsed = entry.second; std::sort(elapsed.begin(), elapsed.end());
      std::cout << (prefix.empty() ? "default" : "namespaced") << ',' << samples << ',' <<
        batch << ',' << entry.first << ',' << elapsed[samples/2] << ',' <<
        elapsed[samples*95/100] << '\n';
    }
  }
}
