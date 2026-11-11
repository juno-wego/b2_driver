// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"

#define main b2_state_bridge_main
// Include the local node implementation rather than replace it with a mock.
#include "../b2_state_bridge.cpp"  // NOLINT(build/include)
#undef main

using namespace std::chrono_literals;

TEST(B2StateBridge, BatteryRemainsFreshForDockingAndStopsWithoutNewTelemetry)
{
  rclcpp::InitOptions options;
  options.set_domain_id(229);  // Loopback test only; never join the B2 graph.
  rclcpp::init(0, nullptr, options);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  auto bridge = std::make_shared<B2StateBridge>();
  auto sensors = std::make_shared<rclcpp::Node>("legacy_battery_test");
  auto low = sensors->create_publisher<unitree_go::msg::LowState>(
    "/lowstate", rclcpp::SensorDataQoS());
  std::vector<sensor_msgs::msg::BatteryState> samples;
  auto battery = sensors->create_subscription<sensor_msgs::msg::BatteryState>(
    "/b2/battery_state", rclcpp::QoS(1).reliable().transient_local(),
    [&](const sensor_msgs::msg::BatteryState & value) {samples.push_back(value);});
  unitree_go::msg::LowState state;
  state.bms_state.current = -3000;
  state.bms_state.soc = 60;
  auto timer = sensors->create_wall_timer(20ms, [&]() {low->publish(state);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(sensors);
  auto advance = [&](std::chrono::milliseconds duration) {
      const auto until = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < until) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    };
  advance(200ms);  // Discovery and the first real LowState.
  samples.clear();
  advance(700ms);
  // More than the 0.5s charging dwell, with no gap near its 0.5s freshness limit.
  ASSERT_GE(samples.size(), 5u);
  for (size_t i = 0; i < samples.size(); ++i) {
    EXPECT_TRUE(samples[i].present);
    EXPECT_FLOAT_EQ(samples[i].current, 3.F);
    EXPECT_EQ(samples[i].power_supply_status,
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
    if (i > 0) {
      const auto dt = rclcpp::Time(samples[i].header.stamp) - samples[i - 1].header.stamp;
      EXPECT_GT(dt.nanoseconds(), 0);
      EXPECT_LT(dt.seconds(), .25);
    }
  }
  timer->cancel();
  advance(40ms);  // Drain a possible in-flight input before recording the count.
  const auto count = samples.size();
  advance(250ms);
  EXPECT_EQ(samples.size(), count) << "Cached LowState must not be retimestamped as fresh battery";
}
