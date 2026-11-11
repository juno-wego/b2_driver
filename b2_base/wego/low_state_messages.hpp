// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <string>

#include <b2_interface/msg/motor_status_array.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <unitree/idl/go2/LowState_.hpp>

namespace b2_base
{
inline constexpr std::array<int, 12> kMotorIndex{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
inline constexpr std::array<const char *, 12> kJointNames{
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint",
  "FR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint",
  "RR_thigh_joint", "RR_calf_joint"};

// One instance belongs to the driver's mutually-exclusive low-state timer.
// Reuse the fixed-size vectors and joint names; publish(const Message&) keeps
// ownership here, and ROS copies/serializes the data before the next update.
struct LowStateMessages
{
  using LowState = unitree_go::msg::dds_::LowState_;

  explicit LowStateMessages(const std::string & frame)
  {
    motors.header.frame_id = frame;
    joints.name.assign(kJointNames.begin(), kJointNames.end());
    joints.position.resize(kMotorIndex.size());
    joints.velocity.resize(kMotorIndex.size());
    joints.effort.resize(kMotorIndex.size());
    motors.motors.resize(kMotorIndex.size());
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      motors.motors[i].joint_name = kJointNames[i];
    }
    const LowState shape;
    foot_force.data.resize(shape.foot_force().size());
    battery.cell_voltage.resize(shape.bms_state().cell_vol().size());
    battery.present = true;
  }

  void update(const LowState & state, const builtin_interfaces::msg::Time & stamp)
  {
    joints.header.stamp = battery.header.stamp = motors.header.stamp = stamp;
    for (std::size_t i = 0; i < kMotorIndex.size(); ++i) {
      const auto & motor = state.motor_state()[kMotorIndex[i]];
      joints.position[i] = motor.q();
      joints.velocity[i] = motor.dq();
      joints.effort[i] = motor.tau_est();
      auto & status = motors.motors[i];
      status.temperature_c = motor.temperature();
      status.mode = motor.mode();
      status.lost = motor.lost();
    }
    std::copy(state.foot_force().begin(), state.foot_force().end(), foot_force.data.begin());
    battery.voltage = state.power_v();
    // BMS current is mA, positive on discharge. ROS uses A, positive on charge.
    const auto current = state.bms_state().current();
    battery.current = -static_cast<float>(current) / 1000.0F;
    battery.power_supply_status = current < 0 ?
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING : current > 0 ?
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING :
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery.percentage = static_cast<float>(state.bms_state().soc()) / 100.0F;
    int hottest = std::numeric_limits<int>::lowest();
    for (const auto temperature : state.bms_state().bq_ntc()) {
      hottest = std::max(hottest, static_cast<int>(temperature));
    }
    for (const auto temperature : state.bms_state().mcu_ntc()) {
      hottest = std::max(hottest, static_cast<int>(temperature));
    }
    battery.temperature = static_cast<float>(hottest);
    const auto & millivolts = state.bms_state().cell_vol();
    for (std::size_t i = 0; i < millivolts.size(); ++i) {
      battery.cell_voltage[i] = static_cast<float>(millivolts[i]) / 1000.0F;
    }
  }

  sensor_msgs::msg::JointState joints;
  sensor_msgs::msg::BatteryState battery;
  std_msgs::msg::Int16MultiArray foot_force;
  b2_interface::msg::MotorStatusArray motors;
};
}  // namespace b2_base
