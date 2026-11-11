// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <b2_interface/msg/driver_status.hpp>
#include <b2_interface/msg/motor_status_array.hpp>
#include <b2_interface/msg/sport_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "driver_parameters.hpp"
#include "low_state_messages.hpp"
#include "measurement_stamp.hpp"
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>

namespace b2_base
{
// Read-only SDK polling has its own callback group and clients. It never calls
// SelectMode/ReleaseMode/ServiceSwitch and cannot take over the robot controller.
class DriverStatus
{
  using Clock = std::chrono::steady_clock;
  using Status = b2_interface::msg::DriverStatus;
  using SportState = unitree_go::msg::dds_::SportModeState_;
  using LowState = unitree_go::msg::dds_::LowState_;

public:
  struct Readiness
  {
    bool motion_ready;
    const char * reason;
  };

  DriverStatus(
    rclcpp::Node & node, double state_timeout, const std::string & frame,
    bool control_enabled = true)
  : node_(node), state_timeout_(state_timeout), frame_(frame), control_enabled_(control_enabled)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    require_ready_ = node.declare_parameter("controller.require_ready", true, descriptor);
    form_ = node.declare_parameter<std::string>("controller.expected_form", "0", descriptor);
    names_ = node.declare_parameter<std::vector<std::string>>(
      "controller.allowed_names", {"normal", "ai", "advanced"}, descriptor);
    const double period = node.declare_parameter("controller.poll_period_s", 1.0, descriptor);
    const double timeout = node.declare_parameter("controller.rpc_timeout_s", 0.15, descriptor);
    max_age_ = node.declare_parameter("controller.max_age_s", 2.5, descriptor);
    stationary_linear_ = node.declare_parameter("configuration.stationary_linear_mps", 0.03,
        descriptor);
    stationary_angular_ = node.declare_parameter("configuration.stationary_angular_radps", 0.05,
        descriptor);
    const bool invalid_names = std::any_of(names_.begin(), names_.end(),
        [](const auto & name) {return name.empty();});
    const double largest_period = std::chrono::duration<double>(
      std::chrono::nanoseconds::max()).count() / 2;
    if (!std::isfinite(period) || period < 1e-9 || period > largest_period ||
      !valid_sdk_timeout(timeout) ||
      !std::isfinite(max_age_) || max_age_ <= period + 3 * timeout ||
      !std::isfinite(stationary_linear_) || stationary_linear_ < 0 ||
      !std::isfinite(stationary_angular_) || stationary_angular_ < 0 ||
      form_.empty() || names_.empty() || invalid_names)
    {
      throw std::invalid_argument(
          "Controller polling needs finite positive timing, "
          "a sufficient max_age_s and explicit form/names");
    }
    switcher_.SetTimeout(static_cast<float>(timeout));
    robot_state_.SetTimeout(static_cast<float>(timeout));
    switcher_.Init();
    robot_state_.Init();
    controller_.check_mode_result = controller_.service_list_result = controller_.version_result =
      -1;
    const auto sensor_qos = rclcpp::SensorDataQoS();
    sport_pub_ = node.create_publisher<b2_interface::msg::SportStatus>("/b2/sport_status",
        sensor_qos);
    motors_pub_ = node.create_publisher<b2_interface::msg::MotorStatusArray>("/b2/motor_status",
        sensor_qos);
    status_pub_ = node.create_publisher<Status>("/b2/driver_status",
        rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
        10);
    probe_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    probe_timer_ = node.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      [this]() {poll_controller();}, probe_group_);
    const auto publish = [this]() {publish_status();};
    status_timer_ = node.create_wall_timer(std::chrono::milliseconds(200), publish);
  }

  void observe_sport(const SportState & state)
  {
    MeasurementStamp compatibility;
    compatibility.valid = true; compatibility.reason = "UNSPECIFIED_TEST_OR_LEGACY_SOURCE";
    observe_sport(state, compatibility);
  }

  void observe_sport(const SportState & state, const MeasurementStamp & timing)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sport_seen_ = true;
    sport_at_ = Clock::now();
    sport_clock_ = timing;
    error_code_ = state.error_code();
    const auto & velocity = state.velocity();
    const double squared_speed = static_cast<double>(velocity[0]) * velocity[0] +
      static_cast<double>(velocity[1]) * velocity[1] + static_cast<double>(velocity[2]) *
      velocity[2];
    if (timing.valid && std::isfinite(squared_speed) && std::isfinite(state.yaw_speed()) &&
      squared_speed <= stationary_linear_ * stationary_linear_ &&
      std::abs(state.yaw_speed()) <= stationary_angular_)
    {
      stopped_at_[0] = stopped_at_[1]; stopped_at_[1] = stopped_at_[2]; stopped_at_[2] = sport_at_;
    } else {stopped_at_.fill(Clock::time_point{});}
  }

  bool stationary_after(Clock::time_point stop_acknowledged) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_at_[0] >= stop_acknowledged && sport_seen_ &&
           age(Clock::now(), sport_at_) <= state_timeout_;
  }

  void observe_low(const LowState & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    low_seen_ = true;
    low_at_ = Clock::now();
    for (std::size_t i = 0; i < kMotorIndex.size(); ++i) {
      const auto & motor = state.motor_state()[kMotorIndex[i]];
      motor_temperature_[i] = motor.temperature();
      motor_mode_[i] = motor.mode();
      motor_lost_[i] = motor.lost();
    }
  }

  void publish_sport(const SportState & state, const builtin_interfaces::msg::Time & stamp)
  {
    b2_interface::msg::SportStatus message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_;
    message.error_code = state.error_code();
    message.mode = state.mode();
    message.gait_type = state.gait_type();
    message.body_height = state.body_height();
    message.foot_raise_height = state.foot_raise_height();
    sport_pub_->publish(message);
  }

  void publish_motors(const b2_interface::msg::MotorStatusArray & message)
  {
    motors_pub_->publish(message);
  }

  Readiness readiness() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return readiness_locked(flags_locked(Clock::now()));
  }

  void suspend_velocity(bool suspended)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_suspended_ = suspended;
  }

  Status status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Status result = controller_;
    result.header.stamp = node_.now();
    result.header.frame_id = frame_;
    const auto flags = flags_locked(Clock::now());
    const auto ready = readiness_locked(flags);
    result.sport_state_fresh = flags.sport_fresh;
    result.low_state_fresh = flags.low_fresh;
    result.sport_error_code = error_code_;
    result.controller_fresh = flags.controller_fresh;
    result.controller_ready_required = require_ready_;
    result.controller_ready = flags.controller_ready;
    result.reason = ready.reason;
    result.motion_ready = ready.motion_ready;
    return result;
  }

private:
  struct Flags
  {
    bool sport_fresh, low_fresh, controller_fresh, controller_ready;
  };

  Flags flags_locked(Clock::time_point now) const
  {
    const bool fresh = controller_seen_ && age(now, controller_at_) <= max_age_;
    return {sport_seen_ && sport_clock_.valid && age(now, sport_at_) <= state_timeout_,
      low_seen_ && age(now, low_at_) <= state_timeout_, fresh,
      fresh && controller_.check_mode_result == 0 && controller_.controller_form == form_ &&
      std::find(names_.begin(), names_.end(), controller_.controller_name) != names_.end()};
  }

  Readiness readiness_locked(const Flags & flags) const
  {
    const bool ready = control_enabled_ && !velocity_suspended_ && flags.sport_fresh && flags.low_fresh &&
      error_code_ == 0 && (!require_ready_ || flags.controller_ready);
    const char * reason = ready ? "READY" : !control_enabled_ ? "CONTROL_DISABLED" :
      velocity_suspended_ ? "POSTURE_MOTION_DISABLED" :
      sport_seen_ && !sport_clock_.valid ? "SPORT_TIME_INVALID" :
      !flags.sport_fresh ? "SPORT_STATE_STALE" :
      !flags.low_fresh ? "LOW_STATE_STALE" :
      error_code_ != 0 ? "SPORT_ERROR" :
      require_ready_ && !flags.controller_fresh ? "CONTROLLER_STATUS_STALE" :
      require_ready_ && controller_.check_mode_result != 0 ? "CONTROLLER_QUERY_FAILED" :
      "CONTROLLER_MODE_UNAVAILABLE";
    return {ready, reason};
  }

  static double age(Clock::time_point now, Clock::time_point then)
  {return std::chrono::duration<double>(now - then).count();}

  void poll_controller()
  {
    Status report;
    report.check_mode_result = report.service_list_result = report.version_result = -1;
    try {
      report.check_mode_result = switcher_.CheckMode(report.controller_form,
          report.controller_name);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "B2 controller query failed: %s", error.what());
    }
    const auto checked_at = Clock::now();
    // Publish CheckMode promptly even when optional metadata RPCs time out.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      controller_.check_mode_result = report.check_mode_result;
      controller_.controller_form = report.controller_form;
      controller_.controller_name = report.controller_name;
      controller_seen_ = true;
      controller_at_ = checked_at;
    }
    try {
      std::vector<unitree::robot::b2::ServiceState> services;
      report.service_list_result = robot_state_.ServiceList(services);
      if (report.service_list_result == 0) {
        for (const auto & service : services) {
          report.service_names.push_back(service.name);
          report.service_status.push_back(service.status);
          report.service_protect.push_back(service.protect);
        }
      }
      std::map<std::string, std::string> modules;
      report.version_result = robot_state_.GetPkgVersion(report.package_version, modules);
      if (report.version_result == 0) {
        for (const auto & module : modules) {
          report.module_names.push_back(module.first);
          report.module_versions.push_back(module.second);
        }
      } else {report.package_version.clear();}
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "B2 metadata query failed: %s", error.what());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    controller_ = std::move(report);
  }

  void publish_status()
  {
    const auto report = status();
    status_pub_->publish(report);
    using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.status.reserve(4 + kMotorIndex.size());
    diagnostics.header = report.header;
    auto add = [&](const std::string & name, uint8_t level,
      const std::string & message) -> Diagnostic & {
        Diagnostic item;
        item.name = "B2/" + name;
        item.hardware_id = "unitree_b2";
        item.level = level;
        item.message = message;
        diagnostics.status.push_back(std::move(item));
        return diagnostics.status.back();
      };
    auto value = [](Diagnostic & item, const std::string & key, const std::string & text) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key; pair.value = text; item.values.push_back(pair);
      };
    auto & sport = add("sport", !report.sport_state_fresh ? Diagnostic::STALE :
        report.sport_error_code ? Diagnostic::ERROR : Diagnostic::OK,
      !report.sport_state_fresh ? "Sport feedback stale" :
      report.sport_error_code ? "SDK reports sport error; see raw error_code" :
        "Sport feedback received");
    value(sport, "error_code", std::to_string(report.sport_error_code));
    MeasurementStamp timing;
    {std::lock_guard<std::mutex> lock(mutex_); timing = sport_clock_;}
    auto & source_clock = add("source_time", !timing.valid ? Diagnostic::ERROR :
        std::string(timing.reason) == "RECEIPT_COMPATIBILITY_NOT_SYNCHRONIZED" ?
        Diagnostic::WARN : Diagnostic::OK, timing.reason);
    value(source_clock, "source_stamp_ns", std::to_string(timing.source_ns));
    value(source_clock, "measurement_stamp_ns", std::to_string(timing.stamp_ns));
    value(source_clock, "received_stamp_ns", std::to_string(timing.received_ns));
    auto & control = add("controller", !report.controller_fresh ? Diagnostic::STALE :
        report.controller_ready ? Diagnostic::OK : Diagnostic::ERROR,
      report.controller_ready ? "Supported motion controller active" :
        "Motion controller not verified ready");
    value(control, "name", report.controller_name);
    value(control, "form", report.controller_form);
    value(control, "check_mode_result", std::to_string(report.check_mode_result));
    value(control, "service_list_result", std::to_string(report.service_list_result));
    value(control, "version_result", std::to_string(report.version_result));
    value(control, "package_version", report.package_version);
    add("motion_ready", report.motion_ready ? Diagnostic::OK :
      report.reason == "CONTROL_DISABLED" || report.reason == "POSTURE_MOTION_DISABLED" ?
      Diagnostic::WARN : Diagnostic::ERROR, report.reason);
    std::array<uint8_t, 12> temperatures, modes;
    std::array<uint32_t, 12> losses;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      temperatures = motor_temperature_; modes = motor_mode_; losses = motor_lost_;
    }
    for (std::size_t i = 0; i < kMotorIndex.size(); ++i) {
      auto & motor = add(kJointNames[i],
          report.low_state_fresh ? Diagnostic::OK : Diagnostic::STALE,
          report.low_state_fresh ? "Raw motor feedback; temperature limits are firmware-specific" :
          "Motor feedback stale");
      motor.values.reserve(3);
      value(motor, "temperature_c", std::to_string(temperatures[i]));
      value(motor, "mode", std::to_string(modes[i]));
      value(motor, "lost_raw", std::to_string(losses[i]));
    }
    diagnostics_pub_->publish(diagnostics);
  }

  rclcpp::Node & node_;
  double state_timeout_, max_age_, stationary_linear_, stationary_angular_;
  std::string frame_, form_;
  bool require_ready_;
  bool control_enabled_;
  bool velocity_suspended_{false};
  std::vector<std::string> names_;
  mutable std::mutex mutex_;
  bool sport_seen_{false}, low_seen_{false}, controller_seen_{false};
  MeasurementStamp sport_clock_;
  Clock::time_point sport_at_{}, low_at_{}, controller_at_{};
  std::array<Clock::time_point, 3> stopped_at_{};
  uint32_t error_code_{0};
  Status controller_;
  std::array<uint8_t, 12> motor_temperature_{}, motor_mode_{};
  std::array<uint32_t, 12> motor_lost_{};
  unitree::robot::b2::MotionSwitcherClient switcher_;
  unitree::robot::b2::RobotStateClient robot_state_;
  rclcpp::CallbackGroup::SharedPtr probe_group_;
  rclcpp::TimerBase::SharedPtr probe_timer_, status_timer_;
  rclcpp::Publisher<b2_interface::msg::SportStatus>::SharedPtr sport_pub_;
  rclcpp::Publisher<b2_interface::msg::MotorStatusArray>::SharedPtr motors_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};
}  // namespace b2_base
