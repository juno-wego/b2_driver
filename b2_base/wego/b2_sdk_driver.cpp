// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include "velocity_limits.hpp"
#include "driver_parameters.hpp"
#include "state_conversion.hpp"
#include "driver_status.hpp"
#include "low_state_messages.hpp"
#include "motion_configuration.hpp"
#include "sdk_callback_executors.hpp"
#include "sport_state_messages.hpp"
#include "b2_interface/srv/set_motion_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <unitree/idl/go2/LowState_.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include "measurement_stamp.hpp"
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{
constexpr char kSportStateTopic[] = "rt/sportmodestate";
constexpr char kLowStateTopic[] = "rt/lowstate";

double clamp_abs(double value, double maximum)
{
  return std::clamp(value, -maximum, maximum);
}

double apply_deadband(double value, double deadband)
{
  return std::abs(value) < deadband ? 0.0 : value;
}

class B2SdkDriver : public rclcpp::Node
{
public:
  explicit B2SdkDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("b2_sdk_driver", options)
  {
    const auto interface = b2_base::startup_parameter<std::string>(*this, "network_interface", "");
    const auto domain = static_cast<int32_t>(b2_base::startup_parameter<int>(*this, "dds_domain",
        0));
    if (interface.empty()) {
      throw std::runtime_error(
              "network_interface is required for b2_sdk_driver (for example: enp3s0)");
    }

    try {
      unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
    } catch (const std::exception & error) {
      throw std::runtime_error("Unitree SDK2 ChannelFactory initialization failed on " + interface +
              ": " + error.what());
    }

    cmd_vel_topic_ = b2_base::startup_parameter<std::string>(*this, "cmd_vel_topic", "/cmd_vel");
    control_enabled_ = b2_base::startup_parameter(*this, "enable_control", true);
    odom_topic_ = b2_base::startup_parameter<std::string>(*this, "odom_topic", "/b2/odom");
    imu_topic_ = b2_base::startup_parameter<std::string>(*this, "imu_topic", "/b2/imu/data");
    joint_state_topic_ = b2_base::startup_parameter<std::string>(*this, "joint_state_topic",
        "/b2/joint_states");
    battery_topic_ = b2_base::startup_parameter<std::string>(*this, "battery_topic",
        "/b2/battery_state");
    foot_force_topic_ = b2_base::startup_parameter<std::string>(*this, "foot_force_topic",
        "/b2/foot_force");
    // MuJoCo already uses /b2/mode as a String command input. Keep the
    // hardware's raw Unitree numeric state separate rather than publishing a
    // conflicting message type on the same ROS name.
    mode_topic_ = b2_base::startup_parameter<std::string>(*this, "mode_topic", "/b2/mode_code");
    comm_ok_topic_ = b2_base::startup_parameter<std::string>(*this, "comm_ok_topic", "/b2/comm_ok");
    odom_frame_ = b2_base::startup_parameter<std::string>(*this, "odom_frame", "odom");
    base_frame_ = b2_base::startup_parameter<std::string>(*this, "base_frame", "base_link");
    imu_frame_ = b2_base::startup_parameter<std::string>(*this, "imu_frame", "imu_link");
    publish_tf_ = b2_base::startup_parameter<bool>(*this, "publish_tf", true);
    const auto time_mode = b2_base::startup_parameter<std::string>(*this, "source_time.mode",
        "source");
    const auto time_offset = b2_base::startup_parameter<int64_t>(*this, "source_time.offset_ns", 0);
    sport_clock_ = std::make_unique<b2_base::MeasurementClock>(time_mode, time_offset,
      b2_base::startup_parameter<double>(*this, "source_time.max_age_s", 0.5),
      b2_base::startup_parameter<double>(*this, "source_time.future_tolerance_s", 0.05));
    time_reference_pub_ = create_publisher<sensor_msgs::msg::TimeReference>(
      "/b2/time_reference", rclcpp::SensorDataQoS());
    max_linear_x_ = b2_base::startup_parameter<double>(*this, "max_linear_x", 1.0);
    max_reverse_linear_x_ = b2_base::startup_parameter<double>(*this, "max_reverse_linear_x", 0.8);
    max_linear_y_ = b2_base::startup_parameter<double>(*this, "max_linear_y", 0.4);
    max_angular_z_ = b2_base::startup_parameter<double>(*this, "max_angular_z", 1.2);
    deadband_ = b2_base::startup_parameter<double>(*this, "deadband", 0.01);
    command_rate_hz_ = b2_base::startup_parameter<double>(*this, "command_rate_hz", 20.0);
    cmd_timeout_ = b2_base::startup_parameter<double>(*this, "cmd_timeout", 0.5);
    stop_on_timeout_ = b2_base::startup_parameter<bool>(*this, "stop_on_timeout", true);
    auto_balance_stand_ = b2_base::startup_parameter<bool>(*this, "auto_balance_stand_on_start",
        false);
    state_timeout_ = b2_base::startup_parameter<double>(*this, "state_timeout", 0.5);
    const auto command_rpc_timeout = b2_base::startup_parameter<double>(*this,
        "command_rpc_timeout", 0.1);
    b2_base::validate_motion_parameters(max_linear_x_, max_reverse_linear_x_, max_linear_y_,
      max_angular_z_, deadband_, command_rate_hz_, cmd_timeout_);
    if (!b2_base::valid_sdk_timeout(command_rpc_timeout) ||
      command_rpc_timeout > cmd_timeout_ || !std::isfinite(state_timeout_) || state_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
          "command_rpc_timeout must be SDK-representable and <= cmd_timeout; "
          "state_timeout must be finite and positive");
    }

    // SportClient creates DDS channels in its constructor, so construct it
    // only after ChannelFactory has selected the B2 network interface.
    sport_client_ = std::make_unique<unitree::robot::b2::SportClient>();
    sport_client_->SetTimeout(static_cast<float>(command_rpc_timeout));
    sport_client_->Init();
    mode_client_ = std::make_unique<unitree::robot::b2::SportClient>();
    mode_client_->SetTimeout(10.0F);
    mode_client_->Init();
    motion_configuration_ = std::make_unique<b2_base::MotionConfiguration>(*this);
    driver_status_ = std::make_unique<b2_base::DriverStatus>(*this, state_timeout_, base_frame_,
        control_enabled_);
    low_state_messages_ = std::make_unique<b2_base::LowStateMessages>(base_frame_);
    sport_messages_ = std::make_unique<b2_base::SportStateMessages>(
      odom_frame_, base_frame_, imu_frame_);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    // Let overlapping requests run far enough to reject them immediately.
    // A mutually-exclusive callback queue would replay old settings after stop.
    mode_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    stop_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_,
        sensor_qos);
    foot_force_pub_ =
      create_publisher<std_msgs::msg::Int16MultiArray>(foot_force_topic_, sensor_qos);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::QoS(1).reliable().transient_local());
    mode_pub_ = create_publisher<std_msgs::msg::Int32>(mode_topic_, sensor_qos);
    comm_ok_pub_ = create_publisher<std_msgs::msg::Bool>(
      comm_ok_topic_, rclcpp::QoS(1).reliable().transient_local());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr command) {
        const bool connected = driver_status_->readiness().motion_ready;
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!control_enabled_ || !connected || mode_busy_ || stop_pending_ ||
          rest_motion_blocked_) {return;}
        last_command_ = b2_base::finite_twist(*command) ? *command : geometry_msgs::msg::Twist{};
        last_command_time_ = std::chrono::steady_clock::now();
        have_command_ = true;
      });

    motion_mode_service_ = create_service<b2_interface::srv::SetMotionMode>(
      "/b2/motion/set_mode",
      std::bind(&B2SdkDriver::set_motion_mode, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), mode_group_);
    motion_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/b2/motion/stop",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!control_enabled_) {
          response->success = false; response->message = "CONTROL_DISABLED"; return;
        }
        std::uint64_t generation;
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          have_command_ = false;
          stop_pending_ = true;
          generation = ++stop_generation_;
          if (configuration_active_ && configuration_abort_reason_.empty()) {
            configuration_abort_reason_ = "CONFIGURATION_INTERRUPTED_BY_STOP";
          }
          last_command_ = geometry_msgs::msg::Twist{};
        }
        std::lock_guard<std::mutex> motion_lock(motion_mutex_);
        const int32_t result = sport_client_->StopMove();
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          if (stop_generation_ == generation) {stop_pending_ = result != 0;}
        }
        response->success = result == 0;
        response->message = "Unitree SDK2 StopMove result=" + std::to_string(result);
        if (result != 0) {
          RCLCPP_WARN(get_logger(), "StopMove failed with Unitree SDK2 result %d.", result);
        }
      }, rclcpp::ServicesQoS(), stop_group_);

    height_service_ = create_service<b2_interface::srv::SetBodyHeight>(
      "/b2/motion/set_body_height", [this](
        const b2_interface::srv::SetBodyHeight::Request::SharedPtr request,
        b2_interface::srv::SetBodyHeight::Response::SharedPtr response) {
        configure_motion(*request, *response, "BodyHeight", [this, request]() {
          return mode_client_->BodyHeight(static_cast<float>(request->height));
        });
      }, rclcpp::ServicesQoS(), mode_group_);
    pose_service_ = create_service<b2_interface::srv::SetBodyPose>(
      "/b2/motion/set_body_pose", [this](
        const b2_interface::srv::SetBodyPose::Request::SharedPtr request,
        b2_interface::srv::SetBodyPose::Response::SharedPtr response) {
        configure_motion(*request, *response, "Euler", [this, request]() {
          return mode_client_->Euler(static_cast<float>(request->roll),
            static_cast<float>(request->pitch), static_cast<float>(request->yaw));
        });
      }, rclcpp::ServicesQoS(), mode_group_);
    gait_service_ = create_service<b2_interface::srv::ConfigureGait>(
      "/b2/motion/configure_gait", [this](
        const b2_interface::srv::ConfigureGait::Request::SharedPtr request,
        b2_interface::srv::ConfigureGait::Response::SharedPtr response) {
        configure_motion(*request, *response, "ConfigureGait", [this, request]() -> int32_t {
          using Request = b2_interface::srv::ConfigureGait::Request;
          switch (request->feature) {
            case Request::SWITCH_GAIT: return mode_client_->SwitchGait(request->gait_id);
            case Request::CONTINUOUS_GAIT: return mode_client_->ContinuousGait(request->enabled);
            case Request::FAST_WALK: return mode_client_->FastWalk(request->enabled);
            case Request::AUTO_RECOVERY: return mode_client_->AutoRecoverySet(request->enabled);
            default: return -1;
          }
        });
      }, rclcpp::ServicesQoS(), mode_group_);
    height_mode_service_ = create_service<b2_interface::srv::ConfigureHeightMode>(
      "/b2/motion/configure_height_mode", [this](
        const b2_interface::srv::ConfigureHeightMode::Request::SharedPtr request,
        b2_interface::srv::ConfigureHeightMode::Response::SharedPtr response) {
        configure_motion(*request, *response, "ConfigureHeightMode", [this, request]() -> int32_t {
          using Request = b2_interface::srv::ConfigureHeightMode::Request;
          switch (request->feature) {
            case Request::FREE_HEIGHT: return mode_client_->FreeHeight(request->enabled);
            case Request::GAIT_HEIGHT: return mode_client_->GaitHeight(request->enabled);
            default: return -1;
          }
        });
      }, rclcpp::ServicesQoS(), mode_group_);

    sport_state_sub_ = std::make_shared<SportSubscriber>(kSportStateTopic);
    sport_state_sub_->InitChannel([this](const void * message) {on_sport_state(message);});
    low_state_sub_ = std::make_shared<LowSubscriber>(kLowStateTopic);
    low_state_sub_->InitChannel([this](const void * message) {on_low_state(message);});

    const auto command_period = std::chrono::duration<double>(1.0 / std::max(1.0,
        command_rate_hz_));
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(command_period),
      std::bind(&B2SdkDriver::send_velocity_command, this), command_group_);
    // No SDK call is made from the fast callback group: a missing RPC response
    // must not delay command expiry, state publication or ROS subscriptions.
    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(20),
      std::bind(&B2SdkDriver::watch_command, this));
    state_timer_ = create_wall_timer(std::chrono::milliseconds(33),
        std::bind(&B2SdkDriver::publish_sport_state, this));
    low_state_timer_ = create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&B2SdkDriver::publish_low_state, this));
    health_timer_ = create_wall_timer(std::chrono::milliseconds(200),
        std::bind(&B2SdkDriver::publish_health, this));

    std_msgs::msg::Bool disconnected;
    disconnected.data = false;
    comm_ok_pub_->publish(disconnected);

    if (auto_balance_stand_ && control_enabled_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
            auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
            request->mode = b2_interface::srv::SetMotionMode::Request::BALANCE_STAND;
            set_motion_mode(request,
            std::make_shared<b2_interface::srv::SetMotionMode::Response>());
            startup_timer_->cancel();
      }, mode_group_);
    }

    sdk_executors_ = std::make_unique<b2_base::SdkCallbackExecutors>(
      get_node_base_interface(), command_group_, mode_group_, stop_group_);
    RCLCPP_INFO(
      get_logger(), "SDK2 direct B2 driver ready: DDS domain=%d, interface=%s, cmd_vel=%s.",
      static_cast<int>(domain), interface.c_str(), cmd_vel_topic_.c_str());
  }

  ~B2SdkDriver() override
  {
    sdk_executors_.reset();
    // SDK listeners own their own threads. Close them while every callback
    // dependency (status publisher, source clock and state storage) still exists.
    if (sport_state_sub_) sport_state_sub_->CloseChannel();
    if (low_state_sub_) low_state_sub_->CloseChannel();
  }

private:
  using SportState = unitree_go::msg::dds_::SportModeState_;
  using LowState = unitree_go::msg::dds_::LowState_;
  using SportSubscriber = unitree::robot::ChannelSubscriber<SportState>;
  using LowSubscriber = unitree::robot::ChannelSubscriber<LowState>;

  template<class Request, class Response, class Call>
  void configure_motion(
    const Request & request, Response & response,
    const std::string & operation, Call call)
  {
    response.success = false;
    response.result_code = -1;
    if (!control_enabled_) {response.message = "CONTROL_DISABLED"; return;}
    response.message = motion_configuration_->validate(request);
    if (!response.message.empty()) {return;}
    std::unique_lock<std::mutex> operation_lock(mode_operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {response.message = "MOTION_OPERATION_BUSY"; return;}
    const auto readiness = driver_status_->readiness();
    if (!readiness.motion_ready) {response.message = readiness.reason; return;}
    std::uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      mode_busy_ = true;
      configuration_active_ = true;
      configuration_abort_reason_.clear();
      have_command_ = false;
      last_command_ = geometry_msgs::msg::Twist{};
      stop_pending_ = true;
      generation = ++stop_generation_;
    }
    try {
      // Acknowledge StopMove before applying a setting. Other explicit StopMove
      // requests keep their own short-timeout client and callback group.
      {
        std::lock_guard<std::mutex> motion_lock(motion_mutex_);
        response.result_code = sport_client_->StopMove();
      }
      bool cancelled;
      std::string cancellation_reason;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cancelled = stop_generation_ != generation;
        cancellation_reason = configuration_abort_reason_.empty() ?
          "CONFIGURATION_CANCELLED_BY_STOP" : configuration_abort_reason_;
        if (!cancelled && response.result_code == 0) {stop_pending_ = false;}
      }
      const auto current = driver_status_->readiness();
      if (cancelled) {
        response.result_code = -1; response.message = cancellation_reason;
      } else if (response.result_code != 0) {
        response.message = "STOP_BEFORE_CONFIGURATION_FAILED";
      } else if (!current.motion_ready) {
        response.result_code = -1; response.message = current.reason;
      } else {
        const auto acknowledged_at = std::chrono::steady_clock::now();
        const auto deadline = acknowledged_at +
          std::chrono::duration<double>(motion_configuration_->stop_timeout);
        // SDK acknowledgement is insufficient: require three NEW stationary
        // telemetry packets. This wait occupies only the configuration group.
        while (!driver_status_->stationary_after(acknowledged_at) &&
          driver_status_->readiness().motion_ready && std::chrono::steady_clock::now() < deadline)
        {
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (stop_generation_ != generation) {break;}
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          cancelled = stop_generation_ != generation;
          cancellation_reason = configuration_abort_reason_.empty() ?
            "CONFIGURATION_CANCELLED_BY_STOP" : configuration_abort_reason_;
        }
        const auto settled = driver_status_->readiness();
        if (cancelled) {
          response.result_code = -1; response.message = cancellation_reason;
        } else if (!settled.motion_ready) {
          response.result_code = -1; response.message = settled.reason;
        } else if (!driver_status_->stationary_after(acknowledged_at)) {
          response.result_code = -1; response.message = "STATIONARY_TELEMETRY_NOT_CONFIRMED";
        } else {
          response.result_code = call();
          response.success = response.result_code == 0;
          response.message = "Unitree SDK2 " + operation + " result=" +
            std::to_string(response.result_code) +
            "; SDK acknowledgement, physical completion is not confirmed";
          const auto after_call = driver_status_->readiness();
          std::lock_guard<std::mutex> lock(command_mutex_);
          if (!after_call.motion_ready && configuration_abort_reason_.empty()) {
            configuration_abort_reason_ = std::string("CONFIGURATION_ABORTED: ") +
              after_call.reason;
            stop_pending_ = true;
            ++stop_generation_;
          }
          if (stop_generation_ != generation) {
            response.success = false;
            response.message = (configuration_abort_reason_.empty() ?
              "CONFIGURATION_INTERRUPTED_BY_STOP" : configuration_abort_reason_) + "; SDK result=" +
              std::to_string(response.result_code) + "; physical outcome is not confirmed";
          }
        }
      }
    } catch (const std::exception & error) {
      response.success = false;
      response.result_code = -1;
      response.message = error.what();
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    configuration_active_ = false;
    mode_busy_ = false;
    have_command_ = false;
  }

  void set_motion_mode(
    const b2_interface::srv::SetMotionMode::Request::SharedPtr request,
    b2_interface::srv::SetMotionMode::Response::SharedPtr response)
  {
    if (!control_enabled_) {
      response->success = false; response->result_code = -1;
      response->message = "CONTROL_DISABLED"; return;
    }
    if (request->mode > b2_interface::srv::SetMotionMode::Request::VISION_WALK_OFF) {
      response->success = false;
      response->result_code = -1;
      response->message = "Unsupported B2 motion mode: " + std::to_string(request->mode);
      return;
    }
    std::unique_lock<std::mutex> operation_lock(mode_operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->success = false;
      response->result_code = -1;
      response->message = "MOTION_OPERATION_BUSY";
      return;
    }
    using Mode = b2_interface::srv::SetMotionMode::Request;
    const bool resting = request->mode == Mode::STAND_DOWN || request->mode == Mode::DAMP;
    std::uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      mode_busy_ = true;
      have_command_ = false;
      stop_pending_ = true;
      generation = ++stop_generation_;
      last_command_ = geometry_msgs::msg::Twist{};
      if (resting) rest_motion_blocked_ = true;
    }
    if (resting) driver_status_->suspend_velocity(true);
    // Block new movement before waiting for the bounded in-flight Move RPC.
    // Do not hold this mutex during the long mode transition itself.
    {std::lock_guard<std::mutex> motion_lock(motion_mutex_);}
    try {
      if (!resting || prepare_rest(request, response, generation)) {
        call_motion_mode(request, response);
      }
    } catch (const std::exception & error) {
      response->success = false;
      response->result_code = -1;
      response->message = error.what();
    }
    const bool resume = response->success && (request->mode == Mode::STAND_UP ||
      request->mode == Mode::BALANCE_STAND || request->mode == Mode::RECOVERY_STAND ||
      request->mode == Mode::FREE_WALK || request->mode == Mode::CLASSIC_WALK_ON ||
      request->mode == Mode::VISION_WALK_ON);
    if (resume) driver_status_->suspend_velocity(false);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (resume) rest_motion_blocked_ = false;
      if (resting && !response->success) {stop_pending_ = true; ++stop_generation_;}
      mode_busy_ = false;
      have_command_ = false;
    }
  }

  bool prepare_rest(
    const b2_interface::srv::SetMotionMode::Request::SharedPtr &request,
    const b2_interface::srv::SetMotionMode::Response::SharedPtr &response,
    std::uint64_t generation)
  {
    using Mode = b2_interface::srv::SetMotionMode::Request;
    const bool stand_down = request->mode == Mode::STAND_DOWN;
    auto acknowledged = std::chrono::steady_clock::now();
    {
      // Drain a command RPC and clear our pending stop while still holding the
      // same lock as the command timer. It must not capture an ordinary stop
      // here and deliver it after StandDown/Damp has started.
      std::lock_guard<std::mutex> motion_lock(motion_mutex_);
      if (stand_down) {
        response->result_code = sport_client_->StopMove();
        acknowledged = std::chrono::steady_clock::now();
        if (response->result_code != 0) {
          response->success = false; response->message = "STAND_DOWN_STOP_NOT_ACKNOWLEDGED";
          return false;
        }
      }
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (stand_down && stop_generation_ != generation) {
        response->success = false; response->result_code = -1;
        response->message = "STAND_DOWN_INTERRUPTED_BY_STOP"; return false;
      }
      // Do not send a delayed ordinary StopMove after StandDown/Damp starts.
      // A newer explicit stop retains its own generation and remains pending.
      if (stop_generation_ == generation) stop_pending_ = false;
    }
    if (!stand_down) return true;  // Damp remains available without healthy state.
    const auto deadline = acknowledged +
      std::chrono::duration<double>(motion_configuration_->stop_timeout);
    while (!driver_status_->stationary_after(acknowledged) &&
      std::chrono::steady_clock::now() < deadline && rclcpp::ok(get_node_base_interface()->get_context()))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (stop_generation_ != generation) {
        response->success = false; response->result_code = -1;
        response->message = "STAND_DOWN_INTERRUPTED_BY_STOP"; return false;
      }
    }
    if (!driver_status_->stationary_after(acknowledged)) {
      response->success = false; response->result_code = -1;
      response->message = "STAND_DOWN_REQUIRES_FRESH_MEASURED_STOP"; return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (stop_generation_ != generation) {
        response->success = false; response->result_code = -1;
        response->message = "STAND_DOWN_INTERRUPTED_BY_STOP"; return false;
      }
    }
    return true;
  }

  void call_motion_mode(
    const b2_interface::srv::SetMotionMode::Request::SharedPtr request,
    b2_interface::srv::SetMotionMode::Response::SharedPtr response)
  {
    using Request = b2_interface::srv::SetMotionMode::Request;

    int32_t result = -1;
    std::string mode_name;
    switch (request->mode) {
      case Request::DAMP: mode_name = "damp"; result = mode_client_->Damp(); break;
      case Request::BALANCE_STAND: mode_name = "balance_stand";
        result = mode_client_->BalanceStand(); break;
      case Request::STAND_UP: mode_name = "stand_up"; result = mode_client_->StandUp(); break;
      case Request::STAND_DOWN: mode_name = "stand_down"; result = mode_client_->StandDown(); break;
      case Request::RECOVERY_STAND: mode_name = "recovery_stand";
        result = mode_client_->RecoveryStand(); break;
      case Request::FREE_WALK: mode_name = "free_walk"; result = mode_client_->FreeWalk(); break;
      case Request::CLASSIC_WALK_ON: mode_name = "classic_walk_on";
        result = mode_client_->ClassicWalk(true); break;
      case Request::CLASSIC_WALK_OFF: mode_name = "classic_walk_off";
        result = mode_client_->ClassicWalk(false); break;
      case Request::SPEED_LOW: mode_name = "speed_low"; result = mode_client_->SpeedLevel(-1);
        break;
      case Request::SPEED_HIGH: mode_name = "speed_high"; result = mode_client_->SpeedLevel(1);
        break;
      case Request::VISION_WALK_ON: mode_name = "vision_walk_on";
        result = mode_client_->VisionWalk(true); break;
      case Request::VISION_WALK_OFF: mode_name = "vision_walk_off";
        result = mode_client_->VisionWalk(false); break;
      default:
        response->success = false;
        response->result_code = -1;
        response->message = "Unsupported B2 motion mode: " + std::to_string(request->mode);
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }
    response->success = result == 0;
    response->result_code = result;
    response->message = "Unitree SDK2 " + mode_name + " result=" + std::to_string(result);
    if (result != 0) {
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  void watch_command()
  {
    const auto readiness = driver_status_->readiness();
    const bool connected = readiness.motion_ready;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (configuration_active_ && !connected && configuration_abort_reason_.empty()) {
      configuration_abort_reason_ = std::string("CONFIGURATION_ABORTED: ") + readiness.reason;
      stop_pending_ = true;
      ++stop_generation_;
    }
    if (!have_command_) {return;}
    const bool expired = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_time_).count() > cmd_timeout_;
    if (!connected || expired || mode_busy_) {
      have_command_ = false;
      last_command_ = geometry_msgs::msg::Twist{};
      if (stop_on_timeout_ || !connected || mode_busy_) {stop_pending_ = true; ++stop_generation_;}
    }
  }

  void send_velocity_command()
  {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    watch_command();
    geometry_msgs::msg::Twist command;
    bool have_command = false;
    bool stop = false;
    std::uint64_t stop_generation = 0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      have_command = have_command_;
      command = last_command_;
      stop = stop_pending_;
      stop_generation = stop_generation_;
    }
    if (stop) {
      const auto result = sport_client_->StopMove();
      if (result == 0) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (stop_generation == stop_generation_) {stop_pending_ = false;}
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "StopMove failed (result=%d); movement remains blocked.", result);
      }
      return;
    }
    if (!have_command) {return;}
    const float vx = static_cast<float>(apply_deadband(b2_base::clamp_forward_speed(
      command.linear.x, max_linear_x_, max_reverse_linear_x_), deadband_));
    const float vy = static_cast<float>(apply_deadband(clamp_abs(command.linear.y, max_linear_y_),
        deadband_));
    const float vyaw = static_cast<float>(apply_deadband(clamp_abs(command.angular.z,
        max_angular_z_), deadband_));
    const auto result = sport_client_->Move(vx, vy, vyaw);
    if (result != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Unitree SDK2 Move failed (result=%d).", result);
    }
  }

  void on_sport_state(const void * message)
  {
    std::lock_guard<std::mutex> lock(sport_state_mutex_);
    latest_sport_state_ = *static_cast<const SportState *>(message);
    sport_state_received_at_ = std::chrono::steady_clock::now();
    const auto received = now().nanoseconds();
    const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
      sport_state_received_at_.time_since_epoch()).count();
    const auto & source = latest_sport_state_.stamp();
    const auto timing = sport_clock_->update(source.sec(), source.nanosec(), received, steady);
    driver_status_->observe_sport(latest_sport_state_, timing);
    if (!timing.valid) {
      sport_state_ready_.store(false, std::memory_order_release);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "B2 source timestamp rejected: %s. Verify robot/host clock synchronization.",
          timing.reason);
      return;
    }
    sport_state_stamp_ = rclcpp::Time(timing.stamp_ns);
    sensor_msgs::msg::TimeReference reference;
    reference.header.stamp = sport_state_stamp_; reference.header.frame_id = base_frame_;
    reference.time_ref.sec = source.sec(); reference.time_ref.nanosec = source.nanosec();
    reference.source = timing.reason;
    time_reference_pub_->publish(reference);
    ++sport_generation_;
    sport_state_ready_.store(true, std::memory_order_release);
  }

  void on_low_state(const void * message)
  {
    std::lock_guard<std::mutex> lock(low_state_mutex_);
    latest_low_state_ = *static_cast<const LowState *>(message);
    driver_status_->observe_low(latest_low_state_);
    low_state_received_at_ = std::chrono::steady_clock::now();
    low_state_stamp_ = now();
    ++low_generation_;
    low_state_ready_.store(true, std::memory_order_release);
  }

  void publish_sport_state()
  {
    if (!sport_state_ready_.load(std::memory_order_acquire)) {
      return;
    }
    SportState state;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(sport_state_mutex_);
      if (sport_generation_ == published_sport_generation_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
          sport_state_received_at_).count() > state_timeout_)
      {
        return;
      }
      state = latest_sport_state_;
      stamp = sport_state_stamp_;
      published_sport_generation_ = sport_generation_;
    }
    sport_messages_->update(state, stamp, static_cast<bool>(tf_broadcaster_));
    imu_pub_->publish(sport_messages_->imu);
    odom_pub_->publish(sport_messages_->odom);
    if (tf_broadcaster_) tf_broadcaster_->sendTransform(sport_messages_->transform);
    mode_pub_->publish(sport_messages_->mode);
    driver_status_->publish_sport(state, stamp);
  }

  void publish_low_state()
  {
    if (!low_state_ready_.load(std::memory_order_acquire)) {
      return;
    }
    LowState state;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(low_state_mutex_);
      if (low_generation_ == published_low_generation_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
          low_state_received_at_).count() > state_timeout_)
      {
        return;
      }
      state = latest_low_state_;
      stamp = low_state_stamp_;
      published_low_generation_ = low_generation_;
    }

    low_state_messages_->update(state, stamp);
    joint_state_pub_->publish(low_state_messages_->joints);
    driver_status_->publish_motors(low_state_messages_->motors);
    foot_force_pub_->publish(low_state_messages_->foot_force);
    battery_pub_->publish(low_state_messages_->battery);
  }

  void publish_health()
  {
    bool healthy = false;
    if (sport_state_ready_.load(std::memory_order_acquire)) {
      std::lock_guard<std::mutex> lock(sport_state_mutex_);
      healthy = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - sport_state_received_at_).count() <= state_timeout_;
    }
    std_msgs::msg::Bool message;
    message.data = healthy;
    comm_ok_pub_->publish(message);
  }

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string joint_state_topic_;
  std::string battery_topic_;
  std::string foot_force_topic_;
  std::string mode_topic_;
  std::string comm_ok_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_tf_{true};
  bool control_enabled_{true};
  double max_linear_x_{1.0};
  double max_reverse_linear_x_{0.8};
  double max_linear_y_{0.4};
  double max_angular_z_{1.2};
  double deadband_{0.01};
  double command_rate_hz_{20.0};
  double cmd_timeout_{0.5};
  bool stop_on_timeout_{true};
  bool auto_balance_stand_{false};
  double state_timeout_{0.5};
  bool have_command_{false};
  bool mode_busy_{false};
  bool rest_motion_blocked_{false};
  bool configuration_active_{false};
  std::string configuration_abort_reason_;
  bool stop_pending_{false};
  std::uint64_t stop_generation_{0};
  geometry_msgs::msg::Twist last_command_;
  std::chrono::steady_clock::time_point last_command_time_{};
  std::chrono::steady_clock::time_point sport_state_received_at_{};
  std::chrono::steady_clock::time_point low_state_received_at_{};
  builtin_interfaces::msg::Time sport_state_stamp_, low_state_stamp_;
  std::unique_ptr<b2_base::MeasurementClock> sport_clock_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_reference_pub_;
  std::uint64_t sport_generation_{0}, published_sport_generation_{0};
  std::uint64_t low_generation_{0}, published_low_generation_{0};
  std::mutex motion_mutex_;
  std::mutex mode_operation_mutex_;
  std::mutex command_mutex_;
  std::mutex sport_state_mutex_;
  std::mutex low_state_mutex_;
  std::atomic<bool> sport_state_ready_{false};
  std::atomic<bool> low_state_ready_{false};
  SportState latest_sport_state_;
  LowState latest_low_state_;

  std::unique_ptr<unitree::robot::b2::SportClient> sport_client_;
  std::unique_ptr<unitree::robot::b2::SportClient> mode_client_;
  std::unique_ptr<b2_base::MotionConfiguration> motion_configuration_;
  std::unique_ptr<b2_base::DriverStatus> driver_status_;
  std::unique_ptr<b2_base::LowStateMessages> low_state_messages_;
  std::unique_ptr<b2_base::SportStateMessages> sport_messages_;
  rclcpp::CallbackGroup::SharedPtr command_group_, mode_group_, stop_group_;
  std::shared_ptr<SportSubscriber> sport_state_sub_;
  std::shared_ptr<LowSubscriber> low_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr foot_force_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr comm_ok_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr low_state_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Service<b2_interface::srv::SetMotionMode>::SharedPtr motion_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motion_stop_service_;
  rclcpp::Service<b2_interface::srv::SetBodyHeight>::SharedPtr height_service_;
  rclcpp::Service<b2_interface::srv::SetBodyPose>::SharedPtr pose_service_;
  rclcpp::Service<b2_interface::srv::ConfigureGait>::SharedPtr gait_service_;
  rclcpp::Service<b2_interface::srv::ConfigureHeightMode>::SharedPtr height_mode_service_;
  std::unique_ptr<b2_base::SdkCallbackExecutors> sdk_executors_;
};
}  // namespace

#ifndef B2_SDK_DRIVER_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto driver = std::make_shared<B2SdkDriver>();
    // Slow actuator RPCs have dedicated executors. Two ROS workers retain one
    // fast state/watchdog lane even while read-only metadata polling waits.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(driver);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("b2_sdk_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
