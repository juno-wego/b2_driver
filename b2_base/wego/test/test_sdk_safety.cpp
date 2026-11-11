// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

// Exercise the real driver and SDK protocol on loopback, with no robot.
#define B2_SDK_DRIVER_NO_MAIN
// Include the local node implementation to exercise its real SDK calls.
#include "../b2_sdk_driver.cpp"  // NOLINT(build/include)
#include <gtest/gtest.h>
#include <algorithm>
#include <sstream>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/robot/internal/internal_idl_decl/Response_.hpp>
#include <unitree/common/json/json.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
using Request = unitree_api::msg::dds_::Request_;
using Response = unitree_api::msg::dds_::Response_;
using namespace std::chrono_literals;

class FakeRobot {
public:
  FakeRobot()
  : responses_("rt/api/sport/response"), requests_("rt/api/sport/request"),
    switcher_responses_("rt/api/motion_switcher/response"),
    switcher_requests_("rt/api/motion_switcher/request"),
    state_responses_("rt/api/robot_state/response"), state_requests_("rt/api/robot_state/request"),
    sport_("rt/sportmodestate"), low_("rt/lowstate")
  {
    unitree::robot::ChannelFactory::Instance()->Init(155, "lo");
    responses_.InitChannel(); switcher_responses_.InitChannel(); state_responses_.InitChannel();
    sport_.InitChannel(); low_.InitChannel();
    requests_.InitChannel([this](const void * raw) {
        const auto & request = *static_cast<const Request *>(raw);
        const auto api = request.header().identity().api_id();
      // SDK clients send version/no-op handshakes even in read-only mode.
      // Count every other API, including lease requests, as control traffic.
        if (api != 1 && api != 2) {++sport_requests;} else {++internal_requests;}
        if (api == 1008) {
          ++moves;
          const auto payload = unitree::common::AnyCast<unitree::common::JsonMap>(
          unitree::common::FromJsonString(request.parameter()));
          last_move_x = unitree::common::AnyNumberCast<float>(payload.at("x"));
          if (drop_move_reply) {return;}
        }
        if (api == 1003) {++stops;}
        if (api == 1005) {++stand_downs;}
        if (api == 1001) {++damps;}
        if (api == 1101) {++modes;}
        if (api == 1013 || api == 1051 || api == 1011 || api == 1019 || api == 1050 ||
        api == 1040 || api == 1052 || api == 1053)
        {
          ++configurations;
          std::lock_guard<std::mutex> lock(mutex_);
          parameters_[api] = request.parameter();
        }
        Response response;
        response.header().identity() = request.header().identity();
        response.header().status().code() = api == 1013 ? configuration_result.load() :
        api == 1052 || api == 1053 ? height_mode_result.load() :
        api == 1003 ? stop_result.load() : 0;
        const auto due = Clock::now() + (api == 1101 || (api == 1051 && slow_pose) ? 1500ms : 0ms);
        std::lock_guard<std::mutex> lock(mutex_);
        replies_.push_back({due, response, 0});
    }, 32);
    switcher_requests_.InitChannel([this](const void * raw) {
        if (drop_controller_reply) {return;}
        const auto & request = *static_cast<const Request *>(raw);
        Response response;
        response.header().identity() = request.header().identity();
        response.header().status().code() = 0;
        response.data() = controller_active ? "{\"form\":\"0\",\"name\":\"normal\"}" :
        "{\"form\":\"0\",\"name\":\"\"}";
        std::lock_guard<std::mutex> lock(mutex_);
        replies_.push_back({Clock::now(), response, 1});
    }, 32);
    state_requests_.InitChannel([this](const void * raw) {
        if (drop_metadata_reply) {return;}
        const auto & request = *static_cast<const Request *>(raw);
        Response response;
        response.header().identity() = request.header().identity();
        response.header().status().code() = 0;
        std::ostringstream payload;
        if (request.header().identity().api_id() == 1003) {
          payload << '[';
          for (int i = 0; i < metadata_count; ++i) {
            if (i) {payload << ',';}
            payload << "{\"name\":\"" << (i ? "fixture_service_" + std::to_string(i) : "sport_mode")
                    << "\",\"status\":1,\"protect\":0}";
          }
          payload << ']';
        } else {
          payload << "{\"packageVersion\":\"fixture-1.0\",\"moduleVersionMap\":{";
          for (int i = 0; i < metadata_count; ++i) {
            if (i) {payload << ',';}
            payload << '"' << (i ? "fixture_module_" + std::to_string(i) : "sport")
                    << "\":\"fixture-2.0\"";
          }
          payload << "}}";
        }
        response.data() = payload.str();
        std::lock_guard<std::mutex> lock(mutex_);
        replies_.push_back({Clock::now(), response, 2});
    }, 32);
    // The real robot publishes native SDK state independently of this PC's ROS
    // executor. Do not let ROS service scheduling pause the simulated sensors.
    worker_ = std::thread([this]() {
          while (running_) {tick(); std::this_thread::sleep_for(10ms);}
    });
  }
  ~FakeRobot()
  {
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
    requests_.CloseChannel(); switcher_requests_.CloseChannel(); state_requests_.CloseChannel();
  }
  std::string parameter(int api)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return parameters_[api];
  }
  void tick()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto item = replies_.begin(); item != replies_.end(); ) {
        if (item->due <= Clock::now()) {
          if (item->service == 0) {responses_.Write(item->response);} else if (item->service == 1) {
            switcher_responses_.Write(item->response);
          } else {state_responses_.Write(item->response);}
          item = replies_.erase(item);
        } else {++item;}
      }
    }
    if (publish_state) {
      unitree_go::msg::dds_::SportModeState_ sport;
      if (publish_timestamp) {
        const auto source = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count() + stamp_offset_ns;
        sport.stamp().sec() = static_cast<int32_t>(source / 1000000000LL);
        sport.stamp().nanosec() = static_cast<uint32_t>(source % 1000000000LL);
      }
      sport.imu_state().quaternion()[0] = .70710678F;
      sport.imu_state().quaternion()[3] = .70710678F;
      sport.velocity()[1] = stationary ? 0.F : .1F;
      sport.error_code() = sport_error;
      sport.body_height() = .51F;
      sport.foot_raise_height() = .08F;
      sport.gait_type() = 2;
      sport_.Write(sport);
      unitree_go::msg::dds_::LowState_ low;
      low.bms_state().current() = bms_current;
      for (std::size_t i = 0; i < low.motor_state().size(); ++i) {
        low.motor_state()[i].temperature() = 30 + i;
        low.motor_state()[i].mode() = 10;
        low.motor_state()[i].lost() = i;
      }
      if (publish_low) {low_.Write(low);}
    }
  }
  std::atomic<int> moves{0}, stops{0}, modes{0}, configurations{0};
  std::atomic<int> stand_downs{0}, damps{0};
  std::atomic<int> sport_requests{0};
  std::atomic<int> internal_requests{0};
  std::atomic<float> last_move_x{0.F};
  std::atomic<bool> publish_state{true}, drop_move_reply{false};
  std::atomic<bool> publish_timestamp{true};
  std::atomic<int64_t> stamp_offset_ns{0};
  std::atomic<int32_t> bms_current{2000};
  std::atomic<uint32_t> sport_error{0};
  std::atomic<bool> controller_active{true}, drop_controller_reply{false}, publish_low{true},
  slow_pose{false};
  std::atomic<int32_t> configuration_result{0};
  std::atomic<int32_t> height_mode_result{0};
  std::atomic<int32_t> stop_result{0};
  std::atomic<bool> stationary{false};
  std::atomic<bool> drop_metadata_reply{false};
  std::atomic<int> metadata_count{1};

private:
  unitree::robot::ChannelPublisher<Response> responses_;
  unitree::robot::ChannelSubscriber<Request> requests_;
  unitree::robot::ChannelPublisher<Response> switcher_responses_;
  unitree::robot::ChannelSubscriber<Request> switcher_requests_;
  unitree::robot::ChannelPublisher<Response> state_responses_;
  unitree::robot::ChannelSubscriber<Request> state_requests_;
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::SportModeState_> sport_;
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_> low_;
  std::mutex mutex_;
  struct Reply {Clock::time_point due; Response response; int service;};
  std::vector<Reply> replies_;
  std::map<int, std::string> parameters_;
  std::atomic<bool> running_{true};
  std::thread worker_;
};

struct RunningExecutor
{
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 4};
  std::thread worker;
  void start() {worker = std::thread([this]() {executor.spin();});}
  ~RunningExecutor() {executor.cancel(); if (worker.joinable()) {worker.join();}}
};

template<class Predicate> bool until(Predicate predicate, std::chrono::milliseconds timeout = 3s)
{
  const auto end = Clock::now() + timeout;
  while (!predicate() && Clock::now() < end) {std::this_thread::sleep_for(10ms);}
  return predicate();
}

TEST(B2SdkSafety, OriginalClockIsRequiredAndWrongEpochCannotKeepMovementReady) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  peer.publish_timestamp = false;
  auto observer = std::make_shared<rclcpp::Node>("b2_source_time_test");
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"cmd_vel_topic", "/test/b2_time/cmd_vel"}, {"publish_tf", false},
      {"controller.require_ready", false}, {"command_rpc_timeout", .05}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  std::atomic<bool> ready{false}, invalid{false}, original_preserved{false};
  auto status = observer->create_subscription<b2_interface::msg::DriverStatus>("/b2/driver_status",
    rclcpp::QoS(1).reliable().transient_local(),
      [&](const b2_interface::msg::DriverStatus & value) {
        ready = value.motion_ready; invalid = value.reason == "SPORT_TIME_INVALID";
    });
  auto reference = observer->create_subscription<sensor_msgs::msg::TimeReference>(
    "/b2/time_reference", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::TimeReference & value) {
      original_preserved = value.header.stamp == value.time_ref &&
      value.source == "SOURCE_TIME_ACCEPTED";
    });
  auto command = observer->create_publisher<geometry_msgs::msg::Twist>("/test/b2_time/cmd_vel", 10);
  RunningExecutor running;
  running.executor.add_node(driver); running.executor.add_node(observer); running.start();
  ASSERT_TRUE(until([&]() {return invalid.load();}));
  EXPECT_FALSE(ready.load()); EXPECT_EQ(peer.moves.load(), 0);
  peer.publish_timestamp = true; peer.stamp_offset_ns = -20000000;
  ASSERT_TRUE(until([&]() {return ready.load() && original_preserved.load();}));
  geometry_msgs::msg::Twist move; move.linear.x = .1;
  ASSERT_TRUE(until([&]() {command->publish(move); return peer.moves.load() > 0;}));
  const int stops = peer.stops;
  peer.stamp_offset_ns = -10000000000LL;
  ASSERT_TRUE(until([&]() {return invalid.load() && peer.stops.load() > stops;}));
  const int moves = peer.moves;
  command->publish(move);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(peer.moves.load(), moves);
  peer.stamp_offset_ns = 0;
  ASSERT_TRUE(until([&]() {return ready.load();}));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(peer.moves.load(), moves);
}

TEST(B2SdkSafety, StopOwnershipCommunicationLossAndSlowRpc) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  auto node = std::make_shared<rclcpp::Node>("b2_sdk_safety_test");
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"cmd_vel_topic", "/test/b2_sdk/cmd_vel"}, {"publish_tf", false},
      {"cmd_timeout", .2}, {"state_timeout", .15}, {"command_rpc_timeout", .05},
      {"controller.poll_period_s", .1}, {"controller.rpc_timeout_s", .05},
      {"controller.max_age_s", .4}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  std::atomic<int> odometry{0}, joints{0}; std::atomic<bool> connected{false};
  std::atomic<double> forward_speed{0.0}, lateral_speed{1.0}, battery_current{0.0};
  std::atomic<int> battery_status{0};
  auto odom = node->create_subscription<nav_msgs::msg::Odometry>("/b2/odom",
    rclcpp::SensorDataQoS(), [&](const nav_msgs::msg::Odometry & value) {
        ++odometry; forward_speed = value.twist.twist.linear.x;
        lateral_speed = value.twist.twist.linear.y;
    });
  auto battery = node->create_subscription<sensor_msgs::msg::BatteryState>("/b2/battery_state",
    rclcpp::QoS(1).reliable().transient_local(), [&](const sensor_msgs::msg::BatteryState & value) {
        battery_current = value.current;
        battery_status = value.present ? value.power_supply_status : 0;
    });
  auto joint = node->create_subscription<sensor_msgs::msg::JointState>("/b2/joint_states",
    rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::JointState &) {++joints;});
  const auto on_health = [&](const std_msgs::msg::Bool & value) {connected = value.data;};
  auto health = node->create_subscription<std_msgs::msg::Bool>("/b2/comm_ok",
    rclcpp::QoS(1).reliable().transient_local(), on_health);
  auto command = node->create_publisher<geometry_msgs::msg::Twist>("/test/b2_sdk/cmd_vel", 10);
  auto stop = node->create_client<std_srvs::srv::Trigger>("/b2/motion/stop");
  auto mode = node->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  RunningExecutor running; running.executor.add_node(driver); running.executor.add_node(node);
      running.start();
  std::atomic<bool> ready{false};
  auto readiness = node->create_subscription<b2_interface::msg::DriverStatus>("/b2/driver_status",
    rclcpp::QoS(1).reliable().transient_local(),
      [&](const b2_interface::msg::DriverStatus & value) {ready = value.motion_ready;});
  const auto ready_for_commands = [&]() {
      return connected && ready && joints > 0 && command->get_subscription_count() > 0;
    };
  ASSERT_TRUE(until(ready_for_commands));
  EXPECT_NEAR(forward_speed.load(), .1, 1e-6);
  EXPECT_NEAR(lateral_speed.load(), 0.0, 1e-6);
  ASSERT_TRUE(
    until([&]() {
      return battery_current == -2.0 &&
             battery_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                                                                                      }));
  peer.bms_current = -3000;
  ASSERT_TRUE(
    until([&]() {
      return battery_current == 3.0 &&
             battery_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
                                                                                   }));
  peer.bms_current = 0;
  ASSERT_TRUE(
    until([&]() {
      return battery_current == 0.0 &&
             battery_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
                                                                                  }));
  ASSERT_TRUE(stop->wait_for_service(1s)); ASSERT_TRUE(mode->wait_for_service(1s));
  auto disabled_height =
    node->create_client<b2_interface::srv::SetBodyHeight>("/b2/motion/set_body_height");
  ASSERT_TRUE(disabled_height->wait_for_service(1s));
  auto disabled_result = disabled_height->async_send_request(
    std::make_shared<b2_interface::srv::SetBodyHeight::Request>());
  ASSERT_EQ(disabled_result.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(disabled_result.get()->success); EXPECT_EQ(peer.configurations, 0);
  auto disabled_height_mode = node->create_client<b2_interface::srv::ConfigureHeightMode>(
    "/b2/motion/configure_height_mode");
  ASSERT_TRUE(disabled_height_mode->wait_for_service(1s));
  auto disabled_mode_result = disabled_height_mode->async_send_request(
    std::make_shared<b2_interface::srv::ConfigureHeightMode::Request>());
  ASSERT_EQ(disabled_mode_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(disabled_mode_result.get()->message, "HEIGHT_MODE_CONFIGURATION_DISABLED");
  EXPECT_EQ(peer.configurations, 0);
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;

  // No new input after StopMove: the cached nonzero command must stay invalid.
  command->publish(velocity); ASSERT_TRUE(
    until([&]() {return peer.moves > 0;}));
  auto stopped = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stopped.wait_for(500ms), std::future_status::ready);
      ASSERT_TRUE(stopped.get()->success);
  const int first_stop_moves = peer.moves;
  std::this_thread::sleep_for(150ms); EXPECT_EQ(peer.moves, first_stop_moves);

  // One terminal zero callback between 20Hz SDK ticks must replace the cached
  // nonzero command even when no further ROS command arrives.
  command->publish(velocity);
  ASSERT_TRUE(
    until([&]() {return peer.last_move_x > .05F;}));
  command->publish(geometry_msgs::msg::Twist{});
  ASSERT_TRUE(
    until([&]() {return peer.last_move_x == 0.F;}));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(peer.last_move_x, 0.F);

  geometry_msgs::msg::Twist invalid;
  for (auto axis : {&invalid.linear.x, &invalid.linear.y, &invalid.linear.z,
      &invalid.angular.x, &invalid.angular.y, &invalid.angular.z})
  {
    command->publish(velocity);
    ASSERT_TRUE(
    until([&]() {return peer.last_move_x > .05F;}));
    invalid = velocity;
    *axis = std::numeric_limits<double>::quiet_NaN();
    command->publish(invalid);
    ASSERT_TRUE(
    until([&]() {return peer.last_move_x == 0.F;}));
  }

  // A long mode RPC must not occupy the command/stop or feedback callbacks.
  auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::VISION_WALK_ON;
  auto switching = mode->async_send_request(request);
  ASSERT_TRUE(
    until([&]() {return peer.modes > 0;}));
  const int feedback_before = odometry;
  auto stop_during_mode =
    stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_during_mode.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(stop_during_mode.get()->success);
  EXPECT_EQ(switching.wait_for(0ms), std::future_status::timeout);
  ASSERT_TRUE(
    until([&]() {return odometry > feedback_before + 2;}));
  ASSERT_EQ(switching.wait_for(3s), std::future_status::ready);
      EXPECT_TRUE(switching.get()->success);

  // Even a lost Move response has a bounded command-channel RPC timeout.
  peer.drop_move_reply = true;
  const int moves_before = peer.moves; command->publish(velocity);
  ASSERT_TRUE(
    until([&]() {return peer.moves > moves_before;}));
  auto stop_during_move =
    stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_during_move.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(stop_during_move.get()->success);
  peer.drop_move_reply = false;
  const int second_stop_moves = peer.moves;
  std::this_thread::sleep_for(150ms); EXPECT_EQ(peer.moves, second_stop_moves);

  // Continue ROS commands while physical telemetry disappears. Stale state
  // must not be relabelled with new timestamps or permit more movement.
  command->publish(velocity); ASSERT_TRUE(
    until([&]() {return peer.moves > second_stop_moves;}));
  const int stops_before = peer.stops; peer.publish_state = false;
  const auto stale_and_stopped = [&]() {
      command->publish(velocity); return !connected && peer.stops > stops_before;
    };
  ASSERT_TRUE(until(stale_and_stopped));
  std::this_thread::sleep_for(200ms);
  const int odom_after = odometry, joints_after = joints, moves_after = peer.moves;
  for (int i = 0; i < 20; ++i) {
    command->publish(velocity); std::this_thread::sleep_for(10ms);
                                                                                              }
  EXPECT_EQ(odometry, odom_after); EXPECT_EQ(joints, joints_after);
      EXPECT_EQ(peer.moves, moves_after);
}

TEST(B2SdkSafety, StatusReadinessAndConfigurationProtocol) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  peer.stationary = true;
  auto node = std::make_shared<rclcpp::Node>("b2_sdk_configuration_test");
  rclcpp::NodeOptions options;
  // Synthetic fixture bounds, deliberately not advertised as B2 limits.
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"cmd_vel_topic", "/test/b2_sdk/config/cmd_vel"}, {"publish_tf", false},
      {"cmd_timeout", .2}, {"state_timeout", .15}, {"command_rpc_timeout", .05},
      {"controller.poll_period_s", .1}, {"controller.rpc_timeout_s", .05},
      {"controller.max_age_s", .4},
      {"body_height.enabled", true}, {"body_height.minimum", -.1}, {"body_height.maximum", .1},
      {"body_pose.enabled", true}, {"body_pose.max_roll_rad", .2},
      {"body_pose.max_pitch_rad", .25}, {"body_pose.max_yaw_rad", .1},
      {"configuration.stop_confirm_timeout_s", .3},
      {"gait_configuration.enabled", true},
      {"height_mode_configuration.enabled", true},
      {"gait_configuration.allowed_ids", std::vector<int64_t>{2, 3}}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  std::mutex status_mutex;
  b2_interface::msg::DriverStatus latest;
  auto status = node->create_subscription<b2_interface::msg::DriverStatus>("/b2/driver_status",
    rclcpp::QoS(1).reliable().transient_local(),
      [&](const b2_interface::msg::DriverStatus & value) {
        std::lock_guard<std::mutex> lock(status_mutex); latest = value;
    });
  auto snapshot = [&]() {std::lock_guard<std::mutex> lock(status_mutex); return latest;};
  std::atomic<bool> sport_valid{false}, motors_valid{false}, sdk_error_visible{false};
  auto sport = node->create_subscription<b2_interface::msg::SportStatus>("/b2/sport_status",
    rclcpp::SensorDataQoS(), [&](const b2_interface::msg::SportStatus & value) {
        sport_valid = value.gait_type == 2 && std::abs(value.body_height - .51) < 1e-6 &&
        std::abs(value.foot_raise_height - .08) < 1e-6 && value.header.stamp.sec != 0;
    });
  auto motors = node->create_subscription<b2_interface::msg::MotorStatusArray>("/b2/motor_status",
    rclcpp::SensorDataQoS(), [&](const b2_interface::msg::MotorStatusArray & value) {
        motors_valid = value.motors.size() == 12 && value.motors[0].joint_name == "FL_hip_joint" &&
        value.motors[0].temperature_c == 33 && value.motors[0].lost == 3 &&
        value.motors[3].joint_name == "FR_hip_joint" && value.motors[3].temperature_c == 30 &&
        value.motors[6].temperature_c == 39 && value.motors[9].temperature_c == 36;
    });
  auto diagnostics =
    node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10,
      [&](const diagnostic_msgs::msg::DiagnosticArray & value) {
        for (const auto & item : value.status) {
          if (item.name == "B2/sport" &&
          item.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
          {
            sdk_error_visible = true;
          }
        }
    });
  auto command = node->create_publisher<geometry_msgs::msg::Twist>("/test/b2_sdk/config/cmd_vel",
      10);
  auto height = node->create_client<b2_interface::srv::SetBodyHeight>("/b2/motion/set_body_height");
  auto pose = node->create_client<b2_interface::srv::SetBodyPose>("/b2/motion/set_body_pose");
  auto gait = node->create_client<b2_interface::srv::ConfigureGait>("/b2/motion/configure_gait");
  auto height_mode = node->create_client<b2_interface::srv::ConfigureHeightMode>(
    "/b2/motion/configure_height_mode");
  auto mode = node->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  auto stop = node->create_client<std_srvs::srv::Trigger>("/b2/motion/stop");
  RunningExecutor running; running.executor.add_node(driver); running.executor.add_node(node);
      running.start();
  ASSERT_TRUE(
    until([&]() {return snapshot().motion_ready && sport_valid && motors_valid;}));
  ASSERT_TRUE(
    until([&]() {return snapshot().package_version == "fixture-1.0";}));
  EXPECT_EQ(snapshot().service_list_result, 0);
  EXPECT_EQ(snapshot().service_names, std::vector<std::string>{"sport_mode"});
  EXPECT_EQ(snapshot().module_names, std::vector<std::string>{"sport"});
  ASSERT_TRUE(height->wait_for_service(1s)); ASSERT_TRUE(pose->wait_for_service(1s));
  ASSERT_TRUE(gait->wait_for_service(1s)); ASSERT_TRUE(stop->wait_for_service(1s));
  ASSERT_TRUE(mode->wait_for_service(1s));
  ASSERT_TRUE(height_mode->wait_for_service(1s));

  auto height_request = std::make_shared<b2_interface::srv::SetBodyHeight::Request>();
  for (double invalid : {.2, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    height_request->height = invalid;
    auto response = height->async_send_request(height_request);
    ASSERT_EQ(response.wait_for(1s), std::future_status::ready);
    EXPECT_FALSE(response.get()->success);
  }
  EXPECT_EQ(peer.configurations, 0);
  height_request->height = -.05;
  peer.stop_result = 88;
  auto failed_stop = height->async_send_request(height_request);
  ASSERT_EQ(failed_stop.wait_for(1s), std::future_status::ready);
  const auto stop_failure = failed_stop.get();
  EXPECT_FALSE(stop_failure->success); EXPECT_EQ(stop_failure->result_code, 88);
  EXPECT_EQ(stop_failure->message, "STOP_BEFORE_CONFIGURATION_FAILED");
  EXPECT_EQ(peer.configurations, 0);
  peer.stop_result = 0;
  auto height_result = height->async_send_request(height_request);
  ASSERT_EQ(height_result.wait_for(1s), std::future_status::ready);
  const auto accepted_height = height_result.get();
  ASSERT_TRUE(accepted_height->success) << accepted_height->message << " (SDK " <<
      accepted_height->result_code << ")";
  const auto height_json = unitree::common::AnyCast<unitree::common::JsonMap>(
    unitree::common::FromJsonString(peer.parameter(1013)));
  EXPECT_NEAR(unitree::common::AnyNumberCast<double>(height_json.at("data")), -.05, 1e-6);
  peer.configuration_result = 77;
  auto rejected = height->async_send_request(height_request);
  ASSERT_EQ(rejected.wait_for(1s), std::future_status::ready);
  const auto rejection = rejected.get(); EXPECT_FALSE(rejection->success);
      EXPECT_EQ(rejection->result_code, 77);
  peer.configuration_result = 0;
  // A StopMove reply cannot prove that the physical robot has stopped.
  peer.stationary = false;
  std::this_thread::sleep_for(50ms);
  const int before_unconfirmed = peer.configurations;
  auto unconfirmed = height->async_send_request(height_request);
  ASSERT_EQ(unconfirmed.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(unconfirmed.get()->message, "STATIONARY_TELEMETRY_NOT_CONFIRMED");
  EXPECT_EQ(peer.configurations, before_unconfirmed);
  peer.stationary = true;

  auto pose_request = std::make_shared<b2_interface::srv::SetBodyPose::Request>();
  for (double invalid : {.3, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    pose_request->roll = invalid;
    const int before_invalid_pose = peer.configurations;
    auto invalid_pose = pose->async_send_request(pose_request);
    ASSERT_EQ(invalid_pose.wait_for(1s), std::future_status::ready);
    EXPECT_FALSE(invalid_pose.get()->success); EXPECT_EQ(peer.configurations, before_invalid_pose);
  }
  pose_request->roll = .1; pose_request->pitch = -.15; pose_request->yaw = .05;
  peer.slow_pose = true;
  const int configuration_count = peer.configurations;
  auto pose_result = pose->async_send_request(pose_request);
  ASSERT_TRUE(
    until([&]() {return peer.configurations > configuration_count;}));
  // A second configuration must be rejected at arrival, not execute later
  // from the callback queue after an operator's stop.
  const auto busy_requested_at = Clock::now();
  auto duplicate = height->async_send_request(height_request);
  EXPECT_EQ(duplicate.wait_for(300ms), std::future_status::ready);
  RecordProperty("busy_rejection_ms", std::to_string(
      std::chrono::duration<double, std::milli>(Clock::now() - busy_requested_at).count()));
  auto mode_request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  mode_request->mode = b2_interface::srv::SetMotionMode::Request::BALANCE_STAND;
  auto duplicate_mode = mode->async_send_request(mode_request);
  ASSERT_EQ(duplicate_mode.wait_for(300ms), std::future_status::ready);
  EXPECT_EQ(duplicate_mode.get()->message, "MOTION_OPERATION_BUSY");
  auto duplicate_height_mode = height_mode->async_send_request(
    std::make_shared<b2_interface::srv::ConfigureHeightMode::Request>());
  ASSERT_EQ(duplicate_height_mode.wait_for(300ms), std::future_status::ready);
  EXPECT_EQ(duplicate_height_mode.get()->message, "MOTION_OPERATION_BUSY");
  const auto feedback_before = snapshot().header.stamp;
  const auto stop_requested_at = Clock::now();
  auto stopped = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stopped.wait_for(500ms), std::future_status::ready);
      EXPECT_TRUE(stopped.get()->success);
  RecordProperty("stop_during_configuration_ms", std::to_string(
      std::chrono::duration<double, std::milli>(Clock::now() - stop_requested_at).count()));
  EXPECT_EQ(pose_result.wait_for(0ms), std::future_status::timeout);
  ASSERT_TRUE(
    until([&]() {return snapshot().header.stamp != feedback_before;}));
  ASSERT_EQ(pose_result.wait_for(3s), std::future_status::ready);
  const auto interrupted = pose_result.get();
  EXPECT_FALSE(interrupted->success); EXPECT_EQ(interrupted->result_code, 0);
  EXPECT_NE(interrupted->message.find("CONFIGURATION_INTERRUPTED_BY_STOP"), std::string::npos);
  ASSERT_EQ(duplicate.wait_for(2s), std::future_status::ready);
  const auto busy = duplicate.get();
  EXPECT_FALSE(busy->success);
  EXPECT_EQ(busy->message, "MOTION_OPERATION_BUSY");
  EXPECT_EQ(peer.configurations, configuration_count + 1);
  const auto pose_json = unitree::common::AnyCast<unitree::common::JsonMap>(
    unitree::common::FromJsonString(peer.parameter(1051)));
  EXPECT_NEAR(unitree::common::AnyNumberCast<double>(pose_json.at("x")), .1, 1e-6);
  EXPECT_NEAR(unitree::common::AnyNumberCast<double>(pose_json.at("y")), -.15, 1e-6);
  EXPECT_NEAR(unitree::common::AnyNumberCast<double>(pose_json.at("z")), .05, 1e-6);

  // A fault during a slow setting must issue StopMove even though no velocity
  // command is cached, and a delayed zero SDK reply must not report success.
  const int before_fault_configuration = peer.configurations;
  auto fault_configuration = pose->async_send_request(pose_request);
  ASSERT_TRUE(
    until([&]() {return peer.configurations > before_fault_configuration;}));
  const int before_configuration_stops = peer.stops;
  const auto fault_started_at = Clock::now();
  peer.sport_error = 0x2222;
  ASSERT_TRUE(
    until([&]() {return peer.stops > before_configuration_stops;}));
  RecordProperty("fault_to_stop_request_ms", std::to_string(
      std::chrono::duration<double, std::milli>(Clock::now() - fault_started_at).count()));
  ASSERT_EQ(fault_configuration.wait_for(3s), std::future_status::ready);
  const auto aborted = fault_configuration.get();
  EXPECT_FALSE(aborted->success); EXPECT_EQ(aborted->result_code, 0);
  EXPECT_NE(aborted->message.find("CONFIGURATION_ABORTED: SPORT_ERROR"), std::string::npos);
  peer.sport_error = 0;
  ASSERT_TRUE(
    until([&]() {return snapshot().motion_ready;}));

  // Optional inventory/version RPC failure must neither imply controller
  // failure nor continue displaying old version data as a new successful read.
  peer.drop_metadata_reply = true;
  const auto metadata_unavailable = [&]() {
      return snapshot().service_list_result != 0 && snapshot().version_result != 0;
    };
  ASSERT_TRUE(until(metadata_unavailable));
  EXPECT_TRUE(snapshot().motion_ready);
  EXPECT_TRUE(snapshot().package_version.empty());
  EXPECT_TRUE(snapshot().service_names.empty());
  peer.drop_metadata_reply = false;
  ASSERT_TRUE(
    until([&]() {return snapshot().version_result == 0;}));

  auto gait_request = std::make_shared<b2_interface::srv::ConfigureGait::Request>();
  gait_request->gait_id = 100;
  auto invalid_gait = gait->async_send_request(gait_request);
  ASSERT_EQ(invalid_gait.wait_for(1s), std::future_status::ready);
      EXPECT_FALSE(invalid_gait.get()->success);
  gait_request->gait_id = 2;
  for (uint8_t feature = 0; feature <= 3; ++feature) {
    gait_request->feature = feature; gait_request->enabled = true;
    auto configured = gait->async_send_request(gait_request);
    ASSERT_EQ(configured.wait_for(1s), std::future_status::ready);
    EXPECT_TRUE(configured.get()->success);
  }
  const auto gait_json = unitree::common::AnyCast<unitree::common::JsonMap>(
    unitree::common::FromJsonString(peer.parameter(1011)));
  EXPECT_EQ(unitree::common::AnyNumberCast<int>(gait_json.at("data")), 2);
  for (int api : {1019, 1050, 1040}) {
    const auto payload = unitree::common::AnyCast<unitree::common::JsonMap>(
      unitree::common::FromJsonString(peer.parameter(api)));
    EXPECT_TRUE(unitree::common::AnyCast<bool>(payload.at("data")));
  }

  auto height_mode_request = std::make_shared<b2_interface::srv::ConfigureHeightMode::Request>();
  const int before_invalid_height_mode = peer.configurations;
  const int stops_before_invalid_height_mode = peer.stops;
  height_mode_request->feature = 255;
  auto invalid_height_mode = height_mode->async_send_request(height_mode_request);
  ASSERT_EQ(invalid_height_mode.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(invalid_height_mode.get()->message, "UNSUPPORTED_HEIGHT_MODE_FEATURE");
  EXPECT_EQ(peer.configurations, before_invalid_height_mode);
  EXPECT_EQ(peer.stops, stops_before_invalid_height_mode);
  for (uint8_t feature : {0, 1}) {
    for (bool enabled : {true, false}) {
      height_mode_request->feature = feature; height_mode_request->enabled = enabled;
      const int before_request_stops = peer.stops;
      auto configured = height_mode->async_send_request(height_mode_request);
      ASSERT_EQ(configured.wait_for(1s), std::future_status::ready);
      const auto result = configured.get();
      EXPECT_TRUE(result->success) << result->message;
      EXPECT_GT(peer.stops, before_request_stops);
      const auto payload = unitree::common::AnyCast<unitree::common::JsonMap>(
        unitree::common::FromJsonString(peer.parameter(feature == 0 ? 1052 : 1053)));
      EXPECT_EQ(unitree::common::AnyCast<bool>(payload.at("data")), enabled);
    }
  }
  peer.height_mode_result = 66;
  auto rejected_height_mode = height_mode->async_send_request(height_mode_request);
  ASSERT_EQ(rejected_height_mode.wait_for(1s), std::future_status::ready);
  const auto height_mode_rejection = rejected_height_mode.get();
  EXPECT_FALSE(height_mode_rejection->success);
  EXPECT_EQ(height_mode_rejection->result_code, 66);
  peer.height_mode_result = 0;

  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;
  command->publish(velocity); ASSERT_TRUE(
    until([&]() {return peer.moves > 0;}));
  const int before_fault_stops = peer.stops;
  peer.sport_error = 0x1234;
  const auto sport_fault_stopped = [&]() {
      return snapshot().reason == "SPORT_ERROR" && sdk_error_visible &&
             peer.stops > before_fault_stops;
    };
  ASSERT_TRUE(until(sport_fault_stopped));
  const int blocked = peer.configurations;
  auto fault_request = height->async_send_request(height_request);
  ASSERT_EQ(fault_request.wait_for(1s), std::future_status::ready);
      EXPECT_FALSE(fault_request.get()->success);
  EXPECT_EQ(peer.configurations, blocked);
  peer.sport_error = 0;
  ASSERT_TRUE(
    until([&]() {return snapshot().motion_ready;}));
  peer.controller_active = false;
  ASSERT_TRUE(
    until([&]() {return snapshot().reason == "CONTROLLER_MODE_UNAVAILABLE";}));
  peer.controller_active = true;
  ASSERT_TRUE(
    until([&]() {return snapshot().motion_ready;}));
  peer.drop_controller_reply = true;
  ASSERT_TRUE(
    until([&]() {return snapshot().reason == "CONTROLLER_QUERY_FAILED";}));
  auto stop_without_controller =
    stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_without_controller.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(stop_without_controller.get()->success);
  peer.drop_controller_reply = false;
  ASSERT_TRUE(
    until([&]() {return snapshot().motion_ready;}));
  peer.publish_low = false;
  ASSERT_TRUE(
    until([&]() {return snapshot().reason == "LOW_STATE_STALE";}));
  const int before_stale = peer.moves;
  for (int i = 0; i < 10; ++i) {
    command->publish(velocity); std::this_thread::sleep_for(10ms);
                                                                                              }
  EXPECT_EQ(peer.moves, before_stale);
}

TEST(B2SdkSafety, ConfigurationRequiresExplicitLimits) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  rclcpp::Node node("b2_configuration_defaults");
  const b2_base::MotionConfiguration config(node);
  EXPECT_FALSE(config.validate(b2_interface::srv::SetBodyHeight::Request{}).empty());
  EXPECT_FALSE(config.validate(b2_interface::srv::SetBodyPose::Request{}).empty());
  EXPECT_FALSE(config.validate(b2_interface::srv::ConfigureGait::Request{}).empty());
  EXPECT_EQ(config.validate(b2_interface::srv::ConfigureHeightMode::Request{}),
    "HEIGHT_MODE_CONFIGURATION_DISABLED");
  rclcpp::NodeOptions invalid;
  invalid.parameter_overrides({{"body_height.enabled", true}});
  rclcpp::Node invalid_node("b2_configuration_invalid", invalid);
  EXPECT_THROW(b2_base::MotionConfiguration bad(invalid_node), std::invalid_argument);
}

TEST(B2SdkSafety, LowStateMessagesKeepMappingAndReuseStorage) {
  b2_base::LowStateMessages messages("base_link");
  const auto storage = [&]() {
      return std::array<const void *, 7>{messages.joints.name.data(),
      messages.joints.position.data(), messages.joints.velocity.data(),
      messages.joints.effort.data(), messages.motors.motors.data(),
      messages.foot_force.data.data(), messages.battery.cell_voltage.data()};
    };
  const auto original_storage = storage();
  const std::array<int, 12> source_index{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
  const std::array<int32_t, 3> currents{-3000, 2000, 0};
  const std::array<uint8_t, 3> statuses{
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
  unitree_go::msg::dds_::LowState_ state;
  for (int sample = 0; sample < 64; ++sample) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 20 + sample; stamp.nanosec = 123;
    for (std::size_t i = 0; i < state.motor_state().size(); ++i) {
      auto & motor = state.motor_state()[i];
      motor.q() = sample + i + .25F;
      motor.dq() = -static_cast<float>(sample + i) - .5F;
      motor.tau_est() = sample + i + .75F;
      motor.mode() = (sample + i) % 16;
      motor.temperature() = 30 + i;
      motor.lost() = 1000 + sample + i;
    }
    for (std::size_t i = 0; i < state.foot_force().size(); ++i) {
      state.foot_force()[i] = -sample - static_cast<int>(i);
    }
    state.power_v() = 51.5F;
    state.bms_state().current() = currents[sample % 3];
    state.bms_state().soc() = 72;
    state.bms_state().bq_ntc() = {30, static_cast<uint8_t>(40 + sample)};
    state.bms_state().mcu_ntc() = {35, 32};
    for (std::size_t i = 0; i < state.bms_state().cell_vol().size(); ++i) {
      state.bms_state().cell_vol()[i] = 3500 + sample + i;
    }
    messages.update(state, stamp);
    EXPECT_EQ(storage(), original_storage);
    EXPECT_EQ(messages.joints.header.stamp, stamp);
    EXPECT_EQ(messages.motors.header.stamp, stamp);
    EXPECT_EQ(messages.battery.header.stamp, stamp);
    EXPECT_EQ(messages.motors.header.frame_id, "base_link");
    ASSERT_EQ(messages.joints.name.size(), 12U);
    ASSERT_EQ(messages.motors.motors.size(), 12U);
    for (std::size_t i = 0; i < source_index.size(); ++i) {
      const int index = source_index[i];
      EXPECT_EQ(messages.joints.position[i], sample + index + .25);
      EXPECT_EQ(messages.joints.velocity[i], -sample - index - .5);
      EXPECT_EQ(messages.joints.effort[i], sample + index + .75);
      EXPECT_EQ(messages.motors.motors[i].joint_name, messages.joints.name[i]);
      EXPECT_EQ(messages.motors.motors[i].mode, (sample + index) % 16);
      EXPECT_EQ(messages.motors.motors[i].temperature_c, 30 + index);
      EXPECT_EQ(messages.motors.motors[i].lost, 1000U + sample + index);
    }
    EXPECT_EQ(messages.joints.name[0], "FL_hip_joint");
    EXPECT_EQ(messages.joints.name[3], "FR_hip_joint");
    EXPECT_EQ(messages.joints.name[6], "RL_hip_joint");
    EXPECT_EQ(messages.joints.name[9], "RR_hip_joint");
    EXPECT_FLOAT_EQ(messages.battery.voltage, 51.5F);
    EXPECT_FLOAT_EQ(messages.battery.current, -currents[sample % 3] / 1000.F);
    EXPECT_EQ(messages.battery.power_supply_status, statuses[sample % 3]);
    EXPECT_TRUE(messages.battery.present);
    EXPECT_FLOAT_EQ(messages.battery.percentage, .72F);
    EXPECT_FLOAT_EQ(messages.battery.temperature, 40 + sample);
    ASSERT_EQ(messages.battery.cell_voltage.size(), state.bms_state().cell_vol().size());
    for (std::size_t i = 0; i < messages.battery.cell_voltage.size(); ++i) {
      EXPECT_FLOAT_EQ(messages.battery.cell_voltage[i], (3500 + sample + i) / 1000.F);
    }
    EXPECT_EQ(messages.foot_force.data, std::vector<int16_t>(
        state.foot_force().begin(), state.foot_force().end()));
  }
  RecordProperty("reused_updates", 64);
  RecordProperty("buffer_relocations", storage() == original_storage ? 0 : 1);
}

TEST(B2SdkSafety, CompatibilityOverrideRetainsTelemetryAndFaultGates) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"controller.require_ready", false}});
  rclcpp::Node node("b2_compatibility_check", options);
  b2_base::DriverStatus status(node, .03, "base_link");
  EXPECT_FALSE(status.status().motion_ready);
  unitree_go::msg::dds_::SportModeState_ sport;
  unitree_go::msg::dds_::LowState_ low;
  status.observe_sport(sport); status.observe_low(low);
  EXPECT_TRUE(status.status().motion_ready);
  EXPECT_TRUE(status.readiness().motion_ready);
  EXPECT_STREQ(status.readiness().reason, status.status().reason.c_str());
  EXPECT_FALSE(status.status().controller_ready);
  EXPECT_FALSE(status.status().controller_ready_required);
  sport.error_code() = 42;
  status.observe_sport(sport);
  EXPECT_EQ(status.status().reason, "SPORT_ERROR");
  EXPECT_STREQ(status.readiness().reason, "SPORT_ERROR");
  sport.error_code() = 0;
  status.observe_sport(sport);
  std::this_thread::sleep_for(40ms);
  EXPECT_EQ(status.status().reason, "SPORT_STATE_STALE");
  EXPECT_STREQ(status.readiness().reason, "SPORT_STATE_STALE");
}

TEST(B2SdkSafety, InvalidMotionParametersFailBeforeSdkCommands) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & invalid : std::vector<rclcpp::Parameter>{
      {"max_linear_x", nan}, {"max_linear_y", -.1}, {"max_angular_z", inf},
      {"max_reverse_linear_x", -.1}, {"deadband", nan}, {"command_rate_hz", 0.},
      {"command_rate_hz", 1e12}, {"cmd_timeout", inf}, {"command_rpc_timeout", 1e-300},
      {"command_rpc_timeout", 1e300}, {"command_rpc_timeout", nan},
      {"command_rpc_timeout", inf}})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155}, invalid});
    EXPECT_THROW(std::make_shared<B2SdkDriver>(options),
        std::invalid_argument) << invalid.get_name();
  }
  EXPECT_EQ(peer.sport_requests, 0);
}

TEST(B2SdkSafety, ReadinessHotPathBenchmark) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer; peer.metadata_count = 64;
  auto node = std::make_shared<rclcpp::Node>("b2_readiness_benchmark");
  b2_base::DriverStatus status(*node, 60., "base_link");
  unitree_go::msg::dds_::SportModeState_ sport;
  unitree_go::msg::dds_::LowState_ low;
  status.observe_sport(sport); status.observe_low(low);
  RunningExecutor running; running.executor.add_node(node); running.start();
  const auto ready_with_metadata = [&]() {
      return status.status().service_names.size() == 64 && status.readiness().motion_ready;
    };
  ASSERT_TRUE(until(ready_with_metadata));
  EXPECT_EQ(status.status().module_names.size(), 64U);
  EXPECT_STREQ(status.readiness().reason, status.status().reason.c_str());
  constexpr int iterations = 10000;
  std::uint64_t checksum = 0;
  auto measure = [&](bool full_snapshot) {
      std::uint64_t count = 0;
      const auto start = Clock::now();
      for (int i = 0; i < iterations; ++i) {
        count += full_snapshot ? status.status().motion_ready : status.readiness().motion_ready;
      }
      checksum += count;
      return std::chrono::duration<double, std::nano>(Clock::now() - start).count() / iterations;
    };
  measure(true); measure(false);
  std::vector<double> full, light;
  for (int sample = 0; sample < 7; ++sample) {
    if (sample % 2) {light.push_back(measure(false)); full.push_back(measure(true));} else {
      full.push_back(measure(true)); light.push_back(measure(false));
    }
  }
  std::sort(full.begin(), full.end()); std::sort(light.begin(), light.end());
  EXPECT_EQ(checksum, static_cast<std::uint64_t>(iterations) * 16U);
  RecordProperty("metadata_entries", 64);
  RecordProperty("full_snapshot_ns", std::to_string(full[3]));
  RecordProperty("readiness_ns", std::to_string(light[3]));
  RecordProperty("readiness_speedup", std::to_string(full[3] / light[3]));
}

TEST(B2SdkSafety, ReadOnlyKeepsTelemetryWithoutSendingSportCommands) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  auto node = std::make_shared<rclcpp::Node>("b2_read_only_test");
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"enable_control", false}, {"auto_balance_stand_on_start", true}, {"publish_tf", false},
      {"cmd_vel_topic", "/test/b2_sdk/read_only/cmd_vel"}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  EXPECT_TRUE(driver->describe_parameter("enable_control").read_only);
  EXPECT_TRUE(driver->describe_parameter("max_linear_x").read_only);
  EXPECT_TRUE(driver->describe_parameter("command_rate_hz").read_only);
  std::atomic<int> odometry{0}; std::atomic<bool> disabled{false};
  auto odom = node->create_subscription<nav_msgs::msg::Odometry>("/b2/odom",
    rclcpp::SensorDataQoS(), [&](const nav_msgs::msg::Odometry &) {++odometry;});
  auto status = node->create_subscription<b2_interface::msg::DriverStatus>("/b2/driver_status",
    rclcpp::QoS(1).reliable().transient_local(),
      [&](const b2_interface::msg::DriverStatus & value) {
        disabled = !value.motion_ready && value.reason == "CONTROL_DISABLED";
    });
  auto mode = node->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  auto height = node->create_client<b2_interface::srv::SetBodyHeight>("/b2/motion/set_body_height");
  auto height_mode = node->create_client<b2_interface::srv::ConfigureHeightMode>(
    "/b2/motion/configure_height_mode");
  auto stop = node->create_client<std_srvs::srv::Trigger>("/b2/motion/stop");
  auto command = node->create_publisher<geometry_msgs::msg::Twist>("/test/b2_sdk/read_only/cmd_vel",
      10);
  RunningExecutor running; running.executor.add_node(driver); running.executor.add_node(node);
      running.start();
  const auto read_only_ready = [&]() {
      return disabled && odometry > 0 && command->get_subscription_count() > 0;
    };
  ASSERT_TRUE(until(read_only_ready));
  ASSERT_TRUE(mode->wait_for_service(1s)); ASSERT_TRUE(height->wait_for_service(1s));
  ASSERT_TRUE(stop->wait_for_service(1s));
  ASSERT_TRUE(height_mode->wait_for_service(1s));
  auto requested_mode = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  requested_mode->mode = b2_interface::srv::SetMotionMode::Request::BALANCE_STAND;
  auto mode_result = mode->async_send_request(requested_mode);
  ASSERT_EQ(mode_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(mode_result.get()->message, "CONTROL_DISABLED");
  auto height_result =
    height->async_send_request(std::make_shared<b2_interface::srv::SetBodyHeight::Request>());
  ASSERT_EQ(height_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(height_result.get()->message, "CONTROL_DISABLED");
  auto height_mode_result = height_mode->async_send_request(
    std::make_shared<b2_interface::srv::ConfigureHeightMode::Request>());
  ASSERT_EQ(height_mode_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(height_mode_result.get()->message, "CONTROL_DISABLED");
  auto stop_result = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(stop_result.get()->message, "CONTROL_DISABLED");
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;
  const int before = odometry;
  for (int i = 0; i < 12; ++i) {
    command->publish(velocity); std::this_thread::sleep_for(50ms);
                                                                                              }
  EXPECT_GT(odometry, before); EXPECT_EQ(peer.sport_requests, 0);
  RecordProperty("read_only_sdk_handshakes", peer.internal_requests.load());
}

class SdkRpcProgress : public ::testing::TestWithParam<unsigned int> {};

TEST(B2SdkSafety, SdkExecutorsCanImmediatelyStopWithIndependentContext) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("b2_sdk_executor_lifetime",
      rclcpp::NodeOptions().context(context));
  for (unsigned int i = 0; i < 20; ++i) {
    auto commands = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto modes = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    auto stops = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    b2_base::SdkCallbackExecutors io(node->get_node_base_interface(), commands, modes, stops);
    // No main executor spin, delay or global context initialization precedes
    // destruction. This also exercises the cancel-before-worker-start race.
  }
  node.reset();
  context->shutdown("SDK executor lifetime test finished");
}

TEST(B2SdkSafety, StandDownBlocksEvenZeroMoveUntilExplicitStanding) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer; peer.stationary = true;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"controller.require_ready", false}, {"publish_tf", false},
      {"cmd_vel_topic", "/test/rest/cmd_vel"}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  auto observer = std::make_shared<rclcpp::Node>("b2_rest_command_test");
  auto command = observer->create_publisher<geometry_msgs::msg::Twist>("/test/rest/cmd_vel", 10);
  auto mode = observer->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  std::atomic<bool> ready{false};
  auto state = observer->create_subscription<b2_interface::msg::DriverStatus>(
    "/b2/driver_status", rclcpp::QoS(1).reliable().transient_local(),
    [&](const b2_interface::msg::DriverStatus &value) {ready = value.motion_ready;});
  RunningExecutor receiver;
  receiver.executor.add_node(driver); receiver.executor.add_node(observer); receiver.start();
  ASSERT_TRUE(until([&] {return ready && command->get_subscription_count() > 0;}));
  ASSERT_TRUE(mode->wait_for_service(1s));
  auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::STAND_DOWN;
  auto resting = mode->async_send_request(request);
  ASSERT_EQ(resting.wait_for(2s), std::future_status::ready);
  ASSERT_TRUE(resting.get()->success);
  const auto moves = peer.moves.load();
  const auto stops = peer.stops.load();
  for (int i = 0; i < 8; ++i) {
    geometry_msgs::msg::Twist velocity;
    if (i % 2) velocity.linear.x = .1;
    command->publish(velocity); std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(peer.moves.load(), moves) << "Move(0) must not disturb StandDown either";
  EXPECT_EQ(peer.stops.load(), stops) << "No ordinary delayed StopMove after StandDown";
  EXPECT_FALSE(ready.load());
  request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::STAND_UP;
  auto standing = mode->async_send_request(request);
  ASSERT_EQ(standing.wait_for(2s), std::future_status::ready);
  ASSERT_TRUE(standing.get()->success);
  ASSERT_TRUE(until([&] {return ready.load();}));
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;
  ASSERT_TRUE(until([&] {command->publish(velocity); return peer.moves.load() > moves;}));
}

TEST(B2SdkSafety, StandDownRequiresStopProofButDampRemainsAvailable) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"controller.require_ready", false}, {"publish_tf", false}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  auto observer = std::make_shared<rclcpp::Node>("b2_posture_stop_test");
  auto mode = observer->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  RunningExecutor running;
  running.executor.add_node(driver); running.executor.add_node(observer); running.start();
  ASSERT_TRUE(mode->wait_for_service(1s));
  auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::STAND_DOWN;
  auto moving = mode->async_send_request(request);
  ASSERT_EQ(moving.wait_for(4s), std::future_status::ready);
  EXPECT_FALSE(moving.get()->success); EXPECT_EQ(peer.stand_downs, 0);
  peer.stop_result = 7;
  auto unacknowledged = mode->async_send_request(request);
  ASSERT_EQ(unacknowledged.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(unacknowledged.get()->success); EXPECT_EQ(peer.stand_downs, 0);
  peer.publish_state = false;
  std::this_thread::sleep_for(600ms);
  request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::DAMP;
  auto damping = mode->async_send_request(request);
  ASSERT_EQ(damping.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(damping.get()->success); EXPECT_EQ(peer.damps, 1);
}

TEST(B2SdkSafety, SportStateMessagesPreserveFramesBodyVelocityAndStorage) {
  b2_base::SportStateMessages messages("inspection_robot_001/odom",
    "inspection_robot_001/base_link", "inspection_robot_001/imu_link");
  const auto *frame_storage = messages.odom.child_frame_id.data();
  unitree_go::msg::dds_::SportModeState_ state;
  state.imu_state().quaternion()[0] = .70710678F;
  state.imu_state().quaternion()[3] = .70710678F;
  state.velocity() = {.1F, 0.F, .02F}; state.position() = {1.F, 2.F, .5F};
  state.yaw_speed() = -.15F; state.mode() = 5;
  builtin_interfaces::msg::Time stamp; stamp.sec = 20;
  for (unsigned int i = 0; i < 1000; ++i) {
    stamp.nanosec = i; messages.update(state, stamp);
  }
  EXPECT_EQ(messages.odom.header.stamp, stamp);
  EXPECT_EQ(messages.imu.header.frame_id, "inspection_robot_001/imu_link");
  EXPECT_EQ(messages.odom.header.frame_id, "inspection_robot_001/odom");
  EXPECT_EQ(messages.odom.child_frame_id.data(), frame_storage);
  EXPECT_NEAR(messages.odom.twist.twist.linear.x, 0., 1e-7);
  EXPECT_NEAR(messages.odom.twist.twist.linear.y, -.1, 1e-7);
  EXPECT_NEAR(messages.odom.twist.twist.linear.z, .02, 1e-7);
  EXPECT_NEAR(messages.odom.twist.twist.angular.z, -.15, 1e-7);
  EXPECT_EQ(messages.transform.transform.translation.x, 1.);
  EXPECT_EQ(messages.transform.transform.translation.y, 2.);
  EXPECT_EQ(messages.transform.transform.translation.z, .5);
  EXPECT_EQ(messages.transform.transform.rotation, messages.imu.orientation);
  EXPECT_EQ(messages.transform.header, messages.odom.header);
  EXPECT_EQ(messages.mode.data, 5);
}

TEST_P(SdkRpcProgress, DelayedModeKeepsTelemetryAndStopResponsive) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer; peer.stationary = true;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
      {"controller.require_ready", false}, {"publish_tf", false}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  auto observer = std::make_shared<rclcpp::Node>("b2_rpc_progress_observer");
  std::atomic<unsigned int> samples{0};
  std::mutex timing_mutex;
  auto previous = Clock::now();
  double maximum_gap_ms = 0.;
  bool measuring = false;
  auto odometry = observer->create_subscription<nav_msgs::msg::Odometry>(
    "/b2/odom", rclcpp::SensorDataQoS(), [&](const nav_msgs::msg::Odometry &) {
      const auto now = Clock::now();
      std::lock_guard<std::mutex> lock(timing_mutex);
      if (measuring) {
        maximum_gap_ms = std::max(maximum_gap_ms,
            std::chrono::duration<double, std::milli>(now - previous).count());
      }
      previous = now; ++samples;
    });
  auto mode = observer->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  auto stop = observer->create_client<std_srvs::srv::Trigger>("/b2/motion/stop");
  RunningExecutor receiver;
  receiver.executor.add_node(observer); receiver.start();
  struct DriverExecutor {
    explicit DriverExecutor(unsigned int threads, const rclcpp::Node::SharedPtr &node)
    : executor(rclcpp::ExecutorOptions(), threads) {
      executor.add_node(node); worker = std::thread([this] {executor.spin();});
    }
    ~DriverExecutor() {executor.cancel(); if (worker.joinable()) worker.join();}
    rclcpp::executors::MultiThreadedExecutor executor;
    std::thread worker;
  } execution(GetParam(), driver);
  ASSERT_TRUE(until([&] {return samples >= 3;}));
  ASSERT_TRUE(mode->wait_for_service(1s)); ASSERT_TRUE(stop->wait_for_service(1s));
  {
    std::lock_guard<std::mutex> lock(timing_mutex);
    previous = Clock::now(); measuring = true;
  }
  auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::VISION_WALK_ON;
  const auto mode_started = Clock::now();
  auto operation = mode->async_send_request(request);
  ASSERT_TRUE(until([&] {return peer.modes > 0;}));
  const auto before = samples.load();
  const auto stop_started = Clock::now();
  auto stopping = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  const bool stop_during_mode = stopping.wait_for(300ms) == std::future_status::ready;
  const auto stop_ms = std::chrono::duration<double, std::milli>(Clock::now()-stop_started).count();
  // The peer intentionally holds this mode RPC for 1.5 s. Sensor publication
  // and explicit StopMove must progress before its result, even on a shared
  // executor with one ROS thread. No physical gait model is involved.
  std::this_thread::sleep_for(150ms);
  const auto during = samples.load() - before;
  EXPECT_TRUE(stop_during_mode);
  EXPECT_GT(during, 1u);
  EXPECT_EQ(operation.wait_for(0ms), std::future_status::timeout);
  ASSERT_EQ(operation.wait_for(3s), std::future_status::ready);
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(operation.get()->success); EXPECT_TRUE(stopping.get()->success);
  const auto mode_ms = std::chrono::duration<double, std::milli>(Clock::now()-mode_started).count();
  std::this_thread::sleep_for(100ms);
  RecordProperty("ros_executor_threads", std::to_string(GetParam()));
  RecordProperty("stop_ready_before_mode", stop_during_mode ? "true" : "false");
  RecordProperty("stop_wait_observed_ms", std::to_string(stop_ms));
  RecordProperty("mode_rpc_observed_ms", std::to_string(mode_ms));
  RecordProperty("odom_samples_during_rpc_window", std::to_string(during));
  std::lock_guard<std::mutex> lock(timing_mutex);
  RecordProperty("maximum_odom_gap_ms", std::to_string(maximum_gap_ms));
}

INSTANTIATE_TEST_SUITE_P(RosExecutors, SdkRpcProgress, ::testing::Values(1u, 2u, 4u));
}  // namespace
