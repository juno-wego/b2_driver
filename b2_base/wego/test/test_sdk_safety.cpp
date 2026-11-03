// Exercise the real driver and SDK protocol on loopback, with no robot.
#define B2_SDK_DRIVER_NO_MAIN
#include "../b2_sdk_driver.cpp"
#include <gtest/gtest.h>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/robot/internal/internal_idl_decl/Response_.hpp>

namespace {
using Clock = std::chrono::steady_clock;
using Request = unitree_api::msg::dds_::Request_;
using Response = unitree_api::msg::dds_::Response_;
using namespace std::chrono_literals;

class FakeRobot {
public:
  FakeRobot() : responses_("rt/api/sport/response"), requests_("rt/api/sport/request"),
    sport_("rt/sportmodestate"), low_("rt/lowstate")
  {
    unitree::robot::ChannelFactory::Instance()->Init(155, "lo");
    responses_.InitChannel(); sport_.InitChannel(); low_.InitChannel();
    requests_.InitChannel([this](const void * raw) {
      const auto & request = *static_cast<const Request *>(raw);
      const auto api = request.header().identity().api_id();
      if (api == 1008) {++moves; if (drop_move_reply) {return;}}
      if (api == 1003) {++stops;}
      if (api == 1101) {++modes;}
      Response response;
      response.header().identity() = request.header().identity();
      response.header().status().code() = 0;
      const auto due = Clock::now() + (api == 1101 ? 1500ms : 0ms);
      std::lock_guard<std::mutex> lock(mutex_);
      replies_.emplace_back(due, response);
    }, 32);
  }
  ~FakeRobot() {requests_.CloseChannel();}
  void tick() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto item = replies_.begin(); item != replies_.end();) {
        if (item->first <= Clock::now()) {responses_.Write(item->second); item = replies_.erase(item);}
        else {++item;}
      }
    }
    if (publish_state) {
      unitree_go::msg::dds_::SportModeState_ sport;
      sport.imu_state().quaternion()[0] = 1.0;
      sport_.Write(sport);
      low_.Write(unitree_go::msg::dds_::LowState_());
    }
  }
  std::atomic<int> moves{0}, stops{0}, modes{0};
  std::atomic<bool> publish_state{true}, drop_move_reply{false};
private:
  unitree::robot::ChannelPublisher<Response> responses_;
  unitree::robot::ChannelSubscriber<Request> requests_;
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::SportModeState_> sport_;
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_> low_;
  std::mutex mutex_;
  std::vector<std::pair<Clock::time_point, Response>> replies_;
};

struct RunningExecutor {
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 4};
  std::thread worker;
  void start() {worker = std::thread([this]() {executor.spin();});}
  ~RunningExecutor() {executor.cancel(); if (worker.joinable()) {worker.join();}}
};

template<class Predicate> bool until(Predicate predicate, std::chrono::milliseconds timeout = 3s) {
  const auto end = Clock::now() + timeout;
  while (!predicate() && Clock::now() < end) {std::this_thread::sleep_for(10ms);}
  return predicate();
}

TEST(B2SdkSafety, StopOwnershipCommunicationLossAndSlowRpc) {
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  FakeRobot peer;
  auto node = std::make_shared<rclcpp::Node>("b2_sdk_safety_test");
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"network_interface", "lo"}, {"dds_domain", 155},
    {"cmd_vel_topic", "/test/b2_sdk/cmd_vel"}, {"publish_tf", false},
    {"cmd_timeout", .2}, {"state_timeout", .15}, {"command_rpc_timeout", .05}});
  auto driver = std::make_shared<B2SdkDriver>(options);
  std::atomic<int> odometry{0}, joints{0}; std::atomic<bool> connected{false};
  auto odom = node->create_subscription<nav_msgs::msg::Odometry>("/b2/odom",
    rclcpp::SensorDataQoS(), [&](const nav_msgs::msg::Odometry &) {++odometry;});
  auto joint = node->create_subscription<sensor_msgs::msg::JointState>("/b2/joint_states",
    rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::JointState &) {++joints;});
  auto health = node->create_subscription<std_msgs::msg::Bool>("/b2/comm_ok",
    rclcpp::QoS(1).reliable().transient_local(), [&](const std_msgs::msg::Bool & value) {connected = value.data;});
  auto command = node->create_publisher<geometry_msgs::msg::Twist>("/test/b2_sdk/cmd_vel", 10);
  auto stop = node->create_client<std_srvs::srv::Trigger>("/b2/motion/stop");
  auto mode = node->create_client<b2_interface::srv::SetMotionMode>("/b2/motion/set_mode");
  auto timer = node->create_wall_timer(10ms, [&]() {peer.tick();});
  RunningExecutor running; running.executor.add_node(driver); running.executor.add_node(node); running.start();
  ASSERT_TRUE(until([&]() {return connected && joints > 0 && command->get_subscription_count() > 0;}));
  ASSERT_TRUE(stop->wait_for_service(1s)); ASSERT_TRUE(mode->wait_for_service(1s));
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;

  // No new input after StopMove: the cached nonzero command must stay invalid.
  command->publish(velocity); ASSERT_TRUE(until([&]() {return peer.moves > 0;}));
  auto stopped = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stopped.wait_for(500ms), std::future_status::ready); ASSERT_TRUE(stopped.get()->success);
  const int first_stop_moves = peer.moves;
  std::this_thread::sleep_for(150ms); EXPECT_EQ(peer.moves, first_stop_moves);

  // A long mode RPC must not occupy the command/stop or feedback callbacks.
  auto request = std::make_shared<b2_interface::srv::SetMotionMode::Request>();
  request->mode = b2_interface::srv::SetMotionMode::Request::VISION_WALK_ON;
  auto switching = mode->async_send_request(request);
  ASSERT_TRUE(until([&]() {return peer.modes > 0;}));
  const int feedback_before = odometry;
  auto stop_during_mode = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_during_mode.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(stop_during_mode.get()->success);
  EXPECT_EQ(switching.wait_for(0ms), std::future_status::timeout);
  ASSERT_TRUE(until([&]() {return odometry > feedback_before + 2;}));
  ASSERT_EQ(switching.wait_for(3s), std::future_status::ready); EXPECT_TRUE(switching.get()->success);

  // Even a lost Move response has a bounded command-channel RPC timeout.
  peer.drop_move_reply = true;
  const int moves_before = peer.moves; command->publish(velocity);
  ASSERT_TRUE(until([&]() {return peer.moves > moves_before;}));
  auto stop_during_move = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(stop_during_move.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(stop_during_move.get()->success);
  peer.drop_move_reply = false;
  const int second_stop_moves = peer.moves;
  std::this_thread::sleep_for(150ms); EXPECT_EQ(peer.moves, second_stop_moves);

  // Continue ROS commands while physical telemetry disappears. Stale state
  // must not be relabelled with new timestamps or permit more movement.
  command->publish(velocity); ASSERT_TRUE(until([&]() {return peer.moves > second_stop_moves;}));
  const int stops_before = peer.stops; peer.publish_state = false;
  ASSERT_TRUE(until([&]() {command->publish(velocity); return !connected && peer.stops > stops_before;}));
  std::this_thread::sleep_for(200ms);
  const int odom_after = odometry, joints_after = joints, moves_after = peer.moves;
  for (int i = 0; i < 20; ++i) {command->publish(velocity); std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(odometry, odom_after); EXPECT_EQ(joints, joints_after); EXPECT_EQ(peer.moves, moves_after);
}
}  // namespace
