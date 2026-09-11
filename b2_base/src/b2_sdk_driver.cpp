#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
  B2SdkDriver()
  : Node("b2_sdk_driver")
  {
    const auto interface = declare_parameter<std::string>("network_interface", "");
    const auto domain = static_cast<int32_t>(declare_parameter<int>("dds_domain", 0));
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

    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/b2/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/b2/imu/data");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/b2/joint_states");
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/b2/battery_state");
    foot_force_topic_ = declare_parameter<std::string>("foot_force_topic", "/b2/foot_force");
    // MuJoCo already uses /b2/mode as a String command input. Keep the
    // hardware's raw Unitree numeric state separate rather than publishing a
    // conflicting message type on the same ROS name.
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/b2/mode_code");
    comm_ok_topic_ = declare_parameter<std::string>("comm_ok_topic", "/b2/comm_ok");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.8);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.4);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.2);
    deadband_ = declare_parameter<double>("deadband", 0.01);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    stop_on_timeout_ = declare_parameter<bool>("stop_on_timeout", true);
    auto_balance_stand_ = declare_parameter<bool>("auto_balance_stand_on_start", false);
    state_timeout_ = declare_parameter<double>("state_timeout", 0.5);

    // SportClient creates DDS channels in its constructor, so construct it
    // only after ChannelFactory has selected the B2 network interface.
    sport_client_ = std::make_unique<unitree::robot::b2::SportClient>();
    sport_client_->SetTimeout(10.0F);
    sport_client_->Init();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, sensor_qos);
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
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_ = *command;
        last_command_time_ = std::chrono::steady_clock::now();
        have_command_ = true;
      });

    add_motion_service("/b2/motion/damp", [this]() { return sport_client_->Damp(); });
    add_motion_service("/b2/motion/free_walk", [this]() { return sport_client_->FreeWalk(); });
    add_motion_service("/b2/motion/classic_walk/on", [this]() { return sport_client_->ClassicWalk(true); });
    add_motion_service("/b2/motion/classic_walk/off", [this]() { return sport_client_->ClassicWalk(false); });
    add_motion_service("/b2/motion/speed/low", [this]() { return sport_client_->SpeedLevel(-1); });
    add_motion_service("/b2/motion/speed/high", [this]() { return sport_client_->SpeedLevel(1); });
    add_motion_service("/b2/motion/stand_up", [this]() { return sport_client_->StandUp(); });
    add_motion_service("/b2/motion/stand_down", [this]() { return sport_client_->StandDown(); });
    add_motion_service("/b2/motion/balance_stand", [this]() { return sport_client_->BalanceStand(); });
    add_motion_service("/b2/motion/vision_walk/on", [this]() { return sport_client_->VisionWalk(true); });
    add_motion_service("/b2/motion/vision_walk/off", [this]() { return sport_client_->VisionWalk(false); });
    add_motion_service("/b2/motion/stop", [this]() { return sport_client_->StopMove(); });

    sport_state_sub_ = std::make_shared<SportSubscriber>(kSportStateTopic);
    sport_state_sub_->InitChannel([this](const void * message) { on_sport_state(message); });
    low_state_sub_ = std::make_shared<LowSubscriber>(kLowStateTopic);
    low_state_sub_->InitChannel([this](const void * message) { on_low_state(message); });

    const auto command_period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(command_period),
      std::bind(&B2SdkDriver::send_velocity_command, this));
    state_timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&B2SdkDriver::publish_sport_state, this));
    low_state_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&B2SdkDriver::publish_low_state, this));
    health_timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&B2SdkDriver::publish_health, this));

    std_msgs::msg::Bool disconnected;
    disconnected.data = false;
    comm_ok_pub_->publish(disconnected);

    if (auto_balance_stand_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        const auto result = sport_client_->BalanceStand();
        RCLCPP_INFO(get_logger(), "Sent BalanceStand through Unitree SDK2 (result=%d).", result);
        startup_timer_->cancel();
      });
    }

    RCLCPP_INFO(
      get_logger(), "SDK2 direct B2 driver ready: DDS domain=%d, interface=%s, cmd_vel=%s.",
      static_cast<int>(domain), interface.c_str(), cmd_vel_topic_.c_str());
  }

private:
  using SportState = unitree_go::msg::dds_::SportModeState_;
  using LowState = unitree_go::msg::dds_::LowState_;
  using SportSubscriber = unitree::robot::ChannelSubscriber<SportState>;
  using LowSubscriber = unitree::robot::ChannelSubscriber<LowState>;

  void add_motion_service(const std::string & name, std::function<int32_t()> command)
  {
    motion_services_.push_back(create_service<std_srvs::srv::Trigger>(
      name,
      [this, name, command = std::move(command)](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        const int32_t result = command();
        response->success = result == 0;
        response->message = "Unitree SDK2 " + name + " result=" + std::to_string(result);
        if (result != 0) {
          RCLCPP_WARN(get_logger(), "%s failed with Unitree SDK2 result %d.", name.c_str(), result);
        }
      }));
  }

  void send_velocity_command()
  {
    geometry_msgs::msg::Twist command;
    bool have_command = false;
    bool timed_out = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      have_command = have_command_;
      if (have_command) {
        command = last_command_;
        const auto command_age = std::chrono::steady_clock::now() - last_command_time_;
        timed_out = std::chrono::duration<double>(command_age).count() > cmd_timeout_;
      }
    }
    if (!have_command) {
      return;
    }
    if (timed_out) {
      if (stop_on_timeout_ && !timeout_stop_sent_) {
        const auto result = sport_client_->StopMove();
        timeout_stop_sent_ = true;
        RCLCPP_WARN(get_logger(), "cmd_vel timed out; sent Unitree SDK2 StopMove (result=%d).", result);
      }
      return;
    }

    timeout_stop_sent_ = false;
    const float vx = static_cast<float>(apply_deadband(clamp_abs(command.linear.x, max_linear_x_), deadband_));
    const float vy = static_cast<float>(apply_deadband(clamp_abs(command.linear.y, max_linear_y_), deadband_));
    const float vyaw = static_cast<float>(apply_deadband(clamp_abs(command.angular.z, max_angular_z_), deadband_));
    const auto result = sport_client_->Move(vx, vy, vyaw);
    if (result != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unitree SDK2 Move failed (result=%d).", result);
    }
  }

  void on_sport_state(const void * message)
  {
    std::lock_guard<std::mutex> lock(sport_state_mutex_);
    latest_sport_state_ = *static_cast<const SportState *>(message);
    sport_state_received_at_ = std::chrono::steady_clock::now();
    sport_state_ready_.store(true, std::memory_order_release);
  }

  void on_low_state(const void * message)
  {
    std::lock_guard<std::mutex> lock(low_state_mutex_);
    latest_low_state_ = *static_cast<const LowState *>(message);
    low_state_ready_.store(true, std::memory_order_release);
  }

  void publish_sport_state()
  {
    if (!sport_state_ready_.load(std::memory_order_acquire)) {
      return;
    }
    SportState state;
    {
      std::lock_guard<std::mutex> lock(sport_state_mutex_);
      state = latest_sport_state_;
    }

    const auto stamp = now();
    const auto & imu = state.imu_state();
    sensor_msgs::msg::Imu imu_message;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = imu_frame_;
    imu_message.orientation.w = imu.quaternion()[0];
    imu_message.orientation.x = imu.quaternion()[1];
    imu_message.orientation.y = imu.quaternion()[2];
    imu_message.orientation.z = imu.quaternion()[3];
    imu_message.angular_velocity.x = imu.gyroscope()[0];
    imu_message.angular_velocity.y = imu.gyroscope()[1];
    imu_message.angular_velocity.z = imu.gyroscope()[2];
    imu_message.linear_acceleration.x = imu.accelerometer()[0];
    imu_message.linear_acceleration.y = imu.accelerometer()[1];
    imu_message.linear_acceleration.z = imu.accelerometer()[2];
    imu_pub_->publish(imu_message);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = state.position()[0];
    odom.pose.pose.position.y = state.position()[1];
    odom.pose.pose.position.z = state.position()[2];
    odom.pose.pose.orientation = imu_message.orientation;
    odom.twist.twist.linear.x = state.velocity()[0];
    odom.twist.twist.linear.y = state.velocity()[1];
    odom.twist.twist.linear.z = state.velocity()[2];
    odom.twist.twist.angular.z = state.yaw_speed();
    odom_pub_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    std_msgs::msg::Int32 mode;
    mode.data = state.mode();
    mode_pub_->publish(mode);
  }

  void publish_low_state()
  {
    if (!low_state_ready_.load(std::memory_order_acquire)) {
      return;
    }
    LowState state;
    {
      std::lock_guard<std::mutex> lock(low_state_mutex_);
      state = latest_low_state_;
    }

    static constexpr std::array<int, 12> kMotorIndex{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    static constexpr std::array<const char *, 12> kJointNames{
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      const auto & motor = state.motor_state()[kMotorIndex[i]];
      joints.name.emplace_back(kJointNames[i]);
      joints.position.push_back(motor.q());
      joints.velocity.push_back(motor.dq());
      joints.effort.push_back(motor.tau_est());
    }
    joint_state_pub_->publish(joints);

    std_msgs::msg::Int16MultiArray foot_force;
    foot_force.data.assign(state.foot_force().begin(), state.foot_force().end());
    foot_force_pub_->publish(foot_force);

    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = joints.header.stamp;
    battery.voltage = state.power_v();
    battery.current = static_cast<float>(state.power_a());
    battery.percentage = static_cast<float>(state.bms_state().soc()) / 100.0F;
    int hottest = std::numeric_limits<int>::lowest();
    for (const auto temperature : state.bms_state().bq_ntc()) {
      hottest = std::max(hottest, static_cast<int>(temperature));
    }
    for (const auto temperature : state.bms_state().mcu_ntc()) {
      hottest = std::max(hottest, static_cast<int>(temperature));
    }
    battery.temperature = static_cast<float>(hottest);
    for (const auto millivolts : state.bms_state().cell_vol()) {
      battery.cell_voltage.push_back(static_cast<float>(millivolts) / 1000.0F);
    }
    battery_pub_->publish(battery);
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
  double max_linear_x_{0.8};
  double max_linear_y_{0.4};
  double max_angular_z_{1.2};
  double deadband_{0.01};
  double command_rate_hz_{20.0};
  double cmd_timeout_{0.5};
  bool stop_on_timeout_{true};
  bool auto_balance_stand_{false};
  double state_timeout_{0.5};
  bool have_command_{false};
  bool timeout_stop_sent_{false};
  geometry_msgs::msg::Twist last_command_;
  std::chrono::steady_clock::time_point last_command_time_{};
  std::chrono::steady_clock::time_point sport_state_received_at_{};
  std::mutex command_mutex_;
  std::mutex sport_state_mutex_;
  std::mutex low_state_mutex_;
  std::atomic<bool> sport_state_ready_{false};
  std::atomic<bool> low_state_ready_{false};
  SportState latest_sport_state_;
  LowState latest_low_state_;

  std::unique_ptr<unitree::robot::b2::SportClient> sport_client_;
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
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr low_state_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> motion_services_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<B2SdkDriver>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("b2_sdk_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
