#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "b2_interface/msg/b2_motion_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_api/msg/request.hpp"

namespace
{
constexpr int32_t kApiDamp = 1001;
constexpr int32_t kApiBalanceStand = 1002;
constexpr int32_t kApiStopMove = 1003;
constexpr int32_t kApiStandDown = 1005;
constexpr int32_t kApiRecoveryStand = 1006;
constexpr int32_t kApiMove = 1008;

double clamp_abs(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

double apply_deadband(double value, double deadband)
{
  return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}

std::string normalize_command(std::string command)
{
  std::transform(
    command.begin(), command.end(), command.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return command;
}
}  // namespace

class B2CmdVelBridge : public rclcpp::Node
{
public:
  B2CmdVelBridge()
  : Node("b2_cmd_vel_bridge")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    motion_command_topic_ =
      declare_parameter<std::string>("motion_command_topic", "/b2/motion_command");
    request_topic_ = declare_parameter<std::string>("request_topic", "/api/sport/request");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.8);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.4);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.2);
    deadband_ = declare_parameter<double>("deadband", 0.01);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    stop_on_timeout_ = declare_parameter<bool>("stop_on_timeout", true);
    auto_balance_stand_ = declare_parameter<bool>("auto_balance_stand_on_start", false);

    request_pub_ = create_publisher<unitree_api::msg::Request>(request_topic_, rclcpp::QoS(10));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });
    motion_cmd_sub_ = create_subscription<b2_interface::msg::B2MotionCommand>(
      motion_command_topic_, rclcpp::QoS(10),
      std::bind(&B2CmdVelBridge::on_motion_command, this, std::placeholders::_1));

    make_service("balance_stand", kApiBalanceStand);
    make_service("recovery_stand", kApiRecoveryStand);
    make_service("stand_down", kApiStandDown);
    make_service("damp", kApiDamp);
    make_service("stop_move", kApiStopMove);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&B2CmdVelBridge::on_timer, this));

    if (auto_balance_stand_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        publish_motion_request(kApiBalanceStand);
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Sent BalanceStand request to B2.");
      });
    }

    RCLCPP_INFO(
      get_logger(), "Bridging cmd_vel=%s and motion_command=%s to %s.",
      cmd_vel_topic_.c_str(), motion_command_topic_.c_str(), request_topic_.c_str());
  }

private:
  void make_service(const std::string & name, int32_t api_id)
  {
    services_.push_back(create_service<std_srvs::srv::Trigger>(
      name,
      [this, api_id, name](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        publish_motion_request(api_id);
        response->success = true;
        response->message = "Published B2 sport request: " + name;
      }));
  }

  void publish_move(double vx, double vy, double vyaw)
  {
    nlohmann::json parameter;
    parameter["x"] = vx;
    parameter["y"] = vy;
    parameter["z"] = vyaw;

    unitree_api::msg::Request request;
    request.header.identity.api_id = kApiMove;
    request.parameter = parameter.dump();
    request_pub_->publish(request);
  }

  void publish_motion_request(
    int32_t api_id, const nlohmann::json & payload = nlohmann::json::object())
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = api_id;
    if (!payload.empty()) {
      request.parameter = payload.dump();
    }
    request_pub_->publish(request);
  }

  bool publish_named_command(const std::string & command)
  {
    const std::string normalized = normalize_command(command);
    if (normalized == "balance_stand") {
      publish_motion_request(kApiBalanceStand);
      return true;
    }
    if (normalized == "recovery_stand") {
      publish_motion_request(kApiRecoveryStand);
      return true;
    }
    if (normalized == "stand_down") {
      publish_motion_request(kApiStandDown);
      return true;
    }
    if (normalized == "damp") {
      publish_motion_request(kApiDamp);
      return true;
    }
    if (normalized == "stop_move") {
      have_cmd_ = false;
      sent_timeout_stop_ = true;
      publish_motion_request(kApiStopMove);
      return true;
    }

    return false;
  }

  void on_motion_command(const b2_interface::msg::B2MotionCommand::SharedPtr msg)
  {
    const std::string command = normalize_command(msg->command);
    if (command.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty B2 motion command.");
      return;
    }

    if (command == "move") {
      last_cmd_.linear.x = msg->linear_x;
      last_cmd_.linear.y = msg->linear_y;
      last_cmd_.linear.z = 0.0;
      last_cmd_.angular.x = 0.0;
      last_cmd_.angular.y = 0.0;
      last_cmd_.angular.z = msg->angular_z;
      last_cmd_time_ = now();
      have_cmd_ = true;
      sent_timeout_stop_ = false;
      return;
    }

    if (!publish_named_command(command)) {
      RCLCPP_WARN(get_logger(), "Unsupported B2 motion command: '%s'.", command.c_str());
    }
  }

  void on_timer()
  {
    if (!have_cmd_) {
      return;
    }

    const auto age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_) {
      if (stop_on_timeout_ && !sent_timeout_stop_) {
        publish_motion_request(kApiStopMove);
        sent_timeout_stop_ = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_vel timed out; sent StopMove.");
      }
      return;
    }

    sent_timeout_stop_ = false;
    const double vx = apply_deadband(clamp_abs(last_cmd_.linear.x, max_linear_x_), deadband_);
    const double vy = apply_deadband(clamp_abs(last_cmd_.linear.y, max_linear_y_), deadband_);
    const double vyaw = apply_deadband(clamp_abs(last_cmd_.angular.z, max_angular_z_), deadband_);
    publish_move(vx, vy, vyaw);
  }

  std::string cmd_vel_topic_;
  std::string motion_command_topic_;
  std::string request_topic_;
  double max_linear_x_{0.8};
  double max_linear_y_{0.4};
  double max_angular_z_{1.2};
  double deadband_{0.01};
  double command_rate_hz_{20.0};
  double cmd_timeout_{0.5};
  bool stop_on_timeout_{true};
  bool auto_balance_stand_{false};
  bool have_cmd_{false};
  bool sent_timeout_stop_{false};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<b2_interface::msg::B2MotionCommand>::SharedPtr motion_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<B2CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
