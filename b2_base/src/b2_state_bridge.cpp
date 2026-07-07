#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "b2_interface/msg/b2_battery_state.hpp"
#include "b2_interface/msg/b2_status.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class B2StateBridge : public rclcpp::Node
{
public:
  B2StateBridge()
  : Node("b2_state_bridge")
  {
    sport_state_topic_ = declare_parameter<std::string>("sport_state_topic", "/sportmodestate");
    low_state_topic_ = declare_parameter<std::string>("low_state_topic", "/lowstate");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/b2/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/b2/imu");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/b2/joint_states");
    foot_force_topic_ = declare_parameter<std::string>("foot_force_topic", "/b2/foot_force");
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/b2/battery_state");
    status_topic_ = declare_parameter<std::string>("status_topic", "/b2/status");
    sport_state_output_topic_ =
      declare_parameter<std::string>("sport_state_output_topic", "/b2/sport_state");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos);
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, sensor_qos);
    foot_force_pub_ =
      create_publisher<std_msgs::msg::Int16MultiArray>(foot_force_topic_, sensor_qos);
    battery_pub_ =
      create_publisher<b2_interface::msg::B2BatteryState>(battery_topic_, reliable_qos);
    status_pub_ = create_publisher<b2_interface::msg::B2Status>(status_topic_, reliable_qos);
    sport_state_pub_ =
      create_publisher<unitree_go::msg::SportModeState>(sport_state_output_topic_, sensor_qos);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_state_topic_, sensor_qos,
      std::bind(&B2StateBridge::on_sport_state, this, std::placeholders::_1));
    low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
      low_state_topic_, sensor_qos,
      std::bind(&B2StateBridge::on_low_state, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Subscribing B2 state: sport=%s, low=%s | publishing odom=%s imu=%s joints=%s.",
      sport_state_topic_.c_str(), low_state_topic_.c_str(), odom_topic_.c_str(),
      imu_topic_.c_str(), joint_state_topic_.c_str());
  }

private:
  void on_sport_state(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    const auto stamp = now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];
    fill_quaternion(msg->imu_state.quaternion, odom.pose.pose.orientation);
    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    odom.twist.twist.angular.z = msg->yaw_speed;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    fill_quaternion(msg->imu_state.quaternion, imu.orientation);
    imu.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu.angular_velocity.z = msg->imu_state.gyroscope[2];
    imu.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu.linear_acceleration.z = msg->imu_state.accelerometer[2];
    imu.orientation_covariance[0] = -1.0;
    imu_pub_->publish(imu);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    b2_interface::msg::B2Status status;
    status.header.stamp = stamp;
    status.header.frame_id = base_frame_;
    status.mode = msg->mode;
    status.gait_type = msg->gait_type;
    status.error_code = msg->error_code;
    status.body_height = msg->body_height;
    status.position = msg->position;
    status.velocity = msg->velocity;
    status.yaw_speed = msg->yaw_speed;
    status.battery_soc = battery_soc_;
    status.battery_voltage = battery_voltage_;
    status.battery_current = battery_current_;
    status.low_state_available = have_low_state_;
    status_pub_->publish(status);
    sport_state_pub_->publish(*msg);
  }

  void on_low_state(const unitree_go::msg::LowState::SharedPtr msg)
  {
    have_low_state_ = true;
    battery_soc_ = msg->bms_state.soc;
    battery_voltage_ = msg->power_v;
    battery_current_ = msg->power_a;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = joint_names_;
    joint_state.position.resize(joint_names_.size());
    joint_state.velocity.resize(joint_names_.size());
    joint_state.effort.resize(joint_names_.size());

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      joint_state.position[i] = msg->motor_state[i].q;
      joint_state.velocity[i] = msg->motor_state[i].dq;
      joint_state.effort[i] = msg->motor_state[i].tau_est;
    }

    joint_state_pub_->publish(joint_state);

    std_msgs::msg::Int16MultiArray foot_force;
    foot_force.data.assign(msg->foot_force.begin(), msg->foot_force.end());
    foot_force_pub_->publish(foot_force);

    b2_interface::msg::B2BatteryState battery_state;
    battery_state.voltage = msg->power_v;
    battery_state.current = msg->power_a;
    battery_state.soc = msg->bms_state.soc;
    battery_state.temperature_ntc1 = msg->temperature_ntc1;
    battery_state.temperature_ntc2 = msg->temperature_ntc2;
    battery_pub_->publish(battery_state);
  }

  void fill_quaternion(
    const std::array<float, 4> & unitree_q,
    geometry_msgs::msg::Quaternion & ros_q) const
  {
    ros_q.w = unitree_q[0];
    ros_q.x = unitree_q[1];
    ros_q.y = unitree_q[2];
    ros_q.z = unitree_q[3];
  }

  const std::vector<std::string> joint_names_{
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  std::string sport_state_topic_;
  std::string low_state_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string joint_state_topic_;
  std::string foot_force_topic_;
  std::string battery_topic_;
  std::string status_topic_;
  std::string sport_state_output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_tf_{true};
  bool have_low_state_{false};
  uint8_t battery_soc_{0};
  float battery_voltage_{0.0F};
  float battery_current_{0.0F};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr foot_force_pub_;
  rclcpp::Publisher<b2_interface::msg::B2BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<b2_interface::msg::B2Status>::SharedPtr status_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_state_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<B2StateBridge>());
  rclcpp::shutdown();
  return 0;
}
