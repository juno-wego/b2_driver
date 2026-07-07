#include <array>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
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
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "joint_states");
    status_topic_ = declare_parameter<std::string>("status_topic", "b2/status");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(10));
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_state_topic_, rclcpp::QoS(10),
      std::bind(&B2StateBridge::on_sport_state, this, std::placeholders::_1));
    low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
      low_state_topic_, rclcpp::QoS(10),
      std::bind(&B2StateBridge::on_low_state, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Subscribing B2 state: sport=%s, low=%s.",
      sport_state_topic_.c_str(), low_state_topic_.c_str());
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

    std_msgs::msg::String status;
    std::ostringstream data;
    data << std::fixed << std::setprecision(3)
         << "mode=" << static_cast<int>(msg->mode)
         << " gait=" << static_cast<int>(msg->gait_type)
         << " pos=[" << msg->position[0] << "," << msg->position[1] << "," << msg->position[2] << "]"
         << " vel=[" << msg->velocity[0] << "," << msg->velocity[1] << "," << msg->velocity[2] << "]"
         << " yaw_speed=" << msg->yaw_speed
         << " body_height=" << msg->body_height
         << " error_code=" << msg->error_code;
    status.data = data.str();
    status_pub_->publish(status);
  }

  void on_low_state(const unitree_go::msg::LowState::SharedPtr msg)
  {
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
  std::string status_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_tf_{true};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
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
