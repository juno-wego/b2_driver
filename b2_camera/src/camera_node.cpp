// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include <net/if.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl/timer.h>
#include <rclcpp/rclcpp.hpp>
#include <unitree/robot/b2/back_video/back_video_client.hpp>
#include <unitree/robot/b2/front_video/front_video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include "b2_camera/image_conversion.hpp"

namespace b2_camera {
namespace {
sensor_msgs::msg::CameraInfo load_named_calibration(
  const std::string & resolved_url, const std::string & expected_name)
{
  std::string path;
  if (resolved_url.rfind("file:///", 0) == 0) {
    path = resolved_url.substr(7);
  } else if (resolved_url.rfind("package://", 0) == 0) {
    const auto separator = resolved_url.find('/', 10);
    if (separator == std::string::npos || separator == 10) {
      throw std::invalid_argument("camera_info_url must include a package and calibration file");
    }
    path = ament_index_cpp::get_package_share_directory(resolved_url.substr(10, separator - 10)) +
      resolved_url.substr(separator);
  } else {
    throw std::invalid_argument("camera_info_url must be an absolute file:// or package:// URL");
  }
  std::string camera_name;
  sensor_msgs::msg::CameraInfo info;
  if (!camera_calibration_parsers::readCalibration(path, camera_name, info)) {
    throw std::runtime_error("failed to read measured camera calibration: " + path);
  }
  if (camera_name != expected_name) {
    throw std::invalid_argument(
      "calibration camera_name must be '" + expected_name + "'; file declares '" + camera_name + "'");
  }
  std::string error;
  if (!valid_calibration(info, error)) {throw std::runtime_error(error);}
  return info;
}
}  // namespace

class CameraNode final : public rclcpp::Node {
public:
  CameraNode() : Node("b2_camera") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto interface = declare_parameter<std::string>("network_interface", "", fixed);
    const auto side = declare_parameter<std::string>("camera_side", "front", fixed);
    const auto domain = declare_parameter<int>("dds_domain", 0, fixed);
    const auto rate = declare_parameter<double>("publish_rate_hz", 5.0, fixed);
    const auto timeout = declare_parameter<double>("rpc_timeout_s", 0.5, fixed);
    const auto url = declare_parameter<std::string>("camera_info_url", "", fixed);
    require_calibration_ = declare_parameter<bool>("require_calibration", true, fixed);
    frame_ = declare_parameter<std::string>("frame_id", "", fixed);
    if (interface.empty() || ::if_nametoindex(interface.c_str()) == 0) {
      throw std::invalid_argument("network_interface must name an existing B2 Ethernet interface");
    }
    if (side != "front" && side != "back") {throw std::invalid_argument("camera_side must be front or back");}
    if (domain < 0 || domain > 232 || !std::isfinite(rate) || rate <= 0.0 || rate > 60.0 ||
      !std::isfinite(1e9 / rate) || 1e9 / rate >= static_cast<double>(std::numeric_limits<int64_t>::max()) ||
      !std::isfinite(timeout) || timeout <= 0.0 || timeout > 10.0)
    {
      throw std::invalid_argument("dds_domain must be 0..232, publish_rate_hz 0..60 and rpc_timeout_s 0..10 (positive)");
    }
    if (frame_.empty() || frame_.front() == '/' || frame_.find_first_of(" \t\r\n") != std::string::npos) {
      throw std::invalid_argument("frame_id must name the measured camera optical frame without a leading slash");
    }
    if (require_calibration_ && url.empty()) {
      throw std::invalid_argument("camera_info_url must reference measured calibration; use require_calibration=false only to collect calibration images");
    }
    const auto camera_name = "b2_" + side + "_camera";
    calibration_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name, url);
    if (!url.empty()) {
      if (!calibration_->validateURL(url)) {throw std::invalid_argument("invalid camera_info_url");}
      // CameraInfoManager only warns on a YAML camera_name mismatch. Enforce
      // the physical side here before initializing either SDK video client.
      // Preserve the configured save URL for the normal set_camera_info service.
      calibration_->setCameraInfo(load_named_calibration(
        calibration_->resolveURL(url, camera_name), camera_name));
    } else {
      // Empty URL normally triggers CameraInfoManager's lazy ~/.ros lookup.
      // Explicit collection mode starts uncalibrated even if an old file exists;
      // the normal set_camera_info service may still replace this in-memory data.
      calibration_->setCameraInfo(sensor_msgs::msg::CameraInfo{});
    }
    if (!require_calibration_) {
      RCLCPP_WARN(get_logger(), "Calibration collection mode; do not use this mode for metric perception");
    }
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
    // Integer microseconds prevent tiny positive seconds rounding to a zero
    // timeout in the SDK's float-to-microsecond conversion.
    const auto timeout_us = std::max<int64_t>(1, static_cast<int64_t>(std::ceil(timeout * 1e6)));
    if (side == "front") {
      front_ = std::make_unique<unitree::robot::b2::FrontVideoClient>();
      front_->SetTimeout(timeout_us);
      front_->Init();
      acquire_ = [this](std::vector<std::uint8_t> & sample) {return front_->GetImageSample(sample);};
    } else {
      back_ = std::make_unique<unitree::robot::b2::BackVideoClient>();
      back_->SetTimeout(timeout_us);
      back_->Init();
      acquire_ = [this](std::vector<std::uint8_t> & sample) {return back_->GetImageSample(sample);};
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, [this, period] {
      // One bounded SDK call at a time. Start the next interval after the call,
      // so a slow/timeout response cannot accumulate catch-up work.
      timer_->cancel();
      const auto bytes = capture_once();
      if (rclcpp::ok()) {
        // Stay below the XML flow controller's 65.536 MB/s transport budget.
        // Otherwise a high requested rate repeatedly evicts partly transmitted
        // frames from BEST_EFFORT history and can starve both camera receivers.
        // Default 1080p/5 Hz is unchanged; larger/faster streams are paced down.
        constexpr double image_budget_bytes_per_second = 50'000'000.0;
        const auto transfer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(static_cast<double>(bytes) / image_budget_bytes_per_second));
        const auto next_period = std::max(period, transfer_period);
        int64_t previous_period = 0;
        const auto result = rcl_timer_exchange_period(
          timer_->get_timer_handle().get(), next_period.count(), &previous_period);
        if (result != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(result, "camera polling period update failed");
        }
        timer_->reset();
      }
    });
    RCLCPP_INFO(get_logger(), "%s video (%s) → image_raw + camera_info; timestamps are SDK receipt-time estimates", side.c_str(), camera_name.c_str());
  }

  ~CameraNode() override {
    if (timer_) {timer_->cancel();}
    // There are no worker threads to join or further network calls here. The
    // single-threaded executor completes its current RPC using SetTimeout first.
    acquire_ = {};
  }

private:
  std::size_t capture_once() {
    try {
      std::vector<std::uint8_t> encoded;
      const auto status = acquire_(encoded);
      const auto received = now();  // GetImageSample supplies no sensor timestamp.
      if (status != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GetImageSample failed: %d; sample dropped", status);
        return 0;
      }
      std_msgs::msg::Header header;
      header.stamp = received;
      header.frame_id = frame_;
      std::string error;
      const auto pair = decode_sample(encoded, calibration_->getCameraInfo(), header, require_calibration_, error);
      if (!pair) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Camera sample dropped: %s", error.c_str());
        return 0;
      }
      if (rclcpp::ok()) {
        image_pub_->publish(pair->image);
        info_pub_->publish(pair->camera_info);
        return pair->image.data.size();
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Camera SDK/decode failure: %s; sample dropped", error.what());
    }
    return 0;
  }

  bool require_calibration_{true};
  std::string frame_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> calibration_;
  std::unique_ptr<unitree::robot::b2::FrontVideoClient> front_;
  std::unique_ptr<unitree::robot::b2::BackVideoClient> back_;
  std::function<int32_t(std::vector<std::uint8_t> &)> acquire_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace b2_camera

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<b2_camera::CameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("b2_camera"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
