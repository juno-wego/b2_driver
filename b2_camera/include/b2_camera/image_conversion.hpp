#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace b2_camera {

struct FramePair {
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo camera_info;
};

bool valid_calibration(const sensor_msgs::msg::CameraInfo & info, std::string & error);

// The SDK provides no acquisition time. The caller supplies one receipt-time
// header, shared by both messages. Failure never returns a cached image.
std::optional<FramePair> decode_sample(
  const std::vector<std::uint8_t> & encoded,
  const sensor_msgs::msg::CameraInfo & calibration,
  const std_msgs::msg::Header & receipt_header,
  bool require_calibration, std::string & error);

}  // namespace b2_camera
