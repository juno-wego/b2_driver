// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include "b2_camera/image_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace b2_camera {
namespace {
template<typename Values>
bool finite(const Values & values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
bool absent_calibration(const sensor_msgs::msg::CameraInfo & info) {
  const auto zeros = [](const auto & values) {
      return std::all_of(values.begin(), values.end(), [](double value) { return value == 0.0; });
    };
  return zeros(info.k) && zeros(info.r) && zeros(info.p) && info.d.empty();
}
}  // namespace

bool valid_calibration(const sensor_msgs::msg::CameraInfo & info, std::string & error) {
  if (info.width == 0 || info.height == 0 || !finite(info.k) || !finite(info.r) ||
    !finite(info.p) || !finite(info.d) || info.k[0] <= 0 || info.k[4] <= 0 ||
    info.p[0] <= 0 || info.p[5] <= 0 || std::abs(info.k[8] - 1.0) > 1e-6 ||
    std::abs(info.p[10] - 1.0) > 1e-6 || info.distortion_model.empty())
  {
    error = "CameraInfo requires measured dimensions, finite K/D/R/P and valid focal/projection matrices";
    return false;
  }
  for (int row = 0; row < 3; ++row) {
    for (int other = row; other < 3; ++other) {
      double dot = 0.0;
      for (int column = 0; column < 3; ++column) {
        dot += info.r[row * 3 + column] * info.r[other * 3 + column];
      }
      if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1e-3) {
        error = "CameraInfo R must be a valid rectification rotation";
        return false;
      }
    }
  }
  const auto & r = info.r;
  const double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) -
    r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
  if (std::abs(determinant - 1.0) > 1e-3) {
    error = "CameraInfo R must be a proper rotation, not a reflection";
    return false;
  }
  error.clear();
  return true;
}

std::optional<FramePair> decode_sample(
  const std::vector<std::uint8_t> & encoded,
  const sensor_msgs::msg::CameraInfo & calibration,
  const std_msgs::msg::Header & receipt_header,
  bool require_calibration, std::string & error)
{
  error.clear();
  if (encoded.empty()) {
    error = "SDK returned an empty image sample";
    return std::nullopt;
  }
  cv::Mat decoded;
  try {
    decoded = cv::imdecode(encoded, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
  } catch (const cv::Exception & exception) {
    error = std::string("image decode failed: ") + exception.what();
    return std::nullopt;
  }
  if (decoded.empty() || decoded.type() != CV_8UC3) {
    error = "SDK image sample did not decode to BGR8";
    return std::nullopt;
  }
  if (receipt_header.frame_id.empty()) {
    error = "camera optical frame_id is required";
    return std::nullopt;
  }
  const bool calibrated = !absent_calibration(calibration);
  if (calibrated) {
    if (!valid_calibration(calibration, error)) {return std::nullopt;}
    if (calibration.width != static_cast<std::uint32_t>(decoded.cols) ||
      calibration.height != static_cast<std::uint32_t>(decoded.rows))
    {
      error = "decoded image dimensions differ from measured CameraInfo; no automatic scaling is allowed";
      return std::nullopt;
    }
  } else if (require_calibration) {
    error = "measured camera calibration is required";
    return std::nullopt;
  }
  FramePair pair;
  if (calibrated) {pair.camera_info = calibration;}
  pair.camera_info.header = receipt_header;
  pair.camera_info.width = static_cast<std::uint32_t>(decoded.cols);
  pair.camera_info.height = static_cast<std::uint32_t>(decoded.rows);
  pair.image.header = receipt_header;
  pair.image.width = pair.camera_info.width;
  pair.image.height = pair.camera_info.height;
  pair.image.encoding = sensor_msgs::image_encodings::BGR8;
  pair.image.is_bigendian = false;
  pair.image.step = pair.image.width * 3;
  pair.image.data.resize(static_cast<std::size_t>(pair.image.step) * pair.image.height);
  for (int row = 0; row < decoded.rows; ++row) {
    std::copy_n(decoded.ptr<std::uint8_t>(row), pair.image.step,
      pair.image.data.begin() + static_cast<std::size_t>(row) * pair.image.step);
  }
  return pair;
}
}  // namespace b2_camera
