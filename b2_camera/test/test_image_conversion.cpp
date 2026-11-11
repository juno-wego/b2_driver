#include "b2_camera/image_conversion.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <opencv2/imgcodecs.hpp>

namespace {
void check(bool condition, const char * message) {
  if (!condition) {std::cerr << "FAILED: " << message << '\n'; std::exit(1);}
}
sensor_msgs::msg::CameraInfo measured_info() {
  sensor_msgs::msg::CameraInfo info;
  info.width = 32; info.height = 24;
  info.distortion_model = "plumb_bob";
  info.d = {0.1, -0.02, 0.0, 0.0, 0.0};
  info.k = {30, 0, 16, 0, 31, 12, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {29, 0, 16, 0, 0, 30, 12, 0, 0, 0, 1, 0};
  info.header.frame_id = "old_frame";
  info.header.stamp.sec = 1;
  return info;
}
std::vector<std::uint8_t> jpeg() {
  cv::Mat source(24, 32, CV_8UC3, cv::Scalar(20, 70, 180));
  std::vector<std::uint8_t> encoded;
  check(cv::imencode(".jpg", source, encoded), "test JPEG encoding");
  return encoded;
}
}  // namespace

int main() {
  const auto encoded = jpeg();
  auto info = measured_info();
  std_msgs::msg::Header received;
  received.frame_id = "f_oc_link";
  received.stamp.sec = 101;
  received.stamp.nanosec = 234;
  std::string error;
  const auto pair = b2_camera::decode_sample(encoded, info, received, true, error);
  check(pair.has_value(), "valid JPEG and measured calibration accepted");
  check(pair->image.width == 32 && pair->image.height == 24 && pair->image.step == 96,
    "decoded dimensions and packed BGR stride");
  check(pair->image.encoding == "bgr8" && pair->image.data.size() == 32 * 24 * 3,
    "explicit BGR8 and matching byte count");
  check(pair->image.header == received && pair->camera_info.header == received,
    "image and CameraInfo share one receipt timestamp and optical frame");
  check(pair->camera_info.k == info.k && pair->camera_info.d == info.d && pair->camera_info.p == info.p,
    "measured intrinsics and distortion are preserved");
  check(info.header.frame_id == "old_frame" && info.header.stamp.sec == 1,
    "calibration input remains unchanged");
  check(!b2_camera::decode_sample({}, info, received, true, error), "empty response rejected");
  check(!b2_camera::decode_sample({1, 2, 3, 4}, info, received, true, error), "invalid JPEG rejected without old frame reuse");
  auto mismatch = info; mismatch.width = 64;
  check(!b2_camera::decode_sample(encoded, mismatch, received, true, error), "wrong resolution rejected without scaling");
  mismatch = info; mismatch.height = 48;
  check(!b2_camera::decode_sample(encoded, mismatch, received, false, error), "collection mode cannot bypass loaded calibration mismatch");
  auto invalid = info; invalid.k[0] = std::numeric_limits<double>::quiet_NaN();
  check(!b2_camera::decode_sample(encoded, invalid, received, true, error), "nonfinite K rejected");
  invalid = info; invalid.p[5] = 0;
  check(!b2_camera::decode_sample(encoded, invalid, received, true, error), "missing projection focal length rejected");
  invalid = info; invalid.d[0] = std::numeric_limits<double>::infinity();
  check(!b2_camera::decode_sample(encoded, invalid, received, true, error), "nonfinite distortion rejected");
  invalid = info; invalid.r.fill(0);
  check(!b2_camera::decode_sample(encoded, invalid, received, true, error), "invalid rectification rotation rejected");
  invalid = info; invalid.r[0] = -1;
  check(!b2_camera::decode_sample(encoded, invalid, received, true, error), "reflection is not a valid rectification rotation");
  sensor_msgs::msg::CameraInfo absent;
  check(!b2_camera::decode_sample(encoded, absent, received, true, error), "metric operation requires measured calibration");
  const auto collection = b2_camera::decode_sample(encoded, absent, received, false, error);
  check(collection && collection->camera_info.k[0] == 0 && collection->camera_info.width == 32 &&
    collection->camera_info.height == 24 && collection->camera_info.header == received,
    "explicit collection mode publishes standard uncalibrated CameraInfo with correct dimensions/header");
  invalid = absent; invalid.k[0] = 30;
  check(!b2_camera::decode_sample(encoded, invalid, received, false, error), "partial calibration is not treated as absent");
  received.frame_id.clear();
  check(!b2_camera::decode_sample(encoded, info, received, true, error), "anonymous optical frame rejected");
  std::cout << "b2_camera image/calibration checks passed\n";
  return 0;
}
