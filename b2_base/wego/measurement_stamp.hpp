// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace b2_base
{
struct MeasurementStamp
{
  int64_t source_ns{0}, stamp_ns{0}, received_ns{0};
  bool valid{false};
  const char * reason{"SOURCE_TIME_UNAVAILABLE"};
};

// Source timestamps are evidence, not receipt timestamps. A configured offset
// must be established outside the driver; one-way receipt latency cannot prove
// an absolute clock offset. No implicit calibration or receipt-time fallback.
class MeasurementClock
{
public:
  MeasurementClock(
    const std::string & mode, int64_t offset_ns = 0,
    double max_age_s = 0.5, double future_tolerance_s = 0.05)
  : receipt_(mode == "receipt"), offset_ns_(offset_ns),
    max_age_ns_(duration(max_age_s)), future_ns_(duration(future_tolerance_s))
  {
    if (mode != "source" && mode != "receipt") {
      throw std::invalid_argument("source_time.mode must be source or receipt");
    }
    if (receipt_ && offset_ns != 0) {
      throw std::invalid_argument("Receipt compatibility mode cannot use a source clock offset");
    }
  }

  MeasurementStamp update(
    int32_t sec, uint32_t nanosec, int64_t received_ns,
    int64_t steady_ns)
  {
    MeasurementStamp result;
    result.received_ns = received_ns;
    if (sec >= 0 && nanosec < 1000000000U) {
      result.source_ns = static_cast<int64_t>(sec) * 1000000000LL + nanosec;
    }
    const auto reject = [&result](const char * reason) {
        result.reason = reason; return result;
      };
    if (received_ns <= 0 || steady_ns <= 0) {return reject("HOST_TIME_INVALID");}
    if (last_received_ns_) {
      const long double jump = static_cast<long double>(received_ns) - last_received_ns_ -
        (static_cast<long double>(steady_ns) - last_steady_ns_);
      if (std::abs(jump) > 100000000.0L) {host_jump_ = true;}
    }
    last_received_ns_ = received_ns; last_steady_ns_ = steady_ns;
    if (host_jump_) {return reject("HOST_CLOCK_JUMP_RESTART_REQUIRED");}
    if (receipt_) {
      result.stamp_ns = received_ns;
      result.valid = true; result.reason = "RECEIPT_COMPATIBILITY_NOT_SYNCHRONIZED";
      return result;
    }
    if (result.source_ns <= 0) {return reject("SOURCE_TIME_INVALID");}
    const long double mapped = static_cast<long double>(result.source_ns) + offset_ns_;
    if (mapped <= 0 || mapped > static_cast<long double>(std::numeric_limits<int32_t>::max()) *
      1000000000LL + 999999999LL) {return reject("SOURCE_TIME_RANGE_INVALID");}
    result.stamp_ns = static_cast<int64_t>(mapped);
    if (result.source_ns <= last_source_ns_) {return reject("SOURCE_TIME_NOT_INCREASING");}
    const auto age = static_cast<long double>(received_ns) - result.stamp_ns;
    if (age > max_age_ns_) {return reject("SOURCE_TIME_STALE_OR_WRONG_EPOCH");}
    if (age < -future_ns_) {return reject("SOURCE_TIME_IN_FUTURE_OR_WRONG_EPOCH");}
    last_source_ns_ = result.source_ns;
    result.valid = true;
    result.reason = offset_ns_ == 0 ? "SOURCE_TIME_ACCEPTED" : "CONFIGURED_SOURCE_OFFSET_ACCEPTED";
    return result;
  }

private:
  static int64_t duration(double seconds)
  {
    if (!std::isfinite(seconds) || seconds < 0 || seconds > 60) {
      throw std::invalid_argument(
          "Source clock age limits must be finite and within 0..60 seconds");
    }
    return static_cast<int64_t>(std::llround(seconds * 1e9));
  }
  bool receipt_, host_jump_{false};
  int64_t offset_ns_, max_age_ns_, future_ns_, last_source_ns_{0};
  int64_t last_received_ns_{0}, last_steady_ns_{0};
};
}  // namespace b2_base
