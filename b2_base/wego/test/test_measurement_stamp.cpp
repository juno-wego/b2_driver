// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

#include <gtest/gtest.h>
#include "measurement_stamp.hpp"

namespace
{
constexpr int64_t kEpoch = 1794430000000000000LL;
constexpr int32_t kSec = static_cast<int32_t>(kEpoch / 1000000000LL);

TEST(MeasurementClock, RetainsAcquisitionTimeDespiteDifferentReceiptDelays)
{
  b2_base::MeasurementClock clock("source");
  auto first = clock.update(kSec, 0, kEpoch + 20000000, 1000000000);
  auto second = clock.update(kSec, 5000000, kEpoch + 80000000, 1060000000);
  ASSERT_TRUE(first.valid); ASSERT_TRUE(second.valid);
  EXPECT_EQ(second.stamp_ns - first.stamp_ns, 5000000);
  EXPECT_NE(second.stamp_ns, second.received_ns);
}

TEST(MeasurementClock, RejectsMissingStaleFutureAndRepeatedSourceTimes)
{
  b2_base::MeasurementClock clock("source");
  EXPECT_FALSE(clock.update(0, 0, kEpoch, 1000000000).valid);
  EXPECT_FALSE(clock.update(kSec - 10, 0, kEpoch, 1000000000).valid);
  EXPECT_FALSE(clock.update(kSec + 10, 0, kEpoch, 1000000000).valid);
  EXPECT_FALSE(clock.update(kSec, 1000000000U, kEpoch, 1000000000).valid);
  ASSERT_TRUE(clock.update(kSec, 0, kEpoch, 1000000000).valid);
  EXPECT_FALSE(clock.update(kSec, 0, kEpoch, 1000000000).valid);
}

TEST(MeasurementClock, AnExplicitOffsetConvertsAnExistingClockWithoutRelabellingEachPacket)
{
  b2_base::MeasurementClock clock("source", kEpoch - 1000000000LL);
  auto result = clock.update(1, 5000000, kEpoch + 10000000, 1000000000);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source_ns, 1005000000);
  EXPECT_EQ(result.stamp_ns, kEpoch + 5000000);
}

TEST(MeasurementClock, AHostClockStepDoesNotSilentlyRebaseMeasurementHistory)
{
  b2_base::MeasurementClock clock("source");
  ASSERT_TRUE(clock.update(kSec, 0, kEpoch, 1000000000).valid);
  auto jumped = clock.update(kSec, 5000000, kEpoch + 1000000000, 1005000000);
  EXPECT_FALSE(jumped.valid);
  EXPECT_STREQ(jumped.reason, "HOST_CLOCK_JUMP_RESTART_REQUIRED");
  EXPECT_FALSE(clock.update(kSec, 10000000, kEpoch + 10000000, 1010000000).valid);
}

TEST(MeasurementClock, LegacyReceiptModeIsExplicitAndLabelled)
{
  b2_base::MeasurementClock clock("receipt");
  auto result = clock.update(0, 0, kEpoch, 1000000000);
  EXPECT_TRUE(result.valid); EXPECT_EQ(result.stamp_ns, kEpoch);
  EXPECT_STREQ(result.reason, "RECEIPT_COMPATIBILITY_NOT_SYNCHRONIZED");
  EXPECT_THROW(b2_base::MeasurementClock("auto"), std::invalid_argument);
}
}  // namespace
