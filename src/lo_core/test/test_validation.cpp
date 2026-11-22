#include <gtest/gtest.h>

#include <vector>

#include "lo_core/preprocess/validation.hpp"

using namespace lo;

namespace {
std::vector<RawPoint> points(int n, std::uint32_t max_offset_ns = 99000000)
{
  std::vector<RawPoint> v(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    v[i].x = 1.0f + 0.001f * i;
    v[i].y = 2.0f;
    v[i].z = 0.5f;
    v[i].offset_ns = static_cast<std::uint32_t>(static_cast<std::uint64_t>(max_offset_ns) * i / n);
  }
  return v;
}
RawScan scan(TimeNs t0, const std::vector<RawPoint> & p) {return RawScan{t0, p.data(), p.size()};}
}  // namespace

TEST(ScanValidator, StampsPeriodsCountsAndOffsets)
{
  Config cfg;
  ScanValidator v(cfg);
  auto p = points(5000);
  EXPECT_EQ(v.check(scan(fromSec(1.0), p)).reason, RejectReason::NONE);
  EXPECT_EQ(v.check(scan(fromSec(1.1), p)).reason, RejectReason::NONE);
  const ScanCheck drop = v.check(scan(fromSec(1.3), p));        // one scan missing
  EXPECT_EQ(drop.reason, RejectReason::NONE);
  EXPECT_EQ(drop.dropped_scans, 1);
  EXPECT_EQ(v.check(scan(fromSec(1.2), p)).reason, RejectReason::STAMP_BACKWARDS);
  EXPECT_EQ(v.check(scan(fromSec(1.37), p)).reason, RejectReason::PERIOD_JUMP);  // 2 x 100 - 30 ms
  EXPECT_EQ(v.check(scan(fromSec(1.43), p)).reason, RejectReason::NONE);         // 60 ms: 1 period
  EXPECT_EQ(v.check(scan(fromSec(1.89), p)).reason, RejectReason::PERIOD_JUMP);  // +460 ms jump
  auto few = points(2000);
  EXPECT_EQ(v.check(scan(fromSec(1.99), few)).reason, RejectReason::TOO_FEW_POINTS);
  auto bad = points(5000, 150000000);   // offsets up to 150 ms
  EXPECT_EQ(v.check(scan(fromSec(2.09), bad)).reason, RejectReason::POINT_TIME_RANGE);
  // no-returns (0,0,0) are not valid points
  auto zeros = points(5000);
  for (size_t i = 0; i < 3000; ++i) {zeros[i].x = zeros[i].y = zeros[i].z = 0.0f;}
  const ScanCheck z = v.check(scan(fromSec(2.19), zeros));
  EXPECT_EQ(z.num_valid, 2000);
  EXPECT_EQ(z.reason, RejectReason::TOO_FEW_POINTS);
}

TEST(ClockHealth, SkewAgainstBaselineAndStableTime)
{
  Config cfg;
  ClockHealth c(cfg.clock);
  for (int i = 0; i < 50; ++i) {
    const TimeNs t = fromSec(0.1 * i);
    c.noteLidar(t, t + fromSec(0.004));
    c.noteImu(t, t + fromSec(0.001));
  }
  c.setBaseline();
  EXPECT_TRUE(c.evaluate(fromSec(5.0)));
  // the LiDAR clock moves 5 ms against the IMU
  for (int i = 50; i < 80; ++i) {
    const TimeNs t = fromSec(0.1 * i);
    c.noteLidar(t - fromSec(0.005), t + fromSec(0.004));
    c.noteImu(t, t + fromSec(0.001));
  }
  EXPECT_FALSE(c.evaluate(fromSec(8.0)));
  EXPECT_GT(std::abs(c.skew()), cfg.clock.max_skew);
  // external PTP offset beyond the limit is a fault too
  ClockHealth p(cfg.clock);
  p.notePtpOffset(fromSec(1.0), 0.003);
  EXPECT_FALSE(p.stable(fromSec(2.0)));
  EXPECT_TRUE(p.stable(fromSec(3.1)));
}
