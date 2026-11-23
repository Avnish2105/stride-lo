#include <gtest/gtest.h>

#include "lo_core/types/flags.hpp"
#include "lo_ros/conversions.hpp"

using namespace lo_ros;
using Msg = lo_msgs::msg::LoFactor;

TEST(Conversions, FlagAndHealthConstantsMatchTheMessage)
{
  EXPECT_EQ(Msg::HEALTH_OK, lo::health::OK);
  EXPECT_EQ(Msg::HEALTH_DEGRADED, lo::health::DEGRADED);
  EXPECT_EQ(Msg::HEALTH_LOW_CONFIDENCE, lo::health::LOW_CONFIDENCE);
  EXPECT_EQ(Msg::FLAG_IMU_MISSING, lo::flags::IMU_MISSING);
  EXPECT_EQ(Msg::FLAG_DESKEW_FALLBACK, lo::flags::DESKEW_FALLBACK);
  EXPECT_EQ(Msg::FLAG_SEED_FALLBACK, lo::flags::SEED_FALLBACK);
  EXPECT_EQ(Msg::FLAG_SEED_NO_GRAPH, lo::flags::SEED_NO_GRAPH);
  EXPECT_EQ(Msg::FLAG_SEED_IDENTITY, lo::flags::SEED_IDENTITY);
  EXPECT_EQ(Msg::FLAG_SENSOR_BLOCKED, lo::flags::SENSOR_BLOCKED);
  EXPECT_EQ(Msg::FLAG_JOINTS_MISSING, lo::flags::JOINTS_MISSING);
  EXPECT_EQ(Msg::FLAG_DEGENERATE, lo::flags::DEGENERATE);
  EXPECT_EQ(Msg::FLAG_SPANS_GAP, lo::flags::SPANS_GAP);
  EXPECT_EQ(Msg::FLAG_AFTER_RECOVERY, lo::flags::AFTER_RECOVERY);
  EXPECT_EQ(Msg::FLAG_GYRO_GATE_DISABLED, lo::flags::GYRO_GATE_DISABLED);
  EXPECT_EQ(Msg::FLAG_REQUESTED, lo::flags::REQUESTED);
  EXPECT_EQ(Msg::FLAG_BIAS_UNCALIBRATED, lo::flags::BIAS_UNCALIBRATED);
}

TEST(Conversions, FactorRoundTripKeepsOrderAndQuaternion)
{
  lo::FactorOut f;
  f.t_i = 1234567890123LL;
  f.t_j = 1234667890123LL;
  f.chain_id = 77;
  lo::Vec6 xi;
  xi << 0.1, -0.2, 0.3, 1.0, 2.0, -3.0;
  f.T_ij = lo::expRT(xi);
  for (int r = 0; r < 6; ++r) {for (int c = 0; c < 6; ++c) {f.covariance(r, c) = r * 10 + c;}}
  f.degenerate_directions[0] << 0, 0, 0, 1, 0, 0;
  f.num_degenerate = 1;
  f.health = lo::health::DEGRADED;
  f.flags = lo::flags::DEGENERATE | lo::flags::REQUESTED;
  const Msg m = toMsg(f);
  EXPECT_EQ(m.covariance[0 * 6 + 3], 3.0);   // row-major: (rx, tx)
  EXPECT_EQ(m.covariance[3 * 6 + 0], 30.0);
  EXPECT_EQ(m.stamp_i.sec, 1234);
  EXPECT_EQ(m.stamp_i.nanosec, 567890123u);
  const lo::FactorOut g = fromMsg(m);
  EXPECT_TRUE(g.T_ij.matrix().isApprox(f.T_ij.matrix(), 1e-12));
  EXPECT_TRUE(g.covariance.isApprox(f.covariance));
  EXPECT_EQ(g.t_i, f.t_i);
  EXPECT_EQ(g.num_degenerate, 1);
  EXPECT_EQ(g.flags, f.flags);
  // ROS (x, y, z, w) vs Eigen (w, x, y, z)
  const lo::SE3 T = lo::fromXyzQuat(0, 0, 0, 0.0, 0.0, std::sin(0.25), std::cos(0.25));
  EXPECT_NEAR(lo::angleOf(T), 0.5, 1e-12);
  EXPECT_NEAR(toPose(T).orientation.w, std::cos(0.25), 1e-12);
}

TEST(Conversions, CustomMsgAndJointMapping)
{
  livox_ros_driver2::msg::CustomMsg msg;
  msg.timebase = 5000000000ULL;
  msg.points.resize(3);
  msg.points[1].x = 1.5f;
  msg.points[1].offset_time = 42;
  msg.points[1].reflectivity = 9;
  std::vector<lo::RawPoint> buf;
  buf.reserve(10);
  const lo::RawScan s = toRawScan(msg, buf);
  EXPECT_EQ(s.t0, 5000000000LL);
  ASSERT_EQ(s.size, 3u);
  EXPECT_EQ(s.points[1].offset_ns, 42u);
  EXPECT_FLOAT_EQ(s.points[1].x, 1.5f);

  JointMapper map({"LF_HAA", "LF_HFE"});
  sensor_msgs::msg::JointState js;
  js.name = {"LF_HFE", "XX", "LF_HAA"};
  js.position = {0.2, 9.0, 0.1};
  js.velocity = {2.0, 9.0, 1.0};
  lo::JointSample out;
  EXPECT_TRUE(map.convert(js, out));
  EXPECT_DOUBLE_EQ(out.q[0], 0.1);
  EXPECT_DOUBLE_EQ(out.q[1], 0.2);
  EXPECT_DOUBLE_EQ(out.qd[1], 2.0);
  EXPECT_TRUE(out.has_velocity);
}
