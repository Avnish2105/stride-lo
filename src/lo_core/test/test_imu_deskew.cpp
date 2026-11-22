#include <gtest/gtest.h>

#include "lo_core/config/config.hpp"
#include "lo_core/deskew/deskew.hpp"
#include "lo_core/deskew/imu_buffer.hpp"
#include "test_common.hpp"

using namespace lo;

namespace {
Vec3 omegaAt(double t) {return Vec3(0.3 + 0.5 * t, -0.2, 0.8 * std::sin(2.0 * t));}

/// Reference rotation by fine numerical integration of omegaAt (body frame).
SO3 integrate(double t0, double t1)
{
  SO3 R;
  const int n = 20000;
  const double h = (t1 - t0) / n;
  for (int i = 0; i < n; ++i) {R = R * SO3::exp(omegaAt(t0 + (i + 0.5) * h) * h);}
  return R;
}
}  // namespace

TEST(ImuBuffer, RelativeRotationConstantAngularAcceleration)
{
  Config cfg;
  ImuBuffer imu(cfg);
  for (int k = 0; k <= 400; ++k) {
    ImuSample s;
    s.t = fromSec(k * 0.005);
    s.gyro = omegaAt(k * 0.005);
    ASSERT_TRUE(imu.push(s));
  }
  SO3 R;
  ASSERT_TRUE(imu.relativeRotation(fromSec(0.1031), fromSec(0.6517), Vec3::Zero(), R));
  const SO3 ref = integrate(0.1031, 0.6517);
  EXPECT_LT(angleOf(R.inverse() * ref), 2e-5);   // rad
  // backwards = inverse
  SO3 Rb;
  ASSERT_TRUE(imu.relativeRotation(fromSec(0.6517), fromSec(0.1031), Vec3::Zero(), Rb));
  EXPECT_LT(angleOf(Rb * R), 1e-9);
  EXPECT_FALSE(imu.relativeRotation(fromSec(1.9), fromSec(2.1), Vec3::Zero(), R));
  EXPECT_NEAR(imu.maxGap(fromSec(0.1), fromSec(0.2)), 0.005, 1e-9);
}

TEST(ImuBuffer, IntervalMeanConventionPlacesSamplesMidInterval)
{
  Config cfg;
  cfg.imu.sample_convention = "interval_mean";
  ImuBuffer imu(cfg);
  ImuSample a, b;
  a.t = fromSec(1.0);
  b.t = fromSec(1.004);
  imu.push(a);
  imu.push(b);
  EXPECT_EQ(imu.ring().back().te, fromSec(1.002));
  ImuSample dup = b;
  EXPECT_FALSE(imu.push(dup));   // duplicate stamp dropped
}

TEST(Deskew, RecoversPointsOfAMovingSensor)
{
  // Sensor rotating (omegaAt, LiDAR = IMU frame) and translating at v: points of a static world
  // observed at their own times, deskewed to t_ref, must match the world seen at t_ref.
  Config cfg;
  Extrinsics ext;
  ImuBuffer imu(cfg);
  for (int k = 0; k <= 100; ++k) {
    ImuSample s;
    s.t = fromSec(k * 0.005);
    s.gyro = omegaAt(k * 0.005);
    imu.push(s);
  }
  const Vec3 v(1.2, -0.3, 0.1);   // LiDAR-frame velocity (constant in the moving frame)
  const double t0 = 0.12, t1 = 0.22, tr = 0.19;
  // reference trajectory on a fine grid (world <- sensor)
  const double h = 1e-5;
  const int steps = static_cast<int>(0.25 / h) + 2;
  std::vector<SO3> Rg(steps);
  std::vector<Vec3> pg(steps);
  for (int k = 1; k < steps; ++k) {
    const double tm = (k - 0.5) * h;
    Rg[k] = Rg[k - 1] * SO3::exp(omegaAt(tm) * h);
    pg[k] = pg[k - 1] + Rg[k - 1] * v * h;
  }
  auto pose = [&](double t) {
      const int k = static_cast<int>(std::lround(t / h));
      return SE3(Rg[k], pg[k]);
    };
  std::mt19937_64 rng(4);
  std::uniform_real_distribution<double> u(-8.0, 8.0);
  Cloud cloud(64);
  std::vector<Vec3> truth;
  const SE3 T_ref = pose(tr);
  for (int i = 0; i < 40; ++i) {
    const double t = t0 + (t1 - t0) * i / 39.0;
    const Vec3 world(u(rng), u(rng), u(rng) * 0.3);
    const SE3 T = pose(t);
    cloud.push(T.inverse() * world, t - tr);
    truth.push_back(T_ref.inverse() * world);
  }
  Deskewer d(cfg, ext);
  // the deskew model uses v expressed at t_ref; constant v in the body frame here
  ASSERT_EQ(d.prepare(imu, true, fromSec(t0), fromSec(t1), fromSec(tr), Vec3::Zero(), v,
    Vec3::Zero()), DeskewMode::IMU);
  d.apply(cloud, 1);
  double worst = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    worst = std::max(worst, (cloud.points[i].head<3>() - truth[i]).norm());
  }
  // rotation exact up to integration error; translation model is first order (v dt)
  EXPECT_LT(worst, 0.01);
}

TEST(Deskew, RotationAboutTheBaseOriginMovesTheLidarExactly)
{
  // Body rotating about the BASE origin with the LiDAR 0.6 m ahead (DECISIONS I28): the LiDAR
  // moves by (dR - I) c, not omega(t_ref) x r dt. Base velocity constant in the base frame.
  Config cfg;
  Extrinsics ext;
  ext.T_base_lidar = SE3(SO3::exp(Vec3(0.0, 1.9198621771937625, 0.0)), Vec3(0.6, 0.0, 0.08));
  const SO3 R_lb = ext.T_base_lidar.so3().inverse();
  ImuBuffer imu(cfg);
  for (int k = 0; k <= 100; ++k) {
    ImuSample s;
    s.t = fromSec(k * 0.005);
    s.gyro = R_lb * omegaAt(k * 0.005);   // IMU = LiDAR frame, body rate given in base axes
    imu.push(s);
  }
  const Vec3 vb(0.5, 0.05, 0.0);
  const double t0 = 0.12, t1 = 0.22, tr = 0.22;
  const double h = 1e-5;
  const int steps = static_cast<int>(0.25 / h) + 2;
  std::vector<SO3> Rg(steps);
  std::vector<Vec3> pg(steps);
  for (int k = 1; k < steps; ++k) {
    const double tm = (k - 0.5) * h;
    Rg[k] = Rg[k - 1] * SO3::exp(omegaAt(tm) * h);
    pg[k] = pg[k - 1] + Rg[k - 1] * vb * h;
  }
  auto lidarPose = [&](double t) {
      const int k = static_cast<int>(std::lround(t / h));
      return SE3(Rg[k], pg[k]) * ext.T_base_lidar;
    };
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> u(-8.0, 8.0);
  Cloud cloud(64);
  std::vector<Vec3> truth;
  const SE3 T_ref = lidarPose(tr);
  for (int i = 0; i < 40; ++i) {
    const double t = t0 + (t1 - t0) * i / 39.0;
    const Vec3 world(u(rng), u(rng), u(rng) * 0.3);
    cloud.push(lidarPose(t).inverse() * world, t - tr);
    truth.push_back(T_ref.inverse() * world);
  }
  Deskewer d(cfg, ext);
  ASSERT_EQ(d.prepare(imu, true, fromSec(t0), fromSec(t1), fromSec(tr), Vec3::Zero(), R_lb * vb,
    Vec3::Zero()), DeskewMode::IMU);
  d.apply(cloud, 1);
  double worst = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    worst = std::max(worst, (cloud.points[i].head<3>() - truth[i]).norm());
  }
  EXPECT_LT(worst, 0.005);
}
