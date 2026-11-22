// Coarse-to-fine GICP recovers a known motion against a Gaussian voxel map.
#include <gtest/gtest.h>

#include "lo_core/map/local_map.hpp"
#include "lo_core/registration/point_covariance.hpp"
#include "lo_core/registration/registration.hpp"
#include "test_common.hpp"

using namespace lo;

TEST(Registration, RecoversKnownMotionInARoom)
{
  Config cfg;
  cfg.limits.map_fine_capacity = 100000;
  cfg.limits.map_coarse_capacity = 20000;
  std::mt19937_64 rng(8);
  const auto world = test::roomPoints(10, 7, 3, 0.08, rng, 0.005);
  // map built from the world seen at the identity pose
  Cloud map_cloud(world.size());
  for (const auto & p : world) {map_cloud.push(p, 0.0);}
  PointCovariance pc_map(cfg, map_cloud);
  pc_map.compute(map_cloud);
  LocalMap map(cfg);
  map.reset(0.3);
  map.insert(map_cloud, SE3(), 0);
  // scan: the same world from a moved sensor, sparser
  Vec6 xi;
  xi << 0.01, -0.02, 0.05, 0.15, -0.08, 0.03;
  const SE3 T_true = expRT(xi);
  Cloud scan(world.size());
  for (std::size_t i = 0; i < world.size(); i += 3) {scan.push(T_true.inverse() * world[i], 0.0);}
  PointCovariance pc_scan(cfg, scan);
  pc_scan.compute(scan);
  Registration reg(cfg, scan.capacity());
  const AlignResult a = reg.align(map, scan, SE3(), 0.3, 3.0, cfg.gate.max, cfg.gicp.coarse_iters);
  ASSERT_TRUE(a.converged);
  EXPECT_EQ(a.degeneracy.num_degenerate, 0);
  const SE3 err = T_true.inverse() * a.T;
  EXPECT_LT(err.translation().norm(), 0.005);
  EXPECT_LT(angleOf(err) * kRadToDeg, 0.05);
  EXPECT_GT(static_cast<double>(a.num_inliers) / a.num_points, 0.8);
}
