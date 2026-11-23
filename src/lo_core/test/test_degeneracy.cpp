#include <gtest/gtest.h>

#include "lo_core/config/config.hpp"
#include "lo_core/integrity/degeneracy.hpp"

using namespace lo;

namespace {
/// Point-to-plane Hessian of a corridor along x: walls y = +-1 (points over 0.2..2.6 m height),
/// floor z = 0 (points across the width).
void addPoint(Mat6 & H, const Vec3 & p, const Vec3 & n)
{
  Eigen::Matrix<double, 3, 6> J;
  J.block<3, 3>(0, 0) = skew(p);
  J.block<3, 3>(0, 3) = -Mat3::Identity();
  const Vec6 jp = (n.transpose() * J).transpose();
  H += jp * jp.transpose();
}

Mat6 corridorHp(int n_per_plane, double length, int & count)
{
  Mat6 H = Mat6::Zero();
  count = 0;
  for (int i = 0; i < n_per_plane; ++i) {
    const double x = -length / 2 + length * i / (n_per_plane - 1);
    const double z = 0.2 + 2.4 * ((i * 7) % 13) / 12.0;
    const double y = -0.9 + 1.8 * ((i * 5) % 11) / 10.0;
    addPoint(H, Vec3(x, 1.0, z), Vec3(0, -1, 0));
    addPoint(H, Vec3(x, -1.0, z), Vec3(0, 1, 0));
    addPoint(H, Vec3(x, y, 0.0), Vec3(0, 0, 1));
    count += 3;
  }
  return H;
}
}  // namespace

TEST(Degeneracy, CorridorAxisIsDegenerateAndHeldAtSeed)
{
  int N = 0;
  const Mat6 Hp = corridorHp(200, 20.0, N);
  const DegeneracyInfo d = analyzeDegeneracy(Hp, 2.0, N, 0.02, 3);
  ASSERT_TRUE(d.valid);
  ASSERT_EQ(d.num_degenerate, 1);
  std::array<Vec6, 3> dirs;
  ASSERT_EQ(degenerateDirections(d, dirs), 1);
  EXPECT_NEAR(std::abs(dirs[0][3]), 1.0, 1e-6);   // translation along x
  // the projected step and the reset never move along x
  Mat6 H = Hp + 1e-3 * Mat6::Identity();
  Vec6 b;
  b << 0.1, -0.2, 0.05, 3.0, -1.0, 0.5;
  const Vec6 step = projectedStep(H, b, d, 1e-9);
  EXPECT_NEAR(step[3], 0.0, 1e-9);
  Vec6 xi;
  xi << 0.01, 0.02, 0.03, 0.5, 0.1, 0.2;
  EXPECT_NEAR(removeDegenerate(xi, d)[3], 0.0, 1e-9);
  // covariance: sigma 1 m along x, the rest from kappa sigma2 H^-1 (+floor)
  Config cfg;
  const Mat6 cov = registrationCovariance(H, d, 1e-3, cfg);
  EXPECT_NEAR(std::sqrt(cov(3, 3)), cfg.degeneracy.sigma_trans, 1e-3);
  EXPECT_LT(std::sqrt(cov(4, 4)), 0.05);
  EXPECT_LT(std::sqrt(cov(5, 5)), 0.05);
}

TEST(Degeneracy, WellConstrainedHasNoDegenerateDirectionAndStepIsGaussNewton)
{
  int N = 0;
  Mat6 Hp = corridorHp(200, 20.0, N);
  // end walls x = +-3 (a room)
  for (int i = 0; i < 200; ++i) {
    for (double xs : {-3.0, 3.0}) {
      addPoint(Hp, Vec3(xs, -1.0 + 2.0 * i / 199, 0.2 + 2.4 * ((i * 3) % 7) / 6.0),
        Vec3(xs > 0 ? -1 : 1, 0, 0));
      ++N;
    }
  }
  const DegeneracyInfo d = analyzeDegeneracy(Hp, 2.0, N, 0.02, 3);
  EXPECT_EQ(d.num_degenerate, 0);
  Vec6 b;
  b << 0.1, -0.2, 0.05, 3.0, -1.0, 0.5;
  const Vec6 gn = -Hp.ldlt().solve(b);
  EXPECT_TRUE(projectedStep(Hp, b, d, 0.0).isApprox(gn, 1e-9));
}
