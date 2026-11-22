// Rotation-first ordering and right perturbation: the doc's "two most common silent bugs".
#include <gtest/gtest.h>

#include "test_common.hpp"

using namespace lo;

TEST(Se3, ExpLogAreRotationFirst)
{
  Vec6 rot;
  rot << 0.1, -0.2, 0.3, 0.0, 0.0, 0.0;
  const SE3 R = expRT(rot);
  EXPECT_NEAR(R.translation().norm(), 0.0, 1e-12);
  EXPECT_NEAR(angleOf(R), rot.head<3>().norm(), 1e-12);
  Vec6 tr;
  tr << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0;
  const SE3 T = expRT(tr);
  EXPECT_NEAR(angleOf(T), 0.0, 1e-12);
  EXPECT_TRUE(T.translation().isApprox(Vec3(1, 2, 3), 1e-12));
  std::mt19937_64 rng(1);
  for (int i = 0; i < 50; ++i) {
    const SE3 X = test::randomPose(rng);
    EXPECT_TRUE(expRT(logRT(X)).matrix().isApprox(X.matrix(), 1e-9));
  }
}

TEST(Se3, AdjointMatchesConjugation)
{
  std::mt19937_64 rng(2);
  for (int i = 0; i < 50; ++i) {
    const SE3 T = test::randomPose(rng);
    Vec6 xi;
    xi << 1e-3, -2e-3, 0.5e-3, 1e-3, 2e-3, -1e-3;
    const SE3 lhs = T * expRT(xi) * T.inverse();
    const SE3 rhs = expRT(adjointRT(T) * xi);
    EXPECT_TRUE(lhs.matrix().isApprox(rhs.matrix(), 1e-9));
  }
}

TEST(Se3, CovarianceFrameChangeIsRightPerturbationConsistent)
{
  // xi_lidar on T_ij^lidar maps to Ad(T_bl) xi_lidar on T_ij^base = T_bl T_ij T_bl^-1.
  std::mt19937_64 rng(3);
  const SE3 T_bl = test::randomPose(rng, 0.5, 0.5);
  const SE3 T_ij = test::randomPose(rng, 0.2, 0.1);
  Vec6 xi;
  xi << 2e-4, -1e-4, 3e-4, 1e-3, -2e-3, 5e-4;
  const SE3 base_pert = T_bl * (T_ij * expRT(xi)) * T_bl.inverse();
  const SE3 base = T_bl * T_ij * T_bl.inverse();
  const Vec6 xi_b = logRT(base.inverse() * base_pert);
  EXPECT_TRUE(xi_b.isApprox(adjointRT(T_bl) * xi, 1e-6));
  // covariance of mapped samples == Ad cov Ad^T (linear map: exact)
  Mat6 cov = Mat6::Zero();
  cov.diagonal() << 1e-4, 2e-4, 3e-4, 1e-2, 2e-2, 3e-2;
  const Mat6 A = adjointRT(T_bl);
  EXPECT_TRUE(transportCovariance(T_bl, cov).isApprox(A * cov * A.transpose(), 1e-12));
}
