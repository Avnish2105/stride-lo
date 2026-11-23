// docs/DESIGN.md "Covariance model": write a known Sigma in, read it back through the adapter
// and check the marginals; rotation-first ordering and perturbation side are the two most
// common silent bugs, so both are checked numerically against lo_core's own conventions.
#include <gtest/gtest.h>

#include <random>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "lo_core/types/se3.hpp"
#include "lo_gtsam_adapter/adapter.hpp"

using gtsam::symbol_shorthand::X;

namespace {

gtsam::Pose3 toGtsam(const lo::SE3 & T)
{
  return gtsam::Pose3(gtsam::Rot3(T.so3().matrix()), gtsam::Point3(T.translation()));
}

lo::SE3 randomPose(std::mt19937 & rng, double rot = 1.0, double trans = 2.0)
{
  std::normal_distribution<double> n(0.0, 1.0);
  lo::Vec6 xi;
  xi << rot * n(rng), rot * n(rng), rot * n(rng), trans * n(rng), trans * n(rng), trans * n(rng);
  return lo::expRT(xi);
}

/// SPD covariance from a synthetic registration Hessian (rotation block stiffer, coupled).
lo::Mat6 randomCovariance(std::mt19937 & rng)
{
  std::normal_distribution<double> n(0.0, 1.0);
  lo::Mat6 A;
  for (int r = 0; r < 6; ++r) {for (int c = 0; c < 6; ++c) {A(r, c) = n(rng);}}
  lo::Mat6 H = A * A.transpose() + lo::Mat6::Identity() * 0.5;
  H.topLeftCorner<3, 3>() *= 400.0;   // ~mrad vs ~cm
  H.topRightCorner<3, 3>() *= 20.0;
  H.bottomLeftCorner<3, 3>() *= 20.0;
  return H.inverse() * 1e-2;
}

lo_gtsam::LoFactor makeMsg(const lo::SE3 & T, const lo::Mat6 & C, std::uint64_t chain = 1,
  std::int64_t ti = 1000000000, std::int64_t tj = 1100000000)
{
  lo_gtsam::LoFactor m;
  m.stamp_i.sec = static_cast<std::int32_t>(ti / 1000000000);
  m.stamp_i.nanosec = static_cast<std::uint32_t>(ti % 1000000000);
  m.stamp_j.sec = static_cast<std::int32_t>(tj / 1000000000);
  m.stamp_j.nanosec = static_cast<std::uint32_t>(tj % 1000000000);
  m.chain_id = chain;
  const Eigen::Quaterniond q = T.unit_quaternion();
  m.t_ij.position.x = T.translation().x();
  m.t_ij.position.y = T.translation().y();
  m.t_ij.position.z = T.translation().z();
  m.t_ij.orientation.w = q.w();
  m.t_ij.orientation.x = q.x();
  m.t_ij.orientation.y = q.y();
  m.t_ij.orientation.z = q.z();
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {m.covariance[static_cast<std::size_t>(r * 6 + c)] = C(r, c);}
  }
  return m;
}

}  // namespace

TEST(Adapter, GtsamTangentIsRotationFirstRightPerturbation)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 0.3);
  for (int k = 0; k < 20; ++k) {
    const lo::SE3 T = randomPose(rng);
    lo::Vec6 xi;
    xi << n(rng), n(rng), n(rng), n(rng), n(rng), n(rng);
    // same exponential and same adjoint in [rot, trans] order
    EXPECT_TRUE(gtsam::Pose3::Expmap(xi).matrix().isApprox(lo::expRT(xi).matrix(), 1e-9));
    EXPECT_TRUE(toGtsam(T).AdjointMap().isApprox(lo::adjointRT(T), 1e-9));
    // retract = right perturbation T * Exp(xi) (GTSAM built with POSE3_EXPMAP)
    EXPECT_TRUE(toGtsam(T).retract(xi).matrix().isApprox((T * lo::expRT(xi)).matrix(), 1e-9));
  }
}

TEST(Adapter, MarginalOfSecondNodeEqualsFactorCovariance)
{
  std::mt19937 rng(2);
  for (int k = 0; k < 5; ++k) {
    const lo::SE3 T0 = randomPose(rng), Tij = randomPose(rng, 0.2, 0.5);
    const lo::Mat6 C = randomCovariance(rng);
    const auto f = lo_gtsam::toBetweenFactor(makeMsg(Tij, C), X(0), X(1));
    ASSERT_TRUE(f.has_value());
    gtsam::NonlinearFactorGraph g;
    g.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), toGtsam(T0), gtsam::noiseModel::Isotropic::Sigma(6, 1e-9)));
    g.add(*f);
    gtsam::Values v;
    v.insert(X(0), toGtsam(T0));
    v.insert(X(1), toGtsam(T0 * Tij));
    const gtsam::Values est = gtsam::LevenbergMarquardtOptimizer(g, v).optimize();
    const gtsam::Matrix6 M = gtsam::Marginals(g, est).marginalCovariance(X(1));
    EXPECT_TRUE(M.isApprox(C, 1e-6)) << "marginal\n" << M << "\nexpected\n" << C;
  }
}

TEST(Adapter, FactorErrorIsMahalanobisOfRightPerturbation)
{
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  const lo::SE3 T0 = randomPose(rng), Tij = randomPose(rng, 0.2, 0.5);
  const lo::Mat6 C = randomCovariance(rng);
  const auto f = lo_gtsam::toBetweenFactor(makeMsg(Tij, C), X(0), X(1));
  ASSERT_TRUE(f.has_value());
  const Eigen::LLT<lo::Mat6> L(C);
  for (int k = 0; k < 10; ++k) {
    lo::Vec6 z;
    z << n(rng), n(rng), n(rng), n(rng), n(rng), n(rng);
    const lo::Vec6 delta = L.matrixL() * z;   // delta ~ N(0, C)
    gtsam::Values v;
    v.insert(X(0), toGtsam(T0));
    v.insert(X(1), toGtsam(T0 * Tij * lo::expRT(delta)));
    const double expected = 0.5 * delta.dot(C.ldlt().solve(delta));
    EXPECT_NEAR(f->error(v), expected, 1e-6 * std::max(1.0, expected));
  }
}

TEST(Adapter, BodyFrameChangeKeepsTheDistribution)
{
  // a perturbation delta of the base-frame T_ij is Ad(body_T_base) delta in the body frame:
  // the body-frame factor must score it exactly like the base-frame one
  std::mt19937 rng(4);
  std::normal_distribution<double> n(0.0, 1.0);
  const lo::SE3 T0 = randomPose(rng), Tij = randomPose(rng, 0.2, 0.5);
  const lo::SE3 body_T_base(lo::SO3::exp(lo::Vec3(0.1, -1.9, 0.3)), lo::Vec3(-0.35, 0.02, -0.07));
  const lo::Mat6 C = randomCovariance(rng);
  const auto msg = makeMsg(Tij, C);
  const auto f_base = lo_gtsam::toBetweenFactor(msg, X(0), X(1));
  const auto f_body = lo_gtsam::toBetweenFactor(msg, X(0), X(1), toGtsam(body_T_base));
  ASSERT_TRUE(f_base && f_body);
  const lo::SE3 base_T_body = body_T_base.inverse();
  for (int k = 0; k < 10; ++k) {
    lo::Vec6 z;
    z << n(rng), n(rng), n(rng), n(rng), n(rng), n(rng);
    const lo::Vec6 delta = Eigen::LLT<lo::Mat6>(C).matrixL() * z;
    const lo::SE3 T1 = T0 * Tij * lo::expRT(delta);
    gtsam::Values vb, vB;
    vb.insert(X(0), toGtsam(T0));
    vb.insert(X(1), toGtsam(T1));
    vB.insert(X(0), toGtsam(T0 * base_T_body));
    vB.insert(X(1), toGtsam(T1 * base_T_body));
    EXPECT_NEAR(f_body->error(vB), f_base->error(vb), 1e-6 * std::max(1.0, f_base->error(vb)));
  }
}

TEST(Adapter, RejectsCovarianceThatIsNotPositiveDefinite)
{
  lo::Mat6 C = lo::Mat6::Identity() * 1e-4;
  C(2, 2) = -1e-6;
  EXPECT_FALSE(lo_gtsam::toBetweenFactor(makeMsg(lo::SE3(), C), X(0), X(1)).has_value());
  C(2, 2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(lo_gtsam::toBetweenFactor(makeMsg(lo::SE3(), C), X(0), X(1)).has_value());
}

TEST(Adapter, ChainTrackerClassifiesFactors)
{
  using D = lo_gtsam::ChainTracker::Decision;
  lo_gtsam::ChainTracker t;
  const lo::Mat6 C = lo::Mat6::Identity() * 1e-4;
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 5, 100, 200)), D::FIRST);
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 5, 200, 300)), D::CONTINUE);
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 5, 400, 500)), D::GAP);           // 300 -> 400 lost
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 5, 400, 500)), D::OUT_OF_ORDER);  // duplicate
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 9, 900, 1000)), D::NEW_CHAIN);
  EXPECT_EQ(t.observe(makeMsg(lo::SE3(), C, 5, 500, 600)), D::STALE);
  EXPECT_EQ(t.breaks(), 1u);
  EXPECT_FALSE(lo_gtsam::ChainTracker::usable(D::STALE));
  EXPECT_TRUE(lo_gtsam::ChainTracker::usable(D::GAP));
}
