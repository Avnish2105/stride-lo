#include <gtest/gtest.h>

#include "lo_core/emit/emitter.hpp"

using namespace lo;

namespace {

Config requestConfig()
{
  Config cfg;
  cfg.emit.mode = "request";
  return cfg;
}

/// Accepted scan [t0, t0 + 0.1] registered at t_ref = scan end, constant yaw rate w and
/// velocity v (LiDAR frame) as its deskew model.
FrameRecord * addScan(Emitter & e, double t0, const SE3 & T, double w = 0.0,
  const Vec3 & v = Vec3::Zero(), const Mat6 & cov = Mat6::Identity() * 1e-4,
  const Vec3 & lever = Vec3::Zero())
{
  FrameRecord & r = e.newRecord();
  r.T_odom_lidar = T;
  r.cov_lidar = cov;
  const TimeNs ts = fromSec(t0), te = fromSec(t0 + 0.1);
  r.setMotion(ts, te, te, fromSec(0.002), v, lever, [w](double dt) {return SO3::exp(Vec3(0, 0, w * dt));});
  e.noteProcessed(te);
  return &r;
}

}  // namespace

TEST(Emitter, RequestsAreSortedAndDeduped)
{
  Emitter e(requestConfig(), 100, SE3());
  for (double t : {1.31, 1.01, 1.11, 1.11, 1.21}) {e.addRequest(fromSec(t));}
  EXPECT_EQ(e.pendingRequests(), 4u);
  EXPECT_EQ(e.requestsReceived(), 4u);
  TimeNs t = 0;
  ASSERT_TRUE(e.firstRequestIn(fromSec(1.05), fromSec(1.15), t));
  EXPECT_EQ(t, fromSec(1.11));
  EXPECT_FALSE(e.firstRequestIn(fromSec(1.12), fromSec(1.20), t));
}

TEST(Emitter, AnswersOnTimeLateAndMissedRequestsInOrder)
{
  Emitter e(requestConfig(), 100, SE3());
  e.startChain(7);
  FactorList out;
  // scan A [1.0, 1.1], request on time at 1.05 -> anchor (no factor)
  e.addRequest(fromSec(1.05));
  addScan(e, 1.0, SE3());
  EXPECT_EQ(e.answerRequests(out), 0);
  EXPECT_TRUE(e.hasAnchor());
  // scan B [1.1, 1.2] processed before its request arrives; scan C rejected
  addScan(e, 1.1, SE3(SO3(), Vec3(0.1, 0, 0)));
  e.noteReject();
  e.noteProcessed(fromSec(1.3));
  // the late request for B and one inside the rejected C arrive now
  e.addRequest(fromSec(1.15));
  e.addRequest(fromSec(1.25));
  ASSERT_EQ(e.answerRequests(out), 1);
  EXPECT_EQ(out.items[0].t_i, fromSec(1.05));
  EXPECT_EQ(out.items[0].t_j, fromSec(1.15));
  EXPECT_EQ(out.items[0].chain_id, 7u);
  EXPECT_TRUE(out.items[0].flags & flags::REQUESTED);
  EXPECT_FALSE(out.items[0].flags & flags::SPANS_GAP);   // the reject came after B
  EXPECT_EQ(e.requestsMissed(), 1u);                    // 1.25 fell into the rejected scan
  // scan D: the factor B -> D spans the rejected scan
  out.clear();
  e.addRequest(fromSec(1.35));
  addScan(e, 1.3, SE3(SO3(), Vec3(0.3, 0, 0)));
  ASSERT_EQ(e.answerRequests(out), 1);
  EXPECT_TRUE(out.items[0].flags & flags::SPANS_GAP);
  EXPECT_EQ(out.items[0].health, health::LOW_CONFIDENCE);
  // a request behind the chain head is missed, never emitted out of order
  e.addRequest(fromSec(1.12));
  out.clear();
  EXPECT_EQ(e.answerRequests(out), 0);
  EXPECT_EQ(e.requestsMissed(), 2u);
  EXPECT_EQ(e.requestsAnswered(), 3u);   // the anchor counts: it is connected by the next factor
}

TEST(Emitter, OneRequestPerScan)
{
  // DECISIONS I12: two frames of one registration would be a factor of pure deskew motion
  Emitter e(requestConfig(), 100, SE3());
  e.startChain(1);
  FactorList out;
  addScan(e, 0.9, SE3());
  e.addRequest(fromSec(0.95));
  e.answerRequests(out);
  e.addRequest(fromSec(1.02));
  e.addRequest(fromSec(1.08));
  addScan(e, 1.0, SE3(SO3(), Vec3(0.1, 0, 0)));
  ASSERT_EQ(e.answerRequests(out), 1);
  EXPECT_EQ(out.items[0].t_j, fromSec(1.02));
  EXPECT_EQ(e.requestsMissed(), 1u);
}

TEST(Emitter, LateFrameUsesTheScanMotionModel)
{
  // frame at t inside the scan = T(t_ref) * (dR(t), v (t - t_ref)) (DECISIONS I23)
  Emitter e(requestConfig(), 100, SE3());
  e.startChain(1);
  FactorList out;
  addScan(e, 0.9, SE3());
  e.addRequest(fromSec(1.0));
  e.answerRequests(out);   // anchor at 1.0 (end of the first scan, identity)
  const double w = 0.8;
  const Vec3 v(1.0, 0.2, 0.0);
  const SE3 T(SO3::exp(Vec3(0, 0, 0.3)), Vec3(0.5, 0.1, 0.0));
  addScan(e, 1.0, T, w, v);   // t_ref = 1.1
  e.addRequest(fromSec(1.04));
  ASSERT_EQ(e.answerRequests(out), 1);
  const double dt = -0.06;
  const SE3 expected = T * SE3(SO3::exp(Vec3(0, 0, w * dt)), v * dt);
  EXPECT_TRUE(out.items[0].T_ij.matrix().isApprox(expected.matrix(), 1e-9));
  // covariance moved to the frame at t (right perturbation): Ad(D^-1) Sigma Ad(D^-1)^T
  const SE3 D(SO3::exp(Vec3(0, 0, w * dt)), v * dt);
  const Mat6 Ad = adjointRT(D.inverse());
  EXPECT_TRUE(out.items[0].covariance.isApprox(Ad * (Mat6::Identity() * 1e-4) * Ad.transpose(), 1e-9));
}

TEST(Emitter, LateFrameRotatesAboutTheBaseOrigin)
{
  // lever arm c: the LiDAR moves by (dR - I) c under a body rotation (DECISIONS I28)
  Emitter e(requestConfig(), 100, SE3());
  e.startChain(1);
  FactorList out;
  addScan(e, 0.9, SE3());
  e.addRequest(fromSec(1.0));
  e.answerRequests(out);
  const double w = 1.5;
  const Vec3 v(0.5, 0.0, 0.0), c(0.2, -0.1, 0.55);
  addScan(e, 1.0, SE3(), w, v, Mat6::Identity() * 1e-4, c);
  e.addRequest(fromSec(1.03));
  ASSERT_EQ(e.answerRequests(out), 1);
  const double dt = -0.07;
  const SO3 R = SO3::exp(Vec3(0, 0, w * dt));
  const SE3 expected(R, R * c - c + v * dt);
  EXPECT_TRUE(out.items[0].T_ij.matrix().isApprox(expected.matrix(), 1e-9));
}

TEST(Emitter, ExpiresRequestsFarBehindTheNewest)
{
  Config cfg = requestConfig();
  cfg.emit.request_max_age = 1.0;
  Emitter e(cfg, 100, SE3());
  e.addRequest(fromSec(1.0));
  e.addRequest(fromSec(1.5));
  e.addRequest(fromSec(2.2));
  EXPECT_EQ(e.pendingRequests(), 2u);
  EXPECT_EQ(e.requestsMissed(), 1u);
}

TEST(Emitter, OverlapModeAnchorThenFactorAndChainBreak)
{
  Config cfg;
  cfg.emit.mode = "overlap";
  const SE3 T_base_lidar(SO3::rotY(0.3), Vec3(0.6, 0.0, 0.1));
  Emitter e(cfg, 100, T_base_lidar);
  e.startChain(42);
  Cloud cloud(4);
  FactorList out;
  const FrameRecord * a = addScan(e, 1.0, SE3());
  EXPECT_FALSE(e.emitNewest(*a, cloud, 0.3, out));   // anchor
  const SE3 Tj(SO3::rotZ(0.1), Vec3(0.5, 0.0, 0.0));
  const FrameRecord * b = addScan(e, 1.1, Tj);
  ASSERT_TRUE(e.emitNewest(*b, cloud, 0.3, out));
  ASSERT_EQ(out.size, 1);
  // base frame: T_ij = T_bl T_odom_lidar_i^-1 T_odom_lidar_j T_bl^-1
  const SE3 expected = T_base_lidar * Tj * T_base_lidar.inverse();
  EXPECT_TRUE(out.items[0].T_ij.matrix().isApprox(expected.matrix(), 1e-9));
  const Mat6 Ad = adjointRT(T_base_lidar);
  EXPECT_TRUE(out.items[0].covariance.isApprox(Ad * (Mat6::Identity() * 1e-4) * Ad.transpose(), 1e-9));
  EXPECT_FALSE(out.items[0].flags & flags::REQUESTED);
  e.noteReject();
  const FrameRecord * c = addScan(e, 1.3, Tj * Tj);
  ASSERT_TRUE(e.emitNewest(*c, cloud, 0.3, out));
  EXPECT_TRUE(out.items[1].flags & flags::SPANS_GAP);
  // Sigma_ij = Ad(T_ij^-1) Sigma_i Ad(T_ij^-1)^T + Sigma_j
  const Mat6 Cb = Ad * (Mat6::Identity() * 1e-4) * Ad.transpose();
  const Mat6 C = transportCovariance(out.items[1].T_ij.inverse(), Cb) + Cb;
  EXPECT_TRUE(out.items[1].covariance.isApprox(0.5 * (C + C.transpose()), 1e-9));
  // after a chain break the next frame is an anchor again
  e.startChain(43);
  const FrameRecord * d = addScan(e, 2.0, SE3());
  EXPECT_FALSE(e.emitNewest(*d, cloud, 0.3, out));
}
