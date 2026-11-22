// S5 seed: the graph odometry gate (DECISIONS I26). The graph is an external estimate that can
// diverge; it may seed and deskew only while plausible and consistent with LO's own motion.
#include <gtest/gtest.h>

#include "lo_core/registration/seed.hpp"

using namespace lo;

namespace {

/// Graph samples at 500 Hz moving along base x at `speed` (base = LiDAR frame here).
void fillGraph(GraphOdomBuffer & g, double t0, double t1, double speed, double t_bad = 1e9,
  double bad_speed = 1e6)
{
  for (double t = t0; t <= t1 + 1e-9; t += 0.002) {
    GraphOdomSample s;
    s.t = fromSec(t);
    const double v = t >= t_bad ? bad_speed : speed;
    const double x = speed * std::min(t, t_bad) + (t >= t_bad ? bad_speed * (t - t_bad) : 0.0);
    s.T_odom_base = SE3(SO3(), Vec3(x, 0.0, 0.0));
    s.v_base = Vec3(v, 0.0, 0.0);
    g.push(s);
  }
}

LoVelocity loVelocity(double speed, double t0, double t1)
{
  LoVelocity lv;
  lv.valid = true;
  lv.v_lidar = Vec3(speed, 0.0, 0.0);
  lv.t0 = fromSec(t0);
  lv.t1 = fromSec(t1);
  lv.gate_valid = true;   // the gate window (normally >= 0.4 s) is the same interval here
  lv.g0 = lv.t0;
  lv.g1 = lv.t1;
  lv.g_disp = lv.v_lidar * (t1 - t0);
  return lv;
}

}  // namespace

TEST(SeedGate, ConsistentGraphSeedsTranslationAndDeskew)
{
  Config cfg;
  Extrinsics ext;
  SeedModel seed(cfg, ext);
  GraphOdomBuffer g(cfg);
  fillGraph(g, 0.0, 1.0, 0.8);
  ImuBuffer imu(cfg);
  const SeedResult r = seed.predict(fromSec(0.8), fromSec(0.9), SE3(), imu, false, Vec3::Zero(), g,
    loVelocity(0.78, 0.7, 0.8));
  EXPECT_TRUE(r.translation_from_graph);
  EXPECT_FALSE(r.graph_gated);
  EXPECT_NEAR(r.delta.translation().x(), 0.08, 1e-6);
  EXPECT_NEAR(r.v_deskew.x(), 0.8, 1e-6);
  EXPECT_FALSE(r.flags & flags::SEED_NO_GRAPH);
}

TEST(SeedGate, DivergedGraphIsGatedWithHoldoff)
{
  Config cfg;
  cfg.seed.graph_holdoff = 2.0;
  cfg.limits.graph_buffer = 4096;   // 4 s at 500 Hz
  Extrinsics ext;
  SeedModel seed(cfg, ext);
  GraphOdomBuffer g(cfg);
  fillGraph(g, 0.0, 1.0, 0.8, 0.85);   // diverges at 0.85 s
  ImuBuffer imu(cfg);
  const LoVelocity lv = loVelocity(0.8, 0.7, 0.8);
  const SeedResult r = seed.predict(fromSec(0.8), fromSec(0.9), SE3(), imu, false, Vec3::Zero(), g, lv);
  EXPECT_FALSE(r.translation_from_graph);
  EXPECT_TRUE(r.graph_gated);
  EXPECT_TRUE(r.flags & flags::SEED_NO_GRAPH);
  EXPECT_NEAR(r.v_deskew.x(), 0.8, 1e-9);                  // deskew falls back to LO's velocity
  EXPECT_NEAR(r.delta.translation().x(), 0.08, 1e-9);     // seed too

  // a healthy graph again: still ignored during the hold-off, used after it
  GraphOdomBuffer g2(cfg);
  fillGraph(g2, 0.0, 4.0, 0.8);
  EXPECT_TRUE(seed.predict(fromSec(1.9), fromSec(2.0), SE3(), imu, false, Vec3::Zero(), g2,
    loVelocity(0.8, 1.8, 1.9)).graph_gated);
  EXPECT_TRUE(seed.predict(fromSec(3.0), fromSec(3.1), SE3(), imu, false, Vec3::Zero(), g2,
    loVelocity(0.8, 2.9, 3.0)).translation_from_graph);
}

TEST(SeedGate, InconsistentMeanVelocityIsGated)
{
  Config cfg;
  cfg.seed.graph_max_dev = 0.15;
  Extrinsics ext;
  SeedModel seed(cfg, ext);
  GraphOdomBuffer g(cfg);
  fillGraph(g, 0.0, 1.0, 1.2);   // plausible on its own, but LO measured 0.6 m/s
  ImuBuffer imu(cfg);
  const SeedResult r = seed.predict(fromSec(0.8), fromSec(0.9), SE3(), imu, false, Vec3::Zero(), g,
    loVelocity(0.6, 0.7, 0.8));
  EXPECT_TRUE(r.graph_gated);
  EXPECT_FALSE(r.translation_from_graph);
}
