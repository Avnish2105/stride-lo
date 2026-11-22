// The fault-injection suite of docs/DESIGN.md on synthetic data (runs on every commit):
// injected faults must never yield a tight wrong factor (false accept = NEES > 40), and the
// nominal scenes must meet the accuracy / availability targets.
#include <gtest/gtest.h>

#include "lo_synth/runner.hpp"

using namespace lo;
using namespace lo::synth;

class FaultSuite : public ::testing::TestWithParam<std::string> {};

TEST_P(FaultSuite, NoFalseAccepts)
{
  const Scenario sc = makeScenario(GetParam(), 1);
  const Config cfg = syntheticConfig(sc, true);
  ASSERT_TRUE(validate(cfg).empty());
  RunOptions opt;
  const RunOutput out = runScenario(sc, cfg, opt);
  std::printf("%-14s %s\n", GetParam().c_str(), out.metrics.summary().c_str());
  EXPECT_EQ(out.metrics.false_accepts, 0);
  EXPECT_GT(out.metrics.factors, 0);
  if (sc.expect_degenerate) {
    // every factor of a degenerate scene must say so
    int degenerate = 0;
    for (const auto & f : out.factors) {degenerate += (f.f.num_degenerate > 0) ? 1 : 0;}
    EXPECT_GT(degenerate, out.metrics.factors * 9 / 10);
  }
}

INSTANTIATE_TEST_SUITE_P(Design, FaultSuite, ::testing::ValuesIn(faultSuiteNames()));

TEST(Nominal, RoomMeetsAccuracyTargets)
{
  const Scenario sc = makeScenario("room", 1);
  const Config cfg = syntheticConfig(sc, true);
  const RunOutput out = runScenario(sc, cfg, RunOptions());
  std::printf("room           %s\n", out.metrics.summary().c_str());
  EXPECT_EQ(out.metrics.false_accepts, 0);
  EXPECT_LT(out.metrics.rpe_trans_pct, 1.0);
  EXPECT_LT(out.metrics.rpe_rot_deg_per_m, 0.1);
  EXPECT_GE(out.metrics.availability, 0.98);
  EXPECT_EQ(out.metrics.chain_breaks, 0u);
}

TEST(Nominal, OverlapModeEmitsKeyframes)
{
  const Scenario sc = makeScenario("warehouse", 2);
  const Config cfg = syntheticConfig(sc, false);
  RunOptions opt;
  opt.use_requests = false;
  const RunOutput out = runScenario(sc, cfg, opt);
  EXPECT_EQ(out.metrics.false_accepts, 0);
  EXPECT_GT(out.metrics.factors, 5);
  EXPECT_LT(out.metrics.factors, out.metrics.scans / 2);   // keyframes, not every scan
}
