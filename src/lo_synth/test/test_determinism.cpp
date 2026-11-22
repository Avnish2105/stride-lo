// Same input -> bit-identical output (docs/DESIGN.md "Deterministic replay").
#include <gtest/gtest.h>

#include <cstring>

#include "lo_synth/runner.hpp"

using namespace lo;
using namespace lo::synth;

namespace {
std::uint64_t fnv(std::uint64_t h, const void * data, std::size_t n)
{
  const auto * p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {h ^= p[i]; h *= 1099511628211ULL;}
  return h;
}

std::uint64_t runHash(const std::string & name, int threads)
{
  const Scenario sc = makeScenario(name, 3);
  Config cfg = syntheticConfig(sc, true);
  cfg.threads.num_threads = threads;
  std::uint64_t h = 1469598103934665603ULL;
  RunOptions opt;
  opt.t_max = 20.0;
  opt.on_scan = [&](const ScanResult & r) {
      h = fnv(h, &r.verdict, sizeof(r.verdict));
      h = fnv(h, r.T_odom_lidar.data(), 7 * sizeof(double));
      for (const lo::FactorOut & f : r.factors) {
        h = fnv(h, f.T_ij.data(), 7 * sizeof(double));
        h = fnv(h, f.covariance.data(), 36 * sizeof(double));
        h = fnv(h, &f.flags, sizeof(f.flags));
      }
    };
  runScenario(sc, cfg, opt);
  return h;
}
}  // namespace

TEST(Determinism, SameInputGivesBitIdenticalOutput)
{
  const std::uint64_t a = runHash("crowd", 2);
  const std::uint64_t b = runHash("crowd", 2);
  EXPECT_EQ(a, b);
}
