// Drive lo::Pipeline with a synthetic scenario (arrival-ordered messages, like a bag replay)
// and collect metrics. Used by lo_synth_run, the fault-suite test and the soak test.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/pipeline.hpp"
#include "lo_synth/evaluation.hpp"
#include "lo_synth/simulator.hpp"

namespace lo::synth {

struct RunOptions
{
  bool use_requests{true};      ///< emit.mode = request with the simulator's node requests
  bool use_kinematics{true};    ///< give the pipeline the robot model (self-filter)
  double t_max{-1.0};
  bool verbose{false};
  /// Called after every processed scan (e.g. hashing outputs for determinism tests).
  std::function<void(const ScanResult &)> on_scan;
};

struct RunOutput
{
  RunMetrics metrics;
  std::vector<PoseRecord> poses;
  std::vector<FactorRecord> factors;
  Diagnostics diagnostics;
};

/// Default config for synthetic runs (Livox internal IMU in g, interval timing instant).
Config syntheticConfig(const Scenario & sc, bool use_requests);

RunOutput runScenario(const Scenario & sc, const Config & cfg, const RunOptions & opt);

struct SoakReport
{
  int repetitions{0};
  double hours{0.0};                 ///< message time processed
  std::vector<double> rss_mb;        ///< resident set size after each repetition
  double rss_spread_pct{0.0};        ///< (max - min) / mean over the second half of the samples
  RunMetrics metrics;                ///< scans, factors, false accepts, timing, chain breaks
  Diagnostics diagnostics;
};

/// Soak (docs/DESIGN.md "8 h, memory flat"): ONE pipeline, the closed-loop scenario repeated
/// back to back in continuous message time until `hours` of data were processed.
SoakReport runSoak(const Scenario & sc, const Config & cfg, double hours,
  const std::function<void(const SoakReport &)> & progress = nullptr);

}  // namespace lo::synth
