// lo_synth_run <scenario> [--seed N] [--overlap] [--verbose] [--tmax S] [--set name=value ...]
//              [--soak-hours H]   (one pipeline, the closed-loop scenario repeated for H hours)
// Runs one synthetic scenario through lo::Pipeline and prints the metrics. Scenarios:
// room warehouse corridor open_field stairs crowd turn180 imu_dropout clock_jump ptp_loss
// point_dropout lens_blocked leg_in_fov ; "suite" runs the 11 fault-injection scenarios.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_synth/param_override.hpp"
#include "lo_synth/runner.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: lo_synth_run <scenario|suite> [--seed N] [--overlap] [--verbose] "
      "[--tmax S] [--set name=value]... [--soak-hours H]\n");
    return 2;
  }
  std::string name = argv[1];
  std::uint64_t seed = 1;
  lo::synth::RunOptions opt;
  std::vector<std::string> overrides;
  double soak_hours = 0.0;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) {seed = std::strtoull(argv[++i], nullptr, 10);}
    else if (!std::strcmp(argv[i], "--overlap")) {opt.use_requests = false;}
    else if (!std::strcmp(argv[i], "--verbose")) {opt.verbose = true;}
    else if (!std::strcmp(argv[i], "--tmax") && i + 1 < argc) {opt.t_max = std::atof(argv[++i]);}
    else if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {overrides.push_back(argv[++i]);}
    else if (!std::strcmp(argv[i], "--soak-hours") && i + 1 < argc) {soak_hours = std::atof(argv[++i]);}
    else {std::fprintf(stderr, "unknown argument %s\n", argv[i]); return 2;}
  }
  std::vector<std::string> names = {name};
  if (name == "suite") {names = lo::synth::faultSuiteNames();}
  int total_false = 0;
  for (const auto & n : names) {
    const lo::synth::Scenario sc = lo::synth::makeScenario(n, seed);
    lo::Config cfg = lo::synth::syntheticConfig(sc, opt.use_requests);
    for (const auto & kv : overrides) {
      if (!lo::synth::applyOverride(cfg, kv)) {
        std::fprintf(stderr, "bad override %s\n", kv.c_str());
        return 2;
      }
    }
    const auto errors = lo::validate(cfg);
    if (!errors.empty()) {
      for (const auto & e : errors) {std::fprintf(stderr, "config: %s\n", e.c_str());}
      return 2;
    }
    if (soak_hours > 0.0) {
      int last_print = 0;
      const lo::synth::SoakReport rep = lo::synth::runSoak(sc, cfg, soak_hours,
          [&last_print](const lo::synth::SoakReport & r) {
            if (static_cast<int>(r.hours * 4.0) > last_print) {   // every 15 min of data
              last_print = static_cast<int>(r.hours * 4.0);
              std::printf("soak: %5.2f h  RSS %7.1f MB  scans %d  factors %d  false accepts %d\n",
                r.hours, r.rss_mb.back(), r.metrics.scans, r.metrics.factors, r.metrics.false_accepts);
              std::fflush(stdout);
            }
          });
      const bool flat = rep.rss_spread_pct <= 5.0;
      const auto & m = rep.metrics;
      std::printf("soak %s: %.2f h of data (%d repetitions)  RSS first %.1f MB last %.1f MB, spread "
        "%.2f %% over the second half -> %s\n", n.c_str(), rep.hours, rep.repetitions,
        rep.rss_mb.front(), rep.rss_mb.back(), rep.rss_spread_pct, flat ? "FLAT" : "NOT FLAT");
      std::printf("soak %s: scans %d accepted %d degraded %d rejected %d factors %d false accepts %d "
        "mean NEES %.2f chain breaks %llu losts %llu  t_med %.1f ms p99 %.1f ms max %.1f ms\n",
        n.c_str(), m.scans, m.accepted, m.degraded, m.rejected, m.factors, m.false_accepts,
        m.mean_nees, static_cast<unsigned long long>(m.chain_breaks),
        static_cast<unsigned long long>(rep.diagnostics.losts), m.timing_median_ms, m.timing_p99_ms,
        m.timing_max_ms);
      return (flat && m.false_accepts == 0) ? 0 : 1;
    }
    const lo::synth::RunOutput out = lo::synth::runScenario(sc, cfg, opt);
    std::printf("%-14s %s\n", n.c_str(), out.metrics.summary().c_str());
    const auto & d = out.diagnostics;
    std::printf("               reasons:");
    for (std::size_t r = 1; r < d.reject_reasons.size(); ++r) {
      if (d.reject_reasons[r]) {
        std::printf(" %s=%llu", lo::toString(static_cast<lo::RejectReason>(r)),
          static_cast<unsigned long long>(d.reject_reasons[r]));
      }
    }
    std::printf(" | losts=%llu recov=%llu fallbacks=%llu blocked=%llu joints_missing=%llu\n",
      static_cast<unsigned long long>(d.losts), static_cast<unsigned long long>(d.recoveries),
      static_cast<unsigned long long>(d.deskew_fallbacks),
      static_cast<unsigned long long>(d.blocked_scans),
      static_cast<unsigned long long>(d.joints_missing_scans));
    total_false += out.metrics.false_accepts;
  }
  return total_false == 0 ? 0 : 1;
}
