// Accuracy / consistency metrics against ground truth (docs/DESIGN.md "Validation and
// acceptance criteria"): RPE over 10 m segments, rotation error per metre, factor NEES,
// false accepts (NEES > 40, DECISIONS I10), availability, recovery time.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "lo_core/types/outputs.hpp"

namespace lo::synth {

struct PoseRecord
{
  TimeNs t{0};               ///< true time
  std::uint64_t chain{0};
  SE3 T_lo;                  ///< LO odom <- base
  SE3 T_gt;                  ///< world <- base
};

struct FactorRecord
{
  FactorOut f;
  SE3 T_gt_ij;               ///< ground-truth relative pose
  double nees{0.0};
  bool false_accept{false};
};

struct RunMetrics
{
  int scans{0}, accepted{0}, degraded{0}, rejected{0};
  int factors{0}, false_accepts{0};
  double mean_nees{0.0};
  double nees_trans_mean{0.0};   ///< 3-DoF NEES of the translation block (diagnostic)
  double rpe_trans_pct{0.0};     ///< mean over 10 m segments
  double rpe_rot_deg_per_m{0.0};
  int rpe_segments{0};
  double path_length{0.0};
  double availability{1.0};      ///< answered requests / requests
  double max_lost_s{0.0};        ///< longest time without an accepted scan
  double timing_median_ms{0.0}, timing_p99_ms{0.0}, timing_max_ms{0.0};
  std::uint64_t chain_breaks{0};
  std::string summary() const;
};

/// NEES of a factor against the true relative pose (base frame, right perturbation).
double factorNees(const FactorOut & f, const SE3 & T_gt_ij);

/// RPE over segments of `segment` metres (GT path length), within one chain only.
void relativeError(const std::vector<PoseRecord> & poses, double segment, double & trans_pct,
  double & rot_deg_per_m, int & segments);

}  // namespace lo::synth
