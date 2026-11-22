#include "lo_synth/evaluation.hpp"

#include <cmath>
#include <cstdio>

namespace lo::synth {

double factorNees(const FactorOut & f, const SE3 & T_gt_ij)
{
  const Vec6 xi = logRT(T_gt_ij.inverse() * f.T_ij);
  const Eigen::LDLT<Mat6> ldlt(f.covariance);
  if (ldlt.info() != Eigen::Success) {return 1e9;}
  return xi.dot(ldlt.solve(xi));
}

void relativeError(const std::vector<PoseRecord> & poses, double segment, double & trans_pct,
  double & rot_deg_per_m, int & segments)
{
  trans_pct = 0.0;
  rot_deg_per_m = 0.0;
  segments = 0;
  const std::size_t n = poses.size();
  if (n < 2) {return;}
  std::vector<double> dist(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    dist[i] = dist[i - 1] + (poses[i].T_gt.translation() - poses[i - 1].T_gt.translation()).norm();
  }
  double sum_t = 0.0, sum_r = 0.0;
  std::size_t j = 0;
  for (std::size_t i = 0; i < n; i += 5) {   // segment starts every 5 poses
    if (j < i) {j = i;}
    while (j < n && dist[j] - dist[i] < segment) {++j;}
    if (j >= n) {break;}
    if (poses[j].chain != poses[i].chain) {continue;}
    const SE3 d_gt = poses[i].T_gt.inverse() * poses[j].T_gt;
    const SE3 d_lo = poses[i].T_lo.inverse() * poses[j].T_lo;
    const SE3 e = d_gt.inverse() * d_lo;
    const double len = dist[j] - dist[i];
    sum_t += e.translation().norm() / len * 100.0;
    sum_r += angleOf(e) * kRadToDeg / len;
    ++segments;
  }
  if (segments > 0) {
    trans_pct = sum_t / segments;
    rot_deg_per_m = sum_r / segments;
  }
}

std::string RunMetrics::summary() const
{
  char buf[640];
  std::snprintf(buf, sizeof(buf),
    "scans=%d acc=%d deg=%d rej=%d factors=%d false_accepts=%d meanNEES=%.2f (trans3 %.2f) "
    "RPE=%.3f%% %.4fdeg/m (%d segs, path %.1f m) avail=%.3f max_gap=%.2fs chain_breaks=%llu "
    "t_med=%.1fms t_p99=%.1fms t_max=%.1fms",
    scans, accepted, degraded, rejected, factors, false_accepts, mean_nees, nees_trans_mean,
    rpe_trans_pct, rpe_rot_deg_per_m, rpe_segments, path_length, availability, max_lost_s,
    static_cast<unsigned long long>(chain_breaks), timing_median_ms, timing_p99_ms, timing_max_ms);
  return buf;
}

}  // namespace lo::synth
