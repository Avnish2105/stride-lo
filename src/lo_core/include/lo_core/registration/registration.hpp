// S6: coarse -> fine GICP (docs/DESIGN.md S6, DECISIONS I5).
//  1. Coarse: coarse_iters Gauss-Newton steps against the coarse (4 v) map, wide gate, no
//     projection (basin widening).
//  2. Degeneracy of the fine problem at the coarse result; degenerate components of
//     (seed^-1 * coarse) are reset to the seed.
//  3. Fine: up to fine_iters steps against the fine (0.5 v) map with the adaptive gate tau,
//     Geman-McClure (tau/3), each update projected onto the observable subspace.
//  Converged when |d_rot| < rot_eps and |d_trans| < trans_eps.
#pragma once

#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/integrity/degeneracy.hpp"
#include "lo_core/map/local_map.hpp"
#include "lo_core/registration/gicp_factor.hpp"

namespace lo {

struct AlignResult
{
  SE3 T;                         ///< odom <- lidar at the reference time
  bool converged{false};
  bool numerical_ok{true};
  bool blowup{false};            ///< a single step exceeded step_max (solver blow-up)
  int iterations_coarse{0};
  int iterations_fine{0};
  Mat6 H{Mat6::Zero()};          ///< fine-stage information at convergence (LiDAR frame)
  Mat6 Hp{Mat6::Zero()};         ///< point-to-plane Hessian at convergence (observability)
  Vec6 b{Vec6::Zero()};
  int num_points{0};
  int num_inliers{0};            ///< correspondences within tau
  int num_orientation_rejected{0};
  double sum_weighted_maha{0.0}; ///< sum w r^T W r over inliers
  double residual_rms{0.0};      ///< point-to-plane RMS over inliers [m]
  double max_step_trans{0.0};
  double max_step_rot{0.0};
  DegeneracyInfo degeneracy;     ///< of the final fine Hessian
};

/// KISS-ICP style adaptive correspondence threshold: tau = sigma_factor * sigma_dev, sigma_dev
/// from the deviation between seed and result, clamped to [gate.min, gate.max].
class AdaptiveGate
{
public:
  AdaptiveGate(const Config::Gate & g, double max_range) : g_(g), max_range_(max_range) {}

  double tau() const;
  /// deviation = seed^-1 * result; motion = translation since the previous scan [m].
  void update(const SE3 & deviation, double motion);
  void reset() {sse_ = 0.0; n_ = 0;}

private:
  Config::Gate g_;
  double max_range_;
  double sse_{0.0};
  std::uint64_t n_{0};
};

class Registration
{
public:
  Registration(const Config & cfg, std::size_t capacity);

  /// L: median range (degeneracy scaling). coarse_gate / coarse_iters: normal operation uses
  /// gate.max / gicp.coarse_iters, RECOVERING uses the wider recovery values.
  AlignResult align(
    const LocalMap & map, const Cloud & cloud, const SE3 & T_seed, double tau, double L,
    double coarse_gate, int coarse_iters);

private:
  /// Gauss-Newton on one map level. Returns iterations used.
  int optimize(
    const VoxelMap & map, const Cloud & cloud, SE3 & T, double gate, int max_iters,
    bool project, const SE3 * reset_seed, double L, double step_max_t, double step_max_r,
    AlignResult & out, bool & converged);
  void collectStats(const Cloud & cloud, AlignResult & out) const;
  /// Deterministic sum of w * jp jp^T over the current inliers (fixed blocks, thread order).
  Mat6 planeHessian(std::size_t n, int & inliers) const;

  Config cfg_;
  std::vector<GicpFactor> factors_;
  FactorParams params_;
  small_gicp::ParallelReductionOMP reduction_;
};

}  // namespace lo
