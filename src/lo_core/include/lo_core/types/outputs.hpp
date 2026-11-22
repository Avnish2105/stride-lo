// Per-scan outputs of lo::Pipeline::process().
#pragma once

#include <array>
#include <cstdint>

#include "lo_core/types/flags.hpp"
#include "lo_core/types/se3.hpp"
#include "lo_core/types/time.hpp"

namespace lo {

/// One LoFactor (base frame, GTSAM tangent order, right perturbation).
struct FactorOut
{
  TimeNs t_i{0};
  TimeNs t_j{0};
  std::uint64_t chain_id{0};
  SE3 T_ij;                                ///< base_i -> base_j
  Mat6 covariance{Mat6::Identity()};       ///< [rx ry rz tx ty tz]
  std::uint8_t health{health::OK};
  Vec6 hessian_eigenvalues{Vec6::Zero()};  ///< ascending, normalised by point count
  std::array<Vec6, 3> degenerate_directions{};   ///< unit vectors, base frame; zero if unused
  int num_degenerate{0};
  float inlier_ratio{0.f};
  float residual_rms{0.f};
  std::uint32_t flags{0};
};

/// Factors produced by one pipeline call, in time order (late request answers first).
struct FactorList
{
  static constexpr int kCapacity = 16;
  std::array<FactorOut, kCapacity> items{};
  int size{0};

  bool empty() const {return size == 0;}
  bool full() const {return size >= kCapacity;}
  void clear() {size = 0;}
  void push(const FactorOut & f) {if (!full()) {items[static_cast<std::size_t>(size++)] = f;}}
  const FactorOut * begin() const {return items.data();}
  const FactorOut * end() const {return items.data() + size;}
  const FactorOut & back() const {return items[static_cast<std::size_t>(size - 1)];}
};

/// Wall-clock cost per stage [ms] (diagnostics only, never used in a decision).
struct StageTimings
{
  double validate{0}, self_filter{0}, downsample{0}, deskew{0}, covariance{0}, seed{0},
    registration{0}, integrity{0}, map{0}, emit{0}, total{0};
};

struct ScanResult
{
  TimeNs t_ref{0};                ///< reference time the scan was deskewed to
  TimeNs t_start{0};
  TimeNs t_end{0};
  Verdict verdict{Verdict::SKIPPED};
  RejectReason reason{RejectReason::NONE};
  HealthState state{HealthState::INIT};   ///< state after this scan
  std::uint64_t chain_id{0};
  bool pose_valid{false};
  SE3 T_odom_lidar;               ///< LO odom frame (debug only)
  SE3 T_odom_base;
  Vec3 v_base{Vec3::Zero()};      ///< debug twist (base axes)
  Vec3 w_base{Vec3::Zero()};
  FactorList factors;             ///< emitted by this call (late request answers come first)
  std::uint32_t flags{0};
  bool map_updated{false};
  int num_raw{0};
  int num_valid{0};
  int num_filtered{0};            ///< removed by the self-filter
  int num_registration{0};
  int iterations_coarse{0};
  int iterations_fine{0};
  int num_degenerate{0};
  Vec6 eigenvalues{Vec6::Zero()};  ///< normalised, scaled Hessian eigenvalues (fine stage)
  double inlier_ratio{0};
  double residual_rms{0};
  double voxel_size{0};           ///< adaptive base voxel v
  double gate{0};                 ///< adaptive tau used by the fine stage
  double near_fraction{0};        ///< share of returns within the blockage radius
  double seed_dev_trans{0};       ///< |registration - seed| [m] (debug)
  double seed_dev_rot_deg{0};
  double max_step_trans{0};       ///< largest solver step [m] (debug)
  double max_step_rot_deg{0};
  StageTimings timings;
};

}  // namespace lo
