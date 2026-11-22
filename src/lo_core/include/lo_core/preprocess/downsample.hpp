// S1 + S2 in one pass over the raw scan: validity, blockage monitor, self-filter, range gate,
// reflectivity/tag filters, adaptive base voxel v, voxel downsampling at 1.0 * v.
//
// Downsampling keeps the FIRST point of each voxel (scan order) instead of averaging: points in
// one voxel can be up to a scan period apart in time, so averaging before deskew would smear
// them. Each kept point keeps its own timestamp for S3.
#pragma once

#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/preprocess/self_filter.hpp"
#include "lo_core/types/points.hpp"
#include "lo_core/util/key_set.hpp"

namespace lo {

struct PreprocessStats
{
  int num_valid{0};
  int num_self{0};          ///< removed by the self-filter
  int num_near{0};          ///< valid returns within blocked_radius
  int num_kept{0};          ///< after range gate and filters
  int num_registration{0};  ///< after downsampling / thinning
  double near_fraction{0.0};
  double median_range{0.0};
  double voxel{0.0};        ///< base voxel v used for this scan
  bool blocked{false};
};

class Preprocessor
{
public:
  explicit Preprocessor(const Config & cfg);

  /// Fills `out` with registration points in the sensor frame (not deskewed); times relative
  /// to t_ref [s]. `self` may be disabled. Never allocates.
  PreprocessStats run(const RawScan & scan, TimeNs t_ref, const SelfFilter & self, Cloud & out);

  double voxel() const {return voxel_;}
  void reset() {has_ema_ = false; voxel_ = initial_voxel_;}

private:
  /// Point time used for deskew/self-filter (scan.time_quantum snapping for render-based sims).
  TimeNs pointTime(TimeNs t) const
  {
    if (quantum_ <= 0) {return t;}
    const TimeNs x = t - phase_ + quantum_ / 2;
    const TimeNs k = x >= 0 ? x / quantum_ : -((-x + quantum_ - 1) / quantum_);
    return phase_ + k * quantum_;
  }

  Config::Range range_;
  Config::Voxel vcfg_;
  Config::SelfFilter sf_;
  std::uint32_t max_offset_ns_;
  TimeNs quantum_;
  TimeNs phase_;
  double initial_voxel_;
  double voxel_;
  double ema_{0.0};
  bool has_ema_{false};
  std::vector<std::int32_t> kept_;   ///< raw indices that passed the filters
  std::vector<float> ranges_;        ///< their ranges (median scratch)
  KeySet voxels_;
};

}  // namespace lo
