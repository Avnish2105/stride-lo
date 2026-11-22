#include "lo_core/preprocess/downsample.hpp"

#include <algorithm>
#include <cmath>

#include "lo_core/preprocess/validation.hpp"

namespace lo {

Preprocessor::Preprocessor(const Config & cfg)
: range_(cfg.range),
  vcfg_(cfg.voxel),
  sf_(cfg.self_filter),
  max_offset_ns_(static_cast<std::uint32_t>(fromSec(cfg.scan.max_point_offset))),
  quantum_(fromSec(cfg.scan.time_quantum)),
  phase_(fromSec(cfg.scan.time_quantum_phase)),
  initial_voxel_(std::clamp(cfg.voxel.initial, cfg.voxel.base_min, cfg.voxel.base_max)),
  voxel_(initial_voxel_),
  kept_(static_cast<std::size_t>(cfg.limits.max_raw_points)),
  ranges_(static_cast<std::size_t>(cfg.limits.max_raw_points)),
  voxels_(static_cast<std::size_t>(cfg.limits.max_raw_points))
{
}

PreprocessStats Preprocessor::run(
  const RawScan & scan, TimeNs t_ref, const SelfFilter & self, Cloud & out)
{
  PreprocessStats st;
  out.clear();
  const double rmin2 = range_.min * range_.min;
  const double rmax2 = range_.max * range_.max;
  const double near2 = sf_.blocked_radius * sf_.blocked_radius;
  const std::size_t n = std::min<std::size_t>(scan.size, kept_.size());
  std::size_t num_kept = 0;

  for (std::size_t i = 0; i < n; ++i) {
    const RawPoint & rp = scan.points[i];
    if (!ScanValidator::valid(rp) || rp.offset_ns > max_offset_ns_) {continue;}
    ++st.num_valid;
    const Vec3 p(rp.x, rp.y, rp.z);
    const double r2 = p.squaredNorm();
    if (r2 < near2) {++st.num_near;}
    if (self.enabled() && self.isSelf(p, pointTime(scan.t0 + static_cast<TimeNs>(rp.offset_ns)))) {
      ++st.num_self;
      continue;
    }
    if (r2 < rmin2 || r2 > rmax2) {continue;}
    if (range_.drop_zero_reflectivity && rp.reflectivity == 0) {continue;}
    if (range_.drop_noise_tag && (rp.tag & 0x03u) == 0x01u) {continue;}
    kept_[num_kept] = static_cast<std::int32_t>(i);
    ranges_[num_kept] = static_cast<float>(std::sqrt(r2));
    ++num_kept;
  }
  st.num_kept = static_cast<int>(num_kept);
  st.near_fraction = st.num_valid > 0 ?
    static_cast<double>(st.num_near) / static_cast<double>(st.num_valid) : 0.0;
  st.blocked = st.near_fraction > sf_.blocked_fraction;
  if (num_kept == 0) {
    st.voxel = voxel_;
    return st;
  }

  // Adaptive base voxel from the smoothed median range ("spaciousness").
  auto mid = ranges_.begin() + static_cast<std::ptrdiff_t>(num_kept / 2);
  std::nth_element(ranges_.begin(), mid, ranges_.begin() + static_cast<std::ptrdiff_t>(num_kept));
  st.median_range = *mid;
  if (!has_ema_) {
    ema_ = st.median_range;
    has_ema_ = true;
  } else {
    ema_ = vcfg_.ema_alpha * st.median_range + (1.0 - vcfg_.ema_alpha) * ema_;
  }
  voxel_ = std::clamp(vcfg_.base_scale * ema_, vcfg_.base_min, vcfg_.base_max);
  st.voxel = voxel_;

  // Voxel downsampling at 1.0 * v: first point per voxel, scan order (deterministic).
  const double inv = 1.0 / voxel_;
  voxels_.clear();
  for (std::size_t k = 0; k < num_kept; ++k) {
    const RawPoint & rp = scan.points[kept_[k]];
    const Vec3 p(rp.x, rp.y, rp.z);
    const Eigen::Vector3i c = voxelOf(p, inv);
    if (!voxels_.insert(packVoxel(c.x(), c.y(), c.z()))) {continue;}
    const TimeNs t = pointTime(scan.t0 + static_cast<TimeNs>(rp.offset_ns));
    if (!out.push(p, toSec(t - t_ref))) {break;}
  }

  // Thin uniformly if still above the registration budget (keeps every k-th point).
  const std::size_t cap = out.capacity();
  if (out.size() >= cap && num_kept > cap) {
    // Rare (huge open scans): re-run with stride over kept points instead of voxels.
    out.clear();
    const std::size_t stride = (num_kept + cap - 1) / cap;
    for (std::size_t k = 0; k < num_kept; k += stride) {
      const RawPoint & rp = scan.points[kept_[k]];
      const TimeNs t = pointTime(scan.t0 + static_cast<TimeNs>(rp.offset_ns));
      out.push(Vec3(rp.x, rp.y, rp.z), toSec(t - t_ref));
    }
  }
  st.num_registration = static_cast<int>(out.size());
  return st;
}

}  // namespace lo
