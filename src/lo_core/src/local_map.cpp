#include "lo_core/map/local_map.hpp"

#include <algorithm>
#include <cmath>

namespace lo {

LocalMap::LocalMap(const Config & cfg)
: cfg_(cfg.map),
  map_ratio_(cfg.voxel.map_ratio),
  coarse_ratio_(cfg.voxel.coarse_ratio),
  rebin_threshold_(cfg.voxel.rebin_threshold),
  v_(std::clamp(cfg.voxel.initial, cfg.voxel.base_min, cfg.voxel.base_max)),
  fine_(cfg.voxel.map_ratio * v_, static_cast<std::size_t>(cfg.limits.map_fine_capacity)),
  coarse_(cfg.voxel.coarse_ratio * v_, static_cast<std::size_t>(cfg.limits.map_coarse_capacity)),
  scratch_info_(static_cast<std::size_t>(std::max(cfg.limits.map_fine_capacity,
    cfg.limits.map_coarse_capacity))),
  scratch_vox_(scratch_info_.size())
{
  fine_.voxel_setting.max_points = cfg.map.max_pts_voxel;
  coarse_.voxel_setting.max_points = cfg.map.max_pts_voxel;
  fine_.set_search_offsets(cfg.gicp.fine_search);
  coarse_.set_search_offsets(cfg.gicp.coarse_search);
}

void LocalMap::reset(double v)
{
  v_ = v;
  fine_.reset(map_ratio_ * v);
  coarse_.reset(coarse_ratio_ * v);
}

bool LocalMap::needsRebin(double v) const
{
  return std::abs(v - v_) > rebin_threshold_ * v_;
}

void LocalMap::rebinLevel(VoxelMap & map, double leaf)
{
  const std::size_t n = map.size();
  std::copy(map.infos.begin(), map.infos.begin() + static_cast<std::ptrdiff_t>(n), scratch_info_.begin());
  std::copy(map.voxels.begin(), map.voxels.begin() + static_cast<std::ptrdiff_t>(n), scratch_vox_.begin());
  map.reset(leaf);
  const int cap = map.voxel_setting.max_points;
  for (std::size_t i = 0; i < n; ++i) {
    const MapVoxel & old = scratch_vox_[i];
    if (old.count == 0) {continue;}
    const Vec3 m = old.mean();
    const Eigen::Vector3i c = map.coord_of(Vec4(m.x(), m.y(), m.z(), 1.0));
    const std::int64_t idx = map.insert_voxel(c);
    if (idx < 0) {continue;}
    map.voxels[idx].merge(old, cap);
    map.infos[idx].last_hit = std::max(map.infos[idx].last_hit, scratch_info_[i].last_hit);
    map.finalize_voxel(static_cast<std::size_t>(idx));
  }
}

void LocalMap::rebin(double v)
{
  rebinLevel(fine_, map_ratio_ * v);
  rebinLevel(coarse_, coarse_ratio_ * v);
  v_ = v;
  ++rebins_;
}

std::size_t LocalMap::insert(const Cloud & cloud, const SE3 & T_odom_lidar, TimeNs stamp)
{
  const Eigen::Isometry3d T = toIsometry(T_odom_lidar);
  std::size_t dropped = fine_.insert(cloud, T, stamp);
  dropped += coarse_.insert(cloud, T, stamp);
  return dropped;
}

void LocalMap::touch(const Cloud & cloud, const SE3 & T_odom_lidar, TimeNs stamp)
{
  const Eigen::Isometry3d T = toIsometry(T_odom_lidar);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const Vec4 p = T * cloud.points[i];
    const std::int64_t f = fine_.find(fine_.coord_of(p));
    if (f >= 0) {fine_.infos[f].last_hit = std::max(fine_.infos[f].last_hit, stamp);}
    const std::int64_t c = coarse_.find(coarse_.coord_of(p));
    if (c >= 0) {coarse_.infos[c].last_hit = std::max(coarse_.infos[c].last_hit, stamp);}
  }
}

std::size_t LocalMap::removeStale(TimeNs now, const Vec3 & center)
{
  const TimeNs min_stamp = now - fromSec(cfg_.ttl);
  return fine_.remove_stale(min_stamp, center, cfg_.radius) +
         coarse_.remove_stale(min_stamp, center, cfg_.radius);
}

std::size_t LocalMap::exportFine(std::vector<Vec3> & out, std::size_t max_points) const
{
  const std::size_t n = fine_.size();
  const std::size_t stride = std::max<std::size_t>(1, (n + max_points - 1) / std::max<std::size_t>(max_points, 1));
  std::size_t k = 0;
  for (std::size_t i = 0; i < n && k < out.size(); i += stride) {
    out[k++] = fine_.voxels[i].mean();
  }
  return k;
}

}  // namespace lo
