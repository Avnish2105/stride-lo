// S8: two-level Gaussian voxel map (fine 0.5 v, coarse 4 v), bounded and allocation-free:
// fixed voxel pools (patched small_gicp FixedIncrementalVoxelMap), time-to-live and radius
// crop by message time, LRU eviction when full, re-binning when the adaptive base voxel v
// drifts by more than voxel.rebin_threshold.
#pragma once

#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/map/map_voxel.hpp"
#include "lo_core/third_party/small_gicp_check.hpp"

namespace lo {

using VoxelMap = small_gicp::FixedIncrementalVoxelMap<MapVoxel>;

class LocalMap
{
public:
  explicit LocalMap(const Config & cfg);

  /// Empty both levels and set their voxel sizes from base voxel v.
  void reset(double v);
  bool empty() const {return fine_.size() == 0;}
  double baseVoxel() const {return v_;}
  bool needsRebin(double v) const;
  /// Re-bin both levels to base voxel v, keeping their content.
  void rebin(double v);

  /// Insert a registration cloud (LiDAR frame at its reference time) at T_odom_lidar.
  /// Returns the number of points dropped for lack of space.
  std::size_t insert(const Cloud & cloud, const SE3 & T_odom_lidar, TimeNs stamp);

  /// Mark the voxels hit by the registered cloud as seen at `stamp` (time-to-live counts from
  /// the last time a voxel was matched or inserted, so a long standstill keeps the map).
  void touch(const Cloud & cloud, const SE3 & T_odom_lidar, TimeNs stamp);

  /// Time-to-live and radius crop around `center` (odom frame).
  std::size_t removeStale(TimeNs now, const Vec3 & center);

  const VoxelMap & fine() const {return fine_;}
  const VoxelMap & coarse() const {return coarse_;}
  std::size_t size() const {return fine_.size();}
  std::uint64_t rebins() const {return rebins_;}

  /// Voxel means of the fine level (debug output), at most max_points, stride sampled.
  std::size_t exportFine(std::vector<Vec3> & out, std::size_t max_points) const;

private:
  void rebinLevel(VoxelMap & map, double leaf);

  Config::Map cfg_;
  double map_ratio_;
  double coarse_ratio_;
  double rebin_threshold_;
  double v_;
  VoxelMap fine_;
  VoxelMap coarse_;
  std::vector<small_gicp::FixedVoxelInfo> scratch_info_;
  std::vector<MapVoxel> scratch_vox_;
  std::uint64_t rebins_{0};
};

}  // namespace lo
