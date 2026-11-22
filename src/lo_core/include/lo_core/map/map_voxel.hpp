// Contents of one local-map voxel (VGICP style, DECISIONS I14): running sums of the inserted
// registration points, their S4 covariances (rotated into the map) and their view-oriented
// normals. The voxel Gaussian is (sum_pt / count, sum_cov / count). After max_points
// contributions it is frozen (like KISS-ICP's 20 points per voxel).
#pragma once

#include <cstdint>

#include <small_gicp/ann/knn_result.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>

#include "lo_core/types/points.hpp"

namespace lo {

struct MapVoxel
{
  struct Setting
  {
    int max_points = 20;
  };

  Vec3 sum_pt{Vec3::Zero()};
  Mat3 sum_cov{Mat3::Zero()};
  Eigen::Vector3f sum_normal{Eigen::Vector3f::Zero()};
  std::uint32_t count{0};

  /// Called by FixedIncrementalVoxelMap::insert (pt = T * point i).
  void add(const Setting & s, const Vec4 & pt, const Cloud & cloud, std::size_t i,
    const Eigen::Isometry3d & T)
  {
    if (count >= static_cast<std::uint32_t>(s.max_points)) {return;}
    const Mat3 R = T.linear();
    sum_pt += pt.head<3>();
    sum_cov += R * cloud.covs[i] * R.transpose();
    sum_normal += (R * cloud.normals[i]).cast<float>();
    ++count;
  }

  /// Merge another voxel's sums (re-binning), keeping the contribution cap.
  void merge(const MapVoxel & o, int max_points)
  {
    sum_pt += o.sum_pt;
    sum_cov += o.sum_cov;
    sum_normal += o.sum_normal;
    count += o.count;
    const auto cap = static_cast<std::uint32_t>(max_points);
    if (count > cap && count > 0) {
      const double s = static_cast<double>(cap) / static_cast<double>(count);
      sum_pt *= s;
      sum_cov *= s;
      sum_normal *= static_cast<float>(s);
      count = cap;
    }
  }

  void finalize() {}

  Vec3 mean() const {return count ? Vec3(sum_pt / static_cast<double>(count)) : Vec3::Zero();}
  Mat3 cov() const {return count ? Mat3(sum_cov / static_cast<double>(count)) : Mat3::Identity();}
  /// |mean normal| in [0, 1]: 1 = all contributions saw the surface from the same side.
  double normalConsistency() const
  {
    return count ? static_cast<double>(sum_normal.norm()) / static_cast<double>(count) : 0.0;
  }
  Vec3 normal() const
  {
    const float n = sum_normal.norm();
    return n > 1e-9f ? Vec3((sum_normal / n).cast<double>()) : Vec3::Zero();
  }
};

}  // namespace lo

namespace small_gicp::traits {

template<>
struct Traits<lo::MapVoxel>
{
  static std::size_t size(const lo::MapVoxel &) {return 1;}
  static bool has_points(const lo::MapVoxel &) {return true;}
  static bool has_covs(const lo::MapVoxel &) {return true;}
  static Eigen::Vector4d point(const lo::MapVoxel & v, std::size_t)
  {
    const Eigen::Vector3d m = v.mean();
    return Eigen::Vector4d(m.x(), m.y(), m.z(), 1.0);
  }
  static Eigen::Vector4d normal(const lo::MapVoxel & v, std::size_t)
  {
    const Eigen::Vector3d n = v.normal();
    return Eigen::Vector4d(n.x(), n.y(), n.z(), 0.0);
  }
  static Eigen::Matrix4d cov(const lo::MapVoxel & v, std::size_t)
  {
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m.block<3, 3>(0, 0) = v.cov();
    return m;
  }
  template<typename Result>
  static void knn_search(const lo::MapVoxel & v, const Eigen::Vector4d & pt, Result & result)
  {
    if (v.count == 0) {return;}
    result.push(0, (v.mean() - pt.head<3>()).squaredNorm());
  }
};

}  // namespace small_gicp::traits
