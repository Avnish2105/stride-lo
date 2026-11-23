// S6 factor: GICP against Gaussian voxels, in small_gicp's factor concept (driven by the
// deterministic ParallelReductionOMP, patch 0002).
//   residual r = mu_voxel - T p,  W = (C_voxel + R C_p R^T)^-1
//   J = [R [p]x, -R]   (right perturbation T <- T Exp(delta), rotation first)
//   weight w = Geman-McClure(d; c = tau/3) on the point-to-plane distance d = |n . r|
//   (DECISIONS I4: the in-plane offset to the voxel mean is expected, not an outlier)
//   H = w J^T W J,  b = w J^T W r,  e = 1/2 w r^T W r
// Far-side check: if the voxel's normals agree (consistency >= min) and its mean normal points
// away from the source point's normal (dot < min_cos), the correspondence is dropped.
#pragma once

#include <limits>

#include "lo_core/map/local_map.hpp"
#include "lo_core/types/points.hpp"

namespace lo {

struct FactorParams
{
  double kernel_c{0.1};
  double min_cos{0.0};
  double min_consistency{0.5};
  /// Keep the correspondences of the previous linearisation (final fine iterations, so the
  /// Gauss-Newton converges on a fixed set instead of chattering between voxels).
  bool reuse{false};
};

struct GicpFactor
{
  static constexpr std::size_t kInvalid = std::numeric_limits<std::size_t>::max();

  struct Setting
  {
    const FactorParams * params{nullptr};
  };

  GicpFactor() = default;
  explicit GicpFactor(const Setting & s) : params(s.params) {}

  template<typename TargetPointCloud, typename SourcePointCloud, typename TargetTree,
    typename CorrespondenceRejector>
  bool linearize(
    const TargetPointCloud & target, const SourcePointCloud & source, const TargetTree & tree,
    const Eigen::Isometry3d & T, std::size_t source_index, const CorrespondenceRejector & rejector,
    Eigen::Matrix<double, 6, 6> * H, Eigen::Matrix<double, 6, 1> * b, double * e)
  {
    const Vec4 & ps4 = source.points[source_index];
    const Vec4 p4 = T * ps4;
    const Mat3 R = T.linear();
    if (params->reuse) {
      if (target_index == kInvalid) {return false;}
    } else {
      target_index = kInvalid;
      orientation_rejected = false;
      weight = 0.0;
      std::size_t k = 0;
      double d2 = 0.0;
      if (!small_gicp::traits::nearest_neighbor_search(tree, p4, &k, &d2) ||
        rejector(target, source, T, k, source_index, d2))
      {
        return false;
      }
      const MapVoxel & vk = target.voxels[target.voxel_id(k)];
      const Vec3 n_src = R * source.normals[source_index];
      if (vk.normalConsistency() >= params->min_consistency && n_src.squaredNorm() > 0.5 &&
        vk.normal().dot(n_src) < params->min_cos)
      {
        orientation_rejected = true;
        return false;
      }
      target_index = k;
    }
    const MapVoxel & v = target.voxels[target.voxel_id(target_index)];
    Vec3 n_ref = R * source.normals[source_index];
    if (v.normalConsistency() >= params->min_consistency) {n_ref = v.normal();}
    const Vec3 ps = ps4.head<3>();
    const Vec3 r = v.mean() - p4.head<3>();
    const Mat3 C = v.cov() + R * source.covs[source_index] * R.transpose();
    const Mat3 W = C.inverse();
    const double pd = std::abs(n_ref.dot(r));
    const double c2 = params->kernel_c * params->kernel_c;
    const double den = c2 + pd * pd;
    const double w = (c2 * c2) / (den * den);
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = R * skew(ps);
    J.block<3, 3>(0, 3) = -R;
    const Eigen::Matrix<double, 6, 3> JtW = J.transpose() * W;
    *H = w * (JtW * J);
    *b = w * (JtW * r);
    jp = (n_ref.transpose() * J).transpose();   // point-to-plane row (observability test)
    maha2 = r.dot(W * r);
    *e = 0.5 * w * maha2;
    weight = w;
    sq_dist = r.squaredNorm();
    plane_dist = pd;
    return true;
  }

  template<typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud &, const SourcePointCloud &, const Eigen::Isometry3d &) const
  {
    return 0.5 * weight * maha2;
  }

  bool inlier() const {return target_index != kInvalid;}

  const FactorParams * params{nullptr};
  std::size_t target_index{kInvalid};
  bool orientation_rejected{false};
  double weight{0.0};
  double maha2{0.0};
  double sq_dist{0.0};
  double plane_dist{0.0};
  Eigen::Matrix<double, 6, 1> jp{Eigen::Matrix<double, 6, 1>::Zero()};
};

}  // namespace lo
