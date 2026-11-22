// S4: per-point covariances (docs/DESIGN.md S4). kNN (k = 10) once on the deskewed registration
// cloud with small_gicp's KdTree rebuilt in place (patch 0003, serial builder = deterministic
// layout). Each covariance is regularised to a plane: eigenvalues (1, 1, plane_eps). The normal
// (smallest eigenvector) is oriented toward the sensor origin, which is where the deskewed cloud
// is expressed. The covariances and normals stay with the points and are reused by the map.
#pragma once

#include "lo_core/config/config.hpp"
#include "lo_core/third_party/small_gicp_check.hpp"
#include "lo_core/types/points.hpp"

namespace lo {

class PointCovariance
{
public:
  /// `cloud` must outlive this object (the tree keeps a reference).
  PointCovariance(const Config & cfg, const Cloud & cloud);

  /// Rebuild the tree for the current cloud contents and fill cloud.covs / cloud.normals.
  void compute(Cloud & cloud);

private:
  int k_;
  double eps_;
  int threads_;
  small_gicp::UnsafeKdTree<Cloud> tree_;
};

}  // namespace lo
