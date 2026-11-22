#include "lo_core/registration/point_covariance.hpp"

#include <algorithm>

#include <Eigen/Eigenvalues>

#include "lo_core/util/parallel.hpp"

namespace lo {

PointCovariance::PointCovariance(const Config & cfg, const Cloud & cloud)
: k_(std::clamp(cfg.knn.k, 3, 64)),
  eps_(cfg.knn.plane_eps),
  threads_(cfg.threads.num_threads),
  tree_(cloud, small_gicp::deferred_build)
{
}

void PointCovariance::compute(Cloud & cloud)
{
  tree_.rebuild(small_gicp::KdTreeBuilder());
  const std::int64_t n = static_cast<std::int64_t>(cloud.size());
  const int k = std::min<int>(k_, static_cast<int>(n));
  if (k < 3) {
    for (std::int64_t i = 0; i < n; ++i) {
      cloud.covs[i].setIdentity();
      cloud.normals[i].setZero();
    }
    return;
  }
  parallelBlocks(n, threads_, [&](std::int64_t begin, std::int64_t end, int) {
      std::size_t idx[64];
      double d2[64];
      for (std::int64_t i = begin; i < end; ++i) {
        const std::size_t found = tree_.knn_search(cloud.points[i], k, idx, d2);
        Vec3 sum = Vec3::Zero();
        Mat3 sum_sq = Mat3::Zero();
        for (std::size_t j = 0; j < found; ++j) {
          const Vec3 q = cloud.points[idx[j]].head<3>();
          sum += q;
          sum_sq += q * q.transpose();
        }
        const double cnt = static_cast<double>(found > 0 ? found : 1);
        const Vec3 mean = sum / cnt;
        const Mat3 C = sum_sq / cnt - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<Mat3> es;
        es.computeDirect(C);
        const Mat3 U = es.eigenvectors();   // ascending eigenvalues: column 0 = normal
        const Vec3 diag(eps_, 1.0, 1.0);
        cloud.covs[i] = U * diag.asDiagonal() * U.transpose();
        Vec3 nrm = U.col(0);
        if (nrm.dot(cloud.points[i].head<3>()) > 0.0) {nrm = -nrm;}   // face the sensor (origin)
        cloud.normals[i] = nrm;
      }
    });
}

}  // namespace lo
