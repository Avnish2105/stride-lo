// Hessian degeneracy (docs/DESIGN.md "Degeneracy handling"; solution remapping, Zhang et al.
// 2016). All tangents rotation-first, LiDAR frame, right perturbation.
//
// Scaling: a rotation dtheta moves a point at range L by L*dtheta, so with
//   M = diag(1/L, 1/L, 1/L, 1, 1, 1),  delta = M delta_s,  H_s = M H M
// rotation and translation become comparable (metres).
//
// Observability is judged on the POINT-TO-PLANE Hessian Hp = sum w (n^T J)^T (n^T J) (what the
// surfaces actually constrain), not on the GICP Hessian: GICP's regularised covariances also
// give a small in-plane "information" from offsets to voxel means, which says nothing about
// sliding along a surface and, multiplied by long lever arms, made e.g. yaw over a bare ground
// plane look observable (DECISIONS I5). Hp_s = V Lambda V^T, direction k is degenerate when
// lambda_k / N < lambda_min (N = inlier count; each point contributes at most 1).
// The step and the covariance use the GICP Hessian restricted to the observable subspace.
#pragma once

#include <array>

#include "lo_core/config/config.hpp"
#include "lo_core/types/se3.hpp"

namespace lo {

struct DegeneracyInfo
{
  bool valid{false};
  int num_degenerate{0};
  double L{1.0};
  double N{1.0};
  Vec6 eigenvalues{Vec6::Zero()};   ///< of H_s, divided by N, ascending
  Mat6 V{Mat6::Identity()};         ///< eigenvectors of H_s (columns, ascending)
  std::array<bool, 6> degenerate{};
};

/// Hp: point-to-plane Hessian (observability), L: rotation scale, N: inlier count.
DegeneracyInfo analyzeDegeneracy(const Mat6 & Hp, double L, double N, double lambda_min, int max_dirs);

/// Gauss-Newton step minimising b^T d + 1/2 d^T H d restricted to the observable subspace of d
/// (H: GICP Hessian). Without degenerate directions this is -(H + damping)^-1 b.
Vec6 projectedStep(const Mat6 & H, const Vec6 & b, const DegeneracyInfo & d, double damping);

/// Remove degenerate components from an unscaled tangent vector.
Vec6 removeDegenerate(const Vec6 & xi, const DegeneracyInfo & d);

/// Unit degenerate directions in the unscaled tangent space. Returns how many were written.
int degenerateDirections(const DegeneracyInfo & d, std::array<Vec6, 3> & out);

/// Registration covariance in the LiDAR frame (docs/DESIGN.md "Covariance model"):
///   kappa * sigma2 * H^-1 on observable directions, sigma 1 m / 0.5 rad on degenerate ones,
///   plus the floor diag(0.05 deg x3, 5 mm x3)^2.
Mat6 registrationCovariance(const Mat6 & H, const DegeneracyInfo & d, double sigma2,
  const Config & cfg);

}  // namespace lo
