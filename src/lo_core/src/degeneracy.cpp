#include "lo_core/integrity/degeneracy.hpp"

#include <Eigen/Eigenvalues>

namespace lo {

namespace {
Mat6 scaling(double L)
{
  Mat6 M = Mat6::Identity();
  const double s = L > 1e-6 ? 1.0 / L : 1.0;
  M(0, 0) = M(1, 1) = M(2, 2) = s;
  return M;
}
}  // namespace

DegeneracyInfo analyzeDegeneracy(const Mat6 & H, double L, double N, double lambda_min, int max_dirs)
{
  DegeneracyInfo d;
  d.L = L > 1e-6 ? L : 1.0;
  d.N = N > 1.0 ? N : 1.0;
  if (!H.allFinite()) {return d;}
  const Mat6 M = scaling(d.L);
  const Mat6 Hs = M * H * M;
  Eigen::SelfAdjointEigenSolver<Mat6> es(0.5 * (Hs + Hs.transpose()));
  if (es.info() != Eigen::Success) {return d;}
  d.V = es.eigenvectors();
  d.eigenvalues = es.eigenvalues() / d.N;
  d.valid = true;
  for (int k = 0; k < 6 && d.num_degenerate < max_dirs; ++k) {   // ascending order
    if (d.eigenvalues[k] < lambda_min) {
      d.degenerate[k] = true;
      ++d.num_degenerate;
    }
  }
  return d;
}

namespace {
// Dynamic size with a fixed maximum of 6: stack storage, no heap allocation (hot path).
using MatX6 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 6, 6>;
using VecX6 = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 6, 1>;
using Mat6X = Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor, 6, 6>;

/// Orthonormal basis (scaled space) of the observable subspace: the non-degenerate eigenvectors.
int observableBasis(const DegeneracyInfo & d, Eigen::Matrix<double, 6, 6> & O)
{
  int m = 0;
  for (int k = 0; k < 6; ++k) {
    if (!d.degenerate[k]) {O.col(m++) = d.V.col(k);}
  }
  return m;
}
}  // namespace

Vec6 projectedStep(const Mat6 & H, const Vec6 & b, const DegeneracyInfo & d, double damping)
{
  const Mat6 M = scaling(d.L);
  const Mat6 Hs = M * H * M;
  const Vec6 g = M * b;
  Eigen::Matrix<double, 6, 6> O;
  const int m = observableBasis(d, O);
  if (m == 0) {return Vec6::Zero();}
  const Mat6X Om = O.leftCols(m);
  MatX6 A = Om.transpose() * Hs * Om;
  A.diagonal().array() += damping;
  const Eigen::LDLT<MatX6> ldlt(A);
  const VecX6 rhs = -(Om.transpose() * g);
  const VecX6 y = ldlt.solve(rhs);
  const Vec6 ds = Om * y;
  return M * ds;
}

Vec6 removeDegenerate(const Vec6 & xi, const DegeneracyInfo & d)
{
  if (d.num_degenerate == 0) {return xi;}
  const Mat6 M = scaling(d.L);
  Vec6 xs = M.inverse() * xi;
  for (int k = 0; k < 6; ++k) {
    if (d.degenerate[k]) {xs -= d.V.col(k).dot(xs) * d.V.col(k);}
  }
  return M * xs;
}

int degenerateDirections(const DegeneracyInfo & d, std::array<Vec6, 3> & out)
{
  const Mat6 M = scaling(d.L);
  int n = 0;
  for (int k = 0; k < 6 && n < 3; ++k) {
    if (!d.degenerate[k]) {continue;}
    Vec6 v = M * d.V.col(k);
    const double norm = v.norm();
    out[n++] = norm > 0 ? Vec6(v / norm) : v;
  }
  for (int k = n; k < 3; ++k) {out[k].setZero();}
  return n;
}

Mat6 registrationCovariance(const Mat6 & H, const DegeneracyInfo & d, double sigma2,
  const Config & cfg)
{
  const Mat6 M = scaling(d.L);
  const Mat6 Hs = M * H * M;
  const double s_rot = cfg.degeneracy.sigma_rot * d.L;   // scaled units (metres at range L)
  const double s_trans = cfg.degeneracy.sigma_trans;
  Mat6 Ss = Mat6::Zero();
  Eigen::Matrix<double, 6, 6> O;
  const int m = observableBasis(d, O);
  if (m > 0) {
    const Mat6X Om = O.leftCols(m);
    const MatX6 A = Om.transpose() * Hs * Om;
    const Eigen::LDLT<MatX6> ldlt(A);
    const MatX6 I = MatX6::Identity(m, m);
    const MatX6 Ainv = ldlt.solve(I);
    Ss += cfg.cov.kappa * sigma2 * (Om * Ainv * Om.transpose());
  }
  for (int k = 0; k < 6; ++k) {
    if (!d.degenerate[k]) {continue;}
    const Vec6 v = d.V.col(k);
    const double var = v.head<3>().squaredNorm() * s_rot * s_rot +
      v.tail<3>().squaredNorm() * s_trans * s_trans;
    Ss += var * v * v.transpose();
  }
  Mat6 S = M * Ss * M;
  const double fr = cfg.cov.floor_rot_deg * kDegToRad;
  const double ft = cfg.cov.floor_trans;
  for (int i = 0; i < 3; ++i) {
    S(i, i) += fr * fr;
    S(i + 3, i + 3) += ft * ft;
  }
  return 0.5 * (S + S.transpose());
}

}  // namespace lo
