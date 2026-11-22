// SE(3)/SO(3) conventions of lo_core. READ THIS before touching any pose or covariance.
//
//  * Tangent vectors are ROTATION FIRST: xi = [omega (rad); rho (m)]  (GTSAM Pose3 order).
//  * Perturbations are on the RIGHT (body frame): T_perturbed = T * Exp(xi).
//  * Covariances are 6x6 in that order and frame. A frame change T_ab maps body-frame tangents
//    of frame b to frame a: xi_a = Ad(T_ab) xi_b, so cov_a = Ad cov_b Ad^T, with
//    Ad(T) = [R 0; [t]x R  R]  (== gtsam::Pose3::AdjointMap).
//  * Sophus uses [rho; omega] (translation first). Sophus is only ever touched through the
//    helpers below; never call SE3::exp/log/Adj directly elsewhere.
#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace lo {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using SO3 = Sophus::SO3d;
using SE3 = Sophus::SE3d;

inline Mat3 skew(const Vec3 & v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

/// Exp of a rotation-first tangent [omega; rho].
inline SE3 expRT(const Vec6 & xi)
{
  Sophus::Vector6d s;
  s << xi.tail<3>(), xi.head<3>();
  return SE3::exp(s);
}

/// Log returning a rotation-first tangent [omega; rho].
inline Vec6 logRT(const SE3 & T)
{
  const Sophus::Vector6d s = T.log();
  Vec6 xi;
  xi << s.tail<3>(), s.head<3>();
  return xi;
}

/// Adjoint in rotation-first order (== gtsam::Pose3::AdjointMap).
inline Mat6 adjointRT(const SE3 & T)
{
  const Mat3 R = T.rotationMatrix();
  Mat6 A = Mat6::Zero();
  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(3, 0) = skew(T.translation()) * R;
  A.block<3, 3>(3, 3) = R;
  return A;
}

/// cov expressed in frame a, given cov in frame b and T_ab.
inline Mat6 transportCovariance(const SE3 & T_ab, const Mat6 & cov_b)
{
  const Mat6 A = adjointRT(T_ab);
  return A * cov_b * A.transpose();
}

/// Rotation angle (rad) of a rotation.
inline double angleOf(const SO3 & R) {return R.log().norm();}
inline double angleOf(const SE3 & T) {return T.so3().log().norm();}

inline Eigen::Isometry3d toIsometry(const SE3 & T)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = T.rotationMatrix();
  iso.translation() = T.translation();
  return iso;
}

inline SE3 fromIsometry(const Eigen::Isometry3d & iso)
{
  Eigen::Quaterniond q(iso.linear());
  q.normalize();
  return SE3(q, iso.translation());
}

/// SE3 from translation + quaternion given as (x, y, z, w) (ROS message order).
inline SE3 fromXyzQuat(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);   // Eigen constructor takes (w, x, y, z)
  q.normalize();
  return SE3(q, Vec3(x, y, z));
}

/// Interpolate on SE(3): a * Exp(alpha * Log(a^-1 b)), alpha in [0, 1].
inline SE3 interpolate(const SE3 & a, const SE3 & b, double alpha)
{
  return a * expRT(alpha * logRT(a.inverse() * b));
}

inline bool allFinite(const SE3 & T)
{
  return T.matrix().allFinite();
}

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

}  // namespace lo
