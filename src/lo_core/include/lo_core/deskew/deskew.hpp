// S3: continuous-time deskew (docs/DESIGN.md S3). Every point is moved to the sensor frame at
// the scan reference time t_ref:  p_ref = dR(t) p + (dR(t) - I) c + v0 (t - t_ref).
//  * dR(t): bias-corrected gyro, constant angular acceleration between IMU samples, mapped into
//    the LiDAR frame by the extrinsic rotation.
//  * The body rotates about the BASE origin: c = R_bl^T t_bl (base->LiDAR lever arm in LiDAR
//    axes) moves the LiDAR by (dR - I) c exactly, instead of the first-order omega(t_ref) x r dt,
//    which is wrong by cm when the trot pitch rate reverses inside a scan (DECISIONS I28).
//  * v0: base-origin velocity in LiDAR axes (graph twist, else LO's velocity minus the lever term).
//  * Fallback (IMU gap > imu.gap_max, IMU lost/missing): constant LO twist for rotation too.
#pragma once

#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/deskew/imu_buffer.hpp"
#include "lo_core/types/points.hpp"

namespace lo {

enum class DeskewMode { IMU, FALLBACK, DISABLED };

class Deskewer
{
public:
  Deskewer(const Config & cfg, const Extrinsics & ext);

  /// Build the motion model over [t_start, t_end] around t_ref. `use_imu`: caller decided the
  /// IMU covers the span without a gap. w_lidar_fallback: LO angular velocity (LiDAR frame).
  DeskewMode prepare(
    const ImuBuffer & imu, bool use_imu, TimeNs t_start, TimeNs t_end, TimeNs t_ref,
    const Vec3 & gyro_bias, const Vec3 & v_base_lidar_axes, const Vec3 & w_lidar_fallback);

  /// Deskew `cloud` in place (its times are relative to t_ref). Parallel, deterministic.
  void apply(Cloud & cloud, int threads) const;

  /// Rotation of the LiDAR frame at t_ref + dt relative to t_ref.
  SO3 rotationAt(double dt) const;
  /// Base->LiDAR lever arm in LiDAR axes (c above).
  const Vec3 & lever() const {return c_;}

  DeskewMode mode() const {return mode_;}

private:
  struct Segment
  {
    double t0{0};        ///< seconds relative to t_ref
    double dt{0};
    SO3 R0;              ///< rotation at t0 relative to the first segment start (LiDAR frame)
    Vec3 w0{Vec3::Zero()};
    Vec3 alpha{Vec3::Zero()};
  };

  bool enabled_;
  SO3 R_li_;             ///< LiDAR <- IMU rotation
  DeskewMode mode_{DeskewMode::DISABLED};
  std::vector<Segment> segs_;
  std::size_t num_segs_{0};
  SO3 R_ref_inv_;        ///< inverse of the rotation at t_ref (same origin as segments)
  Vec3 v_{Vec3::Zero()};        ///< v0: base-origin velocity, LiDAR axes
  Vec3 c_{Vec3::Zero()};        ///< lever arm R_bl^T t_bl
  Vec3 w_fallback_{Vec3::Zero()};
};

}  // namespace lo
