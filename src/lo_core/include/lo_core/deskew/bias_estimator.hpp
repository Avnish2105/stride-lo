// Gyro bias and gravity direction owned by the stack (docs/DESIGN.md contract 2, S9, state
// machine): estimated over standstill windows (DECISIONS I2), never taken from the graph.
#pragma once

#include "lo_core/config/config.hpp"
#include "lo_core/deskew/imu_buffer.hpp"

namespace lo {

class BiasEstimator
{
public:
  explicit BiasEstimator(const Config & cfg) : cfg_(cfg.standstill) {}

  /// Standstill evidence for the scan interval (t0, t1] with the LO pose at t1. `still`: the
  /// per-scan gates passed (joints, gyro, gross LO motion). Returns true when a window of
  /// standstill.min_duration completed with mean LO motion per scan below lo_motion_max /
  /// lo_rot_max (DECISIONS I2); then, if `imu` has samples in it, bias and gravity are updated.
  bool update(bool still, TimeNs t0, TimeNs t1, const ImuBuffer * imu, const SE3 & T_lo);

  void closeWindow() {open_ = false;}
  bool calibrated() const {return calibrated_;}
  bool hasGravity() const {return has_gravity_;}
  const Vec3 & bias() const {return bias_;}
  /// Mean specific force direction in the IMU frame (points "up" at rest).
  const Vec3 & gravityUp() const {return gravity_up_;}
  double windowDuration(TimeNs now) const {return open_ ? toSec(now - start_) : 0.0;}
  std::uint64_t updates() const {return updates_;}
  /// For a warm restart of a new process (not used by default).
  void setBias(const Vec3 & b) {bias_ = b; calibrated_ = true;}

private:
  Config::Standstill cfg_;
  bool open_{false};
  TimeNs start_{0};
  TimeNs last_{0};
  SE3 T_start_;
  int scans_{0};
  Vec6 sum_half_[2]{Vec6::Zero(), Vec6::Zero()};   ///< LO pose (tangent at T_start_) per half
  int n_half_[2]{0, 0};
  Vec3 sum_gyro_{Vec3::Zero()};
  Vec3 sum_acc_{Vec3::Zero()};
  int count_{0};
  bool calibrated_{false};
  bool has_gravity_{false};
  Vec3 bias_{Vec3::Zero()};
  Vec3 gravity_up_{Vec3::UnitZ()};
  std::uint64_t updates_{0};
};

}  // namespace lo
