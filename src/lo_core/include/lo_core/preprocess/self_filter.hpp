// S1: self-filter (static body box + FK leg capsules interpolated to each point's time) and the
// body-blockage monitor (docs/DESIGN.md S1).
#pragma once

#include <memory>
#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/preprocess/kinematics.hpp"
#include "lo_core/types/inputs.hpp"
#include "lo_core/util/time_ring.hpp"

namespace lo {

class SelfFilter
{
public:
  SelfFilter(
    const Config & cfg, const Extrinsics & ext, std::shared_ptr<const KinematicModel> kinematics);

  /// Evaluate leg capsules at fk_samples times across [t_start, t_end] from the joint ring.
  /// Returns false if a kinematic model exists but joint data does not cover the scan
  /// (stale by more than joints_timeout): the static leg box is used instead (JOINTS_MISSING).
  bool prepare(TimeNs t_start, TimeNs t_end, const TimeRing<JointSample> & joints);

  /// Point in the LiDAR frame at absolute time t: is it the robot itself?
  bool isSelf(const Vec3 & p_lidar, TimeNs t) const;

  bool enabled() const {return enabled_;}
  bool jointsMissing() const {return joints_missing_;}
  bool hasKinematics() const {return kin_ != nullptr;}

private:
  bool inBox(const Vec3 & p, const std::vector<double> & box) const
  {
    return p.x() >= box[0] && p.y() >= box[1] && p.z() >= box[2] &&
           p.x() <= box[3] && p.y() <= box[4] && p.z() <= box[5];
  }
  bool interpolateJoints(const TimeRing<JointSample> & ring, TimeNs t, double * q) const;

  bool enabled_;
  SE3 T_base_lidar_;
  std::vector<double> body_box_;
  std::vector<double> leg_box_;
  double inflate_;
  double sphere_radius2_;
  int samples_;
  TimeNs joints_timeout_;
  std::shared_ptr<const KinematicModel> kin_;

  // per scan
  bool joints_missing_{false};
  bool legs_ready_{false};
  TimeNs t0_{0};
  double dt_{1.0};               ///< seconds between samples
  int num_caps_{0};
  std::vector<Vec3> cap_a_;      ///< [sample * num_caps + c], base frame
  std::vector<Vec3> cap_b_;
  std::vector<double> cap_r2_;   ///< (radius + inflate)^2
  std::vector<double> q_scratch_;
};

/// Squared distance from point p to segment [a, b].
inline double segmentDistance2(const Vec3 & p, const Vec3 & a, const Vec3 & b)
{
  const Vec3 ab = b - a;
  const double len2 = ab.squaredNorm();
  double t = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
  t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  return (p - (a + t * ab)).squaredNorm();
}

}  // namespace lo
