#include "lo_core/preprocess/self_filter.hpp"

#include <algorithm>

namespace lo {

SelfFilter::SelfFilter(
  const Config & cfg, const Extrinsics & ext, std::shared_ptr<const KinematicModel> kinematics)
: enabled_(cfg.self_filter.enabled),
  T_base_lidar_(ext.T_base_lidar),
  body_box_(cfg.self_filter.body_box),
  leg_box_(cfg.self_filter.leg_box),
  inflate_(cfg.self_filter.inflate),
  sphere_radius2_(cfg.self_filter.sphere_radius * cfg.self_filter.sphere_radius),
  samples_(std::max(2, cfg.self_filter.fk_samples)),
  joints_timeout_(fromSec(cfg.self_filter.joints_timeout)),
  kin_(std::move(kinematics))
{
  if (kin_) {
    num_caps_ = kin_->numCapsules();
    cap_a_.assign(static_cast<std::size_t>(samples_ * num_caps_), Vec3::Zero());
    cap_b_.assign(cap_a_.size(), Vec3::Zero());
    cap_r2_.resize(static_cast<std::size_t>(num_caps_));
    for (int c = 0; c < num_caps_; ++c) {
      const double r = kin_->capsuleRadius(c) + inflate_;
      cap_r2_[c] = r * r;
    }
    q_scratch_.assign(static_cast<std::size_t>(kin_->numJoints()), 0.0);
  }
}

bool SelfFilter::interpolateJoints(const TimeRing<JointSample> & ring, TimeNs t, double * q) const
{
  if (ring.empty()) {return false;}
  const int n = kin_->numJoints();
  std::size_t ia = 0, ib = 0;
  if (ring.bracket(t, ia, ib)) {
    const JointSample & a = ring[ia];
    const JointSample & b = ring[ib];
    const double alpha = (ib == ia) ? 0.0 :
      static_cast<double>(t - a.t) / static_cast<double>(b.t - a.t);
    for (int j = 0; j < n && j < a.n && j < b.n; ++j) {q[j] = a.q[j] + alpha * (b.q[j] - a.q[j]);}
    return true;
  }
  // Outside the buffered span: hold the nearest sample if it is recent enough.
  const JointSample & s = (t < ring.front().t) ? ring.front() : ring.back();
  const TimeNs age = t > s.t ? t - s.t : s.t - t;
  if (age > joints_timeout_) {return false;}
  for (int j = 0; j < n && j < s.n; ++j) {q[j] = s.q[j];}
  return true;
}

bool SelfFilter::prepare(TimeNs t_start, TimeNs t_end, const TimeRing<JointSample> & joints)
{
  joints_missing_ = false;
  legs_ready_ = false;
  if (!enabled_ || !kin_) {return true;}
  t0_ = t_start;
  const TimeNs span = std::max<TimeNs>(t_end - t_start, 1);
  dt_ = toSec(span) / static_cast<double>(samples_ - 1);
  for (int s = 0; s < samples_; ++s) {
    const TimeNs t = t_start + span * s / (samples_ - 1);
    if (!interpolateJoints(joints, t, q_scratch_.data())) {
      joints_missing_ = true;
      return false;
    }
    kin_->capsules(q_scratch_.data(), &cap_a_[static_cast<std::size_t>(s * num_caps_)],
      &cap_b_[static_cast<std::size_t>(s * num_caps_)]);
  }
  legs_ready_ = true;
  return true;
}

bool SelfFilter::isSelf(const Vec3 & p_lidar, TimeNs t) const
{
  if (!enabled_) {return false;}
  const Vec3 p = T_base_lidar_ * p_lidar;
  if (p.squaredNorm() > sphere_radius2_) {return false;}
  if (inBox(p, body_box_)) {return true;}
  if (!kin_) {return false;}
  if (joints_missing_ || !legs_ready_) {return inBox(p, leg_box_);}

  double u = toSec(t - t0_) / dt_;
  u = std::clamp(u, 0.0, static_cast<double>(samples_ - 1));
  int k = static_cast<int>(u);
  if (k >= samples_ - 1) {k = samples_ - 2;}
  const double alpha = u - static_cast<double>(k);
  const Vec3 * a0 = &cap_a_[static_cast<std::size_t>(k * num_caps_)];
  const Vec3 * b0 = &cap_b_[static_cast<std::size_t>(k * num_caps_)];
  const Vec3 * a1 = a0 + num_caps_;
  const Vec3 * b1 = b0 + num_caps_;
  for (int c = 0; c < num_caps_; ++c) {
    const Vec3 a = a0[c] + alpha * (a1[c] - a0[c]);
    const Vec3 b = b0[c] + alpha * (b1[c] - b0[c]);
    if (segmentDistance2(p, a, b) <= cap_r2_[c]) {return true;}
  }
  return false;
}

}  // namespace lo
