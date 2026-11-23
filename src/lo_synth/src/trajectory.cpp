#include "lo_synth/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace lo::synth {

Trajectory::Trajectory(const Vec3 & start, double start_yaw, const GaitParams & gait)
: gait_(gait), cur_p_(start), cur_yaw_(start_yaw)
{
}

Trajectory & Trajectory::still(double duration)
{
  Segment s;
  s.kind = Segment::STILL;
  s.t0 = this->duration();
  s.T = duration;
  s.p0 = s.p1 = cur_p_;
  s.yaw0 = s.yaw1 = cur_yaw_;
  segs_.push_back(s);
  return *this;
}

Trajectory & Trajectory::moveTo(const Vec3 & p, double speed)
{
  const double L = (p - cur_p_).norm();
  if (L < 1e-9) {return *this;}
  Segment s;
  s.kind = Segment::MOVE;
  s.t0 = duration();
  s.T = std::max(0.5, 1.875 * L / speed);   // peak speed = speed
  s.p0 = cur_p_;
  s.p1 = p;
  s.yaw0 = s.yaw1 = cur_yaw_;
  segs_.push_back(s);
  cur_p_ = p;
  return *this;
}

Trajectory & Trajectory::turnBy(double angle, double yaw_rate)
{
  if (std::abs(angle) < 1e-9) {return *this;}
  Segment s;
  s.kind = Segment::TURN;
  s.t0 = duration();
  s.T = std::max(0.5, 1.875 * std::abs(angle) / yaw_rate);
  s.p0 = s.p1 = cur_p_;
  s.yaw0 = cur_yaw_;
  s.yaw1 = cur_yaw_ + angle;
  segs_.push_back(s);
  cur_yaw_ += angle;
  return *this;
}

Trajectory & Trajectory::turnTo(double yaw, double yaw_rate, bool long_way)
{
  double d = std::remainder(yaw - cur_yaw_, 2.0 * M_PI);
  if (long_way) {d = d > 0 ? d - 2.0 * M_PI : d + 2.0 * M_PI;}
  return turnBy(d, yaw_rate);
}

const Trajectory::Segment & Trajectory::segmentAt(double t) const
{
  if (t <= 0.0 || segs_.empty()) {return segs_.front();}
  for (const auto & s : segs_) {
    if (t < s.t0 + s.T) {return s;}
  }
  return segs_.back();
}

double Trajectory::gaitActivity(double t) const
{
  if (segs_.empty()) {return 0.0;}
  const Segment & s = segmentAt(t);
  if (s.kind == Segment::STILL) {return 0.0;}
  const double tau = std::clamp((t - s.t0) / s.T, 0.0, 1.0);
  return mjd(tau) / 1.875;   // 0 at the ends, 1 at peak speed
}

SE3 Trajectory::pose(double t) const
{
  if (segs_.empty()) {return SE3(SO3::rotZ(cur_yaw_), cur_p_);}
  const Segment & s = segmentAt(t);
  const double tau = std::clamp((t - s.t0) / s.T, 0.0, 1.0);
  const double m = mj(tau);
  const Vec3 p = s.p0 + m * (s.p1 - s.p0);
  const double yaw = s.yaw0 + m * (s.yaw1 - s.yaw0);
  const double k = (s.kind == Segment::STILL) ? 0.0 : mjd(tau) / 1.875;
  const double w = 2.0 * M_PI * gait_.freq * t;
  const Vec3 bounce(0.0, 0.0, k * gait_.bounce * std::sin(2.0 * w));
  const double roll = k * gait_.roll * std::sin(w + 0.3);
  const double pitch = k * gait_.pitch * std::sin(2.0 * w + 1.1);
  const SO3 R = SO3::rotZ(yaw) * SO3::rotY(pitch) * SO3::rotX(roll);
  return SE3(R, p + bounce);
}

Vec3 Trajectory::omegaBody(double t) const
{
  constexpr double h = 1e-5;
  const SO3 Ra = pose(t - h).so3();
  const SO3 Rb = pose(t + h).so3();
  return (Ra.inverse() * Rb).log() / (2.0 * h);
}

Vec3 Trajectory::pointAccelWorld(double t, const Vec3 & r) const
{
  constexpr double h = 1e-4;
  const Vec3 a = pose(t - h) * r;
  const Vec3 b = pose(t) * r;
  const Vec3 c = pose(t + h) * r;
  return (a - 2.0 * b + c) / (h * h);
}

}  // namespace lo::synth
