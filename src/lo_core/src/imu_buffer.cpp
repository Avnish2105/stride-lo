#include "lo_core/deskew/imu_buffer.hpp"

#include <algorithm>
#include <limits>

namespace lo {

bool ImuBuffer::push(const ImuSample & s)
{
  ImuEntry e;
  e.t = s.t;
  e.gyro = s.gyro;
  e.acc = s.acc * accel_scale_;
  e.te = s.t;
  if (interval_mean_ && !ring_.empty() && s.t > ring_.back().t) {
    // Sample = mean over (t_prev, t]: its best time is the middle of that interval.
    e.te = s.t - (s.t - ring_.back().t) / 2;
  }
  if (!ring_.empty() && e.te <= ring_.back().te) {
    return false;
  }
  return ring_.push(e);
}

double ImuBuffer::maxGap(TimeNs t0, TimeNs t1) const
{
  if (!covers(t0, t1)) {return std::numeric_limits<double>::infinity();}
  // first sample at or before t0
  std::size_t i = 0;
  std::size_t lo = 0, hi = ring_.size();
  while (lo < hi) {
    const std::size_t mid = (lo + hi) / 2;
    if (ring_[mid].te <= t0) {lo = mid + 1;} else {hi = mid;}
  }
  i = lo > 0 ? lo - 1 : 0;
  TimeNs gap = 0;
  for (; i + 1 < ring_.size(); ++i) {
    gap = std::max(gap, ring_[i + 1].te - ring_[i].te);
    if (ring_[i + 1].te >= t1) {break;}
  }
  return toSec(gap);
}

bool ImuBuffer::relativeRotation(TimeNs ta, TimeNs tb, const Vec3 & bias, SO3 & R_ab) const
{
  R_ab = SO3();
  if (tb == ta) {return true;}
  const bool forward = tb > ta;
  const TimeNs t0 = forward ? ta : tb;
  const TimeNs t1 = forward ? tb : ta;
  if (!covers(t0, t1)) {return false;}
  std::size_t lo = 0, hi = ring_.size();
  while (lo < hi) {
    const std::size_t mid = (lo + hi) / 2;
    if (ring_[mid].te <= t0) {lo = mid + 1;} else {hi = mid;}
  }
  std::size_t k = lo > 0 ? lo - 1 : 0;
  SO3 R;
  for (; k + 1 < ring_.size(); ++k) {
    const ImuEntry & a = ring_[k];
    const ImuEntry & b = ring_[k + 1];
    if (a.te >= t1) {break;}
    const double dt = toSec(b.te - a.te);
    if (dt <= 0.0) {continue;}
    const Vec3 w0 = a.gyro - bias;
    const Vec3 alpha = (b.gyro - a.gyro) / dt;
    const double s0 = toSec(std::max(t0, a.te) - a.te);
    const double s1 = toSec(std::min(t1, b.te) - a.te);
    if (s1 <= s0) {continue;}
    const Vec3 dtheta = w0 * (s1 - s0) + 0.5 * alpha * (s1 * s1 - s0 * s0);
    R = R * SO3::exp(dtheta);
  }
  R_ab = forward ? R : R.inverse();
  return true;
}

int ImuBuffer::mean(TimeNs t0, TimeNs t1, Vec3 & gyro, Vec3 & acc) const
{
  gyro.setZero();
  acc.setZero();
  int n = 0;
  for (std::size_t i = 0; i < ring_.size(); ++i) {
    const ImuEntry & e = ring_[i];
    if (e.te < t0) {continue;}
    if (e.te > t1) {break;}
    gyro += e.gyro;
    acc += e.acc;
    ++n;
  }
  if (n > 0) {
    gyro /= static_cast<double>(n);
    acc /= static_cast<double>(n);
  }
  return n;
}

double ImuBuffer::rate() const
{
  const std::size_t n = ring_.size();
  if (n < 10) {return 0.0;}
  const std::size_t m = std::min<std::size_t>(n - 1, 200);
  const double span = toSec(ring_[n - 1].t - ring_[n - 1 - m].t);
  return span > 0.0 ? static_cast<double>(m) / span : 0.0;
}

}  // namespace lo
