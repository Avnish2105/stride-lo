#include "lo_core/deskew/bias_estimator.hpp"

#include <algorithm>

namespace lo {

bool BiasEstimator::update(bool still, TimeNs t0, TimeNs t1, const ImuBuffer * imu, const SE3 & T_lo)
{
  if (!still) {
    open_ = false;
    return false;
  }
  if (!open_) {
    open_ = true;
    start_ = t0;
    last_ = t0;
    T_start_ = T_lo;
    scans_ = 0;
    sum_half_[0].setZero();
    sum_half_[1].setZero();
    n_half_[0] = n_half_[1] = 0;
    sum_gyro_.setZero();
    sum_acc_.setZero();
    count_ = 0;
  }
  ++scans_;
  {
    const int h = toSec(t1 - start_) < 0.5 * cfg_.min_duration ? 0 : 1;
    sum_half_[h] += logRT(T_start_.inverse() * T_lo);
    ++n_half_[h];
  }
  if (imu) {
    // accumulate samples with effective time in (last_, t1]
    const auto & ring = imu->ring();
    for (std::size_t i = ring.lowerBound(last_); i < ring.size(); ++i) {
      const ImuEntry & e = ring[i];
      if (e.te <= last_) {continue;}
      if (e.te > t1) {break;}
      sum_gyro_ += e.gyro;
      sum_acc_ += e.acc;
      ++count_;
    }
  }
  last_ = std::max(last_, t1);
  if (toSec(t1 - start_) < cfg_.min_duration) {return false;}

  // Mean LO motion per scan over the window: compare the mean pose of the two window halves
  // (registration jitter averages out; a creeping robot does not).
  bool lo_ok = false;
  if (n_half_[0] > 0 && n_half_[1] > 0) {
    const Vec6 d = sum_half_[1] / n_half_[1] - sum_half_[0] / n_half_[0];
    const double scans_between = 0.5 * static_cast<double>(scans_);
    lo_ok = d.tail<3>().norm() / scans_between < cfg_.lo_motion_max &&
      d.head<3>().norm() / scans_between < cfg_.lo_rot_max_deg * kDegToRad;
  }
  bool completed = false;
  if (lo_ok) {
    completed = true;
    if (count_ >= 10) {
      const Vec3 mean_gyro = sum_gyro_ / static_cast<double>(count_);
      const Vec3 mean_acc = sum_acc_ / static_cast<double>(count_);
      if (!calibrated_) {
        bias_ = mean_gyro;
        calibrated_ = true;
      } else {
        bias_ = (1.0 - cfg_.bias_alpha) * bias_ + cfg_.bias_alpha * mean_gyro;
      }
      if (mean_acc.norm() > 1.0) {   // m/s^2: a real accelerometer reading
        gravity_up_ = mean_acc.normalized();
        has_gravity_ = true;
      }
      ++updates_;
    }
  }
  // next window starts now
  start_ = t1;
  T_start_ = T_lo;
  scans_ = 0;
  sum_half_[0].setZero();
  sum_half_[1].setZero();
  n_half_[0] = n_half_[1] = 0;
  sum_gyro_.setZero();
  sum_acc_.setZero();
  count_ = 0;
  return completed;
}

}  // namespace lo
