// IMU ring (2 s) with gyro integration under a constant-angular-acceleration model between
// samples (DLIO-style). Single-threaded: the ROS layer moves samples from its lock-free queue
// into this buffer on the worker thread.
#pragma once

#include "lo_core/config/config.hpp"
#include "lo_core/types/inputs.hpp"
#include "lo_core/util/time_ring.hpp"

namespace lo {

struct ImuEntry
{
  TimeNs t{0};          ///< message stamp
  TimeNs te{0};         ///< effective sample time (mid-interval for interval_mean samples)
  Vec3 gyro{Vec3::Zero()};
  Vec3 acc{Vec3::Zero()};
};

class ImuBuffer
{
public:
  explicit ImuBuffer(const Config & cfg)
  : ring_(static_cast<std::size_t>(cfg.limits.imu_buffer)),
    interval_mean_(cfg.imu.sample_convention == "interval_mean"),
    accel_scale_(cfg.imu.accel_scale) {}

  /// Accel is multiplied by imu.accel_scale here. False if out of order (dropped).
  bool push(const ImuSample & s);
  void clear() {ring_.clear();}

  bool empty() const {return ring_.empty();}
  std::size_t size() const {return ring_.size();}
  TimeNs latestStamp() const {return ring_.empty() ? 0 : ring_.back().t;}
  const TimeRing<ImuEntry> & ring() const {return ring_;}

  /// Effective samples bracket [t0, t1].
  bool covers(TimeNs t0, TimeNs t1) const
  {
    return !ring_.empty() && ring_.front().te <= t0 && ring_.back().te >= t1;
  }

  /// Largest spacing [s] between consecutive effective samples overlapping [t0, t1]; +inf if
  /// not covered.
  double maxGap(TimeNs t0, TimeNs t1) const;

  /// Relative rotation of the IMU frame from ta to tb (R(ta)^T R(tb)), bias-corrected.
  bool relativeRotation(TimeNs ta, TimeNs tb, const Vec3 & bias, SO3 & R_ab) const;

  /// Mean raw gyro and accel of samples with effective time in [t0, t1]; returns the count.
  int mean(TimeNs t0, TimeNs t1, Vec3 & gyro, Vec3 & acc) const;

  /// Sample rate over the newest samples [Hz] (0 if unknown).
  double rate() const;

private:
  TimeRing<ImuEntry> ring_;
  bool interval_mean_;
  double accel_scale_;
};

}  // namespace lo
