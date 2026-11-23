#include "lo_core/deskew/deskew.hpp"

#include <algorithm>

#include "lo_core/util/parallel.hpp"

namespace lo {

Deskewer::Deskewer(const Config & cfg, const Extrinsics & ext)
: enabled_(cfg.deskew.enabled),
  R_li_(ext.T_lidar_imu.so3()),
  segs_(1024),
  c_(ext.T_base_lidar.so3().inverse() * ext.T_base_lidar.translation())
{
}

DeskewMode Deskewer::prepare(
  const ImuBuffer & imu, bool use_imu, TimeNs t_start, TimeNs t_end, TimeNs t_ref,
  const Vec3 & gyro_bias, const Vec3 & v_base_lidar_axes, const Vec3 & w_lidar_fallback)
{
  v_ = v_base_lidar_axes;
  w_fallback_ = w_lidar_fallback;
  num_segs_ = 0;
  R_ref_inv_ = SO3();
  if (!enabled_) {
    mode_ = DeskewMode::DISABLED;
    return mode_;
  }
  const TimeNs lo_t = std::min(t_start, t_ref);
  const TimeNs hi_t = std::max(t_end, t_ref);
  if (!use_imu || !imu.covers(lo_t, hi_t)) {
    mode_ = DeskewMode::FALLBACK;
    return mode_;
  }

  // Segments between consecutive effective IMU samples spanning [lo_t, hi_t].
  const auto & ring = imu.ring();
  std::size_t lo = 0, hi = ring.size();
  while (lo < hi) {
    const std::size_t mid = (lo + hi) / 2;
    if (ring[mid].te <= lo_t) {lo = mid + 1;} else {hi = mid;}
  }
  std::size_t k = lo > 0 ? lo - 1 : 0;
  SO3 R;   // rotation at the current segment start, relative to the first one
  for (; k + 1 < ring.size() && num_segs_ < segs_.size(); ++k) {
    const ImuEntry & a = ring[k];
    const ImuEntry & b = ring[k + 1];
    const double dt = toSec(b.te - a.te);
    if (dt <= 0.0) {continue;}
    Segment & s = segs_[num_segs_++];
    s.t0 = toSec(a.te - t_ref);
    s.dt = dt;
    s.R0 = R;
    s.w0 = R_li_ * (a.gyro - gyro_bias);
    s.alpha = R_li_ * ((b.gyro - a.gyro) / dt);
    R = R * SO3::exp(s.w0 * dt + 0.5 * s.alpha * dt * dt);
    if (b.te >= hi_t) {break;}
  }
  if (num_segs_ == 0) {
    mode_ = DeskewMode::FALLBACK;
    return mode_;
  }
  mode_ = DeskewMode::IMU;
  R_ref_inv_ = SO3();
  R_ref_inv_ = rotationAt(0.0).inverse();
  return mode_;
}

SO3 Deskewer::rotationAt(double dt) const
{
  if (mode_ == DeskewMode::FALLBACK) {return SO3::exp(w_fallback_ * dt);}
  if (mode_ != DeskewMode::IMU || num_segs_ == 0) {return SO3();}
  // segment containing dt (clamped to the table)
  std::size_t lo = 0, hi = num_segs_;
  while (lo < hi) {
    const std::size_t mid = (lo + hi) / 2;
    if (segs_[mid].t0 <= dt) {lo = mid + 1;} else {hi = mid;}
  }
  const Segment & s = segs_[lo > 0 ? lo - 1 : 0];
  const double tau = std::clamp(dt - s.t0, 0.0, s.dt);
  const SO3 R = s.R0 * SO3::exp(s.w0 * tau + 0.5 * s.alpha * tau * tau);
  return R_ref_inv_ * R;
}

void Deskewer::apply(Cloud & cloud, int threads) const
{
  if (mode_ == DeskewMode::DISABLED) {return;}
  const std::int64_t n = static_cast<std::int64_t>(cloud.size());
  parallelBlocks(n, threads, [&](std::int64_t begin, std::int64_t end, int) {
      for (std::int64_t i = begin; i < end; ++i) {
        const double dt = cloud.times[i];
        const Vec3 p = cloud.points[i].head<3>();
        const Vec3 q = rotationAt(dt) * (p + c_) - c_ + v_ * dt;
        cloud.points[i].head<3>() = q;
      }
    });
}

}  // namespace lo
