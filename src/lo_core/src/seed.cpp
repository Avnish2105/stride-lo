#include "lo_core/registration/seed.hpp"

#include <algorithm>
#include <cmath>

namespace lo {

bool GraphOdomBuffer::poseAt(TimeNs t, SE3 & T) const
{
  if (ring_.empty()) {return false;}
  std::size_t ia = 0, ib = 0;
  if (ring_.bracket(t, ia, ib)) {
    const auto & a = ring_[ia];
    const auto & b = ring_[ib];
    const double alpha = ia == ib ? 0.0 : static_cast<double>(t - a.t) / static_cast<double>(b.t - a.t);
    T = interpolate(a.T_odom_base, b.T_odom_base, alpha);
    return true;
  }
  const auto & s = ring_.back();
  if (t > s.t && t - s.t <= timeout_) {
    // extrapolate with the last twist (base axes)
    const double dt = toSec(t - s.t);
    Vec6 xi;
    xi << s.w_base * dt, s.v_base * dt;
    T = s.T_odom_base * expRT(xi);
    return true;
  }
  return false;
}

bool GraphOdomBuffer::twistAt(TimeNs t, Vec3 & v, Vec3 & w) const
{
  if (ring_.empty()) {return false;}
  std::size_t ia = 0, ib = 0;
  if (ring_.bracket(t, ia, ib)) {
    const auto & a = ring_[ia];
    const auto & b = ring_[ib];
    const double alpha = ia == ib ? 0.0 : static_cast<double>(t - a.t) / static_cast<double>(b.t - a.t);
    v = a.v_base + alpha * (b.v_base - a.v_base);
    w = a.w_base + alpha * (b.w_base - a.w_base);
    return true;
  }
  const auto & s = ring_.back();
  if (t >= s.t && t - s.t <= timeout_) {
    v = s.v_base;
    w = s.w_base;
    return true;
  }
  return false;
}

SeedModel::SeedModel(const Config & cfg, const Extrinsics & ext)
: T_base_lidar_(ext.T_base_lidar),
  T_lidar_base_(ext.T_base_lidar.inverse()),
  R_li_(ext.T_lidar_imu.so3()),
  lever_(ext.T_base_lidar.so3().inverse() * ext.T_base_lidar.translation()),
  v_max_(cfg.integrity.v_max),
  w_max_(cfg.integrity.w_max),
  max_dev_(cfg.seed.graph_max_dev),
  holdoff_(fromSec(cfg.seed.graph_holdoff))
{
}

bool SeedModel::graphMotion(TimeNs t_prev, TimeNs t_ref, const GraphOdomBuffer & graph,
  const LoVelocity & lo_vel, Vec3 & dt_lidar, Vec3 & v_base_lidar_axes, bool & gated)
{
  gated = false;
  SE3 Ta, Tb;
  Vec3 vb, wb;
  if (!graph.poseAt(t_prev, Ta) || !graph.poseAt(t_ref, Tb) || !graph.twistAt(t_ref, vb, wb)) {
    return false;
  }
  const SE3 d_lidar = T_lidar_base_ * (Ta.inverse() * Tb) * T_base_lidar_;
  const Vec3 t_bl = T_base_lidar_.translation();
  const Vec3 v = T_lidar_base_.so3() * (vb + wb.cross(t_bl));
  const double dt = std::max(toSec(t_ref - t_prev), 1e-3);
  const double motion_speed = d_lidar.translation().norm() / dt;
  bool ok = d_lidar.translation().allFinite() && v.allFinite() && wb.allFinite() &&
    vb.norm() <= v_max_ && wb.norm() <= w_max_ && motion_speed <= v_max_ + max_dev_;
  // like for like over LO's last long window: graph vs LO displacement, as a mean velocity
  SE3 Pa, Pb;
  if (ok && lo_vel.gate_valid && lo_vel.g1 > lo_vel.g0 && graph.poseAt(lo_vel.g0, Pa) &&
    graph.poseAt(lo_vel.g1, Pb))
  {
    const SE3 d_win = T_lidar_base_ * (Pa.inverse() * Pb) * T_base_lidar_;
    const Vec3 dv = (d_win.translation() - lo_vel.g_disp) / toSec(lo_vel.g1 - lo_vel.g0);
    ok = dv.allFinite() && dv.norm() <= max_dev_;
  }
  if (!ok) {gated_until_ = t_ref + holdoff_;}
  if (t_ref < gated_until_) {
    gated = true;
    return false;
  }
  dt_lidar = d_lidar.translation();
  v_base_lidar_axes = T_lidar_base_.so3() * vb;
  return true;
}

SeedResult SeedModel::predict(
  TimeNs t_prev, TimeNs t_ref, const SE3 & T_odom_lidar_prev, const ImuBuffer & imu,
  bool imu_usable, const Vec3 & gyro_bias, const GraphOdomBuffer & graph,
  const LoVelocity & lo_vel)
{
  SeedResult r;
  const double dt = toSec(t_ref - t_prev);

  // ---- rotation: gyro -> LO angular velocity -> identity
  SO3 dR;
  SO3 dR_imu;
  if (imu_usable && imu.relativeRotation(t_prev, t_ref, gyro_bias, dR_imu)) {
    dR = R_li_ * dR_imu * R_li_.inverse();
    r.rotation_from_gyro = true;
  } else if (lo_vel.valid) {
    dR = SO3::exp(lo_vel.w_lidar * dt);
    r.flags |= flags::SEED_FALLBACK;
  } else {
    r.flags |= flags::SEED_FALLBACK | flags::SEED_IDENTITY;
  }

  // ---- translation: graph odometry (gated) -> LO velocity -> identity
  Vec3 dt_lidar = Vec3::Zero();
  Vec3 v_graph = Vec3::Zero();
  if (graphMotion(t_prev, t_ref, graph, lo_vel, dt_lidar, v_graph, r.graph_gated)) {
    r.translation_from_graph = true;
  } else if (lo_vel.valid) {
    dt_lidar = lo_vel.v_lidar * dt;
    r.flags |= flags::SEED_FALLBACK | flags::SEED_NO_GRAPH;
  } else {
    r.flags |= flags::SEED_FALLBACK | flags::SEED_NO_GRAPH | flags::SEED_IDENTITY;
  }
  r.delta = SE3(dR, dt_lidar);
  r.T_seed = T_odom_lidar_prev * r.delta;

  // ---- deskew velocity v0 (base origin, LiDAR axes): graph twist -> LO velocity minus the
  //      lever-arm term (the LiDAR origin velocity is v0 + w x c)
  if (r.translation_from_graph) {
    r.v_deskew = v_graph;
  } else if (lo_vel.valid) {
    r.v_deskew = lo_vel.v_lidar - lo_vel.w_lidar.cross(lever_);
  }
  return r;
}

bool jointBlockSpeed(const TimeRing<JointSample> & ring, TimeNs t, TimeNs block, double & speed)
{
  speed = 0.0;
  if (ring.size() < 2) {return false;}
  const std::size_t i0 = ring.lowerBound(t - block);
  std::size_t i1 = ring.lowerBound(t + 1);
  if (i1 == 0) {return false;}
  --i1;   // last sample <= t
  if (i0 >= ring.size() || i1 <= i0) {return false;}
  const JointSample & a = ring[i0];
  const JointSample & b = ring[i1];
  const int nj = std::min(a.n, b.n);
  if (a.has_velocity) {
    // signed mean per joint over the block
    double worst = 0.0;
    for (int j = 0; j < nj; ++j) {
      double sum = 0.0;
      for (std::size_t i = i0; i <= i1; ++i) {sum += ring[i].qd[j];}
      worst = std::max(worst, std::abs(sum / static_cast<double>(i1 - i0 + 1)));
    }
    speed = worst;
  } else {
    const double dt = toSec(b.t - a.t);
    if (dt <= 0.0) {return false;}
    double worst = 0.0;
    for (int j = 0; j < nj; ++j) {worst = std::max(worst, std::abs((b.q[j] - a.q[j]) / dt));}
    speed = worst;
  }
  return true;
}

}  // namespace lo
