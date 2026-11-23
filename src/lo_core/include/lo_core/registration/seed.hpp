// S5: seed (docs/DESIGN.md S5: no information, start point only). Rotation from gyro
// integration, translation from graph odometry; fall back to LO's last velocity, then identity.
// The source is recorded in flags. Also provides the LiDAR velocity used by S3 for the
// deskew translation.
#pragma once

#include <cstdint>

#include "lo_core/config/config.hpp"
#include "lo_core/deskew/imu_buffer.hpp"
#include "lo_core/types/flags.hpp"
#include "lo_core/types/inputs.hpp"
#include "lo_core/util/time_ring.hpp"

namespace lo {

/// Graph odometry history with interpolation (seed and deskew translation only).
class GraphOdomBuffer
{
public:
  explicit GraphOdomBuffer(const Config & cfg)
  : ring_(static_cast<std::size_t>(cfg.limits.graph_buffer)),
    timeout_(fromSec(cfg.seed.graph_timeout)) {}

  bool push(const GraphOdomSample & s) {return ring_.push(s);}
  void clear() {ring_.clear();}
  bool empty() const {return ring_.empty();}

  /// Base pose in the graph odom frame at t (interpolated; may hold the newest sample up to the
  /// timeout beyond its stamp).
  bool poseAt(TimeNs t, SE3 & T) const;
  /// Base twist (base axes) at t.
  bool twistAt(TimeNs t, Vec3 & v, Vec3 & w) const;

private:
  TimeRing<GraphOdomSample> ring_;
  TimeNs timeout_;
};

/// LO's own constant-velocity model: twist of the LiDAR frame between the last two scans.
struct LoVelocity
{
  bool valid{false};
  Vec3 v_lidar{Vec3::Zero()};   ///< m/s, LiDAR frame (mean over [t0, t1])
  Vec3 w_lidar{Vec3::Zero()};   ///< rad/s, LiDAR frame
  TimeNs t0{0}, t1{0};          ///< interval the mean was measured on (t0 == t1: unknown)
  /// Long-baseline LO displacement for the graph gate (>= 0.4 s: LO pose noise of ~1 cm is
  /// ~0.04 m/s there, vs ~0.14 m/s over one scan interval).
  bool gate_valid{false};
  TimeNs g0{0}, g1{0};
  Vec3 g_disp{Vec3::Zero()};    ///< LiDAR translation over [g0, g1], LiDAR frame at g0
};

struct SeedResult
{
  SE3 delta;                    ///< lidar(t_prev) -> lidar(t_ref)
  SE3 T_seed;                   ///< odom <- lidar at t_ref
  /// Deskew velocity v0: base-origin velocity in LiDAR axes at t_ref (the deskewer adds the
  /// rotation lever arm exactly, DECISIONS I28).
  Vec3 v_deskew{Vec3::Zero()};
  std::uint32_t flags{0};
  bool rotation_from_gyro{false};
  bool translation_from_graph{false};
  bool graph_gated{false};      ///< graph available but failed the plausibility/consistency gate
};

class SeedModel
{
public:
  SeedModel(const Config & cfg, const Extrinsics & ext);

  /// Not const: the graph gate keeps a hold-off after a failure (message time).
  SeedResult predict(
    TimeNs t_prev, TimeNs t_ref, const SE3 & T_odom_lidar_prev, const ImuBuffer & imu,
    bool imu_usable, const Vec3 & gyro_bias, const GraphOdomBuffer & graph,
    const LoVelocity & lo_vel);
  void reset() {gated_until_ = kNoHoldoff;}

private:
  static constexpr TimeNs kNoHoldoff = -(static_cast<TimeNs>(1) << 62);

  /// Graph translation over [t_prev, t_ref] and LiDAR velocity at t_ref, if the graph passes
  /// the gate: finite, within the platform limits, its pose change agrees with its twist, and
  /// its velocity agrees with LO's own (when LO has one).
  bool graphMotion(TimeNs t_prev, TimeNs t_ref, const GraphOdomBuffer & graph,
    const LoVelocity & lo_vel, Vec3 & dt_lidar, Vec3 & v_base_lidar_axes, bool & gated);

  SE3 T_base_lidar_;
  SE3 T_lidar_base_;
  SO3 R_li_;
  Vec3 lever_;                  ///< R_bl^T t_bl
  double v_max_, w_max_, max_dev_;
  TimeNs holdoff_;
  TimeNs gated_until_{kNoHoldoff};
};

/// Signed block mean of joint velocities over [t - block, t]: max_j |mean qd_j| (finite
/// differences of q if the stream has no velocities). False if fewer than 2 samples.
bool jointBlockSpeed(const TimeRing<JointSample> & ring, TimeNs t, TimeNs block, double & speed);

}  // namespace lo
