// Smooth ground-truth trajectories: a sequence of commands (stand still, move straight, turn in
// place), each with a minimum-jerk profile (C2: velocity and acceleration are zero at segment
// ends), plus a trot overlay (bounce, roll, pitch) while moving. Derivatives (IMU) by central
// differences of the analytic pose.
#pragma once

#include <vector>

#include "lo_core/types/se3.hpp"

namespace lo::synth {

struct GaitParams
{
  double freq = 2.0;      ///< stride frequency [Hz]
  double bounce = 0.008;  ///< body z amplitude [m]
  double roll = 0.012;    ///< [rad]
  double pitch = 0.008;   ///< [rad]
};

class Trajectory
{
public:
  /// Start pose (base) and nominal base height above the floor at z = 0 of the start.
  Trajectory(const Vec3 & start, double start_yaw, const GaitParams & gait = GaitParams());

  Trajectory & still(double duration);
  Trajectory & moveTo(const Vec3 & p, double speed);
  /// Turn in place to an absolute yaw [rad] (the shortest way unless `long_way`).
  Trajectory & turnTo(double yaw, double yaw_rate, bool long_way = false);
  /// Turn by a relative angle (can exceed pi).
  Trajectory & turnBy(double angle, double yaw_rate);

  double duration() const {return segs_.empty() ? 0.0 : segs_.back().t0 + segs_.back().T;}
  SE3 pose(double t) const;               ///< world <- base
  Vec3 omegaBody(double t) const;         ///< body angular rate
  Vec3 pointAccelWorld(double t, const Vec3 & r_body) const;   ///< accel of a body-fixed point
  double gaitActivity(double t) const;    ///< 0 standing .. 1 full trot
  bool moving(double t) const {return gaitActivity(t) > 0.02;}

private:
  struct Segment
  {
    enum Kind { STILL, MOVE, TURN } kind{STILL};
    double t0{0}, T{0};
    Vec3 p0{Vec3::Zero()}, p1{Vec3::Zero()};
    double yaw0{0}, yaw1{0};
  };
  const Segment & segmentAt(double t) const;
  static double mj(double tau) {return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));}
  static double mjd(double tau) {return 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau);}

  GaitParams gait_;
  Vec3 cur_p_;
  double cur_yaw_;
  std::vector<Segment> segs_;
};

}  // namespace lo::synth
