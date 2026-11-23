// Synthetic quadruped (~0.5 m tall) for self-filter and standstill tests: 4 legs x (HAA, HFE,
// KFE), described with lo_core KinJointSpecs, so the same KinematicModel drives both the
// simulated leg returns and the pipeline's self-filter.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "lo_core/preprocess/kinematics.hpp"

namespace lo::synth {

struct LegGeometry
{
  double hip_x = 0.30;       ///< HAA joint x (front +, hind -)
  double hip_y = 0.10;       ///< HAA joint |y|
  double hip_offset = 0.08;  ///< HAA -> HFE lateral offset
  double thigh = 0.25;
  double shank = 0.25;
  double radius = 0.04;      ///< bone capsule radius
  double foot_radius = 0.03;
};

class SyntheticRobot
{
public:
  explicit SyntheticRobot(const LegGeometry & g = LegGeometry());

  std::vector<KinJointSpec> joints() const;   ///< root link "base"
  const KinematicModel & model() const {return model_;}
  int numJoints() const {return model_.numJoints();}

  /// Joint positions / velocities at time t for gait activity k (0 standing, 1 trot).
  void jointState(double t, double k, double * q, double * qd) const;

  /// Keep one leg (0 LF, 1 RF, 2 LH, 3 RH) in a fixed pose (e.g. raised into the LiDAR FoV).
  void setLegOverride(int leg, double haa, double hfe, double kfe);

  /// Capsules (world frame) for joint positions q and base pose.
  void capsulesWorld(const SE3 & T_world_base, const double * q, std::vector<Vec3> & a,
    std::vector<Vec3> & b, std::vector<double> & r) const;

  double gaitFrequency() const {return 2.0;}

private:
  LegGeometry g_;
  KinematicModel model_;
  int override_leg_{-1};
  std::array<double, 3> override_q_{};
  std::array<int, 12> joint_index_{};   ///< (leg*3 + j) -> model joint index
  mutable std::vector<Vec3> ca_, cb_;
};

}  // namespace lo::synth
