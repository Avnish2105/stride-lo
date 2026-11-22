// lo_sim: descriptions of the synthetic rig shared by the sensor node, the world generator and
// the Gazebo plugin (all driven by the same lo_synth scenario, so geometry, motion and sensor
// data agree exactly).
#pragma once

#include <string>

#include "lo_synth/simulator.hpp"

namespace lo_sim {

/// URDF of the synthetic quadruped (base, 4 legs x HAA/HFE/KFE, livox_frame) for
/// /robot_description: the STRIDE node's leg self-filter is built from it.
std::string rigUrdf(const lo::synth::Scenario & sc, const lo::synth::SyntheticRobot & robot);

/// Gazebo world (SDF 1.9) with the scenario's static primitives, one model per walker
/// (walker_<i>), the rig model `stride_rig` and the lo_sim KinematicRig system plugin that moves
/// them along the scenario trajectories.
std::string worldSdf(const std::string & scenario, std::uint64_t seed, const lo::synth::Scenario & sc);

}  // namespace lo_sim
