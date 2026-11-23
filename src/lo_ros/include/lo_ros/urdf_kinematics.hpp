// URDF (robot_description) -> lo::KinematicModel for the S1 leg self-filter.
#pragma once

#include <string>

#include "lo_core/preprocess/kinematics.hpp"

namespace lo_ros {

/// Parse a URDF XML string; root = the base frame (link name).
bool kinematicsFromUrdf(const std::string & xml, const std::string & root, double link_radius,
  double foot_radius, lo::KinematicModel & out, std::string & error);

}  // namespace lo_ros
