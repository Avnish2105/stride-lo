#include "lo_ros/urdf_kinematics.hpp"

#include <urdf/model.h>

namespace lo_ros {

bool kinematicsFromUrdf(const std::string & xml, const std::string & root, double link_radius,
  double foot_radius, lo::KinematicModel & out, std::string & error)
{
  urdf::Model model;
  if (!model.initString(xml)) {
    error = "robot_description is not a valid URDF";
    return false;
  }
  if (!model.getLink(root)) {
    error = "URDF has no link '" + root + "'";
    return false;
  }
  std::vector<lo::KinJointSpec> specs;
  for (const auto & kv : model.joints_) {
    const urdf::JointSharedPtr & j = kv.second;
    lo::KinJointSpec s;
    s.name = j->name;
    s.parent_link = j->parent_link_name;
    s.child_link = j->child_link_name;
    const auto & o = j->parent_to_joint_origin_transform;
    double qx, qy, qz, qw;
    o.rotation.getQuaternion(qx, qy, qz, qw);
    s.origin = lo::fromXyzQuat(o.position.x, o.position.y, o.position.z, qx, qy, qz, qw);
    s.axis = lo::Vec3(j->axis.x, j->axis.y, j->axis.z);
    switch (j->type) {
      case urdf::Joint::REVOLUTE: s.type = lo::JointType::REVOLUTE; break;
      case urdf::Joint::CONTINUOUS: s.type = lo::JointType::CONTINUOUS; break;
      case urdf::Joint::PRISMATIC: s.type = lo::JointType::PRISMATIC; break;
      default: s.type = lo::JointType::FIXED; break;   // fixed, floating, planar: rigid here
    }
    specs.push_back(s);
  }
  return lo::KinematicModel::build(root, specs, link_radius, foot_radius, out, error);
}

}  // namespace lo_ros
