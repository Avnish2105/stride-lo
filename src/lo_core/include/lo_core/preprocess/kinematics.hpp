// Minimal kinematic tree for the S1 leg self-filter. Robot-agnostic: lo_ros fills it from the
// URDF (robot_description), lo_core never parses files.
//
// Capsules are derived from the tree, no per-robot geometry config:
//   * a link "moves" if any joint between it and the root is not fixed (legs; not the body),
//   * every moving link gets one capsule per child link: the segment from its own origin to the
//     child's joint origin (thigh, shank, ... approximated by the bone between joints),
//   * a moving link without children (foot) gets a sphere at its origin.
#pragma once

#include <string>
#include <vector>

#include "lo_core/types/se3.hpp"

namespace lo {

enum class JointType { FIXED, REVOLUTE, CONTINUOUS, PRISMATIC };

/// One URDF joint (parent link -> child link).
struct KinJointSpec
{
  std::string name;
  std::string parent_link;
  std::string child_link;
  SE3 origin;               ///< parent link frame -> joint frame at q = 0
  Vec3 axis{Vec3::UnitZ()}; ///< in the joint frame
  JointType type{JointType::FIXED};
};

class KinematicModel
{
public:
  /// Build the subtree below `root` (the base frame). Links not connected to it are ignored.
  /// link_radius: capsule radius of moving links; foot_radius: sphere radius of moving leaves.
  static bool build(
    const std::string & root, const std::vector<KinJointSpec> & joints, double link_radius,
    double foot_radius, KinematicModel & out, std::string & error);

  /// Movable joints in the order JointSample::q must use.
  const std::vector<std::string> & jointNames() const {return joint_names_;}
  int numJoints() const {return static_cast<int>(joint_names_.size());}
  int numCapsules() const {return static_cast<int>(caps_.size());}
  double capsuleRadius(int c) const {return caps_[c].radius;}
  const std::vector<std::string> & linkNames() const {return link_names_;}

  /// Capsule endpoints in the root frame for joint positions q[numJoints()]. a, b: numCapsules().
  /// Uses an internal scratch buffer (not thread-safe; no allocation).
  void capsules(const double * q, Vec3 * a, Vec3 * b) const;

private:
  struct Node
  {
    int parent{-1};          ///< parent node index (-1 = root)
    SE3 origin;
    Vec3 axis{Vec3::UnitZ()};
    JointType type{JointType::FIXED};
    int q_index{-1};         ///< index into q, -1 if fixed
    bool moving{false};
  };
  struct Cap
  {
    int node{0};
    Vec3 a;                  ///< in the node's link frame
    Vec3 b;
    double radius{0.0};
  };

  std::vector<Node> nodes_;              ///< topological order, nodes_[0] = root
  std::vector<std::string> link_names_;
  std::vector<std::string> joint_names_;
  std::vector<Cap> caps_;
  mutable std::vector<SE3> poses_;
};

}  // namespace lo
