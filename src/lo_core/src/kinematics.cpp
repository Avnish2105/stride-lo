#include "lo_core/preprocess/kinematics.hpp"

#include <map>

namespace lo {

bool KinematicModel::build(
  const std::string & root, const std::vector<KinJointSpec> & joints, double link_radius,
  double foot_radius, KinematicModel & out, std::string & error)
{
  out = KinematicModel();
  // children of each link
  std::multimap<std::string, const KinJointSpec *> children;
  for (const auto & j : joints) {children.emplace(j.parent_link, &j);}

  // Breadth-first from the root: parents always precede children.
  out.nodes_.push_back(Node{});
  out.link_names_.push_back(root);
  std::vector<std::size_t> queue{0};
  for (std::size_t qi = 0; qi < queue.size(); ++qi) {
    const int parent = static_cast<int>(queue[qi]);
    const std::string parent_name = out.link_names_[parent];
    auto range = children.equal_range(parent_name);
    for (auto it = range.first; it != range.second; ++it) {
      const KinJointSpec & j = *it->second;
      for (const auto & name : out.link_names_) {
        if (name == j.child_link) {
          error = "kinematic loop at link " + j.child_link;
          return false;
        }
      }
      Node n;
      n.parent = parent;
      n.origin = j.origin;
      n.type = j.type;
      const double an = j.axis.norm();
      n.axis = an > 1e-9 ? Vec3(j.axis / an) : Vec3::UnitZ();
      if (j.type != JointType::FIXED) {
        n.q_index = static_cast<int>(out.joint_names_.size());
        out.joint_names_.push_back(j.name);
      }
      n.moving = (j.type != JointType::FIXED) || out.nodes_[parent].moving;
      out.nodes_.push_back(n);
      out.link_names_.push_back(j.child_link);
      queue.push_back(out.nodes_.size() - 1);
    }
  }
  if (out.joint_names_.empty()) {
    error = "no movable joint below root link '" + root + "'";
    return false;
  }

  // Capsules: bone to each child for moving links, sphere for moving leaves.
  std::vector<int> num_children(out.nodes_.size(), 0);
  for (std::size_t i = 1; i < out.nodes_.size(); ++i) {++num_children[out.nodes_[i].parent];}
  for (std::size_t i = 1; i < out.nodes_.size(); ++i) {
    const Node & n = out.nodes_[i];
    if (!n.moving) {continue;}
    if (num_children[i] == 0) {
      out.caps_.push_back(Cap{static_cast<int>(i), Vec3::Zero(), Vec3::Zero(), foot_radius});
    }
  }
  for (std::size_t c = 1; c < out.nodes_.size(); ++c) {
    const Node & child = out.nodes_[c];
    const int p = child.parent;
    if (p <= 0 || !out.nodes_[p].moving) {continue;}
    out.caps_.push_back(Cap{p, Vec3::Zero(), child.origin.translation(), link_radius});
  }
  out.poses_.assign(out.nodes_.size(), SE3());
  return true;
}

void KinematicModel::capsules(const double * q, Vec3 * a, Vec3 * b) const
{
  poses_[0] = SE3();
  for (std::size_t i = 1; i < nodes_.size(); ++i) {
    const Node & n = nodes_[i];
    SE3 motion;
    if (n.q_index >= 0) {
      const double v = q[n.q_index];
      if (n.type == JointType::PRISMATIC) {
        motion = SE3(SO3(), n.axis * v);
      } else {
        motion = SE3(SO3::exp(n.axis * v), Vec3::Zero());
      }
    }
    poses_[i] = poses_[n.parent] * n.origin * motion;
  }
  for (std::size_t c = 0; c < caps_.size(); ++c) {
    const SE3 & T = poses_[caps_[c].node];
    a[c] = T * caps_[c].a;
    b[c] = T * caps_[c].b;
  }
}

}  // namespace lo
