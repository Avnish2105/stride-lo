#include "lo_synth/robot.hpp"

#include <cmath>
#include <stdexcept>

namespace lo::synth {

namespace {
const char * kLegs[4] = {"LF", "RF", "LH", "RH"};
const char * kJoints[3] = {"HAA", "HFE", "KFE"};
}  // namespace

SyntheticRobot::SyntheticRobot(const LegGeometry & g)
: g_(g)
{
  std::string err;
  if (!KinematicModel::build("base", joints(), g_.radius, g_.foot_radius, model_, err)) {
    throw std::runtime_error("synthetic robot: " + err);
  }
  const auto & names = model_.jointNames();
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      const std::string n = std::string(kLegs[leg]) + "_" + kJoints[j];
      for (std::size_t i = 0; i < names.size(); ++i) {
        if (names[i] == n) {joint_index_[leg * 3 + j] = static_cast<int>(i);}
      }
    }
  }
  ca_.resize(static_cast<std::size_t>(model_.numCapsules()));
  cb_.resize(ca_.size());
}

std::vector<KinJointSpec> SyntheticRobot::joints() const
{
  std::vector<KinJointSpec> js;
  for (int leg = 0; leg < 4; ++leg) {
    const double sx = (leg < 2) ? 1.0 : -1.0;
    const double sy = (leg % 2 == 0) ? 1.0 : -1.0;
    const std::string L = kLegs[leg];
    KinJointSpec haa;
    haa.name = L + "_HAA";
    haa.parent_link = "base";
    haa.child_link = L + "_HIP";
    haa.origin = SE3(SO3(), Vec3(sx * g_.hip_x, sy * g_.hip_y, 0.0));
    haa.axis = Vec3::UnitX();
    haa.type = JointType::REVOLUTE;
    KinJointSpec hfe;
    hfe.name = L + "_HFE";
    hfe.parent_link = L + "_HIP";
    hfe.child_link = L + "_THIGH";
    hfe.origin = SE3(SO3(), Vec3(0.0, sy * g_.hip_offset, 0.0));
    hfe.axis = Vec3::UnitY();
    hfe.type = JointType::REVOLUTE;
    KinJointSpec kfe;
    kfe.name = L + "_KFE";
    kfe.parent_link = L + "_THIGH";
    kfe.child_link = L + "_SHANK";
    kfe.origin = SE3(SO3(), Vec3(0.0, 0.0, -g_.thigh));
    kfe.axis = Vec3::UnitY();
    kfe.type = JointType::REVOLUTE;
    KinJointSpec foot;
    foot.name = L + "_FOOT_JOINT";
    foot.parent_link = L + "_SHANK";
    foot.child_link = L + "_FOOT";
    foot.origin = SE3(SO3(), Vec3(0.0, 0.0, -g_.shank));
    foot.type = JointType::FIXED;
    js.insert(js.end(), {haa, hfe, kfe, foot});
  }
  return js;
}

void SyntheticRobot::setLegOverride(int leg, double haa, double hfe, double kfe)
{
  override_leg_ = leg;
  override_q_ = {haa, hfe, kfe};
}

void SyntheticRobot::jointState(double t, double k, double * q, double * qd) const
{
  const double w = 2.0 * M_PI * gaitFrequency();
  for (int leg = 0; leg < 4; ++leg) {
    // trot: LF + RH in phase, RF + LH opposite
    const double phase = (leg == 0 || leg == 3) ? 0.0 : M_PI;
    const double s = std::sin(w * t + phase);
    const double c = std::cos(w * t + phase);
    double haa = 0.0, hfe = 0.6 + k * 0.25 * s, kfe = -1.2 - k * 0.35 * std::max(0.0, s);
    double dhaa = 0.0, dhfe = k * 0.25 * w * c, dkfe = (s > 0.0) ? -k * 0.35 * w * c : 0.0;
    if (leg == override_leg_) {
      haa = override_q_[0]; hfe = override_q_[1]; kfe = override_q_[2];
      dhaa = dhfe = dkfe = 0.0;
    }
    q[joint_index_[leg * 3 + 0]] = haa;
    q[joint_index_[leg * 3 + 1]] = hfe;
    q[joint_index_[leg * 3 + 2]] = kfe;
    if (qd) {
      qd[joint_index_[leg * 3 + 0]] = dhaa;
      qd[joint_index_[leg * 3 + 1]] = dhfe;
      qd[joint_index_[leg * 3 + 2]] = dkfe;
    }
  }
}

void SyntheticRobot::capsulesWorld(const SE3 & T_world_base, const double * q,
  std::vector<Vec3> & a, std::vector<Vec3> & b, std::vector<double> & r) const
{
  model_.capsules(q, ca_.data(), cb_.data());
  const std::size_t n = ca_.size();
  a.resize(n);
  b.resize(n);
  r.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    a[i] = T_world_base * ca_[i];
    b[i] = T_world_base * cb_[i];
    r[i] = model_.capsuleRadius(static_cast<int>(i));
  }
}

}  // namespace lo::synth
