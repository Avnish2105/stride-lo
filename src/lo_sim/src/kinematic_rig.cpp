// lo_sim::KinematicRig: gz-sim system that moves the rig model (`stride_rig`) and the crowd
// (`walker_<i>`) exactly along the lo_synth scenario, so the Gazebo view, the analytic sensors
// of lo_sim_rig and the ground truth agree. No physics acts on them (kinematic rig, like the
// GPS-denied robot).
// Time: lo_sim_rig owns the clock (DECISIONS I37). Its ROS /clock is bridged to the gz topic
// `clock_topic` (default /stride/scenario_clock) and the poses follow that time (scenario time =
// clock - start_delay). Gazebo is display only: a slow GUI makes the view lag, never the data.
// Until the first clock message the rig stays at its start pose.
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "lo_synth/simulator.hpp"

namespace lo_sim {

class KinematicRig : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const std::string name = sdf->Get<std::string>("scenario", "room").first;
    const auto seed = static_cast<std::uint64_t>(sdf->Get<int>("seed", 1).first);
    delay_ = sdf->Get<double>("start_delay", 3.0).first;
    loop_ = sdf->Get<bool>("loop", false).first;
    sc_ = std::make_unique<lo::synth::Scenario>(lo::synth::makeScenario(name, seed));
    period_ = std::ceil(sc_->duration * 10.0) / 10.0;
    const std::string topic = sdf->Get<std::string>("clock_topic", "/stride/scenario_clock").first;
    node_.Subscribe(topic, &KinematicRig::onClock, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    if (rig_ == gz::sim::kNullEntity) {
      rig_ = ecm.EntityByComponents(gz::sim::components::Name("stride_rig"),
        gz::sim::components::Model());
      walkers_.clear();
      for (std::size_t i = 0; i < sc_->scene.walkers.size(); ++i) {
        walkers_.push_back(ecm.EntityByComponents(
          gz::sim::components::Name("walker_" + std::to_string(i)), gz::sim::components::Model()));
      }
      if (rig_ == gz::sim::kNullEntity) {return;}
    }
    double t = clock_s_.load() - delay_;
    if (t < 0.0) {t = 0.0;}
    if (loop_ && period_ > 0.0) {t = std::fmod(t, period_);}
    t = std::min(t, sc_->duration);
    setPose(ecm, rig_, sc_->trajectory.pose(t));
    for (std::size_t i = 0; i < walkers_.size(); ++i) {
      if (walkers_[i] == gz::sim::kNullEntity) {continue;}
      const lo::Vec3 p = sc_->scene.walkers[i].positionAt(t);
      setPose(ecm, walkers_[i], lo::SE3(lo::SO3(), lo::Vec3(p.x(), p.y(),
        0.5 * sc_->scene.walkers[i].height)));
    }
  }

private:
  void onClock(const gz::msgs::Clock & m)
  {
    const auto & st = m.has_sim() ? m.sim() : m.system();
    clock_s_.store(static_cast<double>(st.sec()) + 1e-9 * static_cast<double>(st.nsec()));
  }

  static void setPose(gz::sim::EntityComponentManager & ecm, gz::sim::Entity e, const lo::SE3 & T)
  {
    const Eigen::Quaterniond & q = T.unit_quaternion();
    const gz::math::Pose3d pose(T.translation().x(), T.translation().y(), T.translation().z(),
      q.w(), q.x(), q.y(), q.z());
    // Write the pose directly: the world runs no Physics system, which is what would apply a
    // WorldPoseCmd (with it the models never moved). Top-level models: pose == world pose.
    auto * c = ecm.Component<gz::sim::components::Pose>(e);
    if (c == nullptr) {
      ecm.CreateComponent(e, gz::sim::components::Pose(pose));
    } else {
      c->Data() = pose;
    }
    ecm.SetChanged(e, gz::sim::components::Pose::typeId, gz::sim::ComponentState::PeriodicChange);
  }

  std::unique_ptr<lo::synth::Scenario> sc_;
  double delay_{3.0}, period_{0.0};
  bool loop_{false};
  gz::sim::Entity rig_{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> walkers_;
  std::atomic<double> clock_s_{0.0};
  gz::transport::Node node_;
};

}  // namespace lo_sim

GZ_ADD_PLUGIN(lo_sim::KinematicRig, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
