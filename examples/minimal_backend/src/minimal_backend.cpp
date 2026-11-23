// The smallest useful STRIDE consumer: the wiring a backend plugin needs, without a solver.
//  1. When the backend creates a graph node at time t, publish lo_msgs/NodeRequest{stamp = t}
//     (here: a node every `node_period` s of ROS time; a real backend uses its keyframe times).
//  2. For each LoFactor: ChainTracker decides usable / chain break; match stamp_i and stamp_j to
//     node keys; lo_gtsam::toBetweenFactor gives the BetweenFactor<Pose3> in the backend's body
//     frame (body_T_base). A real backend adds it to its graph; this one only counts.
//  3. Never add a factor between two chains: bridge chain breaks with your own IMU/leg factors.
// See docs/INTEGRATION.md.
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <lo_gtsam_adapter/adapter.hpp>
#include <lo_msgs/msg/lo_factor.hpp>
#include <lo_msgs/msg/node_request.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using gtsam::symbol_shorthand::X;
using lo_msgs::msg::LoFactor;

class MinimalBackend : public rclcpp::Node
{
public:
  MinimalBackend()
  : rclcpp::Node("stride_minimal_backend")
  {
    const double period = declare_parameter("node_period", 0.1);
    tol_ns_ = static_cast<std::int64_t>(declare_parameter("match_tolerance", 0.001) * 1e9);
    // Pose of STRIDE's base frame in the backend's body frame [x y z qx qy qz qw] (identity if
    // the backend state is the base frame itself).
    const auto e = declare_parameter("body_T_base", std::vector<double>{0, 0, 0, 0, 0, 0, 1});
    if (e.size() == 7) {
      body_T_base_ = gtsam::Pose3(gtsam::Rot3::Quaternion(e[6], e[3], e[4], e[5]),
          gtsam::Point3(e[0], e[1], e[2]));
    }
    request_pub_ = create_publisher<lo_msgs::msg::NodeRequest>(
      declare_parameter("request_topic", std::string("/stride_lo/node_request")),
      rclcpp::QoS(100).reliable());
    factor_sub_ = create_subscription<LoFactor>(
      declare_parameter("factor_topic", std::string("/stride_lo/factor")),
      rclcpp::QoS(100).reliable(),
      [this](LoFactor::ConstSharedPtr m) {onFactor(*m);});
    node_timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(period), [this] {createNode();});
    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] {report();});
  }

  ~MinimalBackend() override {report();}

private:
  void createNode()
  {
    const rclcpp::Time t = now();
    if (t.nanoseconds() == 0) {return;}   // sim time not started yet
    lo_msgs::msg::NodeRequest rq;
    rq.stamp = t;
    request_pub_->publish(rq);
    nodes_.emplace(t.nanoseconds(), next_key_++);
    while (nodes_.size() > 10000) {nodes_.erase(nodes_.begin());}
  }

  std::optional<gtsam::Key> nodeAt(std::int64_t t) const
  {
    auto it = nodes_.lower_bound(t - tol_ns_);
    if (it == nodes_.end() || it->first > t + tol_ns_) {return std::nullopt;}
    return X(it->second);
  }

  void onFactor(const LoFactor & f)
  {
    const auto d = tracker_.observe(f);
    if (!lo_gtsam::ChainTracker::usable(d)) {++dropped_; return;}
    if (d == lo_gtsam::ChainTracker::Decision::NEW_CHAIN) {
      // A real backend keeps going: the IMU factors connect the chains; LO never does.
      RCLCPP_WARN(get_logger(), "LO chain break -> chain %lu", static_cast<unsigned long>(f.chain_id));
    }
    const auto ki = nodeAt(lo_gtsam::stampNs(f.stamp_i));
    const auto kj = nodeAt(lo_gtsam::stampNs(f.stamp_j));
    if (!ki || !kj) {++unmatched_; return;}   // e.g. STRIDE in overlap mode
    const auto between = lo_gtsam::toBetweenFactor(f, *ki, *kj, body_T_base_);
    if (!between) {++rejected_; return;}      // Sigma not SPD: never add
    ++accepted_;
    if (f.health != LoFactor::HEALTH_OK) {++not_ok_;}
    // graph.add(*between);   <- your backend
  }

  void report()
  {
    RCLCPP_INFO(get_logger(), "factors: %lu accepted (%lu not OK), %lu unmatched, %lu dropped, "
      "%lu rejected, %lu chain breaks", static_cast<unsigned long>(accepted_),
      static_cast<unsigned long>(not_ok_), static_cast<unsigned long>(unmatched_),
      static_cast<unsigned long>(dropped_), static_cast<unsigned long>(rejected_),
      static_cast<unsigned long>(tracker_.breaks()));
  }

  std::int64_t tol_ns_{1000000};
  gtsam::Pose3 body_T_base_;
  std::map<std::int64_t, std::uint64_t> nodes_;
  std::uint64_t next_key_{0};
  lo_gtsam::ChainTracker tracker_;
  std::uint64_t accepted_{0}, not_ok_{0}, unmatched_{0}, dropped_{0}, rejected_{0};
  rclcpp::Publisher<lo_msgs::msg::NodeRequest>::SharedPtr request_pub_;
  rclcpp::Subscription<LoFactor>::SharedPtr factor_sub_;
  rclcpp::TimerBase::SharedPtr node_timer_, report_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalBackend>());
  rclcpp::shutdown();
  return 0;
}
