// lo_graph_demo: a minimal GTSAM backend that consumes STRIDE LoFactors the way
// factor_graph_estimator would (docs/DECISIONS.md D2), for live checks and demos.
//  * graph nodes every `node_period` s on IMU stamps (like the estimator's keyframes); each node
//    time is published as lo_msgs/NodeRequest so STRIDE answers exactly at it;
//  * every usable LoFactor becomes a BetweenFactor<Pose3> between the nodes at stamp_i/stamp_j
//    (iSAM2); a new chain gets a weak prior at the current estimate (there is no IMU factor here
//    to bridge chain breaks, unlike the real estimator);
//  * publishes ~/path (all node estimates) and ~/pose (latest node with marginal covariance).
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <lo_msgs/msg/lo_factor.hpp>
#include <lo_msgs/msg/node_request.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "lo_gtsam_adapter/adapter.hpp"

namespace {

using gtsam::symbol_shorthand::X;

geometry_msgs::msg::Pose toPose(const gtsam::Pose3 & T)
{
  geometry_msgs::msg::Pose p;
  const gtsam::Quaternion q = T.rotation().toQuaternion();
  p.position.x = T.x();
  p.position.y = T.y();
  p.position.z = T.z();
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  return p;
}

builtin_interfaces::msg::Time toStamp(std::int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  return t;
}

class GraphDemo : public rclcpp::Node
{
public:
  GraphDemo()
  : rclcpp::Node("lo_graph_demo")
  {
    node_period_ns_ = static_cast<std::int64_t>(declare_parameter("node_period", 0.1) * 1e9);
    match_tol_ns_ = static_cast<std::int64_t>(declare_parameter("match_tolerance", 0.001) * 1e9);
    frame_ = declare_parameter("frame", std::string("graph_odom"));
    accept_low_confidence_ = declare_parameter("accept_low_confidence", true);
    const auto imu_topic = declare_parameter("imu_topic", std::string("/imu/data"));
    const auto factor_topic = declare_parameter("factor_topic", std::string("/stride_lo/factor"));
    const auto request_topic = declare_parameter("request_topic", std::string("/stride_lo/node_request"));
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);

    request_pub_ = create_publisher<lo_msgs::msg::NodeRequest>(request_topic, rclcpp::QoS(100).reliable());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});
    factor_sub_ = create_subscription<lo_msgs::msg::LoFactor>(factor_topic, rclcpp::QoS(100).reliable(),
        [this](lo_msgs::msg::LoFactor::ConstSharedPtr m) {onFactor(*m);});
    path_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {publishPath();});
    RCLCPP_INFO(get_logger(), "graph nodes every %.3f s on %s; factors from %s",
      static_cast<double>(node_period_ns_) * 1e-9, imu_topic.c_str(), factor_topic.c_str());
  }

private:
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const std::int64_t t = lo_gtsam::stampNs(m.header.stamp);
    if (next_node_ns_ == 0) {next_node_ns_ = t;}
    if (t < next_node_ns_) {return;}
    lo_msgs::msg::NodeRequest rq;
    rq.stamp = m.header.stamp;
    request_pub_->publish(rq);
    nodes_.emplace(t, next_key_++);
    next_node_ns_ += node_period_ns_;
    if (next_node_ns_ <= t) {next_node_ns_ = t + node_period_ns_;}
    while (nodes_.size() > 20000) {nodes_.erase(nodes_.begin());}
  }

  /// Key of the node published at (within match_tolerance of) stamp t.
  std::optional<gtsam::Key> nodeAt(std::int64_t t) const
  {
    auto it = nodes_.lower_bound(t - match_tol_ns_);
    if (it == nodes_.end() || it->first > t + match_tol_ns_) {return std::nullopt;}
    return X(it->second);
  }

  void onFactor(const lo_msgs::msg::LoFactor & f)
  {
    const auto decision = tracker_.observe(f);
    if (!lo_gtsam::ChainTracker::usable(decision)) {
      ++dropped_;
      return;
    }
    if (f.health == lo_msgs::msg::LoFactor::HEALTH_LOW_CONFIDENCE && !accept_low_confidence_) {
      ++dropped_;
      return;
    }
    const auto ki = nodeAt(lo_gtsam::stampNs(f.stamp_i));
    const auto kj = nodeAt(lo_gtsam::stampNs(f.stamp_j));
    if (!ki || !kj) {   // not one of our nodes (e.g. overlap-mode keyframes)
      ++unmatched_;
      return;
    }
    auto between = lo_gtsam::toBetweenFactor(f, *ki, *kj);
    if (!between) {
      ++dropped_;
      RCLCPP_WARN(get_logger(), "factor %.3f -> %.3f: covariance not SPD, dropped",
        lo_gtsam::stampNs(f.stamp_i) * 1e-9, lo_gtsam::stampNs(f.stamp_j) * 1e-9);
      return;
    }
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    if (!estimate_.exists(*ki)) {
      // first node of the graph (origin) or of a new chain (weak prior at the last estimate)
      const bool origin = !have_last_;
      const gtsam::Pose3 Ti = origin ? gtsam::Pose3() : last_pose_;
      const double s_rot = origin ? 1e-6 : 10.0, s_pos = origin ? 1e-6 : 1e3;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(*ki, Ti, gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() << s_rot, s_rot, s_rot, s_pos, s_pos, s_pos).finished())));
      values.insert(*ki, Ti);
      estimate_.insert(*ki, Ti);
      if (!origin) {
        RCLCPP_WARN(get_logger(), "chain break (%s): new chain %lu starts with a weak prior",
          lo_gtsam::toString(decision), static_cast<unsigned long>(f.chain_id));
      }
    }
    if (!estimate_.exists(*kj)) {
      const gtsam::Pose3 Tj = estimate_.at<gtsam::Pose3>(*ki) * between->measured();
      values.insert(*kj, Tj);
      estimate_.insert(*kj, Tj);
    }
    graph.add(*between);
    isam_->update(graph, values);
    estimate_ = isam_->calculateEstimate();
    last_pose_ = estimate_.at<gtsam::Pose3>(*kj);
    have_last_ = true;
    ++factors_;
    keys_by_time_[lo_gtsam::stampNs(f.stamp_j)] = *kj;

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = f.stamp_j;
    out.header.frame_id = frame_;
    out.pose.pose = toPose(last_pose_);
    const gtsam::Matrix6 C = isam_->marginalCovariance(*kj);   // [rx ry rz tx ty tz] local
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        // ROS PoseWithCovariance order is [x y z rx ry rz]
        out.pose.covariance[static_cast<std::size_t>(((r + 3) % 6) * 6 + (c + 3) % 6)] = C(r, c);
      }
    }
    pose_pub_->publish(out);
  }

  void publishPath()
  {
    if (keys_by_time_.empty()) {return;}
    nav_msgs::msg::Path path;
    path.header.stamp = toStamp(keys_by_time_.rbegin()->first);
    path.header.frame_id = frame_;
    for (const auto & [t, key] : keys_by_time_) {
      if (!estimate_.exists(key)) {continue;}
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = toStamp(t);
      ps.header.frame_id = frame_;
      ps.pose = toPose(estimate_.at<gtsam::Pose3>(key));
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "graph: %lu factors, %lu unmatched, %lu dropped, %lu chain breaks",
      static_cast<unsigned long>(factors_), static_cast<unsigned long>(unmatched_),
      static_cast<unsigned long>(dropped_), static_cast<unsigned long>(tracker_.breaks()));
  }

  std::int64_t node_period_ns_{100000000}, match_tol_ns_{1000000}, next_node_ns_{0};
  std::string frame_;
  bool accept_low_confidence_{true};
  std::map<std::int64_t, std::uint64_t> nodes_;       ///< published node time -> index
  std::map<std::int64_t, gtsam::Key> keys_by_time_;   ///< nodes in the graph
  std::uint64_t next_key_{0};
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values estimate_;
  gtsam::Pose3 last_pose_;
  bool have_last_{false};
  lo_gtsam::ChainTracker tracker_;
  std::uint64_t factors_{0}, unmatched_{0}, dropped_{0};
  rclcpp::Publisher<lo_msgs::msg::NodeRequest>::SharedPtr request_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<lo_msgs::msg::LoFactor>::SharedPtr factor_sub_;
  rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphDemo>());
  rclcpp::shutdown();
  return 0;
}
