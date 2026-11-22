// ROS message <-> lo_core conversions (the only place that touches both). Quaternions in ROS
// messages are (x, y, z, w); Eigen constructors take (w, x, y, z): only convert here.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lo_msgs/msg/lo_factor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lo_core/types/inputs.hpp"
#include "lo_core/types/outputs.hpp"
#include "lo_core/types/points.hpp"

namespace lo_ros {

inline lo::TimeNs toNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<lo::TimeNs>(t.sec) * lo::kNsPerSec + static_cast<lo::TimeNs>(t.nanosec);
}

inline builtin_interfaces::msg::Time toStamp(lo::TimeNs t)
{
  builtin_interfaces::msg::Time s;
  lo::TimeNs sec = t / lo::kNsPerSec;
  lo::TimeNs ns = t % lo::kNsPerSec;
  if (ns < 0) {
    ns += lo::kNsPerSec;
    --sec;
  }
  s.sec = static_cast<int32_t>(sec);
  s.nanosec = static_cast<uint32_t>(ns);
  return s;
}

lo::SE3 toSE3(const geometry_msgs::msg::Pose & p);
lo::SE3 toSE3(const geometry_msgs::msg::Transform & t);
geometry_msgs::msg::Pose toPose(const lo::SE3 & T);

/// CustomMsg -> RawPoint buffer (resized up to max_points, never beyond its capacity).
/// Returns the scan view (t0 = timebase).
lo::RawScan toRawScan(const livox_ros_driver2::msg::CustomMsg & msg, std::vector<lo::RawPoint> & buf);
/// Inverse of toRawScan (synthetic sensors: lo_synth_bag, lo_sim): timebase = scan.t0.
void toCustomMsg(const lo::RawScan & scan, const std::string & frame_id,
  livox_ros_driver2::msg::CustomMsg & msg);

lo::ImuSample toImuSample(const sensor_msgs::msg::Imu & msg);

/// Maps JointState names to the KinematicModel order (rebuilt when the name list changes).
class JointMapper
{
public:
  explicit JointMapper(std::vector<std::string> model_joints) : model_(std::move(model_joints)) {}
  /// false if a model joint is missing from the message.
  bool convert(const sensor_msgs::msg::JointState & msg, lo::JointSample & out);
  const std::vector<std::string> & missing() const {return missing_;}

private:
  void rebuild(const std::vector<std::string> & names);
  std::vector<std::string> model_;
  std::vector<std::string> cached_names_;
  std::vector<int> index_;   ///< model joint -> message index (-1 missing)
  std::vector<std::string> missing_;
};

lo::GraphOdomSample toGraphOdom(const nav_msgs::msg::Odometry & msg);

/// FactorOut -> LoFactor (covariance row-major [rx ry rz tx ty tz], directions flattened).
lo_msgs::msg::LoFactor toMsg(const lo::FactorOut & f);
/// LoFactor -> FactorOut (tests, tools).
lo::FactorOut fromMsg(const lo_msgs::msg::LoFactor & m);

/// Debug odometry (LO frame; never fused). Covariance left zero except a marker diagonal.
nav_msgs::msg::Odometry toOdometry(const lo::ScanResult & r, const std::string & odom_frame,
  const std::string & base_frame);

/// xyz float32 cloud (+ per-point time relative to the reference, float32).
void toPointCloud2(const lo::Cloud & cloud, const lo::SE3 & T, std::size_t stride,
  sensor_msgs::msg::PointCloud2 & out);
void toPointCloud2(const std::vector<lo::Vec3> & pts, std::size_t n,
  sensor_msgs::msg::PointCloud2 & out);

}  // namespace lo_ros
