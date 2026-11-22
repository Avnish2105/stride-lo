// Neutral input samples (the ROS layer / replay tools convert messages into these).
#pragma once

#include <array>
#include <cstdint>

#include "lo_core/types/se3.hpp"
#include "lo_core/types/time.hpp"

namespace lo {

/// IMU sample in the IMU frame. Accelerometer already in m/s^2 (Livox g scaled by the caller).
struct ImuSample
{
  TimeNs t{0};
  Vec3 gyro{Vec3::Zero()};   ///< rad/s
  Vec3 acc{Vec3::Zero()};    ///< m/s^2 (specific force)
};

constexpr int kMaxJoints = 32;

/// Joint state, ordered like KinematicModel::jointNames() (the caller maps names once).
struct JointSample
{
  TimeNs t{0};
  int n{0};
  std::array<double, kMaxJoints> q{};
  std::array<double, kMaxJoints> qd{};
  bool has_velocity{false};
};

/// Graph odometry (nav_msgs/Odometry from the backend): pose of base in the graph's odom frame
/// and the base twist in base axes. Used only as a seed / deskew translation (never fused).
struct GraphOdomSample
{
  TimeNs t{0};
  SE3 T_odom_base;
  Vec3 v_base{Vec3::Zero()};
  Vec3 w_base{Vec3::Zero()};
};

/// Rigid extrinsics. T_base_lidar maps LiDAR-frame points into the base frame.
struct Extrinsics
{
  SE3 T_base_lidar;
  SE3 T_lidar_imu;   ///< only the rotation matters for gyro integration
};

}  // namespace lo
