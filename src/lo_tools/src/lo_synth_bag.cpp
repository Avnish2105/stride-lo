// lo_synth_bag <scenario> <out_dir> [--seed N] [--tmax S]
// Writes a lo_synth scenario as an MCAP bag with the same topics/types as the real drivers:
//   /livox/lidar (CustomMsg), /livox/imu (Imu, accel in g), /joint_states, /ground_truth/odom,
//   /tf_static (base -> livox_frame), /estimator/odometry_keyframe stamps as node requests
//   (nav_msgs/Odometry with identity pose: only the stamp matters).
// Receive timestamps = simulated arrival times, so lo_replay and `ros2 bag play` behave like
// the live system. Use with fault_inject.py / lo_replay / the ROS node.
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "lo_ros/conversions.hpp"
#include "lo_synth/simulator.hpp"

namespace {

class BagSink : public lo::synth::Sink
{
public:
  BagSink(rosbag2_cpp::Writer & w, const lo::synth::Simulator & sim) : w_(w), sim_(sim)
  {
    joint_names_ = sim.robot().model().jointNames();
  }

  void onImu(const lo::ImuSample & s, lo::TimeNs arrival) override
  {
    sensor_msgs::msg::Imu m;
    m.header.stamp = lo_ros::toStamp(s.t);
    m.header.frame_id = "livox_frame";
    m.angular_velocity.x = s.gyro.x();
    m.angular_velocity.y = s.gyro.y();
    m.angular_velocity.z = s.gyro.z();
    m.linear_acceleration.x = s.acc.x();
    m.linear_acceleration.y = s.acc.y();
    m.linear_acceleration.z = s.acc.z();
    w_.write(m, "/livox/imu", rclcpp::Time(arrival, RCL_ROS_TIME));
    // ground truth at 50 Hz on the IMU clock
    if (s.t - last_gt_ >= 20000000LL) {
      last_gt_ = s.t;
      nav_msgs::msg::Odometry gt;
      gt.header.stamp = m.header.stamp;
      gt.header.frame_id = "world";
      gt.child_frame_id = "base";
      gt.pose.pose = lo_ros::toPose(sim_.groundTruth(s.t));
      w_.write(gt, "/ground_truth/odom", rclcpp::Time(arrival, RCL_ROS_TIME));
    }
  }

  void onJoints(const lo::JointSample & s, lo::TimeNs arrival) override
  {
    sensor_msgs::msg::JointState m;
    m.header.stamp = lo_ros::toStamp(s.t);
    m.name = joint_names_;
    m.position.assign(s.q.begin(), s.q.begin() + s.n);
    m.velocity.assign(s.qd.begin(), s.qd.begin() + s.n);
    w_.write(m, "/joint_states", rclcpp::Time(arrival, RCL_ROS_TIME));
  }

  void onRequest(lo::TimeNs stamp, lo::TimeNs arrival) override
  {
    nav_msgs::msg::Odometry m;
    m.header.stamp = lo_ros::toStamp(stamp);
    m.header.frame_id = "odom";
    w_.write(m, "/estimator/odometry_keyframe", rclcpp::Time(arrival, RCL_ROS_TIME));
  }

  void onScan(const lo::RawScan & scan, lo::TimeNs arrival, lo::TimeNs) override
  {
    livox_ros_driver2::msg::CustomMsg m;
    lo_ros::toCustomMsg(scan, "livox_frame", m);
    w_.write(m, "/livox/lidar", rclcpp::Time(arrival, RCL_ROS_TIME));
  }

private:
  rosbag2_cpp::Writer & w_;
  const lo::synth::Simulator & sim_;
  std::vector<std::string> joint_names_;
  lo::TimeNs last_gt_{-1000000000LL};
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: lo_synth_bag <scenario> <out_dir> [--seed N] [--tmax S]\n");
    return 2;
  }
  const std::string name = argv[1], out = argv[2];
  std::uint64_t seed = 1;
  double tmax = -1.0;
  for (int i = 3; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) {seed = std::strtoull(argv[++i], nullptr, 10);}
    else if (!std::strcmp(argv[i], "--tmax") && i + 1 < argc) {tmax = std::atof(argv[++i]);}
  }
  lo::synth::Simulator sim(lo::synth::makeScenario(name, seed));
  std::filesystem::remove_all(out);
  rosbag2_cpp::Writer w;
  rosbag2_storage::StorageOptions so;
  so.uri = out;
  so.storage_id = "mcap";
  so.custom_data = {{"stride_synthetic_scenario", name}, {"seed", std::to_string(seed)}};
  w.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  tf2_msgs::msg::TFMessage tf;
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "base";
  t.child_frame_id = "livox_frame";
  const lo::SE3 & T = sim.scenario().T_base_lidar;
  const auto pose = lo_ros::toPose(T);
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation = pose.orientation;
  tf.transforms.push_back(t);
  w.write(tf, "/tf_static", rclcpp::Time(0, 0, RCL_ROS_TIME));
  BagSink sink(w, sim);
  sim.run(sink, tmax);
  w.close();
  std::printf("lo_synth_bag: wrote %s (%s, %.1f s)\n", out.c_str(), name.c_str(), sim.scenario().duration);
  return 0;
}
