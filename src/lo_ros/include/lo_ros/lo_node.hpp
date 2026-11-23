// lo_ros::LoNode: the STRIDE LiDAR odometry lifecycle node (composable).
//
// Threads (docs/DESIGN.md budget "worker pool of 2 threads + 1 I/O thread"):
//   * I/O: the executor thread runs every subscription callback; callbacks only convert and push
//     into lock-free SPSC queues (IMU, joints, graph odometry, requests) or the depth-1 scan
//     slot (newest wins, never a backlog).
//   * worker: owns lo::Pipeline (single-threaded API); its OpenMP team (num_threads = 2: the
//     worker + 1) is the "pool of 2". Pinned to runtime.cpu_affinity, optional SCHED_FIFO.
// Lifecycle: configure validates every parameter (schema ranges, orthonormal extrinsics, files),
// looks up extrinsics, builds the pipeline and starts processing (INIT runs while inactive);
// activate is refused until INIT has completed; publishers are lifecycle publishers.
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lo_msgs/msg/lo_factor.hpp>
#include <lo_msgs/msg/node_request.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include "lo_core/pipeline.hpp"
#include "lo_core/util/spsc_queue.hpp"
#include "lo_ros/black_box.hpp"
#include "lo_ros/conversions.hpp"

namespace lo_ros {

class LoNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LoNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  struct RosParams
  {
    std::string topic_lidar, topic_imu, topic_joints, topic_graph_odom, topic_node_request,
      topic_ptp_offset;
    std::string frame_base, frame_lidar, frame_imu, frame_odom;
    bool sensor_reliable{true};
    double extrinsics_timeout{10.0};
    std::string urdf_file, robot_description_topic;
    double robot_description_timeout{5.0};
    bool require_kinematics{false};
    std::vector<int64_t> cpu_affinity;
    int sched_fifo_priority{0};
    double coverage_timeout{0.05};
    bool publish_deskewed{false};
    int deskewed_decimation{1};
    double map_period{2.0};
    int map_max_points{60000};
    double lidar_timeout{0.2};
    double imu_timeout{1.0};
    double joints_timeout{0.1};
    BlackBox::Options blackbox;
  };

  struct ImuItem {lo::ImuSample s; lo::TimeNs arrival;};
  struct PtpItem {lo::TimeNs t; double offset;};

  void declareRosParameters();
  bool readRosParameters(std::string & error);
  bool lookupExtrinsics(std::string & error);
  bool loadKinematics(std::string & error);
  void createInterfaces();
  void destroyInterfaces();

  // I/O thread
  void onScan(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void onJoints(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onGraphOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onRequest(lo_msgs::msg::NodeRequest::ConstSharedPtr msg);
  void onPtp(std_msgs::msg::Float64::ConstSharedPtr msg);
  void watchdog();
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // worker thread
  void workerLoop();
  void drainQueues();
  void processScan(const livox_ros_driver2::msg::CustomMsg & msg, lo::TimeNs arrival);
  void publishResult(const lo::ScanResult & r);
  void publishFactors(const lo::FactorList & factors);
  void applyRuntimeSettings();

  RosParams rp_;
  lo::Config cfg_;
  lo::Extrinsics ext_;
  std::shared_ptr<lo::KinematicModel> kin_;
  std::unique_ptr<lo::Pipeline> pipeline_;
  std::unique_ptr<JointMapper> joint_mapper_;
  std::unique_ptr<BlackBox> black_box_;
  std::string config_hash_;

  // queues I/O -> worker
  std::unique_ptr<lo::SpscQueue<ImuItem>> imu_q_;
  std::unique_ptr<lo::SpscQueue<lo::JointSample>> joint_q_;
  std::unique_ptr<lo::SpscQueue<lo::GraphOdomSample>> graph_q_;
  std::unique_ptr<lo::SpscQueue<lo::TimeNs>> request_q_;
  std::unique_ptr<lo::SpscQueue<PtpItem>> ptp_q_;
  std::mutex scan_mtx_;
  std::condition_variable scan_cv_;
  livox_ros_driver2::msg::CustomMsg::ConstSharedPtr pending_scan_;
  lo::TimeNs pending_arrival_{0};
  std::atomic<std::uint64_t> scans_overwritten_{0};

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::vector<lo::RawPoint> raw_buf_;
  std::vector<lo::Vec3> map_buf_;
  sensor_msgs::msg::PointCloud2 deskewed_msg_, map_msg_;
  lo::TimeNs last_map_pub_{0};
  std::uint64_t scan_counter_{0};

  // watchdog state (ROS time of the last message of each stream; written by I/O thread)
  std::atomic<int64_t> last_scan_ns_{0}, last_imu_ns_{0}, last_joint_ns_{0};
  std::atomic<bool> lidar_stopped_{false}, imu_lost_{false}, joints_lost_{false};

  // diagnostics snapshot (worker -> diagnostics)
  std::mutex diag_mtx_;
  lo::Diagnostics diag_snapshot_;
  std::uint64_t imu_q_drops_{0}, joint_q_drops_{0};

  // ROS interfaces
  rclcpp_lifecycle::LifecyclePublisher<lo_msgs::msg::LoFactor>::SharedPtr factor_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr graph_sub_;
  rclcpp::Subscription<lo_msgs::msg::NodeRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ptp_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diag_;
};

}  // namespace lo_ros
