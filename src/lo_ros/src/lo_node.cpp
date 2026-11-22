#include "lo_ros/lo_node.hpp"

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <fstream>
#include <sstream>

#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lo_ros/params.hpp"
#include "lo_ros/urdf_kinematics.hpp"

#ifndef STRIDE_GIT_SHA
#define STRIDE_GIT_SHA "unknown"
#endif

namespace lo_ros {

using namespace std::chrono_literals;

namespace {
int64_t steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t wallNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string expandHome(const std::string & p)
{
  if (!p.empty() && p[0] == '~') {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "") + p.substr(1);
  }
  return p;
}
}  // namespace

LoNode::LoNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("stride_lo", options)
{
  declareCoreParameters(get_node_parameters_interface(), lo::Config());
  declareRosParameters();
  RCLCPP_INFO(get_logger(), "STRIDE LiDAR odometry (git %s): unconfigured", STRIDE_GIT_SHA);
}

LoNode::~LoNode()
{
  running_ = false;
  scan_cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

// ------------------------------------------------------------------------------- parameters

void LoNode::declareRosParameters()
{
  auto d = [this](const std::string & name, auto value, const std::string & desc) {
      rcl_interfaces::msg::ParameterDescriptor pd;
      pd.description = desc;
      declare_parameter(name, value, pd);
    };
  d("topics.lidar", std::string("/livox/lidar"), "livox_ros_driver2/CustomMsg input");
  d("topics.imu", std::string("/livox/imu"), "sensor_msgs/Imu for deskew + seed (gyro)");
  d("topics.joints", std::string("/joint_states"), "sensor_msgs/JointState (self-filter, standstill)");
  d("topics.graph_odom", std::string(""), "nav_msgs/Odometry from the graph (translation seed); empty = off");
  d("topics.node_request", std::string(""), "lo_msgs/NodeRequest (emit.mode request); empty = off");
  d("topics.ptp_offset", std::string(""), "std_msgs/Float64 PTP offset [s] from a PTP monitor; empty = off");
  d("frames.base", std::string("base"), "Robot base frame (factor frame)");
  d("frames.lidar", std::string("livox_frame"), "LiDAR frame (CustomMsg points)");
  d("frames.imu", std::string("livox_frame"), "Frame of the IMU messages");
  d("frames.odom", std::string("stride_odom"), "LO's own odom frame (debug outputs only)");
  d("qos.sensor_reliable", true, "Reliable sensor subscriptions (Livox/Xsens drivers publish reliable)");
  d("extrinsics.timeout", 10.0, "Wait this long for /tf_static at configure [s]");
  d("self_filter.urdf_file", std::string(""), "URDF file for the leg self-filter (overrides the topic)");
  d("self_filter.robot_description_topic", std::string("/robot_description"), "URDF topic (transient local)");
  d("self_filter.robot_description_timeout", 5.0, "Wait for robot_description at configure [s]");
  d("self_filter.require_kinematics", false, "Fail configure if no URDF is available");
  d("runtime.cpu_affinity", std::vector<int64_t>{}, "Cores for the worker (+ its OpenMP thread); empty = any");
  d("runtime.sched_fifo_priority", 0, "SCHED_FIFO priority of the worker (0 = off; needs CAP_SYS_NICE)");
  d("runtime.coverage_timeout", 0.05, "Max wall time to wait for IMU coverage of a scan [s]");
  d("debug.publish_deskewed", false, "Publish the deskewed registration cloud");
  d("debug.deskewed_decimation", 1, "Publish every n-th deskewed cloud");
  d("debug.map_period", 2.0, "Local map publishing period [s] (0 = off)");
  d("debug.map_max_points", 60000, "Max voxel means in the published map");
  d("watchdog.lidar_timeout", 0.2, "No scan for this long -> ERROR [s]");
  d("watchdog.imu_timeout", 1.0, "No IMU for this long -> IMU lost (WARN) [s]");
  d("watchdog.joints_timeout", 0.1, "No joint states for this long -> joints lost [s]");
  d("blackbox.enabled", true, "Black box ring + MCAP dump on LOST / ERROR");
  d("blackbox.duration", 20.0, "Black box window [s]");
  d("blackbox.directory", std::string("~/.ros/stride_blackbox"), "Black box output directory");
  d("blackbox.min_interval", 60.0, "Minimum time between dumps [s]");
}

bool LoNode::readRosParameters(std::string & error)
{
  rp_.topic_lidar = get_parameter("topics.lidar").as_string();
  rp_.topic_imu = get_parameter("topics.imu").as_string();
  rp_.topic_joints = get_parameter("topics.joints").as_string();
  rp_.topic_graph_odom = get_parameter("topics.graph_odom").as_string();
  rp_.topic_node_request = get_parameter("topics.node_request").as_string();
  rp_.topic_ptp_offset = get_parameter("topics.ptp_offset").as_string();
  rp_.frame_base = get_parameter("frames.base").as_string();
  rp_.frame_lidar = get_parameter("frames.lidar").as_string();
  rp_.frame_imu = get_parameter("frames.imu").as_string();
  rp_.frame_odom = get_parameter("frames.odom").as_string();
  rp_.sensor_reliable = get_parameter("qos.sensor_reliable").as_bool();
  rp_.extrinsics_timeout = get_parameter("extrinsics.timeout").as_double();
  rp_.urdf_file = expandHome(get_parameter("self_filter.urdf_file").as_string());
  rp_.robot_description_topic = get_parameter("self_filter.robot_description_topic").as_string();
  rp_.robot_description_timeout = get_parameter("self_filter.robot_description_timeout").as_double();
  rp_.require_kinematics = get_parameter("self_filter.require_kinematics").as_bool();
  rp_.cpu_affinity = get_parameter("runtime.cpu_affinity").as_integer_array();
  rp_.sched_fifo_priority = static_cast<int>(get_parameter("runtime.sched_fifo_priority").as_int());
  rp_.coverage_timeout = get_parameter("runtime.coverage_timeout").as_double();
  rp_.publish_deskewed = get_parameter("debug.publish_deskewed").as_bool();
  rp_.deskewed_decimation = std::max<int>(1, static_cast<int>(get_parameter("debug.deskewed_decimation").as_int()));
  rp_.map_period = get_parameter("debug.map_period").as_double();
  rp_.map_max_points = static_cast<int>(get_parameter("debug.map_max_points").as_int());
  rp_.lidar_timeout = get_parameter("watchdog.lidar_timeout").as_double();
  rp_.imu_timeout = get_parameter("watchdog.imu_timeout").as_double();
  rp_.joints_timeout = get_parameter("watchdog.joints_timeout").as_double();
  rp_.blackbox.enabled = get_parameter("blackbox.enabled").as_bool();
  rp_.blackbox.duration = get_parameter("blackbox.duration").as_double();
  rp_.blackbox.directory = expandHome(get_parameter("blackbox.directory").as_string());
  rp_.blackbox.min_interval = get_parameter("blackbox.min_interval").as_double();
  rp_.blackbox.topic_lidar = rp_.topic_lidar;
  rp_.blackbox.topic_imu = rp_.topic_imu;
  rp_.blackbox.topic_joints = rp_.topic_joints;
  if (rp_.topic_lidar.empty()) {error = "topics.lidar must not be empty"; return false;}
  if (cfg_.emit.mode == "request" && rp_.topic_node_request.empty()) {
    error = "emit.mode is 'request' but topics.node_request is empty";
    return false;
  }
  if (!rp_.urdf_file.empty()) {
    std::ifstream f(rp_.urdf_file);
    if (!f.good()) {error = "self_filter.urdf_file does not exist: " + rp_.urdf_file; return false;}
  }
  return true;
}

bool LoNode::lookupExtrinsics(std::string & error)
{
  if (cfg_.extrinsics.source == "params") {
    ext_ = lo::extrinsicsFromConfig(cfg_);
    return true;
  }
  tf2_ros::Buffer buffer(get_clock());
  tf2_ros::TransformListener listener(buffer, this, true);
  const auto timeout = tf2::durationFromSec(rp_.extrinsics_timeout);
  try {
    const auto bl = buffer.lookupTransform(rp_.frame_base, rp_.frame_lidar, tf2::TimePointZero, timeout);
    ext_.T_base_lidar = toSE3(bl.transform);
    if (rp_.frame_imu == rp_.frame_lidar) {
      ext_.T_lidar_imu = lo::SE3();
    } else {
      const auto li = buffer.lookupTransform(rp_.frame_lidar, rp_.frame_imu, tf2::TimePointZero, timeout);
      ext_.T_lidar_imu = toSE3(li.transform);
    }
  } catch (const std::exception & e) {
    error = std::string("extrinsics from /tf_static failed (") + e.what() +
      "); set extrinsics.source: params to use extrinsics.base_lidar / lidar_imu";
    return false;
  }
  // write back so the config hash and logs show the values actually used
  auto vec7 = [](const lo::SE3 & T) {
      const Eigen::Quaterniond q = T.unit_quaternion();
      return std::vector<double>{T.translation().x(), T.translation().y(), T.translation().z(),
        q.x(), q.y(), q.z(), q.w()};
    };
  cfg_.extrinsics.base_lidar = vec7(ext_.T_base_lidar);
  cfg_.extrinsics.lidar_imu = vec7(ext_.T_lidar_imu);
  return true;
}

bool LoNode::loadKinematics(std::string & error)
{
  kin_.reset();
  if (!cfg_.self_filter.enabled) {return true;}
  std::string xml;
  if (!rp_.urdf_file.empty()) {
    std::ifstream f(rp_.urdf_file);
    std::stringstream ss;
    ss << f.rdbuf();
    xml = ss.str();
  } else if (!rp_.robot_description_topic.empty()) {
    // wait in a private callback group: the main executor is busy running on_configure
    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions so;
    so.callback_group = group;
    auto sub = create_subscription<std_msgs::msg::String>(rp_.robot_description_topic,
      rclcpp::QoS(1).transient_local().reliable(),
      [&xml](std_msgs::msg::String::ConstSharedPtr m) {xml = m->data;}, so);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_callback_group(group, get_node_base_interface());
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(rp_.robot_description_timeout);
    while (xml.empty() && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      exec.spin_some(50ms);
    }
  }
  if (xml.empty()) {
    if (rp_.require_kinematics) {error = "no robot_description for the leg self-filter"; return false;}
    RCLCPP_WARN(get_logger(), "no URDF: self-filter uses the static body box only");
    return true;
  }
  auto model = std::make_shared<lo::KinematicModel>();
  std::string err;
  if (!kinematicsFromUrdf(xml, rp_.frame_base, cfg_.self_filter.link_radius,
    cfg_.self_filter.foot_radius, *model, err))
  {
    if (rp_.require_kinematics) {error = err; return false;}
    RCLCPP_WARN(get_logger(), "URDF not usable (%s): self-filter uses the body box only", err.c_str());
    return true;
  }
  kin_ = model;
  RCLCPP_INFO(get_logger(), "self-filter: %d joints, %d capsules from the URDF", model->numJoints(),
    model->numCapsules());
  return true;
}

// -------------------------------------------------------------------------------- lifecycle

LoNode::CallbackReturn LoNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    cfg_ = readCoreParameters(get_node_parameters_interface());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure: parameter error: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  std::string error;
  if (!readRosParameters(error)) {
    RCLCPP_ERROR(get_logger(), "configure: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!lookupExtrinsics(error) || !loadKinematics(error)) {
    RCLCPP_ERROR(get_logger(), "configure: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  const auto problems = lo::validate(cfg_);
  if (!problems.empty()) {
    for (const auto & p : problems) {RCLCPP_ERROR(get_logger(), "configure: %s", p.c_str());}
    return CallbackReturn::FAILURE;
  }
  char hash[32];
  std::snprintf(hash, sizeof(hash), "%016" PRIx64, lo::configHash(cfg_));
  config_hash_ = hash;
  RCLCPP_INFO(get_logger(), "configuration %s (git %s):\n%s", config_hash_.c_str(), STRIDE_GIT_SHA,
    lo::configDump(cfg_).c_str());

  pipeline_ = std::make_unique<lo::Pipeline>(cfg_, ext_, kin_);
  if (kin_) {joint_mapper_ = std::make_unique<JointMapper>(kin_->jointNames());}
  rp_.blackbox.config_hash = config_hash_;
  rp_.blackbox.git_sha = STRIDE_GIT_SHA;
  black_box_ = std::make_unique<BlackBox>(rp_.blackbox);
  imu_q_ = std::make_unique<lo::SpscQueue<ImuItem>>(4096);
  joint_q_ = std::make_unique<lo::SpscQueue<lo::JointSample>>(4096);
  graph_q_ = std::make_unique<lo::SpscQueue<lo::GraphOdomSample>>(1024);
  request_q_ = std::make_unique<lo::SpscQueue<lo::TimeNs>>(256);
  ptp_q_ = std::make_unique<lo::SpscQueue<PtpItem>>(64);
  raw_buf_.reserve(static_cast<std::size_t>(cfg_.limits.max_raw_points));
  map_buf_.assign(static_cast<std::size_t>(std::max(rp_.map_max_points, 1)), lo::Vec3::Zero());
  createInterfaces();
  running_ = true;
  worker_ = std::thread([this] {workerLoop();});
  RCLCPP_INFO(get_logger(), "configured: processing (INIT needs %.1f s standstill); activate after INIT",
    cfg_.standstill.min_duration);
  return CallbackReturn::SUCCESS;
}

LoNode::CallbackReturn LoNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!pipeline_ || pipeline_->state() == lo::HealthState::INIT) {
    RCLCPP_WARN(get_logger(), "activate refused: INIT not complete (needs standstill)");
    return CallbackReturn::FAILURE;
  }
  factor_pub_->on_activate();
  odom_pub_->on_activate();
  deskewed_pub_->on_activate();
  map_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "active: publishing factors");
  return CallbackReturn::SUCCESS;
}

LoNode::CallbackReturn LoNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  factor_pub_->on_deactivate();
  odom_pub_->on_deactivate();
  deskewed_pub_->on_deactivate();
  map_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LoNode::CallbackReturn LoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  destroyInterfaces();
  return CallbackReturn::SUCCESS;
}

LoNode::CallbackReturn LoNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  destroyInterfaces();
  return CallbackReturn::SUCCESS;
}

LoNode::CallbackReturn LoNode::on_error(const rclcpp_lifecycle::State &)
{
  if (black_box_) {black_box_->trigger("error", wallNs());}
  destroyInterfaces();
  return CallbackReturn::SUCCESS;
}

void LoNode::createInterfaces()
{
  const auto sensor_qos = [this](std::size_t depth) {
      rclcpp::QoS q(depth);
      if (rp_.sensor_reliable) {q.reliable();} else {q.best_effort();}
      return q;
    };
  factor_pub_ = create_publisher<lo_msgs::msg::LoFactor>("~/factor", rclcpp::QoS(100).reliable());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", rclcpp::QoS(10));
  deskewed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed", rclcpp::QoS(2));
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/local_map", rclcpp::QoS(1).transient_local());
  scan_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(rp_.topic_lidar, sensor_qos(5),
    [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr m) {onScan(m);});
  if (!rp_.topic_imu.empty() && cfg_.imu.enabled) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(rp_.topic_imu, sensor_qos(500),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(m);});
  }
  if (!rp_.topic_joints.empty()) {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(rp_.topic_joints, sensor_qos(500),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {onJoints(m);});
  }
  if (!rp_.topic_graph_odom.empty()) {
    graph_sub_ = create_subscription<nav_msgs::msg::Odometry>(rp_.topic_graph_odom, rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onGraphOdom(m);});
  }
  if (!rp_.topic_node_request.empty()) {
    request_sub_ = create_subscription<lo_msgs::msg::NodeRequest>(rp_.topic_node_request,
      rclcpp::QoS(100).reliable(), [this](lo_msgs::msg::NodeRequest::ConstSharedPtr m) {onRequest(m);});
  }
  if (!rp_.topic_ptp_offset.empty()) {
    ptp_sub_ = create_subscription<std_msgs::msg::Float64>(rp_.topic_ptp_offset, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::ConstSharedPtr m) {onPtp(m);});
  }
  watchdog_timer_ = create_wall_timer(100ms, [this] {watchdog();});
  diag_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
  diag_->setHardwareID("livox_mid360");
  diag_->add("stride_lo", this, &LoNode::diagnose);
}

void LoNode::destroyInterfaces()
{
  running_ = false;
  scan_cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
  scan_sub_.reset();
  imu_sub_.reset();
  joint_sub_.reset();
  graph_sub_.reset();
  request_sub_.reset();
  ptp_sub_.reset();
  watchdog_timer_.reset();
  diag_.reset();
  factor_pub_.reset();
  odom_pub_.reset();
  deskewed_pub_.reset();
  map_pub_.reset();
  pipeline_.reset();
  black_box_.reset();
}

// -------------------------------------------------------------------------------- I/O thread

void LoNode::onScan(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  const int64_t now = this->now().nanoseconds();
  last_scan_ns_ = now;
  if (black_box_) {black_box_->addScan(msg, now);}
  {
    std::lock_guard<std::mutex> lk(scan_mtx_);
    if (pending_scan_) {++scans_overwritten_;}   // newest wins, never a backlog
    pending_scan_ = msg;
    pending_arrival_ = now;
  }
  scan_cv_.notify_one();
}

void LoNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const int64_t now = this->now().nanoseconds();
  last_imu_ns_ = now;
  if (black_box_) {black_box_->addImu(msg, now);}
  imu_q_->push(ImuItem{toImuSample(*msg), now});
}

void LoNode::onJoints(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  const int64_t now = this->now().nanoseconds();
  last_joint_ns_ = now;
  if (black_box_) {black_box_->addJoints(msg, now);}
  if (!joint_mapper_) {
    // no kinematic model: joints still serve standstill detection (velocities in message order)
    lo::JointSample s;
    s.t = toNs(msg->header.stamp);
    s.n = static_cast<int>(std::min<std::size_t>(msg->position.size(), lo::kMaxJoints));
    s.has_velocity = msg->velocity.size() == msg->position.size() && !msg->velocity.empty();
    for (int j = 0; j < s.n; ++j) {
      s.q[j] = msg->position[j];
      s.qd[j] = s.has_velocity ? msg->velocity[j] : 0.0;
    }
    joint_q_->push(s);
    return;
  }
  lo::JointSample s;
  if (!joint_mapper_->convert(*msg, s)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "joint states miss %zu model joints",
      joint_mapper_->missing().size());
  }
  joint_q_->push(s);
}

void LoNode::onGraphOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  graph_q_->push(toGraphOdom(*msg));
}

void LoNode::onRequest(lo_msgs::msg::NodeRequest::ConstSharedPtr msg)
{
  request_q_->push(toNs(msg->stamp));
}

void LoNode::onPtp(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  ptp_q_->push(PtpItem{this->now().nanoseconds(), msg->data});
}

void LoNode::watchdog()
{
  const int64_t now = this->now().nanoseconds();
  auto stale = [now](int64_t last, double timeout) {
      return last == 0 || static_cast<double>(now - last) * 1e-9 > timeout;
    };
  const bool lidar_stopped = last_scan_ns_ != 0 && stale(last_scan_ns_, rp_.lidar_timeout);
  if (lidar_stopped && !lidar_stopped_ && black_box_) {black_box_->trigger("lidar_stopped", wallNs());}
  lidar_stopped_ = lidar_stopped;
  imu_lost_ = imu_sub_ != nullptr && stale(last_imu_ns_, rp_.imu_timeout);
  joints_lost_ = joint_sub_ != nullptr && last_joint_ns_ != 0 && stale(last_joint_ns_, rp_.joints_timeout);
}

void LoNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  lo::Diagnostics d;
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    d = diag_snapshot_;
  }
  using diagnostic_msgs::msg::DiagnosticStatus;
  const bool clock_bad = d.reject_reasons[static_cast<std::size_t>(lo::RejectReason::CLOCK_UNSTABLE)] > 0 &&
    d.state == lo::HealthState::LOST;
  if (lidar_stopped_) {
    stat.summary(DiagnosticStatus::ERROR, "LiDAR stopped: no factors");
  } else if (clock_bad) {
    stat.summary(DiagnosticStatus::ERROR, "clock unstable: no factors");
  } else if (d.state == lo::HealthState::LOST || d.state == lo::HealthState::RECOVERING) {
    stat.summary(DiagnosticStatus::WARN, std::string(lo::toString(d.state)) + ": no factors");
  } else if (d.state == lo::HealthState::INIT) {
    stat.summary(DiagnosticStatus::WARN, "INIT: waiting for standstill");
  } else if (imu_lost_) {
    stat.summary(DiagnosticStatus::WARN, "IMU lost: deskew fallback, factors inflated");
  } else if (d.state == lo::HealthState::DEGRADED) {
    stat.summary(DiagnosticStatus::WARN, "DEGRADED: unobservable directions inflated");
  } else {
    stat.summary(DiagnosticStatus::OK, "TRACKING");
  }
  stat.add("state", lo::toString(d.state));
  stat.add("chain_id", d.chain_id);
  stat.add("config_hash", config_hash_);
  stat.add("git_sha", STRIDE_GIT_SHA);
  stat.add("scans", d.scans);
  stat.add("accepted", d.accepted);
  stat.add("degraded", d.degraded);
  stat.add("rejected", d.rejected);
  for (std::size_t r = 1; r < d.reject_reasons.size(); ++r) {
    if (d.reject_reasons[r]) {
      stat.add(std::string("reject.") + lo::toString(static_cast<lo::RejectReason>(r)), d.reject_reasons[r]);
    }
  }
  stat.add("factors", d.factors);
  stat.add("requests_received", d.requests_received);
  stat.add("requests_answered", d.requests_answered);
  stat.add("availability", d.availability());
  stat.add("losts", d.losts);
  stat.add("recoveries", d.recoveries);
  stat.add("chain_breaks", d.chain_breaks);
  stat.add("timing_median_ms", d.timing_median_ms);
  stat.add("timing_p99_ms", d.timing_p99_ms);
  stat.add("timing_max_ms", d.timing_max_ms);
  stat.add("timing_last.registration_ms", d.last_timings.registration);
  stat.add("timing_last.covariance_ms", d.last_timings.covariance);
  stat.add("degenerate_directions", d.last_num_degenerate);
  stat.add("min_eigenvalue", d.last_eigenvalues[0]);
  stat.add("voxel_m", d.voxel);
  stat.add("gate_m", d.gate);
  stat.add("map_fine_voxels", d.map_fine);
  stat.add("map_coarse_voxels", d.map_coarse);
  stat.add("gyro_bias_calibrated", d.bias_calibrated);
  stat.addf("gyro_bias", "%.5f %.5f %.5f", d.gyro_bias.x(), d.gyro_bias.y(), d.gyro_bias.z());
  stat.add("imu_rate_hz", d.imu_rate);
  stat.add("imu_lost", imu_lost_.load());
  stat.add("joints_lost", joints_lost_.load());
  stat.add("clock_skew_s", d.clock_skew);
  stat.add("ptp_offset_s", d.ptp_offset);
  stat.add("clock_faults", d.clock_faults);
  stat.add("deskew_fallbacks", d.deskew_fallbacks);
  stat.add("sensor_blocked_scans", d.blocked_scans);
  stat.add("dropped_scans_input", d.dropped_scans);
  stat.add("dropped_scans_overrun", scans_overwritten_.load());
  stat.add("imu_queue_drops", imu_q_ ? imu_q_->dropped() : 0);
  stat.add("black_box_dumps", black_box_ ? black_box_->dumps() : 0);
}

// ---------------------------------------------------------------------------- worker thread

void LoNode::applyRuntimeSettings()
{
  if (!rp_.cpu_affinity.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int64_t c : rp_.cpu_affinity) {CPU_SET(static_cast<int>(c), &set);}
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      RCLCPP_WARN(get_logger(), "could not pin the worker to runtime.cpu_affinity");
    }
  }
  if (rp_.sched_fifo_priority > 0) {
    sched_param sp{};
    sp.sched_priority = rp_.sched_fifo_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(get_logger(), "SCHED_FIFO not permitted (needs CAP_SYS_NICE / rtprio limits)");
    }
  }
}

void LoNode::drainQueues()
{
  ImuItem imu;
  while (imu_q_->pop(imu)) {pipeline_->addImu(imu.s, imu.arrival);}
  lo::JointSample js;
  while (joint_q_->pop(js)) {pipeline_->addJoints(js);}
  lo::GraphOdomSample g;
  while (graph_q_->pop(g)) {pipeline_->addGraphOdom(g);}
  lo::TimeNs t;
  bool requests = false;
  while (request_q_->pop(t)) {
    pipeline_->addNodeRequest(t);
    requests = true;
  }
  // requests that arrived after their scan was processed are answered right away
  if (requests) {publishFactors(pipeline_->answerRequests());}
  PtpItem p;
  while (ptp_q_->pop(p)) {pipeline_->addPtpOffset(p.t, p.offset);}
}

void LoNode::workerLoop()
{
  applyRuntimeSettings();
  std::unique_lock<std::mutex> lk(scan_mtx_);
  while (running_) {
    scan_cv_.wait_for(lk, 20ms, [this] {return pending_scan_ != nullptr || !running_;});
    if (!running_) {break;}
    if (!pending_scan_) {
      lk.unlock();
      drainQueues();
      lk.lock();
      continue;
    }
    auto msg = pending_scan_;
    const lo::TimeNs arrival = pending_arrival_;
    pending_scan_.reset();
    lk.unlock();

    // wait (briefly) until the IMU covers the scan; a newer scan or the timeout ends the wait
    drainQueues();
    uint32_t max_off = 0;
    for (const auto & p : msg->points) {max_off = std::max(max_off, p.offset_time);}
    const lo::TimeNs t0 = msg->timebase ? static_cast<lo::TimeNs>(msg->timebase) : toNs(msg->header.stamp);
    const lo::TimeNs t_end = t0 + static_cast<lo::TimeNs>(max_off);
    const int64_t deadline = steadyNs() + static_cast<int64_t>(rp_.coverage_timeout * 1e9);
    while (running_ && pipeline_->coverage(t_end) == lo::Pipeline::Coverage::WAIT &&
      steadyNs() < deadline)
    {
      std::this_thread::sleep_for(1ms);
      drainQueues();
      std::lock_guard<std::mutex> g(scan_mtx_);
      if (pending_scan_) {break;}
    }
    processScan(*msg, arrival);
    lk.lock();
  }
}

void LoNode::processScan(const livox_ros_driver2::msg::CustomMsg & msg, lo::TimeNs arrival)
{
  const lo::RawScan scan = toRawScan(msg, raw_buf_);
  const lo::ScanResult & r = pipeline_->process(scan, arrival);
  publishResult(r);
  if (++scan_counter_ % 10 == 0) {pipeline_->updatePercentiles();}
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    diag_snapshot_ = pipeline_->diagnostics();
  }
  if (pipeline_->justLost() && black_box_) {
    if (black_box_->trigger("lost", wallNs())) {
      RCLCPP_WARN(get_logger(), "LOST: black box dump started");
    }
  }
}

void LoNode::publishFactors(const lo::FactorList & factors)
{
  if (!factor_pub_ || !factor_pub_->is_activated()) {return;}
  for (const lo::FactorOut & f : factors) {factor_pub_->publish(toMsg(f));}
}

void LoNode::publishResult(const lo::ScanResult & r)
{
  publishFactors(r.factors);
  if (r.pose_valid && odom_pub_->is_activated()) {
    odom_pub_->publish(toOdometry(r, rp_.frame_odom, rp_.frame_base));
  }
  if (rp_.publish_deskewed && r.pose_valid && deskewed_pub_->is_activated() &&
    scan_counter_ % static_cast<std::uint64_t>(rp_.deskewed_decimation) == 0)
  {
    deskewed_msg_.header.stamp = toStamp(r.t_ref);
    deskewed_msg_.header.frame_id = rp_.frame_lidar;
    toPointCloud2(pipeline_->deskewedCloud(), lo::SE3(), 1, deskewed_msg_);
    deskewed_pub_->publish(deskewed_msg_);
  }
  if (rp_.map_period > 0.0 && map_pub_->is_activated() &&
    r.t_ref - last_map_pub_ >= lo::fromSec(rp_.map_period))
  {
    last_map_pub_ = r.t_ref;
    const std::size_t n = pipeline_->exportMap(map_buf_, map_buf_.size());
    const lo::SE3 & A = pipeline_->outputAlignment();
    for (std::size_t i = 0; i < n; ++i) {map_buf_[i] = A * map_buf_[i];}
    map_msg_.header.stamp = toStamp(r.t_ref);
    map_msg_.header.frame_id = rp_.frame_odom;
    toPointCloud2(map_buf_, n, map_msg_);
    map_pub_->publish(map_msg_);
  }
}

}  // namespace lo_ros

RCLCPP_COMPONENTS_REGISTER_NODE(lo_ros::LoNode)
