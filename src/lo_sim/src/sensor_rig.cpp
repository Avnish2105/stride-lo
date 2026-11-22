// lo_sim_rig: the STRIDE sensor rig, live. Plays an lo_synth scenario on the ROS clock and
// publishes what the real robot's drivers would (DECISIONS D5, I33):
//   /livox/lidar (CustomMsg, Mid-360-like pattern, exact per-point times), /livox/imu (accel in
//   g), /joint_states, /robot_description (URDF of the rig legs), /tf_static base->livox_frame,
//   /ground_truth/odom (50 Hz) and tf world->base_gt.
// clock: "internal" -> this node publishes /clock (scenario time x rate); "external" -> it follows
// the ROS clock (use_sim_time with Gazebo's /clock bridged), so Gazebo's kinematic rig, the
// sensors and STRIDE share one time base. Messages are released at their simulated arrival time.
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lo_msgs/msg/node_request.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "lo_ros/conversions.hpp"
#include "lo_sim/rig_description.hpp"

namespace {

using namespace std::chrono_literals;

class RigNode : public rclcpp::Node
{
public:
  RigNode()
  : rclcpp::Node("lo_sim_rig")
  {
    scenario_ = declare_parameter("scenario", std::string("room"));
    seed_ = static_cast<std::uint64_t>(declare_parameter("seed", 1));
    internal_clock_ = declare_parameter("clock", std::string("internal")) == "internal";
    rate_ = declare_parameter("rate", 1.0);
    loop_ = declare_parameter("loop", false);
    publish_requests_ = declare_parameter("publish_requests", false);
    start_delay_ = declare_parameter("start_delay", 3.0);

    sim_ = std::make_unique<lo::synth::Simulator>(lo::synth::makeScenario(scenario_, seed_));
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", rclcpp::QoS(10).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::QoS(400).reliable());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(200).reliable());
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", rclcpp::QoS(100));
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", latched);
    urdf_pub_ = create_publisher<std_msgs::msg::String>("/robot_description", latched);
    if (publish_requests_) {
      request_pub_ = create_publisher<lo_msgs::msg::NodeRequest>("/stride_lo/node_request",
        rclcpp::QoS(100).reliable());
    }
    if (internal_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
      clock_timer_ = create_wall_timer(1ms, [this] {publishClock();});
    }
    publishStatic();
    joint_names_ = sim_->robot().model().jointNames();
    RCLCPP_INFO(get_logger(), "scenario '%s' (seed %lu, %.1f s), clock %s, rate %.2f",
      scenario_.c_str(), static_cast<unsigned long>(seed_), sim_->scenario().duration,
      internal_clock_ ? "internal" : "external", rate_);
    wall0_ = std::chrono::steady_clock::now();
    worker_ = std::thread([this] {play();});
  }

  ~RigNode() override
  {
    stop_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct Sink : lo::synth::Sink
  {
    explicit Sink(RigNode & n) : n_(n) {}
    void onImu(const lo::ImuSample & s, lo::TimeNs a) override {if (n_.waitUntil(a)) {n_.sendImu(s);}}
    void onJoints(const lo::JointSample & s, lo::TimeNs a) override {if (n_.waitUntil(a)) {n_.sendJoints(s);}}
    void onRequest(lo::TimeNs t, lo::TimeNs a) override {if (n_.waitUntil(a)) {n_.sendRequest(t);}}
    void onScan(const lo::RawScan & scan, lo::TimeNs a, lo::TimeNs) override
    {
      if (n_.waitUntil(a)) {n_.sendScan(scan);}
    }
    RigNode & n_;
  };

  /// Scenario time (plus the repetition offset) of the ROS clock now.
  lo::TimeNs simNow() const
  {
    if (internal_clock_) {
      const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0_).count();
      return lo::fromSec((s - start_delay_) * rate_);
    }
    return get_clock()->now().nanoseconds() - lo::fromSec(start_delay_);
  }

  /// Block until the scenario time `t` (+ repetition offset) is reached; false on shutdown.
  bool waitUntil(lo::TimeNs t)
  {
    const lo::TimeNs target = t + offset_;
    while (!stop_ && rclcpp::ok()) {
      if (simNow() >= target) {return true;}
      std::this_thread::sleep_for(200us);
    }
    return false;
  }

  void play()
  {
    Sink sink(*this);
    do {
      sim_->run(sink);
      offset_ += lo::fromSec(std::ceil(sim_->scenario().duration * 10.0) / 10.0);
    } while (loop_ && !stop_ && rclcpp::ok());
    RCLCPP_INFO(get_logger(), "scenario finished");
  }

  builtin_interfaces::msg::Time stamp(lo::TimeNs t) const
  {
    return lo_ros::toStamp(t + offset_ + lo::fromSec(start_delay_));
  }

  void publishClock()
  {
    rosgraph_msgs::msg::Clock c;
    c.clock = lo_ros::toStamp(std::max<lo::TimeNs>(0, simNow() + lo::fromSec(start_delay_)));
    clock_pub_->publish(c);
  }

  void publishStatic()
  {
    tf2_msgs::msg::TFMessage tf;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base";
    t.child_frame_id = "livox_frame";
    const auto p = lo_ros::toPose(sim_->scenario().T_base_lidar);
    t.transform.translation.x = p.position.x;
    t.transform.translation.y = p.position.y;
    t.transform.translation.z = p.position.z;
    t.transform.rotation = p.orientation;
    tf.transforms.push_back(t);
    tf_static_pub_->publish(tf);
    std_msgs::msg::String urdf;
    urdf.data = lo_sim::rigUrdf(sim_->scenario(), sim_->robot());
    urdf_pub_->publish(urdf);
  }

  void sendImu(const lo::ImuSample & s)
  {
    sensor_msgs::msg::Imu m;
    m.header.stamp = stamp(s.t);
    m.header.frame_id = "livox_frame";
    m.angular_velocity.x = s.gyro.x();
    m.angular_velocity.y = s.gyro.y();
    m.angular_velocity.z = s.gyro.z();
    m.linear_acceleration.x = s.acc.x();
    m.linear_acceleration.y = s.acc.y();
    m.linear_acceleration.z = s.acc.z();
    imu_pub_->publish(m);
    if (s.t - last_gt_ >= 20000000LL) {   // ground truth at 50 Hz
      last_gt_ = s.t;
      nav_msgs::msg::Odometry gt;
      gt.header.stamp = m.header.stamp;
      gt.header.frame_id = "world";
      gt.child_frame_id = "base_gt";
      gt.pose.pose = lo_ros::toPose(sim_->groundTruth(s.t));
      gt_pub_->publish(gt);
      tf2_msgs::msg::TFMessage tf;
      geometry_msgs::msg::TransformStamped t;
      t.header = gt.header;
      t.child_frame_id = "base_gt";
      t.transform.translation.x = gt.pose.pose.position.x;
      t.transform.translation.y = gt.pose.pose.position.y;
      t.transform.translation.z = gt.pose.pose.position.z;
      t.transform.rotation = gt.pose.pose.orientation;
      tf.transforms.push_back(t);
      tf_pub_->publish(tf);
    }
  }

  void sendJoints(const lo::JointSample & s)
  {
    sensor_msgs::msg::JointState m;
    m.header.stamp = stamp(s.t);
    m.name = joint_names_;
    m.position.assign(s.q.begin(), s.q.begin() + s.n);
    m.velocity.assign(s.qd.begin(), s.qd.begin() + s.n);
    joint_pub_->publish(m);
  }

  void sendRequest(lo::TimeNs t)
  {
    if (!request_pub_) {return;}
    lo_msgs::msg::NodeRequest m;
    m.stamp = stamp(t);
    request_pub_->publish(m);
  }

  void sendScan(const lo::RawScan & scan)
  {
    lo::RawScan s = scan;
    s.t0 += offset_ + lo::fromSec(start_delay_);
    livox_ros_driver2::msg::CustomMsg m;
    lo_ros::toCustomMsg(s, "livox_frame", m);
    lidar_pub_->publish(m);
  }

  std::string scenario_;
  std::uint64_t seed_{1};
  bool internal_clock_{true}, loop_{false}, publish_requests_{false};
  double rate_{1.0}, start_delay_{3.0};
  std::unique_ptr<lo::synth::Simulator> sim_;
  std::vector<std::string> joint_names_;
  std::chrono::steady_clock::time_point wall0_;
  std::atomic<bool> stop_{false};
  std::atomic<lo::TimeNs> offset_{0};
  lo::TimeNs last_gt_{-1000000000LL};
  std::thread worker_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_, tf_static_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub_;
  rclcpp::Publisher<lo_msgs::msg::NodeRequest>::SharedPtr request_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RigNode>());
  rclcpp::shutdown();
  return 0;
}
