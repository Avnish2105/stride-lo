// lo_sim_rig: the STRIDE sensor rig, live. Plays an lo_synth scenario on the ROS clock and
// publishes what the real robot's drivers would (DECISIONS D5, I33):
//   /livox/lidar (CustomMsg, Mid-360-like pattern, exact per-point times), /livox/imu (accel in
//   g), /joint_states, /robot_description (URDF of the rig legs), /tf_static base->livox_frame,
//   /ground_truth/odom (50 Hz) and tf world->base_gt.
// Clock: this node always owns /clock (scenario time x rate, from a monotonic wall timer at
// 1 kHz); STRIDE, the graph demo and Gazebo's KinematicRig (bridged, display only) follow it
// (DECISIONS I37).
// Release: a producer thread runs the simulator (ray casting a crowd scan takes 30-45 ms) up to
// `lookahead` s ahead and queues every message ready-built; a release thread publishes each at
// its simulated arrival time. Before, scans were ray cast when due, so they left 30-45 ms late
// (load dependent) while IMU samples caught up at once: a fake LiDAR-IMU clock skew for STRIDE.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
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
    rate_ = declare_parameter("rate", 1.0);
    loop_ = declare_parameter("loop", false);
    publish_requests_ = declare_parameter("publish_requests", false);
    start_delay_ = declare_parameter("start_delay", 3.0);
    lookahead_ = lo::fromSec(declare_parameter("lookahead", 0.5));

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
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    clock_timer_ = create_wall_timer(1ms, [this] {publishClock();});
    publishStatic();
    joint_names_ = sim_->robot().model().jointNames();
    RCLCPP_INFO(get_logger(), "scenario '%s' (seed %lu, %.1f s), rate %.2f",
      scenario_.c_str(), static_cast<unsigned long>(seed_), sim_->scenario().duration, rate_);
    wall0_ = std::chrono::steady_clock::now();
    worker_ = std::thread([this] {play();});
    releaser_ = std::thread([this] {release();});
  }

  ~RigNode() override
  {
    stop_ = true;
    queue_cv_.notify_all();
    if (worker_.joinable()) {worker_.join();}
    if (releaser_.joinable()) {releaser_.join();}
  }

private:
  struct Sink : lo::synth::Sink
  {
    explicit Sink(RigNode & n) : n_(n) {}
    void onImu(const lo::ImuSample & s, lo::TimeNs a) override {n_.enqueue(a, n_.buildImu(s));}
    void onJoints(const lo::JointSample & s, lo::TimeNs a) override {n_.enqueue(a, n_.buildJoints(s));}
    void onRequest(lo::TimeNs t, lo::TimeNs a) override {n_.enqueue(a, n_.buildRequest(t));}
    void onScan(const lo::RawScan & scan, lo::TimeNs a, lo::TimeNs) override
    {
      n_.enqueue(a, n_.buildScan(scan));
    }
    bool stopped() const override {return !n_.running();}
    RigNode & n_;
  };

  /// Scenario time (plus the repetition offset) now: monotonic wall time x rate.
  lo::TimeNs simNow() const
  {
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0_).count();
    return lo::fromSec((s - start_delay_) * rate_);
  }

  using Publish = std::function<void()>;
  struct Item
  {
    lo::TimeNs arrival;   ///< scenario time incl. the repetition offset
    Publish publish;
  };

  bool running() const {return !stop_ && rclcpp::ok();}

  /// Producer: queue a ready-built message for its arrival time; stay at most `lookahead_` ahead.
  void enqueue(lo::TimeNs a, Publish publish)
  {
    if (!publish) {return;}
    const lo::TimeNs arrival = a + offset_;
    {
      std::lock_guard<std::mutex> lk(queue_mtx_);
      queue_.push_back(Item{arrival, std::move(publish)});
    }
    queue_cv_.notify_one();
    while (running() && arrival - simNow() > lookahead_) {std::this_thread::sleep_for(1ms);}
  }

  void play()
  {
    Sink sink(*this);
    do {
      sim_->run(sink);
      offset_ += lo::fromSec(std::ceil(sim_->scenario().duration * 10.0) / 10.0);
    } while (loop_ && running());
    produced_ = true;
    queue_cv_.notify_all();
  }

  /// Release thread: publish every queued message at its arrival time, in order.
  void release()
  {
    while (running()) {
      Item item;
      {
        std::unique_lock<std::mutex> lk(queue_mtx_);
        queue_cv_.wait_for(lk, 10ms, [this] {return !queue_.empty() || produced_ || !running();});
        if (queue_.empty()) {
          if (produced_) {break;}
          continue;
        }
        item = std::move(queue_.front());
        queue_.pop_front();
      }
      while (running() && simNow() < item.arrival) {std::this_thread::sleep_for(100us);}
      if (!running()) {break;}
      item.publish();
    }
    if (produced_) {RCLCPP_INFO(get_logger(), "scenario finished");}
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

  Publish buildImu(const lo::ImuSample & s)
  {
    auto mp = std::make_shared<sensor_msgs::msg::Imu>();
    auto & m = *mp;
    m.header.stamp = stamp(s.t);
    m.header.frame_id = "livox_frame";
    m.angular_velocity.x = s.gyro.x();
    m.angular_velocity.y = s.gyro.y();
    m.angular_velocity.z = s.gyro.z();
    m.linear_acceleration.x = s.acc.x();
    m.linear_acceleration.y = s.acc.y();
    m.linear_acceleration.z = s.acc.z();
    if (s.t - last_gt_ < 20000000LL) {return [this, mp] {imu_pub_->publish(*mp);};}
    {   // ground truth at 50 Hz
      last_gt_ = s.t;
      auto gp = std::make_shared<nav_msgs::msg::Odometry>();
      auto tp = std::make_shared<tf2_msgs::msg::TFMessage>();
      auto & gt = *gp;
      gt.header.stamp = m.header.stamp;
      gt.header.frame_id = "world";
      gt.child_frame_id = "base_gt";
      gt.pose.pose = lo_ros::toPose(sim_->groundTruth(s.t));
      auto & tf = *tp;
      geometry_msgs::msg::TransformStamped t;
      t.header = gt.header;
      t.child_frame_id = "base_gt";
      t.transform.translation.x = gt.pose.pose.position.x;
      t.transform.translation.y = gt.pose.pose.position.y;
      t.transform.translation.z = gt.pose.pose.position.z;
      t.transform.rotation = gt.pose.pose.orientation;
      tf.transforms.push_back(t);
      return [this, mp, gp, tp] {
          imu_pub_->publish(*mp);
          gt_pub_->publish(*gp);
          tf_pub_->publish(*tp);
        };
    }
  }

  Publish buildJoints(const lo::JointSample & s)
  {
    auto mp = std::make_shared<sensor_msgs::msg::JointState>();
    auto & m = *mp;
    m.header.stamp = stamp(s.t);
    m.name = joint_names_;
    m.position.assign(s.q.begin(), s.q.begin() + s.n);
    m.velocity.assign(s.qd.begin(), s.qd.begin() + s.n);
    return [this, mp] {joint_pub_->publish(*mp);};
  }

  Publish buildRequest(lo::TimeNs t)
  {
    if (!request_pub_) {return {};}
    auto mp = std::make_shared<lo_msgs::msg::NodeRequest>();
    mp->stamp = stamp(t);
    return [this, mp] {request_pub_->publish(*mp);};
  }

  Publish buildScan(const lo::RawScan & scan)
  {
    lo::RawScan s = scan;
    s.t0 += offset_ + lo::fromSec(start_delay_);
    auto mp = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    lo_ros::toCustomMsg(s, "livox_frame", *mp);   // copies the points: the sim reuses its buffer
    return [this, mp] {lidar_pub_->publish(*mp);};
  }

  std::string scenario_;
  std::uint64_t seed_{1};
  bool loop_{false}, publish_requests_{false};
  double rate_{1.0}, start_delay_{3.0};
  std::unique_ptr<lo::synth::Simulator> sim_;
  std::vector<std::string> joint_names_;
  std::chrono::steady_clock::time_point wall0_;
  std::atomic<bool> stop_{false};
  std::atomic<lo::TimeNs> offset_{0};
  lo::TimeNs lookahead_{500000000LL};
  lo::TimeNs last_gt_{-1000000000LL};
  std::atomic<bool> produced_{false};
  std::deque<Item> queue_;
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::thread worker_, releaser_;
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
