// Black box (docs/DESIGN.md "Industrial hardening"): an in-RAM ring of the last `duration` s of
// raw scans, IMU and joint states, dumped to MCAP on LOST or ERROR (rate-limited), with the
// config hash and git SHA in the bag metadata. Rings hold message pointers (no copies, no
// allocation once full); writing happens on its own thread.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace lo_ros {

class BlackBox
{
public:
  struct Options
  {
    bool enabled{true};
    double duration{20.0};
    std::string directory;
    double min_interval{60.0};
    std::string config_hash;
    std::string git_sha;
    std::string topic_lidar{"/livox/lidar"};
    std::string topic_imu{"/livox/imu"};
    std::string topic_joints{"/joint_states"};
  };

  explicit BlackBox(const Options & opt);
  ~BlackBox();

  void addScan(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & m, int64_t recv_ns);
  void addImu(const sensor_msgs::msg::Imu::ConstSharedPtr & m, int64_t recv_ns);
  void addJoints(const sensor_msgs::msg::JointState::ConstSharedPtr & m, int64_t recv_ns);

  /// Dump the ring asynchronously; false if disabled, rate-limited or a dump is running.
  /// `now_ns`: wall time (rate limiting and file name only).
  bool trigger(const std::string & reason, int64_t now_ns);
  std::uint64_t dumps() const {return dumps_.load();}
  std::string lastFile() const;

private:
  template<typename MsgT>
  struct Ring
  {
    std::vector<std::pair<int64_t, std::shared_ptr<const MsgT>>> data;
    std::size_t head{0};
    std::size_t size{0};
    void init(std::size_t cap) {data.assign(cap, {0, nullptr}); head = size = 0;}
    void push(int64_t t, const std::shared_ptr<const MsgT> & m)
    {
      if (data.empty()) {return;}
      data[(head + size) % data.size()] = {t, m};
      if (size < data.size()) {++size;} else {head = (head + 1) % data.size();}
    }
    void dropOlderThan(int64_t t_min)
    {
      while (size > 0 && data[head].first < t_min) {
        data[head].second.reset();
        head = (head + 1) % data.size();
        --size;
      }
    }
    void snapshot(std::vector<std::pair<int64_t, std::shared_ptr<const MsgT>>> & out) const
    {
      out.clear();
      for (std::size_t i = 0; i < size; ++i) {out.push_back(data[(head + i) % data.size()]);}
    }
  };

  void write(const std::string & path, const std::string & reason);

  Options opt_;
  mutable std::mutex mtx_;
  Ring<livox_ros_driver2::msg::CustomMsg> scans_;
  Ring<sensor_msgs::msg::Imu> imu_;
  Ring<sensor_msgs::msg::JointState> joints_;
  std::vector<std::pair<int64_t, std::shared_ptr<const livox_ros_driver2::msg::CustomMsg>>> snap_scans_;
  std::vector<std::pair<int64_t, std::shared_ptr<const sensor_msgs::msg::Imu>>> snap_imu_;
  std::vector<std::pair<int64_t, std::shared_ptr<const sensor_msgs::msg::JointState>>> snap_joints_;
  std::thread writer_;
  std::atomic<bool> writing_{false};
  std::atomic<std::uint64_t> dumps_{0};
  int64_t last_dump_ns_{0};
  std::string last_file_;
};

}  // namespace lo_ros
