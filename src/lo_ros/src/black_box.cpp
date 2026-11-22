#include "lo_ros/black_box.hpp"

#include <ctime>
#include <filesystem>

#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace lo_ros {

BlackBox::BlackBox(const Options & opt)
: opt_(opt)
{
  // capacities for the configured window at generous sensor rates
  scans_.init(static_cast<std::size_t>(opt.duration * 20.0) + 8);
  imu_.init(static_cast<std::size_t>(opt.duration * 2000.0) + 64);
  joints_.init(static_cast<std::size_t>(opt.duration * 1100.0) + 64);
}

BlackBox::~BlackBox()
{
  if (writer_.joinable()) {writer_.join();}
}

void BlackBox::addScan(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & m, int64_t t)
{
  if (!opt_.enabled) {return;}
  std::lock_guard<std::mutex> lk(mtx_);
  scans_.push(t, m);
  const int64_t t_min = t - static_cast<int64_t>(opt_.duration * 1e9);
  scans_.dropOlderThan(t_min);
  imu_.dropOlderThan(t_min);
  joints_.dropOlderThan(t_min);
}

void BlackBox::addImu(const sensor_msgs::msg::Imu::ConstSharedPtr & m, int64_t t)
{
  if (!opt_.enabled) {return;}
  std::lock_guard<std::mutex> lk(mtx_);
  imu_.push(t, m);
}

void BlackBox::addJoints(const sensor_msgs::msg::JointState::ConstSharedPtr & m, int64_t t)
{
  if (!opt_.enabled) {return;}
  std::lock_guard<std::mutex> lk(mtx_);
  joints_.push(t, m);
}

std::string BlackBox::lastFile() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return last_file_;
}

bool BlackBox::trigger(const std::string & reason, int64_t now_ns)
{
  if (!opt_.enabled || writing_.load()) {return false;}
  if (last_dump_ns_ != 0 && now_ns - last_dump_ns_ < static_cast<int64_t>(opt_.min_interval * 1e9)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    scans_.snapshot(snap_scans_);
    imu_.snapshot(snap_imu_);
    joints_.snapshot(snap_joints_);
  }
  last_dump_ns_ = now_ns;
  const std::time_t secs = static_cast<std::time_t>(now_ns / 1000000000LL);
  char stamp[32];
  std::tm tm{};
  gmtime_r(&secs, &tm);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
  const std::string path = opt_.directory + "/blackbox_" + stamp + "_" + reason;
  if (writer_.joinable()) {writer_.join();}
  writing_ = true;
  writer_ = std::thread([this, path, reason] {write(path, reason);});
  return true;
}

void BlackBox::write(const std::string & path, const std::string & reason)
{
  try {
    std::filesystem::create_directories(opt_.directory);
    rosbag2_storage::StorageOptions so;
    so.uri = path;
    so.storage_id = "mcap";
    so.custom_data = {{"stride_config_hash", opt_.config_hash}, {"stride_git_sha", opt_.git_sha},
      {"stride_reason", reason}};
    rosbag2_cpp::Writer writer;
    writer.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
    for (const auto & [t, m] : snap_scans_) {writer.write(*m, opt_.topic_lidar, rclcpp::Time(t));}
    for (const auto & [t, m] : snap_imu_) {writer.write(*m, opt_.topic_imu, rclcpp::Time(t));}
    for (const auto & [t, m] : snap_joints_) {writer.write(*m, opt_.topic_joints, rclcpp::Time(t));}
    writer.close();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_file_ = path;
    }
    ++dumps_;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stride_lo.black_box"), "black box dump failed: %s", e.what());
  }
  snap_scans_.clear();
  snap_imu_.clear();
  snap_joints_.clear();
  writing_ = false;
}

}  // namespace lo_ros
