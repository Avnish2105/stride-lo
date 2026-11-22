// Black box: keeps the last `duration` s, dumps them to MCAP with metadata, rate-limited.
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <thread>
#include <unistd.h>

#include <rosbag2_cpp/reader.hpp>

#include "lo_ros/black_box.hpp"

TEST(BlackBox, DumpsTheLastWindowWithMetadataAndIsRateLimited)
{
  const auto dir = std::filesystem::temp_directory_path() /
    ("stride_bb_test_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  lo_ros::BlackBox::Options opt;
  opt.duration = 2.0;
  opt.directory = dir.string();
  opt.min_interval = 60.0;
  opt.config_hash = "cafe";
  opt.git_sha = "abc123";
  lo_ros::BlackBox bb(opt);
  const int64_t s = 1000000000LL;
  for (int k = 0; k < 1000; ++k) {   // 5 s of IMU at 200 Hz, scans at 10 Hz
    const int64_t t = 100 * s + k * 5000000LL;
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    bb.addImu(imu, t);
    if (k % 20 == 0) {
      auto scan = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
      scan->point_num = static_cast<uint32_t>(k);
      bb.addScan(scan, t);
    }
  }
  ASSERT_TRUE(bb.trigger("lost", 1700000000LL * s));
  EXPECT_FALSE(bb.trigger("lost", 1700000001LL * s));   // rate limited (and still writing)
  for (int i = 0; i < 200 && bb.dumps() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(bb.dumps(), 1u);
  EXPECT_FALSE(bb.trigger("error", 1700000030LL * s));   // inside min_interval

  rosbag2_cpp::Reader reader;
  reader.open(bb.lastFile());
  const auto meta = reader.get_metadata();
  EXPECT_EQ(meta.custom_data.at("stride_reason"), "lost");
  EXPECT_EQ(meta.custom_data.at("stride_config_hash"), "cafe");
  EXPECT_EQ(meta.custom_data.at("stride_git_sha"), "abc123");
  int scans = 0, imus = 0;
  int64_t t_min = INT64_MAX;
  while (reader.has_next()) {
    const auto m = reader.read_next();
    scans += m->topic_name == opt.topic_lidar;
    imus += m->topic_name == opt.topic_imu;
    t_min = std::min<int64_t>(t_min, m->recv_timestamp);
  }
  // the window is 2 s before the last scan (t = 104.9 s): scans at 102.9 .. 104.9
  EXPECT_EQ(scans, 21);
  EXPECT_GE(imus, 390);
  EXPECT_LE(imus, 420);
  EXPECT_GE(t_min, 102 * s + 900000000LL);
  reader.close();
  std::filesystem::remove_all(dir);
}
