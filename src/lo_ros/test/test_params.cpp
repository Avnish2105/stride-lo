#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "lo_core/config/config.hpp"
#include "lo_ros/params.hpp"

using namespace lo_ros;

TEST(Params, RobotYamlAppliesThroughTheSchema)
{
  std::map<std::string, std::string> v;
  std::string err;
  ASSERT_TRUE(loadParamsYaml(std::string(LO_ROS_CONFIG_DIR) + "/anymal_c.yaml", "stride_lo", v, err)) << err;
  lo::Config cfg;
  std::vector<std::string> errors;
  const auto unknown = applyToConfig(v, cfg, errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(cfg.imu.accel_scale, 9.80665);
  EXPECT_EQ(cfg.imu.sample_convention, "interval_mean");
  EXPECT_DOUBLE_EQ(cfg.self_filter.body_box[0], -0.55);
  EXPECT_TRUE(lo::validate(cfg).empty());
  bool has_topic = false;
  for (const auto & u : unknown) {has_topic |= (u == "topics.lidar");}
  EXPECT_TRUE(has_topic);   // ROS-layer parameter, not part of lo_core
}

// Every shipped params file (incl. the template users copy) validates, and every key it sets
// that lo_core does not know is a ROS-layer parameter (catches misspelled keys).
TEST(Params, ShippedConfigsValidateWithoutUnknownKeys)
{
  const std::vector<std::string> ros_prefixes = {"topics.", "frames.", "qos.", "runtime.", "debug.",
    "watchdog.", "blackbox.", "extrinsics.timeout", "self_filter.urdf_file",
    "self_filter.robot_description_", "self_filter.require_kinematics"};
  for (const char * name : {"default", "anymal_c", "stride_rig", "robot_template"}) {
    SCOPED_TRACE(name);
    std::map<std::string, std::string> v;
    std::string err;
    ASSERT_TRUE(loadParamsYaml(std::string(LO_ROS_CONFIG_DIR) + "/" + name + ".yaml", "stride_lo",
      v, err)) << err;
    lo::Config cfg;
    std::vector<std::string> errors;
    const auto unknown = applyToConfig(v, cfg, errors);
    EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
    EXPECT_TRUE(lo::validate(cfg).empty());
    for (const auto & u : unknown) {
      bool ros = false;
      for (const auto & p : ros_prefixes) {ros |= u.rfind(p, 0) == 0;}
      EXPECT_TRUE(ros) << "unknown parameter " << u;
    }
  }
}

TEST(Params, DeclareAndReadRoundTripWithRanges)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions opt;
    opt.parameter_overrides({{"cov.kappa", 7.5}, {"emit.mode", std::string("request")}});
    auto node = std::make_shared<rclcpp::Node>("params_test", opt);
    declareCoreParameters(node->get_node_parameters_interface(), lo::Config());
    const lo::Config c = readCoreParameters(node->get_node_parameters_interface());
    EXPECT_DOUBLE_EQ(c.cov.kappa, 7.5);
    EXPECT_EQ(c.emit.mode, "request");
    EXPECT_EQ(lo::configHash(c), lo::configHash(c));
    // out of range override must be refused at declaration
    rclcpp::NodeOptions bad;
    bad.parameter_overrides({{"integrity.inlier_min", 3.0}});
    auto node2 = std::make_shared<rclcpp::Node>("params_test_bad", bad);
    EXPECT_ANY_THROW(declareCoreParameters(node2->get_node_parameters_interface(), lo::Config()));
  }
  rclcpp::shutdown();
}
