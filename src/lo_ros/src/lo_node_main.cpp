// Standalone process for lo_ros::LoNode (one executor thread = the I/O thread). Launched with
// respawn=True so a crash restarts it; lo_supervisor drives its lifecycle.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lo_ros/lo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lo_ros::LoNode>(options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
