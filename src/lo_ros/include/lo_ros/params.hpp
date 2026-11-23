// lo_core schema <-> ROS parameters, and ROS params YAML files for the offline tools.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "lo_core/config/config.hpp"

namespace lo_ros {

using ParamsInterface = rclcpp::node_interfaces::NodeParametersInterface;

/// Declare every lo_core parameter with its range and description (defaults from `defaults`).
/// Out-of-range overrides throw rclcpp::exceptions::InvalidParameterValueException.
void declareCoreParameters(const std::shared_ptr<ParamsInterface> & params, const lo::Config & defaults);

/// Read the declared lo_core parameters into a Config.
lo::Config readCoreParameters(const std::shared_ptr<ParamsInterface> & params);

/// Flattened scalar/list values of a ROS params YAML file for `node_name` (or "/**").
/// Lists are stored as comma-separated strings.
bool loadParamsYaml(const std::string & path, const std::string & node_name,
  std::map<std::string, std::string> & out, std::string & error);

/// Apply flattened values to a Config through the schema. Unknown names are returned (they
/// may be ROS-layer parameters such as topics.* / frames.*).
std::vector<std::string> applyToConfig(const std::map<std::string, std::string> & values,
  lo::Config & cfg, std::vector<std::string> & errors);

}  // namespace lo_ros
