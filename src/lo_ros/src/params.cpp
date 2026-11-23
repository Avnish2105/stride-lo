#include "lo_ros/params.hpp"

#include <set>
#include <sstream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_value.hpp>
#include <yaml-cpp/yaml.h>

#include "lo_core/config/schema.hpp"

namespace lo_ros {

namespace {

rcl_interfaces::msg::ParameterDescriptor describe(const char * desc)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = desc;
  return d;
}

struct Declarer
{
  std::shared_ptr<ParamsInterface> p;
  void number(const char * n, double & v, double lo, double hi, const char * desc)
  {
    auto d = describe(desc);
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = lo;
    r.to_value = hi;
    d.floating_point_range.push_back(r);
    if (!p->has_parameter(n)) {p->declare_parameter(n, rclcpp::ParameterValue(v), d, false);}
  }
  void integer(const char * n, int & v, int lo, int hi, const char * desc)
  {
    auto d = describe(desc);
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = lo;
    r.to_value = hi;
    r.step = 1;
    d.integer_range.push_back(r);
    if (!p->has_parameter(n)) {p->declare_parameter(n, rclcpp::ParameterValue(static_cast<int64_t>(v)), d, false);}
  }
  void boolean(const char * n, bool & v, const char * desc)
  {
    if (!p->has_parameter(n)) {p->declare_parameter(n, rclcpp::ParameterValue(v), describe(desc), false);}
  }
  void choice(const char * n, std::string & v, std::initializer_list<const char *> allowed, const char * desc)
  {
    std::string text = std::string(desc) + " (";
    bool first = true;
    for (const char * a : allowed) {text += (first ? "" : "|") + std::string(a); first = false;}
    text += ")";
    if (!p->has_parameter(n)) {p->declare_parameter(n, rclcpp::ParameterValue(v), describe(text.c_str()), false);}
  }
  void vector(const char * n, std::vector<double> & v, std::size_t, const char * desc)
  {
    if (!p->has_parameter(n)) {p->declare_parameter(n, rclcpp::ParameterValue(v), describe(desc), false);}
  }
};

struct Reader
{
  std::shared_ptr<ParamsInterface> p;
  void number(const char * n, double & v, double, double, const char *)
  {
    const auto pv = p->get_parameter(n).get_parameter_value();
    v = pv.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
      static_cast<double>(pv.get<int64_t>()) : pv.get<double>();
  }
  void integer(const char * n, int & v, int, int, const char *)
  {
    v = static_cast<int>(p->get_parameter(n).as_int());
  }
  void boolean(const char * n, bool & v, const char *) {v = p->get_parameter(n).as_bool();}
  void choice(const char * n, std::string & v, std::initializer_list<const char *>, const char *)
  {
    v = p->get_parameter(n).as_string();
  }
  void vector(const char * n, std::vector<double> & v, std::size_t, const char *)
  {
    v = p->get_parameter(n).as_double_array();
  }
};

void flatten(const YAML::Node & node, const std::string & prefix, std::map<std::string, std::string> & out)
{
  if (node.IsMap()) {
    for (const auto & kv : node) {
      const std::string key = kv.first.as<std::string>();
      flatten(kv.second, prefix.empty() ? key : prefix + "." + key, out);
    }
  } else if (node.IsSequence()) {
    std::string s;
    for (std::size_t i = 0; i < node.size(); ++i) {s += (i ? "," : "") + node[i].as<std::string>();}
    out[prefix] = s;
  } else if (node.IsScalar()) {
    out[prefix] = node.as<std::string>();
  }
}

struct Applier
{
  const std::map<std::string, std::string> & values;
  std::vector<std::string> & errors;
  std::set<std::string> used;
  const std::string * get(const char * n)
  {
    const auto it = values.find(n);
    if (it == values.end()) {return nullptr;}
    used.insert(n);
    return &it->second;
  }
  void number(const char * n, double & v, double, double, const char *)
  {
    if (auto s = get(n)) {try {v = std::stod(*s);} catch (...) {errors.push_back(std::string(n) + ": not a number");}}
  }
  void integer(const char * n, int & v, int, int, const char *)
  {
    if (auto s = get(n)) {try {v = std::stoi(*s);} catch (...) {errors.push_back(std::string(n) + ": not an integer");}}
  }
  void boolean(const char * n, bool & v, const char *)
  {
    if (auto s = get(n)) {v = (*s == "true" || *s == "True" || *s == "1");}
  }
  void choice(const char * n, std::string & v, std::initializer_list<const char *>, const char *)
  {
    if (auto s = get(n)) {v = *s;}
  }
  void vector(const char * n, std::vector<double> & v, std::size_t, const char *)
  {
    if (auto s = get(n)) {
      v.clear();
      std::stringstream ss(*s);
      std::string item;
      while (std::getline(ss, item, ',')) {
        try {v.push_back(std::stod(item));} catch (...) {errors.push_back(std::string(n) + ": bad list");}
      }
    }
  }
};

}  // namespace

void declareCoreParameters(const std::shared_ptr<ParamsInterface> & params, const lo::Config & defaults)
{
  lo::Config c = defaults;
  Declarer d{params};
  lo::visitParams(c, d);
}

lo::Config readCoreParameters(const std::shared_ptr<ParamsInterface> & params)
{
  lo::Config c;
  Reader r{params};
  lo::visitParams(c, r);
  return c;
}

bool loadParamsYaml(const std::string & path, const std::string & node_name,
  std::map<std::string, std::string> & out, std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    for (const std::string & key : {std::string("/**"), node_name, "/" + node_name}) {
      if (root[key] && root[key]["ros__parameters"]) {flatten(root[key]["ros__parameters"], "", out);}
    }
    if (out.empty() && root.IsMap() && !root["ros__parameters"]) {flatten(root, "", out);}   // bare map
    return true;
  } catch (const std::exception & e) {
    error = path + ": " + e.what();
    return false;
  }
}

std::vector<std::string> applyToConfig(const std::map<std::string, std::string> & values,
  lo::Config & cfg, std::vector<std::string> & errors)
{
  Applier a{values, errors, {}};
  lo::visitParams(cfg, a);
  std::vector<std::string> unknown;
  for (const auto & kv : values) {
    if (!a.used.count(kv.first)) {unknown.push_back(kv.first);}
  }
  return unknown;
}

}  // namespace lo_ros
