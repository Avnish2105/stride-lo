#include "lo_sim/rig_description.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

namespace lo_sim {

namespace {

void xyzRpy(std::ostringstream & o, const lo::SE3 & T)
{
  const Eigen::Matrix3d R = T.so3().matrix();
  // ZYX (yaw-pitch-roll) as URDF/SDF expect
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  o << T.translation().x() << ' ' << T.translation().y() << ' ' << T.translation().z() << ' '
    << roll << ' ' << pitch << ' ' << yaw;
}

std::string origin(const lo::SE3 & T)
{
  std::ostringstream o;
  const Eigen::Matrix3d R = T.so3().matrix();
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  o << "<origin xyz=\"" << T.translation().x() << ' ' << T.translation().y() << ' '
    << T.translation().z() << "\" rpy=\"" << roll << ' ' << pitch << ' ' << yaw << "\"/>";
  return o.str();
}

}  // namespace

std::string rigUrdf(const lo::synth::Scenario & sc, const lo::synth::SyntheticRobot & robot)
{
  std::ostringstream u;
  u << "<?xml version=\"1.0\"?>\n<robot name=\"stride_rig\">\n  <link name=\"base\"/>\n";
  for (const lo::KinJointSpec & j : robot.joints()) {
    u << "  <link name=\"" << j.child_link << "\"/>\n";
    const char * type = j.type == lo::JointType::REVOLUTE ? "revolute" :
      (j.type == lo::JointType::PRISMATIC ? "prismatic" : "fixed");
    u << "  <joint name=\"" << j.name << "\" type=\"" << type << "\">\n    <parent link=\""
      << j.parent_link << "\"/>\n    <child link=\"" << j.child_link << "\"/>\n    "
      << origin(j.origin) << "\n";
    if (j.type != lo::JointType::FIXED) {
      u << "    <axis xyz=\"" << j.axis.x() << ' ' << j.axis.y() << ' ' << j.axis.z() << "\"/>\n"
        << "    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"80\" velocity=\"20\"/>\n";
    }
    u << "  </joint>\n";
  }
  u << "  <link name=\"livox_frame\"/>\n  <joint name=\"livox_mount\" type=\"fixed\">\n"
    << "    <parent link=\"base\"/>\n    <child link=\"livox_frame\"/>\n    "
    << origin(sc.T_base_lidar) << "\n  </joint>\n</robot>\n";
  return u.str();
}

std::string rigVizUrdf(const lo::synth::Scenario & sc, const lo::synth::SyntheticRobot & robot)
{
  // Same links and joints as rigUrdf, plus visuals: each joint offset is drawn as a cylinder on
  // its parent link (the leg segments), the body as a box, feet as spheres, the Mid-360 as a puck.
  std::map<std::string, std::string> visuals;
  const auto segment = [](const lo::Vec3 & v, const char * material) {
      std::ostringstream o;
      const double len = v.norm();
      if (len < 1e-3) {return o.str();}
      const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(lo::Vec3::UnitZ(), v);
      o << "    <visual>" << origin(lo::SE3(lo::SO3(q), 0.5 * v)) << "<geometry><cylinder radius=\"0.035\" "
        << "length=\"" << len << "\"/></geometry><material name=\"" << material << "\"/></visual>\n";
      return o.str();
    };
  visuals["base"] = "    <visual><geometry><box size=\"0.8 0.4 0.25\"/></geometry>"
    "<material name=\"body\"/></visual>\n";
  for (const lo::KinJointSpec & j : robot.joints()) {
    if (j.parent_link != "base") {visuals[j.parent_link] += segment(j.origin.translation(), "leg");}
    if (j.type == lo::JointType::FIXED) {
      visuals[j.child_link] += "    <visual><geometry><sphere radius=\"0.04\"/></geometry>"
        "<material name=\"foot\"/></visual>\n";
    }
  }
  visuals["livox_frame"] = "    <visual><geometry><cylinder radius=\"0.05\" length=\"0.065\"/>"
    "</geometry><material name=\"lidar\"/></visual>\n";
  std::string xml = rigUrdf(sc, robot);
  const std::string materials =
    "  <material name=\"body\"><color rgba=\"0.95 0.6 0.1 1\"/></material>\n"
    "  <material name=\"leg\"><color rgba=\"0.25 0.25 0.28 1\"/></material>\n"
    "  <material name=\"foot\"><color rgba=\"0.1 0.1 0.1 1\"/></material>\n"
    "  <material name=\"lidar\"><color rgba=\"0.1 0.7 0.9 1\"/></material>\n";
  xml.insert(xml.find('\n', xml.find("<robot")) + 1, materials);
  for (const auto & [link, v] : visuals) {
    const std::string tag = "<link name=\"" + link + "\"/>";
    const std::size_t at = xml.find(tag);
    if (at == std::string::npos) {continue;}
    xml.replace(at, tag.size(), "<link name=\"" + link + "\">\n" + v + "  </link>");
  }
  return xml;
}

std::string worldSdf(const std::string & scenario, std::uint64_t seed, const lo::synth::Scenario & sc)
{
  std::ostringstream w;
  w << "<?xml version=\"1.0\"?>\n<sdf version=\"1.9\">\n<world name=\"stride_" << scenario << "\">\n"
    << "  <physics name=\"1ms\" type=\"ignored\"><max_step_size>0.001</max_step_size>"
    << "<real_time_factor>1.0</real_time_factor></physics>\n"
    << "  <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n"
    << "  <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>\n"
    << "  <plugin filename=\"lo_sim_kinematic_rig\" name=\"lo_sim::KinematicRig\">\n"
    << "    <scenario>" << scenario << "</scenario>\n    <seed>" << seed << "</seed>\n  </plugin>\n"
    << "  <light type=\"directional\" name=\"sun\"><cast_shadows>false</cast_shadows>"
    << "<pose>0 0 20 0 0 0</pose><direction>-0.3 0.2 -1</direction></light>\n";
  int n = 0;
  auto box = [&](const lo::SE3 & pose, const lo::Vec3 & size, const char * color) {
      w << "  <model name=\"obj_" << n++ << "\"><static>true</static><pose>";
      std::ostringstream p;
      xyzRpy(p, pose);
      w << p.str() << "</pose><link name=\"l\"><visual name=\"v\"><geometry><box><size>"
        << size.x() << ' ' << size.y() << ' ' << size.z() << "</size></box></geometry>"
        << "<material><diffuse>" << color << "</diffuse></material></visual>"
        << "<collision name=\"c\"><geometry><box><size>" << size.x() << ' ' << size.y() << ' '
        << size.z() << "</size></box></geometry></collision></link></model>\n";
    };
  for (const auto & r : sc.scene.rects) {
    // thin box: axes u, v, normal = u x v
    Eigen::Matrix3d R;
    R.col(0) = r.u;
    R.col(1) = r.v;
    R.col(2) = r.u.cross(r.v);
    box(lo::SE3(lo::SO3(Eigen::Quaterniond(R).normalized()), r.c),
      lo::Vec3(2.0 * r.hu, 2.0 * r.hv, 0.02), "0.7 0.7 0.7 1");
  }
  for (const auto & b : sc.scene.boxes) {box(b.pose, 2.0 * b.half, "0.6 0.5 0.4 1");}
  for (const auto & c : sc.scene.cylinders) {
    w << "  <model name=\"obj_" << n++ << "\"><static>true</static><pose>" << c.x << ' ' << c.y
      << ' ' << 0.5 * (c.z_min + c.z_max) << " 0 0 0</pose><link name=\"l\"><visual name=\"v\">"
      << "<geometry><cylinder><radius>" << c.radius << "</radius><length>" << (c.z_max - c.z_min)
      << "</length></cylinder></geometry></visual></link></model>\n";
  }
  for (std::size_t i = 0; i < sc.scene.walkers.size(); ++i) {
    const auto & k = sc.scene.walkers[i];
    w << "  <model name=\"walker_" << i << "\"><pose>0 0 " << 0.5 * k.height << " 0 0 0</pose>"
      << "<link name=\"l\"><gravity>false</gravity><visual name=\"v\"><geometry><cylinder><radius>"
      << k.radius << "</radius><length>" << k.height << "</length></cylinder></geometry>"
      << "<material><diffuse>0.2 0.4 0.9 1</diffuse></material></visual></link></model>\n";
  }
  // the rig: body box, LiDAR puck at the mount (legs are drawn by RViz from the URDF)
  std::ostringstream mount;
  xyzRpy(mount, sc.T_base_lidar);
  w << "  <model name=\"stride_rig\"><pose>0 0 0.5 0 0 0</pose><link name=\"base\"><gravity>false"
    << "</gravity><visual name=\"body\"><geometry><box><size>0.8 0.4 0.25</size></box></geometry>"
    << "<material><diffuse>0.9 0.6 0.1 1</diffuse></material></visual><visual name=\"mid360\">"
    << "<pose>" << mount.str() << "</pose><geometry><cylinder><radius>0.035</radius><length>0.06"
    << "</length></cylinder></geometry></visual></link></model>\n</world>\n</sdf>\n";
  return w.str();
}

}  // namespace lo_sim
