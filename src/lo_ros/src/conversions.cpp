#include "lo_ros/conversions.hpp"

#include <cstring>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lo_ros {

lo::SE3 toSE3(const geometry_msgs::msg::Pose & p)
{
  return lo::fromXyzQuat(p.position.x, p.position.y, p.position.z, p.orientation.x,
    p.orientation.y, p.orientation.z, p.orientation.w);
}

lo::SE3 toSE3(const geometry_msgs::msg::Transform & t)
{
  return lo::fromXyzQuat(t.translation.x, t.translation.y, t.translation.z, t.rotation.x,
    t.rotation.y, t.rotation.z, t.rotation.w);
}

geometry_msgs::msg::Pose toPose(const lo::SE3 & T)
{
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond & q = T.unit_quaternion();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

lo::RawScan toRawScan(const livox_ros_driver2::msg::CustomMsg & msg, std::vector<lo::RawPoint> & buf)
{
  const std::size_t n = std::min(msg.points.size(), buf.capacity());
  buf.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = msg.points[i];
    lo::RawPoint & r = buf[i];
    r.x = p.x;
    r.y = p.y;
    r.z = p.z;
    r.offset_ns = p.offset_time;
    r.reflectivity = p.reflectivity;
    r.tag = p.tag;
    r.line = p.line;
  }
  lo::RawScan s;
  s.t0 = static_cast<lo::TimeNs>(msg.timebase);
  if (s.t0 == 0) {s.t0 = toNs(msg.header.stamp);}   // drivers that leave timebase empty
  s.points = buf.data();
  s.size = n;
  return s;
}

lo::ImuSample toImuSample(const sensor_msgs::msg::Imu & msg)
{
  lo::ImuSample s;
  s.t = toNs(msg.header.stamp);
  s.gyro = lo::Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  s.acc = lo::Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  return s;
}

void JointMapper::rebuild(const std::vector<std::string> & names)
{
  cached_names_ = names;
  index_.assign(model_.size(), -1);
  missing_.clear();
  for (std::size_t j = 0; j < model_.size(); ++j) {
    for (std::size_t k = 0; k < names.size(); ++k) {
      if (names[k] == model_[j]) {index_[j] = static_cast<int>(k);}
    }
    if (index_[j] < 0) {missing_.push_back(model_[j]);}
  }
}

bool JointMapper::convert(const sensor_msgs::msg::JointState & msg, lo::JointSample & out)
{
  if (msg.name != cached_names_) {rebuild(msg.name);}
  out.t = toNs(msg.header.stamp);
  out.n = static_cast<int>(std::min<std::size_t>(model_.size(), lo::kMaxJoints));
  out.has_velocity = msg.velocity.size() == msg.name.size() && !msg.velocity.empty();
  for (int j = 0; j < out.n; ++j) {
    const int k = index_[j];
    out.q[j] = (k >= 0 && static_cast<std::size_t>(k) < msg.position.size()) ? msg.position[k] : 0.0;
    out.qd[j] = (out.has_velocity && k >= 0) ? msg.velocity[k] : 0.0;
  }
  return missing_.empty();
}

lo::GraphOdomSample toGraphOdom(const nav_msgs::msg::Odometry & msg)
{
  lo::GraphOdomSample s;
  s.t = toNs(msg.header.stamp);
  s.T_odom_base = toSE3(msg.pose.pose);
  s.v_base = lo::Vec3(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  s.w_base = lo::Vec3(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
  return s;
}

lo_msgs::msg::LoFactor toMsg(const lo::FactorOut & f)
{
  lo_msgs::msg::LoFactor m;
  m.stamp_i = toStamp(f.t_i);
  m.stamp_j = toStamp(f.t_j);
  m.chain_id = f.chain_id;
  m.t_ij = toPose(f.T_ij);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {m.covariance[r * 6 + c] = f.covariance(r, c);}
    m.hessian_eigenvalues[r] = f.hessian_eigenvalues[r];
  }
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 6; ++i) {m.degenerate_directions[k * 6 + i] = f.degenerate_directions[k][i];}
  }
  m.health = f.health;
  m.inlier_ratio = f.inlier_ratio;
  m.residual_rms = f.residual_rms;
  m.flags = f.flags;
  return m;
}

lo::FactorOut fromMsg(const lo_msgs::msg::LoFactor & m)
{
  lo::FactorOut f;
  f.t_i = toNs(m.stamp_i);
  f.t_j = toNs(m.stamp_j);
  f.chain_id = m.chain_id;
  f.T_ij = toSE3(m.t_ij);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {f.covariance(r, c) = m.covariance[r * 6 + c];}
    f.hessian_eigenvalues[r] = m.hessian_eigenvalues[r];
  }
  f.num_degenerate = 0;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 6; ++i) {f.degenerate_directions[k][i] = m.degenerate_directions[k * 6 + i];}
    if (f.degenerate_directions[k].squaredNorm() > 0.5) {++f.num_degenerate;}
  }
  f.health = m.health;
  f.inlier_ratio = m.inlier_ratio;
  f.residual_rms = m.residual_rms;
  f.flags = m.flags;
  return f;
}

nav_msgs::msg::Odometry toOdometry(const lo::ScanResult & r, const std::string & odom_frame,
  const std::string & base_frame)
{
  nav_msgs::msg::Odometry o;
  o.header.stamp = toStamp(r.t_ref);
  o.header.frame_id = odom_frame;
  o.child_frame_id = base_frame;
  o.pose.pose = toPose(r.T_odom_base);
  o.twist.twist.linear.x = r.v_base.x();
  o.twist.twist.linear.y = r.v_base.y();
  o.twist.twist.linear.z = r.v_base.z();
  o.twist.twist.angular.x = r.w_base.x();
  o.twist.twist.angular.y = r.w_base.y();
  o.twist.twist.angular.z = r.w_base.z();
  return o;
}

namespace {
void xyzLayout(sensor_msgs::msg::PointCloud2 & out, std::size_t n, bool with_time)
{
  sensor_msgs::PointCloud2Modifier mod(out);
  if (with_time) {
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    mod.setPointCloud2FieldsByString(1, "xyz");
  }
  mod.resize(n);
}
}  // namespace

void toPointCloud2(const lo::Cloud & cloud, const lo::SE3 & T, std::size_t stride,
  sensor_msgs::msg::PointCloud2 & out)
{
  stride = std::max<std::size_t>(stride, 1);
  const std::size_t n = (cloud.size() + stride - 1) / stride;
  xyzLayout(out, n, true);
  sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z"), t(out, "time");
  for (std::size_t i = 0; i < cloud.size(); i += stride, ++x, ++y, ++z, ++t) {
    const lo::Vec3 p = T * cloud.points[i].head<3>();
    *x = static_cast<float>(p.x());
    *y = static_cast<float>(p.y());
    *z = static_cast<float>(p.z());
    *t = static_cast<float>(cloud.times[i]);
  }
}

void toPointCloud2(const std::vector<lo::Vec3> & pts, std::size_t n,
  sensor_msgs::msg::PointCloud2 & out)
{
  n = std::min(n, pts.size());
  xyzLayout(out, n, false);
  sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z");
  for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z) {
    *x = static_cast<float>(pts[i].x());
    *y = static_cast<float>(pts[i].y());
    *z = static_cast<float>(pts[i].z());
  }
}

void toCustomMsg(const lo::RawScan & scan, const std::string & frame_id,
  livox_ros_driver2::msg::CustomMsg & m)
{
  m.header.stamp = toStamp(scan.t0);
  m.header.frame_id = frame_id;
  m.timebase = static_cast<uint64_t>(scan.t0);
  m.point_num = static_cast<uint32_t>(scan.size);
  m.lidar_id = 0;
  m.points.resize(scan.size);
  for (std::size_t i = 0; i < scan.size; ++i) {
    const lo::RawPoint & r = scan.points[i];
    auto & p = m.points[i];
    p.x = r.x;
    p.y = r.y;
    p.z = r.z;
    p.offset_time = r.offset_ns;
    p.reflectivity = r.reflectivity;
    p.tag = r.tag;
    p.line = r.line;
  }
}

}  // namespace lo_ros
