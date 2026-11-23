// Point containers. All capacities are fixed at construction (no allocation per scan).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <small_gicp/points/traits.hpp>

#include "lo_core/types/se3.hpp"
#include "lo_core/types/time.hpp"

namespace lo {

/// One raw LiDAR return (layout-independent copy of livox_ros_driver2/CustomPoint).
struct RawPoint
{
  float x{0.f};
  float y{0.f};
  float z{0.f};
  std::uint32_t offset_ns{0};   ///< time since the scan's timebase
  std::uint8_t reflectivity{0};
  std::uint8_t tag{0};
  std::uint8_t line{0};
};

/// A scan as handed to the pipeline: a view on caller-owned points.
struct RawScan
{
  TimeNs t0{0};                     ///< timebase = time of the first point
  const RawPoint * points{nullptr};
  std::size_t size{0};
};

/// Registration cloud (S2 output, deskewed by S3, covariances by S4). Struct of arrays sized to
/// `capacity` once; size() is the number of active points.
class Cloud
{
public:
  explicit Cloud(std::size_t capacity = 0) {reserve(capacity);}

  void reserve(std::size_t capacity)
  {
    points.assign(capacity, Vec4(0.0, 0.0, 0.0, 1.0));
    covs.assign(capacity, Mat3::Identity());
    normals.assign(capacity, Vec3::Zero());
    times.assign(capacity, 0.0);
    n_ = 0;
  }

  std::size_t size() const {return n_;}
  std::size_t capacity() const {return points.size();}
  bool full() const {return n_ >= points.size();}
  void clear() {n_ = 0;}
  void resize(std::size_t n) {n_ = n <= capacity() ? n : capacity();}

  /// Append a point (raw sensor frame) with its time; false if full.
  bool push(const Vec3 & p, double t)
  {
    if (n_ >= points.size()) {return false;}
    points[n_] = Vec4(p.x(), p.y(), p.z(), 1.0);
    times[n_] = t;
    normals[n_].setZero();
    covs[n_].setIdentity();
    ++n_;
    return true;
  }

  std::vector<Vec4> points;    ///< homogeneous, w = 1 (small_gicp convention)
  std::vector<Mat3> covs;      ///< plane-regularised covariance (S4)
  std::vector<Vec3> normals;   ///< unit normal oriented toward the sensor (S4), zero if unknown
  std::vector<double> times;   ///< absolute point time minus the scan reference time [s]

private:
  std::size_t n_{0};
};

}  // namespace lo

namespace small_gicp::traits {

template<>
struct Traits<lo::Cloud>
{
  static std::size_t size(const lo::Cloud & c) {return c.size();}
  static bool has_points(const lo::Cloud & c) {return c.size() > 0;}
  static bool has_normals(const lo::Cloud & c) {return c.size() > 0;}
  static bool has_covs(const lo::Cloud & c) {return c.size() > 0;}
  static const Eigen::Vector4d & point(const lo::Cloud & c, std::size_t i) {return c.points[i];}
  static Eigen::Vector4d normal(const lo::Cloud & c, std::size_t i)
  {
    const auto & n = c.normals[i];
    return Eigen::Vector4d(n.x(), n.y(), n.z(), 0.0);
  }
  static Eigen::Matrix4d cov(const lo::Cloud & c, std::size_t i)
  {
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m.block<3, 3>(0, 0) = c.covs[i];
    return m;
  }
};

}  // namespace small_gicp::traits
