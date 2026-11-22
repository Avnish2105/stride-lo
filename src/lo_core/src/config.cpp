#include "lo_core/config/config.hpp"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "lo_core/config/schema.hpp"
#include "lo_core/types/flags.hpp"

namespace lo {

namespace {

std::string formatDouble(double v)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

struct RangeChecker
{
  std::vector<std::string> errors;

  void number(const char * name, double & v, double lo, double hi, const char *)
  {
    if (!std::isfinite(v) || v < lo || v > hi) {
      errors.push_back(std::string(name) + "=" + formatDouble(v) + " outside [" +
        formatDouble(lo) + ", " + formatDouble(hi) + "]");
    }
  }
  void integer(const char * name, int & v, int lo, int hi, const char *)
  {
    if (v < lo || v > hi) {
      errors.push_back(std::string(name) + "=" + std::to_string(v) + " outside [" +
        std::to_string(lo) + ", " + std::to_string(hi) + "]");
    }
  }
  void boolean(const char *, bool &, const char *) {}
  void choice(const char * name, std::string & v, std::initializer_list<const char *> allowed,
    const char *)
  {
    for (const char * a : allowed) {
      if (v == a) {return;}
    }
    std::string list;
    for (const char * a : allowed) {list += std::string(list.empty() ? "" : "|") + a;}
    errors.push_back(std::string(name) + "='" + v + "' not one of " + list);
  }
  void vector(const char * name, std::vector<double> & v, std::size_t size, const char *)
  {
    if (v.size() != size) {
      errors.push_back(std::string(name) + " has " + std::to_string(v.size()) +
        " values, expected " + std::to_string(size));
      return;
    }
    for (double x : v) {
      if (!std::isfinite(x)) {
        errors.push_back(std::string(name) + " has a non-finite value");
        return;
      }
    }
  }
};

struct Hasher
{
  std::uint64_t h = 1469598103934665603ULL;   // FNV-1a offset basis
  std::string dump;

  void add(const char * name, const std::string & value)
  {
    const std::string kv = std::string(name) + "=" + value + ";";
    for (unsigned char ch : kv) {
      h ^= ch;
      h *= 1099511628211ULL;
    }
    dump += std::string(name) + "=" + value + "\n";
  }
  void number(const char * name, double & v, double, double, const char *) {add(name, formatDouble(v));}
  void integer(const char * name, int & v, int, int, const char *) {add(name, std::to_string(v));}
  void boolean(const char * name, bool & v, const char *) {add(name, v ? "true" : "false");}
  void choice(const char * name, std::string & v, std::initializer_list<const char *>,
    const char *) {add(name, v);}
  void vector(const char * name, std::vector<double> & v, std::size_t, const char *)
  {
    std::string s = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {s += (i ? "," : "") + formatDouble(v[i]);}
    add(name, s + "]");
  }
};

void checkQuaternion(const std::vector<double> & v, const char * name,
  std::vector<std::string> & errors)
{
  if (v.size() != 7) {return;}
  const double n = std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]);
  if (std::abs(n - 1.0) > 1e-3) {
    errors.push_back(std::string(name) + " quaternion norm " + formatDouble(n) +
      " is not 1 (rotation must be orthonormal)");
  }
}

void checkBox(const std::vector<double> & b, const char * name, std::vector<std::string> & errors)
{
  if (b.size() != 6) {return;}
  for (int i = 0; i < 3; ++i) {
    if (b[i] > b[i + 3]) {
      errors.push_back(std::string(name) + " min > max on axis " + std::to_string(i));
    }
  }
}

}  // namespace

Extrinsics extrinsicsFromConfig(const Config & c)
{
  Extrinsics e;
  const auto & a = c.extrinsics.base_lidar;
  const auto & b = c.extrinsics.lidar_imu;
  if (a.size() == 7) {e.T_base_lidar = fromXyzQuat(a[0], a[1], a[2], a[3], a[4], a[5], a[6]);}
  if (b.size() == 7) {e.T_lidar_imu = fromXyzQuat(b[0], b[1], b[2], b[3], b[4], b[5], b[6]);}
  return e;
}

std::vector<std::string> validate(const Config & config)
{
  Config c = config;
  RangeChecker rc;
  visitParams(c, rc);
  auto & e = rc.errors;
  if (c.range.min >= c.range.max) {e.push_back("range.min must be < range.max");}
  if (c.voxel.base_min > c.voxel.base_max) {e.push_back("voxel.base_min must be <= base_max");}
  if (c.gate.min > c.gate.max) {e.push_back("gate.min must be <= gate.max");}
  if (c.gate.initial < c.gate.min || c.gate.initial > c.gate.max) {
    e.push_back("gate.initial must lie in [gate.min, gate.max]");
  }
  if (c.gicp.coarse_search != 1 && c.gicp.coarse_search != 7 && c.gicp.coarse_search != 27) {
    e.push_back("gicp.coarse_search must be 1, 7 or 27");
  }
  if (c.gicp.fine_search != 1 && c.gicp.fine_search != 7 && c.gicp.fine_search != 27) {
    e.push_back("gicp.fine_search must be 1, 7 or 27");
  }
  if (c.limits.imu_buffer < 2) {e.push_back("limits.imu_buffer too small");}
  if (c.scan.max_point_offset < c.scan.period * 0.5) {
    e.push_back("scan.max_point_offset is shorter than half a scan period");
  }
  if (c.limits.max_registration_points > c.limits.max_raw_points) {
    e.push_back("limits.max_registration_points must be <= limits.max_raw_points");
  }
  checkQuaternion(c.extrinsics.base_lidar, "extrinsics.base_lidar", e);
  checkQuaternion(c.extrinsics.lidar_imu, "extrinsics.lidar_imu", e);
  checkBox(c.self_filter.body_box, "self_filter.body_box", e);
  checkBox(c.self_filter.leg_box, "self_filter.leg_box", e);
  return e;
}

std::uint64_t configHash(const Config & config)
{
  Config c = config;
  Hasher h;
  visitParams(c, h);
  return h.h;
}

std::string configDump(const Config & config)
{
  Config c = config;
  Hasher h;
  visitParams(c, h);
  return h.dump;
}

const char * toString(HealthState s)
{
  switch (s) {
    case HealthState::INIT: return "INIT";
    case HealthState::TRACKING: return "TRACKING";
    case HealthState::DEGRADED: return "DEGRADED";
    case HealthState::LOST: return "LOST";
    case HealthState::RECOVERING: return "RECOVERING";
  }
  return "?";
}

const char * toString(Verdict v)
{
  switch (v) {
    case Verdict::ACCEPT: return "ACCEPT";
    case Verdict::DEGRADED: return "DEGRADED";
    case Verdict::REJECT: return "REJECT";
    case Verdict::SKIPPED: return "SKIPPED";
  }
  return "?";
}

const char * toString(RejectReason r)
{
  switch (r) {
    case RejectReason::NONE: return "none";
    case RejectReason::STAMP_BACKWARDS: return "stamp_backwards";
    case RejectReason::PERIOD_JUMP: return "period_jump";
    case RejectReason::TOO_FEW_POINTS: return "too_few_points";
    case RejectReason::POINT_TIME_RANGE: return "point_time_range";
    case RejectReason::CLOCK_UNSTABLE: return "clock_unstable";
    case RejectReason::TOO_FEW_REG_POINTS: return "too_few_registration_points";
    case RejectReason::NOT_CONVERGED: return "not_converged";
    case RejectReason::LOW_INLIERS: return "low_inliers";
    case RejectReason::RESIDUAL: return "residual";
    case RejectReason::GYRO_MISMATCH: return "gyro_mismatch";
    case RejectReason::IMPLAUSIBLE_MOTION: return "implausible_motion";
    case RejectReason::SEED_DEVIATION: return "seed_deviation";
    case RejectReason::NUMERICAL: return "numerical";
    case RejectReason::SOLVER_BLOWUP: return "solver_blowup";
    case RejectReason::COUNT: break;
  }
  return "?";
}

}  // namespace lo
