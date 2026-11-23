#include "lo_synth/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lo::synth {

namespace {

constexpr double kG = 9.80665;

/// Exact ray / capsule intersection (cylinder body + end spheres); s in (lo, hi).
bool rayCapsule(const Vec3 & o, const Vec3 & d, const Vec3 & a, const Vec3 & b, double r, double lo,
  double hi, double & s)
{
  double best = hi;
  bool found = false;
  const Vec3 ab = b - a;
  const double L2 = ab.squaredNorm();
  if (L2 > 1e-12) {
    const Vec3 w = ab / std::sqrt(L2);
    const Vec3 ao = o - a;
    const Vec3 dp = d - d.dot(w) * w;
    const Vec3 op = ao - ao.dot(w) * w;
    const double A = dp.squaredNorm();
    const double B = 2.0 * dp.dot(op);
    const double C = op.squaredNorm() - r * r;
    const double disc = B * B - 4.0 * A * C;
    if (A > 1e-12 && disc >= 0.0) {
      const double t = (-B - std::sqrt(disc)) / (2.0 * A);
      if (t > lo && t < best) {
        const double proj = (ao + t * d).dot(w);
        if (proj >= 0.0 && proj * proj <= L2) {best = t; found = true;}
      }
    }
  }
  for (const Vec3 & c : {a, b}) {
    const Vec3 oc = o - c;
    const double B = 2.0 * oc.dot(d);
    const double C = oc.squaredNorm() - r * r;
    const double disc = B * B - 4.0 * C;
    if (disc < 0.0) {continue;}
    const double t = (-B - std::sqrt(disc)) / 2.0;
    if (t > lo && t < best) {best = t; found = true;}
  }
  if (found) {s = best;}
  return found;
}

Scenario base(const std::string & name, std::uint64_t seed)
{
  Scenario sc;
  sc.name = name;
  sc.seed = seed;
  // Mid-360 front-top, dome up, pitched 20 deg down (DESIGN "Open decisions": pitch down).
  sc.T_base_lidar = SE3(SO3::rotY(20.0 * kDegToRad), Vec3(0.30, 0.0, 0.12));
  sc.T_lidar_imu = SE3(SO3(), Vec3(0.011, 0.02329, -0.04412));   // Livox internal IMU
  return sc;
}

void furnishedRoom(Scene & s, std::mt19937_64 & rng, double sx, double sy, double sz, int boxes,
  int pillars, double route_hx, double route_hy)
{
  s.addRoom(Vec3(0, 0, 0), Vec3(sx, sy, sz));
  // Furniture only where the route loop (half sizes route_hx, route_hy) never goes: a central
  // island inside the loop and a band along the walls outside it, both 1.2 m clear of it.
  const double m = 1.2;
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::uniform_real_distribution<double> us(0.3, 0.9);
  std::uniform_real_distribution<double> uyaw(-M_PI, M_PI);
  auto spot = [&](Vec3 & c, double r) {
      for (int tries = 0; tries < 200; ++tries) {
        c = Vec3((u01(rng) - 0.5) * (sx - 2 * r - 0.4), (u01(rng) - 0.5) * (sy - 2 * r - 0.4), 0.0);
        const bool inside = std::abs(c.x()) < route_hx - m - r && std::abs(c.y()) < route_hy - m - r;
        const bool outside = std::abs(c.x()) > route_hx + m + r || std::abs(c.y()) > route_hy + m + r;
        if (inside || outside) {return true;}
      }
      return false;
    };
  for (int i = 0; i < boxes; ++i) {
    const double w = us(rng), l = us(rng), h = 0.4 + us(rng) * 1.6;
    Vec3 c;
    if (!spot(c, 0.5 * std::hypot(w, l))) {continue;}
    c.z() = h / 2;
    s.addBox(c, Vec3(w, l, h), uyaw(rng), 30.f + 40.f * static_cast<float>(i % 3));
  }
  for (int i = 0; i < pillars; ++i) {
    Cylinder cyl;
    cyl.radius = 0.12 + 0.08 * (i % 3);
    Vec3 c;
    if (!spot(c, cyl.radius)) {continue;}
    cyl.x = c.x();
    cyl.y = c.y();
    cyl.z_min = 0.0;
    cyl.z_max = sz;
    s.cylinders.push_back(cyl);
  }
}

/// Rectangle loop of half size (hx, hy) around the origin, `loops` times, then stand still.
Trajectory loopRoute(double hx, double hy, int loops, double speed, double yaw_rate, double still0)
{
  Trajectory t(Vec3(-hx, -hy, 0.5), 0.0);
  t.still(still0);
  for (int l = 0; l < loops; ++l) {
    t.moveTo(Vec3(hx, -hy, 0.5), speed).turnTo(M_PI / 2, yaw_rate);
    t.moveTo(Vec3(hx, hy, 0.5), speed).turnTo(M_PI, yaw_rate);
    t.moveTo(Vec3(-hx, hy, 0.5), speed).turnTo(-M_PI / 2, yaw_rate);
    t.moveTo(Vec3(-hx, -hy, 0.5), speed).turnTo(0.0, yaw_rate);
  }
  t.still(1.5);
  return t;
}

}  // namespace

std::vector<std::string> scenarioNames()
{
  return {"room", "warehouse", "corridor", "open_field", "stairs", "crowd", "turn180",
    "imu_dropout", "clock_jump", "ptp_loss", "point_dropout", "lens_blocked", "leg_in_fov"};
}

std::vector<std::string> faultSuiteNames()
{
  return {"imu_dropout", "clock_jump", "ptp_loss", "point_dropout", "lens_blocked", "leg_in_fov",
    "crowd", "corridor", "open_field", "stairs", "turn180"};
}

Scenario makeScenario(const std::string & name, std::uint64_t seed)
{
  Scenario sc = base(name, seed);
  std::mt19937_64 rng(seed * 7919 + 17);

  auto room = [&]() {
      furnishedRoom(sc.scene, rng, 14.0, 10.0, 3.2, 16, 6, 3.5, 2.5);
      sc.trajectory = loopRoute(3.5, 2.5, 2, 0.8, 1.2, 3.0);
    };

  if (name == "room" || name == "imu_dropout" || name == "clock_jump" || name == "ptp_loss" ||
    name == "point_dropout" || name == "lens_blocked" || name == "leg_in_fov")
  {
    room();
    if (name == "imu_dropout") {
      sc.faults.push_back({FaultType::IMU_DROPOUT, 9.0, 1.5, 0.0});    // IMU lost (> 1 s)
      sc.faults.push_back({FaultType::IMU_DROPOUT, 15.0, 0.05, 0.0});  // short gap
    } else if (name == "clock_jump") {
      sc.faults.push_back({FaultType::CLOCK_JUMP, 10.0, 3.0, 0.35});
    } else if (name == "ptp_loss") {
      sc.faults.push_back({FaultType::PTP_LOSS, 8.0, 6.0, 0.002});
    } else if (name == "point_dropout") {
      sc.faults.push_back({FaultType::POINT_DROPOUT, 0.0, 1e9, 0.5});
    } else if (name == "lens_blocked") {
      sc.faults.push_back({FaultType::LENS_BLOCKED, 10.0, 3.0, 0.0});
    } else if (name == "leg_in_fov") {
      sc.leg_override = 0;                        // LF raised in front of the sensor
      sc.faults.push_back({FaultType::JOINTS_LOST, 12.0, 2.0, 0.0});
    }
  } else if (name == "warehouse") {
    furnishedRoom(sc.scene, rng, 30.0, 20.0, 6.0, 30, 10, 7.0, 4.0);
    for (int i = -2; i <= 2; ++i) {sc.scene.addBox(Vec3(4.0 * i, 7.5, 1.25), Vec3(1.0, 3.0, 2.5));}
    sc.trajectory = loopRoute(7.0, 4.0, 1, 1.0, 1.2, 3.0);
  } else if (name == "crowd") {
    furnishedRoom(sc.scene, rng, 20.0, 14.0, 3.5, 12, 4, 4.0, 2.5);
    std::uniform_real_distribution<double> ux(-8.0, 8.0), uy(-5.5, 5.5), us(0.8, 1.6);
    for (int i = 0; i < 20; ++i) {
      Walker w;
      const Vec3 a(ux(rng), uy(rng), 0), b(ux(rng), uy(rng), 0);
      w.path = {a, b};
      w.speed = us(rng);
      w.phase = 3.0 * i;
      sc.scene.walkers.push_back(w);
    }
    sc.trajectory = loopRoute(4.0, 2.5, 2, 0.8, 1.2, 3.0);
  } else if (name == "corridor") {
    // 200 m long, featureless: ends are beyond the 70 m range -> x unobservable
    Scene & s = sc.scene;
    s.rects.push_back(Rect{Vec3(0, 1.25, 1.5), Vec3::UnitX(), Vec3::UnitZ(), 100.0, 1.5, 45.f});
    s.rects.push_back(Rect{Vec3(0, -1.25, 1.5), Vec3::UnitX(), Vec3::UnitZ(), 100.0, 1.5, 45.f});
    s.rects.push_back(Rect{Vec3(0, 0, 0), Vec3::UnitX(), Vec3::UnitY(), 100.0, 1.25, 35.f});
    s.rects.push_back(Rect{Vec3(0, 0, 3.0), Vec3::UnitX(), Vec3::UnitY(), 100.0, 1.25, 45.f});
    Trajectory t(Vec3(-10.0, 0.0, 0.5), 0.0);
    t.still(3.0).moveTo(Vec3(12.0, 0.0, 0.5), 1.0).turnBy(M_PI, 1.5).moveTo(Vec3(-10.0, 0.0, 0.5), 1.0).still(1.0);
    sc.trajectory = t;
    sc.expect_degenerate = true;
  } else if (name == "open_field") {
    sc.scene.addFloor(0.0, 300.0, 30.f);
    Trajectory t(Vec3(0.0, 0.0, 0.5), 0.0);
    t.still(3.0).moveTo(Vec3(8.0, 0.0, 0.5), 1.0).turnTo(M_PI / 2, 1.2).moveTo(Vec3(8.0, 6.0, 0.5), 1.0)
    .turnTo(M_PI, 1.2).moveTo(Vec3(0.0, 6.0, 0.5), 1.0).still(1.0);
    sc.trajectory = t;
    sc.expect_degenerate = true;
  } else if (name == "stairs") {
    Scene & s = sc.scene;
    s.addRoom(Vec3(4.0, 0.0, 0.0), Vec3(20.0, 8.0, 6.0));
    const int steps = 12;
    const double rise = 0.17, run = 0.30, x0 = 3.0;
    for (int i = 0; i < steps; ++i) {
      const double h = rise * (i + 1);
      s.addBox(Vec3(x0 + run * i + run / 2, 0.0, h / 2), Vec3(run, 1.6, h), 0.0, 40.f);
    }
    const double top = rise * steps;
    s.addBox(Vec3(x0 + run * steps + 2.0, 0.0, top / 2), Vec3(4.0, 3.0, top), 0.0, 40.f);
    for (int i = 0; i < 6; ++i) {s.addBox(Vec3(-3.0 + 2.0 * i, 3.0, 0.6), Vec3(0.8, 0.8, 1.2), 0.3 * i);}
    Trajectory t(Vec3(-2.0, 0.0, 0.5), 0.0);
    t.still(3.0).moveTo(Vec3(x0 - 0.2, 0.0, 0.5), 0.6)
    .moveTo(Vec3(x0 + run * steps + 0.4, 0.0, 0.5 + top), 0.4)
    .moveTo(Vec3(x0 + run * steps + 2.5, 0.0, 0.5 + top), 0.5).turnBy(M_PI, 1.0)
    .moveTo(Vec3(x0 + run * steps + 0.4, 0.0, 0.5 + top), 0.5)
    .moveTo(Vec3(x0 - 0.2, 0.0, 0.5), 0.4).still(1.0);
    sc.trajectory = t;
  } else if (name == "turn180") {
    furnishedRoom(sc.scene, rng, 14.0, 10.0, 3.2, 14, 6, 3.5, 1.0);
    Trajectory t(Vec3(-3.0, 0.0, 0.5), 0.0);
    t.still(3.0).moveTo(Vec3(2.0, 0.0, 0.5), 1.0).turnBy(M_PI, 3.0).moveTo(Vec3(-3.0, 0.0, 0.5), 1.0)
    .turnBy(-M_PI, 3.0).moveTo(Vec3(1.0, 0.0, 0.5), 1.0).still(1.0);
    sc.trajectory = t;
  } else {
    throw std::invalid_argument("unknown synthetic scenario '" + name + "'");
  }
  sc.duration = sc.trajectory.duration();
  return sc;
}

// ----------------------------------------------------------------------------------- Simulator

Simulator::Simulator(Scenario sc)
: sc_(std::move(sc)), robot_(sc_.legs), rng_(sc_.seed)
{
  if (sc_.leg_override >= 0) {
    robot_.setLegOverride(sc_.leg_override, sc_.leg_override_q[0], sc_.leg_override_q[1],
      sc_.leg_override_q[2]);
  }
  scan_buf_.resize(static_cast<std::size_t>(sc_.pattern.points_per_scan));
  generateImu();
  generateJoints();
  generateRequests();
}

bool Simulator::faultActive(FaultType type, double t, const Fault ** f) const
{
  for (const auto & fl : sc_.faults) {
    if (fl.type == type && t >= fl.start && t < fl.start + fl.duration) {
      if (f) {*f = &fl;}
      return true;
    }
  }
  return false;
}

double Simulator::lidarClockOffset(double t) const
{
  const Fault * f = nullptr;
  if (faultActive(FaultType::CLOCK_JUMP, t, &f)) {return f->magnitude;}
  if (faultActive(FaultType::PTP_LOSS, t, &f)) {return f->magnitude * (t - f->start);}
  return 0.0;
}

TimeNs Simulator::trueTimeOfLidarStamp(TimeNs stamp) const
{
  const double ts = toSec(stamp);
  double t = ts;
  for (int i = 0; i < 3; ++i) {t = ts - lidarClockOffset(t);}
  return fromSec(t);
}

SE3 Simulator::groundTruth(TimeNs t) const {return sc_.trajectory.pose(toSec(t));}

void Simulator::generateImu()
{
  std::mt19937_64 rng(sc_.seed * 31 + 5);
  std::normal_distribution<double> n01(0.0, 1.0);
  const ImuSpec & s = sc_.imu;
  const double dt = 1.0 / s.rate;
  const SE3 T_base_imu = sc_.T_base_lidar * sc_.T_lidar_imu;
  const SO3 & R_bi = T_base_imu.so3();
  const Vec3 & r_bi = T_base_imu.translation();
  Vec3 bg = s.gyro_bias;
  const double sg = s.gyro_noise / std::sqrt(dt);
  const double sa = s.accel_noise / std::sqrt(dt);
  const std::size_t n = static_cast<std::size_t>(sc_.duration * s.rate);
  imu_.reserve(n);
  for (std::size_t j = 1; j < n; ++j) {
    const double t = static_cast<double>(j) * dt;
    bg += s.gyro_bias_rw * std::sqrt(dt) * Vec3(n01(rng), n01(rng), n01(rng));
    const Vec3 noise_g(n01(rng), n01(rng), n01(rng));
    const Vec3 noise_a(n01(rng), n01(rng), n01(rng));
    if (faultActive(FaultType::IMU_DROPOUT, t)) {continue;}
    const SE3 T_wb = sc_.trajectory.pose(t);
    const Vec3 w_body = sc_.trajectory.omegaBody(t);
    const Vec3 a_world = sc_.trajectory.pointAccelWorld(t, r_bi);
    const SO3 R_wi = T_wb.so3() * R_bi;
    ImuSample m;
    m.t = fromSec(t);
    m.gyro = R_bi.inverse() * w_body + bg + sg * noise_g;
    Vec3 f = R_wi.inverse() * (a_world + Vec3(0.0, 0.0, kG)) + s.accel_bias + sa * noise_a;
    if (s.accel_in_g) {f /= kG;}
    m.acc = f;
    imu_.push_back(m);
    imu_arrival_.push_back(m.t + fromSec(s.latency));
  }
}

void Simulator::generateJoints()
{
  if (!sc_.publish_joints || sc_.joint_rate <= 0.0) {return;}
  const double dt = 1.0 / sc_.joint_rate;
  const std::size_t n = static_cast<std::size_t>(sc_.duration * sc_.joint_rate);
  joints_.reserve(n);
  for (std::size_t j = 1; j < n; ++j) {
    const double t = static_cast<double>(j) * dt;
    if (faultActive(FaultType::JOINTS_LOST, t)) {continue;}
    JointSample s;
    s.t = fromSec(t);
    s.n = robot_.numJoints();
    s.has_velocity = true;
    robot_.jointState(t, sc_.trajectory.gaitActivity(t), s.q.data(), s.qd.data());
    joints_.push_back(s);
  }
}

void Simulator::generateRequests()
{
  if (sc_.request_rate <= 0.0) {return;}
  const double dt_imu = 1.0 / sc_.imu.rate;
  const double period = 1.0 / sc_.request_rate;
  for (double t = sc_.request_phase; t < sc_.duration; t += period) {
    // like factor_graph_estimator: keyframes sit on IMU stamps
    const double snapped = std::round(t / dt_imu) * dt_imu;
    requests_.push_back(fromSec(snapped));
  }
}

void Simulator::generateScan(std::size_t k, std::vector<RawPoint> & out, TimeNs & stamp,
  TimeNs & true_t0)
{
  const Mid360Pattern & pat = sc_.pattern;
  const double t0 = 0.05 + static_cast<double>(k) * static_cast<double>(pat.points_per_scan) / pat.rate;
  true_t0 = fromSec(t0);
  stamp = fromSec(t0 + lidarClockOffset(t0));
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  // leg capsules at sub-times across the scan
  constexpr int kSub = 11;
  std::vector<std::vector<Vec3>> sa(kSub), sb(kSub);
  std::vector<double> sr;
  std::array<double, kMaxJoints> q{};
  const double scan_dur = static_cast<double>(pat.points_per_scan) / pat.rate;
  for (int s = 0; s < kSub; ++s) {
    const double ts = t0 + scan_dur * s / (kSub - 1);
    robot_.jointState(ts, sc_.trajectory.gaitActivity(ts), q.data(), nullptr);
    robot_.capsulesWorld(sc_.trajectory.pose(ts), q.data(), sa[s], sb[s], sr);
  }

  const Fault * fdrop = nullptr;
  for (int i = 0; i < pat.points_per_scan; ++i) {
    const double t = t0 + static_cast<double>(i) / pat.rate;
    RawPoint & rp = out[static_cast<std::size_t>(i)];
    rp = RawPoint{};
    rp.offset_ns = static_cast<std::uint32_t>(std::llround(static_cast<double>(i) / pat.rate * 1e9));
    rp.line = static_cast<std::uint8_t>(i % 4);
    const Vec3 dir_s = pat.direction(t, i % 4);
    const double noise = n01(rng_);
    const double u = u01(rng_);
    const double ud = u01(rng_);
    if (faultActive(FaultType::POINT_DROPOUT, t, &fdrop) && ud < fdrop->magnitude) {continue;}
    double range = 0.0;
    float refl = 0.f;
    if (faultActive(FaultType::LENS_BLOCKED, t) && std::abs(std::atan2(dir_s.y(), dir_s.x())) < M_PI / 2) {
      range = 0.15 + 0.25 * u;
      refl = 12.f;
    } else {
      const SE3 T_wl = sc_.trajectory.pose(t) * sc_.T_base_lidar;
      const Vec3 & o = T_wl.translation();
      const Vec3 d = T_wl.so3() * dir_s;
      Hit hit;
      double best = sc_.max_range;
      bool found = false;
      if (sc_.scene.raycast(o, d, sc_.min_range, best, t, hit)) {
        best = hit.range;
        refl = hit.reflectivity;
        found = true;
      }
      const int s = std::clamp(static_cast<int>(std::lround((t - t0) / scan_dur * (kSub - 1))), 0, kSub - 1);
      for (std::size_t c = 0; c < sr.size(); ++c) {
        double sc = 0.0;
        if (rayCapsule(o, d, sa[s][c], sb[s][c], sr[c], sc_.min_range, best, sc)) {
          best = sc;
          refl = 25.f;
          found = true;
        }
      }
      if (!found) {continue;}
      range = best + sc_.range_noise * noise;
    }
    if (range < sc_.min_range || range > sc_.max_range || refl < sc_.min_reflectivity) {continue;}
    const Vec3 p = dir_s * range;
    rp.x = static_cast<float>(p.x());
    rp.y = static_cast<float>(p.y());
    rp.z = static_cast<float>(p.z());
    rp.reflectivity = static_cast<std::uint8_t>(std::clamp(refl, 1.f, 255.f));
  }
}

void Simulator::run(Sink & sink, double t_max)
{
  const Mid360Pattern & pat = sc_.pattern;
  const double scan_dur = static_cast<double>(pat.points_per_scan) / pat.rate;
  const double end = (t_max > 0.0) ? std::min(t_max, sc_.duration) : sc_.duration;
  const std::size_t num_scans = static_cast<std::size_t>(std::max(0.0, (end - 0.05) / scan_dur) - 1);
  std::size_t ii = 0, ij = 0, ir = 0;
  const TimeNs joint_lat = fromSec(0.002);
  const TimeNs req_lat = fromSec(0.003);
  for (std::size_t k = 0; k < num_scans && !sink.stopped(); ++k) {
    const double t0 = 0.05 + static_cast<double>(k) * scan_dur;
    const TimeNs scan_arrival = fromSec(t0 + scan_dur + sc_.scan_latency);
    // deliver everything that arrives before this scan, in arrival order
    while (true) {
      TimeNs a_imu = ii < imu_.size() ? imu_arrival_[ii] : INT64_MAX;
      TimeNs a_j = ij < joints_.size() ? joints_[ij].t + joint_lat : INT64_MAX;
      TimeNs a_r = ir < requests_.size() ? requests_[ir] + req_lat : INT64_MAX;
      const TimeNs a = std::min({a_imu, a_j, a_r});
      if (a > scan_arrival) {break;}
      if (a == a_imu) {sink.onImu(imu_[ii], a_imu); ++ii;}
      else if (a == a_j) {sink.onJoints(joints_[ij], a_j); ++ij;}
      else {sink.onRequest(requests_[ir], a_r); ++ir;}
    }
    TimeNs stamp = 0, true_t0 = 0;
    generateScan(k, scan_buf_, stamp, true_t0);
    RawScan scan;
    scan.t0 = stamp;
    scan.points = scan_buf_.data();
    scan.size = scan_buf_.size();
    sink.onScan(scan, scan_arrival, true_t0);
  }
}

}  // namespace lo::synth
