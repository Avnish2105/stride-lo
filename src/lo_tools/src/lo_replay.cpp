// lo_replay: run lo::Pipeline on a recorded bag, deterministically (message order of the bag,
// no executors, no wall clock in decisions). Same input -> bit-identical output (hash printed).
//
//   ros2 run lo_tools lo_replay --bag <dir> --robot anymal_c --out <dir>
//       [--config extra.yaml]... [--set name=value]...
//       [--requests-from /estimator/odometry_keyframe | --requests-imu /imu/data [--request-rate 10]]
//       [--start S] [--duration S] [--arrival bag|stamp] [--verbose]
//
// Output bag (MCAP): /stride_lo/factor, /stride_lo/odometry, /stride_lo/node_request (requests
// that were fed) and copies of /tf_static, the ground truth and FAST-LIO topics for evaluate.py.
// Summary JSON: <out>/summary.json.
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lo_msgs/msg/lo_factor.hpp>
#include <lo_msgs/msg/node_request.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include "lo_core/pipeline.hpp"
#include "lo_ros/conversions.hpp"
#include "lo_ros/params.hpp"
#include "lo_ros/urdf_kinematics.hpp"

namespace {

struct Args
{
  std::string bag, out = "replay_out", robot = "anymal_c";
  std::vector<std::string> configs, sets;
  std::string requests_from, requests_imu;
  double request_rate = 10.0;
  double request_latency = -1.0;   ///< < 0: requests arrive at their bag receive time
  double start = 0.0, duration = -1.0;
  std::string arrival = "bag";
  std::string gt_topic = "/ground_truth/odom";
  std::string fastlio_topic = "/Odometry";
  bool verbose = false;
};

void usage()
{
  std::fprintf(stderr,
    "lo_replay --bag DIR [--robot anymal_c|default|stride_rig] [--out DIR] [--config FILE]...\n"
    "          [--set name=value]... [--requests-from TOPIC | --requests-imu TOPIC [--request-rate HZ]]\n"
    "          [--request-latency S] [--start S] [--duration S] [--arrival bag|stamp] [--gt TOPIC]\n"
    "          [--verbose]\n"
    "  --request-latency S: deliver --requests-from requests at stamp + S (a backend that publishes\n"
    "                       NodeRequest at node creation) instead of their bag receive time\n");
}

std::uint64_t fnv(std::uint64_t h, const void * data, std::size_t n)
{
  const auto * p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {h ^= p[i]; h *= 1099511628211ULL;}
  return h;
}

template<typename T>
T deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & m)
{
  static rclcpp::Serialization<T> ser;
  rclcpp::SerializedMessage sm(*m->serialized_data);
  T out;
  ser.deserialize_message(&sm, &out);
  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {usage(); std::exit(2);}
        return argv[++i];
      };
    if (s == "--bag") {a.bag = next();}
    else if (s == "--out") {a.out = next();}
    else if (s == "--robot") {a.robot = next();}
    else if (s == "--config") {a.configs.push_back(next());}
    else if (s == "--set") {a.sets.push_back(next());}
    else if (s == "--requests-from") {a.requests_from = next();}
    else if (s == "--requests-imu") {a.requests_imu = next();}
    else if (s == "--request-rate") {a.request_rate = std::stod(next());}
    else if (s == "--request-latency") {a.request_latency = std::stod(next());}
    else if (s == "--start") {a.start = std::stod(next());}
    else if (s == "--duration") {a.duration = std::stod(next());}
    else if (s == "--arrival") {a.arrival = next();}
    else if (s == "--gt") {a.gt_topic = next();}
    else if (s == "--verbose") {a.verbose = true;}
    else {usage(); return 2;}
  }
  if (a.bag.empty()) {usage(); return 2;}

  // ---- configuration: lo_ros default.yaml + robot yaml + extra files + --set
  std::map<std::string, std::string> values;
  std::string err;
  const std::string share = ament_index_cpp::get_package_share_directory("lo_ros");
  std::vector<std::string> files = {share + "/config/default.yaml"};
  if (a.robot != "default") {files.push_back(share + "/config/" + a.robot + ".yaml");}
  files.insert(files.end(), a.configs.begin(), a.configs.end());
  for (const auto & f : files) {
    if (!lo_ros::loadParamsYaml(f, "stride_lo", values, err)) {
      std::fprintf(stderr, "%s\n", err.c_str());
      return 2;
    }
  }
  for (const auto & kv : a.sets) {
    const auto eq = kv.find('=');
    if (eq == std::string::npos) {std::fprintf(stderr, "bad --set %s\n", kv.c_str()); return 2;}
    values[kv.substr(0, eq)] = kv.substr(eq + 1);
  }
  const bool requests = !a.requests_from.empty() || !a.requests_imu.empty();
  if (requests) {values["emit.mode"] = "request";}
  lo::Config cfg;
  std::vector<std::string> errors;
  lo_ros::applyToConfig(values, cfg, errors);
  auto get = [&](const std::string & k, const std::string & def) {
      const auto it = values.find(k);
      return it == values.end() ? def : it->second;
    };
  const std::string topic_lidar = get("topics.lidar", "/livox/lidar");
  const std::string topic_imu = get("topics.imu", "/livox/imu");
  const std::string topic_joints = get("topics.joints", "/joint_states");
  const std::string topic_graph = get("topics.graph_odom", "");
  const std::string frame_base = get("frames.base", "base");
  const std::string frame_lidar = get("frames.lidar", "livox_frame");
  const std::string frame_imu = get("frames.imu", "livox_frame");
  const std::string urdf_file = get("self_filter.urdf_file", "");

  // ---- pass 1: /tf_static -> extrinsics
  lo::Extrinsics ext = lo::extrinsicsFromConfig(cfg);
  std::vector<tf2_msgs::msg::TFMessage> tf_static;
  if (cfg.extrinsics.source == "tf") {
    rosbag2_cpp::Reader reader;
    reader.open(a.bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/tf_static"};
    reader.set_filter(filter);
    tf2::BufferCore tf;
    while (reader.has_next()) {
      const auto msg = deserialize<tf2_msgs::msg::TFMessage>(reader.read_next());
      tf_static.push_back(msg);
      for (const auto & t : msg.transforms) {tf.setTransform(t, "bag", true);}
    }
    try {
      ext.T_base_lidar = lo_ros::toSE3(tf.lookupTransform(frame_base, frame_lidar, tf2::TimePointZero).transform);
      ext.T_lidar_imu = (frame_imu == frame_lidar) ? lo::SE3() :
        lo_ros::toSE3(tf.lookupTransform(frame_lidar, frame_imu, tf2::TimePointZero).transform);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "extrinsics from the bag's /tf_static failed: %s\n", e.what());
      return 2;
    }
    auto vec7 = [](const lo::SE3 & T) {
        const Eigen::Quaterniond & q = T.unit_quaternion();
        return std::vector<double>{T.translation().x(), T.translation().y(), T.translation().z(),
          q.x(), q.y(), q.z(), q.w()};
      };
    cfg.extrinsics.base_lidar = vec7(ext.T_base_lidar);
    cfg.extrinsics.lidar_imu = vec7(ext.T_lidar_imu);
  }
  for (const auto & e : lo::validate(cfg)) {errors.push_back(e);}
  if (!errors.empty()) {
    for (const auto & e : errors) {std::fprintf(stderr, "config: %s\n", e.c_str());}
    return 2;
  }

  std::shared_ptr<lo::KinematicModel> kin;
  if (cfg.self_filter.enabled && !urdf_file.empty()) {
    std::ifstream f(urdf_file);
    std::stringstream ss;
    ss << f.rdbuf();
    kin = std::make_shared<lo::KinematicModel>();
    if (!lo_ros::kinematicsFromUrdf(ss.str(), frame_base, cfg.self_filter.link_radius,
      cfg.self_filter.foot_radius, *kin, err))
    {
      std::fprintf(stderr, "URDF: %s\n", err.c_str());
      return 2;
    }
  }
  std::unique_ptr<lo_ros::JointMapper> mapper;
  if (kin) {mapper = std::make_unique<lo_ros::JointMapper>(kin->jointNames());}

  lo::Pipeline pipeline(cfg, ext, kin);
  char hash[32];
  std::snprintf(hash, sizeof(hash), "%016" PRIx64, lo::configHash(cfg));
  std::printf("lo_replay: %s config %s, extrinsics base->lidar t=(%.3f %.3f %.3f)\n", a.bag.c_str(),
    hash, ext.T_base_lidar.translation().x(), ext.T_base_lidar.translation().y(),
    ext.T_base_lidar.translation().z());

  // ---- output bag
  std::filesystem::remove_all(a.out);
  rosbag2_cpp::Writer writer;
  {
    rosbag2_storage::StorageOptions so;
    so.uri = a.out;
    so.storage_id = "mcap";
    so.custom_data = {{"stride_config_hash", hash}, {"source_bag", a.bag}};
    writer.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  }
  for (const auto & m : tf_static) {writer.write(m, "/tf_static", rclcpp::Time(0, 0, RCL_ROS_TIME));}

  // ---- pass 1b: request stamps, when they are delivered at stamp + latency
  const bool inject_requests = !a.requests_from.empty() && a.request_latency >= 0.0;
  std::vector<builtin_interfaces::msg::Time> request_stamps;
  if (inject_requests) {
    rosbag2_cpp::Reader rq_reader;
    rq_reader.open(a.bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {a.requests_from};
    rq_reader.set_filter(filter);
    while (rq_reader.has_next()) {
      request_stamps.push_back(deserialize<nav_msgs::msg::Odometry>(rq_reader.read_next()).header.stamp);
    }
    std::sort(request_stamps.begin(), request_stamps.end(),
      [](const auto & x, const auto & y) {return lo_ros::toNs(x) < lo_ros::toNs(y);});
  }
  const lo::TimeNs request_latency = lo::fromSec(std::max(0.0, a.request_latency));
  std::size_t next_injected = 0;

  // ---- pass 2: replay
  rosbag2_cpp::Reader reader;
  reader.open(a.bag);
  {
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic_lidar, topic_imu, topic_joints, a.gt_topic, a.fastlio_topic};
    if (!topic_graph.empty()) {filter.topics.push_back(topic_graph);}
    if (!a.requests_from.empty() && !inject_requests) {filter.topics.push_back(a.requests_from);}
    if (!a.requests_imu.empty() && a.requests_imu != topic_imu) {filter.topics.push_back(a.requests_imu);}
    reader.set_filter(filter);
  }
  std::optional<livox_ros_driver2::msg::CustomMsg> pending;
  lo::TimeNs pending_arrival = 0, pending_end = 0;
  std::vector<lo::RawPoint> raw;
  raw.reserve(static_cast<std::size_t>(cfg.limits.max_raw_points));
  std::uint64_t out_hash = 1469598103934665603ULL;
  std::uint64_t factors = 0, scans = 0, requests_fed = 0;
  lo::TimeNs t_first = -1, next_request = -1;
  const lo::TimeNs req_period = lo::fromSec(1.0 / a.request_rate);
  rclcpp::Serialization<lo_msgs::msg::LoFactor> ser_factor;
  const auto wall0 = std::chrono::steady_clock::now();

  auto write_factors = [&](const lo::FactorList & list) {
      for (const lo::FactorOut & f : list) {
        const auto m = lo_ros::toMsg(f);
        writer.write(m, "/stride_lo/factor", rclcpp::Time(f.t_j, RCL_ROS_TIME));
        rclcpp::SerializedMessage sm;
        ser_factor.serialize_message(&m, &sm);
        out_hash = fnv(out_hash, sm.get_rcl_serialized_message().buffer, sm.size());
        ++factors;
      }
    };

  auto process_pending = [&]() {
      if (!pending) {return;}
      const lo::RawScan scan = lo_ros::toRawScan(*pending, raw);
      const lo::ScanResult & r = pipeline.process(scan, pending_arrival);
      ++scans;
      write_factors(r.factors);
      if (r.pose_valid) {
        const auto o = lo_ros::toOdometry(r, "stride_odom", frame_base);
        writer.write(o, "/stride_lo/odometry", rclcpp::Time(r.t_ref, RCL_ROS_TIME));
        out_hash = fnv(out_hash, r.T_odom_base.data(), 7 * sizeof(double));
      }
      if (a.verbose) {
        std::printf("t=%9.3f %-10s %-8s %-24s reg=%5d inl=%.2f res=%.4f it=%d/%d deg=%d v=%.2f "
          "gate=%.3f emin=%.4f fac=%d %.1fms flags=0x%x dev=%.3fm/%.2fdeg step=%.3fm/%.2fdeg\n",
          lo::toSec(r.t_ref), lo::toString(r.state),
          lo::toString(r.verdict), lo::toString(r.reason), r.num_registration, r.inlier_ratio,
          r.residual_rms, r.iterations_coarse, r.iterations_fine, r.num_degenerate, r.voxel_size,
          r.gate, r.eigenvalues[0], r.factors.size, r.timings.total, r.flags, r.seed_dev_trans,
          r.seed_dev_rot_deg, r.max_step_trans, r.max_step_rot_deg);
      }
      pending.reset();
    };

  auto inject_until = [&](lo::TimeNs now) {
      while (next_injected < request_stamps.size() &&
        lo_ros::toNs(request_stamps[next_injected]) + request_latency <= now)
      {
        const auto & st = request_stamps[next_injected++];
        pipeline.addNodeRequest(lo_ros::toNs(st));
        lo_msgs::msg::NodeRequest rq;
        rq.stamp = st;
        writer.write(rq, "/stride_lo/node_request", rclcpp::Time(st));
        ++requests_fed;
        write_factors(pipeline.answerRequests());
      }
    };

  while (reader.has_next()) {
    const auto bm = reader.read_next();
    const std::string & topic = bm->topic_name;
    const lo::TimeNs recv = static_cast<lo::TimeNs>(bm->recv_timestamp);
    if (inject_requests) {inject_until(recv);}
    if (topic == a.gt_topic || topic == a.fastlio_topic) {
      // copied for the evaluation (written with their own stamps as log time)
      const auto m = deserialize<nav_msgs::msg::Odometry>(bm);
      writer.write(m, topic, rclcpp::Time(m.header.stamp));
      continue;
    }
    if (topic == topic_lidar) {
      auto msg = deserialize<livox_ros_driver2::msg::CustomMsg>(bm);
      const lo::TimeNs t0 = msg.timebase ? static_cast<lo::TimeNs>(msg.timebase) : lo_ros::toNs(msg.header.stamp);
      if (t_first < 0) {t_first = t0;}
      if (lo::toSec(t0 - t_first) < a.start) {continue;}
      if (a.duration > 0 && lo::toSec(t0 - t_first) > a.start + a.duration) {break;}
      process_pending();   // a newer scan arrived: the previous one goes now (IMU gap if any)
      std::uint32_t max_off = 0;
      for (const auto & p : msg.points) {max_off = std::max(max_off, p.offset_time);}
      pending_end = t0 + static_cast<lo::TimeNs>(max_off);
      pending_arrival = (a.arrival == "stamp") ? pending_end : recv;
      pending = std::move(msg);
      continue;
    }
    if (topic == topic_imu || topic == a.requests_imu) {
      const auto m = deserialize<sensor_msgs::msg::Imu>(bm);
      const lo::TimeNs t = lo_ros::toNs(m.header.stamp);
      if (topic == topic_imu) {pipeline.addImu(lo_ros::toImuSample(m), a.arrival == "stamp" ? t : recv);}
      if (topic == a.requests_imu) {
        // like factor_graph_estimator: a keyframe every 1/rate s on an IMU stamp
        if (next_request < 0) {next_request = t;}
        if (t + 1000000 >= next_request) {
          pipeline.addNodeRequest(t);
          write_factors(pipeline.answerRequests());
          lo_msgs::msg::NodeRequest rq;
          rq.stamp = m.header.stamp;
          writer.write(rq, "/stride_lo/node_request", rclcpp::Time(t, RCL_ROS_TIME));
          ++requests_fed;
          next_request += req_period;
          if (next_request <= t) {next_request = t + req_period;}
        }
      }
    } else if (topic == topic_joints) {
      const auto m = deserialize<sensor_msgs::msg::JointState>(bm);
      lo::JointSample s;
      if (mapper) {
        mapper->convert(m, s);
      } else {
        s.t = lo_ros::toNs(m.header.stamp);
        s.n = static_cast<int>(std::min<std::size_t>(m.position.size(), lo::kMaxJoints));
        s.has_velocity = m.velocity.size() == m.position.size() && !m.velocity.empty();
        for (int j = 0; j < s.n; ++j) {s.q[j] = m.position[j]; s.qd[j] = s.has_velocity ? m.velocity[j] : 0.0;}
      }
      pipeline.addJoints(s);
    } else if (topic == topic_graph) {
      pipeline.addGraphOdom(lo_ros::toGraphOdom(deserialize<nav_msgs::msg::Odometry>(bm)));
    } else if (topic == a.requests_from) {
      const auto m = deserialize<nav_msgs::msg::Odometry>(bm);
      pipeline.addNodeRequest(lo_ros::toNs(m.header.stamp));
      write_factors(pipeline.answerRequests());   // requests arriving after their scan
      lo_msgs::msg::NodeRequest rq;
      rq.stamp = m.header.stamp;
      writer.write(rq, "/stride_lo/node_request", rclcpp::Time(m.header.stamp));
      ++requests_fed;
    }
    // the pending scan goes as soon as the IMU covers it
    if (pending && pipeline.coverage(pending_end) == lo::Pipeline::Coverage::READY) {process_pending();}
  }
  process_pending();
  if (inject_requests) {inject_until(std::numeric_limits<lo::TimeNs>::max() / 2);}
  writer.close();
  pipeline.updatePercentiles();
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  const lo::Diagnostics & d = pipeline.diagnostics();

  char outhash[32];
  std::snprintf(outhash, sizeof(outhash), "%016" PRIx64, out_hash);
  std::ofstream js(a.out + "/summary.json");
  js << "{\n"
     << "  \"bag\": \"" << a.bag << "\",\n"
     << "  \"config_hash\": \"" << hash << "\",\n"
     << "  \"output_hash\": \"" << outhash << "\",\n"
     << "  \"scans\": " << d.scans << ", \"accepted\": " << d.accepted << ", \"degraded\": " << d.degraded
     << ", \"rejected\": " << d.rejected << ", \"skipped\": " << d.skipped << ",\n"
     << "  \"factors\": " << factors << ", \"requests_fed\": " << requests_fed
     << ", \"requests_answered\": " << d.requests_answered << ", \"availability\": " << d.availability() << ",\n"
     << "  \"losts\": " << d.losts << ", \"recoveries\": " << d.recoveries << ", \"chain_breaks\": " << d.chain_breaks << ",\n"
     << "  \"timing_median_ms\": " << d.timing_median_ms << ", \"timing_p99_ms\": " << d.timing_p99_ms
     << ", \"timing_max_ms\": " << d.timing_max_ms << ",\n"
     << "  \"deskew_fallbacks\": " << d.deskew_fallbacks << ", \"blocked_scans\": " << d.blocked_scans << ",\n"
     << "  \"graph_gated\": " << d.graph_gated << ", \"requests_missed\": " << d.requests_missed << ",\n"
     << "  \"reject_reasons\": {";
  bool first = true;
  for (std::size_t r = 1; r < d.reject_reasons.size(); ++r) {
    if (!d.reject_reasons[r]) {continue;}
    js << (first ? "" : ", ") << "\"" << lo::toString(static_cast<lo::RejectReason>(r)) << "\": " << d.reject_reasons[r];
    first = false;
  }
  js << "},\n  \"wall_s\": " << wall << "\n}\n";
  js.close();
  std::printf("lo_replay: %" PRIu64 " scans, %" PRIu64 " factors (acc %" PRIu64 " deg %" PRIu64 " rej %" PRIu64
    "), losts %" PRIu64 ", chain breaks %" PRIu64 ", t_med %.1f ms p99 %.1f ms, %.1f s wall, output %s\n",
    scans, factors, d.accepted, d.degraded, d.rejected, d.losts, d.chain_breaks, d.timing_median_ms,
    d.timing_p99_ms, wall, outhash);
  if (d.rejected) {
    std::printf("  rejects:");
    for (std::size_t r = 1; r < d.reject_reasons.size(); ++r) {
      if (d.reject_reasons[r]) {std::printf(" %s=%" PRIu64, lo::toString(static_cast<lo::RejectReason>(r)), d.reject_reasons[r]);}
    }
    std::printf("\n");
  }
  return 0;
}
