// The parameter schema: one call per parameter, in a fixed order. A visitor implements
//   void number(const char* name, double& v, double lo, double hi, const char* desc);
//   void integer(const char* name, int& v, int lo, int hi, const char* desc);
//   void boolean(const char* name, bool& v, const char* desc);
//   void choice(const char* name, std::string& v, std::initializer_list<const char*> allowed,
//               const char* desc);
//   void vector(const char* name, std::vector<double>& v, size_t size, const char* desc);
// Users: lo_ros (declare/read ROS parameters with descriptors and ranges), lo_tools (YAML),
// validate(), configHash(), configDump(). Adding a parameter = one line here + the field.
#pragma once

#include <initializer_list>
#include <string>
#include <vector>

#include "lo_core/config/config.hpp"

namespace lo {

template<typename Visitor>
void visitParams(Config & c, Visitor & v)
{
  // ---- limits (memory is sized from these at construction) ----
  v.integer("limits.max_raw_points", c.limits.max_raw_points, 1000, 1000000, "Raw points per scan");
  v.integer("limits.max_registration_points", c.limits.max_registration_points, 500, 200000,
    "Registration points after downsampling");
  v.integer("limits.map_fine_capacity", c.limits.map_fine_capacity, 1000, 10000000,
    "Fine map voxel pool");
  v.integer("limits.map_coarse_capacity", c.limits.map_coarse_capacity, 100, 2000000,
    "Coarse map voxel pool");
  v.integer("limits.imu_buffer", c.limits.imu_buffer, 64, 100000, "IMU ring size");
  v.integer("limits.joint_buffer", c.limits.joint_buffer, 16, 100000, "Joint state ring size");
  v.integer("limits.graph_buffer", c.limits.graph_buffer, 16, 100000, "Graph odometry ring size");
  v.integer("limits.request_buffer", c.limits.request_buffer, 4, 10000, "Pending node requests");
  v.integer("limits.frame_records", c.limits.frame_records, 2, 16,
    "Accepted scans kept to answer node requests that arrive after their scan");
  v.integer("limits.timing_window", c.limits.timing_window, 10, 100000, "Scans for percentiles");
  v.integer("threads.num_threads", c.threads.num_threads, 1, 64, "OpenMP team size");

  // ---- S0 ----
  v.number("scan.period", c.scan.period, 0.01, 1.0, "Expected scan period [s]");
  v.number("scan.period_tolerance", c.scan.period_tolerance, 0.001, 1.0,
    "Max deviation from the expected period [s]");
  v.number("scan.drop_tolerance", c.scan.drop_tolerance, 0.0, 1.0,
    "Gap of n >= 2 periods within this counts as dropped scans, else clock jump [s]");
  v.integer("scan.min_valid_points", c.scan.min_valid_points, 1, 1000000, "Min valid points");
  v.number("scan.max_point_offset", c.scan.max_point_offset, 0.01, 1.0,
    "Max point time offset in a scan [s]");
  v.number("scan.time_offset", c.scan.time_offset, -0.5, 0.5,
    "Added to LiDAR stamps (LiDAR clock -> IMU/graph clock calibration) [s]");
  v.number("scan.time_quantum", c.scan.time_quantum, 0.0, 0.1,
    "Test data only: snap point times to renders every this many s (0 = off) [s]");
  v.number("scan.time_quantum_phase", c.scan.time_quantum_phase, -0.1, 0.1,
    "Render time phase for scan.time_quantum [s]");
  v.number("clock.max_offset", c.clock.max_offset, 1e-5, 1.0, "External PTP offset limit [s]");
  v.number("clock.max_skew", c.clock.max_skew, 1e-5, 1.0,
    "Limit of the arrival-based IMU-LiDAR skew proxy [s]");
  v.number("clock.window", c.clock.window, 0.1, 60.0, "Latency rolling-min window [s]");
  v.number("clock.stable_time", c.clock.stable_time, 0.0, 60.0,
    "Stable time after a clock fault [s]");
  v.boolean("clock.check_skew", c.clock.check_skew, "Check IMU-LiDAR skew drift");

  // ---- S2 ----
  v.number("range.min", c.range.min, 0.0, 10.0, "Range gate min [m]");
  v.number("range.max", c.range.max, 1.0, 500.0, "Range gate max [m]");
  v.boolean("range.drop_zero_reflectivity", c.range.drop_zero_reflectivity,
    "Drop zero-reflectivity returns");
  v.boolean("range.drop_noise_tag", c.range.drop_noise_tag, "Drop Livox high-confidence noise");
  v.number("voxel.base_scale", c.voxel.base_scale, 0.001, 1.0, "v = scale * median range");
  v.number("voxel.base_min", c.voxel.base_min, 0.01, 5.0, "Min base voxel [m]");
  v.number("voxel.base_max", c.voxel.base_max, 0.01, 5.0, "Max base voxel [m]");
  v.number("voxel.ema_alpha", c.voxel.ema_alpha, 0.0, 1.0, "Median range EMA alpha");
  v.number("voxel.initial", c.voxel.initial, 0.01, 5.0, "Base voxel before the first scan [m]");
  v.number("voxel.map_ratio", c.voxel.map_ratio, 0.05, 10.0, "Fine map voxel / v");
  v.number("voxel.coarse_ratio", c.voxel.coarse_ratio, 0.5, 50.0, "Coarse map voxel / v");
  v.number("voxel.rebin_threshold", c.voxel.rebin_threshold, 0.01, 10.0,
    "Re-bin the map when v drifts by this fraction");

  // ---- S3 / S4 / S5 ----
  v.integer("knn.k", c.knn.k, 3, 64, "Neighbours for point covariances");
  v.number("knn.plane_eps", c.knn.plane_eps, 1e-6, 1.0, "Regularised normal eigenvalue");
  v.boolean("imu.enabled", c.imu.enabled, "Use the IMU (deskew, seed, gyro gate)");
  v.number("imu.min_rate", c.imu.min_rate, 1.0, 10000.0, "Min IMU rate [Hz]");
  v.number("imu.accel_scale", c.imu.accel_scale, 1e-6, 1000.0,
    "Accel multiplier (9.80665 for Livox g)");
  v.choice("imu.sample_convention", c.imu.sample_convention, {"instant", "interval_mean"},
    "IMU sample time convention");
  v.number("imu.time_offset", c.imu.time_offset, -0.5, 0.5,
    "Added to IMU stamps (IMU clock -> LiDAR clock calibration) [s]");
  v.number("imu.gap_max", c.imu.gap_max, 0.001, 1.0, "IMU gap -> deskew fallback [s]");
  v.number("imu.lost_timeout", c.imu.lost_timeout, 0.01, 60.0, "No IMU -> IMU lost [s]");
  v.boolean("deskew.enabled", c.deskew.enabled, "Continuous-time deskew");
  v.number("seed.graph_timeout", c.seed.graph_timeout, 0.0, 10.0,
    "Max age of graph odometry for seeding [s]");
  v.number("seed.graph_max_dev", c.seed.graph_max_dev, 0.01, 10.0,
    "Graph LiDAR velocity vs LO velocity: larger deviation gates the graph out [m/s]");
  v.number("seed.graph_holdoff", c.seed.graph_holdoff, 0.0, 60.0,
    "Graph ignored this long after failing the gate [s]");

  // ---- S1 ----
  v.boolean("self_filter.enabled", c.self_filter.enabled, "Leg/body self-filter");
  v.vector("self_filter.body_box", c.self_filter.body_box, 6,
    "Body crop box in base frame [xmin ymin zmin xmax ymax zmax]");
  v.number("self_filter.inflate", c.self_filter.inflate, 0.0, 1.0, "Leg capsule inflation [m]");
  v.number("self_filter.link_radius", c.self_filter.link_radius, 0.0, 1.0,
    "Capsule radius of moving links [m]");
  v.number("self_filter.foot_radius", c.self_filter.foot_radius, 0.0, 1.0,
    "Sphere radius of feet [m]");
  v.number("self_filter.sphere_radius", c.self_filter.sphere_radius, 0.1, 10.0,
    "Self-filter test radius around base [m]");
  v.integer("self_filter.fk_samples", c.self_filter.fk_samples, 2, 101, "FK samples per scan");
  v.number("self_filter.joints_timeout", c.self_filter.joints_timeout, 0.001, 10.0,
    "Joint states lost after [s]");
  v.vector("self_filter.leg_box", c.self_filter.leg_box, 6,
    "Static leg exclusion box (joints lost), base frame");
  v.number("self_filter.blocked_radius", c.self_filter.blocked_radius, 0.0, 10.0,
    "Blockage monitor radius [m]");
  v.number("self_filter.blocked_fraction", c.self_filter.blocked_fraction, 0.0, 1.0,
    "Blockage monitor fraction");

  // ---- S6 ----
  v.integer("gicp.coarse_iters", c.gicp.coarse_iters, 0, 100, "Coarse GICP iterations");
  v.integer("gicp.fine_iters", c.gicp.fine_iters, 1, 200, "Fine GICP iteration cap");
  v.integer("gicp.freeze_after", c.gicp.freeze_after, 1, 100,
    "Fine stage: correspondences are frozen from this iteration on (convergence on a fixed set)");
  v.number("gicp.rot_eps", c.gicp.rot_eps, 1e-9, 1.0, "Convergence rotation [rad]");
  v.number("gicp.trans_eps", c.gicp.trans_eps, 1e-9, 1.0, "Convergence translation [m]");
  v.integer("gicp.coarse_search", c.gicp.coarse_search, 1, 27, "Coarse voxel neighbourhood");
  v.integer("gicp.fine_search", c.gicp.fine_search, 1, 27, "Fine voxel neighbourhood");
  v.number("gicp.damping", c.gicp.damping, 0.0, 1.0, "Gauss-Newton damping");
  v.number("gicp.orientation_min_cos", c.gicp.orientation_min_cos, -1.0, 1.0,
    "Far-side rejection threshold");
  v.number("gicp.normal_consistency_min", c.gicp.normal_consistency_min, 0.0, 1.0,
    "Min voxel normal consistency for the far-side check");
  v.integer("gicp.min_points", c.gicp.min_points, 6, 1000000, "Min registration points");
  v.number("gate.initial", c.gate.initial, 0.01, 100.0, "Initial adaptive gate [m]");
  v.number("gate.min", c.gate.min, 0.001, 100.0, "Min adaptive gate [m]");
  v.number("gate.max", c.gate.max, 0.01, 100.0, "Max adaptive gate [m]");
  v.number("gate.sigma_factor", c.gate.sigma_factor, 0.1, 100.0, "tau = factor * sigma_dev");
  v.number("gate.min_motion", c.gate.min_motion, 0.0, 10.0,
    "Min motion for gate statistics [m]");
  v.number("gate.voxel_floor", c.gate.voxel_floor, 0.0, 10.0,
    "Effective gate >= this * fine voxel size (Gaussian-voxel map spread)");
  v.number("kernel.scale_ratio", c.kernel.scale_ratio, 0.01, 10.0, "GM scale / tau");

  // ---- S7 ----
  v.number("integrity.inlier_min", c.integrity.inlier_min, 0.0, 1.0, "Min inlier ratio");
  v.number("integrity.residual_ratio_max", c.integrity.residual_ratio_max, 1.0, 100.0,
    "Max residual RMS / running median");
  v.integer("integrity.residual_window", c.integrity.residual_window, 1, 10000,
    "Residual median window");
  v.integer("integrity.residual_warmup", c.integrity.residual_warmup, 0, 10000,
    "Scans before the residual check");
  v.number("integrity.gyro_rot_max_deg", c.integrity.gyro_rot_max_deg, 0.01, 180.0,
    "Max LO vs gyro rotation [deg]");
  v.number("integrity.v_max", c.integrity.v_max, 0.01, 100.0, "Max speed [m/s]");
  v.number("integrity.w_max", c.integrity.w_max, 0.01, 100.0, "Max angular rate [rad/s]");
  v.number("integrity.seed_dev_trans", c.integrity.seed_dev_trans, 0.001, 100.0,
    "Max deviation from seed [m]");
  v.number("integrity.seed_dev_rot_deg", c.integrity.seed_dev_rot_deg, 0.01, 180.0,
    "Max deviation from seed [deg]");
  v.number("integrity.step_max_trans", c.integrity.step_max_trans, 0.001, 100.0,
    "Max single solver step [m]");
  v.number("integrity.step_max_rot_deg", c.integrity.step_max_rot_deg, 0.01, 180.0,
    "Max single solver step [deg]");
  v.number("integrity.imu_lost_plausibility_scale", c.integrity.imu_lost_plausibility_scale,
    0.01, 1.0, "Plausibility limits scale with IMU lost");
  v.number("degeneracy.lambda_min", c.degeneracy.lambda_min, 0.0, 1e9,
    "Degenerate if normalised eigenvalue below");
  v.integer("degeneracy.max_directions", c.degeneracy.max_directions, 0, 3,
    "Max degenerate directions handled");
  v.number("degeneracy.sigma_trans", c.degeneracy.sigma_trans, 0.001, 1000.0,
    "Sigma along degenerate translation [m]");
  v.number("degeneracy.sigma_rot", c.degeneracy.sigma_rot, 0.001, 100.0,
    "Sigma along degenerate rotation [rad]");
  v.number("cov.kappa", c.cov.kappa, 1e-3, 1e6, "Covariance inflation (NEES-tuned)");
  v.number("cov.floor_rot_deg", c.cov.floor_rot_deg, 0.0, 10.0, "Rotation floor 1 sigma [deg]");
  v.number("cov.floor_trans", c.cov.floor_trans, 0.0, 1.0, "Translation floor 1 sigma [m]");
  v.number("cov.fallback_scale", c.cov.fallback_scale, 1.0, 1000.0,
    "Whole-factor inflation (deskew fallback, IMU lost, blocked)");

  // ---- S8 / S9 ----
  v.number("map.insert_dist", c.map.insert_dist, 0.0, 100.0, "Insert after this motion [m]");
  v.number("map.insert_rot_deg", c.map.insert_rot_deg, 0.0, 180.0, "Insert after this rotation");
  v.integer("map.max_pts_voxel", c.map.max_pts_voxel, 1, 100000, "Contributions per voxel");
  v.number("map.radius", c.map.radius, 1.0, 10000.0, "Map radius crop [m]");
  v.number("map.ttl", c.map.ttl, 0.1, 3600.0, "Voxel time-to-live [s]");
  v.choice("emit.mode", c.emit.mode, {"overlap", "request"}, "Keyframe trigger");
  v.number("emit.overlap_new_kf", c.emit.overlap_new_kf, 0.0, 1.0, "Emit below this overlap");
  v.number("emit.request_tolerance", c.emit.request_tolerance, 0.0, 1.0,
    "Request may lie this far outside a scan [s]");
  v.number("emit.request_max_age", c.emit.request_max_age, 0.01, 60.0,
    "Pending requests this much older than the newest are missed [s]");
  v.integer("lost.consecutive_rejects", c.lost.consecutive_rejects, 1, 1000,
    "Rejects in a row -> LOST");
  v.number("recovery.timeout", c.recovery.timeout, 0.1, 600.0, "RECOVERING timeout [s]");
  v.number("recovery.gate", c.recovery.gate, 0.01, 100.0, "Recovery coarse gate [m]");
  v.integer("recovery.coarse_iters", c.recovery.coarse_iters, 1, 200, "Recovery coarse iters");
  v.number("degraded.clear_time", c.degraded.clear_time, 0.0, 60.0,
    "Clean time before DEGRADED -> TRACKING [s]");
  v.number("standstill.min_duration", c.standstill.min_duration, 0.1, 60.0,
    "Standstill duration [s]");
  v.number("standstill.block_duration", c.standstill.block_duration, 0.01, 10.0,
    "Block-mean length [s]");
  v.number("standstill.joint_vel_max", c.standstill.joint_vel_max, 0.0, 100.0,
    "Max block-mean |qd| [rad/s]");
  v.number("standstill.lo_motion_max", c.standstill.lo_motion_max, 0.0, 1.0,
    "Max LO translation per scan [m]");
  v.number("standstill.lo_rot_max_deg", c.standstill.lo_rot_max_deg, 0.0, 10.0,
    "Max LO rotation per scan [deg]");
  v.number("standstill.gross_factor", c.standstill.gross_factor, 1.0, 1000.0,
    "Per-scan LO motion gate = factor * window limits");
  v.number("standstill.gyro_max", c.standstill.gyro_max, 0.0, 10.0,
    "Max block-mean |gyro - bias| [rad/s]");
  v.number("standstill.bias_alpha", c.standstill.bias_alpha, 0.0, 1.0,
    "Weight of a new standstill window");
  v.boolean("standstill.use_joints", c.standstill.use_joints, "Use joint velocities");

  v.choice("extrinsics.source", c.extrinsics.source, {"tf", "params"}, "Extrinsics source");
  v.vector("extrinsics.base_lidar", c.extrinsics.base_lidar, 7,
    "T_base_lidar [x y z qx qy qz qw]");
  v.vector("extrinsics.lidar_imu", c.extrinsics.lidar_imu, 7, "T_lidar_imu [x y z qx qy qz qw]");
}

}  // namespace lo
