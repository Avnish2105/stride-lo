// All STRIDE parameters with their defaults (docs/DESIGN.md "Default parameters" + the choices
// in docs/DECISIONS.md). The single schema (names, ranges, descriptions) is visitParams() in
// schema.hpp: lo_ros declares ROS parameters from it, lo_tools loads YAML through it, and
// validate() / configHash() walk it. Angles in this struct are in DEGREES where the name says
// so (`*_deg`), everything else SI.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lo_core/types/inputs.hpp"

namespace lo {

struct Config
{
  struct Limits
  {
    int max_raw_points = 40000;            ///< points per scan accepted (rest dropped + counted)
    int max_registration_points = 20000;   ///< after downsampling (uniform thinning beyond)
    int map_fine_capacity = 600000;        ///< voxels
    int map_coarse_capacity = 80000;       ///< voxels
    int imu_buffer = 2048;                 ///< samples (>= 2 s at 500 Hz + margin)
    int joint_buffer = 4096;               ///< samples
    int graph_buffer = 512;                ///< graph odometry samples
    int request_buffer = 64;               ///< pending node requests
    int frame_records = 8;                 ///< accepted scans kept to answer late requests
    int timing_window = 600;               ///< scans kept for timing percentiles
  } limits;

  struct Threads
  {
    int num_threads = 2;                   ///< OpenMP team (worker + 1)
  } threads;

  struct Scan
  {
    double period = 0.1;                   ///< expected scan period [s]
    double period_tolerance = 0.05;        ///< reject if |dt - period| exceeds this [s]
    double drop_tolerance = 0.02;          ///< dt = n * period (n >= 2) within this: dropped scans
    int min_valid_points = 3000;
    double max_point_offset = 0.12;        ///< valid point times in [0, this] [s]
    double time_offset = 0.0;              ///< added to LiDAR stamps: LiDAR clock -> IMU clock [s]
    /// Test data only: snap point times to the nearest k * quantum + phase (simulators that
    /// render geometry at a fixed rate but stamp points continuously). 0 = off (real sensors).
    double time_quantum = 0.0;
    double time_quantum_phase = 0.0;
  } scan;

  struct Clock
  {
    double max_offset = 0.002;             ///< external PTP offset limit (DESIGN: 2 ms) [s]
    double max_skew = 0.002;               ///< arrival-based IMU-LiDAR skew proxy limit [s]
    double window = 2.0;                   ///< rolling-min window for latency estimates [s]
    double stable_time = 2.0;              ///< after a clock fault: stable this long before emitting [s]
    bool check_skew = true;
  } clock;

  struct Range
  {
    double min = 0.3;
    double max = 60.0;
    bool drop_zero_reflectivity = true;
    bool drop_noise_tag = true;            ///< Livox tag bits[1:0] == 01 (high-confidence noise)
  } range;

  struct Voxel
  {
    double base_scale = 0.1;               ///< v = clamp(base_scale * median_range, min, max)
    double base_min = 0.1;
    double base_max = 0.5;
    double ema_alpha = 0.1;                ///< smoothing of the median range
    double initial = 0.3;                  ///< v before the first scan
    double map_ratio = 0.5;                ///< fine map voxel = map_ratio * v (DECISIONS D7)
    double coarse_ratio = 4.0;             ///< coarse map voxel = coarse_ratio * v
    double rebin_threshold = 0.25;         ///< re-bin the map when v drifts by this fraction
  } voxel;

  struct Knn
  {
    int k = 10;
    double plane_eps = 1e-3;               ///< regularised eigenvalues (1, 1, plane_eps)
  } knn;

  struct Imu
  {
    bool enabled = true;
    double min_rate = 200.0;               ///< below this the IMU counts as degraded [Hz]
    double accel_scale = 1.0;              ///< multiply raw accel (Livox publishes g: 9.80665)
    std::string sample_convention = "instant";   ///< instant | interval_mean (DECISIONS I9)
    double time_offset = 0.0;              ///< added to IMU stamps: IMU clock -> LiDAR clock [s]
    double gap_max = 0.02;                 ///< gap inside a scan span -> DESKEW_FALLBACK [s]
    double lost_timeout = 1.0;             ///< no IMU this long -> IMU lost [s]
  } imu;

  struct SelfFilter
  {
    bool enabled = true;
    /// Static body crop box in the base frame: [xmin ymin zmin xmax ymax zmax].
    std::vector<double> body_box = {-0.45, -0.25, -0.20, 0.45, 0.25, 0.20};
    double inflate = 0.03;                 ///< around FK leg capsules [m]
    double link_radius = 0.05;             ///< capsule radius of moving links (URDF bones) [m]
    double foot_radius = 0.04;             ///< sphere radius of moving leaf links (feet) [m]
    double sphere_radius = 1.5;            ///< only points this close to base are tested [m]
    int fk_samples = 11;                   ///< FK evaluations across a scan (interpolated between)
    double joints_timeout = 0.1;           ///< joint states older than this -> static leg box [s]
    /// Conservative leg exclusion box (base frame) used when joints are lost.
    std::vector<double> leg_box = {-0.75, -0.50, -0.70, 0.75, 0.50, 0.10};
    double blocked_radius = 0.5;           ///< blockage monitor radius (sensor frame) [m]
    double blocked_fraction = 0.3;         ///< > this share within radius -> SENSOR_BLOCKED
  } self_filter;

  struct Deskew
  {
    bool enabled = true;
  } deskew;

  struct Seed
  {
    double graph_timeout = 0.5;            ///< graph odometry older than this is not used [s]
    /// Graph gate (the graph is an external estimate and can diverge): its LiDAR velocity must
    /// match LO's own displacement over its last >= 0.4 s window, as a mean velocity [m/s] ...
    double graph_max_dev = 0.15;
    double graph_holdoff = 2.0;            ///< ... else it is ignored for this long [s]
  } seed;

  struct Gicp
  {
    int coarse_iters = 5;
    int fine_iters = 15;
    int freeze_after = 10;                 ///< correspondences frozen from this iteration on
    double rot_eps = 1e-4;                 ///< convergence [rad]
    double trans_eps = 1e-4;               ///< convergence [m]
    int coarse_search = 7;                 ///< voxel neighbourhood 1 | 7 | 27
    int fine_search = 27;                  ///< fine voxels are 0.5 v: search 3x3x3 (DECISIONS D7)
    double damping = 1e-6;                 ///< Gauss-Newton diagonal damping
    double orientation_min_cos = 0.0;      ///< far-side rejection: dot(n_src, n_voxel) < this
    double normal_consistency_min = 0.5;   ///< voxel normal used only if |sum n|/count >= this
    int min_points = 300;                  ///< fewer registration points -> REJECT
  } gicp;

  struct Gate
  {
    double initial = 1.0;                  ///< KISS-style adaptive tau [m]
    double min = 0.1;
    double max = 2.0;
    double sigma_factor = 3.0;             ///< tau = sigma_factor * sigma_dev
    double min_motion = 0.02;              ///< model deviation counted only above this motion per scan [m]
    double voxel_floor = 1.0;              ///< effective tau >= voxel_floor * fine voxel size
  } gate;

  struct Kernel
  {
    double scale_ratio = 1.0 / 3.0;        ///< Geman-McClure scale = scale_ratio * tau
  } kernel;

  struct Integrity
  {
    double inlier_min = 0.5;
    double residual_ratio_max = 2.0;       ///< residual RMS vs running median
    int residual_window = 50;
    int residual_warmup = 10;              ///< scans before the residual check applies
    double gyro_rot_max_deg = 3.0;         ///< gate only
    double v_max = 2.5;                    ///< [m/s]
    double w_max = 4.0;                    ///< [rad/s]
    double seed_dev_trans = 0.3;           ///< [m]
    double seed_dev_rot_deg = 10.0;
    double step_max_trans = 0.5;           ///< single solver step (blow-up) [m]
    double step_max_rot_deg = 10.0;
    double imu_lost_plausibility_scale = 0.5;   ///< DECISIONS I3
  } integrity;

  struct Degeneracy
  {
    double lambda_min = 0.02;              ///< point-to-plane eig/N: synth corridor <0.01, crowd p01 0.045
    int max_directions = 3;
    double sigma_trans = 1.0;              ///< sigma along degenerate directions [m]
    double sigma_rot = 0.5;                ///< [rad]
  } degeneracy;

  struct Cov
  {
    double kappa = 20.0;                   ///< tune so mean NEES ~ 6
    double floor_rot_deg = 0.05;           ///< 1 sigma per axis
    double floor_trans = 0.005;            ///< 1 sigma per axis [m]
    double fallback_scale = 4.0;           ///< deskew fallback / IMU lost / blocked inflation
    /// Weakly observed directions (lambda_min <= eig/N < weak_lambda): + weak_sigma^2 at
    /// lambda_min, fading to 0 at weak_lambda (scaled units, DECISIONS I31). 0 = off.
    double weak_lambda = 0.2;
    double weak_sigma = 0.025;
  } cov;

  struct Map
  {
    double insert_dist = 0.1;              ///< [m]
    double insert_rot_deg = 2.0;
    int max_pts_voxel = 20;
    double radius = 80.0;                  ///< [m]
    double ttl = 30.0;                     ///< unseen voxels removed after this [s]
  } map;

  struct Emit
  {
    std::string mode = "overlap";          ///< overlap | request (DECISIONS D1)
    double overlap_new_kf = 0.9;           ///< emit when overlap drops below
    double request_tolerance = 0.002;      ///< request may lie this far outside a scan span [s]
    double request_max_age = 1.0;          ///< pending requests older than the newest by this are missed [s]
  } emit;

  struct Lost
  {
    int consecutive_rejects = 5;
  } lost;

  struct Recovery
  {
    double timeout = 2.0;                  ///< then reset map + new chain [s]
    double gate = 4.0;                     ///< wide gate of the recovery coarse match [m]
    int coarse_iters = 10;
  } recovery;

  struct Degraded
  {
    double clear_time = 1.0;               ///< DEGRADED -> TRACKING after this long clean [s]
  } degraded;

  struct Standstill
  {
    double min_duration = 1.0;             ///< [s]
    double block_duration = 0.2;           ///< block means (DECISIONS I2) [s]
    double joint_vel_max = 0.05;           ///< block mean of |qd| [rad/s]
    double lo_motion_max = 0.001;          ///< mean LO translation per scan over a window [m]
    double lo_rot_max_deg = 0.05;          ///< mean LO rotation per scan over a window
    double gross_factor = 20.0;            ///< per-scan gate = gross_factor * the limits above
    double gyro_max = 0.05;                ///< block mean |gyro - bias| sanity [rad/s]
    double bias_alpha = 0.5;               ///< weight of a new standstill window in TRACKING
    bool use_joints = true;
  } standstill;

  struct ExtrinsicsCfg
  {
    std::string source = "tf";             ///< tf | params (lo_ros); lo_core always uses the values
    std::vector<double> base_lidar = {0, 0, 0, 0, 0, 0, 1};   ///< x y z qx qy qz qw
    std::vector<double> lidar_imu = {0, 0, 0, 0, 0, 0, 1};
  } extrinsics;
};

/// Rigid transforms from the extrinsics section.
Extrinsics extrinsicsFromConfig(const Config & c);

/// Every violated constraint (range, ordering, enum value, quaternion norm, ...); empty = valid.
std::vector<std::string> validate(const Config & c);

/// FNV-1a 64-bit over "name=value;" of every parameter in schema order.
std::uint64_t configHash(const Config & c);

/// "name=value" lines in schema order (logging, black box metadata).
std::string configDump(const Config & c);

}  // namespace lo
