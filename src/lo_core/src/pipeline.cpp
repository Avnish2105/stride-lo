#include "lo_core/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "lo_core/util/parallel.hpp"

namespace lo {

namespace {

using WallClock = std::chrono::steady_clock;

double msSince(WallClock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(WallClock::now() - t0).count();
}

}  // namespace

Pipeline::Pipeline(
  const Config & cfg, const Extrinsics & ext, std::shared_ptr<const KinematicModel> kinematics)
: cfg_(cfg),
  ext_(ext),
  T_lidar_base_(ext.T_base_lidar.inverse()),
  validator_(cfg),
  clock_(cfg.clock),
  imu_(cfg),
  joints_(static_cast<std::size_t>(cfg.limits.joint_buffer)),
  graph_(cfg),
  self_(cfg, ext, std::move(kinematics)),
  pre_(cfg),
  cloud_(static_cast<std::size_t>(cfg.limits.max_registration_points)),
  deskew_(cfg, ext),
  cov_(cfg, cloud_),
  seed_model_(cfg, ext),
  map_(cfg),
  reg_(cfg, static_cast<std::size_t>(cfg.limits.max_registration_points)),
  gate_(cfg.gate, cfg.range.max),
  bias_(cfg),
  sm_(cfg),
  emitter_(cfg, static_cast<std::size_t>(cfg.limits.max_registration_points), ext.T_base_lidar),
  scan_offset_(fromSec(cfg.scan.time_offset)),
  imu_offset_(fromSec(cfg.imu.time_offset)),
  residuals_(static_cast<std::size_t>(cfg.integrity.residual_window)),
  timings_(static_cast<std::size_t>(cfg.limits.timing_window))
{
  initParallel();
}

// ---------------------------------------------------------------------------------- inputs

void Pipeline::addImu(const ImuSample & s_in, TimeNs arrival)
{
  if (!cfg_.imu.enabled) {return;}
  ImuSample s = s_in;
  s.t += imu_offset_;                      // imu.time_offset
  if (!imu_.push(s)) {
    ++diag_.imu_rejected;
    return;
  }
  clock_.noteImu(s.t, arrival);
}

void Pipeline::addJoints(const JointSample & s)
{
  if (!joints_.push(s)) {++diag_.joints_rejected;}
}

void Pipeline::addGraphOdom(const GraphOdomSample & s)
{
  if (!graph_.push(s)) {++diag_.graph_rejected;}
}

void Pipeline::addNodeRequest(TimeNs t) {emitter_.addRequest(t);}

void Pipeline::addPtpOffset(TimeNs t, double offset_s) {clock_.notePtpOffset(t, offset_s);}

Pipeline::Coverage Pipeline::coverage(TimeNs scan_end) const
{
  if (!cfg_.imu.enabled || imu_.empty()) {return Coverage::READY;}
  scan_end += scan_offset_;                // LiDAR clock -> IMU clock (scan.time_offset)
  const auto & last = imu_.ring().back();
  if (last.te >= scan_end) {return Coverage::READY;}
  if (scan_end - last.t > fromSec(cfg_.imu.lost_timeout)) {return Coverage::READY;}   // lost
  return Coverage::WAIT;
}

// --------------------------------------------------------------------------------- process

const ScanResult & Pipeline::process(const RawScan & scan_in, TimeNs arrival)
{
  RawScan scan = scan_in;                  // view (pointer + size): no copy of the points
  scan.t0 += scan_offset_;
  const auto t_wall = WallClock::now();
  res_ = ScanResult{};
  just_lost_ = false;
  ++diag_.scans;
  res_.num_raw = static_cast<int>(scan.size);
  res_.t_start = scan.t0;
  res_.t_ref = scan.t0;

  auto finish = [&]() -> const ScanResult & {
      emitter_.answerRequests(res_.factors);   // request mode: everything decidable now
      res_.state = sm_.state();
      res_.chain_id = chain_id_;
      res_.timings.total = msSince(t_wall);
      timings_.push(res_.timings.total);
      fillDiagnostics();
      return res_;
    };

  // ---- S0: validation and time
  auto t0 = WallClock::now();
  const ScanCheck chk = validator_.check(scan);
  if (chk.t_end > 0) {clock_.noteLidar(chk.t_end, arrival);}
  if (chk.clock_fault) {clock_.reportFault(scan.t0);}
  const bool clock_ok = clock_.evaluate(scan.t0);
  diag_.dropped_scans += static_cast<std::uint64_t>(chk.dropped_scans);
  res_.num_valid = chk.num_valid;
  res_.t_end = chk.t_end;
  res_.timings.validate = msSince(t0);
  if (chk.reason != RejectReason::NONE) {
    reject(chk.reason, false);
    return finish();
  }
  if (!clock_ok) {
    reject(RejectReason::CLOCK_UNSTABLE, false);
    return finish();
  }
  const TimeNs t_s = scan.t0;
  const TimeNs t_e = chk.t_end;
  emitter_.noteProcessed(t_e);   // requests up to t_e are decided when this call finishes

  // ---- IMU status for this scan
  const bool imu_on = cfg_.imu.enabled && !imu_.empty();
  const bool imu_lost = !imu_on || (t_e - imu_.latestStamp() > fromSec(cfg_.imu.lost_timeout));
  bool use_imu_deskew = false;
  std::uint32_t scan_flags = 0;
  if (imu_lost) {
    scan_flags |= flags::IMU_MISSING | flags::DESKEW_FALLBACK | flags::GYRO_GATE_DISABLED;
  } else if (imu_.covers(t_s, t_e) && imu_.maxGap(t_s, t_e) <= cfg_.imu.gap_max) {
    use_imu_deskew = true;
  } else {
    scan_flags |= flags::DESKEW_FALLBACK;
  }
  if (cfg_.imu.enabled && !bias_.calibrated()) {scan_flags |= flags::BIAS_UNCALIBRATED;}

  // ---- reference time: a pending graph-node request inside the scan, else the scan end.
  // Requests that arrive after this scan are answered from its FrameRecord (I23).
  TimeNs t_ref = t_e;
  if (emitter_.requestMode()) {
    TimeNs tr = 0;
    if (emitter_.firstRequestIn(t_s, t_e, tr)) {t_ref = tr;}
  }
  res_.t_ref = t_ref;

  // ---- S1 + S2: self-filter, range gate, adaptive voxel, downsampling
  t0 = WallClock::now();
  if (!self_.prepare(t_s, t_e, joints_)) {
    scan_flags |= flags::JOINTS_MISSING;
    ++diag_.joints_missing_scans;
  }
  res_.timings.self_filter = msSince(t0);
  t0 = WallClock::now();
  const PreprocessStats pst = pre_.run(scan, t_ref, self_, cloud_);
  res_.timings.downsample = msSince(t0);
  res_.num_filtered = pst.num_self;
  res_.num_registration = pst.num_registration;
  res_.voxel_size = pst.voxel;
  res_.near_fraction = pst.near_fraction;
  if (pst.blocked) {
    scan_flags |= flags::SENSOR_BLOCKED;
    ++diag_.blocked_scans;
  }
  if (static_cast<int>(cloud_.size()) < cfg_.gicp.min_points) {
    reject(RejectReason::TOO_FEW_REG_POINTS, false);
    return finish();
  }
  if (sm_.state() == HealthState::LOST) {sm_.onScanWhileLost(t_ref);}
  const bool recovering = sm_.state() == HealthState::RECOVERING;

  // ---- S5: seed
  t0 = WallClock::now();
  SeedResult seed;
  if (has_prev_) {
    seed = seed_model_.predict(t_prev_, t_ref, T_prev_, imu_, !imu_lost, bias_.bias(), graph_, lo_vel_);
    if (seed.graph_gated) {++diag_.graph_gated;}
  } else {
    seed.flags = flags::SEED_FALLBACK | flags::SEED_NO_GRAPH | flags::SEED_IDENTITY;
  }
  res_.timings.seed = msSince(t0);

  // ---- S3: continuous-time deskew
  t0 = WallClock::now();
  const Vec3 w_fallback = lo_vel_.valid ? lo_vel_.w_lidar : Vec3::Zero();
  v_deskew_ = seed.v_deskew;
  const DeskewMode mode = deskew_.prepare(imu_, use_imu_deskew, t_s, t_e, t_ref, bias_.bias(),
    seed.v_deskew, w_fallback);
  if (mode == DeskewMode::FALLBACK) {++diag_.deskew_fallbacks;}
  deskew_.apply(cloud_, cfg_.threads.num_threads);
  res_.timings.deskew = msSince(t0);

  // ---- S4: per-point covariances
  t0 = WallClock::now();
  cov_.compute(cloud_);
  res_.timings.covariance = msSince(t0);

  // ---- S6..S9 by state
  if (sm_.state() == HealthState::INIT) {
    handleInit(t_ref, seed, pst.median_range, !imu_lost);
  } else {
    handleTracking(t_ref, seed, pst.median_range, !imu_lost, recovering, scan_flags | seed.flags);
  }
  return finish();
}

// --------------------------------------------------------------------------------- helpers

void Pipeline::reject(RejectReason reason, bool)
{
  res_.verdict = Verdict::REJECT;
  res_.reason = reason;
  ++diag_.rejected;
  ++diag_.reject_reasons[static_cast<std::size_t>(reason)];
  emitter_.noteReject();
  const HealthState before = sm_.state();
  const TimeNs t = res_.t_ref;
  if (before == HealthState::TRACKING || before == HealthState::DEGRADED) {
    sm_.onVerdict(Verdict::REJECT, t);
    if (sm_.state() == HealthState::LOST) {just_lost_ = true;}
  } else if (before == HealthState::LOST) {
    sm_.onScanWhileLost(t);
  }
  if (sm_.state() == HealthState::RECOVERING && sm_.recoveryExpired(t)) {
    sm_.toInit();
    warm_init_ = true;
  }
}

static bool jointsStillAt(const TimeRing<JointSample> & joints, const Config & cfg, TimeNs t)
{
  if (!cfg.standstill.use_joints || joints.empty()) {return true;}
  if (t - joints.back().t > fromSec(cfg.self_filter.joints_timeout)) {return false;}
  double speed = 0.0;
  if (!jointBlockSpeed(joints, t, fromSec(cfg.standstill.block_duration), speed)) {return false;}
  return speed < cfg.standstill.joint_vel_max;
}

static bool gyroStillAt(const ImuBuffer & imu, const Config & cfg, const Vec3 & bias, TimeNs t)
{
  if (!cfg.imu.enabled || imu.empty()) {return true;}
  Vec3 g, a;
  if (imu.mean(t - fromSec(cfg.standstill.block_duration), t, g, a) == 0) {return false;}
  return (g - bias).norm() < cfg.standstill.gyro_max;
}

double Pipeline::effectiveGate() const
{
  // Correspondences go to Gaussian voxel means, which lie up to about a voxel away from any
  // single point on the same surface: never gate tighter than that.
  return std::max(gate_.tau(), cfg_.gate.voxel_floor * map_.fine().leaf_size());
}

void Pipeline::startChain(TimeNs t_ref)
{
  chain_id_ = static_cast<std::uint64_t>(t_ref);
  emitter_.startChain(chain_id_);
}

void Pipeline::handleInit(TimeNs t_ref, const SeedResult & seed, double median_range,
  bool imu_usable)
{
  const auto t0 = WallClock::now();
  res_.verdict = Verdict::SKIPPED;
  ++diag_.skipped;

  if (warm_init_) {
    // DECISIONS D3: after a chain break keep bias/gravity, new map at the predicted pose.
    const SE3 T = has_prev_ ? seed.T_seed : SE3();
    map_.reset(res_.voxel_size);
    map_.insert(cloud_, T, t_ref);
    T_last_insert_ = T;
    has_insert_ = true;
    T_prev_ = T;
    t_prev_ = t_ref;
    has_prev_ = true;
    warm_init_ = false;
    init_map_built_ = true;
    has_vel_anchor_ = false;
    has_gate_anchor_ = false;
    lo_vel_.gate_valid = false;
    gate_.reset();
    residuals_.clear();
    startChain(t_ref);
    sm_.toTracking(t_ref);
    const FrameRecord & rec = recordFrame(t_ref, T, Mat6::Identity() * kAnchorVariance);
    if (!emitter_.requestMode()) {emitter_.emitNewest(rec, cloud_, res_.voxel_size, res_.factors);}
    res_.pose_valid = true;
    res_.T_odom_lidar = T_align_ * T;
    res_.T_odom_base = res_.T_odom_lidar * T_lidar_base_;
    res_.map_updated = true;
    res_.timings.registration = msSince(t0);
    return;
  }

  if (!init_map_built_) {
    // cold start, first usable scan: INIT map in the frame of this scan
    map_.reset(res_.voxel_size);
    map_.insert(cloud_, SE3(), t_ref);
    T_init_first_ = SE3();
    T_prev_ = SE3();
    t_prev_ = t_ref;
    has_prev_ = true;
    init_map_built_ = true;
    bias_.closeWindow();
    res_.map_updated = true;
    return;
  }

  // cold INIT: register against the INIT map to collect standstill evidence
  AlignResult a = reg_.align(map_, cloud_, seed.T_seed, effectiveGate(), median_range,
    cfg_.gate.max, cfg_.gicp.coarse_iters);
  res_.timings.registration = msSince(t0);
  res_.iterations_coarse = a.iterations_coarse;
  res_.iterations_fine = a.iterations_fine;
  const double inlier_ratio = a.num_points ? static_cast<double>(a.num_inliers) / a.num_points : 0.0;
  res_.inlier_ratio = inlier_ratio;
  // the single-scan INIT map is sparse: hold it with half the tracking inlier threshold
  if (!a.converged || !a.numerical_ok || a.blowup ||
    inlier_ratio < 0.5 * cfg_.integrity.inlier_min)
  {
    // the robot moved away from the INIT map: start the INIT map again from this scan
    map_.reset(res_.voxel_size);
    map_.insert(cloud_, SE3(), t_ref);
    T_prev_ = SE3();
    t_prev_ = t_ref;
    bias_.closeWindow();
    return;
  }
  const SE3 T = a.T;
  const SE3 d = T_prev_.inverse() * T;
  const double g = cfg_.standstill.gross_factor;
  const bool lo_still = d.translation().norm() < g * cfg_.standstill.lo_motion_max &&
    angleOf(d) < g * cfg_.standstill.lo_rot_max_deg * kDegToRad;
  const bool still = lo_still && jointsStillAt(joints_, cfg_, t_ref) &&
    gyroStillAt(imu_, cfg_, bias_.bias(), t_ref);
  const bool use_imu = cfg_.imu.enabled && imu_usable;
  const bool ready = bias_.update(still, t_prev_, t_ref, use_imu ? &imu_ : nullptr, T);
  // densify the INIT map while standing (the robot is still, poses are consistent)
  if (still) {
    map_.insert(cloud_, T, t_ref);
    res_.map_updated = true;
  }
  T_prev_ = T;
  t_prev_ = t_ref;
  // INIT poses live in the INIT-map frame: the output odom frame exists once INIT completes
  // (finishInit fixes T_align_), so no pose is reported before that.
  if (ready) {
    finishInit(t_ref, T);
    res_.pose_valid = true;
  }
}

void Pipeline::finishInit(TimeNs t_ref, const SE3 & T_current)
{
  // Output odom frame: gravity aligned (roll/pitch from the standstill accelerometer mean),
  // yaw 0, origin at the base. Only the debug odometry sees it (factors are relative), so the
  // INIT map stays in its own frame and T_align_ maps internal poses to output poses.
  SO3 R_odom_base;
  if (bias_.hasGravity()) {
    const Vec3 up_base =
      ((ext_.T_base_lidar.so3() * ext_.T_lidar_imu.so3()) * bias_.gravityUp()).normalized();
    // minimal rotation taking up_base onto +z (no yaw)
    const Vec3 axis = up_base.cross(Vec3::UnitZ());
    const double s = axis.norm();
    const double c = std::clamp(up_base.dot(Vec3::UnitZ()), -1.0, 1.0);
    if (s > 1e-9) {
      R_odom_base = SO3::exp(axis / s * std::atan2(s, c));
    } else if (c < 0.0) {
      R_odom_base = SO3::exp(Vec3(M_PI, 0.0, 0.0));
    }
  }
  const SE3 T_odom_lidar0 = SE3(R_odom_base, Vec3::Zero()) * ext_.T_base_lidar;
  T_align_ = T_odom_lidar0 * T_current.inverse();
  T_last_insert_ = T_current;
  has_insert_ = true;
  T_prev_ = T_current;
  t_prev_ = t_ref;
  has_prev_ = true;
  lo_vel_ = LoVelocity{};
  lo_vel_.valid = true;   // standing still
  has_vel_anchor_ = false;
  has_gate_anchor_ = false;
  clock_.setBaseline();
  gate_.reset();
  residuals_.clear();
  startChain(t_ref);
  sm_.toTracking(t_ref);
  const FrameRecord & rec = recordFrame(t_ref, T_current, Mat6::Identity() * kAnchorVariance);
  if (!emitter_.requestMode()) {emitter_.emitNewest(rec, cloud_, res_.voxel_size, res_.factors);}
  res_.T_odom_lidar = T_align_ * T_current;
  res_.T_odom_base = res_.T_odom_lidar * T_lidar_base_;
}

Pipeline::Verdicts Pipeline::evaluate(
  const AlignResult & a, const SeedResult & seed, TimeNs t_ref, bool imu_usable, bool recovering,
  std::uint32_t scan_flags)
{
  Verdicts out;
  out.flags = scan_flags;
  auto fail = [&](RejectReason r) {
      out.verdict = Verdict::REJECT;
      out.reason = r;
      return out;
    };
  if (!a.numerical_ok) {return fail(RejectReason::NUMERICAL);}
  if (a.blowup) {return fail(RejectReason::SOLVER_BLOWUP);}
  if (!a.converged) {return fail(RejectReason::NOT_CONVERGED);}
  const double inlier_ratio = a.num_points ? static_cast<double>(a.num_inliers) / a.num_points : 0.0;
  if (inlier_ratio < cfg_.integrity.inlier_min) {return fail(RejectReason::LOW_INLIERS);}
  if (static_cast<int>(residuals_.size()) >= cfg_.integrity.residual_warmup) {
    const double med = residuals_.median();
    if (med > 0.0 && a.residual_rms > cfg_.integrity.residual_ratio_max * med) {
      return fail(RejectReason::RESIDUAL);
    }
  }
  const bool gyro_gate = !(scan_flags & flags::GYRO_GATE_DISABLED) && imu_usable;
  const double scale = gyro_gate ? 1.0 : cfg_.integrity.imu_lost_plausibility_scale;
  if (has_prev_) {
    const SE3 d = T_prev_.inverse() * a.T;
    const double dt = std::max(toSec(t_ref - t_prev_), 1e-3);
    if (gyro_gate) {
      SO3 dR_imu;
      if (imu_.relativeRotation(t_prev_, t_ref, bias_.bias(), dR_imu)) {
        const SO3 R_li = ext_.T_lidar_imu.so3();
        const SO3 dR_gyro = R_li * dR_imu * R_li.inverse();
        if (angleOf(d.so3().inverse() * dR_gyro) > cfg_.integrity.gyro_rot_max_deg * kDegToRad) {
          return fail(RejectReason::GYRO_MISMATCH);
        }
      }
    }
    if (d.translation().norm() / dt > cfg_.integrity.v_max * scale ||
      angleOf(d) / dt > cfg_.integrity.w_max * scale)
    {
      return fail(RejectReason::IMPLAUSIBLE_MOTION);
    }
  }
  if (!recovering) {
    // RECOVERING seeds span seconds; the wide gate is the point there (DECISIONS I13)
    const SE3 dev = seed.T_seed.inverse() * a.T;
    if (dev.translation().norm() > cfg_.integrity.seed_dev_trans * scale ||
      angleOf(dev) > cfg_.integrity.seed_dev_rot_deg * kDegToRad * scale)
    {
      return fail(RejectReason::SEED_DEVIATION);
    }
  }
  Eigen::LLT<Mat6> llt(a.H);
  if (!a.H.allFinite() || llt.info() != Eigen::Success) {return fail(RejectReason::NUMERICAL);}

  if (a.degeneracy.num_degenerate > 0) {
    out.flags |= flags::DEGENERATE;
    out.verdict = Verdict::DEGRADED;
  }
  if (scan_flags & (flags::SENSOR_BLOCKED | flags::DESKEW_FALLBACK)) {out.verdict = Verdict::DEGRADED;}
  return out;
}

void Pipeline::handleTracking(
  TimeNs t_ref, const SeedResult & seed, double median_range, bool imu_usable, bool recovering,
  std::uint32_t scan_flags)
{
  auto t0 = WallClock::now();
  if (recovering) {sm_.onRecoveryAttempt(t_ref);}
  const double coarse_gate = recovering ? cfg_.recovery.gate : cfg_.gate.max;
  const int coarse_iters = recovering ? cfg_.recovery.coarse_iters : cfg_.gicp.coarse_iters;
  const double tau = effectiveGate();
  const AlignResult a = reg_.align(map_, cloud_, seed.T_seed, tau, median_range,
    coarse_gate, coarse_iters);
  res_.timings.registration = msSince(t0);
  res_.iterations_coarse = a.iterations_coarse;
  res_.iterations_fine = a.iterations_fine;
  res_.gate = tau;
  res_.inlier_ratio = a.num_points ? static_cast<double>(a.num_inliers) / a.num_points : 0.0;
  res_.residual_rms = a.residual_rms;
  res_.num_degenerate = a.degeneracy.num_degenerate;
  res_.eigenvalues = a.degeneracy.eigenvalues;
  {
    const SE3 dev = seed.T_seed.inverse() * a.T;
    res_.seed_dev_trans = dev.translation().norm();
    res_.seed_dev_rot_deg = angleOf(dev) * kRadToDeg;
    res_.max_step_trans = a.max_step_trans;
    res_.max_step_rot_deg = a.max_step_rot * kRadToDeg;
  }

  t0 = WallClock::now();
  const Verdicts v = evaluate(a, seed, t_ref, imu_usable, recovering, scan_flags);
  res_.timings.integrity = msSince(t0);
  res_.flags = v.flags;
  if (v.verdict == Verdict::REJECT) {
    reject(v.reason, true);
    return;
  }

  // ---------------- accepted (ACCEPT / DEGRADED)
  if (recovering) {
    // Found again after a long gap: the wide-gate match may lock onto a similar-looking place
    // (seed error grows with the gap; 6.7 m along a degenerate axis after an 11 s clock fault,
    // measured). Keep tracking, but do not bridge the gap with a factor (DECISIONS I38).
    const bool long_gap = has_prev_ && t_ref - t_prev_ > fromSec(cfg_.recovery.max_bridge);
    sm_.toTracking(t_ref);
    emitter_.noteRecovery();
    if (long_gap) {
      sm_.noteChainBreak();
      startChain(t_ref);
    }
  }
  const SE3 T = a.T;
  map_.touch(cloud_, T, t_ref);

  // covariance (docs/DESIGN.md "Covariance model", DECISIONS I4)
  const double dof = std::max(1, a.num_inliers - 6);
  const double sigma2 = a.sum_weighted_maha / dof;
  Mat6 cov_lidar = registrationCovariance(a.H, a.degeneracy, sigma2, cfg_);
  if (v.flags & (flags::DESKEW_FALLBACK | flags::SENSOR_BLOCKED | flags::IMU_MISSING |
    flags::GYRO_GATE_DISABLED))
  {
    cov_lidar *= cfg_.cov.fallback_scale;
  }
  // Deskew without the gyro (constant LO twist) cannot follow a trotting body's roll/pitch
  // oscillation inside a scan: 25-40 mrad rotation errors per factor on rough terrain with a
  // 1.5 s IMU dropout (bag fault suite, DECISIONS I35). Keep the translation, drop the claim.
  if (v.flags & flags::DESKEW_FALLBACK) {
    const double s = cfg_.cov.fallback_rot_sigma;
    cov_lidar.topLeftCorner<3, 3>() += s * s * Mat3::Identity();
  }

  // S8
  t0 = WallClock::now();
  // A constant-velocity deskew (IMU gap / lost) may leave rotational smear: such a scan may
  // emit an inflated factor but must not shape the map that later scans are measured against.
  if (!(v.flags & (flags::DESKEW_FALLBACK | flags::IMU_MISSING))) {
    maybeInsert(T, t_ref, v.verdict, a.degeneracy.num_degenerate);
  }
  res_.timings.map = msSince(t0);

  // S9: this scan becomes a potential graph frame. Request answers (also for requests that
  // arrive later) happen in answerRequests(); overlap mode decides here.
  t0 = WallClock::now();
  FrameRecord & rec = recordFrame(t_ref, T, cov_lidar);
  FactorOut & f = rec.attrs;
  f.health = a.degeneracy.num_degenerate > 0 ? health::DEGRADED : health::OK;
  f.hessian_eigenvalues = a.degeneracy.eigenvalues;
  f.num_degenerate = degenerateDirections(a.degeneracy, f.degenerate_directions);   // LiDAR frame
  f.inlier_ratio = static_cast<float>(res_.inlier_ratio);
  f.residual_rms = static_cast<float>(a.residual_rms);
  f.flags = v.flags;
  if (!emitter_.requestMode() &&
    (!emitter_.hasAnchor() || emitter_.overlap(cloud_, T) < cfg_.emit.overlap_new_kf))
  {
    emitter_.emitNewest(rec, cloud_, res_.voxel_size, res_.factors);
  }
  res_.timings.emit = msSince(t0);

  // bookkeeping
  updateMotionModels(T, t_ref, seed);
  residuals_.push(a.residual_rms);
  updateBias(T, t_ref, imu_usable);
  T_prev_ = T;
  t_prev_ = t_ref;
  has_prev_ = true;

  res_.verdict = v.verdict;
  res_.pose_valid = true;
  res_.T_odom_lidar = T_align_ * T;
  res_.T_odom_base = res_.T_odom_lidar * T_lidar_base_;
  const SO3 R_bl = ext_.T_base_lidar.so3();
  res_.w_base = R_bl * lo_vel_.w_lidar;
  res_.v_base = R_bl * lo_vel_.v_lidar - res_.w_base.cross(ext_.T_base_lidar.translation());
  if (v.verdict == Verdict::ACCEPT) {++diag_.accepted;} else {++diag_.degraded;}
  sm_.onVerdict(v.verdict, t_ref);
}

void Pipeline::updateMotionModels(const SE3 & T, TimeNs t_ref, const SeedResult & seed)
{
  if (!has_prev_) {return;}
  const SE3 d = T_prev_.inverse() * T;
  gate_.update(seed.T_seed.inverse() * T, d.translation().norm());
  // LO velocity over a baseline of >= kVelBaseline: consecutive reference times can be ms apart
  // (a scan referenced at its end, the next at a request time), and 1 cm of pose noise over
  // 7 ms is > 1 m/s (it seeded a 0.3 m error and a LOST on rough 5, measured).
  if (!has_vel_anchor_) {
    vel_anchor_t_ = t_prev_;
    vel_anchor_T_ = T_prev_;
    has_vel_anchor_ = true;
  }
  // graph-gate window (>= kGateBaseline)
  if (!has_gate_anchor_) {
    gate_anchor_t_ = t_prev_;
    gate_anchor_T_ = T_prev_;
    has_gate_anchor_ = true;
  }
  const double gdt = toSec(t_ref - gate_anchor_t_);
  if (gdt >= kGateBaseline) {
    lo_vel_.gate_valid = gdt < 2.0;
    lo_vel_.g0 = gate_anchor_t_;
    lo_vel_.g1 = t_ref;
    lo_vel_.g_disp = (gate_anchor_T_.inverse() * T).translation();
    gate_anchor_t_ = t_ref;
    gate_anchor_T_ = T;
  }
  const double dt = toSec(t_ref - vel_anchor_t_);
  if (dt < kVelBaseline) {return;}
  if (dt < 1.0) {
    const SE3 dv = vel_anchor_T_.inverse() * T;
    lo_vel_.v_lidar = dv.translation() / dt;
    lo_vel_.w_lidar = dv.so3().log() / dt;
    lo_vel_.valid = true;
    lo_vel_.t0 = vel_anchor_t_;
    lo_vel_.t1 = t_ref;
  } else {
    lo_vel_.valid = false;
  }
  vel_anchor_t_ = t_ref;
  vel_anchor_T_ = T;
}

void Pipeline::updateBias(const SE3 & T, TimeNs t_ref, bool imu_usable)
{
  if (!cfg_.imu.enabled || !imu_usable || !has_prev_) {
    bias_.closeWindow();
    return;
  }
  const SE3 d = T_prev_.inverse() * T;
  const double g = cfg_.standstill.gross_factor;
  const bool lo_still = d.translation().norm() < g * cfg_.standstill.lo_motion_max &&
    angleOf(d) < g * cfg_.standstill.lo_rot_max_deg * kDegToRad;
  const bool still = lo_still && jointsStillAt(joints_, cfg_, t_ref) &&
    gyroStillAt(imu_, cfg_, bias_.bias(), t_ref);
  bias_.update(still, t_prev_, t_ref, &imu_, T);
}

void Pipeline::maybeInsert(const SE3 & T, TimeNs t_ref, Verdict v, int num_degenerate)
{
  if (v == Verdict::DEGRADED && num_degenerate > 1) {return;}   // map would be mostly seed
  if (has_insert_) {
    const SE3 d = T_last_insert_.inverse() * T;
    if (d.translation().norm() < cfg_.map.insert_dist &&
      angleOf(d) < cfg_.map.insert_rot_deg * kDegToRad)
    {
      return;
    }
  }
  if (t_ref - last_stale_check_ >= kNsPerSec) {
    map_.removeStale(t_ref, T.translation());
    last_stale_check_ = t_ref;
  }
  if (map_.needsRebin(res_.voxel_size)) {map_.rebin(res_.voxel_size);}
  diag_.map_points_dropped += map_.insert(cloud_, T, t_ref);
  T_last_insert_ = T;
  has_insert_ = true;
  res_.map_updated = true;
}

void Pipeline::updatePercentiles()
{
  diag_.timing_median_ms = timings_.median();
  diag_.timing_p99_ms = timings_.percentile(0.99);
  diag_.timing_max_ms = timings_.max();
}

void Pipeline::fillDiagnostics()
{
  diag_.state = sm_.state();
  diag_.chain_id = chain_id_;
  diag_.factors = emitter_.factors();
  diag_.anchors = emitter_.anchors();
  diag_.requests_received = emitter_.requestsReceived();
  diag_.requests_answered = emitter_.requestsAnswered();
  diag_.requests_missed = emitter_.requestsMissed();
  diag_.losts = sm_.losts();
  diag_.recoveries = sm_.recoveries();
  diag_.chain_breaks = sm_.chainBreaks();
  diag_.map_rebins = map_.rebins();
  diag_.clock_faults = clock_.faults();
  diag_.clock_skew = clock_.skew();
  diag_.lidar_latency = clock_.lidarLatency();
  diag_.imu_latency = clock_.imuLatency();
  diag_.ptp_offset = clock_.ptpOffset();
  diag_.imu_rate = imu_.rate();
  diag_.map_fine = map_.fine().size();
  diag_.map_coarse = map_.coarse().size();
  diag_.voxel = res_.voxel_size;
  diag_.gate = gate_.tau();
  diag_.gyro_bias = bias_.bias();
  diag_.bias_calibrated = bias_.calibrated();
  diag_.last_timings = res_.timings;
  diag_.last_flags = res_.flags;
  diag_.last_num_degenerate = res_.num_degenerate;
  if (!res_.factors.empty()) {diag_.last_eigenvalues = res_.factors.back().hessian_eigenvalues;}
}

FrameRecord & Pipeline::recordFrame(TimeNs t_ref, const SE3 & T, const Mat6 & cov_lidar)
{
  FrameRecord & rec = emitter_.newRecord();
  rec.T_odom_lidar = T;
  rec.cov_lidar = cov_lidar;
  rec.setMotion(res_.t_start, res_.t_end, t_ref, fromSec(cfg_.emit.request_tolerance), v_deskew_,
    deskew_.lever(), [this](double dt) {return deskew_.rotationAt(dt);});
  return rec;
}

const FactorList & Pipeline::answerRequests()
{
  late_.clear();
  if (emitter_.answerRequests(late_) > 0) {
    diag_.last_eigenvalues = late_.back().hessian_eigenvalues;
  }
  diag_.factors = emitter_.factors();
  diag_.anchors = emitter_.anchors();
  diag_.requests_received = emitter_.requestsReceived();
  diag_.requests_answered = emitter_.requestsAnswered();
  diag_.requests_missed = emitter_.requestsMissed();
  return late_;
}

}  // namespace lo
