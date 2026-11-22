#include "lo_synth/runner.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace lo::synth {

Config syntheticConfig(const Scenario & sc, bool use_requests)
{
  Config c;
  c.imu.accel_scale = sc.imu.accel_in_g ? 9.80665 : 1.0;
  c.emit.mode = use_requests ? "request" : "overlap";
  const SE3 & a = sc.T_base_lidar;
  const SE3 & b = sc.T_lidar_imu;
  auto vec7 = [](const SE3 & T) {
      const Eigen::Quaterniond & q = T.unit_quaternion();
      return std::vector<double>{T.translation().x(), T.translation().y(), T.translation().z(),
        q.x(), q.y(), q.z(), q.w()};
    };
  c.extrinsics.source = "params";
  c.extrinsics.base_lidar = vec7(a);
  c.extrinsics.lidar_imu = vec7(b);
  // synthetic robot body: 0.7 x 0.4 x 0.25 around base
  c.self_filter.body_box = {-0.40, -0.22, -0.15, 0.42, 0.22, 0.20};
  return c;
}

namespace {

class PipelineSink : public Sink
{
public:
  PipelineSink(Pipeline & p, const Simulator & sim, const RunOptions & opt, RunOutput & out)
  : p_(p), sim_(sim), opt_(opt), out_(out) {}

  void onImu(const ImuSample & s, TimeNs arrival) override {p_.addImu(s, arrival);}
  void onJoints(const JointSample & s, TimeNs) override {p_.addJoints(s);}
  void onRequest(TimeNs stamp, TimeNs) override
  {
    if (!opt_.use_requests) {return;}
    p_.addNodeRequest(stamp);
    for (const FactorOut & f : p_.answerRequests()) {recordFactor(f);}   // late answers
  }
  void onScan(const RawScan & scan, TimeNs arrival, TimeNs true_t0) override
  {
    const ScanResult & r = p_.process(scan, arrival);
    // every point of a scan carries the scan's clock offset (stamp - true time)
    const TimeNs offset = scan.t0 - true_t0;
    spans_[scan.t0] = {std::max(r.t_end, scan.t0), offset};
    while (spans_.size() > 4096) {spans_.erase(spans_.begin());}
    RunMetrics & m = out_.metrics;
    ++m.scans;
    if (r.verdict == Verdict::ACCEPT) {++m.accepted;}
    if (r.verdict == Verdict::DEGRADED) {++m.degraded;}
    if (r.verdict == Verdict::REJECT) {++m.rejected;}
    const TimeNs t_true = r.t_ref - offset;
    if (r.verdict == Verdict::ACCEPT || r.verdict == Verdict::DEGRADED) {
      if (have_last_good_) {
        m.max_lost_s = std::max(m.max_lost_s, toSec(t_true - last_good_));
      }
      last_good_ = t_true;
      have_last_good_ = true;
      PoseRecord pr;
      pr.t = t_true;
      pr.chain = r.chain_id;
      pr.T_lo = r.T_odom_base;
      pr.T_gt = sim_.groundTruth(t_true);
      out_.poses.push_back(pr);
    }
    for (const FactorOut & f : r.factors) {recordFactor(f);}
    if (opt_.verbose) {
      double dpos_mm = 0.0, err_mm = 0.0;
      const SE3 gt = sim_.groundTruth(t_true);
      if (r.pose_valid && have_prev_dbg_) {
        dpos_mm = 1000.0 * (prev_dbg_.inverse() * r.T_odom_base).translation().norm();
        const SE3 d_gt = prev_gt_.inverse() * gt;
        const SE3 d_lo = prev_dbg_.inverse() * r.T_odom_base;
        err_mm = 1000.0 * (d_gt.inverse() * d_lo).translation().norm();
      }
      if (r.pose_valid) {prev_dbg_ = r.T_odom_base; prev_gt_ = gt; have_prev_dbg_ = true;}
      std::printf("t=%8.3f %-10s %-8s %-22s reg=%5d inl=%.2f res=%.4f it=%d/%d deg=%d v=%.2f "
        "gate=%.3f fac=%d dpos=%.1fmm err=%.1fmm emin=%.4f t=%.1fms\n",
        toSec(r.t_ref), toString(r.state), toString(r.verdict), toString(r.reason),
        r.num_registration, r.inlier_ratio, r.residual_rms, r.iterations_coarse,
        r.iterations_fine, r.num_degenerate, r.voxel_size, r.gate, r.factors.size,
        dpos_mm, err_mm, r.eigenvalues[0], r.timings.total);
    }
    if (opt_.on_scan) {opt_.on_scan(r);}
  }

private:
  void recordFactor(const FactorOut & f)
  {
    FactorRecord fr;
    fr.f = f;
    const SE3 Ti = sim_.groundTruth(trueTime(f.t_i));
    const SE3 Tj = sim_.groundTruth(trueTime(f.t_j));
    fr.T_gt_ij = Ti.inverse() * Tj;
    fr.nees = factorNees(f, fr.T_gt_ij);
    fr.false_accept = fr.nees > 40.0;
    out_.factors.push_back(fr);
    if (opt_.verbose && fr.false_accept) {
      const Vec6 xi = logRT(fr.T_gt_ij.inverse() * f.T_ij);
      std::printf("  FALSE ACCEPT t_i=%.3f t_j=%.3f nees=%.1f health=%d flags=0x%x err_rot=%.3fdeg "
        "err_t=%.1fmm sig_t=%.1fmm\n", toSec(f.t_i), toSec(f.t_j), fr.nees, f.health, f.flags,
        xi.head<3>().norm() * kRadToDeg, 1000.0 * xi.tail<3>().norm(),
        1000.0 * std::sqrt(f.covariance.block<3, 3>(3, 3).trace()));
    }
  }

  /// Factor times lie inside a processed scan: its clock offset maps them to true time.
  TimeNs trueTime(TimeNs stamp) const
  {
    auto it = spans_.upper_bound(stamp + kTol);
    if (it != spans_.begin()) {
      --it;
      if (stamp <= it->second.first + kTol) {return stamp - it->second.second;}
    }
    return sim_.trueTimeOfLidarStamp(stamp);
  }

  static constexpr TimeNs kTol = 2000000;   // 2 ms (emit.request_tolerance)
  std::map<TimeNs, std::pair<TimeNs, TimeNs>> spans_;   ///< scan t0 -> (t_end, clock offset)
  Pipeline & p_;
  const Simulator & sim_;
  const RunOptions & opt_;
  RunOutput & out_;
  TimeNs last_good_{0};
  bool have_last_good_{false};
  SE3 prev_dbg_, prev_gt_;
  bool have_prev_dbg_{false};
};

}  // namespace

RunOutput runScenario(const Scenario & sc, const Config & cfg, const RunOptions & opt)
{
  RunOutput out;
  Simulator sim(sc);
  std::shared_ptr<const KinematicModel> kin;
  if (opt.use_kinematics) {kin = std::make_shared<KinematicModel>(sim.robot().model());}
  Pipeline pipeline(cfg, extrinsicsFromConfig(cfg), kin);
  PipelineSink sink(pipeline, sim, opt, out);
  sim.run(sink, opt.t_max);

  pipeline.updatePercentiles();
  out.diagnostics = pipeline.diagnostics();
  RunMetrics & m = out.metrics;
  m.factors = static_cast<int>(out.factors.size());
  double sum = 0.0, sum3 = 0.0;
  for (const auto & f : out.factors) {
    sum += f.nees;
    m.false_accepts += f.false_accept ? 1 : 0;
    const Vec6 xi = logRT(f.T_gt_ij.inverse() * f.f.T_ij);
    const Eigen::Matrix3d C = f.f.covariance.block<3, 3>(3, 3);
    sum3 += xi.tail<3>().dot(C.ldlt().solve(xi.tail<3>()));
  }
  if (!out.factors.empty()) {
    m.mean_nees = sum / static_cast<double>(out.factors.size());
    m.nees_trans_mean = sum3 / static_cast<double>(out.factors.size());
  }
  relativeError(out.poses, 10.0, m.rpe_trans_pct, m.rpe_rot_deg_per_m, m.rpe_segments);
  for (std::size_t i = 1; i < out.poses.size(); ++i) {
    m.path_length += (out.poses[i].T_gt.translation() - out.poses[i - 1].T_gt.translation()).norm();
  }
  m.availability = out.diagnostics.availability();
  m.timing_median_ms = out.diagnostics.timing_median_ms;
  m.timing_p99_ms = out.diagnostics.timing_p99_ms;
  m.timing_max_ms = out.diagnostics.timing_max_ms;
  m.chain_breaks = out.diagnostics.chain_breaks;
  return out;
}

namespace {

double residentMb()
{
  long size = 0, resident = 0;
  if (FILE * f = std::fopen("/proc/self/statm", "r")) {
    if (std::fscanf(f, "%ld %ld", &size, &resident) != 2) {resident = 0;}
    std::fclose(f);
  }
  return static_cast<double>(resident) * static_cast<double>(sysconf(_SC_PAGESIZE)) / 1048576.0;
}

/// Feeds one repetition shifted by `offset` (the previous repetitions' total duration).
/// Ground truth of an absolute time = the scenario's GT at (t mod period): the route is a
/// closed loop that starts and ends standing still at the same pose.
class SoakSink : public Sink
{
public:
  SoakSink(Pipeline & p, const Simulator & sim, TimeNs offset, TimeNs period, SoakReport & rep)
  : p_(p), sim_(sim), off_(offset), period_(period), rep_(rep) {}

  void onImu(const ImuSample & s, TimeNs arrival) override
  {
    ImuSample x = s;
    x.t += off_;
    p_.addImu(x, arrival + off_);
  }
  void onJoints(const JointSample & s, TimeNs) override
  {
    JointSample x = s;
    x.t += off_;
    p_.addJoints(x);
  }
  void onRequest(TimeNs stamp, TimeNs) override
  {
    p_.addNodeRequest(stamp + off_);
    for (const FactorOut & f : p_.answerRequests()) {score(f);}
  }
  void onScan(const RawScan & scan, TimeNs arrival, TimeNs) override
  {
    RawScan s = scan;
    s.t0 += off_;
    const ScanResult & r = p_.process(s, arrival + off_);
    RunMetrics & m = rep_.metrics;
    ++m.scans;
    if (r.verdict == Verdict::ACCEPT) {++m.accepted;}
    if (r.verdict == Verdict::DEGRADED) {++m.degraded;}
    if (r.verdict == Verdict::REJECT) {++m.rejected;}
    for (const FactorOut & f : r.factors) {score(f);}
  }

private:
  void score(const FactorOut & f)
  {
    RunMetrics & m = rep_.metrics;
    ++m.factors;
    const SE3 Ti = gt(f.t_i), Tj = gt(f.t_j);
    const double nees = factorNees(f, Ti.inverse() * Tj);
    if (nees > 40.0) {++m.false_accepts;}
    m.mean_nees += (nees - m.mean_nees) / static_cast<double>(m.factors);   // running mean
  }

  SE3 gt(TimeNs t) const {return sim_.groundTruth(((t % period_) + period_) % period_);}

  Pipeline & p_;
  const Simulator & sim_;
  TimeNs off_, period_;
  SoakReport & rep_;
};

}  // namespace

SoakReport runSoak(const Scenario & sc, const Config & cfg, double hours,
  const std::function<void(const SoakReport &)> & progress)
{
  SoakReport rep;
  Simulator sim(sc);
  auto kin = std::make_shared<KinematicModel>(sim.robot().model());
  Pipeline pipeline(cfg, extrinsicsFromConfig(cfg), kin);
  // repetition period: a whole number of scan periods covering the scenario, so the scan grid
  // stays regular across repetitions (the boundary looks like one or two dropped scans)
  const double scan_dur = static_cast<double>(sc.pattern.points_per_scan) / sc.pattern.rate;
  const TimeNs period = fromSec(std::ceil(sc.duration / scan_dur) * scan_dur);
  TimeNs offset = 0;
  while (toSec(offset) < hours * 3600.0) {
    SoakSink sink(pipeline, sim, offset, period, rep);
    sim.run(sink);
    offset += period;
    ++rep.repetitions;
    rep.hours = toSec(offset) / 3600.0;
    rep.rss_mb.push_back(residentMb());
    if (progress) {progress(rep);}
  }
  const std::size_t half = rep.rss_mb.size() / 2;
  double lo = 1e300, hi = 0.0, sum = 0.0;
  for (std::size_t i = half; i < rep.rss_mb.size(); ++i) {
    lo = std::min(lo, rep.rss_mb[i]);
    hi = std::max(hi, rep.rss_mb[i]);
    sum += rep.rss_mb[i];
  }
  const std::size_t n = rep.rss_mb.size() - half;
  rep.rss_spread_pct = (n > 0 && sum > 0.0) ? 100.0 * (hi - lo) / (sum / static_cast<double>(n)) : 0.0;
  pipeline.updatePercentiles();
  rep.diagnostics = pipeline.diagnostics();
  rep.metrics.availability = rep.diagnostics.availability();
  rep.metrics.timing_median_ms = rep.diagnostics.timing_median_ms;
  rep.metrics.timing_p99_ms = rep.diagnostics.timing_p99_ms;
  rep.metrics.timing_max_ms = rep.diagnostics.timing_max_ms;
  rep.metrics.chain_breaks = rep.diagnostics.chain_breaks;
  return rep;
}

}  // namespace lo::synth
