#include "lo_core/emit/emitter.hpp"

#include <algorithm>

namespace lo {

SE3 FrameRecord::motionTo(TimeNs t) const
{
  if (t == t_ref) {return SE3();}
  const double dt = toSec(t - t_ref);
  SO3 Rt;
  if (step > 0.0) {
    const double u = std::clamp((dt - rel_start) / step, 0.0, static_cast<double>(kRotSamples - 1));
    const int k = std::min(static_cast<int>(u), kRotSamples - 2);
    const double a = u - k;
    const SO3 & R0 = R[static_cast<std::size_t>(k)];
    const SO3 & R1 = R[static_cast<std::size_t>(k + 1)];
    Rt = R0 * SO3::exp(a * (R0.inverse() * R1).log());
  }
  return SE3(Rt, Rt * lever - lever + v_lidar * dt);
}

Emitter::Emitter(const Config & cfg, std::size_t max_points, const SE3 & T_base_lidar)
: request_mode_(cfg.emit.mode == "request"),
  overlap_threshold_(cfg.emit.overlap_new_kf),
  coarse_ratio_(cfg.voxel.coarse_ratio),
  tolerance_(fromSec(cfg.emit.request_tolerance)),
  max_age_(fromSec(cfg.emit.request_max_age)),
  T_base_lidar_(T_base_lidar),
  T_lidar_base_(T_base_lidar.inverse()),
  Ad_base_lidar_(adjointRT(T_base_lidar)),
  requests_(static_cast<std::size_t>(cfg.limits.request_buffer)),
  records_(static_cast<std::size_t>(cfg.limits.frame_records)),
  last_voxels_(max_points)
{
}

void Emitter::addRequest(TimeNs t)
{
  ++received_;
  // keep sorted, ignore duplicates; when full drop the oldest
  std::size_t pos = static_cast<std::size_t>(
    std::lower_bound(requests_.begin(), requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_), t) -
    requests_.begin());
  if (pos < num_requests_ && requests_[pos] == t) {
    --received_;   // duplicate of a pending request
    return;
  }
  if (num_requests_ == requests_.size()) {
    std::copy(requests_.begin() + 1, requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_), requests_.begin());
    --num_requests_;
    ++dropped_;
    if (pos > 0) {--pos;}
  }
  std::copy_backward(requests_.begin() + static_cast<std::ptrdiff_t>(pos),
    requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_),
    requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_ + 1));
  requests_[pos] = t;
  ++num_requests_;
  // expire requests far behind the newest one (the backend has moved on)
  const TimeNs newest = requests_[num_requests_ - 1];
  std::size_t first = 0;
  while (first < num_requests_ && requests_[first] < newest - max_age_) {
    ++first;
    ++missed_;
  }
  if (first > 0) {
    std::copy(requests_.begin() + static_cast<std::ptrdiff_t>(first),
      requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_), requests_.begin());
    num_requests_ -= first;
  }
}

bool Emitter::firstRequestIn(TimeNs t_start, TimeNs t_end, TimeNs & t_req) const
{
  for (std::size_t i = 0; i < num_requests_; ++i) {
    const TimeNs t = requests_[i];
    if (t < t_start - tolerance_) {continue;}
    if (t > t_end + tolerance_) {break;}
    if (last_.valid && t <= last_.t) {continue;}
    t_req = t;
    return true;
  }
  return false;
}

FrameRecord * Emitter::findRecord(TimeNs t)
{
  FrameRecord * best = nullptr;
  TimeNs best_dist = 0;
  for (auto & r : records_) {
    if (!r.valid || !r.contains(t, tolerance_)) {continue;}
    const TimeNs dist = t < r.t_start ? r.t_start - t : (t > r.t_end ? t - r.t_end : 0);
    if (!best || dist < best_dist) {
      best = &r;
      best_dist = dist;
    }
  }
  return best;
}

int Emitter::answerRequests(FactorList & out)
{
  if (!request_mode_) {return 0;}
  const int before = out.size;
  std::size_t first = 0;
  while (first < num_requests_) {
    const TimeNs t = requests_[first];
    if (t > processed_until_ + tolerance_) {break;}   // its scan has not been processed yet
    if (out.full()) {break;}                         // keep for the next call
    ++first;
    if (last_.valid && t <= last_.t) {++missed_; continue;}   // behind the chain head
    FrameRecord * r = findRecord(t);
    if (!r || r->answered) {++missed_; continue;}   // rejected scan, or a 2nd request (I12)
    emitFrame(*r, t, true, out);
    r->answered = true;
    ++answered_;
  }
  std::copy(requests_.begin() + static_cast<std::ptrdiff_t>(first),
    requests_.begin() + static_cast<std::ptrdiff_t>(num_requests_), requests_.begin());
  num_requests_ -= first;
  return out.size - before;
}

void Emitter::startChain(std::uint64_t chain_id)
{
  chain_id_ = chain_id;
  last_ = Frame{};
  for (auto & r : records_) {r.valid = false;}
  last_voxels_.clear();
}

FrameRecord & Emitter::newRecord()
{
  FrameRecord & r = records_[next_record_];
  next_record_ = (next_record_ + 1) % records_.size();
  r.valid = true;
  r.answered = false;
  r.attrs = FactorOut{};
  r.rejects_before = rejects_;
  r.recoveries = recoveries_;
  return r;
}

void Emitter::rememberFrame(const Cloud & cloud, const SE3 & T_odom_lidar, double voxel)
{
  last_voxels_.clear();
  last_voxel_size_ = coarse_ratio_ * voxel;
  const double inv = 1.0 / last_voxel_size_;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const Vec3 p = T_odom_lidar * cloud.points[i].head<3>();
    const Eigen::Vector3i c = voxelOf(p, inv);
    last_voxels_.insert(packVoxel(c.x(), c.y(), c.z()));
  }
}

double Emitter::overlap(const Cloud & cloud, const SE3 & T_odom_lidar) const
{
  if (!last_.valid || cloud.size() == 0) {return 0.0;}
  const double inv = 1.0 / last_voxel_size_;
  std::size_t hit = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const Vec3 p = T_odom_lidar * cloud.points[i].head<3>();
    const Eigen::Vector3i c = voxelOf(p, inv);
    if (last_voxels_.contains(packVoxel(c.x(), c.y(), c.z()))) {++hit;}
  }
  return static_cast<double>(hit) / static_cast<double>(cloud.size());
}

bool Emitter::emitNewest(const FrameRecord & rec, const Cloud & cloud, double voxel, FactorList & out)
{
  if (out.full()) {return false;}
  const bool produced = emitFrame(rec, rec.t_ref, false, out);
  rememberFrame(cloud, rec.T_odom_lidar, voxel);
  return produced;
}

bool Emitter::emitFrame(const FrameRecord & rec, TimeNs t, bool requested, FactorList & out)
{
  // frame j at t: registered pose moved along the scan's own deskew model (I23)
  const SE3 D = rec.motionTo(t);
  const SE3 T_odom_base = rec.T_odom_lidar * D * T_lidar_base_;
  const Mat6 Ad_shift = adjointRT(D.inverse());
  const Mat6 cov_lidar = Ad_shift * rec.cov_lidar * Ad_shift.transpose();
  const Mat6 cov_base = Ad_base_lidar_ * cov_lidar * Ad_base_lidar_.transpose();

  FactorOut f = rec.attrs;
  const Mat6 Ad_dir = Ad_base_lidar_ * Ad_shift;
  for (int k = 0; k < f.num_degenerate; ++k) {
    const Vec6 db = Ad_dir * rec.attrs.degenerate_directions[static_cast<std::size_t>(k)];
    const double n = db.norm();
    f.degenerate_directions[static_cast<std::size_t>(k)] = n > 0.0 ? Vec6(db / n) : db;
  }
  if (requested) {f.flags |= flags::REQUESTED;}
  f.chain_id = chain_id_;
  f.t_j = t;

  bool produced = false;
  if (last_.valid) {
    f.t_i = last_.t;
    f.T_ij = last_.T_odom_base.inverse() * T_odom_base;
    const bool gap = rec.rejects_before > last_.rejects_before;
    if (gap || last_.health != health::OK) {
      f.covariance = transportCovariance(f.T_ij.inverse(), last_.cov) + cov_base;
      if (gap) {f.flags |= flags::SPANS_GAP;}
    } else {
      f.covariance = cov_base;
    }
    if (rec.recoveries > last_.recoveries) {f.flags |= flags::AFTER_RECOVERY;}
    f.covariance = 0.5 * (f.covariance + f.covariance.transpose());
  }
  if (f.flags & flags::WHOLE_FACTOR_MASK) {f.health = health::LOW_CONFIDENCE;}
  if (last_.valid) {
    out.push(f);
    produced = true;
    ++factors_;
  } else {
    ++anchors_;
  }
  last_.valid = true;
  last_.t = t;
  last_.T_odom_base = T_odom_base;
  last_.cov = cov_base;
  last_.health = f.health;
  last_.rejects_before = rec.rejects_before;
  last_.recoveries = rec.recoveries;
  return produced;
}

}  // namespace lo
