// S9: factor emission (docs/DESIGN.md S9, "Between covariance"; DECISIONS D1, I8, I12, I21, I23).
//  * Every accepted scan leaves a FrameRecord: its registered pose at t_ref, its registration
//    covariance and the intra-scan motion model that deskewed it. A graph frame at a time t
//    inside the scan span is T(t_ref) * D(t), D(t) = (dR(t), v (t - t_ref)): exactly the pose
//    that deskewing the scan to t would register, because the two deskewed clouds differ by the
//    rigid transform D(t) (I23).
//  * request mode: requests are answered in time order as soon as their scan has been
//    processed, also when they arrive after it (backend latency). At most one request per scan
//    (I12): two frames of one registration would make a factor of pure deskew motion (gyro).
//  * overlap mode: the newest scan is emitted at its t_ref when the coarse-voxel overlap with
//    the last emitted frame drops below emit.overlap_new_kf (I8). Overlap is measured on coarse
//    voxels (coarse_ratio * v): the Mid-360's non-repetitive pattern makes consecutive scans hit
//    different fine voxels even at standstill.
//  * T_ij = T_i^-1 T_j (base frame). Sigma_ij = Sigma_j for consecutive emissions; if scans were
//    rejected in between (SPANS_GAP) or frame i was not OK (I21):
//    Sigma_ij = Ad(T_ij^-1) Sigma_i Ad(T_ij^-1)^T + Sigma_j.
#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include "lo_core/config/config.hpp"
#include "lo_core/types/outputs.hpp"
#include "lo_core/types/points.hpp"
#include "lo_core/util/key_set.hpp"

namespace lo {

/// One accepted scan as a potential graph frame.
struct FrameRecord
{
  static constexpr int kRotSamples = 33;

  bool valid{false};
  TimeNs t_start{0}, t_end{0}, t_ref{0};
  SE3 T_odom_lidar;                      ///< registered pose at t_ref (internal LO frame)
  Mat6 cov_lidar{Mat6::Identity()};      ///< registration covariance at t_ref, LiDAR frame
  Vec3 v_lidar{Vec3::Zero()};            ///< deskew v0: base-origin velocity, LiDAR axes at t_ref
  Vec3 lever{Vec3::Zero()};              ///< deskew lever arm c (Deskewer::lever)
  std::array<SO3, kRotSamples> R{};      ///< deskew rotation at rel_start + k * step (rel. t_ref)
  double rel_start{0.0}, step{0.0};      ///< [s] relative to t_ref
  /// health, eigenvalues, degenerate directions (LiDAR frame at t_ref), ratios, scan flags
  FactorOut attrs;
  std::uint64_t rejects_before{0};       ///< cumulative rejected scans before this one
  std::uint64_t recoveries{0};           ///< cumulative recoveries up to and including this one
  bool answered{false};                  ///< a request was served from this scan (I12)

  /// Store the motion model the scan was deskewed with; `rotation(dt)` = LiDAR rotation at
  /// t_ref + dt relative to t_ref (Deskewer::rotationAt). Covers [t_s - tol, t_e + tol].
  template<typename RotationAt>
  void setMotion(TimeNs t_s, TimeNs t_e, TimeNs t_r, TimeNs tol, const Vec3 & v, const Vec3 & c,
    RotationAt && rotation)
  {
    t_start = t_s;
    t_end = t_e;
    t_ref = t_r;
    v_lidar = v;
    lever = c;
    rel_start = toSec(t_s - tol - t_r);
    const double rel_end = toSec(t_e + tol - t_r);
    step = std::max(0.0, (rel_end - rel_start) / (kRotSamples - 1));
    for (int k = 0; k < kRotSamples; ++k) {R[static_cast<std::size_t>(k)] = rotation(rel_start + k * step);}
  }

  /// LiDAR frame at t relative to the frame at t_ref, from the stored motion model:
  /// (dR(t), (dR(t) - I) c + v0 (t - t_ref)), the same model the points were deskewed with.
  SE3 motionTo(TimeNs t) const;

  bool contains(TimeNs t, TimeNs tol) const {return t >= t_start - tol && t <= t_end + tol;}
};

class Emitter
{
public:
  Emitter(const Config & cfg, std::size_t max_points, const SE3 & T_base_lidar);

  bool requestMode() const {return request_mode_;}

  // ---- node requests (request mode)
  /// Pending requests older than the newest one by more than emit.request_max_age are missed.
  void addRequest(TimeNs t);
  /// Earliest pending request inside [t_start - tol, t_end + tol] (not consumed): the scan
  /// containing it is deskewed to that time.
  bool firstRequestIn(TimeNs t_start, TimeNs t_end, TimeNs & t_req) const;
  /// Answer, in time order, every pending request whose scan has been processed: from the
  /// record containing it, else it is missed. Appends to `out`; returns the number appended.
  int answerRequests(FactorList & out);

  // ---- scans and chains
  void startChain(std::uint64_t chain_id);
  void noteReject() {++rejects_;}
  void noteRecovery() {++recoveries_;}
  /// A scan up to t_end has been processed (accepted or not): requests up to there are decidable.
  void noteProcessed(TimeNs t_end) {processed_until_ = std::max(processed_until_, t_end);}
  /// Slot for the accepted scan being processed (a ring; overwrites the oldest record).
  FrameRecord & newRecord();
  bool hasAnchor() const {return last_.valid;}
  std::uint64_t chainId() const {return chain_id_;}

  // ---- overlap mode
  /// Coarse-voxel overlap of the cloud (LiDAR frame at T_odom_lidar) with the last emitted frame.
  double overlap(const Cloud & cloud, const SE3 & T_odom_lidar) const;
  /// Emit `rec` at its t_ref and remember its coarse voxels. True if a factor (not only an
  /// anchor) was appended.
  bool emitNewest(const FrameRecord & rec, const Cloud & cloud, double voxel, FactorList & out);

  // ---- statistics
  std::uint64_t requestsReceived() const {return received_;}
  std::uint64_t requestsAnswered() const {return answered_;}
  std::uint64_t requestsMissed() const {return missed_;}
  std::uint64_t requestsDropped() const {return dropped_;}
  std::uint64_t factors() const {return factors_;}
  std::uint64_t anchors() const {return anchors_;}
  std::size_t pendingRequests() const {return num_requests_;}

private:
  struct Frame
  {
    bool valid{false};
    TimeNs t{0};
    SE3 T_odom_base;
    Mat6 cov{Mat6::Identity()};
    std::uint8_t health{health::OK};
    std::uint64_t rejects_before{0};
    std::uint64_t recoveries{0};
  };

  bool emitFrame(const FrameRecord & rec, TimeNs t, bool requested, FactorList & out);
  FrameRecord * findRecord(TimeNs t);
  void rememberFrame(const Cloud & cloud, const SE3 & T_odom_lidar, double voxel);

  bool request_mode_;
  double overlap_threshold_;
  double coarse_ratio_;
  TimeNs tolerance_;
  TimeNs max_age_;
  SE3 T_base_lidar_;
  SE3 T_lidar_base_;
  Mat6 Ad_base_lidar_;
  std::vector<TimeNs> requests_;   ///< sorted ascending, [0, num_requests_)
  std::size_t num_requests_{0};
  std::vector<FrameRecord> records_;
  std::size_t next_record_{0};
  TimeNs processed_until_{std::numeric_limits<TimeNs>::min() / 2};
  Frame last_;
  std::uint64_t rejects_{0}, recoveries_{0};
  std::uint64_t chain_id_{0};
  KeySet last_voxels_;
  double last_voxel_size_{1.0};
  std::uint64_t received_{0}, answered_{0}, missed_{0}, dropped_{0}, factors_{0}, anchors_{0};
};

}  // namespace lo
