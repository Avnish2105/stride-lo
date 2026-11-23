// lo::Pipeline: the whole STRIDE LiDAR odometry for one sensor (docs/DESIGN.md "Pipeline
// stages"). Single-threaded API (the caller serialises inputs; OpenMP is used inside process).
// All memory is allocated in the constructor; process() does not allocate (tested).
//
// Typical use (lo_ros worker thread, lo_tools replay):
//   pipeline.addImu(...), addJoints(...), addGraphOdom(...), addNodeRequest(...)  as they arrive
//   if (pipeline.coverage(scan_end) == Coverage::READY || next scan already arrived)
//     const ScanResult& r = pipeline.process(scan, arrival);
#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "lo_core/config/config.hpp"
#include "lo_core/deskew/bias_estimator.hpp"
#include "lo_core/deskew/deskew.hpp"
#include "lo_core/deskew/imu_buffer.hpp"
#include "lo_core/emit/emitter.hpp"
#include "lo_core/map/local_map.hpp"
#include "lo_core/preprocess/downsample.hpp"
#include "lo_core/preprocess/self_filter.hpp"
#include "lo_core/preprocess/validation.hpp"
#include "lo_core/registration/point_covariance.hpp"
#include "lo_core/registration/registration.hpp"
#include "lo_core/registration/seed.hpp"
#include "lo_core/state/state_machine.hpp"
#include "lo_core/types/outputs.hpp"
#include "lo_core/util/rolling.hpp"

namespace lo {

/// Counters and latest values for /diagnostics (copied out by the caller).
struct Diagnostics
{
  HealthState state{HealthState::INIT};
  std::uint64_t chain_id{0};
  std::uint64_t scans{0}, accepted{0}, degraded{0}, rejected{0}, skipped{0};
  std::array<std::uint64_t, static_cast<std::size_t>(RejectReason::COUNT)> reject_reasons{};
  std::uint64_t factors{0}, anchors{0};
  std::uint64_t requests_received{0}, requests_answered{0}, requests_missed{0};
  std::uint64_t losts{0}, recoveries{0}, chain_breaks{0}, map_rebins{0};
  std::uint64_t dropped_scans{0};          ///< scans missing in the input stream
  std::uint64_t imu_rejected{0}, joints_rejected{0}, graph_rejected{0};
  std::uint64_t graph_gated{0};            ///< scans whose seed ignored the graph (gate / hold-off)
  std::uint64_t deskew_fallbacks{0}, blocked_scans{0}, joints_missing_scans{0};
  std::uint64_t map_points_dropped{0};
  std::uint64_t clock_faults{0};
  double clock_skew{0}, lidar_latency{0}, imu_latency{0}, ptp_offset{0};
  double imu_rate{0};
  std::size_t map_fine{0}, map_coarse{0};
  double voxel{0}, gate{0};
  Vec3 gyro_bias{Vec3::Zero()};
  bool bias_calibrated{false};
  double timing_median_ms{0}, timing_p99_ms{0}, timing_max_ms{0};
  StageTimings last_timings;
  std::uint32_t last_flags{0};
  int last_num_degenerate{0};
  Vec6 last_eigenvalues{Vec6::Zero()};
  double availability() const
  {
    return requests_received ? static_cast<double>(requests_answered) /
           static_cast<double>(requests_received) : 1.0;
  }
};

class Pipeline
{
public:
  enum class Coverage { READY, WAIT };

  Pipeline(const Config & cfg, const Extrinsics & ext,
    std::shared_ptr<const KinematicModel> kinematics = nullptr);

  // ---- inputs (message time) -------------------------------------------------------------
  void addImu(const ImuSample & s, TimeNs arrival);
  void addJoints(const JointSample & s);
  void addGraphOdom(const GraphOdomSample & s);
  void addNodeRequest(TimeNs t);
  void addPtpOffset(TimeNs t, double offset_s);

  /// READY when the IMU covers the scan end (or the IMU is unused/lost); WAIT otherwise. The
  /// caller processes anyway once the next scan has arrived (then the gap is handled as an
  /// IMU gap), so a stalled IMU never stalls the LiDAR.
  Coverage coverage(TimeNs scan_end) const;

  /// Run S0..S9 on one scan. `arrival`: receive time of the scan message.
  const ScanResult & process(const RawScan & scan, TimeNs arrival);

  /// Answer node requests that arrived after the scan containing them was processed (call
  /// after addNodeRequest(); process() does the same). Valid until the next call.
  const FactorList & answerRequests();

  // ---- outputs / state -------------------------------------------------------------------
  HealthState state() const {return sm_.state();}
  std::uint64_t chainId() const {return chain_id_;}
  const Diagnostics & diagnostics() const {return diag_;}
  /// Refresh timing percentiles (call at the diagnostics rate, not per scan).
  void updatePercentiles();
  const Cloud & deskewedCloud() const {return cloud_;}
  const LocalMap & localMap() const {return map_;}
  std::size_t exportMap(std::vector<Vec3> & out, std::size_t max_points) const
  {
    return map_.exportFine(out, max_points);
  }
  const Config & config() const {return cfg_;}
  /// Output odom frame <- internal LO frame (gravity alignment fixed when INIT completes).
  /// exportMap() returns internal-frame points; apply this for display next to the odometry.
  const SE3 & outputAlignment() const {return T_align_;}
  const Extrinsics & extrinsics() const {return ext_;}
  bool justLost() const {return just_lost_;}

private:
  struct Verdicts
  {
    Verdict verdict{Verdict::ACCEPT};
    RejectReason reason{RejectReason::NONE};
    std::uint32_t flags{0};
  };

  void reject(RejectReason reason, bool registered);
  void handleInit(TimeNs t_ref, const SeedResult & seed, double median_range, bool imu_usable);
  void handleTracking(TimeNs t_ref, const SeedResult & seed, double median_range,
    bool imu_usable, bool recovering, std::uint32_t scan_flags);
  /// FrameRecord of the scan being processed (pose at t_ref, covariance, deskew motion).
  FrameRecord & recordFrame(TimeNs t_ref, const SE3 & T, const Mat6 & cov_lidar);
  Verdicts evaluate(const AlignResult & a, const SeedResult & seed, TimeNs t_ref,
    bool imu_usable, bool recovering, std::uint32_t scan_flags);
  void finishInit(TimeNs t_ref, const SE3 & T_current);
  void startChain(TimeNs t_ref);
  void updateMotionModels(const SE3 & T, TimeNs t_ref, const SeedResult & seed);
  void updateBias(const SE3 & T, TimeNs t_ref, bool imu_usable);
  void maybeInsert(const SE3 & T, TimeNs t_ref, Verdict v, int num_degenerate);
  double effectiveGate() const;
  void fillDiagnostics();

  Config cfg_;
  Extrinsics ext_;
  SE3 T_lidar_base_;

  // modules
  ScanValidator validator_;
  ClockHealth clock_;
  ImuBuffer imu_;
  TimeRing<JointSample> joints_;
  GraphOdomBuffer graph_;
  SelfFilter self_;
  Preprocessor pre_;
  Cloud cloud_;
  Deskewer deskew_;
  PointCovariance cov_;
  SeedModel seed_model_;
  LocalMap map_;
  Registration reg_;
  AdaptiveGate gate_;
  BiasEstimator bias_;
  StateMachine sm_;
  Emitter emitter_;
  TimeNs scan_offset_;           ///< scan.time_offset
  TimeNs imu_offset_;            ///< imu.time_offset
  RollingWindow residuals_;
  RollingWindow timings_;

  // pose / chain state
  bool has_prev_{false};
  TimeNs t_prev_{0};
  SE3 T_prev_;                   ///< odom <- lidar at t_prev_ (last accepted scan)
  SE3 T_last_insert_;
  bool has_insert_{false};
  LoVelocity lo_vel_;
  static constexpr double kVelBaseline = 0.05;   ///< min LO velocity baseline [s]
  bool has_vel_anchor_{false};
  TimeNs vel_anchor_t_{0};
  SE3 vel_anchor_T_;
  static constexpr double kGateBaseline = 0.4;   ///< graph-gate comparison window [s]
  bool has_gate_anchor_{false};
  TimeNs gate_anchor_t_{0};
  SE3 gate_anchor_T_;
  bool init_map_built_{false};
  bool warm_init_{false};        ///< next INIT is a warm restart (DECISIONS D3)
  SE3 T_init_first_;             ///< pose the cold-INIT map was built at
  SE3 T_align_;                  ///< output odom <- internal LO frame (gravity alignment at INIT)
  std::uint64_t chain_id_{0};
  TimeNs last_stale_check_{0};
  bool just_lost_{false};
  bool gyro_available_for_check_{false};

  static constexpr double kAnchorVariance = 1e-4;   ///< INIT frame (anchor only, never a factor end)
  Vec3 v_deskew_{Vec3::Zero()};  ///< velocity the current scan was deskewed with
  ScanResult res_;
  FactorList late_;
  Diagnostics diag_;
};

}  // namespace lo
