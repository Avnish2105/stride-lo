// S0: input validation and time (docs/DESIGN.md S0; DECISIONS I1).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "lo_core/config/config.hpp"
#include "lo_core/types/flags.hpp"
#include "lo_core/types/points.hpp"

namespace lo {

/// Clock health from stamps vs arrival times (no ROS topic carries the PTP offset).
///  * Per stream (LiDAR, IMU) a rolling minimum of (arrival - stamp) over `clock.window`
///    estimates transport latency + clock offset.
///  * Skew = change of (lidar latency - imu latency) since the baseline (set when INIT
///    completes). A PTP/clock problem on one stream shows up here; transport jitter does not
///    (rolling minimum).
///  * An optional external PTP offset (e.g. from a ptp4l monitor) is checked directly.
///  * Any violation, or a fault reported by the scan validator, makes the clock unstable until
///    `clock.stable_time` has passed without a new fault.
/// All times are message/arrival stamps: deterministic on replay (arrival = bag receive time).
class ClockHealth
{
public:
  explicit ClockHealth(const Config::Clock & cfg) : cfg_(cfg) {}

  void noteLidar(TimeNs stamp, TimeNs arrival) {lidar_.add(stamp, arrival, window());}
  void noteImu(TimeNs stamp, TimeNs arrival) {imu_.add(stamp, arrival, window());}
  void notePtpOffset(TimeNs t, double offset_s)
  {
    ptp_offset_ = offset_s;
    has_ptp_ = true;
    if (std::abs(offset_s) > cfg_.max_offset) {reportFault(t);}
  }

  /// Fix the current latency difference as the reference (call when INIT completes).
  void setBaseline()
  {
    if (lidar_.valid() && imu_.valid()) {
      baseline_ = lidar_.latency() - imu_.latency();
      has_baseline_ = true;
    }
  }

  void reportFault(TimeNs t)
  {
    last_fault_ = t;
    ++faults_;
  }

  /// Evaluate the skew rule at time t (call once per scan); returns true if healthy now.
  bool evaluate(TimeNs t)
  {
    if (cfg_.check_skew && has_baseline_ && lidar_.valid() && imu_.valid()) {
      skew_ = (lidar_.latency() - imu_.latency()) - baseline_;
      if (std::abs(skew_) > cfg_.max_skew) {reportFault(t);}
    }
    return stable(t);
  }

  bool stable(TimeNs t) const
  {
    return last_fault_ == kNever || t - last_fault_ >= fromSec(cfg_.stable_time);
  }

  double skew() const {return skew_;}
  double ptpOffset() const {return has_ptp_ ? ptp_offset_ : 0.0;}
  double lidarLatency() const {return lidar_.valid() ? lidar_.latency() : 0.0;}
  double imuLatency() const {return imu_.valid() ? imu_.latency() : 0.0;}
  std::uint64_t faults() const {return faults_;}
  void resetBaseline() {has_baseline_ = false; skew_ = 0.0;}

private:
  static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::min();

  TimeNs window() const {return fromSec(cfg_.window * 0.5);}

  /// Rolling minimum over two half-window blocks (O(1), no allocation).
  struct Stream
  {
    TimeNs block_start{kNever};
    double cur_min{std::numeric_limits<double>::infinity()};
    double prev_min{std::numeric_limits<double>::infinity()};

    void add(TimeNs stamp, TimeNs arrival, TimeNs half_window)
    {
      const double lat = toSec(arrival - stamp);
      if (block_start == kNever || arrival - block_start >= half_window) {
        prev_min = cur_min;
        cur_min = lat;
        block_start = arrival;
      } else if (lat < cur_min) {
        cur_min = lat;
      }
    }
    bool valid() const {return std::isfinite(cur_min);}
    double latency() const {return std::min(cur_min, prev_min);}
  };

  Config::Clock cfg_;
  Stream lidar_;
  Stream imu_;
  double baseline_{0.0};
  bool has_baseline_{false};
  double skew_{0.0};
  double ptp_offset_{0.0};
  bool has_ptp_{false};
  TimeNs last_fault_{kNever};
  std::uint64_t faults_{0};
};

/// Result of S0 for one scan.
struct ScanCheck
{
  RejectReason reason{RejectReason::NONE};
  int num_valid{0};
  int dropped_scans{0};        ///< scans missing before this one (integer period multiples)
  TimeNs t_end{0};             ///< time of the latest valid point
  bool clock_fault{false};     ///< this scan's stamps indicate a clock jump
};

/// Scan-level checks: stamp monotonicity, period, valid point count, point time range.
class ScanValidator
{
public:
  explicit ScanValidator(const Config & cfg) : scan_(cfg.scan) {}

  ScanCheck check(const RawScan & scan);
  void reset() {has_last_ = false;}
  TimeNs lastStamp() const {return last_t0_;}

  /// A point counts as valid if finite and not the driver's (0,0,0) no-return.
  static bool valid(const RawPoint & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           (p.x != 0.f || p.y != 0.f || p.z != 0.f);
  }

private:
  Config::Scan scan_;
  bool has_last_{false};
  TimeNs last_t0_{0};
};

}  // namespace lo
