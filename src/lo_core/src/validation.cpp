#include "lo_core/preprocess/validation.hpp"

#include <cmath>

namespace lo {

ScanCheck ScanValidator::check(const RawScan & scan)
{
  ScanCheck out;
  const TimeNs period = fromSec(scan_.period);
  const TimeNs tolerance = fromSec(scan_.period_tolerance);

  // Stamp monotonicity and period. An integer multiple of the period (within tolerance) is a
  // dropped scan, not a clock fault; anything else is a jump (DESIGN "Clock jump or PTP lost").
  if (has_last_) {
    const TimeNs dt = scan.t0 - last_t0_;
    if (dt <= 0) {
      out.reason = RejectReason::STAMP_BACKWARDS;
      out.clock_fault = true;
      last_t0_ = scan.t0;   // re-anchor so the stream can recover after the jump
      return out;
    }
    // one period within the doc's tolerance; n >= 2 periods within the (tighter) drop
    // tolerance = scans lost in transport; anything else = the clock jumped
    const double n = std::max(1.0, std::round(static_cast<double>(dt) / static_cast<double>(period)));
    const TimeNs err = dt - static_cast<TimeNs>(n) * period;
    const TimeNs tol = (n < 1.5) ? tolerance : std::min(tolerance, fromSec(scan_.drop_tolerance));
    if (std::llabs(err) > tol) {
      out.reason = RejectReason::PERIOD_JUMP;
      out.clock_fault = true;
      last_t0_ = scan.t0;
      return out;
    }
    out.dropped_scans = static_cast<int>(n) - 1;
  }
  has_last_ = true;
  last_t0_ = scan.t0;

  // Valid points and their time range.
  const std::uint32_t max_offset = static_cast<std::uint32_t>(fromSec(scan_.max_point_offset));
  std::uint32_t latest = 0;
  int valid_count = 0;
  bool out_of_range = false;
  for (std::size_t i = 0; i < scan.size; ++i) {
    const RawPoint & p = scan.points[i];
    if (!valid(p)) {continue;}
    if (p.offset_ns > max_offset) {
      out_of_range = true;
      continue;
    }
    ++valid_count;
    if (p.offset_ns > latest) {latest = p.offset_ns;}
  }
  out.num_valid = valid_count;
  out.t_end = scan.t0 + static_cast<TimeNs>(latest);
  if (out_of_range) {
    out.reason = RejectReason::POINT_TIME_RANGE;
  } else if (valid_count < scan_.min_valid_points) {
    out.reason = RejectReason::TOO_FEW_POINTS;
  }
  return out;
}

}  // namespace lo
