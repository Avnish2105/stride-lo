// Enumerations shared by the pipeline, the ROS layer and the tools. Flag bits and health values
// MUST match lo_msgs/msg/LoFactor.msg (checked by lo_ros/test/test_conversions.cpp).
#pragma once

#include <cstdint>

namespace lo {

/// Health state machine (docs/DESIGN.md "Health state machine").
enum class HealthState : std::uint8_t { INIT = 0, TRACKING = 1, DEGRADED = 2, LOST = 3, RECOVERING = 4 };

/// Per-scan verdict. SKIPPED: the scan was not registered (first scan of a chain, INIT warm-up,
/// dropped by input validation before registration counts as REJECT).
enum class Verdict : std::uint8_t { ACCEPT = 0, DEGRADED = 1, REJECT = 2, SKIPPED = 3 };

/// Why a scan was rejected (first failing check). Order = evaluation order.
enum class RejectReason : std::uint8_t {
  NONE = 0,
  STAMP_BACKWARDS,      // S0
  PERIOD_JUMP,          // S0
  TOO_FEW_POINTS,       // S0 (valid points < 3000)
  POINT_TIME_RANGE,     // S0 (offsets outside [0, 120 ms])
  CLOCK_UNSTABLE,       // S0 (PTP/skew > 2 ms, or not yet stable for 2 s after a jump)
  TOO_FEW_REG_POINTS,   // after S2/S4: not enough points to register
  NOT_CONVERGED,        // S7
  LOW_INLIERS,          // S7
  RESIDUAL,             // S7
  GYRO_MISMATCH,        // S7
  IMPLAUSIBLE_MOTION,   // S7 (|v|, |w|)
  SEED_DEVIATION,       // S7
  NUMERICAL,            // S7 (NaN/Inf, Cholesky)
  SOLVER_BLOWUP,        // S7 (single step > 0.5 m / 10 deg)
  COUNT
};

const char * toString(HealthState s);
const char * toString(Verdict v);
const char * toString(RejectReason r);

/// LoFactor.flags bits.
namespace flags {
constexpr std::uint32_t IMU_MISSING = 1u << 0;
constexpr std::uint32_t DESKEW_FALLBACK = 1u << 1;
constexpr std::uint32_t SEED_FALLBACK = 1u << 2;
constexpr std::uint32_t SEED_NO_GRAPH = 1u << 3;
constexpr std::uint32_t SEED_IDENTITY = 1u << 4;
constexpr std::uint32_t SENSOR_BLOCKED = 1u << 5;
constexpr std::uint32_t JOINTS_MISSING = 1u << 6;
constexpr std::uint32_t DEGENERATE = 1u << 7;
constexpr std::uint32_t SPANS_GAP = 1u << 8;
constexpr std::uint32_t AFTER_RECOVERY = 1u << 9;
constexpr std::uint32_t GYRO_GATE_DISABLED = 1u << 10;
constexpr std::uint32_t REQUESTED = 1u << 11;
constexpr std::uint32_t BIAS_UNCALIBRATED = 1u << 12;
/// Flags that inflate the whole factor (health LOW_CONFIDENCE, docs/DECISIONS.md D8).
constexpr std::uint32_t WHOLE_FACTOR_MASK =
  IMU_MISSING | DESKEW_FALLBACK | SENSOR_BLOCKED | SPANS_GAP | AFTER_RECOVERY | GYRO_GATE_DISABLED;
}  // namespace flags

/// LoFactor.health values.
namespace health {
constexpr std::uint8_t OK = 0;
constexpr std::uint8_t DEGRADED = 1;
constexpr std::uint8_t LOW_CONFIDENCE = 2;
}  // namespace health

}  // namespace lo
