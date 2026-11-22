// Message time in nanoseconds (ROS stamps; sim time in simulation). lo_core never reads a clock
// for decisions: every time-based rule uses these stamps (deterministic replay).
#pragma once

#include <cmath>
#include <cstdint>

namespace lo {

using TimeNs = std::int64_t;

constexpr TimeNs kNsPerSec = 1000000000LL;

inline double toSec(TimeNs t) {return 1e-9 * static_cast<double>(t);}
inline TimeNs fromSec(double s) {return static_cast<TimeNs>(std::llround(s * 1e9));}

}  // namespace lo
