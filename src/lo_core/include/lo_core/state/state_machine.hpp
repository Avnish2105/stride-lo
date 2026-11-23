// Health state machine (docs/DESIGN.md: 5 states, 8 transitions) + DECISIONS D3 (warm re-INIT).
//   INIT --ready--> TRACKING --degenerate--> DEGRADED --clear 1 s--> TRACKING
//   TRACKING/DEGRADED --5 rejects--> LOST --next scan--> RECOVERING
//   RECOVERING --match found--> TRACKING ;  RECOVERING --no match 2 s (reset map)--> INIT
#pragma once

#include <cstdint>

#include "lo_core/config/config.hpp"
#include "lo_core/types/flags.hpp"

namespace lo {

class StateMachine
{
public:
  explicit StateMachine(const Config & cfg)
  : max_rejects_(cfg.lost.consecutive_rejects),
    clear_time_(fromSec(cfg.degraded.clear_time)),
    recovery_timeout_(fromSec(cfg.recovery.timeout)) {}

  HealthState state() const {return state_;}
  int consecutiveRejects() const {return rejects_;}

  /// INIT -> TRACKING (ready) or RECOVERING -> TRACKING (match found).
  void toTracking(TimeNs)
  {
    if (state_ == HealthState::RECOVERING) {++recoveries_;}
    state_ = HealthState::TRACKING;
    rejects_ = 0;
  }

  /// Verdict of a registered scan in TRACKING / DEGRADED.
  void onVerdict(Verdict v, TimeNs t)
  {
    if (state_ != HealthState::TRACKING && state_ != HealthState::DEGRADED) {return;}
    if (v == Verdict::REJECT) {
      if (++rejects_ >= max_rejects_) {
        state_ = HealthState::LOST;
        lost_since_ = t;
        ++losts_;
      }
      return;
    }
    rejects_ = 0;
    if (v == Verdict::DEGRADED) {
      state_ = HealthState::DEGRADED;
      last_degraded_ = t;
    } else if (state_ == HealthState::DEGRADED && t - last_degraded_ >= clear_time_) {
      state_ = HealthState::TRACKING;
    }
  }

  /// A scan arrives while LOST: -> RECOVERING.
  void onScanWhileLost(TimeNs)
  {
    if (state_ == HealthState::LOST) {
      state_ = HealthState::RECOVERING;
      attempted_ = false;
    }
  }

  /// A match against the kept map was attempted at t. The 2 s recovery window ("no match 2 s")
  /// starts at the first attempt: scans dropped before matching (clock fault, too few points)
  /// do not use it up (DECISIONS I13).
  void onRecoveryAttempt(TimeNs t)
  {
    if (state_ == HealthState::RECOVERING && !attempted_) {
      attempted_ = true;
      recovering_since_ = t;
    }
  }

  bool recoveryExpired(TimeNs t) const
  {
    return state_ == HealthState::RECOVERING && attempted_ &&
           t - recovering_since_ >= recovery_timeout_;
  }

  /// RECOVERING -> INIT (map reset, chain break).
  void toInit()
  {
    state_ = HealthState::INIT;
    rejects_ = 0;
    ++chain_breaks_;
  }

  std::uint64_t losts() const {return losts_;}
  std::uint64_t recoveries() const {return recoveries_;}
  std::uint64_t chainBreaks() const {return chain_breaks_;}

private:
  int max_rejects_;
  TimeNs clear_time_;
  TimeNs recovery_timeout_;
  HealthState state_{HealthState::INIT};
  int rejects_{0};
  TimeNs lost_since_{0};
  TimeNs recovering_since_{0};
  bool attempted_{false};
  TimeNs last_degraded_{0};
  std::uint64_t losts_{0}, recoveries_{0}, chain_breaks_{0};
};

}  // namespace lo
