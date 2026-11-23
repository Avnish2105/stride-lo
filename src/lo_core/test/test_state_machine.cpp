#include <gtest/gtest.h>

#include "lo_core/state/state_machine.hpp"

using namespace lo;

TEST(StateMachine, AllDesignTransitions)
{
  Config cfg;
  StateMachine sm(cfg);
  EXPECT_EQ(sm.state(), HealthState::INIT);
  sm.toTracking(0);                                         // INIT -ready-> TRACKING
  EXPECT_EQ(sm.state(), HealthState::TRACKING);
  sm.onVerdict(Verdict::DEGRADED, fromSec(1.0));            // -degenerate-> DEGRADED
  EXPECT_EQ(sm.state(), HealthState::DEGRADED);
  sm.onVerdict(Verdict::ACCEPT, fromSec(1.5));
  EXPECT_EQ(sm.state(), HealthState::DEGRADED);             // not clear for 1 s yet
  sm.onVerdict(Verdict::ACCEPT, fromSec(2.1));              // -clear 1 s-> TRACKING
  EXPECT_EQ(sm.state(), HealthState::TRACKING);
  for (int i = 0; i < 4; ++i) {sm.onVerdict(Verdict::REJECT, fromSec(3.0 + 0.1 * i));}
  EXPECT_EQ(sm.state(), HealthState::TRACKING);
  sm.onVerdict(Verdict::ACCEPT, fromSec(3.5));              // resets the reject count
  for (int i = 0; i < 5; ++i) {sm.onVerdict(Verdict::REJECT, fromSec(4.0 + 0.1 * i));}
  EXPECT_EQ(sm.state(), HealthState::LOST);                 // -5 rejects-> LOST
  sm.onScanWhileLost(fromSec(4.5));                         // -next scan-> RECOVERING
  EXPECT_EQ(sm.state(), HealthState::RECOVERING);
  EXPECT_FALSE(sm.recoveryExpired(fromSec(10.0)));          // no match attempted yet
  sm.onRecoveryAttempt(fromSec(10.0));
  EXPECT_FALSE(sm.recoveryExpired(fromSec(11.9)));
  EXPECT_TRUE(sm.recoveryExpired(fromSec(12.0)));           // -no match 2 s-> INIT
  sm.toInit();
  EXPECT_EQ(sm.state(), HealthState::INIT);
  EXPECT_EQ(sm.chainBreaks(), 1u);
  sm.toTracking(fromSec(13.0));
  for (int i = 0; i < 5; ++i) {sm.onVerdict(Verdict::REJECT, fromSec(14.0 + 0.1 * i));}
  sm.onScanWhileLost(fromSec(14.6));
  sm.onRecoveryAttempt(fromSec(14.6));
  sm.toTracking(fromSec(14.7));                             // -match found-> TRACKING
  EXPECT_EQ(sm.state(), HealthState::TRACKING);
  EXPECT_EQ(sm.recoveries(), 1u);
}
