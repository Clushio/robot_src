#include <gtest/gtest.h>

#include <x2bot_teleop/terminal_goal_policy.h>

namespace
{

TEST(TerminalGoalPolicy, LocksOnCapturedStateAndIgnoresTrackingTimeout)
{
  x2bot_teleop::TerminalGoalPolicy policy(60.0, 0.10);
  EXPECT_FALSE(policy.update(x2bot_teleop::TERMINAL_TRACKING, true, 0.0, 100.0).locked);
  EXPECT_TRUE(policy.update(x2bot_teleop::TERMINAL_POSITION_CAPTURED, true, 0.0, 101.0).locked);
  EXPECT_FALSE(policy.update(x2bot_teleop::TERMINAL_TRACKING, true, 0.0, 200.0).error_activated);
  EXPECT_TRUE(policy.locked());
}

TEST(TerminalGoalPolicy, ReportsOnceThenRecoversOnMeasuredYawProgress)
{
  x2bot_teleop::TerminalGoalPolicy policy(60.0, 0.10);
  policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 0.0, 10.0);
  EXPECT_FALSE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 0.05, 69.9).error_activated);
  EXPECT_TRUE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 0.05, 70.0).error_activated);
  EXPECT_FALSE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 0.05, 80.0).error_activated);
  const auto recovered = policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 0.11, 81.0);
  EXPECT_TRUE(recovered.error_recovered);
  EXPECT_FALSE(policy.errorActive());
}

TEST(TerminalGoalPolicy, RearmsTimeoutAfterRecoveryAndHandlesYawWrap)
{
  x2bot_teleop::TerminalGoalPolicy policy(30.0, 0.10);
  policy.update(x2bot_teleop::TERMINAL_ROTATING, true, 3.12, 0.0);
  EXPECT_FALSE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, -3.12, 10.0).error_activated);
  EXPECT_TRUE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, -3.12, 30.0).error_activated);
  EXPECT_TRUE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, -3.00, 31.0).error_recovered);
  EXPECT_TRUE(policy.update(x2bot_teleop::TERMINAL_ROTATING, true, -3.00, 61.0).error_activated);
}

TEST(LegacyIntermediateWaypointPolicy, AppliesOnlyToLegacyMiddleGoals)
{
  EXPECT_TRUE(x2bot_teleop::legacyIntermediateWaypointReached(
      x2bot_teleop::LEGACY_FALLBACK, false, 0.05, 0.05));
  EXPECT_FALSE(x2bot_teleop::legacyIntermediateWaypointReached(
      x2bot_teleop::LEGACY_FALLBACK, false, 0.051, 0.05));
  EXPECT_FALSE(x2bot_teleop::legacyIntermediateWaypointReached(
      x2bot_teleop::REFERENCE_TRACKING, false, 0.01, 0.05));
  EXPECT_FALSE(x2bot_teleop::legacyIntermediateWaypointReached(
      x2bot_teleop::LEGACY_FALLBACK, true, 0.01, 0.05));
}

}  // namespace
