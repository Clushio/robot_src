#include <gtest/gtest.h>

#include <x2bot_teleop/goal_advance_policy.h>

#include <vector>

namespace policy = x2bot_teleop::goal_advance_policy;

TEST(GoalAdvancePolicyTest, IdentifiesOnlyStartAndFinalControllerHandoffs)
{
  EXPECT_FALSE(policy::IsPreciseControllerHandoff(0, 6));
  EXPECT_TRUE(policy::IsPreciseControllerHandoff(1, 6));
  EXPECT_FALSE(policy::IsPreciseControllerHandoff(2, 6));
  EXPECT_FALSE(policy::IsPreciseControllerHandoff(3, 6));
  EXPECT_TRUE(policy::IsPreciseControllerHandoff(4, 6));
  EXPECT_FALSE(policy::IsPreciseControllerHandoff(5, 6));
  EXPECT_FALSE(policy::IsPreciseControllerHandoff(1, 3));
}

TEST(GoalAdvancePolicyTest, NormalBsplinePassThroughRemainsImmediate)
{
  EXPECT_TRUE(policy::ReferencePassedAllowsAdvance(false, true, 0.25, 0.08));
  EXPECT_FALSE(policy::DistanceAllowsAdvance(false, false, true, 0.20,
                                             0.20, 0.08));
  EXPECT_TRUE(policy::DistanceAllowsAdvance(false, false, false, 0.20,
                                            0.20, 0.08));
}

TEST(GoalAdvancePolicyTest, BsplineToLegacyWaitsForEightCentimetres)
{
  const std::vector<double> distances = {0.25, 0.20, 0.081, 0.08};
  for (std::size_t i = 0; i + 1 < distances.size(); ++i)
  {
    EXPECT_FALSE(policy::ReferencePassedAllowsAdvance(
        true, true, distances[i], 0.08));
    EXPECT_FALSE(policy::DistanceAllowsAdvance(
        false, true, true, distances[i], 0.20, 0.08));
  }
  EXPECT_TRUE(policy::ReferencePassedAllowsAdvance(
      true, true, distances.back(), 0.08));
  EXPECT_TRUE(policy::DistanceAllowsAdvance(
      false, true, true, distances.back(), 0.20, 0.08));
}

TEST(GoalAdvancePolicyTest, LegacyToBsplineUsesSameEightCentimetreGate)
{
  EXPECT_FALSE(policy::DistanceAllowsAdvance(false, true, false, 0.20,
                                             0.20, 0.08));
  EXPECT_FALSE(policy::DistanceAllowsAdvance(false, true, false, 0.081,
                                             0.20, 0.08));
  EXPECT_TRUE(policy::DistanceAllowsAdvance(false, true, false, 0.08,
                                            0.20, 0.08));
}

TEST(GoalAdvancePolicyTest, ActiveBsplineCannotBeBypassedByRunNavFallback)
{
  EXPECT_FALSE(policy::DistanceAllowsAdvance(false, false, true, 0.20,
                                             0.20, 0.08));
  EXPECT_FALSE(policy::DistanceAllowsAdvance(false, false, true, 0.15,
                                             0.20, 0.08));
  EXPECT_TRUE(policy::ReferencePassedAllowsAdvance(false, true, 0.15, 0.08));
}

TEST(GoalAdvancePolicyTest, EarlyActionSuccessAndPauseCannotBypassHandoff)
{
  EXPECT_TRUE(policy::PassThroughActionSuccessNeedsValidation(false, true,
                                                              false));
  EXPECT_FALSE(policy::ValidatedActionSuccessAllowsAdvance(true, 0.081, 0.08));
  EXPECT_TRUE(policy::ValidatedActionSuccessAllowsAdvance(true, 0.08, 0.08));
  EXPECT_FALSE(policy::ValidatedActionSuccessAllowsAdvance(false, 0.0, 0.08));

  // Pause re-entry consumes the same REFERENCE_PASSED gate.
  EXPECT_FALSE(policy::ReferencePassedAllowsAdvance(true, true, 0.081, 0.08));
  EXPECT_TRUE(policy::ReferencePassedAllowsAdvance(true, true, 0.08, 0.08));
}

TEST(GoalAdvancePolicyTest, FinalGoalIsNeverCompletedByPassThroughDistance)
{
  EXPECT_FALSE(policy::DistanceAllowsAdvance(true, false, false, 0.0,
                                             0.20, 0.08));
  EXPECT_FALSE(policy::PassThroughActionSuccessNeedsValidation(true, true,
                                                               true));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
