#include <gtest/gtest.h>

#include <limits>

#include <jgl_dwa_local_planner/reference_obstacle_policy.h>

namespace
{

using jgl_dwa_local_planner::referenceObstacleSpeedScale;

TEST(ReferenceObstaclePolicyTest, UsesFiveConfiguredSpeedLevels)
{
  EXPECT_DOUBLE_EQ(1.0, referenceObstacleSpeedScale(1.01, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(1.0, referenceObstacleSpeedScale(1.00, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.75, referenceObstacleSpeedScale(0.95, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.50, referenceObstacleSpeedScale(0.88, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.25, referenceObstacleSpeedScale(0.80, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.25, referenceObstacleSpeedScale(0.71, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.0, referenceObstacleSpeedScale(0.70, 1.0, 0.7));
  EXPECT_DOUBLE_EQ(0.0, referenceObstacleSpeedScale(0.20, 1.0, 0.7));
}

TEST(ReferenceObstaclePolicyTest, FailsSafeForInvalidInputs)
{
  EXPECT_DOUBLE_EQ(
      0.0, referenceObstacleSpeedScale(
               std::numeric_limits<double>::quiet_NaN(), 1.0, 0.5));
  EXPECT_DOUBLE_EQ(0.0, referenceObstacleSpeedScale(0.8, 0.5, 0.5));
  EXPECT_DOUBLE_EQ(0.0, referenceObstacleSpeedScale(0.8, 0.4, 0.5));
  EXPECT_DOUBLE_EQ(0.0, referenceObstacleSpeedScale(0.8, 1.0, -0.1));
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
