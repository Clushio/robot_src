#include <gtest/gtest.h>

#include <cmath>

#include <x2bot_teleop/hybrid_astar.h>

namespace
{

bool boundedFree(double x, double y, double)
{
  return x >= -2.0 && x <= 4.0 && y >= -2.0 && y <= 2.0;
}

TEST(HybridAStar, PlansForwardStraightConnector)
{
  x2bot_teleop::HybridAStar planner;
  x2bot_teleop::HybridPose start;
  start.yaw = 0.0;
  x2bot_teleop::HybridPose goal;
  goal.x = 1.5;
  goal.yaw = 0.0;
  const auto result = planner.plan(start, goal, boundedFree);
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GE(result.poses.size(), 2U);
  EXPECT_NEAR(result.poses.front().x, start.x, 1e-9);
  EXPECT_NEAR(result.poses.back().x, goal.x, 1e-9);
  for (std::size_t i = 1; i + 1 < result.poses.size(); ++i)
  {
    const double dx = result.poses[i].x - result.poses[i - 1].x;
    const double dy = result.poses[i].y - result.poses[i - 1].y;
    EXPECT_GT(dx * std::cos(result.poses[i - 1].yaw) +
              dy * std::sin(result.poses[i - 1].yaw), 0.0);
  }
}

TEST(HybridAStar, RoutesAroundBlockedStraightLine)
{
  x2bot_teleop::HybridAStarOptions options;
  options.max_iterations = 100000;
  options.search_margin = 2.0;
  x2bot_teleop::HybridAStar planner(options);
  x2bot_teleop::HybridPose start;
  start.x = -1.0;
  x2bot_teleop::HybridPose goal;
  goal.x = 1.0;
  const auto obstacle = [](double x, double y, double) {
    if (x < -2.5 || x > 2.5 || y < -2.0 || y > 2.0) return false;
    return !(x > -0.25 && x < 0.25 && y > -0.65 && y < 0.65);
  };
  const auto result = planner.plan(start, goal, obstacle);
  ASSERT_TRUE(result.success) << result.reason;
  bool left_centerline = false;
  for (const auto &pose : result.poses)
  {
    left_centerline = left_centerline || std::fabs(pose.y) > 0.65;
  }
  EXPECT_TRUE(left_centerline);
}

TEST(HybridAStar, RejectsBlockedGoal)
{
  x2bot_teleop::HybridAStar planner;
  x2bot_teleop::HybridPose start;
  x2bot_teleop::HybridPose goal;
  goal.x = 1.0;
  const auto result = planner.plan(
      start, goal, [](double x, double, double) { return x < 0.9; });
  EXPECT_FALSE(result.success);
  EXPECT_EQ("start or goal footprint is blocked", result.reason);
}

TEST(HybridAStar, GuidePointReductionKeepsEndsAndTurns)
{
  std::vector<x2bot_teleop::HybridPose> path(7);
  for (int i = 0; i < 4; ++i) path[i].x = 0.1 * i;
  path[4].x = 0.3; path[4].y = 0.1;
  path[5].x = 0.3; path[5].y = 0.2;
  path[6].x = 0.3; path[6].y = 0.3;
  const auto guides = x2bot_teleop::HybridAStar::guidePoints(path, 0.4, 0.05);
  ASSERT_GE(guides.size(), 3U);
  EXPECT_DOUBLE_EQ(path.front().x, guides.front().x);
  EXPECT_DOUBLE_EQ(path.back().y, guides.back().y);
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
