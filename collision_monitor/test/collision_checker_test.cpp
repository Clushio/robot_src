#include <cmath>
#include <limits>
#include <vector>

#include <collision_monitor/collision_checker.h>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace collision_monitor {
namespace {

nav_msgs::OccupancyGrid MakeGrid(unsigned int width = 120,
                                 unsigned int height = 120,
                                 double resolution = 0.05) {
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = -0.5 * width * resolution;
  grid.info.origin.position.y = -0.5 * height * resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(width * height, 0);
  return grid;
}

void SetCell(nav_msgs::OccupancyGrid& grid, double x, double y, int value) {
  const int mx = static_cast<int>(
      std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
  const int my = static_cast<int>(
      std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
  ASSERT_GE(mx, 0);
  ASSERT_GE(my, 0);
  ASSERT_LT(mx, static_cast<int>(grid.info.width));
  ASSERT_LT(my, static_cast<int>(grid.info.height));
  grid.data[my * grid.info.width + mx] = value;
}

CollisionChecker MakeChecker(double padding = 0.0) {
  std::vector<geometry_msgs::Point> footprint(4);
  footprint[0].x = 0.36;
  footprint[0].y = 0.25;
  footprint[1].x = 0.36;
  footprint[1].y = -0.25;
  footprint[2].x = -0.36;
  footprint[2].y = -0.25;
  footprint[3].x = -0.36;
  footprint[3].y = 0.25;
  CollisionChecker checker;
  EXPECT_TRUE(checker.setFootprint(footprint, padding));
  return checker;
}

GridPolicy StaticPolicy() {
  GridPolicy policy;
  policy.occupied_threshold = 100;
  policy.unknown_is_obstacle = true;
  policy.outside_is_obstacle = true;
  return policy;
}

GridPolicy LocalPolicy() {
  GridPolicy policy;
  policy.occupied_threshold = 100;
  policy.unknown_is_obstacle = false;
  policy.outside_is_obstacle = false;
  return policy;
}

TEST(CollisionCheckerTest, RectangleUsesPoseYaw) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid grid = MakeGrid(600, 600, 0.01);
  SetCell(grid, 0.0, 0.34, 100);

  Pose2D pose;
  EXPECT_FALSE(checker.poseCollides(pose, grid, StaticPolicy()));
  pose.yaw = M_PI_2;
  EXPECT_TRUE(checker.poseCollides(pose, grid, StaticPolicy()));
}

TEST(CollisionCheckerTest, RotationSweepFindsCornerCollision) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid local_map = MakeGrid(600, 600, 0.01);
  RolloutOptions options;
  for (const double direction : {-1.0, 1.0}) {
    nav_msgs::OccupancyGrid static_map = MakeGrid(600, 600, 0.01);
    SetCell(static_map, 0.24, direction * 0.34, 100);

    MotionState initial;
    initial.angular = direction * 0.25;
    CollisionResult result = checker.simulate(
        initial, 0.0, initial.angular, 2.0, static_map, StaticPolicy(),
        local_map, LocalPolicy(), options);
    EXPECT_TRUE(result.collision) << "direction=" << direction;
    EXPECT_TRUE(result.static_collision) << "direction=" << direction;
    EXPECT_GT(result.collision_time, 0.0) << "direction=" << direction;
  }
}

TEST(CollisionCheckerTest, PaddingExpandsCheckedFootprint) {
  CollisionChecker raw_checker = MakeChecker(0.0);
  CollisionChecker padded_checker = MakeChecker(0.05);
  nav_msgs::OccupancyGrid grid = MakeGrid(600, 600, 0.01);
  SetCell(grid, 0.395, 0.0, 100);

  Pose2D pose;
  EXPECT_FALSE(raw_checker.poseCollides(pose, grid, StaticPolicy()));
  EXPECT_TRUE(padded_checker.poseCollides(pose, grid, StaticPolicy()));
}

TEST(CollisionCheckerTest, StraightAndArcRollouts) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid static_map = MakeGrid();
  nav_msgs::OccupancyGrid local_map = MakeGrid();
  SetCell(local_map, 0.80, 0.0, 100);

  MotionState initial;
  RolloutOptions options;
  CollisionResult straight = checker.simulate(
      initial, 0.2, 0.0, 3.0, static_map, StaticPolicy(), local_map,
      LocalPolicy(), options);
  EXPECT_TRUE(straight.collision);
  EXPECT_TRUE(straight.local_collision);

  CollisionResult arc = checker.simulate(
      initial, 0.2, 0.4, 2.0, static_map, StaticPolicy(), local_map,
      LocalPolicy(), options);
  EXPECT_FALSE(arc.collision);
}

TEST(CollisionCheckerTest, ZeroTargetStillChecksMeasuredStoppingMotion) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid static_map = MakeGrid();
  nav_msgs::OccupancyGrid local_map = MakeGrid();
  SetCell(static_map, 0.43, 0.0, 100);

  MotionState initial;
  initial.linear = 0.2;
  RolloutOptions options;
  options.linear_decel = 0.5;
  CollisionResult result = checker.simulate(
      initial, initial.linear, 0.0, 0.30, static_map, StaticPolicy(), local_map,
      LocalPolicy(), options);
  EXPECT_TRUE(result.collision);
}

TEST(CollisionCheckerTest, StaticUnknownBlocksButLocalUnknownDoesNot) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid grid = MakeGrid();
  SetCell(grid, 0.0, 0.0, -1);
  Pose2D pose;
  EXPECT_TRUE(checker.poseCollides(pose, grid, StaticPolicy()));
  EXPECT_FALSE(checker.poseCollides(pose, grid, LocalPolicy()));
}

TEST(CollisionCheckerTest, OutsidePolicyDiffersByMapRole) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid grid = MakeGrid(20, 20, 0.05);
  Pose2D pose;
  pose.x = 0.49;
  EXPECT_TRUE(checker.poseCollides(pose, grid, StaticPolicy()));
  EXPECT_FALSE(checker.poseCollides(pose, grid, LocalPolicy()));
}

TEST(CollisionCheckerTest, NonFinitePoseOrMapMetadataFailsClosed) {
  CollisionChecker checker = MakeChecker();
  nav_msgs::OccupancyGrid grid = MakeGrid();
  Pose2D pose;

  pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checker.poseCollides(pose, grid, LocalPolicy()));

  pose.x = 0.0;
  grid.info.origin.orientation.w =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checker.poseCollides(pose, grid, LocalPolicy()));
}

}  // namespace
}  // namespace collision_monitor

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
