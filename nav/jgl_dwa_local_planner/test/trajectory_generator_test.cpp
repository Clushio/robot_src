#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <jgl_dwa_local_planner/path_follower.h>
#include <jgl_dwa_local_planner/trajectory_generator.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{

geometry_msgs::PoseStamped makePose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf2::convert(q, pose.pose.orientation);
  return pose;
}

nav_msgs::OccupancyGrid makeFreeGrid(double origin_x,
                                     double origin_y,
                                     double resolution,
                                     unsigned int width,
                                     unsigned int height)
{
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(width * height, 0);
  return grid;
}

void addHorizontalWall(nav_msgs::OccupancyGrid &grid,
                       double y,
                       double thickness)
{
  for (unsigned int my = 0; my < grid.info.height; ++my)
  {
    const double cy = grid.info.origin.position.y +
                      (static_cast<double>(my) + 0.5) * grid.info.resolution;
    if (std::fabs(cy - y) > 0.5 * thickness)
    {
      continue;
    }

    for (unsigned int mx = 0; mx < grid.info.width; ++mx)
    {
      grid.data[my * grid.info.width + mx] = 100;
    }
  }
}

double minYInXRange(const nav_msgs::Path &path, double min_x, double max_x)
{
  double best = std::numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < path.poses.size(); ++i)
  {
    const double x = path.poses[i].pose.position.x;
    if (x >= min_x && x <= max_x)
    {
      best = std::min(best, path.poses[i].pose.position.y);
    }
  }
  return best;
}

}  // namespace

TEST(TrajectoryGeneratorBspline, StraightLineUsesSecondToPenultimateEndpoints)
{
  ros::Time::init();
  jgl_dwa_local_planner::TrajectoryGenerator generator;
  generator.setGlobalCostmapForTesting(makeFreeGrid(-1.0, -1.0, 0.05, 100, 40));
  generator.setReferenceLimitsForTesting(0.05, 0.10, 0.30, 2.0, 0.48);
  generator.setBsplineOptimizationForTesting(0.30, 60, 1.0, 2.0, 0.6, 1.0);

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(makePose(0.0, 0.0));
  waypoints.push_back(makePose(1.0, 0.0));
  waypoints.push_back(makePose(2.0, 0.0));
  waypoints.push_back(makePose(3.0, 0.0));
  waypoints.push_back(makePose(4.0, 0.0));

  nav_msgs::Path path;
  ASSERT_TRUE(generator.generate(waypoints, path));
  EXPECT_EQ(jgl_dwa_local_planner::TrajectoryGenerator::PATH_MODE_BSPLINE,
            generator.lastPathMode());
  ASSERT_GE(path.poses.size(), 2U);
  EXPECT_NEAR(1.0, path.poses.front().pose.position.x, 1e-9);
  EXPECT_NEAR(0.0, path.poses.front().pose.position.y, 1e-9);
  EXPECT_NEAR(3.0, path.poses.back().pose.position.x, 1e-9);
  EXPECT_NEAR(0.0, path.poses.back().pose.position.y, 1e-9);

  for (unsigned int i = 0; i < path.poses.size(); ++i)
  {
    EXPECT_NEAR(0.0, path.poses[i].pose.position.y, 1e-5);
  }
  EXPECT_TRUE(generator.checkCollision(path));
  EXPECT_TRUE(generator.checkDeviationFromTopo(path, waypoints));
  EXPECT_TRUE(generator.checkCurvature(path));
}

TEST(TrajectoryGeneratorBspline, WallNearTopoPushesMiddleAway)
{
  ros::Time::init();
  nav_msgs::OccupancyGrid grid = makeFreeGrid(-0.5, -0.2, 0.05, 70, 40);
  addHorizontalWall(grid, 0.0, 0.06);

  jgl_dwa_local_planner::TrajectoryGenerator generator;
  generator.setGlobalCostmapForTesting(grid);
  generator.setReferenceLimitsForTesting(0.05, 0.20, 0.50, 4.0, 0.0);
  generator.setBsplineOptimizationForTesting(0.25, 120, 0.8, 8.0, 0.15, 0.5);

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(makePose(-0.4, 0.40));
  waypoints.push_back(makePose(0.0, 0.40));
  waypoints.push_back(makePose(1.0, 0.22));
  waypoints.push_back(makePose(2.0, 0.40));
  waypoints.push_back(makePose(2.4, 0.40));

  nav_msgs::Path path;
  ASSERT_TRUE(generator.generate(waypoints, path));
  EXPECT_EQ(jgl_dwa_local_planner::TrajectoryGenerator::PATH_MODE_BSPLINE,
            generator.lastPathMode());

  EXPECT_GT(minYInXRange(path, 0.7, 1.3), 0.24);
  EXPECT_TRUE(generator.checkCollision(path));
  EXPECT_TRUE(generator.checkDeviationFromTopo(path, waypoints));
}

TEST(TrajectoryGeneratorBspline, TooSharpFullBsplineFallsBackToChunkedBspline)
{
  ros::Time::init();
  jgl_dwa_local_planner::TrajectoryGenerator generator;
  generator.setGlobalCostmapForTesting(makeFreeGrid(-1.0, -1.0, 0.05, 60, 60));
  generator.setReferenceLimitsForTesting(0.05, 0.0, 1.0, 1.0, 1.0);
  generator.setBsplineOptimizationForTesting(0.40, 80, 1.0, 0.0, 0.4, 1.0);

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(makePose(0.0, 0.0));
  waypoints.push_back(makePose(0.4, 0.0));
  waypoints.push_back(makePose(0.4, 0.4));
  waypoints.push_back(makePose(0.8, 0.4));
  waypoints.push_back(makePose(1.2, 0.4));

  nav_msgs::Path path;
  ASSERT_TRUE(generator.generate(waypoints, path));
  EXPECT_EQ(jgl_dwa_local_planner::TrajectoryGenerator::PATH_MODE_POLYLINE_FALLBACK,
            generator.lastPathMode());
  const std::vector<int> fallback_segments = generator.lastFallbackSegments();
  ASSERT_EQ(waypoints.size() - 1, fallback_segments.size());
  EXPECT_EQ(0, fallback_segments[0]);
  EXPECT_EQ(1, fallback_segments[1]);
  EXPECT_EQ(1, fallback_segments[2]);
  EXPECT_EQ(0, fallback_segments[3]);
  EXPECT_TRUE(generator.checkCollision(path));
  EXPECT_TRUE(generator.checkDeviationFromTopo(path, waypoints));
  EXPECT_FALSE(generator.checkCurvature(path));
}

TEST(TrajectoryGeneratorBspline, TwoCurveWaypointReferenceMarksSecondToThirdSegmentPolyline)
{
  ros::Time::init();
  jgl_dwa_local_planner::TrajectoryGenerator generator;
  generator.setGlobalCostmapForTesting(makeFreeGrid(-1.0, -1.0, 0.05, 60, 60));
  generator.setReferenceLimitsForTesting(0.05, 0.0, 1.0, 2.0, 0.48);

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(makePose(0.0, 0.0));
  waypoints.push_back(makePose(0.4, 0.0));
  waypoints.push_back(makePose(0.4, 0.4));
  waypoints.push_back(makePose(0.8, 0.4));

  nav_msgs::Path path;
  ASSERT_TRUE(generator.generate(waypoints, path));
  EXPECT_EQ(jgl_dwa_local_planner::TrajectoryGenerator::PATH_MODE_POLYLINE_FALLBACK,
            generator.lastPathMode());

  const std::vector<int> fallback_segments = generator.lastFallbackSegments();
  ASSERT_EQ(waypoints.size() - 1, fallback_segments.size());
  EXPECT_EQ(0, fallback_segments[0]);
  EXPECT_EQ(1, fallback_segments[1]);
  EXPECT_EQ(0, fallback_segments[2]);
  EXPECT_TRUE(generator.checkCollision(path));
}

TEST(PathFollowerAckermannOnly, CommandNeverUsesCrabOrReverse)
{
  ros::Time::init();
  jgl_dwa_local_planner::PathFollower follower;

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.5, 0.0));

  geometry_msgs::Twist cmd_vel;
  unsigned int new_index = 0;
  double curvature = 0.0;
  ASSERT_TRUE(follower.computeCommand(path,
                                      makePose(0.0, 0.0, 0.5 * M_PI),
                                      0,
                                      cmd_vel,
                                      new_index,
                                      curvature));

  EXPECT_GE(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(0.0, cmd_vel.linear.y);
  EXPECT_LE(std::fabs(curvature), 2.1 + 1e-9);
  EXPECT_NEAR(cmd_vel.linear.x * curvature, cmd_vel.angular.z, 1e-9);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
