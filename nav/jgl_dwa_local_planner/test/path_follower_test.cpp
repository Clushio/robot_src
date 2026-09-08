#include <cmath>

#include <gtest/gtest.h>

#include <jgl_dwa_local_planner/path_follower.h>

namespace
{

TEST(PathFollower, ComputesCommandFromGeometryQuaternion)
{
  jgl_dwa_local_planner::PathFollower follower;
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].pose.orientation.w = 1.0;
  path.poses[1].pose.position.x = 1.0;
  path.poses[1].pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist command;
  unsigned int new_index = 0;
  double curvature = 0.0;

  ASSERT_TRUE(follower.computeCommand(
      path, current_pose, 0, command, new_index, curvature, false, 0.08));
  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_TRUE(std::isfinite(command.angular.z));
  EXPECT_TRUE(std::isfinite(curvature));
}

}  // namespace
