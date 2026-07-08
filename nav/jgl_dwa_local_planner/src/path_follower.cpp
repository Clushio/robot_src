#include <jgl_dwa_local_planner/path_follower.h>

#include <algorithm>
#include <cmath>

#include <tf2/utils.h>

namespace jgl_dwa_local_planner
{

PathFollower::PathFollower()
    : lookahead_distance_(0.50),
      v_min_(0.04),
      v_max_(0.20),
      end_slow_distance_(0.80),
      k_curve_(1.0),
      max_curvature_(2.1),
      min_turn_radius_(0.476)
{
}

void PathFollower::loadParams(ros::NodeHandle &private_nh)
{
  private_nh.param("lookahead_distance", lookahead_distance_, 0.50);
  private_nh.param("v_min", v_min_, 0.04);
  private_nh.param("v_max", v_max_, 0.20);
  private_nh.param("end_slow_distance", end_slow_distance_, 0.80);
  private_nh.param("k_curve", k_curve_, 1.0);
  private_nh.param("max_curvature", max_curvature_, 2.1);
  private_nh.param("min_turn_radius", min_turn_radius_, 0.476);

  lookahead_distance_ = std::max(0.05, lookahead_distance_);
  v_min_ = std::max(0.0, v_min_);
  v_max_ = std::max(v_min_, v_max_);
  end_slow_distance_ = std::max(0.0, end_slow_distance_);
  k_curve_ = std::max(0.0, k_curve_);
  max_curvature_ = std::max(0.0, max_curvature_);
  min_turn_radius_ = std::max(0.0, min_turn_radius_);
}

bool PathFollower::computeCommand(const nav_msgs::Path &path,
                                  const geometry_msgs::PoseStamped &current_pose,
                                  unsigned int current_index,
                                  geometry_msgs::Twist &cmd_vel,
                                  unsigned int &new_index,
                                  double &curvature) const
{
  cmd_vel = geometry_msgs::Twist();
  curvature = 0.0;
  new_index = current_index;

  if (path.poses.size() < 2)
  {
    return false;
  }

  new_index = advanceIndex(path, current_pose, current_index);
  if (new_index >= path.poses.size() - 1)
  {
    cmd_vel.linear.y = 0.0;
    return true;
  }

  unsigned int target_index = new_index;
  double walked = poseDistance(current_pose, path.poses[new_index]);
  for (unsigned int i = new_index + 1; i < path.poses.size(); ++i)
  {
    walked += poseDistance(path.poses[i - 1], path.poses[i]);
    target_index = i;
    if (walked >= lookahead_distance_)
    {
      break;
    }
  }

  const geometry_msgs::PoseStamped &target = path.poses[target_index];
  const double dx = target.pose.position.x - current_pose.pose.position.x;
  const double dy = target.pose.position.y - current_pose.pose.position.y;
  const double target_distance = std::max(0.05, std::hypot(dx, dy));
  const double yaw = tf2::getYaw(current_pose.pose.orientation);
  const double alpha = normalizeAngle(std::atan2(dy, dx) - yaw);

  const double raw_curvature = 2.0 * std::sin(alpha) / target_distance;
  double velocity = v_max_ / (1.0 + k_curve_ * std::fabs(raw_curvature));
  velocity = clamp(velocity, v_min_, v_max_);

  const double remain = remainingDistance(path, current_pose, new_index);
  if (end_slow_distance_ > 1e-6 && remain < end_slow_distance_)
  {
    const double scale = clamp(remain / end_slow_distance_, 0.0, 1.0);
    velocity = std::max(v_min_, velocity * scale);
  }

  curvature = raw_curvature;
  const double max_curvature = effectiveMaxCurvature();
  if (max_curvature > 0.0 && std::fabs(curvature) > max_curvature)
  {
    curvature = curvature > 0.0 ? max_curvature : -max_curvature;
  }

  cmd_vel.linear.x = std::max(0.0, velocity);
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = cmd_vel.linear.x * curvature;
  return true;
}

unsigned int PathFollower::advanceIndex(const nav_msgs::Path &path,
                                        const geometry_msgs::PoseStamped &current_pose,
                                        unsigned int current_index) const
{
  if (path.poses.empty())
  {
    return 0;
  }

  unsigned int index =
      std::min(current_index, static_cast<unsigned int>(path.poses.size() - 1));
  while (index + 1 < path.poses.size())
  {
    const geometry_msgs::PoseStamped &a = path.poses[index];
    const geometry_msgs::PoseStamped &b = path.poses[index + 1];
    const double ax = a.pose.position.x;
    const double ay = a.pose.position.y;
    const double bx = b.pose.position.x;
    const double by = b.pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len_sq = dx * dx + dy * dy;
    double projection = 0.0;
    if (len_sq > 1e-9)
    {
      projection = ((current_pose.pose.position.x - ax) * dx +
                    (current_pose.pose.position.y - ay) * dy) /
                   len_sq;
    }

    const double current_distance = poseDistance(current_pose, a);
    const double next_distance = poseDistance(current_pose, b);
    if (projection > 1.0 || next_distance + 0.02 < current_distance)
    {
      index++;
      continue;
    }
    break;
  }
  return index;
}

double PathFollower::remainingDistance(const nav_msgs::Path &path,
                                       const geometry_msgs::PoseStamped &current_pose,
                                       unsigned int index) const
{
  if (path.poses.empty())
  {
    return 0.0;
  }

  index = std::min(index, static_cast<unsigned int>(path.poses.size() - 1));
  double distance = poseDistance(current_pose, path.poses[index]);
  for (unsigned int i = index + 1; i < path.poses.size(); ++i)
  {
    distance += poseDistance(path.poses[i - 1], path.poses[i]);
  }
  return distance;
}

double PathFollower::poseDistance(const geometry_msgs::PoseStamped &a,
                                  const geometry_msgs::PoseStamped &b) const
{
  return std::hypot(a.pose.position.x - b.pose.position.x,
                    a.pose.position.y - b.pose.position.y);
}

double PathFollower::clamp(double value, double min_value, double max_value) const
{
  return std::min(std::max(value, min_value), max_value);
}

double PathFollower::normalizeAngle(double angle) const
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double PathFollower::effectiveMaxCurvature() const
{
  double limit = max_curvature_;
  if (min_turn_radius_ > 1e-6)
  {
    const double radius_limit = 1.0 / min_turn_radius_;
    limit = limit > 0.0 ? std::min(limit, radius_limit) : radius_limit;
  }
  return limit;
}

}  // namespace jgl_dwa_local_planner
