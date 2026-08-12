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
      max_curvature_(1.90),
      min_turn_radius_(0.476),
      curvature_filter_tau_(0.25),
      max_curvature_rate_(1.5),
      curvature_deadband_(0.02),
      control_period_(0.10),
      filtered_curvature_(0.0)
{
}

void PathFollower::loadParams(ros::NodeHandle &private_nh)
{
  private_nh.param("lookahead_distance", lookahead_distance_, 0.50);
  private_nh.param("v_min", v_min_, 0.04);
  private_nh.param("v_max", v_max_, 0.20);
  private_nh.param("end_slow_distance", end_slow_distance_, 0.80);
  private_nh.param("k_curve", k_curve_, 1.0);
  // Keep the controller limit separate from the reference-path limit.  The
  // latter may be close to the physical Ranger limit, while tracking needs
  // margin to avoid switching from dual Ackermann to spinning mode.
  private_nh.param("tracking_max_curvature", max_curvature_, 1.90);
  private_nh.param("min_turn_radius", min_turn_radius_, 0.476);
  private_nh.param("curvature_filter_tau", curvature_filter_tau_, 0.25);
  private_nh.param("max_curvature_rate", max_curvature_rate_, 1.5);
  private_nh.param("curvature_deadband", curvature_deadband_, 0.02);
  private_nh.param("control_period", control_period_, 0.10);

  lookahead_distance_ = std::max(0.05, lookahead_distance_);
  v_min_ = std::max(0.0, v_min_);
  v_max_ = std::max(v_min_, v_max_);
  end_slow_distance_ = std::max(0.0, end_slow_distance_);
  k_curve_ = std::max(0.0, k_curve_);
  max_curvature_ = std::max(0.0, max_curvature_);
  min_turn_radius_ = std::max(0.0, min_turn_radius_);
  curvature_filter_tau_ = std::max(0.0, curvature_filter_tau_);
  max_curvature_rate_ = std::max(0.0, max_curvature_rate_);
  curvature_deadband_ = std::max(0.0, curvature_deadband_);
  control_period_ = std::max(0.01, control_period_);
  reset();
}

void PathFollower::reset()
{
  // Start from centred wheels.  The rate limiter then brings steering in
  // smoothly when a new reference path becomes active.
  filtered_curvature_ = 0.0;
}

bool PathFollower::computeCommand(const nav_msgs::Path &path,
                                  const geometry_msgs::PoseStamped &current_pose,
                                  unsigned int current_index,
                                  geometry_msgs::Twist &cmd_vel,
                                  unsigned int &new_index,
                                  double &curvature)
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

  // Interpolate the target at the exact arc-length lookahead.  Selecting the
  // first sampled point beyond the lookahead made the target jump by one path
  // sample and showed up as visible steering-wheel shake.
  const geometry_msgs::PoseStamped target =
      interpolatedLookaheadTarget(path, current_pose, new_index);
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

  double target_curvature = raw_curvature;
  const double max_curvature = effectiveMaxCurvature();
  if (max_curvature > 0.0 && std::fabs(target_curvature) > max_curvature)
  {
    target_curvature = target_curvature > 0.0 ? max_curvature : -max_curvature;
  }
  curvature = smoothCurvatureCommand(target_curvature);

  cmd_vel.linear.x = std::max(0.0, velocity);
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = cmd_vel.linear.x * curvature;
  return true;
}

geometry_msgs::PoseStamped PathFollower::interpolatedLookaheadTarget(
    const nav_msgs::Path &path,
    const geometry_msgs::PoseStamped &current_pose,
    unsigned int current_index) const
{
  const unsigned int last = static_cast<unsigned int>(path.poses.size() - 1);
  current_index = std::min(current_index, last);
  geometry_msgs::PoseStamped target = path.poses[current_index];
  if (current_index >= last)
  {
    return target;
  }

  const geometry_msgs::PoseStamped &a = path.poses[current_index];
  const geometry_msgs::PoseStamped &b = path.poses[current_index + 1];
  const double ab_x = b.pose.position.x - a.pose.position.x;
  const double ab_y = b.pose.position.y - a.pose.position.y;
  const double ab_length_sq = ab_x * ab_x + ab_y * ab_y;
  double projection = 0.0;
  if (ab_length_sq > 1e-9)
  {
    projection = ((current_pose.pose.position.x - a.pose.position.x) * ab_x +
                  (current_pose.pose.position.y - a.pose.position.y) * ab_y) /
                 ab_length_sq;
    projection = clamp(projection, 0.0, 1.0);
  }

  double start_x = a.pose.position.x + projection * ab_x;
  double start_y = a.pose.position.y + projection * ab_y;
  double distance_left = lookahead_distance_;

  for (unsigned int i = current_index; i < last; ++i)
  {
    const double end_x = path.poses[i + 1].pose.position.x;
    const double end_y = path.poses[i + 1].pose.position.y;
    const double segment_x = end_x - start_x;
    const double segment_y = end_y - start_y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length > 1e-9 && distance_left <= segment_length)
    {
      const double ratio = distance_left / segment_length;
      target = path.poses[i + 1];
      target.pose.position.x = start_x + ratio * segment_x;
      target.pose.position.y = start_y + ratio * segment_y;
      return target;
    }

    distance_left -= segment_length;
    start_x = end_x;
    start_y = end_y;
  }

  return path.poses.back();
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

double PathFollower::smoothCurvatureCommand(double target_curvature)
{
  const double max_curvature = effectiveMaxCurvature();
  if (max_curvature > 0.0)
  {
    target_curvature = clamp(target_curvature, -max_curvature, max_curvature);
  }

  if (std::fabs(target_curvature - filtered_curvature_) < curvature_deadband_)
  {
    target_curvature = filtered_curvature_;
  }

  double filtered_target = target_curvature;
  if (curvature_filter_tau_ > 1e-9)
  {
    const double alpha = control_period_ /
                         (curvature_filter_tau_ + control_period_);
    filtered_target = filtered_curvature_ +
                      alpha * (target_curvature - filtered_curvature_);
  }

  double delta = filtered_target - filtered_curvature_;
  if (max_curvature_rate_ > 1e-9)
  {
    const double max_delta = max_curvature_rate_ * control_period_;
    delta = clamp(delta, -max_delta, max_delta);
  }
  filtered_curvature_ += delta;
  return filtered_curvature_;
}

}  // namespace jgl_dwa_local_planner
