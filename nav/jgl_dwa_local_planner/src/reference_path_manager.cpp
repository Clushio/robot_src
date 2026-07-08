#include <jgl_dwa_local_planner/reference_path_manager.h>

#include <algorithm>
#include <cmath>

namespace jgl_dwa_local_planner
{

ReferencePathManager::ReferencePathManager()
    : have_reference_path_(false),
      topology_changed_(false),
      topology_version_(0),
      path_version_(0),
      current_path_index_(0),
      path_deviation_replan_threshold_(0.60),
      path_regenerate_cooldown_(1.0),
      waypoint_match_tolerance_(0.35)
{
}

void ReferencePathManager::initialize(ros::NodeHandle &node_nh,
                                      ros::NodeHandle &private_nh)
{
  private_nh.param("path_deviation_replan_threshold",
                   path_deviation_replan_threshold_, 0.60);
  private_nh.param("path_regenerate_cooldown",
                   path_regenerate_cooldown_, 1.0);

  path_deviation_replan_threshold_ = std::max(0.05, path_deviation_replan_threshold_);
  path_regenerate_cooldown_ = std::max(0.0, path_regenerate_cooldown_);

  topology_sub_ = node_nh.subscribe("/topology_plan", 1,
                                    &ReferencePathManager::topologyCallback, this);
}

void ReferencePathManager::topologyCallback(const nav_msgs::Path::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (msg->poses.size() < 2)
  {
    topo_waypoints_.clear();
    reference_path_.poses.clear();
    have_reference_path_ = false;
    topology_changed_ = true;
    current_path_index_ = 0;
    topology_version_++;
    return;
  }

  if (sameTopology(topo_waypoints_, msg->poses))
  {
    return;
  }

  topo_waypoints_ = msg->poses;
  reference_path_.poses.clear();
  have_reference_path_ = false;
  topology_changed_ = true;
  current_path_index_ = 0;
  topology_version_++;
  ROS_INFO("JGL reference path: received topology path version %d with %zu waypoints.",
           topology_version_, topo_waypoints_.size());
}

bool ReferencePathManager::hasWaypoints() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return topo_waypoints_.size() >= 2;
}

std::vector<geometry_msgs::PoseStamped> ReferencePathManager::waypoints() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return topo_waypoints_;
}

int ReferencePathManager::topologyVersion() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return topology_version_;
}

bool ReferencePathManager::hasValidPath() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return have_reference_path_ && reference_path_.poses.size() >= 2;
}

nav_msgs::Path ReferencePathManager::referencePath() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return reference_path_;
}

void ReferencePathManager::setReferencePath(const nav_msgs::Path &path)
{
  boost::mutex::scoped_lock lock(mutex_);
  reference_path_ = path;
  have_reference_path_ = reference_path_.poses.size() >= 2;
  topology_changed_ = false;
  current_path_index_ = 0;
  if (have_reference_path_)
  {
    path_version_++;
  }
}

void ReferencePathManager::invalidate()
{
  boost::mutex::scoped_lock lock(mutex_);
  reference_path_.poses.clear();
  have_reference_path_ = false;
  current_path_index_ = 0;
}

int ReferencePathManager::pathVersion() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return path_version_;
}

unsigned int ReferencePathManager::currentPathIndex() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return current_path_index_;
}

void ReferencePathManager::advanceCurrentPathIndex(unsigned int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!have_reference_path_ || reference_path_.poses.empty())
  {
    current_path_index_ = 0;
    return;
  }
  const unsigned int max_index =
      static_cast<unsigned int>(reference_path_.poses.size() - 1);
  current_path_index_ = std::min(max_index, std::max(current_path_index_, index));
}

bool ReferencePathManager::needRegenerate(
    const geometry_msgs::PoseStamped &current_pose) const
{
  boost::mutex::scoped_lock lock(mutex_);
  if (topo_waypoints_.size() < 3)
  {
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (!last_regenerate_attempt_.isZero() &&
      (now - last_regenerate_attempt_).toSec() < path_regenerate_cooldown_)
  {
    return false;
  }

  if (topology_changed_ || !have_reference_path_ || reference_path_.poses.size() < 2)
  {
    return true;
  }

  unsigned int start_index =
      std::min(current_path_index_,
               static_cast<unsigned int>(reference_path_.poses.size() - 1));
  double deviation = 1e9;
  for (unsigned int i = start_index; i < reference_path_.poses.size(); ++i)
  {
    deviation = std::min(deviation, poseDistance(current_pose, reference_path_.poses[i]));
  }
  return deviation > path_deviation_replan_threshold_;
}

void ReferencePathManager::markRegenerateAttempt()
{
  boost::mutex::scoped_lock lock(mutex_);
  last_regenerate_attempt_ = ros::Time::now();
}

int ReferencePathManager::goalIndex(const geometry_msgs::PoseStamped &goal) const
{
  boost::mutex::scoped_lock lock(mutex_);
  if (topo_waypoints_.empty())
  {
    return -1;
  }

  int best_index = -1;
  double best_distance = 1e9;
  for (unsigned int i = 0; i < topo_waypoints_.size(); ++i)
  {
    const double distance = poseDistance(goal, topo_waypoints_[i]);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }
  return best_distance <= waypoint_match_tolerance_ ? best_index : -1;
}

bool ReferencePathManager::isMiddleGoal(const geometry_msgs::PoseStamped &goal,
                                        int *goal_index) const
{
  const int index = goalIndex(goal);
  if (goal_index != NULL)
  {
    *goal_index = index;
  }

  boost::mutex::scoped_lock lock(mutex_);
  return index > 1 && index < static_cast<int>(topo_waypoints_.size()) - 1;
}

bool ReferencePathManager::referenceProgressReached(
    const geometry_msgs::PoseStamped &goal,
    const geometry_msgs::PoseStamped &current_pose,
    double pass_distance,
    unsigned int *goal_path_index,
    double *remaining_reference_distance,
    double *goal_distance) const
{
  boost::mutex::scoped_lock lock(mutex_);
  if (goal_path_index != NULL)
  {
    *goal_path_index = current_path_index_;
  }
  if (remaining_reference_distance != NULL)
  {
    *remaining_reference_distance = 1e9;
  }
  if (goal_distance != NULL)
  {
    *goal_distance = 1e9;
  }

  if (!have_reference_path_ || reference_path_.poses.empty())
  {
    return false;
  }

  unsigned int best_index = 0;
  double best_distance = 1e9;
  for (unsigned int i = 0; i < reference_path_.poses.size(); ++i)
  {
    const double distance = poseDistance(goal, reference_path_.poses[i]);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = i;
    }
  }

  const unsigned int current_index =
      std::min(current_path_index_,
               static_cast<unsigned int>(reference_path_.poses.size() - 1));
  const double remaining =
      current_index >= best_index ? 0.0 : pathDistance(current_index, best_index);
  const double direct_goal_distance = poseDistance(current_pose, goal);

  if (goal_path_index != NULL)
  {
    *goal_path_index = best_index;
  }
  if (remaining_reference_distance != NULL)
  {
    *remaining_reference_distance = remaining;
  }
  if (goal_distance != NULL)
  {
    *goal_distance = direct_goal_distance;
  }

  pass_distance = std::max(0.02, pass_distance);
  return current_index >= best_index ||
         remaining <= pass_distance ||
         direct_goal_distance <= pass_distance;
}

double ReferencePathManager::distanceToReference(
    const geometry_msgs::PoseStamped &pose,
    unsigned int *nearest_index) const
{
  if (nearest_index != NULL)
  {
    *nearest_index = current_path_index_;
  }
  if (!have_reference_path_ || reference_path_.poses.empty())
  {
    return 1e9;
  }

  unsigned int start_index =
      std::min(current_path_index_,
               static_cast<unsigned int>(reference_path_.poses.size() - 1));
  double best = 1e9;
  unsigned int best_index = start_index;
  for (unsigned int i = start_index; i < reference_path_.poses.size(); ++i)
  {
    const double distance = poseDistance(pose, reference_path_.poses[i]);
    if (distance < best)
    {
      best = distance;
      best_index = i;
    }
  }
  if (nearest_index != NULL)
  {
    *nearest_index = best_index;
  }
  return best;
}

bool ReferencePathManager::sameTopology(
    const std::vector<geometry_msgs::PoseStamped> &a,
    const std::vector<geometry_msgs::PoseStamped> &b) const
{
  if (a.size() != b.size())
  {
    return false;
  }
  for (unsigned int i = 0; i < a.size(); ++i)
  {
    if (poseDistance(a[i], b[i]) > 0.02)
    {
      return false;
    }
  }
  return true;
}

double ReferencePathManager::poseDistance(
    const geometry_msgs::PoseStamped &a,
    const geometry_msgs::PoseStamped &b) const
{
  return std::hypot(a.pose.position.x - b.pose.position.x,
                    a.pose.position.y - b.pose.position.y);
}

double ReferencePathManager::pathDistance(unsigned int from_index,
                                          unsigned int to_index) const
{
  if (reference_path_.poses.empty())
  {
    return 0.0;
  }

  const unsigned int last_index =
      static_cast<unsigned int>(reference_path_.poses.size() - 1);
  from_index = std::min(from_index, last_index);
  to_index = std::min(to_index, last_index);
  if (from_index >= to_index)
  {
    return 0.0;
  }

  double distance = 0.0;
  for (unsigned int i = from_index + 1; i <= to_index; ++i)
  {
    distance += poseDistance(reference_path_.poses[i - 1],
                             reference_path_.poses[i]);
  }
  return distance;
}

}  // namespace jgl_dwa_local_planner
