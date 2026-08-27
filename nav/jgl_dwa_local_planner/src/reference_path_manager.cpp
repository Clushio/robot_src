#include <jgl_dwa_local_planner/reference_path_manager.h>

#include <algorithm>
#include <cmath>
#include <jgl_dwa_local_planner/parameter_utils.h>

namespace jgl_dwa_local_planner
{

ReferencePathManager::ReferencePathManager()
    : clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      have_reference_path_(false),
      topology_changed_(false),
      topology_version_(0),
      path_version_(0),
      current_path_index_(0),
      path_regenerate_cooldown_(1.0),
      waypoint_match_tolerance_(0.35)
{
}

void ReferencePathManager::initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::string &parameter_prefix)
{
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  path_regenerate_cooldown_ = declareOrGet(
      node, parameter_prefix + ".path_regenerate_cooldown", 1.0);

  path_regenerate_cooldown_ = std::max(0.0, path_regenerate_cooldown_);

  topology_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/topology_plan", rclcpp::QoS(1),
      std::bind(&ReferencePathManager::topologyCallback, this,
                std::placeholders::_1));
}

void ReferencePathManager::topologyCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  const bool repeated_delivery =
      sameTopology(topo_waypoints_, msg->poses) &&
      rclcpp::Time(msg->header.stamp).nanoseconds() != 0 &&
      rclcpp::Time(msg->header.stamp) == last_topology_stamp_;
  if (repeated_delivery)
  {
    return;
  }
  last_topology_stamp_ = rclcpp::Time(msg->header.stamp);

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

  topo_waypoints_ = msg->poses;
  reference_path_.poses.clear();
  have_reference_path_ = false;
  topology_changed_ = true;
  current_path_index_ = 0;
  topology_version_++;
  RCLCPP_INFO(logger_,
              "JGL reference path: received topology path version %d with %zu waypoints.",
              topology_version_, topo_waypoints_.size());
}

bool ReferencePathManager::hasWaypoints() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return topo_waypoints_.size() >= 2;
}

std::vector<geometry_msgs::msg::PoseStamped> ReferencePathManager::waypoints() const
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

nav_msgs::msg::Path ReferencePathManager::referencePath() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return reference_path_;
}

void ReferencePathManager::setReferencePath(const nav_msgs::msg::Path &path)
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

void ReferencePathManager::advanceCurrentPathIndex(unsigned int index,
                                                   bool hold_before_path_end)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!have_reference_path_ || reference_path_.poses.empty())
  {
    current_path_index_ = 0;
    return;
  }
  unsigned int max_index =
      static_cast<unsigned int>(reference_path_.poses.size() - 1);
  if (hold_before_path_end && max_index > 0)
  {
    --max_index;
  }
  current_path_index_ = std::min(max_index, std::max(current_path_index_, index));
}

bool ReferencePathManager::needRegenerate() const
{
  boost::mutex::scoped_lock lock(mutex_);
  if (topo_waypoints_.size() < 4)
  {
    return false;
  }

  const rclcpp::Time now = clock_->now();
  if (last_regenerate_attempt_.nanoseconds() != 0 &&
      (now - last_regenerate_attempt_).seconds() < path_regenerate_cooldown_)
  {
    return false;
  }

  if (topology_changed_ || !have_reference_path_ || reference_path_.poses.size() < 2)
  {
    return true;
  }
  return false;
}

void ReferencePathManager::markRegenerateAttempt()
{
  boost::mutex::scoped_lock lock(mutex_);
  last_regenerate_attempt_ = clock_->now();
}

int ReferencePathManager::goalIndex(const geometry_msgs::msg::PoseStamped &goal) const
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

bool ReferencePathManager::isMiddleGoal(const geometry_msgs::msg::PoseStamped &goal,
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

bool ReferencePathManager::isTerminalReferenceGoal(int goal_index) const
{
  boost::mutex::scoped_lock lock(mutex_);
  return topo_waypoints_.size() >= 4 &&
         goal_index == static_cast<int>(topo_waypoints_.size()) - 2;
}

bool ReferencePathManager::referenceProgressReached(
    const geometry_msgs::msg::PoseStamped &goal,
    const geometry_msgs::msg::PoseStamped &current_pose,
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
    const geometry_msgs::msg::PoseStamped &pose,
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
    const std::vector<geometry_msgs::msg::PoseStamped> &a,
    const std::vector<geometry_msgs::msg::PoseStamped> &b) const
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
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b) const
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
