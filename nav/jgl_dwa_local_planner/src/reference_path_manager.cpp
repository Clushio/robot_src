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
      frozen_mode_(false),
      frozen_plan_id_(0),
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
  frozen_topology_sub_ = node->create_subscription<anav_interfaces::msg::FrozenTopologyPlan>(
      "/anav/frozen_topology_plan", rclcpp::QoS(1).transient_local(),
      std::bind(&ReferencePathManager::frozenTopologyCallback, this,
                std::placeholders::_1));
  frozen_plan_received_pub_ = node->create_publisher<std_msgs::msg::UInt64>(
      "/anav/frozen_plan_received", rclcpp::QoS(1).transient_local());
}

void ReferencePathManager::topologyCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (frozen_mode_)
  {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "JGL frozen reference: ignore ordinary topology update while plan %llu is active.",
        static_cast<unsigned long long>(frozen_plan_id_));
    return;
  }
  const bool repeated_delivery =
      sameTopology(topo_waypoints_, msg->poses) &&
      rclcpp::Time(msg->header.stamp).nanoseconds() != 0 &&
      rclcpp::Time(msg->header.stamp) == last_topology_stamp_;
  if (repeated_delivery)
  {
    return;
  }
  last_topology_stamp_ = rclcpp::Time(msg->header.stamp);
  frozen_mode_ = false;
  frozen_plan_id_ = 0;
  frozen_goal_waypoint_indices_.clear();
  frozen_local_map_snapshot_.data.clear();

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

void ReferencePathManager::activate()
{
  if (frozen_plan_received_pub_) frozen_plan_received_pub_->on_activate();
}

void ReferencePathManager::deactivate()
{
  if (frozen_plan_received_pub_) frozen_plan_received_pub_->on_deactivate();
}

void ReferencePathManager::frozenTopologyCallback(
    const anav_interfaces::msg::FrozenTopologyPlan::SharedPtr msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (msg->plan_id == 0 || msg->waypoints.poses.size() < 4)
  {
    if (msg->plan_id == 0 || msg->plan_id == frozen_plan_id_)
    {
      topo_waypoints_.clear();
      reference_path_.poses.clear();
      have_reference_path_ = false;
      topology_changed_ = true;
      current_path_index_ = 0;
      frozen_mode_ = false;
      frozen_plan_id_ = 0;
      frozen_goal_waypoint_indices_.clear();
      frozen_local_map_snapshot_.data.clear();
      topology_version_++;
      std_msgs::msg::UInt64 cleared;
      cleared.data = 0;
      frozen_plan_received_pub_->publish(cleared);
    }
    return;
  }
  if (frozen_mode_ && msg->plan_id < frozen_plan_id_)
  {
    RCLCPP_WARN(logger_, "JGL frozen reference: ignore stale plan id %llu (current %llu).",
        static_cast<unsigned long long>(msg->plan_id),
        static_cast<unsigned long long>(frozen_plan_id_));
    return;
  }
  if (msg->topo_node_ids.size() != msg->topo_waypoint_indices.size() ||
      msg->topo_waypoint_indices.empty() ||
      msg->local_map_snapshot.info.width == 0 ||
      msg->local_map_snapshot.info.height == 0 ||
      msg->local_map_snapshot.data.size() !=
          static_cast<std::size_t>(msg->local_map_snapshot.info.width) *
          msg->local_map_snapshot.info.height)
  {
    RCLCPP_ERROR(logger_, "JGL frozen reference: plan %llu has an invalid topo waypoint mapping.",
        static_cast<unsigned long long>(msg->plan_id));
    return;
  }
  for (unsigned int index : msg->topo_waypoint_indices)
  {
    if (index >= msg->waypoints.poses.size())
    {
      RCLCPP_ERROR(logger_,
          "JGL frozen reference: plan %llu contains out-of-range waypoint index %u.",
          static_cast<unsigned long long>(msg->plan_id), index);
      return;
    }
  }
  topo_waypoints_ = msg->waypoints.poses;
  frozen_goal_waypoint_indices_.assign(
      msg->topo_waypoint_indices.begin(), msg->topo_waypoint_indices.end());
  frozen_local_map_snapshot_ = msg->local_map_snapshot;
  last_topology_stamp_ = rclcpp::Time(msg->waypoints.header.stamp);
  reference_path_.poses.clear();
  have_reference_path_ = false;
  topology_changed_ = true;
  current_path_index_ = 0;
  frozen_mode_ = true;
  frozen_plan_id_ = msg->plan_id;
  topology_version_++;
  std_msgs::msg::UInt64 received;
  received.data = frozen_plan_id_;
  frozen_plan_received_pub_->publish(received);
  RCLCPP_INFO(logger_,
      "JGL frozen reference: received plan %llu, version %d, %zu guide waypoints.",
      static_cast<unsigned long long>(frozen_plan_id_),
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

bool ReferencePathManager::frozenMode() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return frozen_mode_;
}

uint64_t ReferencePathManager::frozenPlanId() const
{
  boost::mutex::scoped_lock lock(mutex_);
  return frozen_plan_id_;
}

bool ReferencePathManager::frozenLocalMapSnapshot(
    nav_msgs::msg::OccupancyGrid &snapshot) const
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!frozen_mode_ || frozen_local_map_snapshot_.data.empty()) return false;
  snapshot = frozen_local_map_snapshot_;
  return true;
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
  if (frozen_mode_)
  {
    for (unsigned int i : frozen_goal_waypoint_indices_)
    {
      const double distance = poseDistance(goal, topo_waypoints_[i]);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_index = static_cast<int>(i);
      }
    }
  }
  else
  {
    for (unsigned int i = 0; i < topo_waypoints_.size(); ++i)
    {
      const double distance = poseDistance(goal, topo_waypoints_[i]);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_index = static_cast<int>(i);
      }
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
