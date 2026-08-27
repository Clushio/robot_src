#ifndef JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_
#define JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <vector>

namespace jgl_dwa_local_planner
{

class ReferencePathManager
{
public:
  ReferencePathManager();

  void initialize(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      const std::string &parameter_prefix);

  bool hasWaypoints() const;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints() const;
  int topologyVersion() const;

  bool hasValidPath() const;
  nav_msgs::msg::Path referencePath() const;
  void setReferencePath(const nav_msgs::msg::Path &path);
  void invalidate();

  int pathVersion() const;
  unsigned int currentPathIndex() const;
  void advanceCurrentPathIndex(unsigned int index,
                               bool hold_before_path_end = false);

  bool needRegenerate() const;
  void markRegenerateAttempt();

  int goalIndex(const geometry_msgs::msg::PoseStamped &goal) const;
  bool isMiddleGoal(const geometry_msgs::msg::PoseStamped &goal, int *goal_index) const;
  bool isTerminalReferenceGoal(int goal_index) const;
  bool referenceProgressReached(const geometry_msgs::msg::PoseStamped &goal,
                                const geometry_msgs::msg::PoseStamped &current_pose,
                                double pass_distance,
                                unsigned int *goal_path_index,
                                double *remaining_reference_distance,
                                double *goal_distance) const;
  double distanceToReference(const geometry_msgs::msg::PoseStamped &pose,
                             unsigned int *nearest_index) const;

private:
  void topologyCallback(const nav_msgs::msg::Path::SharedPtr msg);
  bool sameTopology(const std::vector<geometry_msgs::msg::PoseStamped> &a,
                    const std::vector<geometry_msgs::msg::PoseStamped> &b) const;
  double poseDistance(const geometry_msgs::msg::PoseStamped &a,
                      const geometry_msgs::msg::PoseStamped &b) const;
  double pathDistance(unsigned int from_index, unsigned int to_index) const;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr topology_sub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("ReferencePathManager")};
  mutable boost::mutex mutex_;

  std::vector<geometry_msgs::msg::PoseStamped> topo_waypoints_;
  nav_msgs::msg::Path reference_path_;
  bool have_reference_path_;
  bool topology_changed_;
  int topology_version_;
  int path_version_;
  unsigned int current_path_index_;
  rclcpp::Time last_regenerate_attempt_;
  rclcpp::Time last_topology_stamp_;

  double path_regenerate_cooldown_;
  double waypoint_match_tolerance_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_
