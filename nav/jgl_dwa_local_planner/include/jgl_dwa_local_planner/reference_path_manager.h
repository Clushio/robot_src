#ifndef JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_
#define JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <vector>

namespace jgl_dwa_local_planner
{

class ReferencePathManager
{
public:
  ReferencePathManager();

  void initialize(ros::NodeHandle &node_nh, ros::NodeHandle &private_nh);

  bool hasWaypoints() const;
  std::vector<geometry_msgs::PoseStamped> waypoints() const;
  int topologyVersion() const;

  bool hasValidPath() const;
  nav_msgs::Path referencePath() const;
  void setReferencePath(const nav_msgs::Path &path);
  void invalidate();

  int pathVersion() const;
  unsigned int currentPathIndex() const;
  void advanceCurrentPathIndex(unsigned int index);

  bool needRegenerate() const;
  void markRegenerateAttempt();

  int goalIndex(const geometry_msgs::PoseStamped &goal) const;
  bool isMiddleGoal(const geometry_msgs::PoseStamped &goal, int *goal_index) const;
  bool referenceProgressReached(const geometry_msgs::PoseStamped &goal,
                                const geometry_msgs::PoseStamped &current_pose,
                                double pass_distance,
                                unsigned int *goal_path_index,
                                double *remaining_reference_distance,
                                double *goal_distance) const;
  double distanceToReference(const geometry_msgs::PoseStamped &pose,
                             unsigned int *nearest_index) const;

private:
  void topologyCallback(const nav_msgs::Path::ConstPtr &msg);
  bool sameTopology(const std::vector<geometry_msgs::PoseStamped> &a,
                    const std::vector<geometry_msgs::PoseStamped> &b) const;
  double poseDistance(const geometry_msgs::PoseStamped &a,
                      const geometry_msgs::PoseStamped &b) const;
  double pathDistance(unsigned int from_index, unsigned int to_index) const;

  ros::Subscriber topology_sub_;
  mutable boost::mutex mutex_;

  std::vector<geometry_msgs::PoseStamped> topo_waypoints_;
  nav_msgs::Path reference_path_;
  bool have_reference_path_;
  bool topology_changed_;
  int topology_version_;
  int path_version_;
  unsigned int current_path_index_;
  ros::Time last_regenerate_attempt_;
  ros::Time last_topology_stamp_;

  double path_regenerate_cooldown_;
  double waypoint_match_tolerance_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_REFERENCE_PATH_MANAGER_H_
