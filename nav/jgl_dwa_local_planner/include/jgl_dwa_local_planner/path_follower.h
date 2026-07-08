#ifndef JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_
#define JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace jgl_dwa_local_planner
{

class PathFollower
{
public:
  PathFollower();

  void loadParams(ros::NodeHandle &private_nh);

  bool computeCommand(const nav_msgs::Path &path,
                      const geometry_msgs::PoseStamped &current_pose,
                      unsigned int current_index,
                      geometry_msgs::Twist &cmd_vel,
                      unsigned int &new_index,
                      double &curvature) const;

  double lookaheadDistance() const { return lookahead_distance_; }

private:
  unsigned int advanceIndex(const nav_msgs::Path &path,
                            const geometry_msgs::PoseStamped &current_pose,
                            unsigned int current_index) const;
  double remainingDistance(const nav_msgs::Path &path,
                           const geometry_msgs::PoseStamped &current_pose,
                           unsigned int index) const;
  double poseDistance(const geometry_msgs::PoseStamped &a,
                      const geometry_msgs::PoseStamped &b) const;
  double clamp(double value, double min_value, double max_value) const;
  double normalizeAngle(double angle) const;
  double effectiveMaxCurvature() const;

  double lookahead_distance_;
  double v_min_;
  double v_max_;
  double end_slow_distance_;
  double k_curve_;
  double max_curvature_;
  double min_turn_radius_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_
