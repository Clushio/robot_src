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
  void reset();

  bool computeCommand(const nav_msgs::Path &path,
                      const geometry_msgs::PoseStamped &current_pose,
                      unsigned int current_index,
                      geometry_msgs::Twist &cmd_vel,
                      unsigned int &new_index,
                      double &curvature);

  double lookaheadDistance() const { return lookahead_distance_; }

private:
  geometry_msgs::PoseStamped interpolatedLookaheadTarget(
      const nav_msgs::Path &path,
      const geometry_msgs::PoseStamped &current_pose,
      unsigned int current_index) const;
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
  double smoothCurvature(double target_curvature);

  double lookahead_distance_;
  double v_min_;
  double v_max_;
  double end_slow_distance_;
  double k_curve_;
  double max_curvature_;
  double min_turn_radius_;
  double curvature_filter_tau_;
  double max_curvature_rate_;
  double curvature_deadband_;
  double control_period_;
  double filtered_curvature_;

#ifdef JGL_DWA_LOCAL_PLANNER_ENABLE_TEST_ACCESS
public:
  void setSmoothingForTesting(double filter_tau,
                              double max_rate,
                              double deadband,
                              double control_period)
  {
    curvature_filter_tau_ = filter_tau;
    max_curvature_rate_ = max_rate;
    curvature_deadband_ = deadband;
    control_period_ = control_period;
    reset();
  }
#endif
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_
