#ifndef JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_
#define JGL_DWA_LOCAL_PLANNER_PATH_FOLLOWER_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace jgl_dwa_local_planner
{

class PathFollower
{
public:
  PathFollower();

  void loadParams(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      const std::string &parameter_prefix);
  void reset();

  bool computeCommand(const nav_msgs::msg::Path &path,
                      const geometry_msgs::msg::PoseStamped &current_pose,
                      unsigned int current_index,
                      geometry_msgs::msg::Twist &cmd_vel,
                      unsigned int &new_index,
                      double &curvature,
                      bool terminal_goal = false,
                      double terminal_xy_tolerance = 0.0);

  // Shared by the reference-path follower and the legacy forward-tracking
  // state. Input and output are path curvature in 1/m.
  double smoothCurvatureCommand(double target_curvature);
  double lookaheadDistance() const { return lookahead_distance_; }

private:
  geometry_msgs::msg::PoseStamped interpolatedLookaheadTarget(
      const nav_msgs::msg::Path &path,
      const geometry_msgs::msg::PoseStamped &current_pose,
      unsigned int current_index) const;
  unsigned int advanceIndex(const nav_msgs::msg::Path &path,
                            const geometry_msgs::msg::PoseStamped &current_pose,
                            unsigned int current_index) const;
  double remainingDistance(const nav_msgs::msg::Path &path,
                           const geometry_msgs::msg::PoseStamped &current_pose,
                           unsigned int index) const;
  double poseDistance(const geometry_msgs::msg::PoseStamped &a,
                      const geometry_msgs::msg::PoseStamped &b) const;
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
