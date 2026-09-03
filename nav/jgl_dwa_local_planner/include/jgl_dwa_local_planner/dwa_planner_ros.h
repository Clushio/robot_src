#ifndef JGL_DWA_LOCAL_PLANNER__DWA_PLANNER_ROS_H_
#define JGL_DWA_LOCAL_PLANNER__DWA_PLANNER_ROS_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <Eigen/Dense>

#include <dwb_core/dwb_local_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <jgl_dwa_local_planner/path_follower.h>
#include <jgl_dwa_local_planner/pursuit.h>
#include <jgl_dwa_local_planner/reference_obstacle_policy.h>
#include <jgl_dwa_local_planner/reference_path_manager.h>
#include <jgl_dwa_local_planner/speedPlan.h>
#include <jgl_dwa_local_planner/terminal_yaw_controller.h>
#include <jgl_dwa_local_planner/trajectory_generator.h>

namespace jgl_dwa_local_planner
{

class DWAPlannerROS : public nav2_core::Controller
{
public:
  DWAPlannerROS();
  ~DWAPlannerROS() override;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override;
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

  bool isGoalReached();
  bool isInitialized() const { return initialized_; }

  void logToFile(const std::string &message, const std::string &filename);
  std::string getCurrentTimestamp();
  void computeRelativePosition(
      const geometry_msgs::msg::PoseStamped &p,
      const geometry_msgs::msg::PoseStamped &q);
  Eigen::Vector2d toEigenVector2d(
      const geometry_msgs::msg::PoseStamped &pose);
  double signedDistanceToLine(
      const geometry_msgs::msg::PoseStamped &a,
      const geometry_msgs::msg::PoseStamped &b,
      const geometry_msgs::msg::PoseStamped &c);
  bool lineComputeVelocityCommands(
      std::vector<geometry_msgs::msg::PoseStamped> path,
      geometry_msgs::msg::Twist &cmd_vel);
  bool lineComputeVelocityCommands_modJGL(
      std::vector<geometry_msgs::msg::PoseStamped> path,
      geometry_msgs::msg::Twist &cmd_vel);
  void cmd_pub(
      std::vector<geometry_msgs::msg::PoseStamped> points,
      geometry_msgs::msg::PoseStamped pose,
      std::vector<double> &fov_speed);
  double comDistance(
      geometry_msgs::msg::PoseStamped p,
      geometry_msgs::msg::PoseStamped q);
  bool comparePose(
      geometry_msgs::msg::PoseStamped p,
      geometry_msgs::msg::PoseStamped q);
  float base_plan_direction_check(
      const std::vector<geometry_msgs::msg::PoseStamped> &plan,
      geometry_msgs::msg::PoseStamped &robot_pose);

  geometry_msgs::msg::PoseStamped Qtar;
  float xdis{0.0F};
  float angle_err_H{0.0F};
  double dis2line{0.0};
  int state4counter{0};
  int state5counter{0};
  double pid_PA{0.7};
  double pid_PB{0.2};
  double frontdis_X{0.02};
  std::string logfilename;

private:
  enum ReferenceStatus
  {
    REFERENCE_ACTIVE = 1,
    REFERENCE_PASSED = 2,
    REFERENCE_PATH_DEVIATED = 3
  };
  enum TerminalMotionState
  {
    TERMINAL_TRACKING = 0,
    TERMINAL_POSITION_CAPTURED = 1,
    TERMINAL_ROTATING = 2,
    TERMINAL_COMPLETE = 3
  };

  void publishLocalPlan(std::vector<geometry_msgs::msg::PoseStamped> &path);
  void publishGlobalPlan(std::vector<geometry_msgs::msg::PoseStamped> &path);
  void stopCmd(geometry_msgs::msg::Twist &cmd_vel) const;
  void publishReferenceStatus(ReferenceStatus status);
  void publishTerminalMotionState(TerminalMotionState state);
  bool shouldUseReferencePath();
  bool prepareReferencePath(bool &generation_pending, bool &generation_failed);
  void maybeStartReferencePathJob();
  bool startReferencePathJob(
      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
      int topology_version);
  void referencePathGenerationThread(
      std::vector<geometry_msgs::msg::PoseStamped> waypoints,
      int topology_version);
  bool consumeReferencePathJob();
  bool referencePathJobRunning() const;
  bool referencePathJobFailedForCurrentTopology() const;
  bool referenceGenerationUsefulForCurrentGoal(
      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
  void waitForReferencePathJob();
  const char *referencePathModeName(TrajectoryGenerator::PathMode mode) const;
  bool computeReferenceVelocityCommands(
      geometry_msgs::msg::Twist &cmd_vel, bool &hard_failure);
  bool referencePathObstacleDistance(
      const nav_msgs::msg::Path &path, double &obstacle_distance);
  bool fixedRouteBlocked();
  void fixedRouteModeCallback(const std_msgs::msg::Bool::SharedPtr mode);
  bool costmapPointBlocked(
      nav2_costmap_2d::Costmap2D *costmap,
      unsigned int mx, unsigned int my, int radius_cells) const;
  void updateLineGoalRelativeState();
  bool referenceGoalReached(
      unsigned int *goal_path_index,
      double *remaining_reference_distance,
      double *goal_distance);
  void publishReferencePathMarker(
      const nav_msgs::msg::Path &path,
      TrajectoryGenerator::PathMode path_mode) const;
  bool referencePathCanFollowGoal(int goal_index) const;
  bool referenceGoalExitsToFallback(int goal_index) const;
  bool referenceEntryHeadingAligned(
      const nav_msgs::msg::Path &reference_path,
      double *heading_error) const;
  void forceLegacyLineRotate(const char *reason);
  bool syncReferencePathIndex(
      double *distance_to_reference,
      unsigned int *nearest_index,
      bool hold_before_path_end = false);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DWAPlannerROS")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      g_plan_pub_, l_plan_pub_, reference_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      reference_path_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      reference_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      terminal_motion_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fixed_route_mode_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  std::mutex control_parameter_mutex_;

  std::unique_ptr<dwb_core::DWBLocalPlanner> dwb_planner_;
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::Twist current_velocity_;
  nav2_core::GoalChecker *goal_checker_{nullptr};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool initialized_{false};

  std::unique_ptr<SpeedPlan> sp;
  std::unique_ptr<Pursuit> ps;
  int useLine{0};
  std::vector<geometry_msgs::msg::PoseStamped> linePath;
  bool enable_bspline_reference_path_{false};
  double reference_safe_distance_{0.25};
  double reference_fallback_boundary_distance_{0.15};
  double reference_middle_pass_distance_{0.25};
  double reference_terminal_xy_tolerance_{0.08};
  double obstacle_wait_time_{10.0};
  double reference_obstacle_slowdown_distance_{1.0};
  double reference_obstacle_stop_distance_{0.7};
  double path_deviation_replan_threshold_{0.60};
  std::atomic<bool> fixed_route_mode_{false};
  rclcpp::Time reference_obstacle_start_{0, 0, RCL_ROS_TIME};
  int current_topology_goal_index_{-1};
  int legacy_line_forced_goal_index_{-1};
  bool reference_goal_reached_{false};
  TrajectoryGenerator::PathMode reference_path_mode_{TrajectoryGenerator::PATH_MODE_INVALID};
  std::vector<int> reference_fallback_segments_;
  mutable boost::mutex reference_job_mutex_;
  boost::thread reference_job_thread_;
  bool reference_job_running_{false};
  bool reference_job_result_ready_{false};
  bool reference_job_result_success_{false};
  int reference_job_topology_version_{-1};
  int reference_job_failed_topology_version_{-1};
  nav_msgs::msg::Path reference_job_path_;
  TrajectoryGenerator::PathMode reference_job_path_mode_{TrajectoryGenerator::PATH_MODE_INVALID};
  std::vector<int> reference_job_fallback_segments_;
  TrajectoryGenerator trajectory_generator_;
  ReferencePathManager reference_path_manager_;
  PathFollower path_follower_;
  TerminalYawController terminal_yaw_controller_;
  double goal_yaw_err{0.0};
  double yaw_goal_tolerance{0.1};
  double xy_goal_tolerance{0.1};
  int status{0};
  int published_terminal_motion_state_{-1};
  double max_vel_x{0.4};
  double brake_distance{1.0};
  double lfc{0.3};
  double forwNum{0.1};
  bool is_start_rotating{false};
  double startrotangle{0.5};
  double stoprotangle{0.2};
  int lastz{1};
  geometry_msgs::msg::PoseStamped firstPose;
  rclcpp::Time begin{0, 0, RCL_ROS_TIME};
  double back_distance{2.5};
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER__DWA_PLANNER_ROS_H_
