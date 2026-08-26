#ifndef MYGLOBAL_PLANNER__PLANNER_CORE_H_
#define MYGLOBAL_PLANNER__PLANNER_CORE_H_

#define POT_HIGH 1.0e10

#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>

#include <myglobal_planner/expander.h>
#include <myglobal_planner/orientation_filter.h>
#include <myglobal_planner/potential_calculator.h>
#include <myglobal_planner/traceback.h>

namespace myglobal_planner
{

class Expander;
class GridPath;

class MyGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  MyGlobalPlanner();
  ~MyGlobalPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  // The ROS1 implementation was initialized twice by mxb_move_base: first
  // with the controller costmap and then with the planner costmap. Nav2 only
  // supplies the planner costmap through GlobalPlanner::configure(), so the
  // custom navigation server calls this extension to preserve local obstacle
  // inspection. A standard Nav2 load safely uses the global costmap for both.
  void setLocalCostmap(
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & local_costmap_ros);

  bool makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  bool makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);

  bool computePotential(const geometry_msgs::msg::Point & world_point);
  bool getPlanFromPotential(
    double start_x, double start_y, double end_x, double end_y,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  double getPointPotential(const geometry_msgs::msg::Point & world_point);
  bool validPointPotential(const geometry_msgs::msg::Point & world_point);
  bool validPointPotential(
    const geometry_msgs::msg::Point & world_point, double tolerance);
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> & path);

  void obsCheck(geometry_msgs::msg::PoseStamped start);
  bool makePlanLine(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  void codPath(
    double xx, double yy,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  void mapToOdom(
    geometry_msgs::msg::PoseStamped map_pose,
    geometry_msgs::msg::PoseStamped & odom_pose);
  void odomToMap(
    geometry_msgs::msg::PoseStamped odom_pose,
    geometry_msgs::msg::PoseStamped & map_pose);
  bool pointCheck(unsigned int x, unsigned int y);
  double comDistance(
    geometry_msgs::msg::PoseStamped p,
    geometry_msgs::msg::PoseStamped q);
  bool comparePose(
    geometry_msgs::msg::PoseStamped p,
    geometry_msgs::msg::PoseStamped q);
  void stateSwitch();
  bool dealState(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  bool planCheck(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);
  void setGoal(
    double & goal_x, double & goal_y,
    const geometry_msgs::msg::PoseStamped & start,
    unsigned char * master_array, int nx, int ny);
  void smooth(
    std::vector<std::pair<float, float>> & path,
    double weight_data, double weight_smooth, double tolerance);

protected:
  nav2_costmap_2d::Costmap2D * costmap_;
  nav2_costmap_2d::Costmap2D * local_costmap_;
  std::string frame_id_;
  std::string local_frame_id_;
  bool initialized_;
  bool allow_unknown_;

  bool obstacle_exist;
  bool first_line_plan;
  int res_index;
  rclcpp::Time begin;
  std::vector<geometry_msgs::msg::PoseStamped> linePath;
  geometry_msgs::msg::PoseStamped iniGoal;
  geometry_msgs::msg::PoseStamped iniStart;
  double neardis;
  int status;
  bool endOcc;
  double distance_convert_line;
  double point_per_meter;
  double distance_check_obstacle;
  double distance_behind_obstacle;
  int lethal_cost;
  double wait_time;
  bool enable_dwa_obstacle_avoidance;
  bool flag;

private:
  void initializePlannerObjects();
  void resetPlannerObjects();
  void mapToWorld(double mx, double my, double & wx, double & wy);
  bool worldToMap(double wx, double wy, double & mx, double & my);
  void clearRobotCell(
    const geometry_msgs::msg::PoseStamped & global_pose,
    unsigned int mx, unsigned int my);
  void publishPotential(float * potential);
  void outlineMap(
    unsigned char * costarr, int ini_x, int ini_y,
    int nx, int ny, unsigned char value);
  void makePlanService(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  double planner_window_x_;
  double planner_window_y_;
  double default_tolerance_;
  std::mutex mutex_;

  PotentialCalculator * p_calc_;
  Expander * planner_;
  Traceback * path_maker_;
  OrientationFilter * orientation_filter_;
  bool publish_potential_;
  int publish_scale_;
  float * potential_array_;
  bool old_navfn_behavior_;
  float convert_offset_;
  bool outline_map_;

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    potential_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    tmp_costmap_pub_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr make_plan_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
};

}  // namespace myglobal_planner

#endif  // MYGLOBAL_PLANNER__PLANNER_CORE_H_
