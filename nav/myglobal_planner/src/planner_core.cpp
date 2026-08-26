/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <nav2_costmap_2d/cost_values.hpp>
#include <myglobal_planner/astar.h>
#include <myglobal_planner/dijkstra.h>
#include <myglobal_planner/gradient_path.h>
#include <myglobal_planner/grid_path.h>
#include <myglobal_planner/planner_core.h>
#include <myglobal_planner/quadratic_calculator.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#define PI 3.1415926

using namespace std;

// Register this planner as a Nav2 GlobalPlanner plugin.
PLUGINLIB_EXPORT_CLASS(myglobal_planner::MyGlobalPlanner,
                       nav2_core::GlobalPlanner)

namespace myglobal_planner
{

  void MyGlobalPlanner::outlineMap(
      unsigned char *costarr,int ini_x,int ini_y, int nx, int ny,
      unsigned char value)
  { //使得costmap的四条边为障碍物
    unsigned char *pc = costarr+ini_y*nx+ini_x;
    for (int i = ini_x; i < nx-ini_x; i++)
      *pc++ = value;
    pc = costarr + (ny - 1-ini_y) * nx+ini_x;
    for (int i = ini_x; i < nx-ini_x; i++)
      *pc++ = value;
    pc = costarr+ini_y*nx+ini_x;
    for (int i = ini_y; i < ny-ini_y; i++, pc += nx)
      *pc = value;
    pc = costarr + ini_y*nx+nx - 1-ini_x;
    for (int i = ini_y; i < ny-ini_y; i++, pc += nx)
      *pc = value;
  }

  MyGlobalPlanner::MyGlobalPlanner()
      : costmap_(nullptr), local_costmap_(nullptr), initialized_(false),
        allow_unknown_(false), obstacle_exist(false), first_line_plan(true),
        res_index(10), begin(0, 0, RCL_ROS_TIME), neardis(1.0), status(0),
        endOcc(false), distance_convert_line(0.5), point_per_meter(0.1),
        distance_check_obstacle(1.0), distance_behind_obstacle(1.0),
        lethal_cost(253), wait_time(3.0), enable_dwa_obstacle_avoidance(false),
        flag(false), planner_window_x_(0.0), planner_window_y_(0.0),
        default_tolerance_(0.0), p_calc_(nullptr), planner_(nullptr),
        path_maker_(nullptr), orientation_filter_(nullptr),
        publish_potential_(true), publish_scale_(100), potential_array_(nullptr),
        old_navfn_behavior_(false), convert_offset_(0.5), outline_map_(true),
        logger_(rclcpp::get_logger("MyGlobalPlanner")) {}

  MyGlobalPlanner::~MyGlobalPlanner()
  {
    resetPlannerObjects();
  }

  void MyGlobalPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    auto node = parent.lock();
    if (!node)
    {
      throw std::runtime_error("Failed to lock lifecycle node for MyGlobalPlanner");
    }
    if (!costmap_ros || !costmap_ros->getCostmap())
    {
      throw std::runtime_error("MyGlobalPlanner received an invalid costmap");
    }

    node_ = parent;
    name_ = std::move(name);
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();
    frame_id_ = costmap_ros_->getGlobalFrameID();

    // Nav2 only supplies the global costmap here. mxb_move_base injects the
    // controller costmap later through setLocalCostmap().
    local_costmap_ros_ = costmap_ros_;
    local_costmap_ = costmap_;
    local_frame_id_ = frame_id_;
    begin = rclcpp::Time(0, 0, clock_->get_clock_type());

    const auto prefix = name_ + ".";
    auto declare = [&node, &prefix](const std::string &key, const auto &value) {
      nav2_util::declare_parameter_if_not_declared(
        node, prefix + key, rclcpp::ParameterValue(value));
    };
    declare("old_navfn_behavior", false);
    declare("use_quadratic", false);
    declare("use_dijkstra", false);
    declare("use_grid_path", false);
    declare("allow_unknown", false);
    declare("planner_window_x", 0.0);
    declare("planner_window_y", 0.0);
    declare("default_tolerance", 0.0);
    declare("publish_scale", 100);
    declare("outline_map", true);
    declare("lethal_cost", 253);
    declare("neutral_cost", 10);
    declare("cost_factor", 3.0);
    declare("publish_potential", true);
    declare("orientation_mode", 1);
    declare("orientation_window_size", 1);
    declare("distance_convert_line", 0.5);
    declare("point_per_meter", 0.1);
    declare("distance_check_obstacle", 1.0);
    declare("distance_behind_obstacle", 1.0);
    declare("wait_time", 3.0);
    declare("enable_dwa_obstacle_avoidance", false);

    node->get_parameter(prefix + "old_navfn_behavior", old_navfn_behavior_);
    node->get_parameter(prefix + "allow_unknown", allow_unknown_);
    node->get_parameter(prefix + "planner_window_x", planner_window_x_);
    node->get_parameter(prefix + "planner_window_y", planner_window_y_);
    node->get_parameter(prefix + "default_tolerance", default_tolerance_);
    node->get_parameter(prefix + "publish_scale", publish_scale_);
    node->get_parameter(prefix + "outline_map", outline_map_);
    node->get_parameter(prefix + "lethal_cost", lethal_cost);
    node->get_parameter(prefix + "publish_potential", publish_potential_);
    node->get_parameter(prefix + "distance_convert_line", distance_convert_line);
    node->get_parameter(prefix + "point_per_meter", point_per_meter);
    node->get_parameter(prefix + "distance_check_obstacle", distance_check_obstacle);
    node->get_parameter(prefix + "distance_behind_obstacle", distance_behind_obstacle);
    node->get_parameter(prefix + "wait_time", wait_time);
    node->get_parameter(
      prefix + "enable_dwa_obstacle_avoidance", enable_dwa_obstacle_avoidance);
    convert_offset_ = old_navfn_behavior_ ? 0.0 : 0.5;

    initializePlannerObjects();
    int neutral_cost = 10;
    double cost_factor = 3.0;
    int orientation_mode = 1;
    int orientation_window_size = 1;
    node->get_parameter(prefix + "neutral_cost", neutral_cost);
    node->get_parameter(prefix + "cost_factor", cost_factor);
    node->get_parameter(prefix + "orientation_mode", orientation_mode);
    node->get_parameter(prefix + "orientation_window_size", orientation_window_size);
    planner_->setLethalCost(lethal_cost);
    path_maker_->setLethalCost(lethal_cost);
    planner_->setNeutralCost(neutral_cost);
    planner_->setFactor(cost_factor);
    planner_->setHasUnknown(allow_unknown_);
    orientation_filter_->setMode(orientation_mode);
    orientation_filter_->setWindowSize(orientation_window_size);

    plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "~/" + name_ + "/plan", rclcpp::QoS(1));
    potential_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/" + name_ + "/potential", rclcpp::QoS(1));
    tmp_costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/temp_costmap", rclcpp::QoS(1));
    make_plan_srv_ = node->create_service<nav_msgs::srv::GetPlan>(
      "~/" + name_ + "/make_plan",
      std::bind(
        &MyGlobalPlanner::makePlanService, this,
        std::placeholders::_1, std::placeholders::_2));
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &MyGlobalPlanner::dynamicParametersCallback, this,
        std::placeholders::_1));

    first_line_plan = true;
    iniStart.header.frame_id.clear();
    initialized_ = true;
    RCLCPP_INFO(
      logger_, "MyGlobalPlanner configured in %s; delayed DWA avoidance is %s.",
      frame_id_.c_str(), enable_dwa_obstacle_avoidance ? "enabled" : "disabled");
  }

  void MyGlobalPlanner::initializePlannerObjects()
  {
    resetPlannerObjects();
    auto node = node_.lock();
    const auto prefix = name_ + ".";
    bool use_quadratic = false;
    bool use_dijkstra = false;
    bool use_grid_path = false;
    node->get_parameter(prefix + "use_quadratic", use_quadratic);
    node->get_parameter(prefix + "use_dijkstra", use_dijkstra);
    node->get_parameter(prefix + "use_grid_path", use_grid_path);
    const auto cx = local_costmap_->getSizeInCellsX();
    const auto cy = local_costmap_->getSizeInCellsY();
    p_calc_ = use_quadratic ?
      static_cast<PotentialCalculator *>(new QuadraticCalculator(cx, cy)) :
      new PotentialCalculator(cx, cy);
    if (use_dijkstra) {
      auto * dijkstra = new DijkstraExpansion(p_calc_, cx, cy);
      if (!old_navfn_behavior_) {
        dijkstra->setPreciseStart(true);
      }
      planner_ = dijkstra;
    } else {
      planner_ = new AStarExpansion(p_calc_, cx, cy);
    }
    path_maker_ = use_grid_path ?
      static_cast<Traceback *>(new GridPath(p_calc_)) :
      new GradientPath(p_calc_);
    orientation_filter_ = new OrientationFilter();
  }

  void MyGlobalPlanner::resetPlannerObjects()
  {
    delete[] potential_array_;
    potential_array_ = nullptr;
    delete orientation_filter_;
    orientation_filter_ = nullptr;
    delete path_maker_;
    path_maker_ = nullptr;
    delete planner_;
    planner_ = nullptr;
    delete p_calc_;
    p_calc_ = nullptr;
  }

  void MyGlobalPlanner::setLocalCostmap(
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &local_costmap_ros)
  {
    if (!local_costmap_ros || !local_costmap_ros->getCostmap()) {
      throw std::invalid_argument("MyGlobalPlanner local costmap is invalid");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    local_costmap_ros_ = local_costmap_ros;
    local_costmap_ = local_costmap_ros_->getCostmap();
    local_frame_id_ = local_costmap_ros_->getGlobalFrameID();
    RCLCPP_INFO(
      logger_, "MyGlobalPlanner local obstacle costmap set to frame %s.",
      local_frame_id_.c_str());
  }

  void MyGlobalPlanner::activate()
  {
    plan_pub_->on_activate();
    potential_pub_->on_activate();
    tmp_costmap_pub_->on_activate();
  }

  void MyGlobalPlanner::deactivate()
  {
    plan_pub_->on_deactivate();
    potential_pub_->on_deactivate();
    tmp_costmap_pub_->on_deactivate();
  }

  void MyGlobalPlanner::cleanup()
  {
    initialized_ = false;
    dyn_params_handler_.reset();
    make_plan_srv_.reset();
    plan_pub_.reset();
    potential_pub_.reset();
    tmp_costmap_pub_.reset();
    resetPlannerObjects();
    local_costmap_ = nullptr;
    costmap_ = nullptr;
    local_costmap_ros_.reset();
    costmap_ros_.reset();
    tf_.reset();
    clock_.reset();
  }

  nav_msgs::msg::Path MyGlobalPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = clock_->now();
    makePlan(start, goal, path.poses);
    return path;
  }

  rcl_interfaces::msg::SetParametersResult
  MyGlobalPlanner::dynamicParametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const auto prefix = name_ + ".";
    for (const auto &parameter : parameters) {
      const auto &key = parameter.get_name();
      try {
        if (key == prefix + "lethal_cost") {
          const auto value = parameter.as_int();
          if (value < 1 || value > 255) {
            throw std::out_of_range("lethal_cost must be in [1, 255]");
          }
          lethal_cost = static_cast<int>(value);
          planner_->setLethalCost(lethal_cost);
          path_maker_->setLethalCost(lethal_cost);
        } else if (key == prefix + "neutral_cost") {
          const auto value = parameter.as_int();
          if (value < 1 || value > 255) {
            throw std::out_of_range("neutral_cost must be in [1, 255]");
          }
          planner_->setNeutralCost(static_cast<unsigned char>(value));
        } else if (key == prefix + "cost_factor") {
          const auto value = parameter.as_double();
          if (value < 0.01 || value > 5.0) {
            throw std::out_of_range("cost_factor must be in [0.01, 5.0]");
          }
          planner_->setFactor(value);
        } else if (key == prefix + "publish_potential") {
          publish_potential_ = parameter.as_bool();
        } else if (key == prefix + "orientation_mode") {
          const auto value = parameter.as_int();
          if (value < 0 || value > 6) {
            throw std::out_of_range("orientation_mode must be in [0, 6]");
          }
          orientation_filter_->setMode(static_cast<int>(value));
        } else if (key == prefix + "orientation_window_size") {
          const auto value = parameter.as_int();
          if (value < 1 || value > 255) {
            throw std::out_of_range("orientation_window_size must be in [1, 255]");
          }
          orientation_filter_->setWindowSize(static_cast<size_t>(value));
        } else if (key == prefix + "distance_convert_line") {
          const auto value = parameter.as_double();
          if (!std::isfinite(value) || value < 0.2 || value > 1.0) {
            throw std::out_of_range("distance_convert_line must be in [0.2, 1.0]");
          }
          distance_convert_line = value;
        } else if (key == prefix + "distance_check_obstacle") {
          const auto value = parameter.as_double();
          if (!std::isfinite(value) || value < 0.5 || value > 2.0) {
            throw std::out_of_range("distance_check_obstacle must be in [0.5, 2.0]");
          }
          distance_check_obstacle = value;
        } else if (key == prefix + "distance_behind_obstacle") {
          const auto value = parameter.as_double();
          if (!std::isfinite(value) || value < 0.5 || value > 2.0) {
            throw std::out_of_range("distance_behind_obstacle must be in [0.5, 2.0]");
          }
          distance_behind_obstacle = value;
        } else if (key == prefix + "wait_time") {
          const auto value = parameter.as_double();
          if (!std::isfinite(value) || value < 1.0 || value > 10.0) {
            throw std::out_of_range("wait_time must be in [1.0, 10.0]");
          }
          wait_time = value;
        } else if (key == prefix + "enable_dwa_obstacle_avoidance") {
          enable_dwa_obstacle_avoidance = parameter.as_bool();
        }
      } catch (const std::exception &error) {
        result.successful = false;
        result.reason = error.what();
        return result;
      }
    }
    return result;
  }

  void MyGlobalPlanner::clearRobotCell(
      const geometry_msgs::msg::PoseStamped &global_pose, unsigned int mx,
      unsigned int my)
  {
    (void)global_pose;
    if (!initialized_)
    {
      RCLCPP_ERROR(logger_,
          "This planner has not been initialized yet, but it is being used, "
          "please call initialize() before use");
      return;
    }

    // set the associated costs in the cost map to be free
    local_costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  }

  void MyGlobalPlanner::makePlanService(
      const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
      std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
  {
    makePlan(request->start, request->goal, request->tolerance, response->plan.poses);
    response->plan.header.stamp = clock_->now();
    response->plan.header.frame_id = frame_id_;
  }

  void MyGlobalPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
  {
    wx = local_costmap_->getOriginX() +
         (mx + convert_offset_) * local_costmap_->getResolution();
    wy = local_costmap_->getOriginY() +
         (my + convert_offset_) * local_costmap_->getResolution();
  }

  bool MyGlobalPlanner::worldToMap(double wx, double wy, double &mx, double &my)
  {
    double origin_x = local_costmap_->getOriginX(), origin_y = local_costmap_->getOriginY();
    double resolution = local_costmap_->getResolution();
    if (wx < origin_x || wy < origin_y)
      return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < local_costmap_->getSizeInCellsX() && my < local_costmap_->getSizeInCellsY())
      return true;

    return false;
  }

  void MyGlobalPlanner::mapToOdom(geometry_msgs::msg::PoseStamped map_pose,
                                  geometry_msgs::msg::PoseStamped &odom_pose)
  {
    map_pose.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());

    //用于将map坐标系下的轨迹坐标转化成odom坐标系下
    try
    {
      odom_pose = tf_->transform(
        map_pose, local_frame_id_, tf2::durationFromSec(3.0));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(logger_, "transform exception: %s", ex.what());
    }
  }
  void MyGlobalPlanner::odomToMap(geometry_msgs::msg::PoseStamped odom_pose,
                                  geometry_msgs::msg::PoseStamped &map_pose)
  {
    //用于将odom坐标系下的轨迹坐标转化成map坐标系下
    try
    {
      odom_pose.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
      map_pose = tf_->transform(
        odom_pose, frame_id_, tf2::durationFromSec(3.0));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(logger_, "transform exception: %s", ex.what());
    }
  }
  bool MyGlobalPlanner::pointCheck(unsigned int x, unsigned int y)
  {
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        if (x + i > 0 && x + i < local_costmap_->getSizeInCellsX() && y + j > 0 &&
            y + j < local_costmap_->getSizeInCellsY())
        {
          if (i == 0 && j == 0)
          {
            continue;
          }
          else
          {
            if ((int)local_costmap_->getCost(x + i, y + j) >= lethal_cost)
            {
              return true; //八领域存在一点为障碍物则表示该点为障碍物，此函数是为了防止噪声的干扰
            }
          }
        }
      }
    }
    return false;
  }
  double MyGlobalPlanner::comDistance(geometry_msgs::msg::PoseStamped p,
                                      geometry_msgs::msg::PoseStamped q)
  {
    double distance = sqrt((p.pose.position.x - q.pose.position.x) *
                               (p.pose.position.x - q.pose.position.x) +
                           (p.pose.position.y - q.pose.position.y) *
                               (p.pose.position.y - q.pose.position.y));
    return distance;
  }
  void MyGlobalPlanner::obsCheck(geometry_msgs::msg::PoseStamped start)
  {
    if(status==0){
      begin = clock_->now();
    }
    obstacle_exist = false;
    endOcc=false;
    flag=false;
    int index = 0;
    double min = 1000;
    int ind=0;
    //从直线找出一个离机器人最近的点
    for (std::size_t i = 0; i < linePath.size(); i++)
    {
      double distence = comDistance(start, linePath[i]);
      if (distence < min)
      {
        min = distence;
        ind = i;
      }
    }
    geometry_msgs::msg::PoseStamped cur_pose;
    geometry_msgs::msg::PoseStamped temp_pose;
    // double dis = state.v * state.v / (2 * sp->ata);

    int end_index = 0; //当前costmap下机器人看到障碍物在直线上消失的临界坐标

    //代表机器人当前位置在ind，我们只需要检查ind后面的点是否存在障碍物
    cur_pose.header.stamp = clock_->now();
    cur_pose.header.frame_id = frame_id_;
    cur_pose.pose.position.x = start.pose.position.x;
    cur_pose.pose.position.y = start.pose.position.y;
    while (ind + index < static_cast<int>(linePath.size()) - 1 && rclcpp::ok())
    {
      geometry_msgs::msg::PoseStamped map_pose;
      geometry_msgs::msg::PoseStamped map_next;
      map_pose = linePath[ind + index];
      map_pose.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
      map_next = linePath[ind + index + 1];
      map_next.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());

      //转化到local_costmap坐标
      unsigned int local_x, local_y, next_x, next_y;
      geometry_msgs::msg::PoseStamped odom_pose;
      geometry_msgs::msg::PoseStamped odom_next;
      mapToOdom(map_pose, odom_pose);

      if (local_costmap_->worldToMap(odom_pose.pose.position.x,
                                     odom_pose.pose.position.y, local_x,
                                     local_y))
      { //该点处于局部窗口内

        if ((int)local_costmap_->getCost(local_x, local_y) >= lethal_cost && pointCheck(local_x, local_y)) //当该点costmap值大于等于253时且其八领域上的点也存在一个值大于等于253则代表该点为障碍物
        {
          double distance = comDistance(cur_pose, map_pose);
          if(distance<0.5)
            flag=true;
          if (distance <=distance_check_obstacle) //该障碍物点与机器人的位置小于1m时将obstacle_exist置为true
          {
            obstacle_exist = true;
          }
          if (obstacle_exist)
          {
            mapToOdom(map_next, odom_next);
            if (local_costmap_->worldToMap(odom_next.pose.position.x,
                                           odom_next.pose.position.y, next_x,
                                           next_y))
            {
              if ((int)local_costmap_->getCost(next_x, next_y) < lethal_cost) //找出障碍物在直线上消失的临界点,该点为障碍物，但是下一个点不是障碍物
              {
                end_index = ind + index;
                break;
              }
              else if (ind + index >= static_cast<int>(linePath.size()) - 2)
              {
                endOcc=true;
                res_index=ind+index;

                RCLCPP_ERROR(logger_, "there are a obstacle in  endpoint");

                return;
              }
            }
            else
            {
              end_index =ind + index; //假如在当前costmap范围内未找到消失临界点，我们则将costmap边界与直线的交点作为end_index
            }
          }
        }
      }
      else
      {
        break; //后方点更不可能在窗口内，故直接退出循环
      }
      index++;
    }

    if (obstacle_exist &&end_index != 0) //当存在障碍物且end_index存在时，找出end_index后方一米处的下标res_index，方便对动态障碍物或者长障碍物进行res_index下标更新
    {
      double app_dis = 10;
      res_index = end_index;
      for (int j = end_index; j < static_cast<int>(linePath.size()); j++)
      {
        if (fabs(comDistance(linePath[j], linePath[end_index]) - distance_behind_obstacle) < app_dis)
        {
          app_dis = fabs(comDistance(linePath[j], linePath[end_index]) - distance_behind_obstacle);
          res_index = j;
        }
      }
    }
    RCLCPP_INFO_STREAM(logger_, "res_index:" << res_index );
    return;
  }
  bool MyGlobalPlanner::makePlanLine(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal, double tolerance,
      std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    (void)tolerance;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_)
    {
      RCLCPP_ERROR(logger_,
          "This planner has not been initialized yet, but it is being used, "
          "please call initialize() before use");
      return false;
    }

    // clear the plan, just in case
    plan.clear();
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    unsigned int startx, starty, goalx, goaly;
    if (!costmap_->worldToMap(wx, wy, startx, starty))
    {
      RCLCPP_WARN(logger_,
          "The robot's start position is off the global costmap. Planning will "
          "always fail, are you sure the robot has been properly localized?");
      return false;
    }
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    if (!costmap_->worldToMap(wx, wy, goalx, goaly))
    {
      RCLCPP_WARN(logger_,
          "The robot's start position is off the global costmap. Planning will "
          "always fail, are you sure the robot has been properly localized?");
      return false;
    }
    costmap_->setCost(startx, starty, nav2_costmap_2d::FREE_SPACE);
    double start_x = startx;
    double start_y = starty;
    double goal_x = goalx;
    double goal_y = goaly;
    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double scale = 0;
    double distance = costmap_->getResolution() * sqrt(diff_x * diff_x + diff_y * diff_y);
    double dscale = point_per_meter/distance; //一米十个点，两点之间线越长，中间的点越多

    while (scale < 1)
    {
      double target_x = start_x + scale * diff_x;
      double target_y = start_y + scale * diff_y;
      if (target_x < costmap_->getSizeInCellsX() &&                                   //处于局部窗口
          target_y < costmap_->getSizeInCellsY() && target_x > 0 &&
          target_y > 0)
      {
        /*if ((int)costmap_->getCost(target_x, target_y) < lethal_cost)
        {
          codPath(target_x, target_y, plan);
        }
        else
        {
          RCLCPP_ERROR(logger_,  "There are obstacles in the straight line, please reset the target point " );

          return false;
        }*/
        codPath(target_x, target_y, plan);//mode by jgl 20250528, remove target check
      }
      scale = scale + dscale;
    }
    geometry_msgs::msg::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = clock_->now();
    plan.push_back(goal_copy);
    return !plan.empty();
  }

  bool MyGlobalPlanner::comparePose(geometry_msgs::msg::PoseStamped p,
                                    geometry_msgs::msg::PoseStamped q)
  {
    if (p.pose.position.x != q.pose.position.x)
    {
      return false;
    }
    if (p.pose.position.y != q.pose.position.y)
    {
      return false;
    }
    if (p.pose.orientation.x != q.pose.orientation.x)
    {
      return false;
    }
    if (p.pose.orientation.y != q.pose.orientation.y)
    {
      return false;
    }
    if (p.pose.orientation.z != q.pose.orientation.z)
    {
      return false;
    }
    if (p.pose.orientation.w != q.pose.orientation.w)
    {
      return false;
    }
    return true;
  }
  void MyGlobalPlanner::codPath(double xx, double yy,
                                std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    rclcpp::Time plan_time = clock_->now();
    std::string global_frame = frame_id_;
    double world_x, world_y;
    world_x = xx * costmap_->getResolution() + costmap_->getOriginX();
    world_y = yy * costmap_->getResolution() + costmap_->getOriginY();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = plan_time;

    pose.header.frame_id = global_frame;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;

    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
  void MyGlobalPlanner::stateSwitch()
  {
    if (status == 0 && obstacle_exist && !endOcc)//当前为状态为0，障碍物存在且该障碍物没有占据终点进入状态1，进行原地等待
    {
      status = 1;
      return;
    }
    if (status == 1 && !obstacle_exist)//当前状态为1，且障碍物消失
    {
      status = 0;
      return;
    }
    if (status == 1 && begin + rclcpp::Duration::from_seconds(wait_time) < clock_->now())//当前状态为1，且原地等待已超过wait_time秒
    {
      if (!enable_dwa_obstacle_avoidance)
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Obstacle is still present after %.1f seconds; keep stopping and let AutoNAV replan.",
          wait_time);
        return;
      }
      RCLCPP_INFO(logger_, "The obstacle is still there after waiting for %.1f seconds; switch to DWA avoidance.",
               wait_time);
      status = 2;
      return;
    }
    if (status == 2 && neardis <= distance_convert_line)//当前状态为2，且机器人当前位置距离目标点距离小于指定距离则转化为状态0
    {
      status = 0;
      return;
    }
    if (status == 0 && endOcc)
    {
      status = 3;
      return;
    }
    if (status == 2 && endOcc)
    {
      status = 4;
      return;
    }
    if (status == 3 && !endOcc)
    {
      status = 0;
      return;
    }
    if (status == 4 && !endOcc)
    {
       RCLCPP_INFO(logger_, " the obstacle is still there after waiting for 3 seconds " );
      status = 2;
      return;
    }
  }
  bool MyGlobalPlanner::dealState(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal,std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    switch (status)
    {
    case 0:
    {
      for (const auto &line_pose : linePath)
      {
        geometry_msgs::msg::PoseStamped pose = line_pose;
        pose.pose.position.z = 1; //纯追踪的标记
        plan.push_back(pose);
      }

      break;
    }
    case 1:
    {
      RCLCPP_INFO(logger_, "There is an obstacle,please wait for 3s");
      for (const auto &line_pose : linePath)
      {
        geometry_msgs::msg::PoseStamped pose = line_pose;
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }
    case 2:
    {


      geometry_msgs::msg::PoseStamped tempGoal = linePath[res_index];
      neardis=comDistance(start,tempGoal);
      bool gotGlobal = makePlan(start, tempGoal, default_tolerance_, plan);
      if (!gotGlobal)
      {
        return false;
      }
      if(!planCheck(start,goal,plan))
      {
        RCLCPP_ERROR(logger_, "the direction of plan in contrast with the direction of start to goal,or the length of  plan is too long");
        return false;
      }
      break;
    }
    case 3:
    {
      RCLCPP_INFO(logger_, "There is an obstacle in endpoint,please wait for 3s");
      for (const auto &line_pose : linePath)
      {
        geometry_msgs::msg::PoseStamped pose = line_pose;
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }
    case 4:
    {
      RCLCPP_INFO(logger_, "There is an obstacle in endpoint,please wait for 3s");
      for (const auto &line_pose : linePath)
      {
        geometry_msgs::msg::PoseStamped pose = line_pose;
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }

    }
    return true;
  }
  bool MyGlobalPlanner::makePlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal,
                                 std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    //return makePlan(start, goal, default_tolerance_, plan);
    plan.clear();
    const double distance_to_goal = comDistance(start, goal);
    const double terminal_replan_freeze_distance = 0.15;
    const bool start_changed =
        distance_to_goal > terminal_replan_freeze_distance &&
        (iniStart.header.frame_id.empty() || comDistance(iniStart, start) > 0.02);
    if (start_changed || !comparePose(iniGoal, goal))//当传进来的goal或start与上次不一致时，重新进行直线规划
    {
      RCLCPP_INFO(logger_, "new goal or start" );
      iniGoal = goal;
      iniStart = start;
      first_line_plan = true;
    }

    if (first_line_plan)
    {
      RCLCPP_INFO(logger_, "Straight-line planning for the first time ");
      bool gotLine = makePlanLine(start, goal, default_tolerance_, linePath);
      iniGoal = goal;
      iniStart = start;
      status = 0;

      if (!gotLine)
      {
        return false;
      }
      first_line_plan = false;                      //只有在传进来的goal不一致时，才进行直线规划
    }


    //障碍物检测
    obsCheck(start);
    //根据当前状态和障碍物情况进行状态切换
    stateSwitch();
    RCLCPP_INFO_STREAM(logger_, "LINE_PLANNER STATUS:  "<<status);
    //不同状态处理
    if(!dealState(start,goal,plan))
    {
      return false;
    }
    publishPlan(plan);
    return !plan.empty();
  }


  bool MyGlobalPlanner::planCheck(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal, std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    if(plan.size()<40)
      return true;
    //判断路径上前0.5米是否往后

    double yaw=atan2(goal.pose.position.y-start.pose.position.y,goal.pose.position.x-start.pose.position.x);

    int count=0;
    for(int i=1;i<40;i++)
    {
      double errYaw=atan2(plan[i].pose.position.y-start.pose.position.y,plan[i].pose.position.x-start.pose.position.x)-yaw;

      if(errYaw<=-PI){
        errYaw=errYaw+2*PI;
      }else if(errYaw>=PI){
          errYaw=errYaw-2*PI;
      }

      if(fabs(errYaw*180/PI)>90)
      {
        count++;
      }

    }


    int distance=comDistance(start,goal)/costmap_->getResolution();
    double sum=0;
    for(std::size_t i = 0; i + 1 < plan.size(); i++)
    {
        sum=sum+comDistance(plan[i],plan[i+1]);
    }
    sum=sum/costmap_->getResolution();
    std::cout<<"count,distance,length:"<<count<<","<<distance<<","<<sum<<std::endl;
    //modify to back up
    // if(count>10||plan.size()>3*distance)
    // {
    //     return false;
    // }

    if(count > 25 || plan.size() > static_cast<std::size_t>(3 * distance))    //long
    {
        return false;
    }
    if((count>10&&count<=25)||flag)               //short
    {
        for(auto &pose : plan)
        {
            pose.pose.position.z=-1;
        }
        return true;
    }


    return true;
  }
  void MyGlobalPlanner::setGoal(double &goal_x,double &goal_y,const geometry_msgs::msg::PoseStamped &start,unsigned char* master_array,int nx,int ny)
  {
    double min = 1000;
    int ind = 0;
    //从直线找出一个离机器人最近的点
    for (std::size_t j = 0; j < linePath.size(); j++)
    {
      double distence = comDistance(start, linePath[j]);
      if (distence < min)
      {
        min = distence;
        ind = static_cast<int>(j);
      }
    }
    for(int i = ind; i < static_cast<int>(linePath.size()); i++)
    {
      geometry_msgs::msg::PoseStamped odomPose;
      mapToOdom(linePath[i],odomPose);
      double local_x,local_y;
      if(worldToMap(odomPose.pose.position.x,odomPose.pose.position.y,local_x,local_y))
      {


        if(fabs(local_x-nx+1)<=2)
        {
          goal_x=nx-1;
          goal_y=(int)local_y;
          unsigned char*pc=master_array+nx+nx-2;
          for(int i=1;i<nx-1;i++,pc+=nx)
          {
            *pc=nav2_costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+nx-3;
          for(int i=2;i<nx-2;i++,pc+=nx)
          {
            *pc=nav2_costmap_2d::FREE_SPACE;
          }
          break;
        }
        if(fabs(local_x-1)<=2)
        {
          goal_x=1;
          goal_y=(int)local_y;
          unsigned char*pc=master_array+nx+1;
          for(int i=1;i<nx-1;i++,pc+=nx)
          {
            *pc=nav2_costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+2;
          for(int i=2;i<nx-2;i++,pc+=nx)
          {
            *pc=nav2_costmap_2d::FREE_SPACE;
          }
          break;
        }

        if(fabs(local_y-ny+1)<=2)
        {
          goal_x=(int)local_x;
          goal_y=ny-1;
          unsigned char*pc=master_array+(ny-2)*nx+2;
          for(int i=1;i<nx-1;i++)
          {
            *pc++=nav2_costmap_2d::FREE_SPACE;
          }
          pc=master_array+(ny-3)*nx+1;
          for(int i=2;i<nx-2;i++)
          {
            *pc++=nav2_costmap_2d::FREE_SPACE;
          }
          break;
        }
        if(fabs(local_y-1)<=2)
        {
          goal_x=(int)local_x;
          goal_y=1;
          unsigned char*pc=master_array+nx+1;
          for(int i=1;i<nx-1;i++)
          {
            *pc++=nav2_costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+2;
          for(int i=2;i<nx-2;i++)
          {
            *pc++=nav2_costmap_2d::FREE_SPACE;
          }
          break;
        }
      }
    }
  }


  bool MyGlobalPlanner::makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
                                 double tolerance, std::vector<geometry_msgs::msg::PoseStamped> &plan)
    {
        (void)tolerance;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_)
        {
            RCLCPP_ERROR(logger_,
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        //clear the plan, just in case
        plan.clear();

        std::string global_frame = local_frame_id_;

        //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
        if (goal.header.frame_id != global_frame)
        {
            RCLCPP_ERROR(logger_,
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != global_frame)
        {
            RCLCPP_ERROR(logger_,
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
            return false;
        }



        geometry_msgs::msg::PoseStamped startOdom;
        geometry_msgs::msg::PoseStamped goalOdom;
        mapToOdom(start,startOdom);
        mapToOdom(goal,goalOdom);

        double wx = startOdom.pose.position.x;
        double wy = startOdom.pose.position.y;

        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        double start_x, start_y, goal_x, goal_y;

        if (!local_costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
        {
            RCLCPP_WARN(logger_,
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }
        if (old_navfn_behavior_)
        {
            start_x = start_x_i;
            start_y = start_y_i;
        }
        else
        {
            worldToMap(wx, wy, start_x, start_y);
        }

        wx = goalOdom.pose.position.x;
        wy = goalOdom.pose.position.y;
        // The ROS1 customization disabled this guard, but still used goal_x_i and
        // goal_y_i below. An off-window goal therefore caused undefined memory
        // access rather than a usable plan. Reject it explicitly; the caller's
        // straight-line logic selects a local detour endpoint before reaching here.
        if (!local_costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
        {
            RCLCPP_WARN(logger_,
                "The goal sent to the local planner is outside its costmap window");
            return false;
        }
        if (old_navfn_behavior_)
        {
            goal_x = goal_x_i;
            goal_y = goal_y_i;
        }
        else
        {
            worldToMap(wx, wy, goal_x, goal_y);
        }


        RCLCPP_INFO_STREAM(logger_, "goal:"<<goal_x<<","<<goal_y);

        //clear the starting cell within the costmap because we know it can't be an obstacle
        clearRobotCell(start, start_x_i, start_y_i);

        int nx = local_costmap_->getSizeInCellsX(), ny = local_costmap_->getSizeInCellsY();

        //make sure to resize the underlying array that Navfn uses
        p_calc_->setSize(nx, ny);
        planner_->setSize(nx, ny);
        path_maker_->setSize(nx, ny);
        potential_array_ = new float[nx * ny];

        if (outline_map_)
            outlineMap(local_costmap_->getCharMap(),0,0, nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);

        bool found_legal = planner_->calculatePotentials(local_costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                         nx * ny * 2, potential_array_);

        if (!old_navfn_behavior_)
            planner_->clearEndpoint(local_costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
        if (publish_potential_)
            publishPotential(potential_array_);

        if (found_legal)
        {
            //extract the plan
            if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan))
            {
                //make sure the goal we push on has the same timestamp as the rest of the plan
                geometry_msgs::msg::PoseStamped goal_copy = goal;
                goal_copy.header.stamp = clock_->now();
                plan.push_back(goal_copy);
            }
            else
            {
                RCLCPP_ERROR(logger_, "Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to get a plan.");
        }

        // add orientations if needed
        orientation_filter_->processPath(start, plan);

        //publish the plan for visualization purposes
        publishPlan(plan);
        delete[] potential_array_;
        potential_array_ = nullptr;
        return !plan.empty();
    }

  void MyGlobalPlanner::publishPlan(
      const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(logger_,
          "This planner has not been initialized yet, but it is being used, "
          "please call initialize() before use");
      return;
    }

    // create a message for the plan
    nav_msgs::msg::Path gui_path;

    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = clock_->now();

    // Extract the plan in world co-ordinates, we assume the path is all in the
    // same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
      gui_path.poses[i].pose.position.z = 0;//发布的时候让所有不为0的都变成0以供展示
    }

    plan_pub_->publish(gui_path);
  }
void MyGlobalPlanner::smooth(std::vector<std::pair<float, float> > &path, double weight_data, double weight_smooth, double tolerance)
{
    if (path.size() < 6) {
        return;
    }
    double change = tolerance;
    int Iterations = 0;
    while (change >= tolerance) {
        change = 0.0;
        for (std::size_t i = 2; i + 3 < path.size(); i++) {
            double xtemp = path[i].first;
            double ytemp = path[i].second;

            path[i].first += weight_data * (path[i].first - path[i].first);
             path[i].second += weight_data * ( path[i].second -  path[i].second);

            path[i].first+= weight_smooth * (path[i-1].first + path[i+1].first- (2.0 * path[i].first));
            path[i].second += weight_smooth * (path[i-1].second + path[i+1].second - (2.0 * path[i].second));

            change += fabs(xtemp - path[i].first);
            change += fabs(ytemp -path[i].second);
        }
        Iterations++;
    }
}
 bool MyGlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::msg::PoseStamped& goal,
                                       std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_,
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;
    std::string local_frame=local_frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        RCLCPP_ERROR(logger_, "NO PATH!");
        return false;
    }

    rclcpp::Time plan_time = clock_->now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;


       //需要将局部路径转化到全局路径
        mapToWorld(point.first, point.second, world_x, world_y);
        geometry_msgs::msg::PoseStamped world;
        world.header.stamp=plan_time;
        world.header.frame_id=local_frame;
        world.pose.position.x=world_x;
        world.pose.position.y=world_y;
        world.pose.orientation.w=1;
        geometry_msgs::msg::PoseStamped pose;
        odomToMap(world,pose);
        pose.header.frame_id=global_frame;


        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}


  void MyGlobalPlanner::publishPotential(float *potential)
  {
    (void)potential;
    int nx = local_costmap_->getSizeInCellsX(), ny = local_costmap_->getSizeInCellsY();
    double resolution = local_costmap_->getResolution();
    nav_msgs::msg::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = clock_->now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    local_costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
      float potential = potential_array_[i];
      if (potential < POT_HIGH)
      {
        if (potential > max)
        {
          max = potential;
        }
      }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
      if (potential_array_[i] >= POT_HIGH)
      {
        grid.data[i] = -1;
      }
      else
        grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_->publish(grid);
  }

} // end namespace global_planner
