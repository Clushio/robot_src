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
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <myglobal_planner/astar.h>
#include <myglobal_planner/dijkstra.h>
#include <myglobal_planner/gradient_path.h>
#include <myglobal_planner/grid_path.h>
#include <myglobal_planner/planner_core.h>
#include <myglobal_planner/quadratic_calculator.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926

using namespace std;

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myglobal_planner::MyGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

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
      : costmap_(NULL), initialized_(false), allow_unknown_(true),local_costmap_(NULL) {}

  MyGlobalPlanner::MyGlobalPlanner(std::string name,
                                   costmap_2d::Costmap2D *costmap,
                                   std::string frame_id)
      : costmap_(NULL), initialized_(false), allow_unknown_(true),local_costmap_(NULL)
  {
    // initialize the planner
    initialize(name, costmap, frame_id);
  }

  MyGlobalPlanner::~MyGlobalPlanner()
  {
    if (p_calc_)
      delete p_calc_;
    if (planner_)
      delete planner_;
    if (path_maker_)
      delete path_maker_;
    if (dsrv_)
      delete dsrv_;
  }

  void MyGlobalPlanner::initialize(std::string name,costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (name == "MyGlobalPlanner")                                                        //初始化local_costmap
      initialize(name, costmap_ros->getCostmap(),
                 costmap_ros->getGlobalFrameID());
    else                                                                                                                //初始化全global_ostmap                                                                                     
    {
      costmap_ = costmap_ros->getCostmap();
      frame_id_=costmap_ros->getGlobalFrameID();
    }
  }

  void MyGlobalPlanner::initialize(std::string name,costmap_2d::Costmap2D *costmap,std::string frame_id)
  {
    ROS_INFO("my_global_planner  initializing");

    if (!initialized_)
    {
      ros::NodeHandle private_nh("~/" + name); // planner
      local_costmap_ = costmap;
      local_frame_id_ = frame_id;

      unsigned int cx = costmap->getSizeInCellsX(),
                   cy = costmap->getSizeInCellsY();

      private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);         //是否使用和navfn一致的规划，默认false
      if (!old_navfn_behavior_)
        convert_offset_ = 0.5;
      else
        convert_offset_ = 0.0;

      bool use_quadratic;
      private_nh.param("use_quadratic", use_quadratic, false);
      if (use_quadratic)
        p_calc_ = new QuadraticCalculator(cx, cy);
      else
        p_calc_ = new PotentialCalculator(cx, cy);

      bool use_dijkstra;
      private_nh.param("use_dijkstra", use_dijkstra, false);
      if (use_dijkstra)
      {
        DijkstraExpansion *de = new DijkstraExpansion(p_calc_, cx, cy);
        if (!old_navfn_behavior_)
          de->setPreciseStart(true);
        planner_ = de;
      }
      else
        planner_ = new AStarExpansion(p_calc_, cx, cy);

      bool use_grid_path;
      private_nh.param("use_grid_path", use_grid_path, false);
      if (use_grid_path)
        path_maker_ = new GridPath(p_calc_);
      else
        path_maker_ = new GradientPath(p_calc_);

      orientation_filter_ =new OrientationFilter();        //让路径拐弯的时候角度别变得太快,给路径点加入角度

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      potential_pub_ =private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, false);                       
      planner_->setHasUnknown(allow_unknown_);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);
      private_nh.param("publish_scale", publish_scale_, 100);
      private_nh.param("outline_map", outline_map_, true);

      make_plan_srv_ = private_nh.advertiseService("make_plan", &MyGlobalPlanner::makePlanService, this);

      dsrv_ =new dynamic_reconfigure::Server<myglobal_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
      dynamic_reconfigure::Server<
          myglobal_planner::GlobalPlannerConfig>::CallbackType cb =
          boost::bind(&MyGlobalPlanner::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
      initialized_ = true;

      //add

      

      private_nh.param("lethal_cost", lethal_cost, 253);
      private_nh.param("distance_convert_line", distance_convert_line, 0.5);
      private_nh.param("point_per_meter", point_per_meter, 0.1);
      private_nh.param("distance_check_obstacle", distance_check_obstacle, 1.0);
      private_nh.param("distance_behind_obstacle", distance_behind_obstacle, 1.0);
      private_nh.param("wait_time",wait_time,3.0);
      private_nh.param("enable_dwa_obstacle_avoidance",
                       enable_dwa_obstacle_avoidance, false);
      ROS_INFO("Global planner DWA obstacle avoidance after wait is %s.",
               enable_dwa_obstacle_avoidance ? "enabled" : "disabled");
      first_line_plan = true;
      iniStart.header.frame_id = "";
      neardis=1.0;
      res_index=10;

      tmp_costmap_pub=private_nh.advertise<nav_msgs::OccupancyGrid>("/temp_costmap",1);

      

    }
    else
      ROS_WARN(
          "This planner has already been initialized, you can't call it twice, "
          "doing nothing");
  }

  void MyGlobalPlanner::reconfigureCB(myglobal_planner::GlobalPlannerConfig &config,
                                      uint32_t level)
  {
    planner_->setLethalCost(config.lethal_cost);
    lethal_cost=config.lethal_cost;
    distance_convert_line=config.distance_convert_line;
    distance_check_obstacle=config.distance_check_obstacle;
    distance_behind_obstacle=config.distance_behind_obstacle;
    wait_time=config.wait_time;
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
  }

  void MyGlobalPlanner::clearRobotCell(
      const geometry_msgs::PoseStamped &global_pose, unsigned int mx,
      unsigned int my)
  {
    if (!initialized_)
    {
      ROS_ERROR(
          "This planner has not been initialized yet, but it is being used, "
          "please call initialize() before use");
      return;
    }

    // set the associated costs in the cost map to be free
    local_costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool MyGlobalPlanner::makePlanService(nav_msgs::GetPlan::Request &req,
                                        nav_msgs::GetPlan::Response &resp)
  {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
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

  void MyGlobalPlanner::mapToOdom(geometry_msgs::PoseStamped map_pose,
                                  geometry_msgs::PoseStamped &odom_pose)
  {
    map_pose.header.stamp=ros::Time();

    //用于将map坐标系下的轨迹坐标转化成odom坐标系下
    try
    {
      lis.waitForTransform(frame_id_, local_frame_id_, ros::Time(0), ros::Duration(3.0));
      lis.transformPose(local_frame_id_, map_pose, odom_pose);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("transform exception:%s", ex.what());
    }
  }
  void MyGlobalPlanner::odomToMap(geometry_msgs::PoseStamped odom_pose,
                                  geometry_msgs::PoseStamped &map_pose)
  {
    //用于将odom坐标系下的轨迹坐标转化成map坐标系下
    try
    {
      lis.waitForTransform(local_frame_id_, frame_id_, ros::Time(0), ros::Duration(3.0));
      lis.transformPose(frame_id_, odom_pose, map_pose);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("transform exception:%s", ex.what());
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
  double MyGlobalPlanner::comDistance(geometry_msgs::PoseStamped p,
                                      geometry_msgs::PoseStamped q)
  {
    double distance = sqrt((p.pose.position.x - q.pose.position.x) *
                               (p.pose.position.x - q.pose.position.x) +
                           (p.pose.position.y - q.pose.position.y) *
                               (p.pose.position.y - q.pose.position.y));
    return distance;
  }
  void MyGlobalPlanner::obsCheck(geometry_msgs::PoseStamped start)
  {
    if(status==0){
      begin=ros::Time::now();
    }
    obstacle_exist = false;
    endOcc=false;
    flag=false;
    int index = 0;
    double min = 1000;
    int ind=0;
    //从直线找出一个离机器人最近的点
    for (int i = 0; i < linePath.size(); i++)
    {
      double distence = comDistance(start, linePath[i]);
      if (distence < min)
      {
        min = distence;
        ind = i;
      }
    }
    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::PoseStamped temp_pose;
    // double dis = state.v * state.v / (2 * sp->ata);
               
    int end_index = 0; //当前costmap下机器人看到障碍物在直线上消失的临界坐标

    //代表机器人当前位置在ind，我们只需要检查ind后面的点是否存在障碍物
    cur_pose.header.stamp = ros::Time::now();
    cur_pose.header.frame_id = frame_id_;
    cur_pose.pose.position.x = start.pose.position.x;
    cur_pose.pose.position.y = start.pose.position.y;
    while (ind + index < linePath.size() - 1 && !ros::isShuttingDown())
    {
      geometry_msgs::PoseStamped map_pose;
      geometry_msgs::PoseStamped map_next;
      map_pose = linePath[ind + index];
      map_pose.header.stamp = ros::Time();
      map_next = linePath[ind + index + 1];
      map_next.header.stamp = ros::Time();

      //转化到local_costmap坐标
      unsigned int local_x, local_y, next_x, next_y;
      geometry_msgs::PoseStamped odom_pose;
      geometry_msgs::PoseStamped odom_next;
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
              else if (ind + index >= linePath.size() - 2)
              {
                endOcc=true;
                res_index=ind+index;

                ROS_ERROR("there are a obstacle in  endpoint");
                
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
      for (int j = end_index; j < linePath.size(); j++)
      {
        if (fabs(comDistance(linePath[j], linePath[end_index]) - distance_behind_obstacle) < app_dis)
        {
          app_dis = fabs(comDistance(linePath[j], linePath[end_index]) - distance_behind_obstacle);
          res_index = j;
        }
      }
    }
    ROS_INFO_STREAM("res_index:" << res_index );
    return;
  }
  bool MyGlobalPlanner::makePlanLine(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal, double tolerance,
      std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_)
    {
      ROS_ERROR(
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
      ROS_WARN(
          "The robot's start position is off the global costmap. Planning will "
          "always fail, are you sure the robot has been properly localized?");
      return false;
    }
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    if (!costmap_->worldToMap(wx, wy, goalx, goaly))
    {
      ROS_WARN(
          "The robot's start position is off the global costmap. Planning will "
          "always fail, are you sure the robot has been properly localized?");
      return false;
    }
    costmap_->setCost(startx, starty, costmap_2d::FREE_SPACE);
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
          ROS_ERROR( "There are obstacles in the straight line, please reset the target point " );

          return false;
        }*/
        codPath(target_x, target_y, plan);//mode by jgl 20250528, remove target check
      }
      scale = scale + dscale;
    }
    geometry_msgs::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = ros::Time::now();
    plan.push_back(goal_copy);
    return !plan.empty();
  }

  bool MyGlobalPlanner::comparePose(geometry_msgs::PoseStamped p,
                                    geometry_msgs::PoseStamped q)
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
                                std::vector<geometry_msgs::PoseStamped> &plan)
  {
    ros::Time plan_time = ros::Time::now();
    std::string global_frame = frame_id_;
    double world_x, world_y;
    world_x = xx * costmap_->getResolution() + costmap_->getOriginX();
    world_y = yy * costmap_->getResolution() + costmap_->getOriginY();
    geometry_msgs::PoseStamped pose;
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
    if (status == 1 && begin + ros::Duration(wait_time) < ros::Time::now())//当前状态为1，且原地等待已超过wait_time秒
    {
      if (!enable_dwa_obstacle_avoidance)
      {
        ROS_WARN_THROTTLE(2.0,
                          "Obstacle is still present after %.1f seconds; keep stopping and let AutoNAV replan.",
                          wait_time);
        return;
      }
      ROS_INFO("The obstacle is still there after waiting for %.1f seconds; switch to DWA avoidance.",
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
       ROS_INFO(" the obstacle is still there after waiting for 3 seconds " );
      status = 2;
      return;
    }
  }
  bool MyGlobalPlanner::dealState(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan)
  {
    switch (status)
    {
    case 0:
    {
      for (int i = 0; i < linePath.size(); i++)
      {
        geometry_msgs::PoseStamped pose = linePath[i];
        pose.pose.position.z = 1; //纯追踪的标记
        plan.push_back(pose);
      }

      break;
    }
    case 1:
    {
      ROS_INFO("There is an obstacle,please wait for 3s");
      for (int i = 0; i < linePath.size(); i++)
      {
        geometry_msgs::PoseStamped pose = linePath[i];
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }
    case 2:
    {
      
     
      geometry_msgs::PoseStamped tempGoal = linePath[res_index];
      neardis=comDistance(start,tempGoal);
      bool gotGlobal = makePlan(start, tempGoal, default_tolerance_, plan);
      if (!gotGlobal)
      {
        return false;
      }
      if(!planCheck(start,goal,plan))
      {
        ROS_ERROR("the direction of plan in contrast with the direction of start to goal,or the length of  plan is too long");
        return false;
      } 
      break;
    }
    case 3:
    {
      ROS_INFO("There is an obstacle in endpoint,please wait for 3s");
      for (int i = 0; i < linePath.size(); i++)
      {
        geometry_msgs::PoseStamped pose = linePath[i];
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }
    case 4:
    {
      ROS_INFO("There is an obstacle in endpoint,please wait for 3s");
      for (int i = 0; i < linePath.size(); i++)
      {
        geometry_msgs::PoseStamped pose = linePath[i];
        pose.pose.position.z = 2; //用于停止等待的标记
        plan.push_back(pose);
      }
      break;
    }

    }
    return true;
  }
  bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &plan)
  {
    //return makePlan(start, goal, default_tolerance_, plan);
    plan.clear();
    const bool start_changed =
        iniStart.header.frame_id.empty() || comDistance(iniStart, start) > 0.02;
    if (start_changed || !comparePose(iniGoal, goal))//当传进来的goal或start与上次不一致时，重新进行直线规划
    {
      ROS_INFO("new goal or start" );
      iniGoal = goal;
      iniStart = start;
      first_line_plan = true;
    }

    if (first_line_plan)
    {
      ROS_INFO("Straight-line planning for the first time ");
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
    ROS_INFO_STREAM("LINE_PLANNER STATUS:  "<<status);
    //不同状态处理
    if(!dealState(start,goal,plan))
    {
      return false;
    }
    publishPlan(plan);
    return !plan.empty();
  }
  
  
  bool MyGlobalPlanner::planCheck(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
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
    for(int i=0;i<plan.size()-1;i++)
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

    if(count>25||plan.size()>3*distance)    //long
    {
        return false;
    }
    if((count>10&&count<=25)||flag)               //short
    {
        for(int i=0;i<plan.size();i++)
        {
            plan[i].pose.position.z=-1;
        }
        return true;
    }
      

    return true;
  }
  void MyGlobalPlanner::setGoal(double &goal_x,double &goal_y,const geometry_msgs::PoseStamped &start,unsigned char* master_array,int nx,int ny)
  {
    double min = 1000;
    int ind = 0;
    //从直线找出一个离机器人最近的点
    for (int j = 0; j < linePath.size(); j++)
    {
      double distence = comDistance(start, linePath[j]);
      if (distence < min)
      {
        min = distence;
        ind = j;
      }
    }
    for(int i=ind;i<linePath.size();i++)
    {
      geometry_msgs::PoseStamped odomPose;
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
            *pc=costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+nx-3;
          for(int i=2;i<nx-2;i++,pc+=nx)
          {
            *pc=costmap_2d::FREE_SPACE;
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
            *pc=costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+2;
          for(int i=2;i<nx-2;i++,pc+=nx)
          {
            *pc=costmap_2d::FREE_SPACE;
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
            *pc++=costmap_2d::FREE_SPACE;
          }
          pc=master_array+(ny-3)*nx+1;
          for(int i=2;i<nx-2;i++)
          {
            *pc++=costmap_2d::FREE_SPACE;
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
            *pc++=costmap_2d::FREE_SPACE;
          }
          pc=master_array+2*nx+2;
          for(int i=2;i<nx-2;i++)
          {
            *pc++=costmap_2d::FREE_SPACE;
          }
          break;
        }
      }
    }
  }


  bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                 double tolerance, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (!initialized_)
        {
            ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        //clear the plan, just in case
        plan.clear();

        ros::NodeHandle n;
        std::string global_frame = local_frame_id_;

        //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
        if (goal.header.frame_id != global_frame)
        {
            ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != global_frame)
        {
            ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
            return false;
        }



        geometry_msgs::PoseStamped startOdom;
        geometry_msgs::PoseStamped goalOdom;
        mapToOdom(start,startOdom);
        mapToOdom(goal,goalOdom);

        double wx = startOdom.pose.position.x;
        double wy = startOdom.pose.position.y;

        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        double start_x, start_y, goal_x, goal_y;

        if (!local_costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
        {
            ROS_WARN(
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
        //add by jgl -2026-06-05  
       /* if (!local_costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
        {
            ROS_WARN_THROTTLE(1.0,
                              "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
            //setGoal(goal_x,goal_y,start,master_array,nx,ny);
        }*/
        if (old_navfn_behavior_)
        {
            goal_x = goal_x_i;
            goal_y = goal_y_i;
        }
        else
        {
            worldToMap(wx, wy, goal_x, goal_y);
        }


        ROS_INFO_STREAM("goal:"<<goal_x<<","<<goal_y);

        //clear the starting cell within the costmap because we know it can't be an obstacle
        clearRobotCell(start, start_x_i, start_y_i);

        int nx = local_costmap_->getSizeInCellsX(), ny = local_costmap_->getSizeInCellsY();

        //make sure to resize the underlying array that Navfn uses
        p_calc_->setSize(nx, ny);
        planner_->setSize(nx, ny);
        path_maker_->setSize(nx, ny);
        potential_array_ = new float[nx * ny];

        if (outline_map_)
            outlineMap(local_costmap_->getCharMap(),0,0, nx, ny, costmap_2d::LETHAL_OBSTACLE);

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
                geometry_msgs::PoseStamped goal_copy = goal;
                goal_copy.header.stamp = ros::Time::now();
                plan.push_back(goal_copy);
            }
            else
            {
                ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
        }
        else
        {
            ROS_ERROR("Failed to get a plan.");
        }

        // add orientations if needed
        orientation_filter_->processPath(start, plan);

        //publish the plan for visualization purposes
        publishPlan(plan);
        delete[] potential_array_;
        return !plan.empty();
    }
  
  void MyGlobalPlanner::publishPlan(
      const std::vector<geometry_msgs::PoseStamped> &path)
  {
    if (!initialized_)
    {
      ROS_ERROR(
          "This planner has not been initialized yet, but it is being used, "
          "please call initialize() before use");
      return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;

    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the
    // same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
      gui_path.poses[i].pose.position.z = 0;//发布的时候让所有不为0的都变成0以供展示
    }

    plan_pub_.publish(gui_path);
  }
void MyGlobalPlanner::smooth(std::vector<std::pair<float, float> > &path, double weight_data, double weight_smooth, double tolerance)
{
    double change = tolerance;
    int Iterations = 0;
    while (change >= tolerance) {
        change = 0.0;
        for (int i = 2; i < path.size()-3; i++) {
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
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;
    std::string local_frame=local_frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    double tolerance=0.2;
    double weight_data=0.1;
    double weight_smooth=0.1;
    //smooth(path,weight_data,weight_smooth,tolerance); 


    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;


       //需要将局部路径转化到全局路径
        mapToWorld(point.first, point.second, world_x, world_y);
        geometry_msgs::PoseStamped world;
        world.header.stamp=plan_time;
        world.header.frame_id=local_frame;
        world.pose.position.x=world_x;
        world.pose.position.y=world_y;
        world.pose.orientation.w=1;
        geometry_msgs::PoseStamped pose;
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
    int nx = local_costmap_->getSizeInCellsX(), ny = local_costmap_->getSizeInCellsY();
    double resolution = local_costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
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
    potential_pub_.publish(grid);
  }

} // end namespace global_planner
