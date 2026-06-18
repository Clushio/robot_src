/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/

#include <jgl_dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>



#define PI 3.1415926

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(jgl_dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace jgl_dwa_local_planner
{

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level)
  {
    if (setup_ && config.restore_defaults)
    {
      config = default_config_;
      config.restore_defaults = false;
    }
    if (!setup_)
    {
      default_config_ = config;
      setup_ = true;
    }

    startrotangle = config.jgl_rot_start_angle;
    stoprotangle = config.jgl_rot_stop_angle;
    yaw_goal_tolerance=config.yaw_goal_tolerance;
    xy_goal_tolerance=config.xy_goal_tolerance;
    max_vel_x=config.max_vel_x;
    brake_distance=config.brake_distance;
    lfc=config.lfc;
    forwNum=config.forwNum;

    pid_PA = config.pid_PA;
    pid_PB = config.pid_PB;

    frontdis_X = config.frontdis_X;

    state4counter = 5;
    state5counter = 5;

                
    xdis = xy_goal_tolerance;
    angle_err_H = yaw_goal_tolerance;

    // update generic local planner params
    base_local_planner::LocalPlannerLimits limits;
    limits.max_vel_trans = config.max_vel_trans;
    limits.min_vel_trans = config.min_vel_trans;
    limits.max_vel_x = config.max_vel_x;
    limits.min_vel_x = config.min_vel_x;
    limits.max_vel_y = config.max_vel_y;
    limits.min_vel_y = config.min_vel_y;
    limits.max_vel_theta = config.max_vel_theta;
    limits.min_vel_theta = config.min_vel_theta;
    limits.acc_lim_x = config.acc_lim_x;
    limits.acc_lim_y = config.acc_lim_y;
    limits.acc_lim_theta = config.acc_lim_theta;
    limits.acc_lim_trans = config.acc_lim_trans;
    limits.xy_goal_tolerance = config.xy_goal_tolerance;
    limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
    limits.prune_plan = config.prune_plan;
    limits.trans_stopped_vel = config.trans_stopped_vel;
    limits.theta_stopped_vel = config.theta_stopped_vel;
    planner_util_.reconfigureCB(limits, config.restore_defaults);

    // update dwa specific configuration
    ROS_WARN_NAMED("jgl_dwa_local_planner", "start angle >%f and stop angle <%f is set", startrotangle, stoprotangle);
    dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
                                   odom_helper_("odom"), setup_(false)
  {
  }

  void  DWAPlannerROS::logToFile(const std::string& message, const std::string& filename) {
    std::ofstream logfile;
    logfile.open(filename, std::ios_base::app); // 以追加模式打开文件
    if (logfile.is_open()) {
        logfile << message << std::endl;
        logfile.close();
    } else {
        ROS_ERROR("Unable to open log file: %s", filename.c_str());
    }
}
  // 获取当前时间戳并格式化为字符串
  std::string DWAPlannerROS::getCurrentTimestamp() {
      std::time_t now = std::time(nullptr);
      std::tm* now_tm = std::localtime(&now);
      std::ostringstream oss;
      oss << std::put_time(now_tm, "%Y%m%d_%H%M%S");
      return oss.str();
  }


  void DWAPlannerROS::initialize(
    std::string name,
    tf2_ros::Buffer *tf,
    costmap_2d::Costmap2DROS *costmap_ros)
{
    if (!isInitialized()) {

        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ros_->getRobotPose(current_pose_);

        
        private_nh.param("max_vel_x", max_vel_x, 0.4);
        private_nh.param("brake_distance", brake_distance, 1.0);
        private_nh.param("lfc", lfc, 0.3);
        private_nh.param("forwNum", forwNum, 0.1);
        private_nh.param("back_distance", back_distance, 2.5);

        private_nh.param("pid_PA", pid_PA, 0.7);
        private_nh.param("pid_PB", pid_PB, 0.2);
        private_nh.param("frontdis_X", frontdis_X, 0.02);
        //private_nh.param("back_distance", back_distance, 2.5);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;
      state4counter = 5;
      state5counter = 3;

      //pid_PA = config.pid_PA;
      //pid_PB = config.pid_PB;
      logfilename = "/home/nav/maps/testrecordgoal"+getCurrentTimestamp()+".txt";


            
      xdis = xy_goal_tolerance;
      angle_err_H = yaw_goal_tolerance;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

        double max_acc=max_vel_x*max_vel_x/(2*brake_distance);
        sp = new SpeedPlan(max_vel_x, max_acc);
        ps = new Pursuit(forwNum, lfc);
        lastz=1;
    }
    else {
        ROS_WARN("This planner has already been initialized, doing nothing.");
    }
}
 bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
    if (!isInitialized()) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    //add by mxb
    if((lastz>0&&orig_global_plan[0].pose.position.z==0)||(lastz==-1&&orig_global_plan[0].pose.position.z<=0))//modify to back up
    {
        firstPose=orig_global_plan[0];
    }

    lastz=orig_global_plan[0].pose.position.z;
    //modify to back up
    //useLine=orig_global_plan[0].pose.position.z==0?false:true;
    useLine = orig_global_plan[0].pose.position.z;

    if (useLine>0) {
      //每次传进来的goal和上次传进来的goal不一样或者第一次传进来goal时，将控制状态设置为1
      if(linePath.size()==0){
        status=1;
      }else {
      
        if(!comparePose(linePath.back(),orig_global_plan.back()))
        {
          status=1;
        }
      }
      linePath.clear();
      for (int j = 0; j < orig_global_plan.size(); j++) {
          linePath.push_back(orig_global_plan[j]);
      }
        std::cout<<"plan points()=  "<<linePath.size()<<std::endl;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
  
    return dp_->setPlan(orig_global_plan);
}

//jiaodu panduan 
 bool DWAPlannerROS::mygoalReachPanduan_angle()
 {
  if(fabs(goal_yaw_err) < angle_err_H)
  {
    return true;
  }
  else
  {
    return false;
  }
 }

//speed panduan
bool DWAPlannerROS::mygoalReachPanduan()
{ 
   computeRelativePosition(current_pose_,linePath.back());

   int targetType = linePath.back().pose.position.z;
   std::cout<<"current target type: "<<targetType<<std::endl;

   if((Qtar.pose.position.x<xdis)&&((abs(Qtar.pose.position.y)<xdis))&&(fabs(goal_yaw_err) < angle_err_H))
   {
      return true;
   }
   else{
      return false;
   }
}

 bool DWAPlannerROS::lineComputeVelocityCommands_modJGL(std::vector<geometry_msgs::PoseStamped> linePath, geometry_msgs::Twist &cmd_vel)
 {


 }
   
void DWAPlannerROS::computeRelativePosition(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q)
{
    // 提取位置
    Eigen::Vector3d pos_p(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    Eigen::Vector3d pos_q(q.pose.position.x, q.pose.position.y, q.pose.position.z);



    // 提取旋转（四元数）
    Eigen::Quaterniond quat_p(p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z);

    // 计算相对位置
    Eigen::Vector3d relative_pos = pos_q - pos_p;

    // 将相对位置从全局坐标系转换到局部坐标系
    Eigen::Vector3d local_relative_pos = quat_p.inverse() * relative_pos;

    // 将结果填充到 Qtar
    Qtar.pose.position.x = local_relative_pos.x();
    Qtar.pose.position.y = local_relative_pos.y();
    Qtar.pose.position.z = local_relative_pos.z();

    // 设置时间戳和参考坐标系
    Qtar.header.stamp = q.header.stamp; // 使用 q 的时间戳
    Qtar.header.frame_id = p.header.frame_id; // `Qtar` 在 `p` 的坐标系下

    return;
}

bool DWAPlannerROS::isGoalReached()
{
    if (!isInitialized()) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    if (!costmap_ros_->getRobotPose(current_pose_)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }
    //pure oursuit 判断是否到达目标点，包括距离和角度

    if (useLine>0) {
        ROS_INFO_STREAM("distance to goal:" << comDistance(current_pose_,linePath.back()) );
        //if (comDistance(current_pose_,linePath.back()) < xy_goal_tolerance &&fabs(goal_yaw_err) < yaw_goal_tolerance) 
        //mode by jgl 20241121
        if(mygoalReachPanduan())
        {

            ROS_INFO("Goal reached");
            std::stringstream ss1;
            ss1<<"goal: x="<<linePath.back().pose.position.x<<" y="<<linePath.back().pose.position.y<<std::endl;
            ss1<<"stop cmd at: x="<<current_pose_.pose.position.x<<" y="<<current_pose_.pose.position.y<<std::endl;
            ss1 << "distance to goal:" << comDistance(current_pose_,linePath.back()) <<std::endl;
            ss1 <<"Qx err = "<<Qtar.pose.position.x<<" Qy err = "<<Qtar.pose.position.y<<std::endl;
            ss1<<"_______________________________"<<std::endl;
            logToFile(ss1.str(),logfilename);
            return true; 
        }
        else if (status ==3) {

            ROS_INFO("Goal reached by rotation");
            std::stringstream ss1;
            ss1<<"goal: x="<<linePath.back().pose.position.x<<" y="<<linePath.back().pose.position.y<<std::endl;
            ss1<<"stop cmd at: x="<<current_pose_.pose.position.x<<" y="<<current_pose_.pose.position.y<<std::endl;
            ss1 << "distance to goal:" << comDistance(current_pose_,linePath.back()) <<std::endl;
            ss1 <<"Qx err = "<<Qtar.pose.position.x<<" Qy err = "<<Qtar.pose.position.y<<std::endl;
            ss1<<"_______________________________"<<std::endl;
            logToFile(ss1.str(),logfilename);

            return true; 
        }
        else {
            
            return false;
        }
    }
    else {
        if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        }
        else {
            return false;
        }
    }
}


  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
  {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
  {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS()
  {
    //make sure to clean things up
    delete dsrv_;
  }

  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel)
  {
    // dynamic window sampling approach to get useful velocity commands
    if (!isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path.cost_ < 0)
    {
      ROS_DEBUG_NAMED("jgl_dwa_local_planner", "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_INFO("A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i)
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }

  void DWAPlannerROS::cmd_pub(std::vector<geometry_msgs::PoseStamped> Point, geometry_msgs::PoseStamped pose, std::vector<double> &fov_speed)
{

    double length = comDistance(Point.back(), Point[0]); //直线总长度
    double d = comDistance(pose, Point[0]);              //机器人当前位置距离直线起点距离
    sp->speedComputeLine(d, length, fov_speed, 1);       //T型速度曲线计算每点速度值
}

bool DWAPlannerROS::comparePose(geometry_msgs::PoseStamped p,geometry_msgs::PoseStamped q)
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

double DWAPlannerROS::comDistance(geometry_msgs::PoseStamped p, geometry_msgs::PoseStamped q)
{
    double distance = sqrt((p.pose.position.x - q.pose.position.x) * (p.pose.position.x - q.pose.position.x) + (p.pose.position.y - q.pose.position.y) * (p.pose.position.y - q.pose.position.y));
    return distance;
}


//Eigen::Vector3d DWAPlannerROS::toEigenVector3d(const geometry_msgs::PoseStamped& pose) {
//    return Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
//}

Eigen::Vector2d  DWAPlannerROS::toEigenVector2d(const geometry_msgs::PoseStamped& pose) {
    return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}
/*
double DWAPlannerROS::distanceToLine(const geometry_msgs::PoseStamped& A, const geometry_msgs::PoseStamped& B, const geometry_msgs::PoseStamped& C) {
    Eigen::Vector2d A_vec = toEigenVector2d(A);
    Eigen::Vector2d B_vec = toEigenVector2d(B);
    Eigen::Vector2d C_vec = toEigenVector2d(C);

    Eigen::Vector2d AB = B_vec - A_vec;
    Eigen::Vector2d AC = C_vec - A_vec;

    // 计算向量AB和AC的叉积
    double cross_product_magnitude = AB.x() * AC.y() - AB.y() * AC.x();

    // 计算向量AB的长度
    double AB_length = AB.norm();

    if (AB_length == 0) {
        // 如果AB长度为0，说明A和B重合，返回点C到A的距离
        return AC.norm();
    }

    // 计算点C到直线AB的垂直距离
    double distance = std::abs(cross_product_magnitude) / AB_length;

    return distance;
}
*/
// 计算点C到直线AB的垂直距离，并保留正负号
double DWAPlannerROS::signedDistanceToLine(const geometry_msgs::PoseStamped& A, const geometry_msgs::PoseStamped& B, const geometry_msgs::PoseStamped& C) {
    Eigen::Vector2d A_vec = toEigenVector2d(A);
    Eigen::Vector2d B_vec = toEigenVector2d(B);
    Eigen::Vector2d C_vec = toEigenVector2d(C);

    Eigen::Vector2d AB = B_vec - A_vec;
    Eigen::Vector2d AC = C_vec - A_vec;

    // 计算向量AB和AC的叉积
    double cross_product_magnitude = AB.x() * AC.y() - AB.y() * AC.x();

    // 计算向量AB的长度
    double AB_length = AB.norm();

    if (AB_length == 0) {
        // 如果AB长度为0，说明A和B重合，返回点C到A的距离
        return AC.norm();
    }

    // 计算点C到直线AB的垂直距离，并保留正负号
    double signed_distance = cross_product_magnitude / AB_length;

    return signed_distance;
}



bool DWAPlannerROS::lineComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped> linePath, geometry_msgs::Twist &cmd_vel)
{
  computeRelativePosition(current_pose_,linePath.back());
  dis2line = signedDistanceToLine(linePath.at(0),linePath.back(),current_pose_);

  ROS_INFO_STREAM("distance to goal in xy :" << sqrt(Qtar.pose.position.x*Qtar.pose.position.x+Qtar.pose.position.y*Qtar.pose.position.y));
  std::cout<<"Qx err = "<<Qtar.pose.position.x<<"Qy err = "<<Qtar.pose.position.y<<"front dis="<<frontdis_X<<std::endl;
           // ss1<<"goal: x="<<linePath.back().pose.position.x<<" y="<<linePath.back().pose.position.y<<std::endl;
            //ss1<<"stop cmd at: x="<<current_pose_.pose.position.x<<" y="<<current_pose_.pose.position.y<<std::endl;
            //


  if(mygoalReachPanduan())
  {
    status = 3;
  }
  //std::cout<<"QX="
 if (linePath.size() < 2)
  {
    return false;
  }

  for (int i = 0; i < linePath.size(); i++)
  {
    if (linePath[i].pose.position.z == 2)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      return true;
    }
  }
  std::vector<double> fov_speed;
  //纯追踪初始化
  struct Result ret;
  double w;
  double mergew=0;
  VehicleState state(0, 0, 0, 0); //初始状态
 state.x=current_pose_.pose.position.x;
 state.y=current_pose_.pose.position.y;
 state.yaw=tf::getYaw(current_pose_.pose.orientation);


  //计算机器人当前位置距离终点的角度误差
  double yaw = tf::getYaw(linePath.back().pose.orientation);
  goal_yaw_err = yaw - state.yaw;      
  if (goal_yaw_err <= -PI)
  {
    goal_yaw_err = goal_yaw_err + 2 * PI;
  }
  else if (goal_yaw_err >= PI)
  {
    goal_yaw_err = goal_yaw_err - 2 * PI;
  }
 // ROS_INFO_STREAM( "self rotation:" << goal_yaw_err );

  double angle_line = atan2(linePath.back().pose.position.y - linePath[0].pose.position.y, linePath.back().pose.position.x - linePath[0].pose.position.x);
  double angle_Robot2End = atan2(linePath.back().pose.position.y - state.y, linePath.back().pose.position.x - state.x);
//计算机器人当前朝向和直线朝向的误差
  double angle_err = atan2(linePath.back().pose.position.y - linePath[0].pose.position.y, linePath.back().pose.position.x - linePath[0].pose.position.x) - state.yaw;
  if (angle_err <= -PI)
  {
    angle_err = angle_err + 2 * PI;
  }
  else if (angle_err >= PI)
  {
    angle_err = angle_err - 2 * PI;
  }
  //ROS_INFO_STREAM("angle_err:"<<angle_err);
  //robot he moduan dian wucha 
  double angle_err2_over = angle_line -angle_Robot2End;
//  double distance = sqrt((state.x-linePath.back().pose.position.x )*(state.x-linePath.back().pose.position.x)+((state.y-linePath.back().pose.position.y)*((state.y-linePath.back().pose.position.y)))
  if((abs(angle_err2_over)>1.57)&&(Qtar.pose.position.x<-0.1))
  {
    std::cout<<"over position="<<angle_err2_over<<std::endl;
    //std::cout<<"using new angle error"<<std::endl;
    double angle_err2 = angle_Robot2End-state.yaw;
    std::cout<<"using new angle error"<< angle_err2<<std::endl;
    angle_err = angle_err2;
    if (angle_err <= -PI)
    {
      angle_err = angle_err + 2 * PI;
    }
    else if (angle_err >= PI)
    {
      angle_err = angle_err - 2 * PI;
    }
  }

//状态切换，当前状态为1，且机器人朝向与直线方向角度误差小于1时转化到状态0，当前状态为0且机器人朝向与直线方向角度误差大于2转化到状态1，当前状态为0，且机器人距离终点小于指定距离进入状态2
  float angleerr_staChange1 = angle_err_H*1.5;
  float angleerr_staChange2 = 0.6;
  if(status==1 && fabs(angle_err)<angleerr_staChange1){
    status=4;
    state4counter = 8;
  }else if(status==0 && fabs(angle_err)>angleerr_staChange2){
    status=1;
  }
  else if(status==0 && (Qtar.pose.position.x<xdis*0.1+frontdis_X))//&&((abs(Qtar.pose.position.y)<xdis)))
  {
    //status=2;
    std::stringstream ss1;
    ss1<<"_______________________________"<<std::endl;
    ss1 << "stage 0 stop at:"<<" Qx err = "<<Qtar.pose.position.x<<" Qy err = "<<Qtar.pose.position.y<<std::endl;     
    logToFile(ss1.str(),logfilename);

    status = 5;
    state5counter = 8;
  }else if(status==2&&mygoalReachPanduan_angle())
  {
    status=3;
  }
  else if((status==4) &&(state4counter<=0) ){
    status = 0;
  }
  else if((status==5) &&(state5counter<=0) ){
    status = 2;
  }

  ROS_INFO_STREAM("---------------------CONTROL STATUS:"<<status);


//状态0  纯追踪，状态1  原地旋转使得初始方向和直线方向大致一致，状态 2 原地旋转使得与终点方向一致
  if(status==0)  
  {
    std::stringstream ss1;
    ss1 << "distance to goal:" << comDistance(current_pose_,linePath.back()) <<"Qx err = "<<Qtar.pose.position.x<<" Qy err = "<<Qtar.pose.position.y<<" dis2line="<<dis2line<<std::endl;
        //    ss1<<"_______________________________"<<std::endl;
    logToFile(ss1.str(),logfilename);

    int ind = ps->calc_target_index(state, linePath);
    ret = ps->pure_pursuit(state, linePath, ind);
    ind = ret.y;          //距离机器人当前位置前视距离的下标
    double alpha = ret.x; //角度误差
   
    //ROS_INFO_STREAM( "alpha:" << alpha<<" PA"<<pid_PA<<" PB"<<pid_PB);

    w = 0.7 * alpha;
    mergew =  pid_PA*alpha + Qtar.pose.position.y*pid_PB;

    //std::cout<<"org w="<<w<<" neww="<<mergew<<std::endl;

    cmd_pub(linePath, current_pose_, fov_speed); //速度规划
    state.v = fov_speed.back();

    if (state.v > sp->vmax)
    { //速度限制
      state.v = sp->vmax;
    }
    if (fabs(w) > 1)
    { //角速度限制
      w = 1*alpha/(fabs(alpha)+0.00001);
    }

    //当机器人当前朝向与其到前视点角度相差较大时，让速度降低
    if (fabs(alpha) > 0.4)
    {
      state.v = 0.2;
    }
    if (fabs(alpha) > 0.8)
    {
      state.v = 0.1;
    }
    //当机器人当前朝向与直线方向角度相差较大时，让速度降低

    if (fabs(angle_err) > 0.2)
    {
      state.v = 0.2;
    }
    if (fabs(angle_err) > 0.8)
    {
      state.v = 0.1;
    }
  }
  else if(status==1)
  { 
    //当机器人当前朝向与直线方向角度相差很大时，进行旋转，一般只有在初始位置的时候才会发生
      state.v = 0;
      w = 0.25*angle_err/(fabs(angle_err)+0.000001);

  }
  else if(status==2)//终点时旋转方向到终点朝向一定角度范围内
  {
    state.v=0;
    w=0.25*goal_yaw_err/(fabs(goal_yaw_err)+0.000001);
  }
  else if(status==3)
  {
      state.v=0;
      w=0;
  }
  else if(status==4)
  {
      state4counter = state4counter-1;
      state.v=0;
      w=0;
  }
  else if(status==5)
  {
      state5counter = state5counter-1;
      state.v=0;
      w=0;
  }

  cmd_vel.linear.x = state.v;
  //cmd_vel.angular.z = w;
  if(status==0)
  {
    cmd_vel.angular.z = mergew; //mix pid by jgl
  }
  else{
    cmd_vel.angular.z = w;   
  }
  

  std::cout<<"pure persuit cmd: v="<<state.v<<"  w="<<w <<" megw="<<mergew<<std::endl;
  return true;
}


  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal

// dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if (!costmap_ros_->getRobotPose(current_pose_)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if (transformed_plan.empty()) {
        ROS_WARN_NAMED("mxb_dwa_local_planner", "Received an empty transformed plan.");
        return false;
    }
    if (useLine>0) {
      begin=ros::Time::now();
    //  ROS_INFO("pure pursuit!!!");
      publishGlobalPlan(transformed_plan);
      return lineComputeVelocityCommands(linePath, cmd_vel);
    }
    //modify to back up
    else if(useLine<0&&begin+ros::Duration(back_distance)>ros::Time::now())
    {
        ROS_INFO("backing up");
        cmd_vel.linear.x=-0.1;
        cmd_vel.linear.y=0;
        cmd_vel.angular.z=0;
        return true;
    }

    ROS_INFO("DWA!!!!" );
    ROS_DEBUG_NAMED("jgl_dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // pose : current_pose_
    // plan : transformed_plan
    // add by jgl
    float directionchange = base_plan_direction_check(transformed_plan, current_pose_);
    directionchange=directionchange*180/PI;

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
    {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
      is_start_rotating = false;
    }
    else
    {
      //dwa successful
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      ROS_INFO_STREAM("directionchange:"<<directionchange);
      //add by jgl 2021 01 27

      std::cout<<"cmd_vel for dwa:"<<isOk<<"  "<<cmd_vel.linear.x <<" z="<<cmd_vel.angular.z<<std::endl;

      //   double posediff=comDistance(current_pose_,firstPose);
      //   ROS_INFO_STREAM("posediff:"<<posediff);

      // if (posediff<0.1 && (abs(directionchange) > startrotangle) && (!is_start_rotating))
      // {
      //   is_start_rotating = true;
      // }

      //   if ((abs(directionchange) < stoprotangle) && (is_start_rotating))
      //   {
      //       is_start_rotating = false;
      //       ROS_WARN_NAMED("jgl_dwa_local_planner", "the angle dis=%f DWA planner start rotation finished", directionchange);

      //   }

      //   if (is_start_rotating)
      //   {
      //       ROS_WARN_NAMED("jgl_dwa_local_planner", "the angle dis=%f DWA planner start rotation.", directionchange);
      //       cmd_vel.linear.x = 0;
      //       cmd_vel.linear.y = 0;
      //       cmd_vel.angular.z = 0.4 * directionchange / (abs(directionchange) + 0.001);
      //   }

      if (isOk)
      {
        publishGlobalPlan(transformed_plan);
      }
      else
      {
        ROS_WARN_NAMED("jgl_dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }


      return isOk;
    }
  }

  float DWAPlannerROS::base_plan_direction_check(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan,
                                                 geometry_msgs::PoseStamped &robot_pose_)
  {
    if (orig_global_plan.size() < 10)
    {
      return 0;
    }

    geometry_msgs::PoseStamped planposes[10];
    geometry_msgs::Quaternion robotdirection = robot_pose_.pose.orientation;
    geometry_msgs::Point robotposition = robot_pose_.pose.position;


    double robotangle = tf2::getYaw(robot_pose_.pose.orientation);                    //通过赋值方式就可以得到需要的绕z轴偏转角度了
                                                          // ROS_ERROR("the robot angle is %f",robotangle);

    geometry_msgs::Quaternion targetOrientation = orig_global_plan[9].pose.orientation;
    geometry_msgs::Point targetPosition = orig_global_plan[9].pose.position;

    double d = sqrt((robotposition.x - targetPosition.x) * (robotposition.x - targetPosition.x) + (robotposition.y - targetPosition.y) * (robotposition.y - targetPosition.y));


        double direction_angle =
            atan2(targetPosition.y - robotposition.y, targetPosition.x - robotposition.x) ;
        // ROS_ERROR("the robot pose %f,%f, planpose is %f,%f",robotposition.x,robotposition.y,targetPosition.x,targetPosition.y);
        //  ROS_ERROR("the robot planned angle is %f, distance = %f",direction_angle,d);

        double diffangle = direction_angle - robotangle;
        
        if (diffangle <= -PI)
        {
        diffangle = diffangle + 2*PI;
        }
        if (diffangle >= PI)
        {
        diffangle = diffangle - 2*PI;
        }

        //  ROS_ERROR("the change angle is %f",diffangle);

        return (float)diffangle;


  }

};
