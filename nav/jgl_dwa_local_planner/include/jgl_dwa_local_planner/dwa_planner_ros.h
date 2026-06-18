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
#ifndef jgl_dwa_local_planner_DWA_PLANNER_ROS_H_
#define jgl_dwa_local_planner_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <jgl_dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <jgl_dwa_local_planner/dwa_planner.h>

#include <jgl_dwa_local_planner/speedPlan.h>
#include <jgl_dwa_local_planner/pursuit.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace jgl_dwa_local_planner
{
   /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
   class DWAPlannerROS : public nav_core::BaseLocalPlanner
   {
   public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS();

      void  logToFile(const std::string& message, const std::string& filename) ;
      std::string getCurrentTimestamp();
      std::string logfilename;

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer *tf,
                      costmap_2d::Costmap2DROS *costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      bool mygoalReachPanduan();
      bool mygoalReachPanduan_angle();
      geometry_msgs::PoseStamped Qtar;

      void  computeRelativePosition(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q);
      
      
      Eigen::Vector2d toEigenVector2d(const geometry_msgs::PoseStamped& pose);

      double signedDistanceToLine(const geometry_msgs::PoseStamped& A, const geometry_msgs::PoseStamped& B, const geometry_msgs::PoseStamped& C); 

      float xdis;
      float angle_err_H;

      double dis2line;

      /**
       * @brief  直线控制
       * @param linePath 储存直线点
       * @param cmd_vel 控制所需速度和角速度
       * @return True if the navigation function was computed successfully, false otherwise
       */
      bool lineComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped> linePath, geometry_msgs::Twist &cmd_vel);

      bool lineComputeVelocityCommands_modJGL(std::vector<geometry_msgs::PoseStamped> linePath, geometry_msgs::Twist &cmd_vel);
   
      /**
       * @brief  获取机器人当前位置在整条直线上规划出的速度
       * @param point 直线点
       * @param pose 机器人当前位置
       * @param fov_speed 储存速度
       */
      void cmd_pub(std::vector<geometry_msgs::PoseStamped> Point, geometry_msgs::PoseStamped pose, std::vector<double> &fov_speed);
      /**
       * @brief  计算两点之间距离
       * @param p 第一个点
       * @param q 第二个点
       * @return 两点距离
       */
      double comDistance(geometry_msgs::PoseStamped p, geometry_msgs::PoseStamped q);
      /**
       * @brief  判断两个geometry_msgs/PoseStamped是否相同
       * @param p 第一个点 
       * @param q 第二个点 
       * @return 两点相同返回true 
       */
      bool comparePose(geometry_msgs::PoseStamped p, geometry_msgs::PoseStamped q);

      //add by jgl : for the condition of fanxiangyundong
      float base_plan_direction_check(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan, geometry_msgs::PoseStamped &robot_pose_);
      // return the direction change of the plan


      bool isInitialized()
      {
         return initialized_;
      }
      //add by jgl
      int state4counter;
      int state5counter;

      double pid_PA,pid_PB;
      double frontdis_X;

      

   private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

      tf2_ros::Buffer *tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS *costmap_ros_;

      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      jgl_dwa_local_planner::DWAPlannerConfig default_config_;
      bool setup_;
      geometry_msgs::PoseStamped current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;

      bool initialized_;

      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      //add by  mxb
      SpeedPlan *sp; //T型速度规划
      Pursuit *ps;
      tf::TransformListener lis;
      tf::StampedTransform trans;
      int useLine;                                     //是否使用直线控制
      std::vector<geometry_msgs::PoseStamped> linePath; //将直线存到此中
      double goal_yaw_err;                              //到达目标点的角度误差
      double yaw_goal_tolerance;                        //允许到达目标点的角度误差
      double xy_goal_tolerance;                         //允许到达目标点的距离误差
      int status;                                       //直线控制状态，0代表进行纯追踪，1代表运行过程中(起点)角度偏离过大进行旋转，2代表到达终点后的旋转
      double max_vel_x;                                 //最大运行速度
      double brake_distance;                            //刹车距离
      double lfc, forwNum;                              //pure pursuit coefficient
      bool is_start_rotating;
    double startrotangle;
    double stoprotangle;
    int lastz;
    geometry_msgs::PoseStamped firstPose;
    ros::Time begin;
    double back_distance;
      

   };
};
#endif
