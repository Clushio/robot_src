#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <myglobal_planner/potential_calculator.h>
#include <myglobal_planner/expander.h>
#include <myglobal_planner/traceback.h>
#include <myglobal_planner/orientation_filter.h>
#include <myglobal_planner/GlobalPlannerConfig.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace myglobal_planner {

class Expander;
class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the myglobal_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        MyGlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        MyGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~MyGlobalPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Computes the full navigation function for the map given a point in the world to start from
         * @param world_point The point to use for seeding the navigation function
         * @return True if the navigation function was computed successfully, false otherwise
         */
        bool computePotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return The navigation function's value at that point in the world
         */
        double getPointPotential(const geometry_msgs::Point& world_point);

     

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @param tolerance The tolerance on searching around the world_point specified
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

       /**
       * @brief  障碍物检测
       * @param start 机器人当前位置 
       */    
        void obsCheck(geometry_msgs::PoseStamped start);
       /**
       * @brief 直线生成 
       * @param start 起点
       * @param goal 终点
       * @param plan 直线储存 
       * @return 成功生成返回true
       */          
        bool makePlanLine(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                       double tolerance, std::vector<geometry_msgs::PoseStamped> &plan);
       /**
       * @brief 将指定map坐标系下的点转化成世界坐标系下 
       * @param xx 指定点x坐标 
       * @param yy 指定点y坐标
       * @param plan 直线储存 
       */                                         
        void codPath(double xx, double yy, std::vector<geometry_msgs::PoseStamped> &plan);
       /**
       * @brief  将map坐标系转化成odom坐标系下的坐标
       * @param map_pose map坐标系下的坐标 
       * @param odom_pose odom坐标系下的坐标 
       */            
        void mapToOdom(geometry_msgs::PoseStamped map_pose, geometry_msgs::PoseStamped &odom_pose);
        /**
       * @brief  将map坐标系转化成odom坐标系下的坐标
       * @param odom_pose odom坐标系下的坐标 
       * @param map_pose map坐标系下的坐标 
       */
        void odomToMap(geometry_msgs::PoseStamped odom_pose,geometry_msgs::PoseStamped &map_pose);
       /**
       * @brief  某障碍点的八邻域检测是否存在障碍点
       * @param x 障碍点的x坐标
       * @param y 障碍点的y坐标
       * @return 八邻域存在障碍点返回true
       */            
        bool pointCheck(unsigned int x, unsigned int y);
       /**
       * @brief  计算两点距离
       * @param p 第一个点
       * @param q 第二个点
       * @return 两点距离
       */          
        double comDistance(geometry_msgs::PoseStamped p, geometry_msgs::PoseStamped q);
       /**
       * @brief 判断两个pose是否相同 
       * @param p 第一个pose
       * @param q 第二个pose
       * @return 相同返回true
       */          
        bool comparePose(geometry_msgs::PoseStamped p,geometry_msgs::PoseStamped q);
       /**
       * @brief  规划状态切换
       */            
        void stateSwitch();
       /**
       * @brief  状态处理
       * @param start 机器人当前位置 
       * @param goal 目标点 
       * @param plan 储存直线 
       * @return 绕行时曲线生存错误则返回false
       */    
        bool dealState(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Determine whether the path is reasonable 
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return false if the direction is opposite or the length is too long ,otherwise true
         */
        bool planCheck(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan);
//
        void setGoal(double &goal_x,double &goal_y,const geometry_msgs::PoseStamped &start,unsigned char* master_array,int nx,int ny);
        void smooth(std::vector<std::pair<float, float> > &path, double weight_data, double weight_smooth, double tolerance);
        

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_;
        costmap_2d::Costmap2D* local_costmap_;           //局部costmap
        costmap_2d::LayeredCostmap* layered_costmap_; //use to modify the costmap
        ros::Publisher tmp_costmap_pub;                               //publish the modified costmap


        std::string frame_id_;
        std::string local_frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_;


        bool obstacle_exist;                             //障碍物是否存在
        bool first_line_plan;                            //是否需要重新直线规划
        int res_index;                                   //障碍物后方绕行点在直线中下标
        ros::Time begin;                                 //用于计算等待时间是否超时
        std::vector<geometry_msgs::PoseStamped> linePath;//储存直线
        tf::TransformListener lis;
        geometry_msgs::PoseStamped iniGoal;               //上一次传进来的goal，用于判断goal是否发生了改变
        double neardis;                                   //绕行时机器人当前位置与绕行终点距离
        int status;                                       //规划状态，0直线生成，1出现障碍物等待，2绕行，3直线行驶时终点出现障碍物，4绕行时终点出现障碍物                                    
        bool endOcc;                                      //终点是否被占据
        double distance_convert_line;                        //绕行时机器人当前位置与终点距离小于多少时转化成直线，用于绕行和直线轨迹的顺滑衔接
        double point_per_meter;                           //每米多少个点
        double distance_check_obstacle;                   //障碍物前方检测范围，机器人前方多少米处的障碍物让机器人认为其前方出现了障碍物
        double distance_behind_obstacle;                  //障碍物后方多少米作为绕行终点
        int lethal_cost;                                   //costmap致命障碍物的cost
        double wait_time;                                  //检测到前方有障碍物的等待时间
        bool flag;


    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void publishPotential(float* potential);

        double planner_window_x_, planner_window_y_, default_tolerance_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        PotentialCalculator* p_calc_;
        Expander* planner_;
        Traceback* path_maker_;
        OrientationFilter* orientation_filter_;

        bool publish_potential_;
        ros::Publisher potential_pub_;
        int publish_scale_;

        void outlineMap(unsigned char* costarr, int ini_x,int ini_y,int nx, int ny, unsigned char value);
        unsigned char* cost_array_;
        float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
        float convert_offset_;

        bool outline_map_;

        dynamic_reconfigure::Server<myglobal_planner::GlobalPlannerConfig> *dsrv_;
        void reconfigureCB(myglobal_planner::GlobalPlannerConfig &config, uint32_t level);

};

} //end namespace myglobal_planner

#endif