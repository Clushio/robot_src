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
#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <costmap_2d/cost_values.h>
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
    useLine = 0;
    goal_yaw_err = 0.0;
    yaw_goal_tolerance = 0.0;
    xy_goal_tolerance = 0.0;
    status = 0;
    published_terminal_motion_state_ = -1;
    published_path_control_mode_ = -1;
    is_start_rotating = false;
    state4counter = 0;
    state5counter = 0;
    lastz = 1;
    enable_bspline_reference_path_ = false;
    reference_safe_distance_ = 0.25;
    reference_fallback_boundary_distance_ = 0.15;
    reference_middle_pass_distance_ = 0.25;
    reference_terminal_xy_tolerance_ = 0.08;
    obstacle_wait_time_ = 10.0;
    reference_obstacle_slowdown_distance_ = 1.0;
    reference_obstacle_stop_distance_ = 0.7;
    path_deviation_replan_threshold_ = 0.60;
    fixed_route_mode_.store(false);
    reference_obstacle_start_ = ros::Time(0);
    current_topology_goal_index_ = -1;
    legacy_line_forced_goal_index_ = -1;
    reference_goal_reached_ = false;
    reference_path_mode_ = TrajectoryGenerator::PATH_MODE_INVALID;
    dsrv_ = NULL;
    reference_job_running_ = false;
    reference_job_result_ready_ = false;
    reference_job_result_success_ = false;
    reference_job_topology_version_ = -1;
    reference_job_failed_topology_version_ = -1;
    reference_job_path_mode_ = TrajectoryGenerator::PATH_MODE_INVALID;
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

  void DWAPlannerROS::stopCmd(geometry_msgs::Twist &cmd_vel) const
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  void DWAPlannerROS::publishReferenceStatus(ReferenceStatus status)
  {
    geometry_msgs::Vector3Stamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.vector.x = static_cast<double>(reference_path_manager_.topologyVersion());
    message.vector.y = static_cast<double>(current_topology_goal_index_);
    message.vector.z = static_cast<double>(status);
    reference_status_pub_.publish(message);
  }

  void DWAPlannerROS::publishTerminalMotionState(TerminalMotionState state)
  {
    const int value = static_cast<int>(state);
    if (published_terminal_motion_state_ == value)
    {
      return;
    }
    published_terminal_motion_state_ = value;
    std_msgs::UInt8 message;
    message.data = static_cast<uint8_t>(value);
    terminal_motion_state_pub_.publish(message);
    ROS_INFO("Terminal motion state changed to %d.", value);
  }

  void DWAPlannerROS::publishPathControlMode(PathControlMode mode, bool force)
  {
    const int value = static_cast<int>(mode);
    if (!force && published_path_control_mode_ == value)
    {
      return;
    }
    published_path_control_mode_ = value;
    std_msgs::UInt8 message;
    message.data = static_cast<uint8_t>(value);
    path_control_mode_pub_.publish(message);
    ROS_INFO("Path control mode changed to %d.", value);
  }

  void DWAPlannerROS::updateLineGoalRelativeState()
  {
    if (linePath.empty())
    {
      return;
    }

    computeRelativePosition(current_pose_, linePath.back());
    const double goal_yaw = tf::getYaw(linePath.back().pose.orientation);
    const double robot_yaw = tf::getYaw(current_pose_.pose.orientation);
    goal_yaw_err = goal_yaw - robot_yaw;
    if (goal_yaw_err <= -PI)
    {
      goal_yaw_err += 2.0 * PI;
    }
    else if (goal_yaw_err >= PI)
    {
      goal_yaw_err -= 2.0 * PI;
    }
  }

  bool DWAPlannerROS::shouldUseReferencePath()
  {
    current_topology_goal_index_ = -1;
    if (!enable_bspline_reference_path_ || useLine <= 0 || linePath.empty())
    {
      return false;
    }
    if (linePath.back().pose.position.z == 2)
    {
      return false;
    }

    int goal_index = -1;
    if (!reference_path_manager_.isMiddleGoal(linePath.back(), &goal_index))
    {
      return false;
    }

    current_topology_goal_index_ = goal_index;
    return true;
  }

  const char *DWAPlannerROS::referencePathModeName(
      TrajectoryGenerator::PathMode mode) const
  {
    switch (mode)
    {
      case TrajectoryGenerator::PATH_MODE_BSPLINE:
        return "bspline";
      case TrajectoryGenerator::PATH_MODE_CUBIC:
        return "cubic";
      case TrajectoryGenerator::PATH_MODE_HYBRID:
        return "hybrid";
      case TrajectoryGenerator::PATH_MODE_POLYLINE_FALLBACK:
        return "polyline_fallback";
      default:
        return "invalid";
    }
  }

  bool DWAPlannerROS::referencePathJobRunning() const
  {
    boost::mutex::scoped_lock lock(reference_job_mutex_);
    return reference_job_running_;
  }

  bool DWAPlannerROS::referencePathJobFailedForCurrentTopology() const
  {
    const int topology_version = reference_path_manager_.topologyVersion();
    boost::mutex::scoped_lock lock(reference_job_mutex_);
    return reference_job_failed_topology_version_ == topology_version;
  }

  bool DWAPlannerROS::referenceGenerationUsefulForCurrentGoal(
      const std::vector<geometry_msgs::PoseStamped> &waypoints) const
  {
    if (reference_goal_reached_ || linePath.empty() || waypoints.size() < 4)
    {
      return false;
    }
    if (linePath.back().pose.position.z == 2)
    {
      return false;
    }

    const int goal_index = reference_path_manager_.goalIndex(linePath.back());
    if (goal_index <= 0)
    {
      return false;
    }

    return goal_index < static_cast<int>(waypoints.size()) - 1;
  }

  void DWAPlannerROS::waitForReferencePathJob()
  {
    if (reference_job_thread_.joinable())
    {
      reference_job_thread_.join();
    }
  }

  bool DWAPlannerROS::startReferencePathJob(
      const std::vector<geometry_msgs::PoseStamped> &waypoints,
      int topology_version)
  {
    if (waypoints.size() < 4)
    {
      return false;
    }

    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      if (reference_job_running_)
      {
        return false;
      }
      if (reference_job_result_ready_ &&
          reference_job_topology_version_ == topology_version)
      {
        return false;
      }
      if (reference_job_failed_topology_version_ == topology_version)
      {
        return false;
      }
    }

    waitForReferencePathJob();

    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      if (reference_job_running_)
      {
        return false;
      }
      reference_job_running_ = true;
      reference_job_result_ready_ = false;
      reference_job_result_success_ = false;
      reference_job_topology_version_ = topology_version;
      reference_job_path_.poses.clear();
      reference_job_path_mode_ = TrajectoryGenerator::PATH_MODE_INVALID;
      reference_job_fallback_segments_.clear();
    }

    reference_job_thread_ =
        boost::thread(&DWAPlannerROS::referencePathGenerationThread,
                      this, waypoints, topology_version);
    ROS_INFO("JGL reference path: started async generation for topology version %d with %zu waypoints.",
             topology_version, waypoints.size());
    return true;
  }

  void DWAPlannerROS::referencePathGenerationThread(
      std::vector<geometry_msgs::PoseStamped> waypoints,
      int topology_version)
  {
    nav_msgs::Path reference_path;
    const bool success = trajectory_generator_.generate(waypoints, reference_path);
    const TrajectoryGenerator::PathMode path_mode =
        success ? trajectory_generator_.lastPathMode()
                : TrajectoryGenerator::PATH_MODE_INVALID;
    const std::vector<int> fallback_segments =
        success ? trajectory_generator_.lastFallbackSegments()
                : std::vector<int>();

    boost::mutex::scoped_lock lock(reference_job_mutex_);
    reference_job_path_ = reference_path;
    reference_job_path_mode_ = path_mode;
    reference_job_fallback_segments_ = fallback_segments;
    reference_job_result_success_ = success;
    reference_job_result_ready_ = true;
    reference_job_running_ = false;
    reference_job_topology_version_ = topology_version;
  }

  bool DWAPlannerROS::consumeReferencePathJob()
  {
    bool ready = false;
    bool success = false;
    int job_topology_version = -1;
    nav_msgs::Path reference_path;
    TrajectoryGenerator::PathMode path_mode =
        TrajectoryGenerator::PATH_MODE_INVALID;
    std::vector<int> fallback_segments;
    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      ready = reference_job_result_ready_;
      success = reference_job_result_success_;
      job_topology_version = reference_job_topology_version_;
      reference_path = reference_job_path_;
      path_mode = reference_job_path_mode_;
      fallback_segments = reference_job_fallback_segments_;
    }

    if (!ready)
    {
      return false;
    }

    waitForReferencePathJob();

    const int current_topology_version =
        reference_path_manager_.topologyVersion();
    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      reference_job_result_ready_ = false;
      reference_job_path_.poses.clear();
      reference_job_path_mode_ = TrajectoryGenerator::PATH_MODE_INVALID;
      reference_job_fallback_segments_.clear();
    }

    if (job_topology_version != current_topology_version)
    {
      ROS_WARN("JGL reference path: discard stale async result for topology version %d, current version is %d.",
               job_topology_version, current_topology_version);
      return false;
    }

    const std::vector<geometry_msgs::PoseStamped> current_waypoints =
        reference_path_manager_.waypoints();
    if (!referenceGenerationUsefulForCurrentGoal(current_waypoints))
    {
      ROS_INFO("JGL reference path: discard async result because current topology goal no longer needs a reference path.");
      return false;
    }

    if (!success)
    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      reference_job_failed_topology_version_ = job_topology_version;
      ROS_ERROR("JGL reference path: async generation failed for topology version %d.",
                job_topology_version);
      return false;
    }

    reference_path_mode_ = path_mode;
    reference_fallback_segments_ = fallback_segments;
    legacy_line_forced_goal_index_ = -1;
    reference_path_manager_.setReferencePath(reference_path);
    path_follower_.reset();
    double nearest_distance = 0.0;
    unsigned int nearest_index = 0;
    syncReferencePathIndex(&nearest_distance, &nearest_index);
    reference_path_pub_.publish(reference_path);
    publishReferencePathMarker(reference_path, reference_path_mode_);
    ROS_INFO("JGL reference path: published /reference_path version %d mode=%s samples=%zu init_idx=%u init_dist=%.3f.",
             reference_path_manager_.pathVersion(),
             referencePathModeName(reference_path_mode_),
             reference_path.poses.size(),
             nearest_index,
             nearest_distance);
    return true;
  }

  void DWAPlannerROS::maybeStartReferencePathJob()
  {
    if (!enable_bspline_reference_path_ || useLine <= 0 ||
        !reference_path_manager_.hasWaypoints())
    {
      return;
    }
    const std::vector<geometry_msgs::PoseStamped> waypoints =
        reference_path_manager_.waypoints();
    if (!referenceGenerationUsefulForCurrentGoal(waypoints))
    {
      return;
    }
    if (!reference_path_manager_.needRegenerate())
    {
      return;
    }
    if (referencePathJobRunning())
    {
      return;
    }
    if (referencePathJobFailedForCurrentTopology())
    {
      return;
    }

    const int topology_version = reference_path_manager_.topologyVersion();
    if (startReferencePathJob(waypoints, topology_version))
    {
      reference_path_manager_.markRegenerateAttempt();
    }
  }

  bool DWAPlannerROS::prepareReferencePath(bool &generation_pending,
                                           bool &generation_failed)
  {
    generation_pending = false;
    generation_failed = false;

    if (!reference_path_manager_.hasWaypoints())
    {
      return false;
    }

    if (consumeReferencePathJob())
    {
      return reference_path_manager_.hasValidPath();
    }

    maybeStartReferencePathJob();

    if (reference_path_manager_.hasValidPath())
    {
      return true;
    }

    if (consumeReferencePathJob())
    {
      return reference_path_manager_.hasValidPath();
    }

    if (referencePathJobRunning())
    {
      generation_pending = true;
      return false;
    }

    if (referencePathJobFailedForCurrentTopology())
    {
      generation_failed = true;
      return false;
    }

    if (reference_path_manager_.hasWaypoints())
    {
      generation_pending = true;
    }
    return false;
  }

  void DWAPlannerROS::publishReferencePathMarker(const nav_msgs::Path &path,
                                                 TrajectoryGenerator::PathMode path_mode) const
  {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.header.stamp = ros::Time::now();
    marker.ns = "jgl_reference_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.a = 1.0;
    if (path_mode == TrajectoryGenerator::PATH_MODE_POLYLINE_FALLBACK)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.45;
      marker.color.b = 0.0;
    }
    else if (path_mode == TrajectoryGenerator::PATH_MODE_HYBRID)
    {
      marker.color.r = 0.1;
      marker.color.g = 1.0;
      marker.color.b = 0.25;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 0.85;
      marker.color.b = 1.0;
    }

    marker.points.reserve(path.poses.size());
    for (unsigned int i = 0; i < path.poses.size(); ++i)
    {
      geometry_msgs::Point point;
      point.x = path.poses[i].pose.position.x;
      point.y = path.poses[i].pose.position.y;
      point.z = path.poses[i].pose.position.z + 0.03;
      marker.points.push_back(point);
    }
    reference_path_marker_pub_.publish(marker);
  }

  bool DWAPlannerROS::referencePathCanFollowGoal(int goal_index) const
  {
    if (goal_index < 0)
    {
      return false;
    }

    if (reference_path_mode_ == TrajectoryGenerator::PATH_MODE_POLYLINE_FALLBACK)
    {
      return false;
    }

    if (reference_path_mode_ != TrajectoryGenerator::PATH_MODE_HYBRID)
    {
      return true;
    }

    const int incoming_segment = goal_index - 1;
    if (incoming_segment >= 0 &&
        incoming_segment < static_cast<int>(reference_fallback_segments_.size()) &&
        reference_fallback_segments_[incoming_segment] != 0)
    {
      return false;
    }
    return true;
  }

  bool DWAPlannerROS::referenceGoalExitsToFallback(int goal_index) const
  {
    if (reference_path_mode_ != TrajectoryGenerator::PATH_MODE_HYBRID ||
        goal_index < 0)
    {
      return false;
    }

    const int outgoing_segment = goal_index;
    return outgoing_segment >= 0 &&
           outgoing_segment < static_cast<int>(reference_fallback_segments_.size()) &&
           reference_fallback_segments_[outgoing_segment] != 0;
  }

  bool DWAPlannerROS::referenceEntryHeadingAligned(
      const nav_msgs::Path &reference_path,
      double *heading_error) const
  {
    if (heading_error != NULL)
    {
      *heading_error = 0.0;
    }
    if (reference_path.poses.size() < 2)
    {
      return true;
    }

    const unsigned int current_index =
        std::min(reference_path_manager_.currentPathIndex(),
                 static_cast<unsigned int>(reference_path.poses.size() - 1));
    const double entry_distance =
        std::hypot(current_pose_.pose.position.x -
                       reference_path.poses.front().pose.position.x,
                   current_pose_.pose.position.y -
                       reference_path.poses.front().pose.position.y);
    const double entry_gate_distance =
        std::max(0.60, path_follower_.lookaheadDistance() + 0.20);
    if (current_index > 6 && entry_distance > entry_gate_distance)
    {
      return true;
    }

    unsigned int tangent_index = current_index;
    double walked = 0.0;
    for (unsigned int i = current_index + 1; i < reference_path.poses.size(); ++i)
    {
      walked += std::hypot(reference_path.poses[i].pose.position.x -
                               reference_path.poses[i - 1].pose.position.x,
                           reference_path.poses[i].pose.position.y -
                               reference_path.poses[i - 1].pose.position.y);
      tangent_index = i;
      if (walked >= 0.30)
      {
        break;
      }
    }

    if (tangent_index <= current_index)
    {
      return true;
    }

    const geometry_msgs::PoseStamped &from = reference_path.poses[current_index];
    const geometry_msgs::PoseStamped &to = reference_path.poses[tangent_index];
    const double dx = to.pose.position.x - from.pose.position.x;
    const double dy = to.pose.position.y - from.pose.position.y;
    if (std::hypot(dx, dy) < 1e-4)
    {
      return true;
    }

    const double path_yaw = std::atan2(dy, dx);
    const double robot_yaw = tf::getYaw(current_pose_.pose.orientation);
    double error = path_yaw - robot_yaw;
    if (error <= -PI)
    {
      error += 2.0 * PI;
    }
    else if (error >= PI)
    {
      error -= 2.0 * PI;
    }

    if (heading_error != NULL)
    {
      *heading_error = error;
    }
    const double tolerance = std::max(0.35, static_cast<double>(angle_err_H) * 1.5);
    return std::fabs(error) <= tolerance;
  }

  void DWAPlannerROS::forceLegacyLineRotate(const char *reason)
  {
    if (legacy_line_forced_goal_index_ == current_topology_goal_index_)
    {
      return;
    }
    // Reset only on the transition into legacy control. This function is
    // called every control cycle for the same fallback goal; resetting before
    // the guard above would prevent status 0 steering from ever ramping beyond
    // its first limited step.
    path_follower_.reset();
    if (status != 1)
    {
      ROS_WARN("JGL reference path: fallback to legacy rotate-line controller, reset line status to rotate. reason=%s",
               reason == NULL ? "unknown" : reason);
    }
    status = 1;
    state4counter = 0;
    state5counter = 0;
    terminal_yaw_controller_.reset();
    reference_goal_reached_ = false;
    legacy_line_forced_goal_index_ = current_topology_goal_index_;
  }

  bool DWAPlannerROS::syncReferencePathIndex(double *distance_to_reference,
                                             unsigned int *nearest_index,
                                             bool hold_before_path_end)
  {
    unsigned int nearest = reference_path_manager_.currentPathIndex();
    const double distance = reference_path_manager_.distanceToReference(current_pose_,
                                                                       &nearest);
    reference_path_manager_.advanceCurrentPathIndex(nearest,
                                                    hold_before_path_end);
    if (distance_to_reference != NULL)
    {
      *distance_to_reference = distance;
    }
    if (nearest_index != NULL)
    {
      *nearest_index = nearest;
    }
    return distance <= path_deviation_replan_threshold_;
  }

  bool DWAPlannerROS::referenceGoalReached(
      unsigned int *goal_path_index,
      double *remaining_reference_distance,
      double *goal_distance)
  {
    if (!shouldUseReferencePath() || !reference_path_manager_.hasValidPath())
    {
      return false;
    }
    if (!referencePathCanFollowGoal(current_topology_goal_index_))
    {
      return false;
    }

    publishReferenceStatus(REFERENCE_ACTIVE);

    const bool terminal_goal =
        reference_path_manager_.isTerminalReferenceGoal(
            current_topology_goal_index_);
    if (terminal_goal)
    {
      const nav_msgs::Path reference_path =
          reference_path_manager_.referencePath();
      const double direct_goal_distance =
          std::hypot(current_pose_.pose.position.x -
                         linePath.back().pose.position.x,
                     current_pose_.pose.position.y -
                         linePath.back().pose.position.y);
      if (goal_path_index != NULL)
      {
        *goal_path_index = reference_path.poses.empty()
                               ? 0U
                               : static_cast<unsigned int>(
                                     reference_path.poses.size() - 1);
      }
      if (remaining_reference_distance != NULL)
      {
        *remaining_reference_distance = direct_goal_distance;
      }
      if (goal_distance != NULL)
      {
        *goal_distance = direct_goal_distance;
      }
      return direct_goal_distance <= reference_terminal_xy_tolerance_;
    }

    unsigned int reached_goal_path_index = 0;
    double reached_remaining_reference_distance = 0.0;
    double reached_goal_distance = 0.0;
    const bool reached = reference_path_manager_.referenceProgressReached(
        linePath.back(), current_pose_, reference_middle_pass_distance_,
        &reached_goal_path_index,
        &reached_remaining_reference_distance, &reached_goal_distance);

    if (goal_path_index != NULL)
    {
      *goal_path_index = reached_goal_path_index;
    }
    if (remaining_reference_distance != NULL)
    {
      *remaining_reference_distance = reached_remaining_reference_distance;
    }
    if (goal_distance != NULL)
    {
      *goal_distance = reached_goal_distance;
    }

    if (!reached)
    {
      return false;
    }

    if (referenceGoalExitsToFallback(current_topology_goal_index_) &&
        reached_goal_distance > reference_fallback_boundary_distance_)
    {
      ROS_INFO_THROTTLE(
          1.0,
          "JGL reference path: goal_idx=%d is before fallback; wait for direct distance %.3f <= %.3f before switching to legacy rotate-line.",
          current_topology_goal_index_,
          reached_goal_distance,
          reference_fallback_boundary_distance_);
      return false;
    }

    return true;
  }

  bool DWAPlannerROS::costmapPointBlocked(costmap_2d::Costmap2D *costmap,
                                          unsigned int mx, unsigned int my,
                                          int radius_cells) const
  {
    if (costmap == NULL)
    {
      return true;
    }

    const int size_x = static_cast<int>(costmap->getSizeInCellsX());
    const int size_y = static_cast<int>(costmap->getSizeInCellsY());
    const int center_x = static_cast<int>(mx);
    const int center_y = static_cast<int>(my);
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= size_x || y >= size_y)
        {
          continue;
        }

        const unsigned char cost = costmap->getCost(x, y);
        if (cost != costmap_2d::NO_INFORMATION &&
            cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          return true;
        }
      }
    }
    return false;
  }

  bool DWAPlannerROS::referencePathObstacleDistance(
      const nav_msgs::Path &path, double &obstacle_distance)
  {
    obstacle_distance = std::numeric_limits<double>::infinity();
    if (path.poses.empty())
    {
      return false;
    }
    unsigned int index = reference_path_manager_.currentPathIndex();
    index = std::min(index, static_cast<unsigned int>(path.poses.size() - 1));
    return pathObstacleDistance(path.poses, index, obstacle_distance);
  }

  bool DWAPlannerROS::pathObstacleDistance(
      const std::vector<geometry_msgs::PoseStamped> &path,
      unsigned int start_index, double &obstacle_distance)
  {
    obstacle_distance = std::numeric_limits<double>::infinity();
    if (path.empty() || costmap_ros_ == NULL ||
        costmap_ros_->getCostmap() == NULL)
    {
      return false;
    }

    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    const double resolution = std::max(0.01, costmap->getResolution());
    const int radius_cells =
        static_cast<int>(std::ceil(reference_safe_distance_ / resolution));
    start_index = std::min(
        start_index, static_cast<unsigned int>(path.size() - 1));

    double walked = 0.0;
    geometry_msgs::PoseStamped previous = current_pose_;
    for (unsigned int i = start_index; i < path.size(); ++i)
    {
      walked += comDistance(previous, path[i]);
      if (walked > reference_obstacle_slowdown_distance_)
      {
        break;
      }
      previous = path[i];

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(path[i].pose.position.x,
                               path[i].pose.position.y,
                               mx, my))
      {
        continue;
      }
      if (costmapPointBlocked(costmap, mx, my, radius_cells))
      {
        obstacle_distance = walked;
        return true;
      }
    }
    return false;
  }

  bool DWAPlannerROS::legacyPathObstacleDistance(double &obstacle_distance)
  {
    obstacle_distance = std::numeric_limits<double>::infinity();
    if (linePath.empty() || costmap_ros_ == NULL ||
        costmap_ros_->getCostmap() == NULL)
    {
      return false;
    }

    // The reference-path controller starts at its current path index.  The
    // legacy plan has no persistent index, so use the closest forward sample
    // as the equivalent starting point and apply the same look-ahead,
    // clearance and cost threshold below.
    unsigned int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < linePath.size(); ++i)
    {
      const double distance = comDistance(current_pose_, linePath[i]);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    return pathObstacleDistance(linePath, nearest_index, obstacle_distance);
  }

  bool DWAPlannerROS::fixedRouteBlocked()
  {
    if (!fixed_route_mode_.load())
    {
      return false;
    }
    if (linePath.size() < 2 || costmap_ros_ == NULL ||
        costmap_ros_->getCostmap() == NULL)
    {
      return false;
    }

    unsigned int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < linePath.size(); ++i)
    {
      const double distance = comDistance(current_pose_, linePath[i]);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    const double resolution = std::max(0.01, costmap->getResolution());
    const int radius_cells =
        static_cast<int>(std::ceil(reference_safe_distance_ / resolution));
    const double check_distance =
        std::max(1.0, path_follower_.lookaheadDistance() * 2.0);
    double walked = 0.0;
    geometry_msgs::PoseStamped previous = current_pose_;
    for (unsigned int i = nearest_index; i < linePath.size(); ++i)
    {
      walked += comDistance(previous, linePath[i]);
      if (walked > check_distance)
      {
        break;
      }
      previous = linePath[i];
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap->worldToMap(linePath[i].pose.position.x,
                              linePath[i].pose.position.y, mx, my) &&
          costmapPointBlocked(costmap, mx, my, radius_cells))
      {
        return true;
      }
    }
    return false;
  }

  void DWAPlannerROS::fixedRouteModeCallback(
      const std_msgs::Bool::ConstPtr &mode)
  {
    fixed_route_mode_.store(mode->data);
    if (!mode->data)
    {
      reference_obstacle_start_ = ros::Time(0);
    }
  }

  bool DWAPlannerROS::computeReferenceVelocityCommands(geometry_msgs::Twist &cmd_vel,
                                                       bool &hard_failure)
  {
    hard_failure = false;
    if (!shouldUseReferencePath())
    {
      return false;
    }

    bool generation_pending = false;
    bool generation_failed = false;
    if (!prepareReferencePath(generation_pending, generation_failed))
    {
      if (generation_pending)
      {
        stopCmd(cmd_vel);
        ROS_INFO_THROTTLE(1.0,
                          "JGL reference path: async generation is still running, wait at topology goal idx=%d.",
                          current_topology_goal_index_);
        return true;
      }
      if (generation_failed)
      {
        stopCmd(cmd_vel);
        hard_failure = true;
        ROS_ERROR_THROTTLE(1.0,
                           "JGL reference path: async generation failed for topology version %d, keep stopped.",
                           reference_path_manager_.topologyVersion());
        return false;
      }
      return false;
    }

    if (!referencePathCanFollowGoal(current_topology_goal_index_))
    {
      forceLegacyLineRotate("polyline_fallback_near_goal");
      ROS_INFO_THROTTLE(1.0,
                        "JGL reference path: goal_idx=%d uses legacy rotate-line controller because reference mode=%s has fallback polyline nearby.",
                        current_topology_goal_index_,
                        referencePathModeName(reference_path_mode_));
      return false;
    }

    nav_msgs::Path reference_path = reference_path_manager_.referencePath();
    if (reference_path.poses.size() < 2)
    {
      return false;
    }

    updateLineGoalRelativeState();

    const bool terminal_reference_goal =
        reference_path_manager_.isTerminalReferenceGoal(
            current_topology_goal_index_);
    const double terminal_goal_distance =
        std::hypot(current_pose_.pose.position.x -
                       linePath.back().pose.position.x,
                   current_pose_.pose.position.y -
                       linePath.back().pose.position.y);
    const bool terminal_goal_unreached =
        terminal_reference_goal &&
        terminal_goal_distance > reference_terminal_xy_tolerance_;

    double distance_to_reference = 0.0;
    unsigned int nearest_reference_index = 0;
    if (!syncReferencePathIndex(&distance_to_reference,
                                &nearest_reference_index,
                                terminal_goal_unreached))
    {
      stopCmd(cmd_vel);
      publishReferenceStatus(REFERENCE_PATH_DEVIATED);
      hard_failure = true;
      ROS_WARN_THROTTLE(1.0,
                        "JGL reference path: robot is %.3f m away from reference path at nearest idx=%u, threshold=%.3f. Stop instead of chasing a far reference.",
                        distance_to_reference,
                        nearest_reference_index,
                        path_deviation_replan_threshold_);
      return false;
    }

    double reference_entry_heading_error = 0.0;
    if (!referenceEntryHeadingAligned(reference_path,
                                      &reference_entry_heading_error))
    {
      forceLegacyLineRotate("reference_entry_heading_misaligned");
      ROS_WARN_THROTTLE(1.0,
                        "JGL reference path: entry heading error %.3f rad is too large, use legacy rotate-line before reference tracking.",
                        reference_entry_heading_error);
      return false;
    }

    unsigned int goal_path_index = 0;
    double remaining_to_goal_on_reference = 0.0;
    double direct_goal_distance = 0.0;
    if (referenceGoalReached(&goal_path_index,
                             &remaining_to_goal_on_reference,
                             &direct_goal_distance))
    {
      publishReferenceStatus(REFERENCE_PASSED);
      if (terminal_reference_goal)
      {
        ROS_INFO_THROTTLE(
            1.0,
            "JGL reference path: terminal goal idx=%d reached by base_link XY distance %.3f <= %.3f.",
            current_topology_goal_index_, direct_goal_distance,
            reference_terminal_xy_tolerance_);
      }
      else
      {
        ROS_INFO_THROTTLE(1.0,
                          "JGL reference path: pass-through goal idx=%d reached; keep following without stopping "
                          "(path_idx=%u target_path_idx=%u remain=%.3f direct_dist=%.3f).",
                          current_topology_goal_index_,
                          reference_path_manager_.currentPathIndex(),
                          goal_path_index,
                          remaining_to_goal_on_reference,
                          direct_goal_distance);
      }
    }

    double reference_obstacle_distance =
        std::numeric_limits<double>::infinity();
    const bool reference_obstacle_ahead =
        referencePathObstacleDistance(reference_path,
                                      reference_obstacle_distance);
    if (reference_obstacle_ahead)
    {
      if (reference_obstacle_start_.isZero())
      {
        reference_obstacle_start_ = ros::Time::now();
        ROS_WARN("JGL reference path: obstacle %.3f m ahead on path, enter staged slowdown.",
                 reference_obstacle_distance);
      }

      if ((ros::Time::now() - reference_obstacle_start_).toSec() >=
          obstacle_wait_time_)
      {
        stopCmd(cmd_vel);
        hard_failure = true;
        ROS_WARN_THROTTLE(1.0,
                          "JGL reference path: obstacle wait timeout, keep stopped and let topology replan.");
        return false;
      }
    }
    else
    {
      reference_obstacle_start_ = ros::Time(0);
    }

    unsigned int new_index = reference_path_manager_.currentPathIndex();
    double curvature = 0.0;
    if (!path_follower_.computeCommand(reference_path, current_pose_,
                                       reference_path_manager_.currentPathIndex(),
                                       cmd_vel, new_index, curvature,
                                       terminal_reference_goal,
                                       reference_terminal_xy_tolerance_))
    {
      return false;
    }
    publishPathControlMode(REFERENCE_TRACKING);
    reference_path_manager_.advanceCurrentPathIndex(new_index);
    cmd_vel.linear.y = 0.0;

    if (reference_obstacle_ahead)
    {
      const double obstacle_scale = referenceObstacleSpeedScale(
          reference_obstacle_distance,
          reference_obstacle_slowdown_distance_,
          reference_obstacle_stop_distance_);
      cmd_vel.linear.x *= obstacle_scale;
      cmd_vel.linear.y *= obstacle_scale;
      cmd_vel.angular.z *= obstacle_scale;
      if (obstacle_scale <= 0.0)
      {
        // A zero Twist centres Ranger's wheels. Keep the curvature filter in
        // the same state so obstacle clearing starts steering from zero.
        path_follower_.reset();
        stopCmd(cmd_vel);
      }
      ROS_WARN_THROTTLE(
          0.5,
          "JGL reference path: obstacle distance=%.3f m, candidate speed scale=%.2f.",
          reference_obstacle_distance, obstacle_scale);
    }

    if (referenceGoalReached(&goal_path_index,
                             &remaining_to_goal_on_reference,
                             &direct_goal_distance))
    {
      publishReferenceStatus(REFERENCE_PASSED);
    }

    std::vector<geometry_msgs::PoseStamped> local_reference;
    const unsigned int start_index = reference_path_manager_.currentPathIndex();
    const unsigned int end_index =
        std::min(static_cast<unsigned int>(reference_path.poses.size()),
                 start_index + 80U);
    for (unsigned int i = start_index; i < end_index; ++i)
    {
      local_reference.push_back(reference_path.poses[i]);
    }
    publishLocalPlan(local_reference);

    ROS_INFO_THROTTLE(0.5,
                      "JGL reference path: follow idx=%u goal_idx=%d v=%.3f w=%.3f curv=%.3f.",
                      reference_path_manager_.currentPathIndex(),
                      current_topology_goal_index_,
                      cmd_vel.linear.x, cmd_vel.angular.z, curvature);
    return true;
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
        private_nh.param("enable_bspline_reference_path",
                         enable_bspline_reference_path_, false);
        private_nh.param("safe_distance", reference_safe_distance_, 0.25);
        private_nh.param("reference_fallback_boundary_distance",
                         reference_fallback_boundary_distance_, 0.15);
        private_nh.param("reference_middle_pass_distance",
                         reference_middle_pass_distance_, 0.25);
        private_nh.param("reference_terminal_xy_tolerance",
                         reference_terminal_xy_tolerance_, 0.08);
        private_nh.param("obstacle_wait_time", obstacle_wait_time_, 10.0);
        private_nh.param("obstacle_slowdown_distance",
                         reference_obstacle_slowdown_distance_, 1.0);
        private_nh.param("obstacle_stop_distance",
                         reference_obstacle_stop_distance_, 0.7);
        private_nh.param("path_deviation_replan_threshold",
                         path_deviation_replan_threshold_, 0.60);
        double terminal_yaw_tolerance = 0.04;
        double terminal_yaw_kp = 1.2;
        double terminal_yaw_max_speed = 0.25;
        double terminal_yaw_min_speed = 0.06;
        int terminal_yaw_stable_cycles = 3;
        private_nh.param("terminal_yaw_tolerance",
                         terminal_yaw_tolerance, terminal_yaw_tolerance);
        private_nh.param("terminal_yaw_kp",
                         terminal_yaw_kp, terminal_yaw_kp);
        private_nh.param("terminal_yaw_max_speed",
                         terminal_yaw_max_speed, terminal_yaw_max_speed);
        private_nh.param("terminal_yaw_min_speed",
                         terminal_yaw_min_speed, terminal_yaw_min_speed);
        private_nh.param("terminal_yaw_stable_cycles",
                         terminal_yaw_stable_cycles,
                         terminal_yaw_stable_cycles);
        terminal_yaw_controller_.configure(
            terminal_yaw_tolerance, terminal_yaw_kp,
            terminal_yaw_max_speed, terminal_yaw_min_speed,
            terminal_yaw_stable_cycles);
        reference_safe_distance_ = std::max(0.0, reference_safe_distance_);
        reference_fallback_boundary_distance_ =
            std::max(0.02, reference_fallback_boundary_distance_);
        reference_middle_pass_distance_ =
            std::max(0.02, reference_middle_pass_distance_);
        reference_terminal_xy_tolerance_ =
            std::max(0.01, reference_terminal_xy_tolerance_);
        obstacle_wait_time_ = std::max(0.0, obstacle_wait_time_);
        reference_obstacle_stop_distance_ =
            std::max(0.0, reference_obstacle_stop_distance_);
        reference_obstacle_slowdown_distance_ =
            std::max(reference_obstacle_stop_distance_ + 0.01,
                     reference_obstacle_slowdown_distance_);
        path_deviation_replan_threshold_ =
            std::max(0.05, path_deviation_replan_threshold_);
        //private_nh.param("back_distance", back_distance, 2.5);

        ros::NodeHandle node_nh;
        reference_path_pub_ = node_nh.advertise<nav_msgs::Path>("/reference_path", 1, true);
        reference_path_marker_pub_ =
            node_nh.advertise<visualization_msgs::Marker>("/reference_path_marker", 1, true);
        reference_status_pub_ =
            node_nh.advertise<geometry_msgs::Vector3Stamped>("/bspline_status", 1, false);
        terminal_motion_state_pub_ =
            node_nh.advertise<std_msgs::UInt8>("/anav/terminal_motion_state", 1, true);
        path_control_mode_pub_ =
            node_nh.advertise<std_msgs::UInt8>("/anav/path_control_mode", 1, true);
        publishTerminalMotionState(TERMINAL_TRACKING);
        publishPathControlMode(PATH_CONTROL_UNKNOWN, true);
        fixed_route_mode_sub_ = node_nh.subscribe<std_msgs::Bool>(
            "/anav/fixed_route_mode", 1,
            &DWAPlannerROS::fixedRouteModeCallback, this);
        trajectory_generator_.initialize(private_nh, node_nh);
        reference_path_manager_.initialize(node_nh, private_nh);
        path_follower_.loadParams(private_nh);

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
    bool fixed_route_parameter = false;
    ros::param::param("/anav/fixed_route_mode", fixed_route_parameter, false);
    fixed_route_mode_.store(fixed_route_parameter);
    if (!isInitialized()) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    publishPathControlMode(PATH_CONTROL_UNKNOWN, true);
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
        publishTerminalMotionState(TERMINAL_TRACKING);
        terminal_yaw_controller_.reset();
        reference_goal_reached_ = false;
        legacy_line_forced_goal_index_ = -1;
      }else {
      
        if(!comparePose(linePath.back(),orig_global_plan.back()))
        {
          status=1;
          publishTerminalMotionState(TERMINAL_TRACKING);
          terminal_yaw_controller_.reset();
          reference_goal_reached_ = false;
          legacy_line_forced_goal_index_ = -1;
        }
      }
      linePath.clear();
      for (int j = 0; j < orig_global_plan.size(); j++) {
          geometry_msgs::PoseStamped pose = orig_global_plan[j];
          // In fixed-route mode obstacle stopping is decided continuously by
          // fixedRouteBlocked().  A z==2 marker is only a snapshot written by
          // the global planner when this plan was created.  Keeping that
          // marker in a frozen MoveBase plan would command zero velocity
          // forever even after the obstacle has cleared.
          if (fixed_route_mode_.load() && pose.pose.position.z == 2)
          {
            pose.pose.position.z = 1;
          }
          linePath.push_back(pose);
      }
      if (fixed_route_mode_.load() && useLine == 2)
      {
        useLine = 1;
        lastz = 1;
        ROS_WARN("Fixed route: discarded a stale global-planner WAIT marker; "
                 "live costmap obstacle checking remains active on the locked path.");
      }
      current_topology_goal_index_ = reference_path_manager_.goalIndex(linePath.back());
      reference_obstacle_start_ = ros::Time(0);
        std::cout<<"plan points()=  "<<linePath.size()<<std::endl;
    }
    else
    {
      current_topology_goal_index_ = -1;
      reference_obstacle_start_ = ros::Time(0);
      reference_goal_reached_ = false;
      legacy_line_forced_goal_index_ = -1;
      terminal_yaw_controller_.reset();
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
  
    return dp_->setPlan(orig_global_plan);
}

 bool DWAPlannerROS::lineComputeVelocityCommands_modJGL(std::vector<geometry_msgs::PoseStamped> linePath, geometry_msgs::Twist &cmd_vel)
 {
    stopCmd(cmd_vel);
    return false;
 }
   
void DWAPlannerROS::computeRelativePosition(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q)
{
    // 导航路径是二维的，position.z 被全局规划器用作控制模式标记
    // (1: 纯追踪, 2: 停止等待)。不能将该 z 值和实车 roll/pitch 参与
    // 三维旋转，否则会在 Qtar.x/y 中产生虚假的终点距离。
    const double dx = q.pose.position.x - p.pose.position.x;
    const double dy = q.pose.position.y - p.pose.position.y;
    const double robot_yaw = tf::getYaw(p.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);

    // 将全局 XY 差值旋转到机器人平面坐标系（旋转 -yaw）。
    Qtar.pose.position.x = cos_yaw * dx + sin_yaw * dy;
    Qtar.pose.position.y = -sin_yaw * dx + cos_yaw * dy;
    Qtar.pose.position.z = 0.0;

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
        if (shouldUseReferencePath() &&
            reference_path_manager_.hasValidPath() &&
            referencePathCanFollowGoal(current_topology_goal_index_))
        {
            // A B-spline middle waypoint is a pass-through marker. run_nav will
            // immediately preempt it with the next waypoint; completing this
            // action here would make move_base publish a zero velocity.
            return false;
        }
        if (reference_goal_reached_)
        {
            ROS_INFO("Goal reached by reference path progress");
            return true;
        }
        unsigned int goal_path_index = 0;
        double remaining_to_goal_on_reference = 0.0;
        double direct_goal_distance = 0.0;
        if (referenceGoalReached(&goal_path_index,
                                 &remaining_to_goal_on_reference,
                                 &direct_goal_distance))
        {
            reference_goal_reached_ = true;
            ROS_INFO("Goal reached by reference path progress "
                     "(path_idx=%u target_path_idx=%u remain=%.3f direct_dist=%.3f)",
                     reference_path_manager_.currentPathIndex(),
                     goal_path_index,
                     remaining_to_goal_on_reference,
                     direct_goal_distance);
            return true;
        }
        // Line goals are completed only by the terminal state machine.  A
        // one-cycle XY/yaw match is not enough because it may be encoder/TF
        // noise while the chassis is still settling.
        if (status ==3) {

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
    waitForReferencePathJob();
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
  if (linePath.size() < 2)
  {
    return false;
  }

  computeRelativePosition(current_pose_,linePath.back());
  dis2line = signedDistanceToLine(linePath.at(0),linePath.back(),current_pose_);

  ROS_INFO_STREAM("distance to goal in xy :" << sqrt(Qtar.pose.position.x*Qtar.pose.position.x+Qtar.pose.position.y*Qtar.pose.position.y));
  std::cout<<"Qx err = "<<Qtar.pose.position.x<<"Qy err = "<<Qtar.pose.position.y<<"front dis="<<frontdis_X<<std::endl;
           // ss1<<"goal: x="<<linePath.back().pose.position.x<<" y="<<linePath.back().pose.position.y<<std::endl;
            //ss1<<"stop cmd at: x="<<current_pose_.pose.position.x<<" y="<<current_pose_.pose.position.y<<std::endl;
            //

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

  double terminal_yaw_command = 0.0;
  bool terminal_yaw_updated = false;
  //std::cout<<"QX="

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

// 状态切换：终点位置判断必须优先于路径方向对正。
// 终点附近重新生成的“当前位置->终点”直线可能因越点而翻转约 PI，
// 此时禁止进入 status 1，否则会先朝错误的路径方向原地旋转。
  float angleerr_staChange1 = angle_err_H*1.5;
  float angleerr_staChange2 = 0.6;
  const double xy_error = hypot(Qtar.pose.position.x, Qtar.pose.position.y);
  const double terminal_capture_distance = xdis;
  const double terminal_no_realign_distance = 0.10;
  if((status==0 || status==1) && xy_error<=terminal_capture_distance){
    status = 5;
    state5counter = 8;
    terminal_yaw_controller_.reset();
    ROS_INFO("Terminal position captured at %.3f m; switch to final-yaw preparation.",
             xy_error);
    publishTerminalMotionState(TERMINAL_POSITION_CAPTURED);
  }else if(status==1 && xy_error<=terminal_no_realign_distance){
    status = 0;
    ROS_WARN("Suppress path realignment near goal at %.3f m.", xy_error);
  }else if(status==1 && fabs(angle_err)<angleerr_staChange1){
    status=4;
    state4counter = 8;
  }else if(status==0 &&
           xy_error>terminal_no_realign_distance &&
           fabs(angle_err)>angleerr_staChange2){
    status=1;
  }
  else if(status==2)
  {
    terminal_yaw_updated = true;
    if (terminal_yaw_controller_.update(goal_yaw_err,
                                        &terminal_yaw_command))
    {
      status=3;
      publishTerminalMotionState(TERMINAL_COMPLETE);
      ROS_INFO("Terminal yaw stable; goal completed.");
    }
  }
  else if((status==4) &&(state4counter<=0) ){
    status = 0;
  }
  else if((status==5) &&(state5counter<=0) ){
    status = 2;
    publishTerminalMotionState(TERMINAL_ROTATING);
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
    // The first cycle after the preparation hold intentionally stays at zero;
    // subsequent cycles use the proportional, bounded terminal controller.
    w=terminal_yaw_updated ? terminal_yaw_command : 0.0;
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
    // The legacy controller produces angular velocity directly. Convert it to
    // curvature so the same Ackermann-safe limiter used by reference tracking
    // can smooth steering without changing the commanded forward speed.
    const double speed_for_curvature = std::max(0.01, std::fabs(state.v));
    const double target_curvature = mergew / speed_for_curvature;
    const double smoothed_curvature =
        path_follower_.smoothCurvatureCommand(target_curvature);
    cmd_vel.angular.z = state.v * smoothed_curvature;
  }
  else{
    // Status 1/2 are intentional in-place rotations and must keep their direct
    // response. Reset only the forward-tracking filter while they are active.
    path_follower_.reset();
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
      consumeReferencePathJob();
      maybeStartReferencePathJob();
      bool reference_hard_failure = false;
      if (computeReferenceVelocityCommands(cmd_vel, reference_hard_failure))
      {
        return true;
      }
      if (reference_hard_failure)
      {
        publishPathControlMode(PATH_CONTROL_UNKNOWN);
        stopCmd(cmd_vel);
        return false;
      }
      // Only the legacy straight-line fallback keeps the original hard-stop
      // check. Active reference paths were already handled above with staged
      // slowdown, so the two policies cannot mask each other.
      publishPathControlMode(LEGACY_FALLBACK);
      if (fixedRouteBlocked())
      {
        path_follower_.reset();
        stopCmd(cmd_vel);
        ROS_WARN_THROTTLE(
            1.0,
            "Fixed route: obstacle detected on the locked legacy path; wait in place until it clears.");
        return true;
      }

      double legacy_obstacle_distance =
          std::numeric_limits<double>::infinity();
      const bool legacy_obstacle_ahead =
          legacyPathObstacleDistance(legacy_obstacle_distance);
      if (legacy_obstacle_ahead)
      {
        if (reference_obstacle_start_.isZero())
        {
          reference_obstacle_start_ = ros::Time::now();
          ROS_WARN("JGL legacy path: obstacle %.3f m ahead on path, enter staged slowdown.",
                   legacy_obstacle_distance);
        }
        if ((ros::Time::now() - reference_obstacle_start_).toSec() >=
            obstacle_wait_time_)
        {
          stopCmd(cmd_vel);
          ROS_WARN_THROTTLE(
              1.0,
              "JGL legacy path: obstacle wait timeout, keep stopped and let topology replan.");
          return false;
        }
      }
      else
      {
        reference_obstacle_start_ = ros::Time(0);
      }

      if (!lineComputeVelocityCommands(linePath, cmd_vel))
      {
        return false;
      }
      if (legacy_obstacle_ahead)
      {
        const double obstacle_scale = referenceObstacleSpeedScale(
            legacy_obstacle_distance,
            reference_obstacle_slowdown_distance_,
            reference_obstacle_stop_distance_);
        cmd_vel.linear.x *= obstacle_scale;
        cmd_vel.linear.y *= obstacle_scale;
        cmd_vel.angular.z *= obstacle_scale;
        if (obstacle_scale <= 0.0)
        {
          stopCmd(cmd_vel);
        }
        ROS_WARN_THROTTLE(
            0.5,
            "JGL legacy path: obstacle distance=%.3f m, candidate speed scale=%.2f.",
            legacy_obstacle_distance, obstacle_scale);
      }
      return true;
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

    ROS_ERROR("DWA control dispatch reached an unexpected state.");
    stopCmd(cmd_vel);
    return false;
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
