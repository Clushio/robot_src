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
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <jgl_dwa_local_planner/parameter_utils.h>
#include <math.h>



#define PI 3.1415926

PLUGINLIB_EXPORT_CLASS(jgl_dwa_local_planner::DWAPlannerROS, nav2_core::Controller)

namespace jgl_dwa_local_planner
{

  DWAPlannerROS::DWAPlannerROS()
  {
    fixed_route_mode_.store(false);
  }

  void  DWAPlannerROS::logToFile(const std::string& message, const std::string& filename) {
    std::ofstream logfile;
    logfile.open(filename, std::ios_base::app); // 以追加模式打开文件
    if (logfile.is_open()) {
        logfile << message << std::endl;
        logfile.close();
    } else {
        RCLCPP_ERROR(logger_, "Unable to open log file: %s", filename.c_str());
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

  void DWAPlannerROS::stopCmd(geometry_msgs::msg::Twist &cmd_vel) const
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
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = clock_->now();
    message.header.frame_id = "map";
    message.vector.x = static_cast<double>(reference_path_manager_.topologyVersion());
    message.vector.y = static_cast<double>(current_topology_goal_index_);
    message.vector.z = static_cast<double>(status);
    reference_status_pub_->publish(message);
  }

  void DWAPlannerROS::publishTerminalMotionState(TerminalMotionState state)
  {
    const int value = static_cast<int>(state);
    if (published_terminal_motion_state_ == value || !terminal_motion_state_pub_ ||
        !terminal_motion_state_pub_->is_activated())
    {
      return;
    }
    published_terminal_motion_state_ = value;
    std_msgs::msg::UInt8 message;
    message.data = static_cast<uint8_t>(value);
    terminal_motion_state_pub_->publish(message);
    RCLCPP_INFO(logger_, "Terminal motion state changed to %d.", value);
  }

  void DWAPlannerROS::updateLineGoalRelativeState()
  {
    if (linePath.empty())
    {
      return;
    }

    computeRelativePosition(current_pose_, linePath.back());
    const double goal_yaw = tf2::getYaw(linePath.back().pose.orientation);
    const double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);
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
      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const
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
      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
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
    RCLCPP_INFO(logger_, "JGL reference path: started async generation for topology version %d with %zu waypoints.",
             topology_version, waypoints.size());
    return true;
  }

  void DWAPlannerROS::referencePathGenerationThread(
      std::vector<geometry_msgs::msg::PoseStamped> waypoints,
      int topology_version)
  {
    nav_msgs::msg::Path reference_path;
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
    nav_msgs::msg::Path reference_path;
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
      RCLCPP_WARN(logger_, "JGL reference path: discard stale async result for topology version %d, current version is %d.",
               job_topology_version, current_topology_version);
      return false;
    }

    const std::vector<geometry_msgs::msg::PoseStamped> current_waypoints =
        reference_path_manager_.waypoints();
    if (!referenceGenerationUsefulForCurrentGoal(current_waypoints))
    {
      RCLCPP_INFO(logger_, "JGL reference path: discard async result because current topology goal no longer needs a reference path.");
      return false;
    }

    if (!success)
    {
      boost::mutex::scoped_lock lock(reference_job_mutex_);
      reference_job_failed_topology_version_ = job_topology_version;
      RCLCPP_ERROR(logger_, "JGL reference path: async generation failed for topology version %d.",
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
    reference_path_pub_->publish(reference_path);
    publishReferencePathMarker(reference_path, reference_path_mode_);
    RCLCPP_INFO(logger_, "JGL reference path: published /reference_path version %d mode=%s samples=%zu init_idx=%u init_dist=%.3f.",
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
    const std::vector<geometry_msgs::msg::PoseStamped> waypoints =
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

  void DWAPlannerROS::publishReferencePathMarker(const nav_msgs::msg::Path &path,
                                                 TrajectoryGenerator::PathMode path_mode) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.header.stamp = clock_->now();
    marker.ns = "jgl_reference_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
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
      geometry_msgs::msg::Point point;
      point.x = path.poses[i].pose.position.x;
      point.y = path.poses[i].pose.position.y;
      point.z = path.poses[i].pose.position.z + 0.03;
      marker.points.push_back(point);
    }
    reference_path_marker_pub_->publish(marker);
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
      const nav_msgs::msg::Path &reference_path,
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

    const geometry_msgs::msg::PoseStamped &from = reference_path.poses[current_index];
    const geometry_msgs::msg::PoseStamped &to = reference_path.poses[tangent_index];
    const double dx = to.pose.position.x - from.pose.position.x;
    const double dy = to.pose.position.y - from.pose.position.y;
    if (std::hypot(dx, dy) < 1e-4)
    {
      return true;
    }

    const double path_yaw = std::atan2(dy, dx);
    const double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);
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
      RCLCPP_WARN(logger_, "JGL reference path: fallback to legacy rotate-line controller, reset line status to rotate. reason=%s",
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
      const nav_msgs::msg::Path reference_path =
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
      RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 1000,
          "JGL reference path: goal_idx=%d is before fallback; wait for direct distance %.3f <= %.3f before switching to legacy rotate-line.",
          current_topology_goal_index_,
          reached_goal_distance,
          reference_fallback_boundary_distance_);
      return false;
    }

    return true;
  }

  bool DWAPlannerROS::costmapPointBlocked(nav2_costmap_2d::Costmap2D *costmap,
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
        if (cost != nav2_costmap_2d::NO_INFORMATION &&
            cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          return true;
        }
      }
    }
    return false;
  }

  bool DWAPlannerROS::referencePathObstacleDistance(
      const nav_msgs::msg::Path &path, double &obstacle_distance)
  {
    obstacle_distance = std::numeric_limits<double>::infinity();
    if (path.poses.empty() || costmap_ros_ == NULL || costmap_ros_->getCostmap() == NULL)
    {
      return false;
    }

    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    const double resolution = std::max(0.01, costmap->getResolution());
    const int radius_cells =
        static_cast<int>(std::ceil(reference_safe_distance_ / resolution));
    const double check_distance = reference_obstacle_slowdown_distance_;

    unsigned int index = reference_path_manager_.currentPathIndex();
    index = std::min(index, static_cast<unsigned int>(path.poses.size() - 1));

    double walked = 0.0;
    geometry_msgs::msg::PoseStamped previous = current_pose_;
    for (unsigned int i = index; i < path.poses.size(); ++i)
    {
      walked += comDistance(previous, path.poses[i]);
      if (walked > check_distance)
      {
        break;
      }
      previous = path.poses[i];

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(path.poses[i].pose.position.x,
                               path.poses[i].pose.position.y,
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

    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    const double resolution = std::max(0.01, costmap->getResolution());
    const int radius_cells =
        static_cast<int>(std::ceil(reference_safe_distance_ / resolution));
    const double check_distance =
        std::max(1.0, path_follower_.lookaheadDistance() * 2.0);
    double walked = 0.0;
    geometry_msgs::msg::PoseStamped previous = current_pose_;
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
      const std_msgs::msg::Bool::SharedPtr mode)
  {
    fixed_route_mode_.store(mode->data);
    if (!mode->data)
    {
      reference_obstacle_start_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    }
  }

  bool DWAPlannerROS::computeReferenceVelocityCommands(geometry_msgs::msg::Twist &cmd_vel,
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
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
                          "JGL reference path: async generation is still running, wait at topology goal idx=%d.",
                          current_topology_goal_index_);
        return true;
      }
      if (generation_failed)
      {
        stopCmd(cmd_vel);
        hard_failure = true;
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
                           "JGL reference path: async generation failed for topology version %d, keep stopped.",
                           reference_path_manager_.topologyVersion());
        return false;
      }
      return false;
    }

    if (!referencePathCanFollowGoal(current_topology_goal_index_))
    {
      forceLegacyLineRotate("polyline_fallback_near_goal");
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
                        "JGL reference path: goal_idx=%d uses legacy rotate-line controller because reference mode=%s has fallback polyline nearby.",
                        current_topology_goal_index_,
                        referencePathModeName(reference_path_mode_));
      return false;
    }

    nav_msgs::msg::Path reference_path = reference_path_manager_.referencePath();
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
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
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
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
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
        RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 1000,
            "JGL reference path: terminal goal idx=%d reached by base_link XY distance %.3f <= %.3f.",
            current_topology_goal_index_, direct_goal_distance,
            reference_terminal_xy_tolerance_);
      }
      else
      {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
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
      if (reference_obstacle_start_.nanoseconds() == 0)
      {
        reference_obstacle_start_ = clock_->now();
        RCLCPP_WARN(logger_, "JGL reference path: obstacle %.3f m ahead on path, enter staged slowdown.",
                 reference_obstacle_distance);
      }

      if ((clock_->now() - reference_obstacle_start_).seconds() >=
          obstacle_wait_time_)
      {
        stopCmd(cmd_vel);
        hard_failure = true;
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                          "JGL reference path: obstacle wait timeout, keep stopped and let topology replan.");
        return false;
      }
    }
    else
    {
      reference_obstacle_start_ = rclcpp::Time(0, 0, clock_->get_clock_type());
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
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 500,
          "JGL reference path: obstacle distance=%.3f m, candidate speed scale=%.2f.",
          reference_obstacle_distance, obstacle_scale);
    }

    if (referenceGoalReached(&goal_path_index,
                             &remaining_to_goal_on_reference,
                             &direct_goal_distance))
    {
      publishReferenceStatus(REFERENCE_PASSED);
    }

    std::vector<geometry_msgs::msg::PoseStamped> local_reference;
    const unsigned int start_index = reference_path_manager_.currentPathIndex();
    const unsigned int end_index =
        std::min(static_cast<unsigned int>(reference_path.poses.size()),
                 start_index + 80U);
    for (unsigned int i = start_index; i < end_index; ++i)
    {
      local_reference.push_back(reference_path.poses[i]);
    }
    publishLocalPlan(local_reference);

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
                      "JGL reference path: follow idx=%u goal_idx=%d v=%.3f w=%.3f curv=%.3f.",
                      reference_path_manager_.currentPathIndex(),
                      current_topology_goal_index_,
                      cmd_vel.linear.x, cmd_vel.angular.z, curvature);
    return true;
  }
  rcl_interfaces::msg::SetParametersResult DWAPlannerROS::onSetParameters(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    const auto key = [this](const std::string &parameter) {
        return plugin_name_ + "." + parameter;
      };
    double new_max_vel_x = max_vel_x;
    double new_brake_distance = brake_distance;
    double new_lfc = lfc;
    double new_forw_num = forwNum;
    double new_pid_pa = pid_PA;
    double new_pid_pb = pid_PB;
    double new_front_distance = frontdis_X;
    double new_start_angle = startrotangle;
    double new_stop_angle = stoprotangle;
    double new_yaw_tolerance = yaw_goal_tolerance;
    double new_xy_tolerance = xy_goal_tolerance;

    for (const auto &parameter : parameters)
    {
      const auto &name = parameter.get_name();
      if (name == key("max_vel_x"))
      {
        new_max_vel_x = parameter.as_double();
      }
      else if (name == key("brake_distance"))
      {
        new_brake_distance = parameter.as_double();
      }
      else if (name == key("lfc"))
      {
        new_lfc = parameter.as_double();
      }
      else if (name == key("forwNum"))
      {
        new_forw_num = parameter.as_double();
      }
      else if (name == key("pid_PA"))
      {
        new_pid_pa = parameter.as_double();
      }
      else if (name == key("pid_PB"))
      {
        new_pid_pb = parameter.as_double();
      }
      else if (name == key("frontdis_X"))
      {
        new_front_distance = parameter.as_double();
      }
      else if (name == key("jgl_rot_start_angle"))
      {
        new_start_angle = parameter.as_double();
      }
      else if (name == key("jgl_rot_stop_angle"))
      {
        new_stop_angle = parameter.as_double();
      }
      else if (name == key("yaw_goal_tolerance"))
      {
        new_yaw_tolerance = parameter.as_double();
      }
      else if (name == key("xy_goal_tolerance"))
      {
        new_xy_tolerance = parameter.as_double();
      }
    }

    if (new_max_vel_x <= 0.0 || new_brake_distance <= 0.0 ||
        new_lfc < 0.1 || new_forw_num < 0.1 || new_forw_num > 0.5 ||
        new_pid_pa < 0.0 || new_pid_pb < 0.0 || new_front_distance < 0.0 ||
        new_start_angle < 0.0 || new_stop_angle < 0.0 ||
        new_yaw_tolerance < 0.0 || new_xy_tolerance < 0.0)
    {
      result.successful = false;
      result.reason = "JGL controller parameter is outside the ROS1-compatible range";
      return result;
    }

    std::lock_guard<std::mutex> lock(control_parameter_mutex_);
    const bool pursuit_changed = new_lfc != lfc || new_forw_num != forwNum;
    const bool speed_changed =
      new_max_vel_x != max_vel_x || new_brake_distance != brake_distance;
    max_vel_x = new_max_vel_x;
    brake_distance = new_brake_distance;
    lfc = new_lfc;
    forwNum = new_forw_num;
    pid_PA = new_pid_pa;
    pid_PB = new_pid_pb;
    frontdis_X = new_front_distance;
    startrotangle = new_start_angle;
    stoprotangle = new_stop_angle;
    yaw_goal_tolerance = new_yaw_tolerance;
    xy_goal_tolerance = new_xy_tolerance;
    xdis = xy_goal_tolerance;
    angle_err_H = yaw_goal_tolerance;
    state4counter = 5;
    state5counter = 5;

    if (speed_changed)
    {
      const double max_acc = max_vel_x * max_vel_x /
        (2.0 * std::max(0.01, brake_distance));
      sp = std::make_unique<SpeedPlan>(max_vel_x, max_acc);
    }
    if (pursuit_changed)
    {
      ps = std::make_unique<Pursuit>(forwNum, lfc);
    }
    return result;
  }

  void DWAPlannerROS::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    parent_ = parent;
    node_ = parent.lock();
    if (!node_)
    {
      throw std::runtime_error("DWAPlannerROS lifecycle node expired");
    }
    plugin_name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    clock_ = node_->get_clock();
    logger_ = node_->get_logger();

    const auto key = [this](const std::string &parameter) {
        return plugin_name_ + "." + parameter;
      };
    max_vel_x = declareOrGet(node_, key("max_vel_x"), 0.4);
    brake_distance = declareOrGet(node_, key("brake_distance"), 1.0);
    lfc = declareOrGet(node_, key("lfc"), 0.3);
    forwNum = declareOrGet(node_, key("forwNum"), 0.1);
    back_distance = declareOrGet(node_, key("back_distance"), 2.5);
    pid_PA = declareOrGet(node_, key("pid_PA"), 0.7);
    pid_PB = declareOrGet(node_, key("pid_PB"), 0.2);
    frontdis_X = declareOrGet(node_, key("frontdis_X"), 0.02);
    startrotangle = declareOrGet(node_, key("jgl_rot_start_angle"), 30.0);
    stoprotangle = declareOrGet(node_, key("jgl_rot_stop_angle"), 15.0);
    yaw_goal_tolerance = declareOrGet(node_, key("yaw_goal_tolerance"), 0.025);
    xy_goal_tolerance = declareOrGet(node_, key("xy_goal_tolerance"), 0.03);
    enable_bspline_reference_path_ = declareOrGet(
        node_, key("enable_bspline_reference_path"), false);
    reference_safe_distance_ = declareOrGet(node_, key("safe_distance"), 0.25);
    reference_fallback_boundary_distance_ = declareOrGet(
        node_, key("reference_fallback_boundary_distance"), 0.15);
    reference_middle_pass_distance_ = declareOrGet(
        node_, key("reference_middle_pass_distance"), 0.25);
    reference_terminal_xy_tolerance_ = declareOrGet(
        node_, key("reference_terminal_xy_tolerance"), 0.08);
    obstacle_wait_time_ = declareOrGet(node_, key("obstacle_wait_time"), 10.0);
    reference_obstacle_slowdown_distance_ = declareOrGet(
        node_, key("obstacle_slowdown_distance"), 1.0);
    reference_obstacle_stop_distance_ = declareOrGet(
        node_, key("obstacle_stop_distance"), 0.7);
    path_deviation_replan_threshold_ = declareOrGet(
        node_, key("path_deviation_replan_threshold"), 0.60);
    fixed_route_mode_.store(declareOrGet(node_, key("fixed_route_mode"), false));

    const double terminal_yaw_tolerance = declareOrGet(
        node_, key("terminal_yaw_tolerance"), 0.04);
    const double terminal_yaw_kp = declareOrGet(node_, key("terminal_yaw_kp"), 1.2);
    const double terminal_yaw_max_speed = declareOrGet(
        node_, key("terminal_yaw_max_speed"), 0.25);
    const double terminal_yaw_min_speed = declareOrGet(
        node_, key("terminal_yaw_min_speed"), 0.06);
    const int terminal_yaw_stable_cycles = declareOrGet(
        node_, key("terminal_yaw_stable_cycles"), 3);
    terminal_yaw_controller_.configure(
        terminal_yaw_tolerance, terminal_yaw_kp,
        terminal_yaw_max_speed, terminal_yaw_min_speed,
        terminal_yaw_stable_cycles);

    reference_safe_distance_ = std::max(0.0, reference_safe_distance_);
    reference_fallback_boundary_distance_ =
        std::max(0.02, reference_fallback_boundary_distance_);
    reference_middle_pass_distance_ = std::max(0.02, reference_middle_pass_distance_);
    reference_terminal_xy_tolerance_ =
        std::max(0.01, reference_terminal_xy_tolerance_);
    obstacle_wait_time_ = std::max(0.0, obstacle_wait_time_);
    reference_obstacle_stop_distance_ =
        std::max(0.0, reference_obstacle_stop_distance_);
    reference_obstacle_slowdown_distance_ = std::max(
        reference_obstacle_stop_distance_ + 0.01,
        reference_obstacle_slowdown_distance_);
    path_deviation_replan_threshold_ =
        std::max(0.05, path_deviation_replan_threshold_);

    g_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        plugin_name_ + "/global_plan", rclcpp::QoS(1));
    l_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        plugin_name_ + "/local_plan", rclcpp::QoS(1));
    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    reference_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "/reference_path", latched_qos);
    reference_path_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "/reference_path_marker", latched_qos);
    reference_status_pub_ =
        node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/bspline_status", rclcpp::QoS(1));
    terminal_motion_state_pub_ =
        node_->create_publisher<std_msgs::msg::UInt8>(
            "/anav/terminal_motion_state", latched_qos);
    fixed_route_mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/anav/fixed_route_mode", rclcpp::QoS(1),
        std::bind(&DWAPlannerROS::fixedRouteModeCallback, this,
                  std::placeholders::_1));

    trajectory_generator_.initialize(node_, plugin_name_);
    reference_path_manager_.initialize(node_, plugin_name_);
    path_follower_.loadParams(node_, plugin_name_);

    // base_local_planner has no ROS2 port. Nav2 DWB is its official successor;
    // delegate only the sampling-DWA branch while retaining all project-specific
    // z-mode, topology, B-spline and pure-pursuit behavior in this wrapper.
    declareOrGet(node_, key("trajectory_generator_name"),
                 std::string("dwb_plugins::StandardTrajectoryGenerator"));
    declareOrGet(node_, key("critics"), std::vector<std::string>{
        "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
        "PathAlign", "PathDist", "GoalDist"});
    dwb_planner_ = std::make_unique<dwb_core::DWBLocalPlanner>();
    dwb_planner_->configure(parent_, plugin_name_, tf_, costmap_ros_);

    const double max_acc = max_vel_x * max_vel_x /
        (2.0 * std::max(0.01, brake_distance));
    sp = std::make_unique<SpeedPlan>(max_vel_x, max_acc);
    ps = std::make_unique<Pursuit>(forwNum, lfc);
    state4counter = 5;
    state5counter = 5;
    xdis = xy_goal_tolerance;
    angle_err_H = yaw_goal_tolerance;
    reference_obstacle_start_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    begin = rclcpp::Time(0, 0, clock_->get_clock_type());
    logfilename = "/tmp/testrecordgoal" + getCurrentTimestamp() + ".txt";
    // ROS2 parameters replace the ROS1 dynamic_reconfigure server for the
    // project-specific control gains and tolerances. DWB owns its standard
    // sampling parameters through its own callback.
    parameter_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&DWAPlannerROS::onSetParameters, this, std::placeholders::_1));
    initialized_ = true;
  }

  void DWAPlannerROS::activate()
  {
    g_plan_pub_->on_activate();
    l_plan_pub_->on_activate();
    reference_path_pub_->on_activate();
    reference_path_marker_pub_->on_activate();
    reference_status_pub_->on_activate();
    terminal_motion_state_pub_->on_activate();
    published_terminal_motion_state_ = -1;
    publishTerminalMotionState(TERMINAL_TRACKING);
    dwb_planner_->activate();
  }

  void DWAPlannerROS::deactivate()
  {
    dwb_planner_->deactivate();
    g_plan_pub_->on_deactivate();
    l_plan_pub_->on_deactivate();
    reference_path_pub_->on_deactivate();
    reference_path_marker_pub_->on_deactivate();
    reference_status_pub_->on_deactivate();
    terminal_motion_state_pub_->on_deactivate();
  }

  void DWAPlannerROS::cleanup()
  {
    waitForReferencePathJob();
    if (dwb_planner_)
    {
      dwb_planner_->cleanup();
    }
    parameter_callback_handle_.reset();
    dwb_planner_.reset();
    fixed_route_mode_sub_.reset();
    g_plan_pub_.reset();
    l_plan_pub_.reset();
    reference_path_pub_.reset();
    reference_path_marker_pub_.reset();
    reference_status_pub_.reset();
    terminal_motion_state_pub_.reset();
    sp.reset();
    ps.reset();
    initialized_ = false;
  }
 void DWAPlannerROS::setPlan(const nav_msgs::msg::Path &path)
{
    if (!isInitialized()) {
        throw std::runtime_error(
            "DWAPlannerROS has not been configured");
    }
    if (path.poses.empty()) {
        throw std::invalid_argument("DWAPlannerROS received an empty plan");
    }
    global_plan_ = path;
    const auto &orig_global_plan = global_plan_.poses;
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
      for (std::size_t j = 0; j < orig_global_plan.size(); j++) {
          geometry_msgs::msg::PoseStamped pose = orig_global_plan[j];
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
        RCLCPP_WARN(logger_, "Fixed route: discarded a stale global-planner WAIT marker; "
                 "live costmap obstacle checking remains active on the locked path.");
      }
      current_topology_goal_index_ = reference_path_manager_.goalIndex(linePath.back());
      reference_obstacle_start_ = rclcpp::Time(0, 0, clock_->get_clock_type());
        std::cout<<"plan points()=  "<<linePath.size()<<std::endl;
    }
    else
    {
      current_topology_goal_index_ = -1;
      reference_obstacle_start_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      reference_goal_reached_ = false;
      legacy_line_forced_goal_index_ = -1;
      terminal_yaw_controller_.reset();
    }
    RCLCPP_INFO(logger_, "Got new plan");
    dwb_planner_->setPlan(global_plan_);
}

 bool DWAPlannerROS::lineComputeVelocityCommands_modJGL(std::vector<geometry_msgs::msg::PoseStamped> linePath, geometry_msgs::msg::Twist &cmd_vel)
 {
    (void)linePath;
    stopCmd(cmd_vel);
    return false;
 }
   
void DWAPlannerROS::computeRelativePosition(const geometry_msgs::msg::PoseStamped& p, const geometry_msgs::msg::PoseStamped& q)
{
    // 导航路径是二维的，position.z 被全局规划器用作控制模式标记
    // (1: 纯追踪, 2: 停止等待)。不能将该 z 值和实车 roll/pitch 参与
    // 三维旋转，否则会在 Qtar.x/y 中产生虚假的终点距离。
    const double dx = q.pose.position.x - p.pose.position.x;
    const double dy = q.pose.position.y - p.pose.position.y;
    const double robot_yaw = tf2::getYaw(p.pose.orientation);
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
        RCLCPP_ERROR(logger_, "This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    //pure oursuit 判断是否到达目标点，包括距离和角度

    if (useLine>0) {
        RCLCPP_INFO_STREAM(logger_, "distance to goal:" << comDistance(current_pose_,linePath.back()) );
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
            RCLCPP_INFO(logger_, "Goal reached by reference path progress");
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
            RCLCPP_INFO(logger_, "Goal reached by reference path progress "
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

            RCLCPP_INFO(logger_, "Goal reached by rotation");
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
    if (!goal_checker_ || global_plan_.poses.empty()) {
        return false;
    }
    return goal_checker_->isGoalReached(
        current_pose_.pose, global_plan_.poses.back().pose, current_velocity_);
}


  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    nav_msgs::msg::Path message;
    message.header.stamp = clock_->now();
    message.header.frame_id = costmap_ros_->getGlobalFrameID();
    message.poses = path;
    l_plan_pub_->publish(message);
  }

  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    nav_msgs::msg::Path message;
    message.header.stamp = clock_->now();
    message.header.frame_id = costmap_ros_->getGlobalFrameID();
    message.poses = path;
    g_plan_pub_->publish(message);
  }

  DWAPlannerROS::~DWAPlannerROS()
  {
    waitForReferencePathJob();
  }

  void DWAPlannerROS::cmd_pub(std::vector<geometry_msgs::msg::PoseStamped> Point, geometry_msgs::msg::PoseStamped pose, std::vector<double> &fov_speed)
{

    double length = comDistance(Point.back(), Point[0]); //直线总长度
    double d = comDistance(pose, Point[0]);              //机器人当前位置距离直线起点距离
    sp->speedComputeLine(d, length, fov_speed, 1);       //T型速度曲线计算每点速度值
}

bool DWAPlannerROS::comparePose(geometry_msgs::msg::PoseStamped p,geometry_msgs::msg::PoseStamped q)
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

double DWAPlannerROS::comDistance(geometry_msgs::msg::PoseStamped p, geometry_msgs::msg::PoseStamped q)
{
    double distance = sqrt((p.pose.position.x - q.pose.position.x) * (p.pose.position.x - q.pose.position.x) + (p.pose.position.y - q.pose.position.y) * (p.pose.position.y - q.pose.position.y));
    return distance;
}


//Eigen::Vector3d DWAPlannerROS::toEigenVector3d(const geometry_msgs::msg::PoseStamped& pose) {
//    return Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
//}

Eigen::Vector2d  DWAPlannerROS::toEigenVector2d(const geometry_msgs::msg::PoseStamped& pose) {
    return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}
/*
double DWAPlannerROS::distanceToLine(const geometry_msgs::msg::PoseStamped& A, const geometry_msgs::msg::PoseStamped& B, const geometry_msgs::msg::PoseStamped& C) {
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
double DWAPlannerROS::signedDistanceToLine(const geometry_msgs::msg::PoseStamped& A, const geometry_msgs::msg::PoseStamped& B, const geometry_msgs::msg::PoseStamped& C) {
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



bool DWAPlannerROS::lineComputeVelocityCommands(std::vector<geometry_msgs::msg::PoseStamped> linePath, geometry_msgs::msg::Twist &cmd_vel)
{
  if (linePath.size() < 2)
  {
    return false;
  }

  computeRelativePosition(current_pose_,linePath.back());
  dis2line = signedDistanceToLine(linePath.at(0),linePath.back(),current_pose_);

  RCLCPP_INFO_STREAM(logger_, "distance to goal in xy :" << sqrt(Qtar.pose.position.x*Qtar.pose.position.x+Qtar.pose.position.y*Qtar.pose.position.y));
  std::cout<<"Qx err = "<<Qtar.pose.position.x<<"Qy err = "<<Qtar.pose.position.y<<"front dis="<<frontdis_X<<std::endl;
           // ss1<<"goal: x="<<linePath.back().pose.position.x<<" y="<<linePath.back().pose.position.y<<std::endl;
            //ss1<<"stop cmd at: x="<<current_pose_.pose.position.x<<" y="<<current_pose_.pose.position.y<<std::endl;
            //

  for (std::size_t i = 0; i < linePath.size(); i++)
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
 state.yaw=tf2::getYaw(current_pose_.pose.orientation);


  //计算机器人当前位置距离终点的角度误差
  double yaw = tf2::getYaw(linePath.back().pose.orientation);
  goal_yaw_err = yaw - state.yaw;      
  if (goal_yaw_err <= -PI)
  {
    goal_yaw_err = goal_yaw_err + 2 * PI;
  }
  else if (goal_yaw_err >= PI)
  {
    goal_yaw_err = goal_yaw_err - 2 * PI;
  }
 // RCLCPP_INFO_STREAM(logger_,  "self rotation:" << goal_yaw_err );

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
  //RCLCPP_INFO_STREAM(logger_, "angle_err:"<<angle_err);
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
    RCLCPP_INFO(logger_, "Terminal position captured at %.3f m; switch to final-yaw preparation.",
             xy_error);
    publishTerminalMotionState(TERMINAL_POSITION_CAPTURED);
  }else if(status==1 && xy_error<=terminal_no_realign_distance){
    status = 0;
    RCLCPP_WARN(logger_, "Suppress path realignment near goal at %.3f m.", xy_error);
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
      RCLCPP_INFO(logger_, "Terminal yaw stable; goal completed.");
    }
  }
  else if((status==4) &&(state4counter<=0) ){
    status = 0;
  }
  else if((status==5) &&(state5counter<=0) ){
    status = 2;
    publishTerminalMotionState(TERMINAL_ROTATING);
  }

  RCLCPP_INFO_STREAM(logger_, "---------------------CONTROL STATUS:"<<status);


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
   
    //RCLCPP_INFO_STREAM(logger_,  "alpha:" << alpha<<" PA"<<pid_PA<<" PB"<<pid_PB);

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


  geometry_msgs::msg::TwistStamped DWAPlannerROS::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker)
  {
    std::lock_guard<std::mutex> lock(control_parameter_mutex_);
    current_pose_ = pose;
    current_velocity_ = velocity;
    goal_checker_ = goal_checker;
    if (global_plan_.poses.empty())
    {
      throw std::runtime_error("DWAPlannerROS has no plan");
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = clock_->now();
    command.header.frame_id = costmap_ros_->getBaseFrameID();
    auto &cmd_vel = command.twist;
    std::vector<geometry_msgs::msg::PoseStamped> plan = global_plan_.poses;

    if (useLine>0) {
      begin = clock_->now();
      publishGlobalPlan(plan);
      consumeReferencePathJob();
      maybeStartReferencePathJob();
      bool reference_hard_failure = false;
      if (computeReferenceVelocityCommands(cmd_vel, reference_hard_failure))
      {
        return command;
      }
      if (reference_hard_failure)
      {
        stopCmd(cmd_vel);
        throw std::runtime_error("reference-path generation failed");
      }
      // Only the legacy straight-line fallback keeps the original hard-stop
      // check. Active reference paths were already handled above with staged
      // slowdown, so the two policies cannot mask each other.
      if (fixedRouteBlocked())
      {
        path_follower_.reset();
        stopCmd(cmd_vel);
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Fixed route: obstacle detected on the locked legacy path; wait in place until it clears.");
        return command;
      }
      if (!lineComputeVelocityCommands(linePath, cmd_vel))
      {
        throw std::runtime_error("legacy line controller failed");
      }
      return command;
    }
    //modify to back up
    if (useLine < 0 &&
        begin + rclcpp::Duration::from_seconds(back_distance) > clock_->now())
    {
        RCLCPP_INFO(logger_, "backing up");
        cmd_vel.linear.x=-0.1;
        cmd_vel.linear.y=0;
        cmd_vel.angular.z=0;
        return command;
    }

    RCLCPP_DEBUG(logger_, "Delegating %zu-point sampling plan to Nav2 DWB", plan.size());
    return dwb_planner_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  void DWAPlannerROS::setSpeedLimit(
      const double &speed_limit, const bool &percentage)
  {
    std::lock_guard<std::mutex> lock(control_parameter_mutex_);
    dwb_planner_->setSpeedLimit(speed_limit, percentage);
    if (!sp)
    {
      return;
    }
    if (speed_limit <= 0.0)
    {
      sp->vmax = max_vel_x;
    }
    else if (percentage)
    {
      sp->vmax = max_vel_x * std::clamp(speed_limit, 0.0, 100.0) / 100.0;
    }
    else
    {
      sp->vmax = std::min(max_vel_x, speed_limit);
    }
  }

  float DWAPlannerROS::base_plan_direction_check(const std::vector<geometry_msgs::msg::PoseStamped> &orig_global_plan,
                                                 geometry_msgs::msg::PoseStamped &robot_pose_)
  {
    if (orig_global_plan.size() < 10)
    {
      return 0;
    }

    geometry_msgs::msg::Point robotposition = robot_pose_.pose.position;


    double robotangle = tf2::getYaw(robot_pose_.pose.orientation);                    //通过赋值方式就可以得到需要的绕z轴偏转角度了
                                                          // RCLCPP_ERROR(logger_, "the robot angle is %f",robotangle);

    geometry_msgs::msg::Point targetPosition = orig_global_plan[9].pose.position;


        double direction_angle =
            atan2(targetPosition.y - robotposition.y, targetPosition.x - robotposition.x) ;
        // RCLCPP_ERROR(logger_, "the robot pose %f,%f, planpose is %f,%f",robotposition.x,robotposition.y,targetPosition.x,targetPosition.y);
        //  RCLCPP_ERROR(logger_, "the robot planned angle is %f, distance = %f",direction_angle,d);

        double diffangle = direction_angle - robotangle;
        
        if (diffangle <= -PI)
        {
        diffangle = diffangle + 2*PI;
        }
        if (diffangle >= PI)
        {
        diffangle = diffangle - 2*PI;
        }

        //  RCLCPP_ERROR(logger_, "the change angle is %f",diffangle);

        return (float)diffangle;


  }

};
