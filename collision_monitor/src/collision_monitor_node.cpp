#include <algorithm>
#include <cstdio>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmd_vel_arbiter/ArbitratedCommand.h>
#include <collision_monitor/collision_checker.h>
#include <costmap_2d/footprint.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>

namespace collision_monitor {
namespace {

bool IsFinite(const geometry_msgs::Twist& twist) {
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

bool IsZero(const geometry_msgs::Twist& twist, double epsilon) {
  return std::abs(twist.linear.x) <= epsilon &&
         std::abs(twist.linear.y) <= epsilon &&
         std::abs(twist.linear.z) <= epsilon &&
         std::abs(twist.angular.x) <= epsilon &&
         std::abs(twist.angular.y) <= epsilon &&
         std::abs(twist.angular.z) <= epsilon;
}

double YawDifference(double a, double b) {
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

void AddDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                        const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

std::string ToString(double value) {
  if (!std::isfinite(value)) {
    return "-1";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.4f", value);
  return buffer;
}

struct MotionProfile {
  std::string name;
  double footprint_padding = 0.15;
  double max_linear_x = 0.0;
  double max_linear_y = 0.0;
  double max_angular_z = 0.0;
  bool allow_reverse = false;
  bool allow_lateral = false;
  CollisionChecker checker;
};

}  // namespace

class CollisionMonitorNode {
 public:
  CollisionMonitorNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    LoadParameters();
    LoadFootprint();

    candidate_sub_ = nh_.subscribe(candidate_topic_, 1,
                                   &CollisionMonitorNode::CandidateCallback,
                                   this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1,
                              &CollisionMonitorNode::OdomCallback, this);
    static_map_sub_ = nh_.subscribe(static_map_topic_, 1,
                                    &CollisionMonitorNode::StaticMapCallback,
                                    this);
    local_map_sub_ = nh_.subscribe(local_map_topic_, 1,
                                   &CollisionMonitorNode::LocalMapCallback,
                                   this);
    output_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 1, false);
    status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        status_topic_, 1, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        marker_topic_, 1, false);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / monitor_rate_),
                                 &CollisionMonitorNode::TimerCallback, this);

    ROS_INFO(
        "collision_monitor: %s -> %s at %.1f Hz, frames=%s/%s, "
        "padding(nav/tag/teleop)=%.3f/%.3f/%.3f",
        candidate_topic_.c_str(), output_topic_.c_str(), monitor_rate_,
        global_frame_.c_str(), base_frame_.c_str(),
        navigation_profile_.footprint_padding,
        tag_profile_.footprint_padding, teleop_profile_.footprint_padding);
  }

  ~CollisionMonitorNode() {
    geometry_msgs::Twist zero;
    for (int i = 0; i < 3; ++i) {
      output_pub_.publish(zero);
    }
  }

 private:
  void LoadParameters() {
    navigation_profile_.name = "nav";
    navigation_profile_.allow_reverse = false;
    navigation_profile_.allow_lateral = false;
    tag_profile_.name = "tag";
    tag_profile_.allow_reverse = true;
    tag_profile_.allow_lateral = true;
    teleop_profile_.name = "teleop";
    teleop_profile_.allow_reverse = false;
    teleop_profile_.allow_lateral = false;

    private_nh_.param("candidate_topic", candidate_topic_,
                      std::string("/cmd_vel/candidate"));
    private_nh_.param("output_topic", output_topic_, std::string("/cmd_vel"));
    private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh_.param("static_map_topic", static_map_topic_,
                      std::string("/map"));
    private_nh_.param("local_map_topic", local_map_topic_,
                      std::string("/mxb_move_base/local_costmap/costmap"));
    private_nh_.param("status_topic", status_topic_,
                      std::string("/collision_monitor/status"));
    private_nh_.param("marker_topic", marker_topic_,
                      std::string("/collision_monitor/markers"));
    private_nh_.param("global_frame", global_frame_, std::string("map"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("teleop_source", teleop_source_,
                      std::string("teleop"));
    private_nh_.param("tag_source", tag_source_, std::string("tag"));
    private_nh_.param("navigation_source", navigation_source_,
                      std::string("nav"));

    private_nh_.param("monitor_rate", monitor_rate_, 50.0);
    private_nh_.param("candidate_timeout", candidate_timeout_, 0.25);
    private_nh_.param("odom_timeout", odom_timeout_, 0.20);
    private_nh_.param("tf_timeout", tf_timeout_, 0.30);
    private_nh_.param("local_map_timeout", local_map_timeout_, 0.30);
    double legacy_padding = 0.15;
    private_nh_.param("footprint_padding", legacy_padding, 0.15);
    navigation_profile_.footprint_padding = legacy_padding;
    teleop_profile_.footprint_padding = legacy_padding;
    tag_profile_.footprint_padding = 0.08;
    private_nh_.param("navigation_footprint_padding",
                      navigation_profile_.footprint_padding, legacy_padding);
    private_nh_.param("tag_footprint_padding",
                      tag_profile_.footprint_padding, 0.08);
    private_nh_.param("teleop_footprint_padding",
                      teleop_profile_.footprint_padding, legacy_padding);

    private_nh_.param("max_navigation_linear_x",
                      navigation_profile_.max_linear_x, 0.20);
    navigation_profile_.max_linear_y = 0.0;
    private_nh_.param("max_navigation_angular_z",
                      navigation_profile_.max_angular_z, 0.50);
    private_nh_.param("max_tag_linear_x", tag_profile_.max_linear_x, 0.05);
    private_nh_.param("max_tag_linear_y", tag_profile_.max_linear_y, 0.05);
    private_nh_.param("max_tag_angular_z", tag_profile_.max_angular_z, 0.15);
    private_nh_.param("max_teleop_linear_x", teleop_profile_.max_linear_x,
                      0.35);
    teleop_profile_.max_linear_y = 0.0;
    private_nh_.param("max_teleop_angular_z", teleop_profile_.max_angular_z,
                      1.10);
    private_nh_.param("zero_epsilon", zero_epsilon_, 1e-4);
    private_nh_.param("reaction_time", reaction_time_, 0.30);
    private_nh_.param("preview_time", preview_time_, 2.0);
    private_nh_.param("clear_hold_time", clear_hold_time_, 0.50);
    private_nh_.param("source_transition_hold_time",
                      source_transition_hold_time_, 0.20);
    private_nh_.param("transition_stopped_linear",
                      transition_stopped_linear_, 0.01);
    private_nh_.param("transition_stopped_angular",
                      transition_stopped_angular_, 0.02);
    private_nh_.param("max_pose_jump_distance", max_pose_jump_distance_,
                      0.50);
    private_nh_.param("max_pose_jump_yaw", max_pose_jump_yaw_, 0.80);
    private_nh_.param("pose_jump_hold_time", pose_jump_hold_time_, 0.50);
    private_nh_.param("static_occupied_threshold",
                      static_policy_.occupied_threshold, 100);
    private_nh_.param("local_occupied_threshold",
                      local_policy_.occupied_threshold, 100);
    private_nh_.param("static_unknown_is_obstacle",
                      static_policy_.unknown_is_obstacle, true);
    private_nh_.param("local_unknown_is_obstacle",
                      local_policy_.unknown_is_obstacle, false);
    private_nh_.param("static_outside_is_obstacle",
                      static_policy_.outside_is_obstacle, true);
    private_nh_.param("local_outside_is_obstacle",
                      local_policy_.outside_is_obstacle, false);
    private_nh_.param("max_dt", rollout_options_.max_dt, 0.02);
    private_nh_.param("max_corner_step", rollout_options_.max_corner_step,
                      0.025);
    private_nh_.param("linear_accel", rollout_options_.linear_accel, 3.0);
    private_nh_.param("angular_accel", rollout_options_.angular_accel, 4.2);
    private_nh_.param("linear_decel", rollout_options_.linear_decel, 0.50);
    private_nh_.param("angular_decel", rollout_options_.angular_decel, 0.80);

    const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    const auto finite_nonnegative = [](double value) {
      return std::isfinite(value) && value >= 0.0;
    };
    if (!finite_positive(monitor_rate_) ||
        !finite_positive(candidate_timeout_) ||
        !finite_positive(odom_timeout_) || !finite_positive(tf_timeout_) ||
        !finite_positive(local_map_timeout_) ||
        !finite_nonnegative(navigation_profile_.footprint_padding) ||
        !finite_nonnegative(tag_profile_.footprint_padding) ||
        !finite_nonnegative(teleop_profile_.footprint_padding) ||
        !finite_nonnegative(navigation_profile_.max_linear_x) ||
        !finite_nonnegative(navigation_profile_.max_angular_z) ||
        !finite_nonnegative(tag_profile_.max_linear_x) ||
        !finite_nonnegative(tag_profile_.max_linear_y) ||
        !finite_nonnegative(tag_profile_.max_angular_z) ||
        !finite_nonnegative(teleop_profile_.max_linear_x) ||
        !finite_nonnegative(teleop_profile_.max_angular_z) ||
        !finite_positive(zero_epsilon_) ||
        !finite_nonnegative(reaction_time_) ||
        !finite_nonnegative(preview_time_) ||
        !finite_nonnegative(clear_hold_time_) ||
        !finite_nonnegative(source_transition_hold_time_) ||
        !finite_positive(transition_stopped_linear_) ||
        !finite_positive(transition_stopped_angular_) ||
        !finite_positive(max_pose_jump_distance_) ||
        !finite_positive(max_pose_jump_yaw_) ||
        !finite_nonnegative(pose_jump_hold_time_) ||
        !finite_positive(rollout_options_.max_dt) ||
        !finite_positive(rollout_options_.max_corner_step) ||
        !finite_positive(rollout_options_.linear_accel) ||
        !finite_positive(rollout_options_.angular_accel) ||
        !finite_positive(rollout_options_.linear_decel) ||
        !finite_positive(rollout_options_.angular_decel)) {
      throw std::runtime_error(
          "collision_monitor parameters must be finite and positive where required");
    }

    monitor_rate_ = std::max(1.0, monitor_rate_);
    candidate_timeout_ = std::max(0.02, candidate_timeout_);
    odom_timeout_ = std::max(0.02, odom_timeout_);
    tf_timeout_ = std::max(0.02, tf_timeout_);
    local_map_timeout_ = std::max(0.02, local_map_timeout_);
    reaction_time_ = std::max(0.0, reaction_time_);
    preview_time_ = std::max(0.0, preview_time_);
    clear_hold_time_ = std::max(0.0, clear_hold_time_);
    static_policy_.occupied_threshold =
        std::max(0, std::min(100, static_policy_.occupied_threshold));
    local_policy_.occupied_threshold =
        std::max(0, std::min(100, local_policy_.occupied_threshold));
  }

  void LoadFootprint() {
    XmlRpc::XmlRpcValue footprint_xml;
    if (!private_nh_.getParam("footprint", footprint_xml)) {
      ROS_FATAL("collision_monitor: required private parameter ~footprint is missing");
      throw std::runtime_error("missing footprint");
    }
    std::vector<geometry_msgs::Point> footprint;
    try {
      footprint = costmap_2d::makeFootprintFromXMLRPC(
          footprint_xml, private_nh_.resolveName("footprint"));
    } catch (const std::exception& exception) {
      ROS_FATAL("collision_monitor: invalid footprint: %s", exception.what());
      throw;
    }
    std::string error;
    MotionProfile* profiles[] = {
        &navigation_profile_, &tag_profile_, &teleop_profile_};
    for (MotionProfile* profile : profiles) {
      if (!profile->checker.setFootprint(
              footprint, profile->footprint_padding, &error)) {
        ROS_FATAL("collision_monitor: invalid %s footprint: %s",
                  profile->name.c_str(), error.c_str());
        throw std::runtime_error(error);
      }
    }
  }

  MotionProfile* ProfileForSource(const std::string& source) {
    if (source == navigation_source_) {
      return &navigation_profile_;
    }
    if (source == tag_source_) {
      return &tag_profile_;
    }
    if (source == teleop_source_) {
      return &teleop_profile_;
    }
    return nullptr;
  }

  const MotionProfile* ProfileForSource(const std::string& source) const {
    if (source == navigation_source_) {
      return &navigation_profile_;
    }
    if (source == tag_source_) {
      return &tag_profile_;
    }
    if (source == teleop_source_) {
      return &teleop_profile_;
    }
    return nullptr;
  }

  void CandidateCallback(
      const cmd_vel_arbiter::ArbitratedCommand::ConstPtr& message) {
    candidate_ = *message;
    candidate_received_at_ = ros::WallTime::now();
    have_candidate_ = true;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    odom_ = *message;
    odom_received_at_ = ros::WallTime::now();
    have_odom_ = true;
  }

  void StaticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message) {
    static_map_ = *message;
    have_static_map_ = true;
  }

  void LocalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message) {
    local_map_ = *message;
    local_map_received_at_ = ros::WallTime::now();
    have_local_map_ = true;
  }

  bool MapValid(const nav_msgs::OccupancyGrid& map) const {
    const geometry_msgs::Quaternion& orientation = map.info.origin.orientation;
    const double quaternion_norm =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    return map.info.width > 0 && map.info.height > 0 &&
           std::isfinite(map.info.resolution) && map.info.resolution > 0.0 &&
           std::isfinite(map.info.origin.position.x) &&
           std::isfinite(map.info.origin.position.y) &&
           std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
           quaternion_norm > 1e-9 &&
           map.data.size() == static_cast<std::size_t>(map.info.width) *
                                  static_cast<std::size_t>(map.info.height) &&
           map.header.frame_id == global_frame_;
  }

  bool GetRobotPose(Pose2D& pose, std::string& reason) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(global_frame_, base_frame_,
                                             ros::Time(0), ros::Duration(0.01));
    } catch (const tf2::TransformException& exception) {
      reason = std::string("TF_UNAVAILABLE: ") + exception.what();
      return false;
    }

    if (transform.header.stamp.isZero()) {
      reason = "TF_HAS_ZERO_STAMP";
      return false;
    }
    const double age = (ros::Time::now() - transform.header.stamp).toSec();
    if (!std::isfinite(age) || age > tf_timeout_ || age < -0.10) {
      reason = "TF_STALE";
      return false;
    }

    pose.x = transform.transform.translation.x;
    pose.y = transform.transform.translation.y;
    const geometry_msgs::Quaternion& orientation =
        transform.transform.rotation;
    const double quaternion_norm =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    pose.yaw = tf2::getYaw(transform.transform.rotation);
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.yaw) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w) || quaternion_norm <= 1e-9) {
      reason = "TF_NON_FINITE";
      return false;
    }

    if (transform.header.stamp != last_tf_stamp_) {
      if (have_last_pose_ &&
          (std::hypot(pose.x - last_pose_.x, pose.y - last_pose_.y) >
               max_pose_jump_distance_ ||
           YawDifference(pose.yaw, last_pose_.yaw) > max_pose_jump_yaw_)) {
        pose_jump_until_ =
            ros::WallTime::now() + ros::WallDuration(pose_jump_hold_time_);
        ROS_ERROR("collision_monitor: localization jump detected");
      }
      last_pose_ = pose;
      last_tf_stamp_ = transform.header.stamp;
      have_last_pose_ = true;
    }
    if (!pose_jump_until_.isZero() && ros::WallTime::now() < pose_jump_until_) {
      reason = "TF_JUMP";
      return false;
    }
    return true;
  }

  bool DataReady(const ros::WallTime& now, Pose2D& pose,
                 std::string& reason) {
    if (!footprintReady()) {
      reason = "FOOTPRINT_NOT_READY";
      return false;
    }
    if (!have_odom_ || (now - odom_received_at_).toSec() > odom_timeout_) {
      reason = "ODOM_STALE";
      return false;
    }
    if (!IsFinite(odom_.twist.twist)) {
      reason = "ODOM_NON_FINITE";
      return false;
    }
    if (!have_static_map_ || !MapValid(static_map_)) {
      reason = "STATIC_MAP_NOT_READY";
      return false;
    }
    if (!have_local_map_ ||
        (now - local_map_received_at_).toSec() > local_map_timeout_ ||
        !MapValid(local_map_)) {
      reason = "LOCAL_MAP_NOT_READY";
      return false;
    }
    return GetRobotPose(pose, reason);
  }

  bool footprintReady() const {
    return navigation_profile_.checker.footprint().size() >= 3 &&
           tag_profile_.checker.footprint().size() >= 3 &&
           teleop_profile_.checker.footprint().size() >= 3;
  }

  bool SupportedCommand(const MotionProfile& profile,
                        const geometry_msgs::Twist& command,
                        std::string& reason) const {
    if (!IsFinite(command)) {
      reason = "NON_FINITE_COMMAND";
      return false;
    }
    if (std::abs(command.linear.z) > zero_epsilon_ ||
        std::abs(command.angular.x) > zero_epsilon_ ||
        std::abs(command.angular.y) > zero_epsilon_) {
      reason = "UNSUPPORTED_COMMAND_DOF";
      return false;
    }
    if ((!profile.allow_reverse && command.linear.x < -zero_epsilon_) ||
        (!profile.allow_lateral &&
         std::abs(command.linear.y) > zero_epsilon_)) {
      reason = "UNSUPPORTED_COMMAND_DOF";
      return false;
    }
    if (std::abs(command.linear.x) >
            profile.max_linear_x + zero_epsilon_ ||
        std::abs(command.linear.y) >
            profile.max_linear_y + zero_epsilon_ ||
        std::abs(command.angular.z) >
            profile.max_angular_z + zero_epsilon_) {
      reason = "COMMAND_LIMIT_EXCEEDED";
      return false;
    }
    return true;
  }

  bool SupportedMeasuredMotion(const MotionProfile& profile,
                               const geometry_msgs::Twist& measured,
                               std::string& reason) const {
    if (!IsFinite(measured)) {
      reason = "NON_FINITE_MEASURED_MOTION";
      return false;
    }
    if (std::abs(measured.linear.z) > transition_stopped_linear_ ||
        std::abs(measured.angular.x) > transition_stopped_angular_ ||
        std::abs(measured.angular.y) > transition_stopped_angular_) {
      reason = "UNSUPPORTED_MEASURED_DOF";
      return false;
    }
    if ((!profile.allow_reverse &&
         measured.linear.x < -transition_stopped_linear_) ||
        (!profile.allow_lateral &&
         std::abs(measured.linear.y) > transition_stopped_linear_)) {
      reason = "UNSUPPORTED_MEASURED_DOF";
      return false;
    }
    return true;
  }

  bool MeasuredStopped(const geometry_msgs::Twist& measured) const {
    return std::hypot(measured.linear.x, measured.linear.y) <=
               transition_stopped_linear_ &&
           std::abs(measured.linear.z) <= transition_stopped_linear_ &&
           std::abs(measured.angular.x) <= transition_stopped_angular_ &&
           std::abs(measured.angular.y) <= transition_stopped_angular_ &&
           std::abs(measured.angular.z) <= transition_stopped_angular_;
  }

  bool SourceTransitionBlocks(const std::string& source,
                              const ros::WallTime& now) {
    if (source == active_source_ && pending_source_.empty()) {
      return false;
    }
    if (pending_source_ != source) {
      pending_source_ = source;
      transition_stopped_since_ = ros::WallTime();
      ForceStoppedScale();
    }
    if (!MeasuredStopped(odom_.twist.twist)) {
      transition_stopped_since_ = ros::WallTime();
      return true;
    }
    if (transition_stopped_since_.isZero()) {
      transition_stopped_since_ = now;
      return true;
    }
    if ((now - transition_stopped_since_).toSec() <
        source_transition_hold_time_) {
      return true;
    }
    active_source_ = source;
    pending_source_.clear();
    transition_stopped_since_ = ros::WallTime();
    ForceStoppedScale();
    return false;
  }

  double ApplyScaleHysteresis(double desired, const ros::WallTime& now) {
    desired = std::max(0.0, std::min(1.0, desired));
    if (desired < applied_scale_ - 1e-9) {
      applied_scale_ = desired;
      scale_increase_since_ = ros::WallTime();
      return applied_scale_;
    }
    if (desired <= applied_scale_ + 1e-9) {
      scale_increase_since_ = ros::WallTime();
      return applied_scale_;
    }
    if (scale_increase_since_.isZero()) {
      scale_increase_since_ = now;
      return applied_scale_;
    }
    if ((now - scale_increase_since_).toSec() < clear_hold_time_) {
      return applied_scale_;
    }
    static const double kLevels[] = {0.0, 0.25, 0.50, 0.75, 1.0};
    for (double level : kLevels) {
      if (level > applied_scale_ + 1e-9) {
        applied_scale_ = std::min(level, desired);
        break;
      }
    }
    scale_increase_since_ = ros::WallTime();
    return applied_scale_;
  }

  void ForceStoppedScale() {
    applied_scale_ = 0.0;
    scale_increase_since_ = ros::WallTime();
  }

  void TimerCallback(const ros::WallTimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    geometry_msgs::Twist output;
    std::string state = "STOPPED";
    std::string reason;
    double scale = 0.0;
    double collision_time = -1.0;
    CollisionResult visualization;

    if (!have_candidate_ ||
        (now - candidate_received_at_).toSec() > candidate_timeout_) {
      ForceStoppedScale();
      reason = "CANDIDATE_STALE";
      Publish(output, state, reason, scale, collision_time, visualization,
              std::string());
      return;
    }

    const geometry_msgs::Twist& command = candidate_.command;
    if (!IsFinite(command)) {
      ForceStoppedScale();
      reason = "CANDIDATE_NON_FINITE";
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    if (IsZero(command, zero_epsilon_)) {
      state = "STOP_COMMAND";
      reason = "ZERO_COMMAND_PASSED";
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    Pose2D current_pose;
    if (!DataReady(now, current_pose, reason)) {
      if (candidate_.source == teleop_source_) {
        output = command;
        state = "MANUAL_BYPASS";
        scale = 1.0;
        active_source_ = teleop_source_;
        pending_source_.clear();
        transition_stopped_since_ = ros::WallTime();
        applied_scale_ = 1.0;
        scale_increase_since_ = ros::WallTime();
        ROS_WARN_THROTTLE(1.0,
                          "collision_monitor: teleop bypass because %s",
                          reason.c_str());
      } else {
        ForceStoppedScale();
        state = "DATA_NOT_READY";
      }
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    MotionProfile* profile = ProfileForSource(candidate_.source);
    if (profile == nullptr) {
      ForceStoppedScale();
      state = "UNSUPPORTED_MOTION";
      reason = "UNKNOWN_COMMAND_SOURCE";
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    std::string unsupported_reason;
    if (!SupportedCommand(*profile, command, unsupported_reason)) {
      ForceStoppedScale();
      state = "UNSUPPORTED_MOTION";
      reason = unsupported_reason;
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    MotionState initial;
    initial.pose = current_pose;
    initial.linear_x = odom_.twist.twist.linear.x;
    initial.linear_y = odom_.twist.twist.linear.y;
    initial.angular_z = odom_.twist.twist.angular.z;

    if (SourceTransitionBlocks(candidate_.source, now)) {
      const CollisionResult transition_stop =
          navigation_profile_.checker.simulate(
              initial, initial.linear_x, initial.linear_y, initial.angular_z,
              reaction_time_, static_map_, static_policy_, local_map_,
              local_policy_, rollout_options_);
      visualization = transition_stop;
      if (transition_stop.collision) {
        state = "EMERGENCY_STOP";
        reason = transition_stop.static_collision
                     ? "STATIC_COLLISION_DURING_PROFILE_TRANSITION"
                     : "LOCAL_COLLISION_DURING_PROFILE_TRANSITION";
        collision_time = transition_stop.collision_time;
      } else {
        state = "PROFILE_TRANSITION";
        reason = MeasuredStopped(odom_.twist.twist)
                     ? "WAITING_FOR_STATIONARY_HOLD"
                     : "WAITING_FOR_ROBOT_TO_STOP";
      }
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source, &navigation_profile_);
      return;
    }

    if (!SupportedMeasuredMotion(*profile, odom_.twist.twist,
                                 unsupported_reason)) {
      ForceStoppedScale();
      state = "UNSUPPORTED_MOTION";
      reason = unsupported_reason;
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    const CollisionResult emergency = profile->checker.simulate(
        initial, initial.linear_x, initial.linear_y, initial.angular_z,
        reaction_time_, static_map_, static_policy_, local_map_, local_policy_,
        rollout_options_);
    if (emergency.collision) {
      ForceStoppedScale();
      state = "EMERGENCY_STOP";
      reason = emergency.static_collision ? "STATIC_COLLISION_IN_STOPPING_PATH"
                                          : "LOCAL_COLLISION_IN_STOPPING_PATH";
      collision_time = emergency.collision_time;
      visualization = emergency;
      Publish(output, state, reason, scale, collision_time, visualization,
              candidate_.source);
      return;
    }

    static const double kScales[] = {1.0, 0.75, 0.50, 0.25};
    double desired_scale = 0.0;
    CollisionResult full_rollout;
    CollisionResult selected_rollout;
    for (double candidate_scale : kScales) {
      CollisionResult rollout = profile->checker.simulate(
          initial, candidate_scale * command.linear.x,
          candidate_scale * command.linear.y,
          candidate_scale * command.angular.z, preview_time_, static_map_,
          static_policy_, local_map_, local_policy_, rollout_options_);
      if (candidate_scale == 1.0) {
        full_rollout = rollout;
        if (rollout.collision) {
          collision_time = rollout.collision_time;
        }
      }
      if (!rollout.collision) {
        desired_scale = candidate_scale;
        selected_rollout = rollout;
        break;
      }
    }
    if (desired_scale <= 0.0) {
      selected_rollout = emergency;
      if (full_rollout.collision) {
        reason = full_rollout.static_collision ? "STATIC_PATH_BLOCKED"
                                               : "LOCAL_PATH_BLOCKED";
      } else {
        reason = "NO_SAFE_SPEED_LEVEL";
      }
    }

    scale = ApplyScaleHysteresis(desired_scale, now);
    output.linear.x = scale * command.linear.x;
    output.linear.y = scale * command.linear.y;
    output.angular.z = scale * command.angular.z;
    visualization = selected_rollout;
    if (scale <= 0.0) {
      state = "COLLISION_STOP";
      if (reason.empty()) {
        reason = "CLEAR_HOLD_OR_NO_SAFE_LEVEL";
      }
    } else if (scale < 1.0) {
      state = "SLOWDOWN";
      reason = scale < desired_scale ? "RECOVERY_RAMP" : "PATH_CLEARANCE";
    } else {
      state = "OK";
      reason = "FULL_COMMAND_SAFE";
    }
    Publish(output, state, reason, scale, collision_time, visualization,
            candidate_.source);
  }

  void Publish(const geometry_msgs::Twist& output, const std::string& state,
               const std::string& reason, double scale,
               double collision_time, const CollisionResult& rollout,
               const std::string& source,
               const MotionProfile* profile_override = nullptr) {
    output_pub_.publish(output);
    PublishStatus(state, reason, scale, collision_time, source, output,
                  profile_override);
    PublishMarkers(state, rollout, source, profile_override);

    if (state == "EMERGENCY_STOP" || state == "COLLISION_STOP" ||
        state == "DATA_NOT_READY" || state == "UNSUPPORTED_MOTION" ||
        state == "PROFILE_TRANSITION") {
      ROS_WARN_THROTTLE(1.0,
                        "collision_monitor: state=%s source=%s reason=%s "
                        "scale=%.2f ttc=%.3f",
                        state.c_str(), source.c_str(), reason.c_str(), scale,
                        collision_time);
    }
  }

  void PublishStatus(const std::string& state, const std::string& reason,
                     double scale, double collision_time,
                     const std::string& source,
                     const geometry_msgs::Twist& output,
                     const MotionProfile* profile_override) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "collision_monitor";
    status.hardware_id = "ranger";
    status.message = state + ": " + reason;
    if (state == "OK" || state == "STOP_COMMAND") {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else if (state == "SLOWDOWN" || state == "MANUAL_BYPASS" ||
               state == "PROFILE_TRANSITION") {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    AddDiagnosticValue(status, "state", state);
    AddDiagnosticValue(status, "reason", reason);
    AddDiagnosticValue(status, "source", source);
    const MotionProfile* profile = profile_override != nullptr
                                       ? profile_override
                                       : ProfileForSource(source);
    AddDiagnosticValue(
        status, "profile",
        state == "PROFILE_TRANSITION"
            ? std::string("transition")
            : (profile == nullptr ? std::string("none") : profile->name));
    AddDiagnosticValue(status, "footprint_padding",
                       profile == nullptr
                           ? std::string("-1")
                           : ToString(profile->footprint_padding));
    AddDiagnosticValue(status, "scale", ToString(scale));
    AddDiagnosticValue(status, "collision_time", ToString(collision_time));
    AddDiagnosticValue(status, "output_linear_x", ToString(output.linear.x));
    AddDiagnosticValue(status, "output_linear_y", ToString(output.linear.y));
    AddDiagnosticValue(status, "output_angular_z", ToString(output.angular.z));
    AddDiagnosticValue(status, "measured_linear_x",
                       have_odom_ ? ToString(odom_.twist.twist.linear.x)
                                  : std::string("-1"));
    AddDiagnosticValue(status, "measured_linear_y",
                       have_odom_ ? ToString(odom_.twist.twist.linear.y)
                                  : std::string("-1"));
    AddDiagnosticValue(status, "measured_angular_z",
                       have_odom_ ? ToString(odom_.twist.twist.angular.z)
                                  : std::string("-1"));
    array.status.push_back(status);
    status_pub_.publish(array);
  }

  void PublishMarkers(const std::string& state,
                      const CollisionResult& rollout,
                      const std::string& source,
                      const MotionProfile* profile_override) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    const MotionProfile* profile = profile_override != nullptr
                                       ? profile_override
                                       : ProfileForSource(source);
    if (rollout.poses.empty() || profile == nullptr) {
      marker_pub_.publish(array);
      return;
    }

    const ros::Time stamp = ros::Time::now();
    visualization_msgs::Marker path;
    path.header.frame_id = global_frame_;
    path.header.stamp = stamp;
    path.ns = "predicted_path";
    path.id = 0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.action = visualization_msgs::Marker::ADD;
    path.scale.x = 0.02;
    path.color.a = 0.9;
    path.color.g = state == "OK" ? 1.0 : 0.5;
    path.color.r = state == "OK" ? 0.0 : 1.0;
    for (const Pose2D& pose : rollout.poses) {
      geometry_msgs::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = 0.03;
      path.points.push_back(point);
    }
    array.markers.push_back(path);

    visualization_msgs::Marker footprints;
    footprints.header = path.header;
    footprints.ns = "predicted_footprints";
    footprints.id = 1;
    footprints.type = visualization_msgs::Marker::LINE_LIST;
    footprints.action = visualization_msgs::Marker::ADD;
    footprints.scale.x = 0.01;
    footprints.color.a = 0.45;
    footprints.color.b = 1.0;
    const std::size_t stride = std::max<std::size_t>(
        1, rollout.poses.size() / static_cast<std::size_t>(20));
    for (std::size_t i = 0; i < rollout.poses.size(); i += stride) {
      std::vector<geometry_msgs::Point> transformed;
      const Pose2D& pose = rollout.poses[i];
      costmap_2d::transformFootprint(pose.x, pose.y, pose.yaw,
                                     profile->checker.footprint(), transformed);
      for (std::size_t j = 0; j < transformed.size(); ++j) {
        geometry_msgs::Point a = transformed[j];
        geometry_msgs::Point b = transformed[(j + 1) % transformed.size()];
        a.z = 0.035;
        b.z = 0.035;
        footprints.points.push_back(a);
        footprints.points.push_back(b);
      }
    }
    array.markers.push_back(footprints);

    if (rollout.collision) {
      visualization_msgs::Marker collision;
      collision.header = path.header;
      collision.ns = "collision";
      collision.id = 2;
      collision.type = visualization_msgs::Marker::SPHERE;
      collision.action = visualization_msgs::Marker::ADD;
      collision.pose.position.x = rollout.collision_pose.x;
      collision.pose.position.y = rollout.collision_pose.y;
      collision.pose.position.z = 0.10;
      collision.pose.orientation.w = 1.0;
      collision.scale.x = 0.18;
      collision.scale.y = 0.18;
      collision.scale.z = 0.18;
      collision.color.r = 1.0;
      collision.color.a = 0.9;
      array.markers.push_back(collision);
    }
    marker_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  MotionProfile navigation_profile_;
  MotionProfile tag_profile_;
  MotionProfile teleop_profile_;
  GridPolicy static_policy_;
  GridPolicy local_policy_;
  RolloutOptions rollout_options_;

  ros::Subscriber candidate_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber static_map_sub_;
  ros::Subscriber local_map_sub_;
  ros::Publisher output_pub_;
  ros::Publisher status_pub_;
  ros::Publisher marker_pub_;
  ros::WallTimer timer_;

  cmd_vel_arbiter::ArbitratedCommand candidate_;
  nav_msgs::Odometry odom_;
  nav_msgs::OccupancyGrid static_map_;
  nav_msgs::OccupancyGrid local_map_;
  ros::WallTime candidate_received_at_;
  ros::WallTime odom_received_at_;
  ros::WallTime local_map_received_at_;
  bool have_candidate_ = false;
  bool have_odom_ = false;
  bool have_static_map_ = false;
  bool have_local_map_ = false;

  Pose2D last_pose_;
  ros::Time last_tf_stamp_;
  ros::WallTime pose_jump_until_;
  bool have_last_pose_ = false;
  double applied_scale_ = 0.0;
  ros::WallTime scale_increase_since_;
  std::string active_source_;
  std::string pending_source_;
  ros::WallTime transition_stopped_since_;

  std::string candidate_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string static_map_topic_;
  std::string local_map_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string global_frame_;
  std::string base_frame_;
  std::string teleop_source_;
  std::string tag_source_;
  std::string navigation_source_;
  double monitor_rate_ = 50.0;
  double candidate_timeout_ = 0.25;
  double odom_timeout_ = 0.20;
  double tf_timeout_ = 0.30;
  double local_map_timeout_ = 0.30;
  double zero_epsilon_ = 1e-4;
  double reaction_time_ = 0.30;
  double preview_time_ = 2.0;
  double clear_hold_time_ = 0.50;
  double source_transition_hold_time_ = 0.20;
  double transition_stopped_linear_ = 0.01;
  double transition_stopped_angular_ = 0.02;
  double max_pose_jump_distance_ = 0.50;
  double max_pose_jump_yaw_ = 0.80;
  double pose_jump_hold_time_ = 0.50;
};

}  // namespace collision_monitor

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_monitor");
  try {
    collision_monitor::CollisionMonitorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("collision_monitor failed to start: %s", exception.what());
    return 1;
  }
  return 0;
}
