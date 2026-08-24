/**
* @file ranger_messenger.hpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ranger_base/ranger_params.hpp"
#include "ranger_msgs/action/stop_and_center.hpp"
#include "ranger_msgs/msg/actuator_state_array.hpp"
#include "ranger_msgs/msg/motion_state.hpp"
#include "ranger_msgs/msg/ranger_light_cmd.hpp"
#include "ranger_msgs/msg/rs_status.hpp"
#include "ranger_msgs/msg/system_state.hpp"
#include "ranger_msgs/srv/trigger_park_mode.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

namespace westonrobot {
class RangerROSMessenger {
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_central;
    double max_steer_angle_parallel;
    double max_steer_angle_ackermann;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType {
    kRanger = 0,
    kRangerMiniV1,
    kRangerMiniV2,
    kRangerMiniV3
  };

  enum class StopCenterState : uint8_t {
    kIdle = 0,
    kStopping = 1,
    kWaitStill = 2,
    kSwitchMode = 3,
    kWaitMode = 4,
    kCentering = 5,
    kDone = 6,
    kFailed = 7
  };

  using StopAndCenter = ranger_msgs::action::StopAndCenter;
  using StopAndCenterGoalHandle =
      rclcpp_action::ServerGoalHandle<StopAndCenter>;
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

 public:
  explicit RangerROSMessenger(rclcpp::Node::SharedPtr& node);
  ~RangerROSMessenger();

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  void PublishStateToROS(const RangerCoreState& state,
                         const RangerActuatorState& actuator_state);
  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void CheckCmdVelWatchdog();
  void SendZeroMotionCommand();
  rclcpp_action::GoalResponse HandleStopCenterGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const StopAndCenter::Goal> goal);
  rclcpp_action::CancelResponse HandleStopCenterCancel(
      const std::shared_ptr<StopAndCenterGoalHandle> goal_handle);
  void HandleStopCenterAccepted(
      const std::shared_ptr<StopAndCenterGoalHandle> goal_handle);
  void RequestStopAndCenter(uint8_t reason, bool report_action_result);
  void UpdateStopAndCenter(const RangerCoreState& state,
                           const RangerActuatorState& actuator_state);
  void CheckChassisFault(const RangerCoreState& state);
  void CompleteStopAndCenter(bool success, const std::string& message);
  void PublishDiagnostic(uint8_t level, const std::string& code,
                         const std::string& message,
                         const std::string& detail,
                         const std::string& action, bool active);
  bool StopCenterActive() const;
  bool VehicleIsStill(const RangerCoreState& state,
                      const RangerActuatorState& actuator_state) const;
  double MaxWheelSpeed(const RangerActuatorState& actuator_state) const;
  double MaxSteeringError(const RangerActuatorState& actuator_state) const;
  bool StateTimedOut(double timeout) const;
  void PublishStopCenterFeedback(const RangerActuatorState& actuator_state);
  void BestEffortStopAndCenterOnShutdown();
  bool IsFiniteTwist(const geometry_msgs::msg::Twist& msg) const;
  bool IsZeroTwist(const geometry_msgs::msg::Twist& msg) const;
  void LightCmdCallback(
      const ranger_msgs::msg::RangerLightCmd::SharedPtr msg);
  void TriggerParkingService(
      const std::shared_ptr<ranger_msgs::srv::TriggerParkMode::Request> request,
      std::shared_ptr<ranger_msgs::srv::TriggerParkMode::Response> response);
  double CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                double& radius);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(double yaw);
  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;
  RobotParams robot_params_;
  bool connected_ = false;

  std::string robot_model_;
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_;
  double cmd_vel_timeout_ = 0.25;
  double stop_velocity_threshold_ = 0.01;
  double stop_wheel_speed_threshold_ = 0.01;
  double steering_center_tolerance_ = 0.01;
  double stop_wait_timeout_ = 2.0;
  double mode_switch_timeout_ = 2.0;
  double centering_timeout_ = 3.0;
  int stop_stable_frames_ = 3;
  int center_stable_frames_ = 3;
  SteadyTimePoint last_cmd_vel_time_;
  bool cmd_vel_received_ = false;
  bool watchdog_active_ = true;

  uint8_t motion_mode_ = 0;
  bool parking_mode_ = false;

  StopCenterState stop_center_state_ = StopCenterState::kIdle;
  uint8_t stop_center_reason_ = StopAndCenter::Goal::TASK_FINISHED;
  SteadyTimePoint stop_center_state_started_at_;
  SteadyTimePoint last_mode_request_time_;
  int still_feedback_count_ = 0;
  int centered_feedback_count_ = 0;
  bool report_stop_center_action_result_ = false;
  uint16_t last_error_code_ = 0;
  bool fault_motion_lock_ = false;
  bool fault_recenter_pending_ = false;

  rclcpp::Publisher<ranger_msgs::msg::SystemState>::SharedPtr
      system_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::MotionState>::SharedPtr
      motion_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::ActuatorStateArray>::SharedPtr
      actuator_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::RsStatus>::SharedPtr rs_state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<ranger_msgs::msg::RangerLightCmd>::SharedPtr
      light_cmd_subscriber_;
  rclcpp::Service<ranger_msgs::srv::TriggerParkMode>::SharedPtr
      trigger_parking_server_;
  rclcpp_action::Server<StopAndCenter>::SharedPtr stop_center_server_;
  std::shared_ptr<StopAndCenterGoalHandle> stop_center_goal_handle_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
