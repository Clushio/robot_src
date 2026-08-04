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

#include <string>

#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseArray.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ranger_msgs/SystemState.h>
#include <ranger_msgs/MotionState.h>
#include <ranger_msgs/ActuatorStateArray.h>
#include <ranger_msgs/RangerLightCmd.h>
#include <ranger_msgs/StopAndCenterAction.h>
#include <ranger_msgs/TriggerParkMode.h>
#include <ranger_msgs/BatteryState.h>

#include "ranger_base/ranger_params.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

using namespace ros::master;

namespace westonrobot {
class RangerROSMessenger {
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_ackermann;
    double max_steer_angle_parallel;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType { kRanger = 0, kRangerMiniV1, kRangerMiniV2 };

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

 public:
  RangerROSMessenger(ros::NodeHandle* nh);
  ~RangerROSMessenger();

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  void PublishStateToROS(const RangerCoreState& state,
                         const RangerActuatorState& actuator_state);
  void PublishSimStateToROS(double linear, double angular);
  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void CheckCmdVelWatchdog();
  void SendZeroMotionCommand();
  void StopCenterGoalCallback();
  void StopCenterPreemptCallback();
  void RequestStopAndCenter(uint8_t reason, bool report_action_result);
  void UpdateStopAndCenter(const RangerCoreState& state,
                           const RangerActuatorState& actuator_state);
  void CheckChassisFault(const RangerCoreState& state);
  void CompleteStopAndCenter(bool success, const std::string& message);
  bool StopCenterActive() const;
  bool VehicleIsStill(const RangerCoreState& state,
                      const RangerActuatorState& actuator_state) const;
  double MaxWheelSpeed(const RangerActuatorState& actuator_state) const;
  double MaxSteeringError(const RangerActuatorState& actuator_state) const;
  bool StateTimedOut(double timeout) const;
  void PublishStopCenterFeedback(const RangerActuatorState& actuator_state);
  void BestEffortStopAndCenterOnShutdown();
  bool IsFiniteTwist(const geometry_msgs::Twist& msg) const;
  bool IsZeroTwist(const geometry_msgs::Twist& msg) const;
  void LightCmdCallback(const ranger_msgs::RangerLightCmd::ConstPtr &msg);
  double CalculateSteeringAngle(geometry_msgs::Twist msg, double& radius);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);
  bool TriggerParkingService(ranger_msgs::TriggerParkMode::Request &req, ranger_msgs::TriggerParkMode::Response &res); 

  ros::NodeHandle* nh_;
  std::shared_ptr<RangerRobot> robot_;
  actionlib::SimpleActionServer<ranger_msgs::StopAndCenterAction>
      stop_center_server_;
  RangerSubType robot_type_;
  RobotParams robot_params_;

  // constants
  // parameters
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
  ros::WallTime last_cmd_vel_time_;
  bool cmd_vel_received_ = false;
  bool watchdog_active_ = true;

  uint8_t motion_mode_ = 0;
  bool parking_mode_;

  StopCenterState stop_center_state_ = StopCenterState::kIdle;
  uint8_t stop_center_reason_ = ranger_msgs::StopAndCenterGoal::TASK_FINISHED;
  ros::WallTime stop_center_state_started_at_;
  int still_feedback_count_ = 0;
  int centered_feedback_count_ = 0;
  bool report_stop_center_action_result_ = false;
  uint16_t last_error_code_ = 0;
  bool fault_motion_lock_ = false;
  bool fault_recenter_pending_ = false;

  ros::Publisher system_state_pub_;
  ros::Publisher motion_state_pub_;
  ros::Publisher actuator_state_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher battery_state_pub_;
  ros::Publisher rs_state_pub_;

  ros::Subscriber motion_cmd_sub_;
  ros::Subscriber light_cmd_subscriber_;

  ros::ServiceServer trigger_parking_server;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // odom variables
  ros::Time last_time_;
  ros::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
