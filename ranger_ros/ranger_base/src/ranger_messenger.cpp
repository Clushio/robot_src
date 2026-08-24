/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "ranger_base/kinematics_model.hpp"

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
namespace {
void AddDiagnosticValue(diagnostic_msgs::msg::DiagnosticStatus& status,
                        const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}
}  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node)
    : node_(node) {
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect to the CAN port");
      rclcpp::shutdown();
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid port name: %s",
                 port_name_.c_str());
    rclcpp::shutdown();
    return;
  }

  connected_ = true;
  SetupSubscription();
}

RangerROSMessenger::~RangerROSMessenger() { SendZeroMotionCommand(); }

void RangerROSMessenger::Run() {
  if (!connected_) {
    return;
  }

  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    CheckCmdVelWatchdog();
    const auto state = robot_->GetRobotState();
    const auto actuator_state = robot_->GetActuatorState();
    CheckChassisFault(state);
    UpdateStopAndCenter(state, actuator_state);
    PublishStateToROS(state, actuator_state);
    rate.sleep();
  }

  BestEffortStopAndCenterOnShutdown();
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  cmd_vel_timeout_ =
      node_->declare_parameter<double>("cmd_vel_timeout", cmd_vel_timeout_);
  stop_velocity_threshold_ = node_->declare_parameter<double>(
      "stop_velocity_threshold", stop_velocity_threshold_);
  stop_wheel_speed_threshold_ = node_->declare_parameter<double>(
      "stop_wheel_speed_threshold", stop_wheel_speed_threshold_);
  steering_center_tolerance_ = node_->declare_parameter<double>(
      "steering_center_tolerance", steering_center_tolerance_);
  stop_wait_timeout_ = node_->declare_parameter<double>(
      "stop_wait_timeout", stop_wait_timeout_);
  mode_switch_timeout_ = node_->declare_parameter<double>(
      "mode_switch_timeout", mode_switch_timeout_);
  centering_timeout_ = node_->declare_parameter<double>(
      "centering_timeout", centering_timeout_);
  stop_stable_frames_ = node_->declare_parameter<int>(
      "stop_stable_frames", stop_stable_frames_);
  center_stable_frames_ = node_->declare_parameter<int>(
      "center_stable_frames", center_stable_frames_);

  cmd_vel_timeout_ = std::max(0.02, cmd_vel_timeout_);
  stop_velocity_threshold_ = std::max(0.0, stop_velocity_threshold_);
  stop_wheel_speed_threshold_ = std::max(0.0, stop_wheel_speed_threshold_);
  steering_center_tolerance_ = std::max(0.001, steering_center_tolerance_);
  stop_wait_timeout_ = std::max(0.1, stop_wait_timeout_);
  mode_switch_timeout_ = std::max(0.1, mode_switch_timeout_);
  centering_timeout_ = std::max(0.1, centering_timeout_);
  stop_stable_frames_ = std::max(1, stop_stable_frames_);
  center_stable_frames_ = std::max(1, center_stable_frames_);

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n cmd_vel_timeout: %.3f s\n"
      " stop_velocity_threshold: %.3f\n stop_wheel_speed_threshold: %.3f\n"
      " steering_center_tolerance: %.4f\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_, cmd_vel_timeout_, stop_velocity_threshold_,
      stop_wheel_speed_threshold_, steering_center_tolerance_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV1Params::max_steer_angle_ackermann;
  } else if (robot_model_ == "ranger_mini_v2") {
    robot_type_ = RangerSubType::kRangerMiniV2;

    robot_params_.track = RangerMiniV2Params::track;
    robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV2Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV2Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann =
        RangerMiniV2Params::max_steer_angle_ackermann;
  } else if (robot_model_ == "ranger_mini_v3") {
    robot_type_ = RangerSubType::kRangerMiniV3;

    robot_params_.track = RangerMiniV3Params::track;
    robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV3Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV3Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann =
        RangerMiniV3Params::max_steer_angle_ackermann;
  } else {
    robot_type_ = RangerSubType::kRanger;

    robot_params_.track = RangerParams::track;
    robot_params_.wheelbase = RangerParams::wheelbase;
    robot_params_.max_linear_speed = RangerParams::max_linear_speed;
    robot_params_.max_angular_speed = RangerParams::max_angular_speed;
    robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerParams::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerParams::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerParams::max_round_angle;
    robot_params_.min_turn_radius = RangerParams::min_turn_radius;
    robot_params_.max_steer_angle_ackermann =
        RangerParams::max_steer_angle_ackermann;
  }
  parking_mode_ = false;

}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>("/system_state", 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  rs_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::RsStatus>("/rs_state", 10);
  diagnostics_pub_ =
      node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "/diagnostics", rclcpp::QoS(10).transient_local());

  // subscriber
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1)
      );
  light_cmd_subscriber_ =
      node_->create_subscription<ranger_msgs::msg::RangerLightCmd>(
          "/ranger_light_control", 5,
          std::bind(&RangerROSMessenger::LightCmdCallback, this,
                    std::placeholders::_1));

  trigger_parking_server_ =
      node_->create_service<ranger_msgs::srv::TriggerParkMode>(
          "/parking_service",
          std::bind(&RangerROSMessenger::TriggerParkingService, this,
                    std::placeholders::_1, std::placeholders::_2));

  stop_center_server_ = rclcpp_action::create_server<StopAndCenter>(
      node_, "/stop_and_center",
      std::bind(&RangerROSMessenger::HandleStopCenterGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&RangerROSMessenger::HandleStopCenterCancel, this,
                std::placeholders::_1),
      std::bind(&RangerROSMessenger::HandleStopCenterAccepted, this,
                std::placeholders::_1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::PublishStateToROS(
    const RangerCoreState& state,
    const RangerActuatorState& actuator_state) {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  // update odometry
  {
    double dt = (current_time_ - last_time_).seconds();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.angular_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // publish remote-control state
  {
    ranger_msgs::msg::RsStatus rs_msg;
    rs_msg.header.stamp = current_time_;
    rs_msg.stick_left_h = state.rc_state.stick_left_h;
    rs_msg.stick_left_v = state.rc_state.stick_left_v;
    rs_msg.stick_right_h = state.rc_state.stick_right_h;
    rs_msg.stick_right_v = state.rc_state.stick_right_v;
    rs_msg.swa = state.rc_state.swa;
    rs_msg.swb = state.rc_state.swb;
    rs_msg.swc = state.rc_state.swc;
    rs_msg.swd = state.rc_state.swd;
    rs_msg.var_a = state.rc_state.var_a;
    rs_state_pub_->publish(rs_msg);
  }

  // publish motion mode
  {
    motion_mode_ = state.motion_mode_state.motion_mode;

    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // publish actuator state
  {
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
    //                 actuator_state.motor_angles.angle_5,
    //                 actuator_state.motor_angles.angle_6,
    //                 actuator_state.motor_angles.angle_7,
    //                 actuator_state.motor_angles.angle_8);
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
    //                 actuator_state.motor_speeds.speed_1,
    //                 actuator_state.motor_speeds.speed_2,
    //                 actuator_state.motor_speeds.speed_3,
    //                 actuator_state.motor_speeds.speed_4);

    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;

    // Actuator IDs 0-3 are drive motors and IDs 4-7 are steering motors.
    const float motor_speeds[8] = {
        actuator_state.motor_speeds.speed_1,
        actuator_state.motor_speeds.speed_2,
        actuator_state.motor_speeds.speed_3,
        actuator_state.motor_speeds.speed_4,
        0.0F,
        0.0F,
        0.0F,
        0.0F};
    const float motor_angles[8] = {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        actuator_state.motor_angles.angle_5,
        actuator_state.motor_angles.angle_6,
        actuator_state.motor_angles.angle_7,
        actuator_state.motor_angles.angle_8};

    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state[i].driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state[i].driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state[i].motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state[i].driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state[i].current;
      motor_state_msg.pulse_count =
          actuator_state.actuator_hs_state[i].pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state[i].rpm;
      motor_state_msg.motor_angles = motor_angles[i];
      motor_state_msg.motor_speeds = motor_speeds[i];

      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    const uint8_t battery_soc =
        common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.percentage =
        battery_soc <= 100
            ? static_cast<float>(battery_soc) / 100.0F
            : std::numeric_limits<float>::quiet_NaN();
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = true;

    battery_state_pub_->publish(batt_msg);
  }
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double angle, double dt) {
  // update odometry calculations
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = ConvertInnerAngleToCentral(angle);

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(robot_params_.wheelbase, u), x, 0.0, dt, (dt / 10.0));
    //std::cout<<" steer: "<<angle<<" central: "<<u.phi<<std::endl;
    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      u.phi = M_PI / 2.0;
    } else {
      u.phi = angle;
    }
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }

  // update odometry topics
  geometry_msgs::msg::Quaternion odom_quat =
      CreateQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z =
        2 * linear * std::sin(ConvertInnerAngleToCentral(angle)) /
        robot_params_.wheelbase;
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    double phi = angle;

    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      phi = M_PI / 2.0;
    }
    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);

    odom_msg.twist.twist.angular.z = 0;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = angular;
  }

  odom_pub_->publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  double steer_cmd;
  double radius;

  if (!IsFiniteTwist(*msg)) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Rejected non-finite /cmd_vel; commanding zero speed");
    cmd_vel_received_ = false;
    watchdog_active_ = true;
    RequestStopAndCenter(StopAndCenter::Goal::CMD_TIMEOUT, false);
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                      "ANAV-BASE-007", "底盘收到非法速度指令",
                      "/cmd_vel 中包含 NaN 或无穷值。",
                      "检查速度仲裁输出和上游速度发布节点。", true);
    return;
  }

  const bool watchdog_recovered = watchdog_active_ && cmd_vel_received_;
  last_cmd_vel_time_ = std::chrono::steady_clock::now();
  cmd_vel_received_ = true;
  watchdog_active_ = false;
  if (watchdog_recovered && last_error_code_ == 0) {
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                      "ANAV-BASE-000", "底盘速度指令已恢复", "", "",
                      false);
  }

  if (StopCenterActive() || fault_motion_lock_) {
    SendZeroMotionCommand();
    return;
  }

  if (IsZeroTwist(*msg)) {
    SendZeroMotionCommand();
    return;
  }

  // analyze Twist msg and switch motion_mode
  // check for parking mode, only applicable to RangerMiniV2
  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  } else if (msg->linear.y != 0) {
    if (msg->linear.x == 0.0 && robot_type_ == RangerSubType::kRangerMiniV1) {
      motion_mode_ = MotionState::MOTION_MODE_SIDE_SLIP;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SIDE_SLIP);
    } else {
      motion_mode_ = MotionState::MOTION_MODE_PARALLEL;
      robot_->SetMotionMode(MotionState::MOTION_MODE_PARALLEL);
    }
  } else {
    steer_cmd = CalculateSteeringAngle(*msg, radius);
    // Use minimum turn radius to switch between dual ackerman and spinning mode
    if (radius < robot_params_.min_turn_radius) {
      motion_mode_ = MotionState::MOTION_MODE_SPINNING;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SPINNING);
    } else {
      motion_mode_ = MotionState::MOTION_MODE_DUAL_ACKERMAN;
      robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
    }
  }
  // send motion command to robot
  switch (motion_mode_) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      robot_->SetMotionCommand(msg->linear.x, steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_PARALLEL: {
      steer_cmd = atan(msg->linear.y / msg->linear.x);

      static double last_nonzero_x = 1.0; 
      
      if (msg->linear.x != 0.0) {
          last_nonzero_x = msg->linear.x; 
      }

      if (std::signbit(msg->linear.x))
      {
        steer_cmd = -steer_cmd;
      }
      
      if (steer_cmd > robot_params_.max_steer_angle_parallel) {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel) {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      double vel = 1.0;
      
      if (msg->linear.x == 0.0 && msg->linear.y != 0.0) {
          // std::cout << "MOTION_MODE_SIDE_SLIP" << std::endl;
          
          if (std::signbit(last_nonzero_x)) {
              steer_cmd = -std::abs(steer_cmd); 
          } else {
              steer_cmd = std::abs(steer_cmd);
          }
          vel = msg->linear.y >= 0 ? 1.0 : -1.0;
      } else {
          vel = msg->linear.x >= 0 ? 1.0 : -1.0;
      }
      robot_->SetMotionCommand(vel * sqrt(msg->linear.x * msg->linear.x +
                                          msg->linear.y * msg->linear.y),
                               steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_SPINNING: {
      double a_v = msg->angular.z;
      if (a_v > robot_params_.max_angular_speed) {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed) {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      break;
    }
    case MotionState::MOTION_MODE_SIDE_SLIP: {
      double l_v = msg->linear.y;
      if (l_v > robot_params_.max_linear_speed) {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed) {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      break;
    }
  }
}

void RangerROSMessenger::CheckCmdVelWatchdog() {
  const auto now = std::chrono::steady_clock::now();
  const bool timed_out =
      !cmd_vel_received_ ||
      std::chrono::duration<double>(now - last_cmd_vel_time_).count() >
          cmd_vel_timeout_;
  if (!timed_out) {
    return;
  }

  if (StopCenterActive()) {
    SendZeroMotionCommand();
    watchdog_active_ = true;
    return;
  }

  if (!watchdog_active_) {
    RCLCPP_WARN(node_->get_logger(),
                "/cmd_vel timeout after %.3f s; commanding zero speed",
                cmd_vel_timeout_);
    RequestStopAndCenter(StopAndCenter::Goal::CMD_TIMEOUT, false);
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                      "ANAV-BASE-008", "底盘速度指令超时，已停车回正",
                      "/cmd_vel 超过设定时限未更新。",
                      "检查速度仲裁节点和 /cmd_vel 话题。", true);
  } else {
    SendZeroMotionCommand();
  }
  watchdog_active_ = true;
}

void RangerROSMessenger::SendZeroMotionCommand() {
  if (robot_ && connected_) {
    robot_->SetMotionCommand(0.0, 0.0, 0.0);
  }
}

rclcpp_action::GoalResponse RangerROSMessenger::HandleStopCenterGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const StopAndCenter::Goal> goal) {
  (void)uuid;
  if (goal->reason > StopAndCenter::Goal::CHASSIS_FAULT) {
    RCLCPP_WARN(node_->get_logger(),
                "Rejecting stop-and-center goal with invalid reason %u",
                static_cast<unsigned int>(goal->reason));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (stop_center_goal_handle_) {
    RCLCPP_WARN(node_->get_logger(),
                "Rejecting stop-and-center goal because one is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RangerROSMessenger::HandleStopCenterCancel(
    const std::shared_ptr<StopAndCenterGoalHandle> goal_handle) {
  if (goal_handle != stop_center_goal_handle_) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RangerROSMessenger::HandleStopCenterAccepted(
    const std::shared_ptr<StopAndCenterGoalHandle> goal_handle) {
  stop_center_goal_handle_ = goal_handle;
  RequestStopAndCenter(goal_handle->get_goal()->reason, true);
}

void RangerROSMessenger::RequestStopAndCenter(
    uint8_t reason, bool report_action_result) {
  stop_center_reason_ = reason;
  report_stop_center_action_result_ =
      report_stop_center_action_result_ || report_action_result;
  still_feedback_count_ = 0;
  centered_feedback_count_ = 0;
  stop_center_state_started_at_ = std::chrono::steady_clock::now();
  stop_center_state_ = StopCenterState::kStopping;
  SendZeroMotionCommand();
  RCLCPP_WARN(node_->get_logger(),
              "Stop-and-center requested (reason=%u)",
              static_cast<unsigned int>(reason));
}

bool RangerROSMessenger::StopCenterActive() const {
  return stop_center_state_ != StopCenterState::kIdle;
}

double RangerROSMessenger::MaxWheelSpeed(
    const RangerActuatorState& actuator_state) const {
  return std::max(
      std::max(std::abs(actuator_state.motor_speeds.speed_1),
               std::abs(actuator_state.motor_speeds.speed_2)),
      std::max(std::abs(actuator_state.motor_speeds.speed_3),
               std::abs(actuator_state.motor_speeds.speed_4)));
}

double RangerROSMessenger::MaxSteeringError(
    const RangerActuatorState& actuator_state) const {
  return std::max(
      std::max(std::abs(actuator_state.motor_angles.angle_5),
               std::abs(actuator_state.motor_angles.angle_6)),
      std::max(std::abs(actuator_state.motor_angles.angle_7),
               std::abs(actuator_state.motor_angles.angle_8)));
}

bool RangerROSMessenger::VehicleIsStill(
    const RangerCoreState& state,
    const RangerActuatorState& actuator_state) const {
  return std::abs(state.motion_state.linear_velocity) <=
             stop_velocity_threshold_ &&
         std::abs(state.motion_state.lateral_velocity) <=
             stop_velocity_threshold_ &&
         std::abs(state.motion_state.angular_velocity) <=
             stop_velocity_threshold_ &&
         MaxWheelSpeed(actuator_state) <= stop_wheel_speed_threshold_;
}

bool RangerROSMessenger::StateTimedOut(double timeout) const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      stop_center_state_started_at_)
             .count() >= timeout;
}

void RangerROSMessenger::PublishStopCenterFeedback(
    const RangerActuatorState& actuator_state) {
  if (!report_stop_center_action_result_ || !stop_center_goal_handle_) {
    return;
  }

  auto feedback = std::make_shared<StopAndCenter::Feedback>();
  feedback->state = static_cast<uint8_t>(stop_center_state_);
  feedback->max_wheel_speed = MaxWheelSpeed(actuator_state);
  feedback->max_steering_error = MaxSteeringError(actuator_state);
  stop_center_goal_handle_->publish_feedback(feedback);
}

void RangerROSMessenger::CompleteStopAndCenter(
    bool success, const std::string& message) {
  if (success &&
      stop_center_reason_ == StopAndCenter::Goal::CHASSIS_FAULT &&
      last_error_code_ == 0) {
    fault_motion_lock_ = false;
    fault_recenter_pending_ = false;
  }

  if (report_stop_center_action_result_ && stop_center_goal_handle_) {
    auto result = std::make_shared<StopAndCenter::Result>();
    result->success = success;
    result->message = message;
    if (success) {
      stop_center_goal_handle_->succeed(result);
    } else {
      stop_center_goal_handle_->abort(result);
    }
  }

  stop_center_goal_handle_.reset();
  report_stop_center_action_result_ = false;
  stop_center_state_ = StopCenterState::kIdle;
  still_feedback_count_ = 0;
  centered_feedback_count_ = 0;

  if (!success) {
    std::string code = "ANAV-BASE-011";
    if (message.find("stationary") != std::string::npos) {
      code = "ANAV-BASE-009";
    } else if (message.find("mode switch") != std::string::npos) {
      code = "ANAV-BASE-010";
    }
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, code,
                      "底盘停车回正失败", message,
                      "检查底盘反馈、运动模式与转向执行机构。", true);
  } else if (last_error_code_ == 0) {
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                      "ANAV-BASE-000", "底盘已停车且轮组回正", message,
                      "", false);
  }
}

void RangerROSMessenger::UpdateStopAndCenter(
    const RangerCoreState& state,
    const RangerActuatorState& actuator_state) {
  if (!StopCenterActive()) {
    return;
  }

  if (stop_center_goal_handle_ && stop_center_goal_handle_->is_canceling()) {
    auto result = std::make_shared<StopAndCenter::Result>();
    result->success = false;
    result->message =
        "stop-and-center action canceled; safe stop continues";
    stop_center_goal_handle_->canceled(result);
    stop_center_goal_handle_.reset();
    report_stop_center_action_result_ = false;
  }

  PublishStopCenterFeedback(actuator_state);

  switch (stop_center_state_) {
    case StopCenterState::kIdle:
      return;
    case StopCenterState::kStopping:
      SendZeroMotionCommand();
      still_feedback_count_ = 0;
      stop_center_state_started_at_ = std::chrono::steady_clock::now();
      stop_center_state_ = StopCenterState::kWaitStill;
      break;
    case StopCenterState::kWaitStill:
      SendZeroMotionCommand();
      if (VehicleIsStill(state, actuator_state)) {
        ++still_feedback_count_;
      } else {
        still_feedback_count_ = 0;
      }
      if (still_feedback_count_ >= stop_stable_frames_) {
        stop_center_state_ = StopCenterState::kSwitchMode;
      } else if (StateTimedOut(stop_wait_timeout_)) {
        CompleteStopAndCenter(false,
                              "vehicle did not become stationary in time");
      }
      break;
    case StopCenterState::kSwitchMode:
      SendZeroMotionCommand();
      robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
      stop_center_state_started_at_ = std::chrono::steady_clock::now();
      last_mode_request_time_ = stop_center_state_started_at_;
      stop_center_state_ = StopCenterState::kWaitMode;
      break;
    case StopCenterState::kWaitMode: {
      SendZeroMotionCommand();
      if (state.motion_mode_state.motion_mode ==
              MotionState::MOTION_MODE_DUAL_ACKERMAN &&
          state.motion_mode_state.mode_changing == 0) {
        robot_->SetMotionCommand(0.0, 0.0, 0.0);
        centered_feedback_count_ = 0;
        stop_center_state_started_at_ = std::chrono::steady_clock::now();
        stop_center_state_ = StopCenterState::kCentering;
      } else {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_mode_request_time_)
                .count() >= 0.1) {
          robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
          last_mode_request_time_ = now;
        }
        if (StateTimedOut(mode_switch_timeout_)) {
          CompleteStopAndCenter(false,
                                "dual-Ackermann mode switch timed out");
        }
      }
      break;
    }
    case StopCenterState::kCentering:
      robot_->SetMotionCommand(0.0, 0.0, 0.0);
      if (MaxSteeringError(actuator_state) <= steering_center_tolerance_) {
        ++centered_feedback_count_;
      } else {
        centered_feedback_count_ = 0;
      }
      if (centered_feedback_count_ >= center_stable_frames_) {
        stop_center_state_ = StopCenterState::kDone;
      } else if (StateTimedOut(centering_timeout_)) {
        CompleteStopAndCenter(false, "steering centering timed out");
      }
      break;
    case StopCenterState::kDone:
      SendZeroMotionCommand();
      RCLCPP_INFO(node_->get_logger(), "Stop-and-center completed");
      CompleteStopAndCenter(true,
                            "vehicle stopped and steering centered");
      break;
    case StopCenterState::kFailed:
      SendZeroMotionCommand();
      CompleteStopAndCenter(false, "stop-and-center failed");
      break;
  }
}

void RangerROSMessenger::CheckChassisFault(const RangerCoreState& state) {
  const uint16_t error_code = state.system_state.error_code;
  const uint16_t previous_error = last_error_code_;
  last_error_code_ = error_code;

  if (error_code != 0 && previous_error == 0) {
    fault_motion_lock_ = true;
    fault_recenter_pending_ = true;
    RCLCPP_ERROR(node_->get_logger(),
                 "Ranger chassis fault detected: error_code=0x%04x",
                 static_cast<unsigned int>(error_code));
    RequestStopAndCenter(StopAndCenter::Goal::CHASSIS_FAULT, false);
    std::ostringstream detail;
    detail << "底盘 error_code=0x" << std::hex << error_code;
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                      "ANAV-BASE-005", "底盘控制器报告故障", detail.str(),
                      "查阅 Ranger 底盘错误码并排除硬件故障。", true);
    return;
  }

  if (error_code == 0 && previous_error != 0 && fault_recenter_pending_) {
    RCLCPP_WARN(node_->get_logger(),
                "Ranger chassis fault cleared; center steering before "
                "unlocking");
    RequestStopAndCenter(StopAndCenter::Goal::CHASSIS_FAULT, false);
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                      "ANAV-BASE-015", "底盘故障已清除，正在停车回正",
                      "解除运动锁前需确认车辆静止且轮组回正。",
                      "等待自动回正完成。", true);
  }
}

void RangerROSMessenger::PublishDiagnostic(
    uint8_t level, const std::string& code, const std::string& message,
    const std::string& detail, const std::string& action, bool active) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "/anav/ranger_base";
  status.hardware_id = robot_model_;
  status.message = message;
  AddDiagnosticValue(status, "code", code);
  AddDiagnosticValue(status, "active", active ? "true" : "false");
  AddDiagnosticValue(status, "kind", active ? "FAULT" : "STATE");
  AddDiagnosticValue(status, "detail", detail);
  AddDiagnosticValue(status, "action", action);
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

void RangerROSMessenger::BestEffortStopAndCenterOnShutdown() {
  if (!robot_ || !connected_) {
    return;
  }

  for (int i = 0; i < 3; ++i) {
    SendZeroMotionCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
  for (int i = 0; i < 10; ++i) {
    robot_->SetMotionCommand(0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool RangerROSMessenger::IsFiniteTwist(
    const geometry_msgs::msg::Twist& msg) const {
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

bool RangerROSMessenger::IsZeroTwist(
    const geometry_msgs::msg::Twist& msg) const {
  constexpr double kEpsilon = 1e-9;
  return std::abs(msg.linear.x) <= kEpsilon &&
         std::abs(msg.linear.y) <= kEpsilon &&
         std::abs(msg.linear.z) <= kEpsilon &&
         std::abs(msg.angular.x) <= kEpsilon &&
         std::abs(msg.angular.y) <= kEpsilon &&
         std::abs(msg.angular.z) <= kEpsilon;
}

void RangerROSMessenger::LightCmdCallback(
    const ranger_msgs::msg::RangerLightCmd::SharedPtr msg) {
  if (!msg->enable_cmd_light_control) {
    robot_->DisableLightControl();
    return;
  }

  AgxLightMode mode;
  switch (msg->front_mode) {
    case ranger_msgs::msg::RangerLightCmd::LIGHT_CONST_OFF:
      mode = CONST_OFF;
      break;
    case ranger_msgs::msg::RangerLightCmd::LIGHT_CONST_ON:
      mode = CONST_ON;
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Ignoring invalid light mode %u",
                  static_cast<unsigned int>(msg->front_mode));
      return;
  }
  robot_->SetLightCommand(mode, 0, mode, 0);
}

void RangerROSMessenger::TriggerParkingService(
    const std::shared_ptr<ranger_msgs::srv::TriggerParkMode::Request> request,
    std::shared_ptr<ranger_msgs::srv::TriggerParkMode::Response> response) {
  if (robot_type_ != RangerSubType::kRangerMiniV2) {
    RCLCPP_WARN(node_->get_logger(),
                "Parking mode is only supported by Ranger Mini V2");
    response->is_parked = false;
    return;
  }

  SendZeroMotionCommand();
  if (request->trigger_parked_mode) {
    robot_->SetMotionMode(MotionState::MOTION_MODE_PARKING);
    response->is_parked = true;
  } else {
    robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
    SendZeroMotionCommand();
    response->is_parked = false;
  }
  parking_mode_ = response->is_parked;
}


geometry_msgs::msg::Quaternion
RangerROSMessenger::CreateQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  if (angular < 1e-6) {
    radius = std::numeric_limits<double>::infinity();
    return 0.0;
  }
  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1 : -1;

  double l, phi_i;
  l = robot_params_.wheelbase;
  phi_i = atan((l / 2) / radius);

  const double max_phi_rad = 40.0 * M_PI / 180.0;
  phi_i = std::min(phi_i, max_phi_rad);

  return k * phi_i;
}

double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = std::abs(angle);

  phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                  (robot_params_.wheelbase * std::cos(phi_i) +
                   robot_params_.track * std::sin(phi_i)));

  phi *= angle >= 0 ? 1.0 : -1.0;
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = std::abs(angle);
  double phi_i = 0;

  phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                    (robot_params_.wheelbase * std::cos(phi) -
                     robot_params_.track * std::sin(phi)));
  phi_i *= angle >= 0 ? 1.0 : -1.0;
  return phi_i;
}
}  // namespace westonrobot
