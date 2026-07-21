/**
* @file ranger_base_node.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/details/robot_base/ranger_base.hpp"

using namespace westonrobot;

int main(int argc, char** argv)
{
  // setup ROS node
  ros::init(argc, argv, "ranger_node");
  ros::NodeHandle node("~");

  // instantiate a robot object
  // robot = std::make_shared<RangerRobot>();
  RangerROSMessenger messenger(&node);
  messenger.Run();

  // // publish robot state at 50Hz while listening to twist commands
  // ros::Rate rate(50);
  // while (ros::ok()) {
  //   if (!messenger.simulated_robot_) {
  //     messenger.PublishStateToROS();
  //   } else {
  //     double linear, angular;
  //     messenger.GetCurrentMotionCmdForSim(linear, angular);
  //     messenger.PublishSimStateToROS(linear, angular);
  //   }
  //   ros::spinOnce();
  //   rate.sleep();
  // }

  return 0;
}
