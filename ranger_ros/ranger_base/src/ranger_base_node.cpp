/**
* @file ranger_base_node.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

using namespace westonrobot;

int main(int argc, char** argv)
{
  // setup ROS node
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ranger_base_node");
  // instantiate a robot object
  RangerROSMessenger messenger(node);
  messenger.Run();

  return 0;
}
