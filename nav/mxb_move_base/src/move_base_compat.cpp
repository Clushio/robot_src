// Copyright 2026 MXB navigation maintainers
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

namespace mxb_move_base
{

class MoveBaseCompatibilityNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;
  using GetPlan = nav_msgs::srv::GetPlan;

  MoveBaseCompatibilityNode()
  : Node("mxb_move_base")
  {
    const auto navigate_action = declare_parameter<std::string>(
      "navigate_action", "/navigate_to_pose");
    planner_service_name_ = declare_parameter<std::string>(
      "planner_service", "/planner_server/GridBased/make_plan");
    local_clear_service_name_ = declare_parameter<std::string>(
      "local_clear_service", "/local_costmap/clear_entirely_local_costmap");
    global_clear_service_name_ = declare_parameter<std::string>(
      "global_clear_service", "/global_costmap/clear_entirely_global_costmap");
    fixed_route_behavior_tree_ = declare_parameter<std::string>(
      "fixed_route_behavior_tree", "");
    service_timeout_ = std::chrono::duration<double>(
      declare_parameter<double>("service_timeout", 3.0));

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigate_action, callback_group_);
    planner_client_ = create_client<GetPlan>(
      planner_service_name_, rmw_qos_profile_services_default, callback_group_);
    local_clear_client_ = create_client<ClearCostmap>(
      local_clear_service_name_, rmw_qos_profile_services_default, callback_group_);
    global_clear_client_ = create_client<ClearCostmap>(
      global_clear_service_name_, rmw_qos_profile_services_default, callback_group_);

    current_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/current_goal", rclcpp::QoS(1));
    simple_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(10),
      std::bind(&MoveBaseCompatibilityNode::simpleGoalCallback, this, std::placeholders::_1));
    fixed_route_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/anav/fixed_route_mode", rclcpp::QoS(1).transient_local(),
      std::bind(&MoveBaseCompatibilityNode::fixedRouteCallback, this, std::placeholders::_1));

    make_plan_service_ = create_service<GetPlan>(
      "~/make_plan",
      std::bind(
        &MoveBaseCompatibilityNode::makePlanCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    clear_costmaps_service_ = create_service<std_srvs::srv::Empty>(
      "~/clear_costmaps",
      std::bind(
        &MoveBaseCompatibilityNode::clearCostmapsCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "ROS1 compatibility endpoints are ready; navigation goals are forwarded to %s.",
      navigate_action.c_str());
  }

private:
  template<typename ServiceT>
  bool waitForService(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const std::string & service_name)
  {
    if (client->wait_for_service(500ms)) {
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Service %s is unavailable.", service_name.c_str());
    return false;
  }

  void fixedRouteCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool previous = fixed_route_mode_.exchange(message->data);
    if (previous != message->data) {
      RCLCPP_INFO(
        get_logger(), "Fixed-route compatibility mode %s.",
        message->data ? "enabled" : "disabled");
    }
  }

  void simpleGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!navigate_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(
        get_logger(), "NavigateToPose action server is unavailable; simple goal rejected.");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = *message;
    if (goal.pose.header.stamp.sec == 0 && goal.pose.header.stamp.nanosec == 0) {
      goal.pose.header.stamp = now();
    }
    if (fixed_route_mode_.load() && !fixed_route_behavior_tree_.empty()) {
      goal.behavior_tree = fixed_route_behavior_tree_;
    }

    current_goal_pub_->publish(goal.pose);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const NavigateGoalHandle::SharedPtr & handle) {
        if (handle) {
          RCLCPP_INFO(get_logger(), "Simple goal accepted by NavigateToPose.");
        } else {
          RCLCPP_ERROR(get_logger(), "Simple goal rejected by NavigateToPose.");
        }
      };
    options.result_callback =
      [this](const NavigateGoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Simple navigation goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Simple navigation goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Simple navigation goal canceled.");
            break;
          default:
            RCLCPP_WARN(get_logger(), "Simple navigation goal ended with an unknown result.");
            break;
        }
      };
    navigate_client_->async_send_goal(goal, options);
  }

  void makePlanCallback(
    const GetPlan::Request::SharedPtr request,
    GetPlan::Response::SharedPtr response)
  {
    if (!waitForService<GetPlan>(planner_client_, planner_service_name_)) {
      return;
    }
    auto future = planner_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(), "Timed out forwarding make_plan to %s.", planner_service_name_.c_str());
      return;
    }
    response->plan = future.get()->plan;
  }

  bool clearOneCostmap(
    const rclcpp::Client<ClearCostmap>::SharedPtr & client,
    const std::string & service_name)
  {
    if (!waitForService<ClearCostmap>(client, service_name)) {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<ClearCostmap::Request>());
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out clearing costmap through %s.", service_name.c_str());
      return false;
    }
    (void)future.get();
    return true;
  }

  void clearCostmapsCallback(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    const bool local_cleared = clearOneCostmap(local_clear_client_, local_clear_service_name_);
    const bool global_cleared = clearOneCostmap(global_clear_client_, global_clear_service_name_);
    if (local_cleared && global_cleared) {
      RCLCPP_INFO(get_logger(), "Local and global costmaps cleared.");
    }
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp::Client<GetPlan>::SharedPtr planner_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr global_clear_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr simple_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fixed_route_sub_;
  rclcpp::Service<GetPlan>::SharedPtr make_plan_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_costmaps_service_;

  std::string planner_service_name_;
  std::string local_clear_service_name_;
  std::string global_clear_service_name_;
  std::string fixed_route_behavior_tree_;
  std::chrono::duration<double> service_timeout_{3.0};
  std::atomic<bool> fixed_route_mode_{false};
};

}  // namespace mxb_move_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mxb_move_base::MoveBaseCompatibilityNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
