#ifndef JGL_DWA_LOCAL_PLANNER_PARAMETER_UTILS_H_
#define JGL_DWA_LOCAL_PLANNER_PARAMETER_UTILS_H_

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace jgl_dwa_local_planner
{

template<typename T>
T declareOrGet(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::string &name,
    const T &default_value)
{
  if (!node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }
  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_PARAMETER_UTILS_H_
