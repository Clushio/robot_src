#include <jgl_dwa_local_planner/blind_clear_layer.h>

#include <algorithm>
#include <cmath>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(jgl_dwa_local_planner::BlindClearLayer, costmap_2d::Layer)

namespace jgl_dwa_local_planner
{

BlindClearLayer::BlindClearLayer()
  : min_angle_(-110.0 * M_PI / 180.0)
  , max_angle_(110.0 * M_PI / 180.0)
  , clear_range_(2.2)
  , keep_radius_(0.15)
  , robot_x_(0.0)
  , robot_y_(0.0)
  , robot_yaw_(0.0)
{
}

void BlindClearLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  double min_angle_deg = -110.0;
  double max_angle_deg = 110.0;
  nh.param("enabled", enabled_, true);
  nh.param("min_angle_deg", min_angle_deg, min_angle_deg);
  nh.param("max_angle_deg", max_angle_deg, max_angle_deg);
  nh.param("clear_range", clear_range_, clear_range_);
  nh.param("keep_radius", keep_radius_, keep_radius_);

  min_angle_ = min_angle_deg * M_PI / 180.0;
  max_angle_ = max_angle_deg * M_PI / 180.0;
  current_ = true;
}

void BlindClearLayer::matchSize()
{
}

void BlindClearLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
  {
    return;
  }

  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;

  const double range = std::max(0.0, clear_range_);
  *min_x = std::min(*min_x, robot_x_ - range);
  *min_y = std::min(*min_y, robot_y_ - range);
  *max_x = std::max(*max_x, robot_x_ + range);
  *max_y = std::max(*max_y, robot_y_ + range);
}

void BlindClearLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                  int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(size_x, max_i);
  max_j = std::min(size_y, max_j);

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);
      if (isInBlindZone(wx, wy))
      {
        master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
      }
    }
  }
}

bool BlindClearLayer::isInBlindZone(double world_x, double world_y) const
{
  const double dx = world_x - robot_x_;
  const double dy = world_y - robot_y_;
  const double distance = std::hypot(dx, dy);
  if (distance < keep_radius_ || distance > clear_range_)
  {
    return false;
  }

  const double c = std::cos(robot_yaw_);
  const double s = std::sin(robot_yaw_);
  const double base_x = c * dx + s * dy;
  const double base_y = -s * dx + c * dy;
  const double angle = std::atan2(base_y, base_x);

  return angle < min_angle_ || angle > max_angle_;
}

}  // namespace jgl_dwa_local_planner
