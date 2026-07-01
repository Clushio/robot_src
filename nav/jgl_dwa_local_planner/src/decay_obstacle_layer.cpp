#include <jgl_dwa_local_planner/decay_obstacle_layer.h>

#include <algorithm>
#include <cmath>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(jgl_dwa_local_planner::DecayObstacleLayer, costmap_2d::Layer)

namespace jgl_dwa_local_planner
{

DecayObstacleLayer::DecayObstacleLayer()
  : decay_time_(0.8)
  , prune_time_(10.0)
  , update_bounds_range_(2.2)
  , min_obstacle_height_(0.2)
  , max_obstacle_height_(1.2)
  , obstacle_range_(3.0)
  , transform_tolerance_(0.2)
  , resolution_(0.05)
  , lethal_threshold_(costmap_2d::LETHAL_OBSTACLE)
{
}

void DecayObstacleLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("enabled", enabled_, true);
  private_nh.param<std::string>("input_topic", input_topic_, "/livox_pcl0");
  private_nh.param("decay_time", decay_time_, decay_time_);
  private_nh.param("prune_time", prune_time_, prune_time_);
  private_nh.param("update_bounds_range", update_bounds_range_, update_bounds_range_);
  private_nh.param("min_obstacle_height", min_obstacle_height_, min_obstacle_height_);
  private_nh.param("max_obstacle_height", max_obstacle_height_, max_obstacle_height_);
  private_nh.param("obstacle_range", obstacle_range_, obstacle_range_);
  private_nh.param("transform_tolerance", transform_tolerance_, transform_tolerance_);

  int lethal_threshold = static_cast<int>(lethal_threshold_);
  private_nh.param("lethal_threshold", lethal_threshold, lethal_threshold);
  lethal_threshold_ = static_cast<unsigned char>(std::max(0, std::min(254, lethal_threshold)));

  if (layered_costmap_ != nullptr)
  {
    global_frame_ = layered_costmap_->getGlobalFrameID();
    if (layered_costmap_->getCostmap() != nullptr)
    {
      resolution_ = layered_costmap_->getCostmap()->getResolution();
    }
  }

  cloud_sub_ = nh_.subscribe(input_topic_, 1, &DecayObstacleLayer::cloudCallback, this);
  current_ = true;

  ROS_INFO("DecayObstacleLayer: input=%s decay_time=%.2f frame=%s",
           input_topic_.c_str(), decay_time_, global_frame_.c_str());
}

void DecayObstacleLayer::matchSize()
{
  if (layered_costmap_ != nullptr && layered_costmap_->getCostmap() != nullptr)
  {
    resolution_ = layered_costmap_->getCostmap()->getResolution();
  }
}

void DecayObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
  {
    return;
  }

  (void)robot_yaw;
  const double range = std::max(0.0, update_bounds_range_);
  *min_x = std::min(*min_x, robot_x - range);
  *min_y = std::min(*min_y, robot_y - range);
  *max_x = std::max(*max_x, robot_x + range);
  *max_y = std::max(*max_y, robot_y + range);
}

void DecayObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
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

  const ros::Time now = ros::Time::now();
  std::lock_guard<std::mutex> lock(mutex_);

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      const auto iter = last_seen_.find(worldKey(wx, wy));
      if (iter == last_seen_.end())
      {
        continue;
      }

      if ((now - iter->second).toSec() > decay_time_ &&
          master_grid.getCost(i, j) >= lethal_threshold_)
      {
        master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
      }
    }
  }

  pruneOldCells(now, master_grid);
}

void DecayObstacleLayer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (!enabled_ || global_frame_.empty())
  {
    return;
  }

  geometry_msgs::TransformStamped transform_msg;
  try
  {
    transform_msg = tf_->lookupTransform(global_frame_, cloud_msg->header.frame_id,
                                         cloud_msg->header.stamp, ros::Duration(transform_tolerance_));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "DecayObstacleLayer transform failed: %s", ex.what());
    current_ = false;
    return;
  }

  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);
  const ros::Time stamp = ros::Time::now();

  std::lock_guard<std::mutex> lock(mutex_);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }

    if (obstacle_range_ > 0.0 && std::hypot(x, y) > obstacle_range_)
    {
      continue;
    }

    const tf2::Vector3 point = transform * tf2::Vector3(x, y, z);
    if (point.z() < min_obstacle_height_ || point.z() > max_obstacle_height_)
    {
      continue;
    }

    last_seen_[worldKey(point.x(), point.y())] = stamp;
  }

  current_ = true;
}

uint64_t DecayObstacleLayer::worldKey(double world_x, double world_y) const
{
  const double resolution = std::max(resolution_, 1e-6);
  const int cell_x = static_cast<int>(std::floor(world_x / resolution));
  const int cell_y = static_cast<int>(std::floor(world_y / resolution));
  return (static_cast<uint64_t>(static_cast<uint32_t>(cell_x)) << 32) |
         static_cast<uint32_t>(cell_y);
}

void DecayObstacleLayer::keyToWorld(uint64_t key, double* world_x, double* world_y) const
{
  const double resolution = std::max(resolution_, 1e-6);
  const int cell_x = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
  const int cell_y = static_cast<int32_t>(static_cast<uint32_t>(key & 0xffffffff));
  *world_x = (static_cast<double>(cell_x) + 0.5) * resolution;
  *world_y = (static_cast<double>(cell_y) + 0.5) * resolution;
}

void DecayObstacleLayer::pruneOldCells(const ros::Time& now, const costmap_2d::Costmap2D& master_grid)
{
  const double prune_time = std::max(prune_time_, decay_time_);
  const double origin_x = master_grid.getOriginX();
  const double origin_y = master_grid.getOriginY();
  const double max_x = origin_x + master_grid.getSizeInMetersX();
  const double max_y = origin_y + master_grid.getSizeInMetersY();

  for (auto iter = last_seen_.begin(); iter != last_seen_.end();)
  {
    double wx = 0.0;
    double wy = 0.0;
    keyToWorld(iter->first, &wx, &wy);
    const bool outside_window = wx < origin_x || wx > max_x || wy < origin_y || wy > max_y;
    if (outside_window && (now - iter->second).toSec() > prune_time)
    {
      iter = last_seen_.erase(iter);
    }
    else
    {
      ++iter;
    }
  }
}

}  // namespace jgl_dwa_local_planner
