#ifndef JGL_DWA_LOCAL_PLANNER_DECAY_OBSTACLE_LAYER_H_
#define JGL_DWA_LOCAL_PLANNER_DECAY_OBSTACLE_LAYER_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>

namespace jgl_dwa_local_planner
{

class DecayObstacleLayer : public costmap_2d::Layer
{
public:
  DecayObstacleLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;
  void matchSize() override;

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  uint64_t worldKey(double world_x, double world_y) const;
  void keyToWorld(uint64_t key, double* world_x, double* world_y) const;
  void pruneOldCells(const ros::Time& now, const costmap_2d::Costmap2D& master_grid);

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  std::unordered_map<uint64_t, ros::Time> last_seen_;
  mutable std::mutex mutex_;

  std::string input_topic_;
  std::string global_frame_;
  double decay_time_;
  double prune_time_;
  double update_bounds_range_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double obstacle_range_;
  double transform_tolerance_;
  double resolution_;
  unsigned char lethal_threshold_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_DECAY_OBSTACLE_LAYER_H_
