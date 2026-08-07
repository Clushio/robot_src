#ifndef COLLISION_MONITOR_COLLISION_CHECKER_H_
#define COLLISION_MONITOR_COLLISION_CHECKER_H_

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

namespace collision_monitor {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct MotionState {
  Pose2D pose;
  double linear = 0.0;
  double angular = 0.0;
};

struct GridPolicy {
  int occupied_threshold = 100;
  bool unknown_is_obstacle = true;
  bool outside_is_obstacle = true;
};

struct RolloutOptions {
  double max_dt = 0.02;
  double max_corner_step = 0.025;
  double linear_accel = 3.0;
  double angular_accel = 4.2;
  double linear_decel = 0.5;
  double angular_decel = 0.8;
  double stopped_linear = 0.005;
  double stopped_angular = 0.01;
};

struct CollisionResult {
  bool collision = false;
  bool static_collision = false;
  bool local_collision = false;
  double collision_time = -1.0;
  Pose2D collision_pose;
  std::vector<Pose2D> poses;
};

class CollisionChecker {
 public:
  CollisionChecker();

  bool setFootprint(const std::vector<geometry_msgs::Point>& footprint,
                    double padding, std::string* error = nullptr);
  const std::vector<geometry_msgs::Point>& footprint() const;
  double circumscribedRadius() const;

  bool poseCollides(const Pose2D& pose, const nav_msgs::OccupancyGrid& grid,
                    const GridPolicy& policy) const;

  CollisionResult simulate(const MotionState& initial,
                           double target_linear,
                           double target_angular,
                           double hold_time,
                           const nav_msgs::OccupancyGrid& static_map,
                           const GridPolicy& static_policy,
                           const nav_msgs::OccupancyGrid& local_map,
                           const GridPolicy& local_policy,
                           const RolloutOptions& options) const;

 private:
  bool footprintIntersectsGrid(
      const std::vector<geometry_msgs::Point>& world_footprint,
      const nav_msgs::OccupancyGrid& grid,
      const GridPolicy& policy) const;

  std::vector<geometry_msgs::Point> footprint_;
  double circumscribed_radius_ = 0.0;
};

}  // namespace collision_monitor

#endif  // COLLISION_MONITOR_COLLISION_CHECKER_H_
