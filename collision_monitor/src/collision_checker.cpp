#include <collision_monitor/collision_checker.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <costmap_2d/footprint.h>
#include <tf2/utils.h>

namespace collision_monitor {
namespace {

constexpr double kEpsilon = 1e-9;

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double Approach(double current, double target, double accel, double decel,
                double dt) {
  double rate = accel;
  if (std::abs(target) < std::abs(current) && current * target >= 0.0) {
    rate = decel;
  }
  rate = std::max(rate, 1e-6);
  const double delta = target - current;
  const double step = rate * dt;
  if (std::abs(delta) <= step) {
    return target;
  }
  return current + std::copysign(step, delta);
}

void ApproachLinearVelocity(double current_x, double current_y,
                            double target_x, double target_y, double accel,
                            double decel, double dt, double& next_x,
                            double& next_y) {
  const double current_speed = std::hypot(current_x, current_y);
  const double target_speed = std::hypot(target_x, target_y);
  const double dot = current_x * target_x + current_y * target_y;
  const bool slowing_without_reversal =
      target_speed < current_speed && dot >= 0.0;
  const double rate = std::max(
      slowing_without_reversal ? decel : accel, 1e-6);
  const double delta_x = target_x - current_x;
  const double delta_y = target_y - current_y;
  const double delta_speed = std::hypot(delta_x, delta_y);
  const double step = rate * dt;
  if (delta_speed <= step || delta_speed <= kEpsilon) {
    next_x = target_x;
    next_y = target_y;
    return;
  }
  const double ratio = step / delta_speed;
  next_x = current_x + ratio * delta_x;
  next_y = current_y + ratio * delta_y;
}

double Cross(double ax, double ay, double bx, double by, double cx,
             double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool PointInRect(double x, double y, double min_x, double min_y, double max_x,
                 double max_y) {
  return x >= min_x - kEpsilon && x <= max_x + kEpsilon &&
         y >= min_y - kEpsilon && y <= max_y + kEpsilon;
}

bool PointInPolygon(double x, double y,
                    const std::vector<geometry_msgs::Point>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const double cross = Cross(xj, yj, xi, yi, x, y);
    if (std::abs(cross) <= kEpsilon &&
        x >= std::min(xi, xj) - kEpsilon &&
        x <= std::max(xi, xj) + kEpsilon &&
        y >= std::min(yi, yj) - kEpsilon &&
        y <= std::max(yi, yj) + kEpsilon) {
      return true;
    }
    const bool intersects = ((yi > y) != (yj > y)) &&
                            (x < (xj - xi) * (y - yi) /
                                         (yj - yi) +
                                     xi);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

int Orientation(double ax, double ay, double bx, double by, double cx,
                double cy) {
  const double value = Cross(ax, ay, bx, by, cx, cy);
  if (std::abs(value) <= kEpsilon) {
    return 0;
  }
  return value > 0.0 ? 1 : -1;
}

bool OnSegment(double ax, double ay, double bx, double by, double px,
               double py) {
  return std::abs(Cross(ax, ay, bx, by, px, py)) <= kEpsilon &&
         px >= std::min(ax, bx) - kEpsilon &&
         px <= std::max(ax, bx) + kEpsilon &&
         py >= std::min(ay, by) - kEpsilon &&
         py <= std::max(ay, by) + kEpsilon;
}

bool SegmentsIntersect(double ax, double ay, double bx, double by, double cx,
                       double cy, double dx, double dy) {
  const int o1 = Orientation(ax, ay, bx, by, cx, cy);
  const int o2 = Orientation(ax, ay, bx, by, dx, dy);
  const int o3 = Orientation(cx, cy, dx, dy, ax, ay);
  const int o4 = Orientation(cx, cy, dx, dy, bx, by);
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  return (o1 == 0 && OnSegment(ax, ay, bx, by, cx, cy)) ||
         (o2 == 0 && OnSegment(ax, ay, bx, by, dx, dy)) ||
         (o3 == 0 && OnSegment(cx, cy, dx, dy, ax, ay)) ||
         (o4 == 0 && OnSegment(cx, cy, dx, dy, bx, by));
}

bool PolygonIntersectsCell(const std::vector<geometry_msgs::Point>& polygon,
                           double min_x, double min_y, double max_x,
                           double max_y) {
  for (const geometry_msgs::Point& point : polygon) {
    if (PointInRect(point.x, point.y, min_x, min_y, max_x, max_y)) {
      return true;
    }
  }

  const double corners[4][2] = {
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
  for (const auto& corner : corners) {
    if (PointInPolygon(corner[0], corner[1], polygon)) {
      return true;
    }
  }

  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const geometry_msgs::Point& a = polygon[i];
    const geometry_msgs::Point& b = polygon[(i + 1) % polygon.size()];
    for (int edge = 0; edge < 4; ++edge) {
      const double* c = corners[edge];
      const double* d = corners[(edge + 1) % 4];
      if (SegmentsIntersect(a.x, a.y, b.x, b.y, c[0], c[1], d[0], d[1])) {
        return true;
      }
    }
  }
  return false;
}

bool IsBlocked(int value, const GridPolicy& policy) {
  if (value < 0) {
    return policy.unknown_is_obstacle;
  }
  return value >= policy.occupied_threshold;
}

}  // namespace

CollisionChecker::CollisionChecker() = default;

bool CollisionChecker::setFootprint(
    const std::vector<geometry_msgs::Point>& footprint, double padding,
    std::string* error) {
  if (footprint.size() < 3) {
    if (error != nullptr) {
      *error = "footprint must contain at least three points";
    }
    return false;
  }
  for (const geometry_msgs::Point& point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      if (error != nullptr) {
        *error = "footprint contains a non-finite point";
      }
      return false;
    }
  }

  footprint_ = footprint;
  costmap_2d::padFootprint(footprint_, std::max(0.0, padding));
  circumscribed_radius_ = 0.0;
  for (const geometry_msgs::Point& point : footprint_) {
    circumscribed_radius_ =
        std::max(circumscribed_radius_, std::hypot(point.x, point.y));
  }
  return true;
}

const std::vector<geometry_msgs::Point>& CollisionChecker::footprint() const {
  return footprint_;
}

double CollisionChecker::circumscribedRadius() const {
  return circumscribed_radius_;
}

bool CollisionChecker::poseCollides(const Pose2D& pose,
                                    const nav_msgs::OccupancyGrid& grid,
                                    const GridPolicy& policy) const {
  const geometry_msgs::Quaternion& orientation = grid.info.origin.orientation;
  const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  if (footprint_.size() < 3 || grid.info.resolution <= 0.0 ||
      grid.info.width == 0 || grid.info.height == 0 ||
      grid.data.size() != static_cast<std::size_t>(grid.info.width) *
                              static_cast<std::size_t>(grid.info.height) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw) || !std::isfinite(grid.info.resolution) ||
      !std::isfinite(grid.info.origin.position.x) ||
      !std::isfinite(grid.info.origin.position.y) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      quaternion_norm < kEpsilon) {
    return true;
  }

  std::vector<geometry_msgs::Point> transformed;
  costmap_2d::transformFootprint(pose.x, pose.y, pose.yaw, footprint_,
                                 transformed);
  return footprintIntersectsGrid(transformed, grid, policy);
}

bool CollisionChecker::footprintIntersectsGrid(
    const std::vector<geometry_msgs::Point>& world_footprint,
    const nav_msgs::OccupancyGrid& grid, const GridPolicy& policy) const {
  const double origin_yaw = tf2::getYaw(grid.info.origin.orientation);
  const double c = std::cos(origin_yaw);
  const double s = std::sin(origin_yaw);
  const double resolution = grid.info.resolution;

  std::vector<geometry_msgs::Point> polygon;
  polygon.reserve(world_footprint.size());
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const geometry_msgs::Point& world : world_footprint) {
    const double dx = world.x - grid.info.origin.position.x;
    const double dy = world.y - grid.info.origin.position.y;
    geometry_msgs::Point local;
    local.x = c * dx + s * dy;
    local.y = -s * dx + c * dy;
    polygon.push_back(local);
    min_x = std::min(min_x, local.x);
    min_y = std::min(min_y, local.y);
    max_x = std::max(max_x, local.x);
    max_y = std::max(max_y, local.y);
  }

  const double map_width = grid.info.width * resolution;
  const double map_height = grid.info.height * resolution;
  if (policy.outside_is_obstacle &&
      (min_x < 0.0 || min_y < 0.0 || max_x >= map_width ||
       max_y >= map_height)) {
    return true;
  }

  const int min_mx = std::max(0, static_cast<int>(std::floor(min_x / resolution)));
  const int min_my = std::max(0, static_cast<int>(std::floor(min_y / resolution)));
  const int max_mx = std::min(static_cast<int>(grid.info.width) - 1,
                              static_cast<int>(std::floor(max_x / resolution)));
  const int max_my = std::min(static_cast<int>(grid.info.height) - 1,
                              static_cast<int>(std::floor(max_y / resolution)));

  if (min_mx > max_mx || min_my > max_my) {
    return policy.outside_is_obstacle;
  }

  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      const int index = my * static_cast<int>(grid.info.width) + mx;
      if (!IsBlocked(grid.data[index], policy)) {
        continue;
      }
      const double cell_min_x = mx * resolution;
      const double cell_min_y = my * resolution;
      if (PolygonIntersectsCell(polygon, cell_min_x, cell_min_y,
                                cell_min_x + resolution,
                                cell_min_y + resolution)) {
        return true;
      }
    }
  }
  return false;
}

CollisionResult CollisionChecker::simulate(
    const MotionState& initial, double target_linear_x, double target_linear_y,
    double target_angular_z, double hold_time,
    const nav_msgs::OccupancyGrid& static_map,
    const GridPolicy& static_policy, const nav_msgs::OccupancyGrid& local_map,
    const GridPolicy& local_policy, const RolloutOptions& options) const {
  CollisionResult result;
  MotionState state = initial;
  result.poses.push_back(state.pose);

  auto check_collision = [&](double time) {
    const bool static_collision =
        poseCollides(state.pose, static_map, static_policy);
    const bool local_collision =
        poseCollides(state.pose, local_map, local_policy);
    if (!static_collision && !local_collision) {
      return false;
    }
    result.collision = true;
    result.static_collision = static_collision;
    result.local_collision = local_collision;
    result.collision_time = time;
    result.collision_pose = state.pose;
    return true;
  };

  if (check_collision(0.0)) {
    return result;
  }

  const double maximum_brake_time = std::max(
      std::hypot(target_linear_x, target_linear_y) /
          std::max(options.linear_decel, 1e-6),
      std::abs(target_angular_z) /
          std::max(options.angular_decel, 1e-6));
  const double maximum_initial_brake_time = std::max(
      std::hypot(initial.linear_x, initial.linear_y) /
          std::max(options.linear_decel, 1e-6),
      std::abs(initial.angular_z) /
          std::max(options.angular_decel, 1e-6));
  const double end_time = std::max(0.0, hold_time) +
                          std::max(maximum_brake_time,
                                   maximum_initial_brake_time) +
                          2.0;

  double time = 0.0;
  for (int step = 0; step < 20000 && time < end_time; ++step) {
    const bool holding = time < hold_time;
    const double desired_linear_x = holding ? target_linear_x : 0.0;
    const double desired_linear_y = holding ? target_linear_y : 0.0;
    const double desired_angular_z = holding ? target_angular_z : 0.0;
    double dt = std::max(0.001, options.max_dt);
    for (int refinement = 0; refinement < 2; ++refinement) {
      double estimate_linear_x = 0.0;
      double estimate_linear_y = 0.0;
      ApproachLinearVelocity(
          state.linear_x, state.linear_y, desired_linear_x, desired_linear_y,
          options.linear_accel, options.linear_decel, dt,
          estimate_linear_x, estimate_linear_y);
      const double estimate_angular_z =
          Approach(state.angular_z, desired_angular_z, options.angular_accel,
                   options.angular_decel, dt);
      const double point_speed =
          std::max(std::hypot(state.linear_x, state.linear_y),
                   std::hypot(estimate_linear_x, estimate_linear_y)) +
          circumscribed_radius_ *
              std::max(std::abs(state.angular_z),
                       std::abs(estimate_angular_z));
      if (point_speed > 1e-6) {
        dt = std::min(dt, options.max_corner_step / point_speed);
      }
      dt = std::max(0.001, dt);
    }

    double next_linear_x = 0.0;
    double next_linear_y = 0.0;
    ApproachLinearVelocity(
        state.linear_x, state.linear_y, desired_linear_x, desired_linear_y,
        options.linear_accel, options.linear_decel, dt, next_linear_x,
        next_linear_y);
    const double next_angular_z =
        Approach(state.angular_z, desired_angular_z, options.angular_accel,
                 options.angular_decel, dt);
    const double linear_x = 0.5 * (state.linear_x + next_linear_x);
    const double linear_y = 0.5 * (state.linear_y + next_linear_y);
    const double angular_z = 0.5 * (state.angular_z + next_angular_z);

    const double c = std::cos(state.pose.yaw);
    const double s = std::sin(state.pose.yaw);
    if (std::abs(angular_z) < 1e-8) {
      state.pose.x += (linear_x * c - linear_y * s) * dt;
      state.pose.y += (linear_x * s + linear_y * c) * dt;
    } else {
      const double angle = angular_z * dt;
      const double sin_angle = std::sin(angle);
      const double one_minus_cos = 1.0 - std::cos(angle);
      const double body_x =
          (sin_angle * linear_x - one_minus_cos * linear_y) / angular_z;
      const double body_y =
          (one_minus_cos * linear_x + sin_angle * linear_y) / angular_z;
      state.pose.x += c * body_x - s * body_y;
      state.pose.y += s * body_x + c * body_y;
      const double next_yaw = state.pose.yaw + angle;
      state.pose.yaw = NormalizeAngle(next_yaw);
    }
    state.linear_x = next_linear_x;
    state.linear_y = next_linear_y;
    state.angular_z = next_angular_z;
    time += dt;
    result.poses.push_back(state.pose);

    if (check_collision(time)) {
      return result;
    }
    if (!holding && std::abs(state.linear_x) <= options.stopped_linear &&
        std::abs(state.linear_y) <= options.stopped_linear &&
        std::abs(state.angular_z) <= options.stopped_angular) {
      break;
    }
  }
  return result;
}

}  // namespace collision_monitor
