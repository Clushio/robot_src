#include <jgl_dwa_local_planner/trajectory_generator.h>

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace jgl_dwa_local_planner
{

TrajectoryGenerator::TrajectoryGenerator()
    : have_global_costmap_(false),
      sample_resolution_(0.10),
      safe_distance_(0.25),
      max_deviation_from_topo_(0.50),
      max_curvature_(2.0),
      occupied_threshold_(98),
      last_path_mode_(PATH_MODE_INVALID)
{
}

void TrajectoryGenerator::initialize(ros::NodeHandle &private_nh, ros::NodeHandle &node_nh)
{
  private_nh.param("bspline_sample_resolution", sample_resolution_, 0.10);
  private_nh.param("safe_distance", safe_distance_, 0.25);
  private_nh.param("max_deviation_from_topo", max_deviation_from_topo_, 0.50);
  private_nh.param("max_curvature", max_curvature_, 2.0);
  private_nh.param("reference_occupied_threshold", occupied_threshold_, 98);

  sample_resolution_ = std::max(0.02, sample_resolution_);
  safe_distance_ = std::max(0.0, safe_distance_);
  max_deviation_from_topo_ = std::max(0.0, max_deviation_from_topo_);
  max_curvature_ = std::max(0.0, max_curvature_);
  occupied_threshold_ = std::max(1, std::min(100, occupied_threshold_));

  global_costmap_sub_ = node_nh.subscribe("/mxb_move_base/global_costmap/costmap", 1,
                                          &TrajectoryGenerator::globalCostmapCallback, this);
}

void TrajectoryGenerator::globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(costmap_mutex_);
  global_costmap_ = *msg;
  have_global_costmap_ = true;
}

bool TrajectoryGenerator::generate(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                                   nav_msgs::Path &out_path)
{
  out_path.poses.clear();
  if (waypoints.size() < 2)
  {
    return false;
  }

  nav_msgs::Path spline_path = catmullRomPath(waypoints);
  if (pathChecksPass(spline_path, waypoints, true))
  {
    out_path = spline_path;
    last_path_mode_ = PATH_MODE_CUBIC;
    last_fallback_segments_.assign(waypoints.size() - 1, 0);
    ROS_INFO("JGL reference path: generated cubic path with %zu samples.",
             out_path.poses.size());
    return true;
  }

  ROS_WARN("JGL reference path: full cubic path failed checks, trying segment-wise hybrid fallback.");
  unsigned int hybrid_fallback_segments = 0;
  unsigned int total_segments = 0;
  std::vector<int> hybrid_fallback_flags;
  nav_msgs::Path hybrid_path = hybridPath(waypoints,
                                          &hybrid_fallback_segments,
                                          &total_segments,
                                          &hybrid_fallback_flags);
  if (hybrid_fallback_segments < total_segments &&
      pathChecksPass(hybrid_path, waypoints, false))
  {
    out_path = hybrid_path;
    last_path_mode_ = hybrid_fallback_segments == 0 ? PATH_MODE_CUBIC : PATH_MODE_HYBRID;
    last_fallback_segments_ = hybrid_fallback_flags;
    ROS_WARN("JGL reference path: using hybrid path with %u/%u segments as polyline fallback, %zu samples.",
             hybrid_fallback_segments, total_segments, out_path.poses.size());
    return true;
  }

  ROS_WARN("JGL reference path: hybrid path failed checks, fallback to full polyline.");
  nav_msgs::Path fallback_path = fallbackPolylinePath(waypoints);
  if (pathChecksPass(fallback_path, waypoints, false))
  {
    out_path = fallback_path;
    last_path_mode_ = PATH_MODE_POLYLINE_FALLBACK;
    last_fallback_segments_.assign(waypoints.size() - 1, 1);
    ROS_WARN("JGL reference path: using full collision-checked polyline fallback with %zu samples.",
             out_path.poses.size());
    return true;
  }

  ROS_ERROR("JGL reference path: cubic and polyline fallback both failed checks.");
  last_path_mode_ = PATH_MODE_INVALID;
  last_fallback_segments_.clear();
  return false;
}

const char *TrajectoryGenerator::lastPathModeName() const
{
  switch (last_path_mode_)
  {
    case PATH_MODE_CUBIC:
      return "cubic";
    case PATH_MODE_HYBRID:
      return "hybrid";
    case PATH_MODE_POLYLINE_FALLBACK:
      return "polyline_fallback";
    default:
      return "invalid";
  }
}

bool TrajectoryGenerator::pathChecksPass(
    const nav_msgs::Path &path,
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    bool check_curvature)
{
  return path.poses.size() >= 2 &&
         checkCollision(path) &&
         checkDeviationFromTopo(path, waypoints) &&
         (!check_curvature || checkCurvature(path));
}

nav_msgs::Path TrajectoryGenerator::catmullRomPath(
    const std::vector<geometry_msgs::PoseStamped> &waypoints)
{
  nav_msgs::Path path;
  if (waypoints.empty())
  {
    return path;
  }

  if (waypoints.size() < 3)
  {
    return fallbackPolylinePath(waypoints);
  }

  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    appendPathSegment(path, catmullRomSegment(waypoints, i));
  }

  if (!path.poses.empty())
  {
    path.poses.front().pose.position.x = waypoints.front().pose.position.x;
    path.poses.front().pose.position.y = waypoints.front().pose.position.y;
    path.poses.back().pose.position.x = waypoints.back().pose.position.x;
    path.poses.back().pose.position.y = waypoints.back().pose.position.y;
  }
  applyPathOrientations(path, waypoints.back());
  return path;
}

nav_msgs::Path TrajectoryGenerator::catmullRomSegment(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    unsigned int segment_index) const
{
  nav_msgs::Path path;
  if (waypoints.empty() || segment_index + 1 >= waypoints.size())
  {
    return path;
  }

  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();

  const geometry_msgs::PoseStamped &p0 =
      (segment_index == 0) ? waypoints[segment_index] : waypoints[segment_index - 1];
  const geometry_msgs::PoseStamped &p1 = waypoints[segment_index];
  const geometry_msgs::PoseStamped &p2 = waypoints[segment_index + 1];
  const geometry_msgs::PoseStamped &p3 =
      (segment_index + 2 < waypoints.size()) ? waypoints[segment_index + 2]
                                             : waypoints[segment_index + 1];

  const double segment_length = poseDistance(p1, p2);
  const int samples =
      std::max(1, static_cast<int>(std::ceil(segment_length / sample_resolution_)));
  for (int j = 0; j <= samples; ++j)
  {
    const double t = static_cast<double>(j) / static_cast<double>(samples);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double x = 0.5 * ((2.0 * p1.pose.position.x) +
                            (-p0.pose.position.x + p2.pose.position.x) * t +
                            (2.0 * p0.pose.position.x - 5.0 * p1.pose.position.x +
                             4.0 * p2.pose.position.x - p3.pose.position.x) *
                                t2 +
                            (-p0.pose.position.x + 3.0 * p1.pose.position.x -
                             3.0 * p2.pose.position.x + p3.pose.position.x) *
                                t3);
    const double y = 0.5 * ((2.0 * p1.pose.position.y) +
                            (-p0.pose.position.y + p2.pose.position.y) * t +
                            (2.0 * p0.pose.position.y - 5.0 * p1.pose.position.y +
                             4.0 * p2.pose.position.y - p3.pose.position.y) *
                                t2 +
                            (-p0.pose.position.y + 3.0 * p1.pose.position.y -
                             3.0 * p2.pose.position.y + p3.pose.position.y) *
                                t3);
    path.poses.push_back(makePoseLike(p1, x, y));
  }

  return path;
}

nav_msgs::Path TrajectoryGenerator::hybridPath(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    unsigned int *fallback_segments,
    unsigned int *total_segments,
    std::vector<int> *fallback_segment_flags)
{
  nav_msgs::Path path;
  if (fallback_segments != NULL)
  {
    *fallback_segments = 0;
  }
  if (total_segments != NULL)
  {
    *total_segments = 0;
  }
  if (waypoints.size() < 2)
  {
    return path;
  }

  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();
  const unsigned int segment_count =
      static_cast<unsigned int>(waypoints.size() - 1);
  if (fallback_segment_flags != NULL)
  {
    fallback_segment_flags->assign(segment_count, 0);
  }
  if (total_segments != NULL)
  {
    *total_segments = segment_count;
  }

  for (unsigned int i = 0; i < segment_count; ++i)
  {
    nav_msgs::Path segment = catmullRomSegment(waypoints, i);
    const bool segment_ok = pathChecksPass(segment, waypoints, true);
    if (!segment_ok)
    {
      segment = fallbackPolylineSegment(waypoints[i], waypoints[i + 1]);
      if (fallback_segments != NULL)
      {
        ++(*fallback_segments);
      }
      if (fallback_segment_flags != NULL)
      {
        (*fallback_segment_flags)[i] = 1;
      }
      ROS_WARN("JGL reference path: segment %u falls back to polyline.", i);
    }
    appendPathSegment(path, segment);
  }

  if (!path.poses.empty())
  {
    path.poses.front().pose.position.x = waypoints.front().pose.position.x;
    path.poses.front().pose.position.y = waypoints.front().pose.position.y;
    path.poses.back().pose.position.x = waypoints.back().pose.position.x;
    path.poses.back().pose.position.y = waypoints.back().pose.position.y;
  }
  applyPathOrientations(path, waypoints.back());
  return path;
}

nav_msgs::Path TrajectoryGenerator::fallbackPolylinePath(
    const std::vector<geometry_msgs::PoseStamped> &waypoints)
{
  nav_msgs::Path path;
  if (waypoints.empty())
  {
    return path;
  }

  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    appendPathSegment(path, fallbackPolylineSegment(waypoints[i], waypoints[i + 1]));
  }

  applyPathOrientations(path, waypoints.back());
  return path;
}

nav_msgs::Path TrajectoryGenerator::fallbackPolylineSegment(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &end) const
{
  nav_msgs::Path path;
  path.header = start.header;
  path.header.stamp = ros::Time::now();

  const double dx = end.pose.position.x - start.pose.position.x;
  const double dy = end.pose.position.y - start.pose.position.y;
  const double length = std::hypot(dx, dy);
  const int samples =
      std::max(1, static_cast<int>(std::ceil(length / sample_resolution_)));
  for (int j = 0; j <= samples; ++j)
  {
    const double ratio = static_cast<double>(j) / static_cast<double>(samples);
    path.poses.push_back(makePoseLike(start,
                                      start.pose.position.x + ratio * dx,
                                      start.pose.position.y + ratio * dy));
  }
  return path;
}

void TrajectoryGenerator::appendPathSegment(nav_msgs::Path &path,
                                            const nav_msgs::Path &segment) const
{
  if (segment.poses.empty())
  {
    return;
  }
  const unsigned int start_index = path.poses.empty() ? 0U : 1U;
  for (unsigned int i = start_index; i < segment.poses.size(); ++i)
  {
    path.poses.push_back(segment.poses[i]);
  }
}

geometry_msgs::PoseStamped TrajectoryGenerator::makePoseLike(
    const geometry_msgs::PoseStamped &reference, double x, double y) const
{
  geometry_msgs::PoseStamped pose = reference;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 1.0;
  return pose;
}

void TrajectoryGenerator::applyPathOrientations(
    nav_msgs::Path &path,
    const geometry_msgs::PoseStamped &final_pose) const
{
  if (path.poses.empty())
  {
    return;
  }

  for (unsigned int i = 0; i + 1 < path.poses.size(); ++i)
  {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 1e-6)
    {
      continue;
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(dy, dx));
    tf2::convert(q, path.poses[i].pose.orientation);
  }
  path.poses.back().pose.orientation = final_pose.pose.orientation;
}

bool TrajectoryGenerator::checkCollision(const nav_msgs::Path &path)
{
  nav_msgs::OccupancyGrid grid;
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    if (!have_global_costmap_)
    {
      ROS_WARN_THROTTLE(2.0, "JGL reference path: waiting for global costmap.");
      return false;
    }
    grid = global_costmap_;
  }

  for (unsigned int i = 0; i < path.poses.size(); ++i)
  {
    if (poseCollides(path.poses[i], grid))
    {
      ROS_WARN("JGL reference path: collision check failed at sample %u (x=%.3f y=%.3f, threshold=%d, safe_distance=%.3f).",
               i,
               path.poses[i].pose.position.x,
               path.poses[i].pose.position.y,
               occupied_threshold_,
               safe_distance_);
      return false;
    }
  }
  return true;
}

bool TrajectoryGenerator::poseCollides(const geometry_msgs::PoseStamped &pose,
                                       const nav_msgs::OccupancyGrid &grid) const
{
  int mx = 0;
  int my = 0;
  if (!worldToMap(grid, pose.pose.position.x, pose.pose.position.y, mx, my))
  {
    return true;
  }

  const double resolution = std::max(1e-6, static_cast<double>(grid.info.resolution));
  const int radius_cells = static_cast<int>(std::ceil(safe_distance_ / resolution));
  for (int dx = -radius_cells; dx <= radius_cells; ++dx)
  {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      if (std::hypot(dx * resolution, dy * resolution) > safe_distance_ + 0.5 * resolution)
      {
        continue;
      }
      if (occupiedCell(grid, mx + dx, my + dy))
      {
        return true;
      }
    }
  }
  return false;
}

bool TrajectoryGenerator::worldToMap(const nav_msgs::OccupancyGrid &grid, double wx, double wy,
                                     int &mx, int &my) const
{
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double resolution = std::max(1e-6, static_cast<double>(grid.info.resolution));
  mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  my = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(grid.info.width) &&
         my < static_cast<int>(grid.info.height);
}

bool TrajectoryGenerator::occupiedCell(const nav_msgs::OccupancyGrid &grid, int mx, int my) const
{
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(grid.info.width) ||
      my >= static_cast<int>(grid.info.height))
  {
    return true;
  }

  const int index = my * static_cast<int>(grid.info.width) + mx;
  if (index < 0 || index >= static_cast<int>(grid.data.size()))
  {
    return true;
  }

  const int value = grid.data[index];
  return value < 0 || value >= occupied_threshold_;
}

bool TrajectoryGenerator::checkDeviationFromTopo(
    const nav_msgs::Path &path,
    const std::vector<geometry_msgs::PoseStamped> &waypoints)
{
  if (max_deviation_from_topo_ <= 0.0)
  {
    return true;
  }

  for (unsigned int i = 0; i < path.poses.size(); ++i)
  {
    const double distance = pointToPolylineDistance(path.poses[i], waypoints);
    if (distance > max_deviation_from_topo_)
    {
      ROS_WARN("JGL reference path: deviation %.3f exceeds max %.3f at sample %u.",
               distance, max_deviation_from_topo_, i);
      return false;
    }
  }
  return true;
}

double TrajectoryGenerator::pointToPolylineDistance(
    const geometry_msgs::PoseStamped &pose,
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  if (waypoints.empty())
  {
    return 0.0;
  }
  if (waypoints.size() == 1)
  {
    return poseDistance(pose, waypoints.front());
  }

  double best = 1e9;
  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    best = std::min(best, pointToSegmentDistance(pose.pose.position.x,
                                                 pose.pose.position.y,
                                                 waypoints[i],
                                                 waypoints[i + 1]));
  }
  return best;
}

double TrajectoryGenerator::pointToSegmentDistance(
    double px, double py,
    const geometry_msgs::PoseStamped &a,
    const geometry_msgs::PoseStamped &b) const
{
  const double ax = a.pose.position.x;
  const double ay = a.pose.position.y;
  const double bx = b.pose.position.x;
  const double by = b.pose.position.y;
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len_sq = dx * dx + dy * dy;
  if (len_sq < 1e-9)
  {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

bool TrajectoryGenerator::checkCurvature(const nav_msgs::Path &path)
{
  if (max_curvature_ <= 0.0 || path.poses.size() < 3)
  {
    return true;
  }

  for (unsigned int i = 1; i + 1 < path.poses.size(); ++i)
  {
    const geometry_msgs::PoseStamped &a = path.poses[i - 1];
    const geometry_msgs::PoseStamped &b = path.poses[i];
    const geometry_msgs::PoseStamped &c = path.poses[i + 1];
    const double ab = poseDistance(a, b);
    const double bc = poseDistance(b, c);
    const double ac = poseDistance(a, c);
    if (ab < 1e-4 || bc < 1e-4 || ac < 1e-4)
    {
      continue;
    }
    const double cross = std::fabs((b.pose.position.x - a.pose.position.x) *
                                       (c.pose.position.y - a.pose.position.y) -
                                   (b.pose.position.y - a.pose.position.y) *
                                       (c.pose.position.x - a.pose.position.x));
    const double curvature = 2.0 * cross / (ab * bc * ac);
    if (curvature > max_curvature_)
    {
      ROS_WARN("JGL reference path: curvature %.3f exceeds max %.3f at sample %u.",
               curvature, max_curvature_, i);
      return false;
    }
  }
  return true;
}

double TrajectoryGenerator::poseDistance(const geometry_msgs::PoseStamped &a,
                                         const geometry_msgs::PoseStamped &b) const
{
  return std::hypot(a.pose.position.x - b.pose.position.x,
                    a.pose.position.y - b.pose.position.y);
}

}  // namespace jgl_dwa_local_planner
