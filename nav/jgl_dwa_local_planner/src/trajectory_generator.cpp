#include <jgl_dwa_local_planner/trajectory_generator.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace jgl_dwa_local_planner
{

TrajectoryGenerator::Point2d::Point2d() : x(0.0), y(0.0)
{
}

TrajectoryGenerator::Point2d::Point2d(double px, double py) : x(px), y(py)
{
}

TrajectoryGenerator::DistanceField::DistanceField()
    : valid(false),
      width(0),
      height(0),
      resolution(0.0),
      origin_x(0.0),
      origin_y(0.0)
{
}

int TrajectoryGenerator::DistanceField::index(int mx, int my) const
{
  return my * static_cast<int>(width) + mx;
}

TrajectoryGenerator::TrajectoryGenerator()
    : have_global_costmap_(false),
      sample_resolution_(0.10),
      safe_distance_(0.25),
      max_deviation_from_topo_(0.50),
      max_curvature_(2.1),
      min_turn_radius_(0.476),
      bspline_control_point_spacing_(0.40),
      bspline_opt_iterations_(60),
      bspline_weight_smooth_(1.0),
      bspline_weight_obstacle_(2.0),
      bspline_weight_topo_(0.6),
      bspline_weight_curvature_(1.0),
      occupied_threshold_(98),
      reference_curve_type_("bspline"),
      last_path_mode_(PATH_MODE_INVALID)
{
}

void TrajectoryGenerator::initialize(ros::NodeHandle &private_nh, ros::NodeHandle &node_nh)
{
  private_nh.param("bspline_sample_resolution", sample_resolution_, 0.10);
  private_nh.param("safe_distance", safe_distance_, 0.25);
  private_nh.param("max_deviation_from_topo", max_deviation_from_topo_, 0.50);
  private_nh.param("max_curvature", max_curvature_, 2.1);
  private_nh.param("reference_occupied_threshold", occupied_threshold_, 98);
  private_nh.param("reference_curve_type", reference_curve_type_, std::string("bspline"));
  private_nh.param("bspline_control_point_spacing", bspline_control_point_spacing_, 0.40);
  private_nh.param("bspline_opt_iterations", bspline_opt_iterations_, 60);
  private_nh.param("bspline_weight_smooth", bspline_weight_smooth_, 1.0);
  private_nh.param("bspline_weight_obstacle", bspline_weight_obstacle_, 2.0);
  private_nh.param("bspline_weight_topo", bspline_weight_topo_, 0.6);
  private_nh.param("bspline_weight_curvature", bspline_weight_curvature_, 1.0);
  private_nh.param("min_turn_radius", min_turn_radius_, 0.476);

  sample_resolution_ = std::max(0.02, sample_resolution_);
  safe_distance_ = std::max(0.0, safe_distance_);
  max_deviation_from_topo_ = std::max(0.0, max_deviation_from_topo_);
  max_curvature_ = std::max(0.0, max_curvature_);
  min_turn_radius_ = std::max(0.0, min_turn_radius_);
  bspline_control_point_spacing_ = std::max(0.10, bspline_control_point_spacing_);
  bspline_opt_iterations_ = std::max(0, bspline_opt_iterations_);
  bspline_weight_smooth_ = std::max(0.0, bspline_weight_smooth_);
  bspline_weight_obstacle_ = std::max(0.0, bspline_weight_obstacle_);
  bspline_weight_topo_ = std::max(0.0, bspline_weight_topo_);
  bspline_weight_curvature_ = std::max(0.0, bspline_weight_curvature_);
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
  const std::vector<geometry_msgs::PoseStamped> curve_waypoints =
      referenceCurveWaypoints(waypoints);
  if (curve_waypoints.size() < 2)
  {
    last_path_mode_ = PATH_MODE_INVALID;
    last_fallback_segments_.clear();
    return false;
  }

  if (reference_curve_type_ == "bspline")
  {
    nav_msgs::Path bspline_path;
    if (generateBsplineReference(curve_waypoints, bspline_path))
    {
      out_path = bspline_path;
      expandFallbackSegmentsForFullTopology(
          static_cast<unsigned int>(waypoints.size()), last_fallback_segments_);
      return true;
    }
    ROS_WARN("JGL reference path: optimized B-spline failed, falling back to cubic/hybrid/polyline.");

    nav_msgs::Path chunked_bspline_path;
    if (generateChunkedBsplineReference(curve_waypoints, chunked_bspline_path))
    {
      out_path = chunked_bspline_path;
      expandFallbackSegmentsForFullTopology(
          static_cast<unsigned int>(waypoints.size()), last_fallback_segments_);
      return true;
    }
    ROS_WARN("JGL reference path: chunked B-spline fallback failed, falling back to cubic/hybrid/polyline.");
  }

  const bool success = generateCubicReference(curve_waypoints, out_path);
  if (success)
  {
    expandFallbackSegmentsForFullTopology(
        static_cast<unsigned int>(waypoints.size()), last_fallback_segments_);
  }
  return success;
}

std::vector<geometry_msgs::PoseStamped> TrajectoryGenerator::referenceCurveWaypoints(
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  std::vector<geometry_msgs::PoseStamped> curve_waypoints;
  if (waypoints.size() < 4)
  {
    return curve_waypoints;
  }

  curve_waypoints.reserve(waypoints.size() - 2);
  for (unsigned int i = 1; i + 1 < waypoints.size(); ++i)
  {
    curve_waypoints.push_back(waypoints[i]);
  }
  return curve_waypoints;
}

void TrajectoryGenerator::expandFallbackSegmentsForFullTopology(
    unsigned int full_waypoint_count,
    const std::vector<int> &curve_fallback_segments)
{
  if (full_waypoint_count < 2)
  {
    last_fallback_segments_.clear();
    return;
  }

  std::vector<int> expanded(full_waypoint_count - 1, 0);
  for (unsigned int i = 0; i < curve_fallback_segments.size(); ++i)
  {
    const unsigned int original_segment_index = i + 1;
    if (original_segment_index < expanded.size())
    {
      expanded[original_segment_index] = curve_fallback_segments[i];
    }
  }
  last_fallback_segments_ = expanded;
}

bool TrajectoryGenerator::generateCubicReference(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    nav_msgs::Path &out_path)
{
  out_path.poses.clear();
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
    case PATH_MODE_BSPLINE:
      return "bspline";
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

bool TrajectoryGenerator::generateBsplineReference(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    nav_msgs::Path &out_path)
{
  out_path.poses.clear();

  nav_msgs::OccupancyGrid grid;
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    if (!have_global_costmap_)
    {
      ROS_WARN_THROTTLE(2.0, "JGL reference path: waiting for global costmap before B-spline optimization.");
      return false;
    }
    grid = global_costmap_;
  }

  DistanceField distance_field;
  if (!buildDistanceField(grid, distance_field))
  {
    ROS_WARN("JGL reference path: failed to build distance field for B-spline optimization.");
    return false;
  }

  unsigned int control_point_count = 0;
  nav_msgs::Path bspline_path;
  if (!buildOptimizedBsplinePath(waypoints,
                                 distance_field,
                                 bspline_path,
                                 &control_point_count))
  {
    return false;
  }

  if (!pathChecksPass(bspline_path, waypoints, true))
  {
    ROS_WARN("JGL reference path: optimized B-spline failed final safety checks.");
    return false;
  }

  out_path = bspline_path;
  last_path_mode_ = PATH_MODE_BSPLINE;
  last_fallback_segments_.assign(waypoints.size() - 1, 0);
  ROS_INFO("JGL reference path: generated optimized B-spline path with %zu samples and %u control points.",
           out_path.poses.size(), control_point_count);
  return true;
}

bool TrajectoryGenerator::buildOptimizedBsplinePath(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    const DistanceField &distance_field,
    nav_msgs::Path &out_path,
    unsigned int *control_point_count) const
{
  out_path.poses.clear();
  if (control_point_count != NULL)
  {
    *control_point_count = 0;
  }

  std::vector<Point2d> control_points = initializeBsplineControlPoints(waypoints);
  if (control_points.size() < 4)
  {
    return false;
  }

  if (!optimizeBsplineControlPoints(control_points, waypoints, distance_field))
  {
    return false;
  }

  out_path = sampleBsplinePath(control_points, waypoints);
  if (control_point_count != NULL)
  {
    *control_point_count = static_cast<unsigned int>(control_points.size());
  }
  return out_path.poses.size() >= 2;
}

bool TrajectoryGenerator::generateChunkedBsplineReference(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    nav_msgs::Path &out_path)
{
  out_path.poses.clear();
  if (waypoints.size() < 2)
  {
    return false;
  }

  nav_msgs::OccupancyGrid grid;
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    if (!have_global_costmap_)
    {
      ROS_WARN_THROTTLE(2.0, "JGL reference path: waiting for global costmap before chunked B-spline optimization.");
      return false;
    }
    grid = global_costmap_;
  }

  DistanceField distance_field;
  if (!buildDistanceField(grid, distance_field))
  {
    ROS_WARN("JGL reference path: failed to build distance field for chunked B-spline optimization.");
    return false;
  }

  nav_msgs::Path failed_full_path;
  buildOptimizedBsplinePath(waypoints, distance_field, failed_full_path, NULL);
  const std::vector<unsigned int> breakpoints =
      findBsplineBreakpoints(waypoints, failed_full_path, distance_field);

  nav_msgs::Path path;
  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();

  const unsigned int segment_count =
      static_cast<unsigned int>(waypoints.size() - 1);
  std::vector<int> fallback_segment_flags(segment_count, 0);
  unsigned int bspline_chunks = 0;
  unsigned int cubic_chunks = 0;
  unsigned int polyline_chunks = 0;
  unsigned int polyline_segments = 0;

  for (unsigned int i = 1; i + 1 < breakpoints.size(); ++i)
  {
    const unsigned int breakpoint = breakpoints[i];
    if (breakpoint > 0 && breakpoint - 1 < fallback_segment_flags.size())
    {
      fallback_segment_flags[breakpoint - 1] = 1;
    }
    if (breakpoint < fallback_segment_flags.size())
    {
      fallback_segment_flags[breakpoint] = 1;
    }
  }

  for (unsigned int chunk = 0; chunk + 1 < breakpoints.size(); ++chunk)
  {
    const unsigned int start_index = breakpoints[chunk];
    const unsigned int end_index = breakpoints[chunk + 1];
    if (start_index >= end_index || end_index >= waypoints.size())
    {
      continue;
    }

    nav_msgs::Path chunk_path;
    if (generateBsplineChunk(waypoints,
                             start_index,
                             end_index,
                             distance_field,
                             chunk_path))
    {
      ++bspline_chunks;
      appendPathSegment(path, chunk_path);
      continue;
    }

    std::vector<geometry_msgs::PoseStamped> chunk_waypoints;
    for (unsigned int i = start_index; i <= end_index; ++i)
    {
      chunk_waypoints.push_back(waypoints[i]);
    }

    if (chunk_waypoints.size() >= 3)
    {
      chunk_path = catmullRomPath(chunk_waypoints);
      if (pathChecksPass(chunk_path, waypoints, true))
      {
        ++cubic_chunks;
        ROS_WARN("JGL reference path: chunk [%u,%u] falls back from B-spline to cubic.",
                 start_index, end_index);
        appendPathSegment(path, chunk_path);
        continue;
      }
    }

    chunk_path = fallbackPolylinePath(chunk_waypoints);
    if (!pathChecksPass(chunk_path, waypoints, false))
    {
      ROS_ERROR("JGL reference path: chunk [%u,%u] failed B-spline, cubic, and collision-checked polyline.",
                start_index, end_index);
      last_path_mode_ = PATH_MODE_INVALID;
      last_fallback_segments_.clear();
      return false;
    }

    ++polyline_chunks;
    for (unsigned int i = start_index; i < end_index; ++i)
    {
      if (i < fallback_segment_flags.size())
      {
        fallback_segment_flags[i] = 1;
        ++polyline_segments;
      }
    }
    ROS_WARN("JGL reference path: chunk [%u,%u] falls back from B-spline/cubic to polyline.",
             start_index, end_index);
    appendPathSegment(path, chunk_path);
  }

  if (path.poses.empty())
  {
    last_path_mode_ = PATH_MODE_INVALID;
    last_fallback_segments_.clear();
    return false;
  }

  path.poses.front().pose.position.x = waypoints.front().pose.position.x;
  path.poses.front().pose.position.y = waypoints.front().pose.position.y;
  path.poses.back().pose.position.x = waypoints.back().pose.position.x;
  path.poses.back().pose.position.y = waypoints.back().pose.position.y;
  applyPathOrientations(path, waypoints.back());

  if (!pathChecksPass(path, waypoints, false))
  {
    ROS_WARN("JGL reference path: chunked B-spline path failed final collision/deviation checks.");
    last_path_mode_ = PATH_MODE_INVALID;
    last_fallback_segments_.clear();
    return false;
  }

  out_path = path;
  last_fallback_segments_ = fallback_segment_flags;
  last_path_mode_ =
      polyline_segments == segment_count ? PATH_MODE_POLYLINE_FALLBACK : PATH_MODE_HYBRID;
  ROS_WARN("JGL reference path: using chunked B-spline fallback with %zu breakpoints, %u B-spline chunks, %u cubic chunks, %u polyline chunks, %zu samples.",
           breakpoints.size(), bspline_chunks, cubic_chunks, polyline_chunks, out_path.poses.size());
  return true;
}

bool TrajectoryGenerator::generateBsplineChunk(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    unsigned int start_index,
    unsigned int end_index,
    const DistanceField &distance_field,
    nav_msgs::Path &out_path)
{
  out_path.poses.clear();
  if (start_index >= end_index || end_index >= waypoints.size())
  {
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> chunk_waypoints;
  chunk_waypoints.reserve(end_index - start_index + 1);
  for (unsigned int i = start_index; i <= end_index; ++i)
  {
    chunk_waypoints.push_back(waypoints[i]);
  }

  nav_msgs::Path chunk_path;
  if (!buildOptimizedBsplinePath(chunk_waypoints,
                                 distance_field,
                                 chunk_path,
                                 NULL))
  {
    return false;
  }

  if (!pathChecksPass(chunk_path, waypoints, true))
  {
    ROS_WARN("JGL reference path: chunk [%u,%u] B-spline failed checks.",
             start_index, end_index);
    return false;
  }

  out_path = chunk_path;
  return true;
}

std::vector<unsigned int> TrajectoryGenerator::findBsplineBreakpoints(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    const nav_msgs::Path &failed_path,
    const DistanceField &distance_field) const
{
  std::vector<unsigned int> breakpoints;
  if (waypoints.size() < 2)
  {
    return breakpoints;
  }

  const unsigned int last_index =
      static_cast<unsigned int>(waypoints.size() - 1);
  breakpoints.push_back(0);
  breakpoints.push_back(last_index);

  const double max_curvature = effectiveMaxCurvature();
  for (unsigned int i = 1; i + 1 < waypoints.size(); ++i)
  {
    const Point2d a(waypoints[i - 1].pose.position.x,
                    waypoints[i - 1].pose.position.y);
    const Point2d b(waypoints[i].pose.position.x,
                    waypoints[i].pose.position.y);
    const Point2d c(waypoints[i + 1].pose.position.x,
                    waypoints[i + 1].pose.position.y);
    if (max_curvature > 0.0 &&
        controlPolygonCurvature(a, b, c) > 0.85 * max_curvature)
    {
      breakpoints.push_back(i);
    }
  }

  for (unsigned int i = 0; i < failed_path.poses.size(); ++i)
  {
    bool bad_sample = false;

    double obstacle_distance = 0.0;
    const Point2d point(failed_path.poses[i].pose.position.x,
                        failed_path.poses[i].pose.position.y);
    if (!distanceAtPoint(distance_field, point, obstacle_distance) ||
        obstacle_distance <= safe_distance_ + 1e-9)
    {
      bad_sample = true;
    }

    if (!bad_sample && max_deviation_from_topo_ > 0.0 &&
        pointToPolylineDistance(failed_path.poses[i], waypoints) >
            max_deviation_from_topo_)
    {
      bad_sample = true;
    }

    if (!bad_sample && max_curvature > 0.0 &&
        pathSampleCurvature(failed_path, i) > max_curvature)
    {
      bad_sample = true;
    }

    if (!bad_sample)
    {
      continue;
    }

    const unsigned int segment =
        nearestTopoSegmentIndex(failed_path.poses[i], waypoints);
    if (segment > 0)
    {
      breakpoints.push_back(segment);
    }
    if (segment + 1 < last_index)
    {
      breakpoints.push_back(segment + 1);
    }
  }

  if (breakpoints.size() <= 2 && waypoints.size() > 2)
  {
    unsigned int best_index = 1;
    double best_curvature = -1.0;
    for (unsigned int i = 1; i + 1 < waypoints.size(); ++i)
    {
      const Point2d a(waypoints[i - 1].pose.position.x,
                      waypoints[i - 1].pose.position.y);
      const Point2d b(waypoints[i].pose.position.x,
                      waypoints[i].pose.position.y);
      const Point2d c(waypoints[i + 1].pose.position.x,
                      waypoints[i + 1].pose.position.y);
      const double curvature = controlPolygonCurvature(a, b, c);
      if (curvature > best_curvature)
      {
        best_curvature = curvature;
        best_index = i;
      }
    }
    breakpoints.push_back(best_index);
  }

  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()),
                    breakpoints.end());

  ROS_WARN("JGL reference path: chunked B-spline found %zu breakpoints.",
           breakpoints.size());
  for (unsigned int i = 0; i < breakpoints.size(); ++i)
  {
    ROS_DEBUG("JGL reference path: breakpoint[%u]=%u", i, breakpoints[i]);
  }

  return breakpoints;
}

unsigned int TrajectoryGenerator::nearestTopoSegmentIndex(
    const geometry_msgs::PoseStamped &pose,
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  if (waypoints.size() < 2)
  {
    return 0;
  }

  unsigned int best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    const double distance =
        pointToSegmentDistance(pose.pose.position.x,
                               pose.pose.position.y,
                               waypoints[i],
                               waypoints[i + 1]);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

double TrajectoryGenerator::pathSampleCurvature(
    const nav_msgs::Path &path,
    unsigned int sample_index) const
{
  if (sample_index == 0 || sample_index + 1 >= path.poses.size())
  {
    return 0.0;
  }

  const geometry_msgs::PoseStamped &a = path.poses[sample_index - 1];
  const geometry_msgs::PoseStamped &b = path.poses[sample_index];
  const geometry_msgs::PoseStamped &c = path.poses[sample_index + 1];
  const double ab = poseDistance(a, b);
  const double bc = poseDistance(b, c);
  const double ac = poseDistance(a, c);
  if (ab < 1e-4 || bc < 1e-4 || ac < 1e-4)
  {
    return 0.0;
  }

  const double cross = std::fabs(
      (b.pose.position.x - a.pose.position.x) *
          (c.pose.position.y - a.pose.position.y) -
      (b.pose.position.y - a.pose.position.y) *
          (c.pose.position.x - a.pose.position.x));
  return 2.0 * cross / (ab * bc * ac);
}

std::vector<TrajectoryGenerator::Point2d> TrajectoryGenerator::initializeBsplineControlPoints(
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  std::vector<Point2d> control_points;
  const double length = topoPolylineLength(waypoints);
  if (waypoints.size() < 2 || length < 1e-6)
  {
    return control_points;
  }

  const int control_count =
      std::max(6, static_cast<int>(std::ceil(length / bspline_control_point_spacing_)) + 1);
  control_points.reserve(control_count);
  for (int i = 0; i < control_count; ++i)
  {
    const double distance_along =
        length * static_cast<double>(i) / static_cast<double>(control_count - 1);
    control_points.push_back(interpolateTopoPolyline(waypoints, distance_along));
  }

  const Point2d start(waypoints.front().pose.position.x,
                      waypoints.front().pose.position.y);
  const Point2d end(waypoints.back().pose.position.x,
                    waypoints.back().pose.position.y);
  const Point2d start_tangent = waypointTangent(waypoints, true);
  const Point2d end_tangent = waypointTangent(waypoints, false);
  const double handle = std::min(bspline_control_point_spacing_, length / 3.0);

  control_points.front() = start;
  control_points.back() = end;
  control_points[1] = pointAdd(start, pointScale(start_tangent, handle));
  control_points[control_points.size() - 2] =
      pointSub(end, pointScale(end_tangent, handle));
  return control_points;
}

bool TrajectoryGenerator::optimizeBsplineControlPoints(
    std::vector<Point2d> &control_points,
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    const DistanceField &distance_field) const
{
  if (control_points.size() < 4 || !distance_field.valid)
  {
    return false;
  }

  const double topo_length = std::max(topoPolylineLength(waypoints), sample_resolution_);
  const int sample_count =
      std::max(8, static_cast<int>(std::ceil(topo_length / sample_resolution_)));
  const double obstacle_push_distance =
      safe_distance_ + std::max(distance_field.resolution, 0.02);
  const double max_curvature = effectiveMaxCurvature();
  const double max_step = std::min(0.06, 0.25 * bspline_control_point_spacing_);

  for (int iter = 0; iter < bspline_opt_iterations_; ++iter)
  {
    std::vector<Point2d> deltas(control_points.size(), Point2d());

    for (unsigned int i = 0; i < control_points.size(); ++i)
    {
      if (isFixedControlPoint(i, control_points.size()))
      {
        continue;
      }

      Point2d smooth_delta;
      if (i >= 2 && i + 2 < control_points.size())
      {
        const Point2d fourth = pointAdd(
            pointAdd(control_points[i - 2], pointScale(control_points[i - 1], -4.0)),
            pointAdd(pointScale(control_points[i], 6.0),
                     pointAdd(pointScale(control_points[i + 1], -4.0),
                              control_points[i + 2])));
        smooth_delta = pointScale(fourth, -0.010 * bspline_weight_smooth_);
      }
      else
      {
        const Point2d laplacian =
            pointSub(pointScale(pointAdd(control_points[i - 1], control_points[i + 1]), 0.5),
                     control_points[i]);
        smooth_delta = pointScale(laplacian, 0.040 * bspline_weight_smooth_);
      }
      deltas[i] = pointAdd(deltas[i], smooth_delta);
    }

    for (int sample = 0; sample <= sample_count; ++sample)
    {
      const double u = static_cast<double>(sample) / static_cast<double>(sample_count);
      Point2d point;
      std::vector<int> indices;
      std::vector<double> weights;
      if (!evaluateBspline(control_points, u, point) ||
          !bsplineBasisWeights(control_points.size(), u, indices, weights))
      {
        continue;
      }

      const Point2d topo_projection = projectPointToTopo(point, waypoints);
      const Point2d topo_delta = pointSub(topo_projection, point);
      const Point2d topo_step = pointScale(topo_delta, 0.020 * bspline_weight_topo_);

      for (unsigned int j = 0; j < indices.size(); ++j)
      {
        const int idx = indices[j];
        if (idx < 0 || idx >= static_cast<int>(control_points.size()) ||
            isFixedControlPoint(idx, control_points.size()))
        {
          continue;
        }
        deltas[idx] = pointAdd(deltas[idx], pointScale(topo_step, weights[j]));
      }

      double obstacle_distance = 0.0;
      Point2d obstacle_gradient;
      if (distanceAtPoint(distance_field, point, obstacle_distance) &&
          obstacle_distance < obstacle_push_distance &&
          distanceGradientAtPoint(distance_field, point, obstacle_gradient))
      {
        const double push =
            (obstacle_push_distance - obstacle_distance) * 0.060 * bspline_weight_obstacle_;
        const Point2d obstacle_step = pointScale(obstacle_gradient, push);
        for (unsigned int j = 0; j < indices.size(); ++j)
        {
          const int idx = indices[j];
          if (idx < 0 || idx >= static_cast<int>(control_points.size()) ||
              isFixedControlPoint(idx, control_points.size()))
          {
            continue;
          }
          deltas[idx] = pointAdd(deltas[idx], pointScale(obstacle_step, weights[j]));
        }
      }
    }

    if (max_curvature > 0.0)
    {
      for (unsigned int i = 1; i + 1 < control_points.size(); ++i)
      {
        if (isFixedControlPoint(i, control_points.size()))
        {
          continue;
        }
        const double curvature = controlPolygonCurvature(control_points[i - 1],
                                                         control_points[i],
                                                         control_points[i + 1]);
        if (curvature > max_curvature)
        {
          const Point2d midpoint =
              pointScale(pointAdd(control_points[i - 1], control_points[i + 1]), 0.5);
          const double scale =
              std::min(2.0, curvature / std::max(1e-6, max_curvature) - 1.0);
          const Point2d curvature_step =
              pointScale(pointSub(midpoint, control_points[i]),
                         0.050 * bspline_weight_curvature_ * scale);
          deltas[i] = pointAdd(deltas[i], curvature_step);
        }
      }
    }

    for (unsigned int i = 0; i < control_points.size(); ++i)
    {
      if (isFixedControlPoint(i, control_points.size()))
      {
        continue;
      }
      limitPointStep(deltas[i], max_step);
      const Point2d candidate = pointAdd(control_points[i], deltas[i]);
      double candidate_distance = 0.0;
      if (!distanceAtPoint(distance_field, candidate, candidate_distance))
      {
        continue;
      }
      control_points[i] = candidate;
    }
  }

  return true;
}

nav_msgs::Path TrajectoryGenerator::sampleBsplinePath(
    const std::vector<Point2d> &control_points,
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  nav_msgs::Path path;
  if (control_points.size() < 4 || waypoints.empty())
  {
    return path;
  }

  const double length = std::max(topoPolylineLength(waypoints), sample_resolution_);
  const int samples =
      std::max(1, static_cast<int>(std::ceil(length / sample_resolution_)));
  path.header = waypoints.front().header;
  path.header.stamp = ros::Time::now();
  path.poses.reserve(samples + 1);

  for (int i = 0; i <= samples; ++i)
  {
    const double u = static_cast<double>(i) / static_cast<double>(samples);
    Point2d point;
    if (!evaluateBspline(control_points, u, point))
    {
      continue;
    }
    path.poses.push_back(makePoseLike(waypoints.front(), point.x, point.y));
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

bool TrajectoryGenerator::evaluateBspline(
    const std::vector<Point2d> &control_points,
    double u,
    Point2d &point) const
{
  std::vector<int> indices;
  std::vector<double> weights;
  if (!bsplineBasisWeights(control_points.size(), u, indices, weights))
  {
    return false;
  }

  point = Point2d();
  for (unsigned int i = 0; i < indices.size(); ++i)
  {
    const int idx = indices[i];
    if (idx < 0 || idx >= static_cast<int>(control_points.size()))
    {
      return false;
    }
    point = pointAdd(point, pointScale(control_points[idx], weights[i]));
  }
  return true;
}

bool TrajectoryGenerator::bsplineBasisWeights(
    int control_point_count,
    double u,
    std::vector<int> &indices,
    std::vector<double> &weights) const
{
  indices.clear();
  weights.clear();
  const int degree = 3;
  if (control_point_count < degree + 1)
  {
    return false;
  }

  u = std::max(0.0, std::min(1.0, u));
  const int n = control_point_count - 1;
  std::vector<double> knots;
  buildClampedKnotVector(control_point_count, knots);

  int span = n;
  if (u < 1.0)
  {
    int low = degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (u < knots[mid] || u >= knots[mid + 1])
    {
      if (u < knots[mid])
      {
        high = mid;
      }
      else
      {
        low = mid;
      }
      mid = (low + high) / 2;
    }
    span = mid;
  }

  std::vector<double> basis(degree + 1, 0.0);
  std::vector<double> left(degree + 1, 0.0);
  std::vector<double> right(degree + 1, 0.0);
  basis[0] = 1.0;
  for (int j = 1; j <= degree; ++j)
  {
    left[j] = u - knots[span + 1 - j];
    right[j] = knots[span + j] - u;
    double saved = 0.0;
    for (int r = 0; r < j; ++r)
    {
      const double denom = right[r + 1] + left[j - r];
      const double temp = std::fabs(denom) > 1e-12 ? basis[r] / denom : 0.0;
      basis[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    basis[j] = saved;
  }

  for (int j = 0; j <= degree; ++j)
  {
    const int idx = span - degree + j;
    if (idx >= 0 && idx < control_point_count && std::fabs(basis[j]) > 1e-12)
    {
      indices.push_back(idx);
      weights.push_back(basis[j]);
    }
  }
  return !indices.empty();
}

void TrajectoryGenerator::buildClampedKnotVector(
    int control_point_count,
    std::vector<double> &knots) const
{
  const int degree = 3;
  const int n = control_point_count - 1;
  const int knot_count = control_point_count + degree + 1;
  knots.assign(knot_count, 0.0);
  for (int i = 0; i < knot_count; ++i)
  {
    if (i <= degree)
    {
      knots[i] = 0.0;
    }
    else if (i >= control_point_count)
    {
      knots[i] = 1.0;
    }
    else
    {
      knots[i] = static_cast<double>(i - degree) /
                 static_cast<double>(n - degree + 1);
    }
  }
}

bool TrajectoryGenerator::buildDistanceField(
    const nav_msgs::OccupancyGrid &grid,
    DistanceField &distance_field) const
{
  if (grid.info.width == 0 || grid.info.height == 0 ||
      grid.data.size() != grid.info.width * grid.info.height)
  {
    return false;
  }

  distance_field.valid = true;
  distance_field.width = grid.info.width;
  distance_field.height = grid.info.height;
  distance_field.resolution = std::max(1e-6, static_cast<double>(grid.info.resolution));
  distance_field.origin_x = grid.info.origin.position.x;
  distance_field.origin_y = grid.info.origin.position.y;
  distance_field.distance.assign(grid.data.size(),
                                 std::numeric_limits<double>::infinity());

  typedef std::pair<double, int> QueueItem;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem> > queue;

  for (unsigned int my = 0; my < grid.info.height; ++my)
  {
    for (unsigned int mx = 0; mx < grid.info.width; ++mx)
    {
      const int index = my * grid.info.width + mx;
      const int value = grid.data[index];
      if (value < 0 || value >= occupied_threshold_)
      {
        distance_field.distance[index] = 0.0;
        queue.push(QueueItem(0.0, index));
      }
    }
  }

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!queue.empty())
  {
    const QueueItem item = queue.top();
    queue.pop();
    if (item.first > distance_field.distance[item.second] + 1e-9)
    {
      continue;
    }

    const int mx = item.second % static_cast<int>(grid.info.width);
    const int my = item.second / static_cast<int>(grid.info.width);
    for (int k = 0; k < 8; ++k)
    {
      const int nx = mx + dx[k];
      const int ny = my + dy[k];
      if (nx < 0 || ny < 0 ||
          nx >= static_cast<int>(grid.info.width) ||
          ny >= static_cast<int>(grid.info.height))
      {
        continue;
      }

      const double step = (dx[k] == 0 || dy[k] == 0) ? distance_field.resolution
                                                     : distance_field.resolution * std::sqrt(2.0);
      const int next_index = distance_field.index(nx, ny);
      const double next_distance = item.first + step;
      if (next_distance + 1e-9 < distance_field.distance[next_index])
      {
        distance_field.distance[next_index] = next_distance;
        queue.push(QueueItem(next_distance, next_index));
      }
    }
  }

  return true;
}

bool TrajectoryGenerator::distanceAtPoint(
    const DistanceField &distance_field,
    const Point2d &point,
    double &distance) const
{
  if (!distance_field.valid || distance_field.resolution <= 0.0)
  {
    return false;
  }
  const int mx = static_cast<int>(std::floor((point.x - distance_field.origin_x) /
                                            distance_field.resolution));
  const int my = static_cast<int>(std::floor((point.y - distance_field.origin_y) /
                                            distance_field.resolution));
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(distance_field.width) ||
      my >= static_cast<int>(distance_field.height))
  {
    return false;
  }

  distance = distance_field.distance[distance_field.index(mx, my)];
  if (!std::isfinite(distance))
  {
    distance = 1e6;
  }
  return true;
}

bool TrajectoryGenerator::distanceGradientAtPoint(
    const DistanceField &distance_field,
    const Point2d &point,
    Point2d &gradient) const
{
  if (!distance_field.valid || distance_field.resolution <= 0.0)
  {
    return false;
  }
  const int mx = static_cast<int>(std::floor((point.x - distance_field.origin_x) /
                                            distance_field.resolution));
  const int my = static_cast<int>(std::floor((point.y - distance_field.origin_y) /
                                            distance_field.resolution));
  if (mx <= 0 || my <= 0 ||
      mx + 1 >= static_cast<int>(distance_field.width) ||
      my + 1 >= static_cast<int>(distance_field.height))
  {
    return false;
  }

  double left = distance_field.distance[distance_field.index(mx - 1, my)];
  double right = distance_field.distance[distance_field.index(mx + 1, my)];
  double down = distance_field.distance[distance_field.index(mx, my - 1)];
  double up = distance_field.distance[distance_field.index(mx, my + 1)];
  if (!std::isfinite(left) || !std::isfinite(right) ||
      !std::isfinite(down) || !std::isfinite(up))
  {
    return false;
  }

  gradient.x = (right - left) / (2.0 * distance_field.resolution);
  gradient.y = (up - down) / (2.0 * distance_field.resolution);
  const double norm = pointNorm(gradient);
  if (norm < 1e-6)
  {
    return false;
  }
  gradient = pointScale(gradient, 1.0 / norm);
  return true;
}

TrajectoryGenerator::Point2d TrajectoryGenerator::projectPointToTopo(
    const Point2d &point,
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  if (waypoints.empty())
  {
    return point;
  }
  if (waypoints.size() == 1)
  {
    return Point2d(waypoints.front().pose.position.x, waypoints.front().pose.position.y);
  }

  Point2d best(waypoints.front().pose.position.x, waypoints.front().pose.position.y);
  double best_distance = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    const Point2d a(waypoints[i].pose.position.x, waypoints[i].pose.position.y);
    const Point2d b(waypoints[i + 1].pose.position.x, waypoints[i + 1].pose.position.y);
    const Point2d ab = pointSub(b, a);
    const double len_sq = ab.x * ab.x + ab.y * ab.y;
    double t = 0.0;
    if (len_sq > 1e-9)
    {
      const Point2d ap = pointSub(point, a);
      t = (ap.x * ab.x + ap.y * ab.y) / len_sq;
      t = std::max(0.0, std::min(1.0, t));
    }
    const Point2d projection = pointAdd(a, pointScale(ab, t));
    const double distance = pointDistance(point, projection);
    if (distance < best_distance)
    {
      best_distance = distance;
      best = projection;
    }
  }
  return best;
}

TrajectoryGenerator::Point2d TrajectoryGenerator::interpolateTopoPolyline(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    double distance_along) const
{
  if (waypoints.empty())
  {
    return Point2d();
  }
  if (distance_along <= 0.0 || waypoints.size() == 1)
  {
    return Point2d(waypoints.front().pose.position.x, waypoints.front().pose.position.y);
  }

  double walked = 0.0;
  for (unsigned int i = 0; i + 1 < waypoints.size(); ++i)
  {
    const double segment_length = poseDistance(waypoints[i], waypoints[i + 1]);
    if (walked + segment_length >= distance_along && segment_length > 1e-9)
    {
      const double ratio = (distance_along - walked) / segment_length;
      const double x = waypoints[i].pose.position.x +
                       ratio * (waypoints[i + 1].pose.position.x -
                                waypoints[i].pose.position.x);
      const double y = waypoints[i].pose.position.y +
                       ratio * (waypoints[i + 1].pose.position.y -
                                waypoints[i].pose.position.y);
      return Point2d(x, y);
    }
    walked += segment_length;
  }

  return Point2d(waypoints.back().pose.position.x, waypoints.back().pose.position.y);
}

double TrajectoryGenerator::topoPolylineLength(
    const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  double length = 0.0;
  for (unsigned int i = 1; i < waypoints.size(); ++i)
  {
    length += poseDistance(waypoints[i - 1], waypoints[i]);
  }
  return length;
}

TrajectoryGenerator::Point2d TrajectoryGenerator::waypointTangent(
    const std::vector<geometry_msgs::PoseStamped> &waypoints,
    bool start) const
{
  if (waypoints.size() < 2)
  {
    return Point2d(1.0, 0.0);
  }

  if (start)
  {
    const Point2d origin(waypoints.front().pose.position.x,
                         waypoints.front().pose.position.y);
    for (unsigned int i = 1; i < waypoints.size(); ++i)
    {
      Point2d tangent = pointSub(Point2d(waypoints[i].pose.position.x,
                                         waypoints[i].pose.position.y),
                                 origin);
      const double norm = pointNorm(tangent);
      if (norm > 1e-6)
      {
        return pointScale(tangent, 1.0 / norm);
      }
    }
  }
  else
  {
    const Point2d end(waypoints.back().pose.position.x,
                      waypoints.back().pose.position.y);
    for (int i = static_cast<int>(waypoints.size()) - 2; i >= 0; --i)
    {
      Point2d tangent = pointSub(end,
                                 Point2d(waypoints[i].pose.position.x,
                                         waypoints[i].pose.position.y));
      const double norm = pointNorm(tangent);
      if (norm > 1e-6)
      {
        return pointScale(tangent, 1.0 / norm);
      }
    }
  }
  return Point2d(1.0, 0.0);
}

double TrajectoryGenerator::effectiveMaxCurvature() const
{
  double limit = max_curvature_;
  if (min_turn_radius_ > 1e-6)
  {
    const double radius_limit = 1.0 / min_turn_radius_;
    limit = limit > 0.0 ? std::min(limit, radius_limit) : radius_limit;
  }
  return limit;
}

double TrajectoryGenerator::controlPolygonCurvature(
    const Point2d &a,
    const Point2d &b,
    const Point2d &c) const
{
  const double ab = pointDistance(a, b);
  const double bc = pointDistance(b, c);
  const double ac = pointDistance(a, c);
  if (ab < 1e-4 || bc < 1e-4 || ac < 1e-4)
  {
    return 0.0;
  }
  const double cross = std::fabs((b.x - a.x) * (c.y - a.y) -
                                 (b.y - a.y) * (c.x - a.x));
  return 2.0 * cross / (ab * bc * ac);
}

void TrajectoryGenerator::limitPointStep(Point2d &delta, double max_step) const
{
  const double norm = pointNorm(delta);
  if (norm > max_step && norm > 1e-9)
  {
    delta = pointScale(delta, max_step / norm);
  }
}

bool TrajectoryGenerator::isFixedControlPoint(unsigned int index, unsigned int count) const
{
  if (count <= 4)
  {
    return true;
  }
  return index <= 1 || index + 2 >= count;
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
  const double max_curvature = effectiveMaxCurvature();
  if (max_curvature <= 0.0 || path.poses.size() < 3)
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
    if (curvature > max_curvature)
    {
      ROS_WARN("JGL reference path: curvature %.3f exceeds max %.3f at sample %u.",
               curvature, max_curvature, i);
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

double TrajectoryGenerator::pointDistance(const Point2d &a, const Point2d &b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double TrajectoryGenerator::pointNorm(const Point2d &point) const
{
  return std::hypot(point.x, point.y);
}

TrajectoryGenerator::Point2d TrajectoryGenerator::pointAdd(
    const Point2d &a,
    const Point2d &b) const
{
  return Point2d(a.x + b.x, a.y + b.y);
}

TrajectoryGenerator::Point2d TrajectoryGenerator::pointSub(
    const Point2d &a,
    const Point2d &b) const
{
  return Point2d(a.x - b.x, a.y - b.y);
}

TrajectoryGenerator::Point2d TrajectoryGenerator::pointScale(
    const Point2d &point,
    double scale) const
{
  return Point2d(point.x * scale, point.y * scale);
}

}  // namespace jgl_dwa_local_planner
