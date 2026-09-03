#ifndef JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
#define JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>
#include <vector>

namespace jgl_dwa_local_planner
{

class TrajectoryGenerator
{
public:
	  enum PathMode
	  {
	    PATH_MODE_INVALID = 0,
	    PATH_MODE_BSPLINE,
	    PATH_MODE_CUBIC,
	    PATH_MODE_HYBRID,
	    PATH_MODE_POLYLINE_FALLBACK
	  };

  TrajectoryGenerator();

  void initialize(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      const std::string &parameter_prefix);

  bool generate(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                nav_msgs::msg::Path &out_path,
                bool validate_local_snapshot = false,
                const nav_msgs::msg::OccupancyGrid *local_snapshot = nullptr);
  bool checkCollision(const nav_msgs::msg::Path &path);
  bool checkDeviationFromTopo(const nav_msgs::msg::Path &path,
                              const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
  bool checkCurvature(const nav_msgs::msg::Path &path);
  nav_msgs::msg::Path fallbackPolylinePath(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);

  double sampleResolution() const { return sample_resolution_; }
  double safeDistance() const { return safe_distance_; }
  bool lastPathWasFallback() const { return last_path_mode_ == PATH_MODE_POLYLINE_FALLBACK; }
  PathMode lastPathMode() const { return last_path_mode_; }
  const char *lastPathModeName() const;
  std::vector<int> lastFallbackSegments() const { return last_fallback_segments_; }

#ifdef JGL_DWA_LOCAL_PLANNER_ENABLE_TEST_ACCESS
  void setGlobalCostmapForTesting(const nav_msgs::msg::OccupancyGrid &grid)
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    global_costmap_ = grid;
    have_global_costmap_ = true;
  }

  void setLocalCostmapForTesting(const nav_msgs::msg::OccupancyGrid &grid)
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    local_costmap_ = grid;
    have_local_costmap_ = true;
  }

  void setReferenceLimitsForTesting(double sample_resolution,
                                    double safe_distance,
                                    double max_deviation,
                                    double max_curvature,
                                    double min_turn_radius)
  {
    sample_resolution_ = sample_resolution;
    safe_distance_ = safe_distance;
    max_deviation_from_topo_ = max_deviation;
    max_curvature_ = max_curvature;
    min_turn_radius_ = min_turn_radius;
  }

  void setBsplineOptimizationForTesting(double control_point_spacing,
                                        int iterations,
                                        double weight_smooth,
                                        double weight_obstacle,
                                        double weight_topo,
                                        double weight_curvature)
  {
    bspline_control_point_spacing_ = control_point_spacing;
    bspline_opt_iterations_ = iterations;
    bspline_weight_smooth_ = weight_smooth;
    bspline_weight_obstacle_ = weight_obstacle;
    bspline_weight_topo_ = weight_topo;
    bspline_weight_curvature_ = weight_curvature;
  }
#endif

private:
	  struct Point2d
	  {
	    double x;
	    double y;

	    Point2d();
	    Point2d(double px, double py);
	  };

	  struct DistanceField
	  {
	    DistanceField();

	    int index(int mx, int my) const;
	    bool valid;
	    unsigned int width;
	    unsigned int height;
	    double resolution;
	    double origin_x;
	    double origin_y;
	    std::vector<double> distance;
	  };

	  void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
	  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
	  bool poseCollidesLocal(const geometry_msgs::msg::PoseStamped &pose,
	                         const nav_msgs::msg::OccupancyGrid &grid) const;

	  bool generateBsplineReference(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	                                nav_msgs::msg::Path &out_path);
	  bool buildOptimizedBsplinePath(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      const DistanceField &distance_field,
	      nav_msgs::msg::Path &out_path,
	      unsigned int *control_point_count) const;
	  bool generateChunkedBsplineReference(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      nav_msgs::msg::Path &out_path);
	  bool generateBsplineChunk(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      unsigned int start_index,
	      unsigned int end_index,
	      const DistanceField &distance_field,
	      nav_msgs::msg::Path &out_path);
	  std::vector<unsigned int> findBsplineBreakpoints(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      const nav_msgs::msg::Path &failed_path,
	      const DistanceField &distance_field) const;
	  unsigned int nearestTopoSegmentIndex(
	      const geometry_msgs::msg::PoseStamped &pose,
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  double pathSampleCurvature(const nav_msgs::msg::Path &path,
	                             unsigned int sample_index) const;
	  bool generateCubicReference(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	                              nav_msgs::msg::Path &out_path);
	  std::vector<geometry_msgs::msg::PoseStamped> referenceCurveWaypoints(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  void expandFallbackSegmentsForFullTopology(
	      unsigned int full_waypoint_count,
	      const std::vector<int> &curve_fallback_segments);
	  std::vector<Point2d> initializeBsplineControlPoints(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  bool optimizeBsplineControlPoints(
	      std::vector<Point2d> &control_points,
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      const DistanceField &distance_field) const;
	  double bsplineOptimizationCost(
	      const std::vector<Point2d> &control_points,
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      const DistanceField &distance_field) const;
	  nav_msgs::msg::Path sampleBsplinePath(
	      const std::vector<Point2d> &control_points,
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  bool evaluateBspline(const std::vector<Point2d> &control_points,
	                       double u,
	                       Point2d &point) const;
	  bool bsplineBasisWeights(int control_point_count,
	                           double u,
	                           std::vector<int> &indices,
	                           std::vector<double> &weights) const;
	  void buildClampedKnotVector(int control_point_count,
	                              std::vector<double> &knots) const;
	  bool buildDistanceField(const nav_msgs::msg::OccupancyGrid &grid,
	                          DistanceField &distance_field) const;
	  bool distanceAtPoint(const DistanceField &distance_field,
	                       const Point2d &point,
	                       double &distance) const;
	  bool distanceGradientAtPoint(const DistanceField &distance_field,
	                               const Point2d &point,
	                               Point2d &gradient) const;
	  Point2d projectPointToTopo(
	      const Point2d &point,
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  Point2d interpolateTopoPolyline(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      double distance_along) const;
	  double topoPolylineLength(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
	  Point2d waypointTangent(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
	      bool start) const;
	  double effectiveMaxCurvature() const;
	  double controlPolygonCurvature(const Point2d &a,
	                                 const Point2d &b,
	                                 const Point2d &c) const;
	  void limitPointStep(Point2d &delta, double max_step) const;
	  bool isFixedControlPoint(unsigned int index, unsigned int count) const;

	  nav_msgs::msg::Path catmullRomPath(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
	  nav_msgs::msg::Path catmullRomSegment(
	      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
      unsigned int segment_index) const;
  nav_msgs::msg::Path hybridPath(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                            unsigned int *fallback_segments,
                            unsigned int *total_segments,
                            std::vector<int> *fallback_segment_flags);
  nav_msgs::msg::Path fallbackPolylineSegment(const geometry_msgs::msg::PoseStamped &start,
                                         const geometry_msgs::msg::PoseStamped &end) const;
  void appendPathSegment(nav_msgs::msg::Path &path, const nav_msgs::msg::Path &segment) const;
  void applyPathOrientations(nav_msgs::msg::Path &path,
                             const geometry_msgs::msg::PoseStamped &final_pose) const;
  geometry_msgs::msg::PoseStamped makePoseLike(const geometry_msgs::msg::PoseStamped &reference,
                                          double x, double y) const;

  bool pathChecksPass(const nav_msgs::msg::Path &path,
                      const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                      bool check_curvature);
  bool poseCollides(const geometry_msgs::msg::PoseStamped &pose,
                    const nav_msgs::msg::OccupancyGrid &grid) const;
  bool worldToMap(const nav_msgs::msg::OccupancyGrid &grid, double wx, double wy,
                  int &mx, int &my) const;
  bool occupiedCell(const nav_msgs::msg::OccupancyGrid &grid, int mx, int my) const;

  double pointToPolylineDistance(const geometry_msgs::msg::PoseStamped &pose,
                                 const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) const;
  double pointToSegmentDistance(double px, double py,
                                const geometry_msgs::msg::PoseStamped &a,
                                const geometry_msgs::msg::PoseStamped &b) const;
	  double poseDistance(const geometry_msgs::msg::PoseStamped &a,
	                      const geometry_msgs::msg::PoseStamped &b) const;
	  double pointDistance(const Point2d &a, const Point2d &b) const;
	  double pointNorm(const Point2d &point) const;
	  Point2d pointAdd(const Point2d &a, const Point2d &b) const;
	  Point2d pointSub(const Point2d &a, const Point2d &b) const;
	  Point2d pointScale(const Point2d &point, double scale) const;

	  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
	  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
	  rclcpp::Clock::SharedPtr clock_;
	  rclcpp::Logger logger_{rclcpp::get_logger("TrajectoryGenerator")};
	  mutable boost::mutex costmap_mutex_;
  nav_msgs::msg::OccupancyGrid global_costmap_;
  bool have_global_costmap_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
  nav_msgs::msg::OccupancyGrid generation_local_costmap_;
  bool have_local_costmap_;
  bool validate_local_snapshot_;

  double sample_resolution_;
	  double safe_distance_;
	  double max_deviation_from_topo_;
	  double max_curvature_;
	  double min_turn_radius_;
	  double bspline_control_point_spacing_;
	  int bspline_opt_iterations_;
	  double bspline_weight_smooth_;
	  double bspline_weight_obstacle_;
	  double bspline_weight_topo_;
	  double bspline_weight_curvature_;
	  int occupied_threshold_;
	  std::string reference_curve_type_;
	  PathMode last_path_mode_;
	  std::vector<int> last_fallback_segments_;
	};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
