#ifndef JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
#define JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

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

  void initialize(ros::NodeHandle &private_nh, ros::NodeHandle &node_nh);

  bool generate(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                nav_msgs::Path &out_path);
  bool checkCollision(const nav_msgs::Path &path);
  bool checkDeviationFromTopo(const nav_msgs::Path &path,
                              const std::vector<geometry_msgs::PoseStamped> &waypoints);
  bool checkCurvature(const nav_msgs::Path &path);
  nav_msgs::Path fallbackPolylinePath(const std::vector<geometry_msgs::PoseStamped> &waypoints);

  double sampleResolution() const { return sample_resolution_; }
  double safeDistance() const { return safe_distance_; }
  bool lastPathWasFallback() const { return last_path_mode_ == PATH_MODE_POLYLINE_FALLBACK; }
  PathMode lastPathMode() const { return last_path_mode_; }
  const char *lastPathModeName() const;
  std::vector<int> lastFallbackSegments() const { return last_fallback_segments_; }

#ifdef JGL_DWA_LOCAL_PLANNER_ENABLE_TEST_ACCESS
  void setGlobalCostmapForTesting(const nav_msgs::OccupancyGrid &grid)
  {
    boost::mutex::scoped_lock lock(costmap_mutex_);
    global_costmap_ = grid;
    have_global_costmap_ = true;
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

	  void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	  bool generateBsplineReference(const std::vector<geometry_msgs::PoseStamped> &waypoints,
	                                nav_msgs::Path &out_path);
	  bool generateCubicReference(const std::vector<geometry_msgs::PoseStamped> &waypoints,
	                              nav_msgs::Path &out_path);
	  std::vector<Point2d> initializeBsplineControlPoints(
	      const std::vector<geometry_msgs::PoseStamped> &waypoints) const;
	  bool optimizeBsplineControlPoints(
	      std::vector<Point2d> &control_points,
	      const std::vector<geometry_msgs::PoseStamped> &waypoints,
	      const DistanceField &distance_field) const;
	  nav_msgs::Path sampleBsplinePath(
	      const std::vector<Point2d> &control_points,
	      const std::vector<geometry_msgs::PoseStamped> &waypoints) const;
	  bool evaluateBspline(const std::vector<Point2d> &control_points,
	                       double u,
	                       Point2d &point) const;
	  bool bsplineBasisWeights(int control_point_count,
	                           double u,
	                           std::vector<int> &indices,
	                           std::vector<double> &weights) const;
	  void buildClampedKnotVector(int control_point_count,
	                              std::vector<double> &knots) const;
	  bool buildDistanceField(const nav_msgs::OccupancyGrid &grid,
	                          DistanceField &distance_field) const;
	  bool distanceAtPoint(const DistanceField &distance_field,
	                       const Point2d &point,
	                       double &distance) const;
	  bool distanceGradientAtPoint(const DistanceField &distance_field,
	                               const Point2d &point,
	                               Point2d &gradient) const;
	  Point2d projectPointToTopo(
	      const Point2d &point,
	      const std::vector<geometry_msgs::PoseStamped> &waypoints) const;
	  Point2d interpolateTopoPolyline(
	      const std::vector<geometry_msgs::PoseStamped> &waypoints,
	      double distance_along) const;
	  double topoPolylineLength(
	      const std::vector<geometry_msgs::PoseStamped> &waypoints) const;
	  Point2d waypointTangent(
	      const std::vector<geometry_msgs::PoseStamped> &waypoints,
	      bool start) const;
	  double effectiveMaxCurvature() const;
	  double controlPolygonCurvature(const Point2d &a,
	                                 const Point2d &b,
	                                 const Point2d &c) const;
	  void limitPointStep(Point2d &delta, double max_step) const;
	  bool isFixedControlPoint(unsigned int index, unsigned int count) const;

	  nav_msgs::Path catmullRomPath(const std::vector<geometry_msgs::PoseStamped> &waypoints);
	  nav_msgs::Path catmullRomSegment(
	      const std::vector<geometry_msgs::PoseStamped> &waypoints,
      unsigned int segment_index) const;
  nav_msgs::Path hybridPath(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                            unsigned int *fallback_segments,
                            unsigned int *total_segments,
                            std::vector<int> *fallback_segment_flags);
  nav_msgs::Path fallbackPolylineSegment(const geometry_msgs::PoseStamped &start,
                                         const geometry_msgs::PoseStamped &end) const;
  void appendPathSegment(nav_msgs::Path &path, const nav_msgs::Path &segment) const;
  void applyPathOrientations(nav_msgs::Path &path,
                             const geometry_msgs::PoseStamped &final_pose) const;
  geometry_msgs::PoseStamped makePoseLike(const geometry_msgs::PoseStamped &reference,
                                          double x, double y) const;

  bool pathChecksPass(const nav_msgs::Path &path,
                      const std::vector<geometry_msgs::PoseStamped> &waypoints,
                      bool check_curvature);
  bool poseCollides(const geometry_msgs::PoseStamped &pose,
                    const nav_msgs::OccupancyGrid &grid) const;
  bool worldToMap(const nav_msgs::OccupancyGrid &grid, double wx, double wy,
                  int &mx, int &my) const;
  bool occupiedCell(const nav_msgs::OccupancyGrid &grid, int mx, int my) const;

  double pointToPolylineDistance(const geometry_msgs::PoseStamped &pose,
                                 const std::vector<geometry_msgs::PoseStamped> &waypoints) const;
  double pointToSegmentDistance(double px, double py,
                                const geometry_msgs::PoseStamped &a,
                                const geometry_msgs::PoseStamped &b) const;
	  double poseDistance(const geometry_msgs::PoseStamped &a,
	                      const geometry_msgs::PoseStamped &b) const;
	  double pointDistance(const Point2d &a, const Point2d &b) const;
	  double pointNorm(const Point2d &point) const;
	  Point2d pointAdd(const Point2d &a, const Point2d &b) const;
	  Point2d pointSub(const Point2d &a, const Point2d &b) const;
	  Point2d pointScale(const Point2d &point, double scale) const;

	  ros::Subscriber global_costmap_sub_;
	  mutable boost::mutex costmap_mutex_;
  nav_msgs::OccupancyGrid global_costmap_;
  bool have_global_costmap_;

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
