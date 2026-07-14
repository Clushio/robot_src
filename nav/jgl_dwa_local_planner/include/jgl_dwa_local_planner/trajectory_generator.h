#ifndef JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
#define JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <vector>

namespace jgl_dwa_local_planner
{

class TrajectoryGenerator
{
public:
  enum PathMode
  {
    PATH_MODE_INVALID = 0,
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

private:
  void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

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

  ros::Subscriber global_costmap_sub_;
  mutable boost::mutex costmap_mutex_;
  nav_msgs::OccupancyGrid global_costmap_;
  bool have_global_costmap_;

  double sample_resolution_;
  double safe_distance_;
  double max_deviation_from_topo_;
  double max_curvature_;
  int occupied_threshold_;
  PathMode last_path_mode_;
  std::vector<int> last_fallback_segments_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
