#ifndef JGL_DWA_LOCAL_PLANNER_REFERENCE_OBSTACLE_POLICY_H_
#define JGL_DWA_LOCAL_PLANNER_REFERENCE_OBSTACLE_POLICY_H_

#include <cmath>

namespace jgl_dwa_local_planner
{

// Selects a candidate-speed cap from the arc-length clearance to the first
// blocked reference-path sample.  Collision Monitor remains responsible for
// validating the resulting command against the full robot footprint.
inline double referenceObstacleSpeedScale(double obstacle_distance,
                                          double slowdown_distance,
                                          double stop_distance)
{
  if (!std::isfinite(obstacle_distance) ||
      !std::isfinite(slowdown_distance) ||
      !std::isfinite(stop_distance) ||
      stop_distance < 0.0 || slowdown_distance <= stop_distance)
  {
    return 0.0;
  }
  if (obstacle_distance <= stop_distance)
  {
    return 0.0;
  }
  if (obstacle_distance >= slowdown_distance)
  {
    return 1.0;
  }

  const double normalized =
      (obstacle_distance - stop_distance) /
      (slowdown_distance - stop_distance);
  if (normalized >= 0.75)
  {
    return 0.75;
  }
  if (normalized >= 0.50)
  {
    return 0.50;
  }
  return 0.25;
}

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_REFERENCE_OBSTACLE_POLICY_H_
