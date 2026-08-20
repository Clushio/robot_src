#ifndef X2BOT_TELEOP_GOAL_ADVANCE_POLICY_H_
#define X2BOT_TELEOP_GOAL_ADVANCE_POLICY_H_

#include <cstddef>

namespace x2bot_teleop
{
namespace goal_advance_policy
{

inline bool IsPreciseControllerHandoff(std::size_t path_position,
                                       std::size_t path_size)
{
  return path_size >= 4 &&
         (path_position == 1 || path_position + 2 == path_size);
}

inline double PassThroughDistance(bool precise_controller_handoff,
                                  double waypoint_reached_distance,
                                  double controller_handoff_distance)
{
  return precise_controller_handoff ? controller_handoff_distance
                                    : waypoint_reached_distance;
}

inline bool ReferencePassedAllowsAdvance(bool precise_controller_handoff,
                                         bool pose_valid,
                                         double target_distance,
                                         double controller_handoff_distance)
{
  return !precise_controller_handoff ||
         (pose_valid && target_distance <= controller_handoff_distance);
}

inline bool DistanceAllowsAdvance(bool final_goal,
                                  bool precise_controller_handoff,
                                  bool bspline_tracking,
                                  double target_distance,
                                  double waypoint_reached_distance,
                                  double controller_handoff_distance)
{
  if (final_goal)
  {
    return false;
  }
  const double required_distance =
      PassThroughDistance(precise_controller_handoff,
                          waypoint_reached_distance,
                          controller_handoff_distance);
  if (target_distance > required_distance)
  {
    return false;
  }
  // An active B-spline owns ordinary waypoint progression through its
  // REFERENCE_PASSED status. Precise start/end handoffs use the direct
  // base_link distance even while B-spline tracking is active.
  return precise_controller_handoff || !bspline_tracking;
}

inline bool PassThroughActionSuccessNeedsValidation(
    bool final_goal,
    bool precise_controller_handoff,
    bool validate_normal_pass_through)
{
  return !final_goal &&
         (precise_controller_handoff || validate_normal_pass_through);
}

inline bool ValidatedActionSuccessAllowsAdvance(bool pose_valid,
                                                double target_distance,
                                                double required_distance)
{
  return pose_valid && target_distance <= required_distance;
}

}  // namespace goal_advance_policy
}  // namespace x2bot_teleop

#endif  // X2BOT_TELEOP_GOAL_ADVANCE_POLICY_H_
