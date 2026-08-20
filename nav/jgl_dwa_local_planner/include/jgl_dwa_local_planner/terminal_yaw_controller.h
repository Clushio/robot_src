#ifndef JGL_DWA_LOCAL_PLANNER_TERMINAL_YAW_CONTROLLER_H_
#define JGL_DWA_LOCAL_PLANNER_TERMINAL_YAW_CONTROLLER_H_

#include <algorithm>
#include <cmath>

namespace jgl_dwa_local_planner
{

// Final-yaw controller used after the terminal XY position has been captured.
// It deliberately commands zero inside the tolerance and requires several
// consecutive safe samples before declaring completion, preventing a single
// noisy yaw sample from completing a loop leg.
class TerminalYawController
{
 public:
  TerminalYawController()
      : tolerance_(0.04),
        kp_(1.2),
        max_speed_(0.25),
        min_speed_(0.06),
        required_stable_cycles_(3),
        stable_cycles_(0)
  {
  }

  void configure(double tolerance,
                 double kp,
                 double max_speed,
                 double min_speed,
                 int required_stable_cycles)
  {
    tolerance_ = std::max(0.0, tolerance);
    kp_ = std::max(0.0, kp);
    max_speed_ = std::max(0.0, max_speed);
    min_speed_ = std::max(0.0, std::min(min_speed, max_speed_));
    required_stable_cycles_ = std::max(1, required_stable_cycles);
    reset();
  }

  void reset()
  {
    stable_cycles_ = 0;
  }

  // Updates the controller once per control cycle. Returns true only after
  // the yaw error has remained inside tolerance for the configured number of
  // consecutive cycles.
  bool update(double yaw_error, double *angular_command)
  {
    if (angular_command == NULL)
    {
      return false;
    }

    *angular_command = 0.0;
    if (!std::isfinite(yaw_error))
    {
      reset();
      return false;
    }

    if (std::fabs(yaw_error) <= tolerance_)
    {
      stable_cycles_ = std::min(stable_cycles_ + 1,
                                required_stable_cycles_);
      return stable_cycles_ >= required_stable_cycles_;
    }

    stable_cycles_ = 0;
    double command = kp_ * yaw_error;
    command = std::max(-max_speed_, std::min(max_speed_, command));
    if (std::fabs(command) < min_speed_ && max_speed_ > 0.0)
    {
      command = std::copysign(min_speed_, yaw_error);
    }
    *angular_command = command;
    return false;
  }

  int stableCycles() const
  {
    return stable_cycles_;
  }

 private:
  double tolerance_;
  double kp_;
  double max_speed_;
  double min_speed_;
  int required_stable_cycles_;
  int stable_cycles_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_TERMINAL_YAW_CONTROLLER_H_
