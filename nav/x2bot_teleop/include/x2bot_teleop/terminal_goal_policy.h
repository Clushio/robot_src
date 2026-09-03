#ifndef X2BOT_TELEOP__TERMINAL_GOAL_POLICY_H_
#define X2BOT_TELEOP__TERMINAL_GOAL_POLICY_H_

#include <cmath>
#include <cstdint>

namespace x2bot_teleop
{

enum TerminalMotionState : uint8_t
{
  TERMINAL_TRACKING = 0,
  TERMINAL_POSITION_CAPTURED = 1,
  TERMINAL_ROTATING = 2,
  TERMINAL_COMPLETE = 3
};

struct TerminalGoalUpdate
{
  bool locked = false;
  bool error_activated = false;
  bool error_recovered = false;
};

class TerminalGoalPolicy
{
public:
  TerminalGoalPolicy(double wait_timeout, double progress_yaw)
  : wait_timeout_(wait_timeout), progress_yaw_(progress_yaw) {}

  void configure(double wait_timeout, double progress_yaw)
  {
    wait_timeout_ = wait_timeout;
    progress_yaw_ = progress_yaw;
  }

  void reset()
  {
    locked_ = false;
    rotating_ = false;
    error_active_ = false;
    have_progress_yaw_ = false;
    last_progress_yaw_ = 0.0;
    last_progress_time_ = 0.0;
  }

  TerminalGoalUpdate update(uint8_t state, bool have_yaw, double yaw, double now)
  {
    TerminalGoalUpdate result;
    if (state == TERMINAL_POSITION_CAPTURED || state == TERMINAL_ROTATING ||
        state == TERMINAL_COMPLETE)
    {
      locked_ = true;
    }
    result.locked = locked_;

    if (state == TERMINAL_COMPLETE)
    {
      if (error_active_)
      {
        error_active_ = false;
        result.error_recovered = true;
      }
      return result;
    }

    if (state != TERMINAL_ROTATING || !have_yaw || !std::isfinite(yaw) ||
        !std::isfinite(now))
    {
      return result;
    }

    if (!rotating_ || !have_progress_yaw_)
    {
      rotating_ = true;
      have_progress_yaw_ = true;
      last_progress_yaw_ = yaw;
      last_progress_time_ = now;
      return result;
    }

    if (yawDistance(yaw, last_progress_yaw_) >= progress_yaw_)
    {
      last_progress_yaw_ = yaw;
      last_progress_time_ = now;
      if (error_active_)
      {
        error_active_ = false;
        result.error_recovered = true;
      }
      return result;
    }

    if (!error_active_ && wait_timeout_ > 0.0 &&
        now - last_progress_time_ >= wait_timeout_)
    {
      error_active_ = true;
      result.error_activated = true;
    }
    return result;
  }

  bool locked() const { return locked_; }
  bool errorActive() const { return error_active_; }

private:
  static double yawDistance(double a, double b)
  {
    double delta = a - b;
    constexpr double pi = 3.14159265358979323846;
    while (delta > pi) delta -= 2.0 * pi;
    while (delta < -pi) delta += 2.0 * pi;
    return std::fabs(delta);
  }

  double wait_timeout_;
  double progress_yaw_;
  bool locked_ = false;
  bool rotating_ = false;
  bool error_active_ = false;
  bool have_progress_yaw_ = false;
  double last_progress_yaw_ = 0.0;
  double last_progress_time_ = 0.0;
};

}  // namespace x2bot_teleop

#endif  // X2BOT_TELEOP__TERMINAL_GOAL_POLICY_H_
