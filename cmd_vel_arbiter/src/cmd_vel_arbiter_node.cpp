#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <actionlib/client/simple_action_client.h>
#include <cmd_vel_arbiter/FinishMotion.h>
#include <cmd_vel_arbiter/ArbitratedCommand.h>
#include <geometry_msgs/Twist.h>
#include <ranger_msgs/StopAndCenterAction.h>
#include <ros/ros.h>

namespace {

double Clamp(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

bool IsFinite(const geometry_msgs::Twist& msg) {
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

bool IsZero(const geometry_msgs::Twist& msg) {
  constexpr double kEpsilon = 1e-9;
  return std::abs(msg.linear.x) <= kEpsilon &&
         std::abs(msg.linear.y) <= kEpsilon &&
         std::abs(msg.linear.z) <= kEpsilon &&
         std::abs(msg.angular.x) <= kEpsilon &&
         std::abs(msg.angular.y) <= kEpsilon &&
         std::abs(msg.angular.z) <= kEpsilon;
}

class CmdVelArbiter {
 public:
  CmdVelArbiter() : private_nh_("~") {
    private_nh_.param("output_topic", output_topic_,
                      std::string("/cmd_vel/candidate"));
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("switch_stop_cycles", switch_stop_cycles_, 1);
    private_nh_.param("stop_and_center_action", stop_and_center_action_,
                      std::string("/stop_and_center"));
    private_nh_.param("stop_and_center_wait_timeout",
                      stop_and_center_wait_timeout_, 7.0);
    private_nh_.param("finish_source_suppression",
                      finish_source_suppression_, 0.5);
    private_nh_.param("center_on_nav_loss", center_on_nav_loss_, true);
    private_nh_.param("center_on_tag_loss", center_on_tag_loss_, true);

    publish_rate_ = std::max(1.0, publish_rate_);
    switch_stop_cycles_ = std::max(1, switch_stop_cycles_);
    stop_and_center_wait_timeout_ =
        std::max(0.5, stop_and_center_wait_timeout_);
    finish_source_suppression_ = std::max(0.0, finish_source_suppression_);

    LoadInput("safety", "/cmd_vel/safety", 100, 0.25,
              0.0, 0.0, 0.0, true);
    LoadInput("teleop", "/cmd_vel/teleop", 80, 0.25,
              0.35, 0.10, 1.10, false);
    LoadInput("tag", "/cmd_vel/tag", 60, 0.25,
              0.15, 0.15, 0.25, false);
    LoadInput("nav", "/cmd_vel/nav", 20, 0.25,
              0.30, 0.10, 4.10, false);

    output_pub_ =
        nh_.advertise<cmd_vel_arbiter::ArbitratedCommand>(output_topic_, 1,
                                                          false);
    stop_center_client_.reset(
        new StopCenterClient(stop_and_center_action_, true));
    finish_motion_server_ = private_nh_.advertiseService(
        "finish_motion", &CmdVelArbiter::FinishMotionCallback, this);
    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / publish_rate_),
        &CmdVelArbiter::TimerCallback, this);
    ROS_INFO("cmd_vel arbiter publishing %s at %.1f Hz",
             output_topic_.c_str(), publish_rate_);
    ROS_INFO("stop-and-center action=%s, finish service=%s",
             stop_and_center_action_.c_str(),
             private_nh_.resolveName("finish_motion").c_str());
  }

  ~CmdVelArbiter() {
    geometry_msgs::Twist zero;
    for (int i = 0; i < 3; ++i) {
      PublishCommand(std::string(), zero);
    }
  }

 private:
  typedef actionlib::SimpleActionClient<ranger_msgs::StopAndCenterAction>
      StopCenterClient;

  struct Input {
    std::string name;
    std::string topic;
    int priority = 0;
    double timeout = 0.25;
    double max_linear_x = 0.0;
    double max_linear_y = 0.0;
    double max_angular_z = 0.0;
    bool safety_only = false;
    bool received = false;
    geometry_msgs::Twist command;
    ros::WallTime received_at;
    ros::WallTime ignored_until;
    ros::Subscriber subscriber;
  };

  void LoadInput(const std::string& name, const std::string& default_topic,
                 int default_priority, double default_timeout,
                 double default_max_linear_x, double default_max_linear_y,
                 double default_max_angular_z, bool safety_only) {
    std::shared_ptr<Input> input(new Input());
    input->name = name;
    input->safety_only = safety_only;
    private_nh_.param(name + "_topic", input->topic, default_topic);
    private_nh_.param(name + "_priority", input->priority, default_priority);
    private_nh_.param(name + "_timeout", input->timeout, default_timeout);
    private_nh_.param(name + "_max_linear_x", input->max_linear_x,
                      default_max_linear_x);
    private_nh_.param(name + "_max_linear_y", input->max_linear_y,
                      default_max_linear_y);
    private_nh_.param(name + "_max_angular_z", input->max_angular_z,
                      default_max_angular_z);

    input->timeout = std::max(0.02, input->timeout);
    input->max_linear_x = std::max(0.0, input->max_linear_x);
    input->max_linear_y = std::max(0.0, input->max_linear_y);
    input->max_angular_z = std::max(0.0, input->max_angular_z);
    input->subscriber = nh_.subscribe<geometry_msgs::Twist>(
        input->topic, 1,
        boost::bind(&CmdVelArbiter::InputCallback, this, _1, input));
    inputs_.push_back(input);
    ROS_INFO(
        "cmd_vel input %-7s topic=%s priority=%d timeout=%.3f s "
        "limits=(%.3f, %.3f, %.3f)",
        name.c_str(), input->topic.c_str(), input->priority, input->timeout,
        input->max_linear_x, input->max_linear_y, input->max_angular_z);
  }

  void InputCallback(const geometry_msgs::Twist::ConstPtr& msg,
                     const std::shared_ptr<Input>& input) {
    const ros::WallTime now = ros::WallTime::now();
    if (!input->ignored_until.isZero() && now < input->ignored_until) {
      return;
    }

    geometry_msgs::Twist command = *msg;
    if (!IsFinite(command)) {
      ROS_ERROR_THROTTLE(1.0, "Rejected non-finite velocity from %s",
                         input->topic.c_str());
      if (!input->safety_only) {
        input->received = false;
        return;
      }
      command = geometry_msgs::Twist();
    }

    if (input->safety_only && !IsZero(command)) {
      ROS_ERROR_THROTTLE(
          1.0, "Non-zero command on safety-only input %s was forced to zero",
          input->topic.c_str());
      command = geometry_msgs::Twist();
    }

    input->command = command;
    input->received_at = now;
    input->received = true;
  }

  std::shared_ptr<Input> SelectInput(const ros::WallTime& now) const {
    std::shared_ptr<Input> selected;
    for (const std::shared_ptr<Input>& input : inputs_) {
      if (!input->received ||
          (now - input->received_at).toSec() > input->timeout) {
        continue;
      }
      if (!selected || input->priority > selected->priority ||
          (input->priority == selected->priority &&
           input->received_at > selected->received_at)) {
        selected = input;
      }
    }
    return selected;
  }

  std::shared_ptr<Input> FindInput(const std::string& name) const {
    for (const std::shared_ptr<Input>& input : inputs_) {
      if (input->name == name) {
        return input;
      }
    }
    return std::shared_ptr<Input>();
  }

  bool ShouldCenterOnLoss(const std::string& source) const {
    return (source == "nav" && center_on_nav_loss_) ||
           (source == "tag" && center_on_tag_loss_);
  }

  void StopCenterDone(
      const actionlib::SimpleClientGoalState& state,
      const ranger_msgs::StopAndCenterResultConstPtr& result) {
    const bool success =
        state == actionlib::SimpleClientGoalState::SUCCEEDED && result &&
        result->success;
    last_center_success_.store(success);
    centering_active_.store(false);
    if (success) {
      ROS_INFO("stop-and-center completed: %s", result->message.c_str());
    } else {
      ROS_ERROR("stop-and-center failed: state=%s message=%s",
                state.toString().c_str(),
                result ? result->message.c_str() : "no result");
    }
  }

  bool RequestStopAndCenter(uint8_t reason) {
    if (centering_active_.load()) {
      return true;
    }
    if (!stop_center_client_ ||
        !stop_center_client_->waitForServer(ros::Duration(0.2))) {
      ROS_ERROR_THROTTLE(1.0, "stop-and-center action server %s unavailable",
                         stop_and_center_action_.c_str());
      return false;
    }

    ranger_msgs::StopAndCenterGoal goal;
    goal.reason = reason;
    centering_active_.store(true);
    last_center_success_.store(false);
    motion_since_center_ = false;
    stop_center_client_->sendGoal(
        goal, boost::bind(&CmdVelArbiter::StopCenterDone, this, _1, _2));
    return true;
  }

  bool WaitForStopAndCenter() {
    if (!stop_center_client_) {
      return false;
    }

    // FinishMotionCallback runs on the arbiter's single ROS callback thread.
    // Keep the zero-speed heartbeat alive while the action client's private
    // thread receives the stop-and-center result; otherwise Ranger's cmd_vel
    // watchdog fires after 0.25 s and restarts the same centering operation.
    const ros::WallTime deadline =
        ros::WallTime::now() +
        ros::WallDuration(stop_and_center_wait_timeout_);
    ros::WallRate zero_rate(publish_rate_);
    while (ros::ok() &&
           !stop_center_client_->getState().isDone() &&
           ros::WallTime::now() < deadline) {
      PublishZero();
      zero_rate.sleep();
    }
    PublishZero();

    if (!stop_center_client_->getState().isDone()) {
      ROS_ERROR("stop-and-center did not finish within %.1f seconds",
                stop_and_center_wait_timeout_);
      return false;
    }
    const ranger_msgs::StopAndCenterResultConstPtr result =
        stop_center_client_->getResult();
    return result && result->success;
  }

  bool FinishMotionCallback(cmd_vel_arbiter::FinishMotion::Request& req,
                            cmd_vel_arbiter::FinishMotion::Response& res) {
    const std::shared_ptr<Input> input = FindInput(req.source);
    if (!input) {
      res.accepted = false;
      res.centered = false;
      res.message = "unknown motion source: " + req.source;
      return true;
    }

    const std::shared_ptr<Input> selected =
        SelectInput(ros::WallTime::now());
    const bool owns_control =
        active_input_ == req.source ||
        (selected && selected->name == req.source);
    res.accepted = true;

    const bool force_safety_stop =
        req.reason == cmd_vel_arbiter::FinishMotionRequest::SOFTWARE_ESTOP;
    if (!owns_control && !centering_active_.load() && !force_safety_stop) {
      res.centered = false;
      res.message = "source no longer owns motion control; no centering issued";
      return true;
    }

    const ros::WallTime now = ros::WallTime::now();
    if (force_safety_stop) {
      input->command = geometry_msgs::Twist();
      input->received = true;
      input->received_at = now;
      active_input_ = input->name;
    } else {
      input->received = false;
      input->ignored_until =
          now + ros::WallDuration(finish_source_suppression_);
      active_input_.clear();
    }
    stop_cycles_remaining_ = switch_stop_cycles_;
    PublishZero();

    if (!RequestStopAndCenter(req.reason)) {
      res.centered = false;
      res.message = "stop-and-center action server unavailable";
      return true;
    }

    res.centered = WaitForStopAndCenter();
    res.message = res.centered
                      ? "vehicle stopped and steering centered"
                      : "vehicle stopped, but steering centering failed";
    return true;
  }

  void TimerCallback(const ros::WallTimerEvent&) {
    const std::shared_ptr<Input> selected = SelectInput(ros::WallTime::now());
    const std::string selected_name = selected ? selected->name : std::string();

    if (selected_name != active_input_) {
      const std::string previous_name = active_input_;
      ROS_INFO("cmd_vel source changed: %s -> %s",
               previous_name.empty() ? "none" : previous_name.c_str(),
               selected_name.empty() ? "none" : selected_name.c_str());
      active_input_ = selected_name;
      stop_cycles_remaining_ = switch_stop_cycles_;

      if (selected_name == "safety") {
        RequestStopAndCenter(
            ranger_msgs::StopAndCenterGoal::SOFTWARE_ESTOP);
      } else if (selected_name.empty() &&
                 ShouldCenterOnLoss(previous_name) &&
                 motion_since_center_) {
        ROS_WARN("motion source %s disappeared; stop and center",
                 previous_name.c_str());
        RequestStopAndCenter(ranger_msgs::StopAndCenterGoal::CMD_TIMEOUT);
      }
    }

    if (centering_active_.load() || !selected ||
        stop_cycles_remaining_ > 0) {
      PublishZero();
      if (stop_cycles_remaining_ > 0) {
        --stop_cycles_remaining_;
      }
      return;
    }

    geometry_msgs::Twist output;
    output.linear.x =
        Clamp(selected->command.linear.x, selected->max_linear_x);
    output.linear.y =
        Clamp(selected->command.linear.y, selected->max_linear_y);
    output.angular.z =
        Clamp(selected->command.angular.z, selected->max_angular_z);
    if (!IsZero(output)) {
      motion_since_center_ = true;
    }
    PublishCommand(selected->name, output);
  }

  void PublishZero() {
    geometry_msgs::Twist zero;
    PublishCommand(active_input_, zero);
  }

  void PublishCommand(const std::string& source,
                      const geometry_msgs::Twist& command) {
    cmd_vel_arbiter::ArbitratedCommand output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "base_link";
    output.source = source;
    output.command = command;
    output_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher output_pub_;
  ros::WallTimer timer_;
  ros::ServiceServer finish_motion_server_;
  std::vector<std::shared_ptr<Input> > inputs_;
  std::unique_ptr<StopCenterClient> stop_center_client_;
  std::string output_topic_;
  std::string stop_and_center_action_;
  std::string active_input_;
  double publish_rate_ = 50.0;
  double stop_and_center_wait_timeout_ = 7.0;
  double finish_source_suppression_ = 0.5;
  int switch_stop_cycles_ = 1;
  int stop_cycles_remaining_ = 0;
  bool center_on_nav_loss_ = true;
  bool center_on_tag_loss_ = true;
  bool motion_since_center_ = false;
  std::atomic<bool> centering_active_{false};
  std::atomic<bool> last_center_success_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_arbiter");
  CmdVelArbiter arbiter;
  ros::spin();
  return 0;
}
