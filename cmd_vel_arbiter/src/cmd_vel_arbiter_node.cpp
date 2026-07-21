#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
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
    private_nh_.param("output_topic", output_topic_, std::string("/cmd_vel"));
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("switch_stop_cycles", switch_stop_cycles_, 1);

    publish_rate_ = std::max(1.0, publish_rate_);
    switch_stop_cycles_ = std::max(1, switch_stop_cycles_);

    LoadInput("safety", "/cmd_vel/safety", 100, 0.25,
              0.0, 0.0, 0.0, true);
    LoadInput("teleop", "/cmd_vel/teleop", 80, 0.25,
              0.35, 0.10, 1.10, false);
    LoadInput("tag", "/cmd_vel/tag", 60, 0.25,
              0.15, 0.15, 0.25, false);
    LoadInput("nav", "/cmd_vel/nav", 20, 0.25,
              0.30, 0.10, 4.10, false);

    output_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 1, false);
    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / publish_rate_),
        &CmdVelArbiter::TimerCallback, this);
    ROS_INFO("cmd_vel arbiter publishing %s at %.1f Hz",
             output_topic_.c_str(), publish_rate_);
  }

  ~CmdVelArbiter() {
    geometry_msgs::Twist zero;
    for (int i = 0; i < 3; ++i) {
      output_pub_.publish(zero);
    }
  }

 private:
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
    input->received_at = ros::WallTime::now();
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

  void TimerCallback(const ros::WallTimerEvent&) {
    const std::shared_ptr<Input> selected = SelectInput(ros::WallTime::now());
    const std::string selected_name = selected ? selected->name : std::string();

    if (selected_name != active_input_) {
      ROS_INFO("cmd_vel source changed: %s -> %s",
               active_input_.empty() ? "none" : active_input_.c_str(),
               selected_name.empty() ? "none" : selected_name.c_str());
      active_input_ = selected_name;
      stop_cycles_remaining_ = switch_stop_cycles_;
    }

    if (!selected || stop_cycles_remaining_ > 0) {
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
    output_pub_.publish(output);
  }

  void PublishZero() {
    geometry_msgs::Twist zero;
    output_pub_.publish(zero);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher output_pub_;
  ros::WallTimer timer_;
  std::vector<std::shared_ptr<Input> > inputs_;
  std::string output_topic_;
  std::string active_input_;
  double publish_rate_ = 50.0;
  int switch_stop_cycles_ = 1;
  int stop_cycles_remaining_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_arbiter");
  CmdVelArbiter arbiter;
  ros::spin();
  return 0;
}
