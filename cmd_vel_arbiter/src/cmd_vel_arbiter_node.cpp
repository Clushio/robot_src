#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "cmd_vel_arbiter/msg/arbitrated_command.hpp"
#include "cmd_vel_arbiter/srv/finish_motion.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ranger_msgs/action/stop_and_center.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

void AddDiagnosticValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

double Clamp(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}

bool IsFinite(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

bool IsZero(const geometry_msgs::msg::Twist & msg)
{
  constexpr double kEpsilon = 1e-9;
  return std::abs(msg.linear.x) <= kEpsilon &&
         std::abs(msg.linear.y) <= kEpsilon &&
         std::abs(msg.linear.z) <= kEpsilon &&
         std::abs(msg.angular.x) <= kEpsilon &&
         std::abs(msg.angular.y) <= kEpsilon &&
         std::abs(msg.angular.z) <= kEpsilon;
}

class CmdVelArbiter : public rclcpp::Node
{
public:
  CmdVelArbiter()
  : Node("cmd_vel_arbiter")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/cmd_vel/candidate");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    diagnostic_rate_ = declare_parameter<double>("diagnostic_rate", 1.0);
    switch_stop_cycles_ = declare_parameter<int>("switch_stop_cycles", 1);
    stop_and_center_action_ = declare_parameter<std::string>(
      "stop_and_center_action", "/stop_and_center");
    stop_and_center_wait_timeout_ = declare_parameter<double>(
      "stop_and_center_wait_timeout", 7.0);
    finish_source_suppression_ = declare_parameter<double>(
      "finish_source_suppression", 0.5);
    center_on_nav_loss_ = declare_parameter<bool>("center_on_nav_loss", true);
    center_on_tag_loss_ = declare_parameter<bool>("center_on_tag_loss", true);

    publish_rate_ = std::max(1.0, publish_rate_);
    diagnostic_rate_ = std::max(0.1, diagnostic_rate_);
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

    output_pub_ = create_publisher<cmd_vel_arbiter::msg::ArbitratedCommand>(
      output_topic_, 1);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // FinishMotion waits synchronously while publishing a zero-speed heartbeat.
    // Keep action result callbacks in a separate group so they can complete.
    action_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    stop_center_client_ = rclcpp_action::create_client<StopAndCenter>(
      this, stop_and_center_action_, action_callback_group_);

    finish_motion_server_ = create_service<cmd_vel_arbiter::srv::FinishMotion>(
      "~/finish_motion",
      std::bind(
        &CmdVelArbiter::FinishMotionCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&CmdVelArbiter::TimerCallback, this));

    RCLCPP_INFO(
      get_logger(), "cmd_vel arbiter publishing %s at %.1f Hz",
      output_topic_.c_str(), publish_rate_);
    RCLCPP_INFO(
      get_logger(), "cmd_vel arbiter diagnostics heartbeat at %.1f Hz",
      diagnostic_rate_);
    RCLCPP_INFO(
      get_logger(), "stop-and-center action=%s, finish service=%s",
      stop_and_center_action_.c_str(),
      finish_motion_server_->get_service_name());
  }

  ~CmdVelArbiter() override
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      PublishCommand(std::string(), zero);
    }
  }

private:
  using StopAndCenter = ranger_msgs::action::StopAndCenter;
  using StopCenterGoalHandle = rclcpp_action::ClientGoalHandle<StopAndCenter>;

  struct Input
  {
    std::string name;
    std::string topic;
    int priority = 0;
    double timeout = 0.25;
    double max_linear_x = 0.0;
    double max_linear_y = 0.0;
    double max_angular_z = 0.0;
    bool safety_only = false;
    bool received = false;
    geometry_msgs::msg::Twist command;
    TimePoint received_at{};
    TimePoint ignored_until{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
  };

  void LoadInput(
    const std::string & name, const std::string & default_topic,
    int default_priority, double default_timeout,
    double default_max_linear_x, double default_max_linear_y,
    double default_max_angular_z, bool safety_only)
  {
    auto input = std::make_shared<Input>();
    input->name = name;
    input->safety_only = safety_only;
    input->topic = declare_parameter<std::string>(name + "_topic", default_topic);
    input->priority = declare_parameter<int>(name + "_priority", default_priority);
    input->timeout = declare_parameter<double>(name + "_timeout", default_timeout);
    input->max_linear_x = declare_parameter<double>(
      name + "_max_linear_x", default_max_linear_x);
    input->max_linear_y = declare_parameter<double>(
      name + "_max_linear_y", default_max_linear_y);
    input->max_angular_z = declare_parameter<double>(
      name + "_max_angular_z", default_max_angular_z);

    input->timeout = std::max(0.02, input->timeout);
    input->max_linear_x = std::max(0.0, input->max_linear_x);
    input->max_linear_y = std::max(0.0, input->max_linear_y);
    input->max_angular_z = std::max(0.0, input->max_angular_z);
    const std::weak_ptr<Input> weak_input = input;
    input->subscriber = create_subscription<geometry_msgs::msg::Twist>(
      input->topic, 1,
      [this, weak_input](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (const auto input = weak_input.lock()) {
          InputCallback(msg, input);
        }
      });
    inputs_.push_back(input);
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel input %-7s topic=%s priority=%d timeout=%.3f s "
      "limits=(%.3f, %.3f, %.3f)",
      name.c_str(), input->topic.c_str(), input->priority, input->timeout,
      input->max_linear_x, input->max_linear_y, input->max_angular_z);
  }

  void InputCallback(
    geometry_msgs::msg::Twist::ConstSharedPtr msg,
    const std::shared_ptr<Input> & input)
  {
    const TimePoint now = SteadyClock::now();
    if (input->ignored_until != TimePoint{} && now < input->ignored_until) {
      return;
    }

    geometry_msgs::msg::Twist command = *msg;
    if (!IsFinite(command)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected non-finite velocity from %s", input->topic.c_str());
      if (!input->safety_only) {
        SetDiagnosticFault(
          "ANAV-ARB-005", "速度指令包含 NaN 或无穷值",
          "检查发布该速度的节点：" + input->topic,
          diagnostic_msgs::msg::DiagnosticStatus::ERROR);
        input->received = false;
        return;
      }
      command = geometry_msgs::msg::Twist();
    }

    if (input->safety_only && !IsZero(command)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Non-zero command on safety-only input %s was forced to zero",
        input->topic.c_str());
      command = geometry_msgs::msg::Twist();
      SetDiagnosticFault(
        "ANAV-ARB-006", "安全通道收到非零速度，已强制置零",
        "检查 /cmd_vel/safety 的发布节点。",
        diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }

    input->command = command;
    input->received_at = now;
    input->received = true;
  }

  std::shared_ptr<Input> SelectInput(const TimePoint & now) const
  {
    std::shared_ptr<Input> selected;
    for (const auto & input : inputs_) {
      const double age = std::chrono::duration<double>(
        now - input->received_at).count();
      if (!input->received || age > input->timeout) {
        continue;
      }
      if (!selected || input->priority > selected->priority ||
        (input->priority == selected->priority &&
        input->received_at > selected->received_at))
      {
        selected = input;
      }
    }
    return selected;
  }

  std::shared_ptr<Input> FindInput(const std::string & name) const
  {
    for (const auto & input : inputs_) {
      if (input->name == name) {
        return input;
      }
    }
    return nullptr;
  }

  bool ShouldCenterOnLoss(const std::string & source) const
  {
    return (source == "nav" && center_on_nav_loss_) ||
           (source == "tag" && center_on_tag_loss_);
  }

  void CompleteActionWait(bool success, const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      action_done_ = true;
      action_success_ = success;
      action_message_ = message;
    }
    last_center_success_.store(success);
    centering_active_.store(false);
    action_cv_.notify_all();

    if (success) {
      RCLCPP_INFO(get_logger(), "stop-and-center completed: %s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "stop-and-center failed: %s", message.c_str());
      SetDiagnosticFault(
        "ANAV-ARB-013", "轮组停车回正失败",
        message.empty() ? "action 未返回结果" : message,
        diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }
  }

  bool RequestStopAndCenter(uint8_t reason)
  {
    if (centering_active_.load()) {
      return true;
    }
    if (!stop_center_client_ || !stop_center_client_->wait_for_action_server(200ms)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "stop-and-center action server %s unavailable",
        stop_and_center_action_.c_str());
      SetDiagnosticFault(
        "ANAV-ARB-011", "停车回正服务不可用",
        "检查 ranger_base 的 /stop_and_center action。",
        diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      action_done_ = false;
      action_success_ = false;
      action_message_.clear();
    }
    centering_active_.store(true);
    last_center_success_.store(false);
    motion_since_center_ = false;

    StopAndCenter::Goal goal;
    goal.reason = reason;
    rclcpp_action::Client<StopAndCenter>::SendGoalOptions options;
    options.goal_response_callback =
      [this](StopCenterGoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          CompleteActionWait(false, "stop-and-center goal was rejected");
        }
      };
    options.result_callback =
      [this](const StopCenterGoalHandle::WrappedResult & wrapped_result) {
        const bool success =
          wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped_result.result && wrapped_result.result->success;
        const std::string message = wrapped_result.result ?
          wrapped_result.result->message : "action returned no result";
        CompleteActionWait(success, message);
      };
    stop_center_client_->async_send_goal(goal, options);
    return true;
  }

  bool WaitForStopAndCenter()
  {
    const TimePoint deadline = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(stop_and_center_wait_timeout_));
    const auto zero_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / publish_rate_));

    std::unique_lock<std::mutex> lock(action_mutex_);
    while (rclcpp::ok() && !action_done_ && SteadyClock::now() < deadline) {
      action_cv_.wait_for(lock, zero_period);
      lock.unlock();
      PublishZero();
      lock.lock();
    }
    const bool done = action_done_;
    const bool success = action_success_;
    lock.unlock();
    PublishZero();

    if (!done) {
      RCLCPP_ERROR(
        get_logger(), "stop-and-center did not finish within %.1f seconds",
        stop_and_center_wait_timeout_);
      SetDiagnosticFault(
        "ANAV-ARB-012", "停车回正等待超时",
        "检查底盘反馈、轮组模式和转向执行机构。",
        diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return false;
    }
    return success;
  }

  void FinishMotionCallback(
    const std::shared_ptr<cmd_vel_arbiter::srv::FinishMotion::Request> request,
    std::shared_ptr<cmd_vel_arbiter::srv::FinishMotion::Response> response)
  {
    const auto input = FindInput(request->source);
    if (!input) {
      response->accepted = false;
      response->centered = false;
      response->message = "unknown motion source: " + request->source;
      return;
    }

    const auto selected = SelectInput(SteadyClock::now());
    const bool owns_control = active_input_ == request->source ||
      (selected && selected->name == request->source);
    response->accepted = true;

    const bool force_safety_stop =
      request->reason ==
      cmd_vel_arbiter::srv::FinishMotion::Request::SOFTWARE_ESTOP;
    if (!owns_control && !centering_active_.load() && !force_safety_stop) {
      response->centered = false;
      response->message =
        "source no longer owns motion control; no centering issued";
      return;
    }

    const TimePoint now = SteadyClock::now();
    if (force_safety_stop) {
      input->command = geometry_msgs::msg::Twist();
      input->received = true;
      input->received_at = now;
      active_input_ = input->name;
    } else {
      input->received = false;
      input->ignored_until = now +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(finish_source_suppression_));
      active_input_.clear();
    }
    stop_cycles_remaining_ = switch_stop_cycles_;
    PublishZero();

    if (!RequestStopAndCenter(request->reason)) {
      response->centered = false;
      response->message = "stop-and-center action server unavailable";
      return;
    }

    response->centered = WaitForStopAndCenter();
    response->message = response->centered ?
      "vehicle stopped and steering centered" :
      "vehicle stopped, but steering centering failed";
  }

  void TimerCallback()
  {
    const auto selected = SelectInput(SteadyClock::now());
    const std::string selected_name = selected ? selected->name : std::string();

    if (selected_name != active_input_) {
      const std::string previous_name = active_input_;
      RCLCPP_INFO(
        get_logger(), "cmd_vel source changed: %s -> %s",
        previous_name.empty() ? "none" : previous_name.c_str(),
        selected_name.empty() ? "none" : selected_name.c_str());
      active_input_ = selected_name;
      stop_cycles_remaining_ = switch_stop_cycles_;

      if (selected_name == "safety") {
        SetDiagnosticFault(
          "ANAV-ARB-008", "安全速度通道正在接管",
          "确认现场安全后再解除紧急停止。",
          diagnostic_msgs::msg::DiagnosticStatus::WARN, 1.0);
        RequestStopAndCenter(StopAndCenter::Goal::SOFTWARE_ESTOP);
      } else if (selected_name.empty() &&
        ShouldCenterOnLoss(previous_name) && motion_since_center_)
      {
        RCLCPP_WARN(
          get_logger(), "motion source %s disappeared; stop and center",
          previous_name.c_str());
        SetDiagnosticFault(
          previous_name == "tag" ? "ANAV-ARB-003" : "ANAV-ARB-002",
          "运动速度来源超时，已停车并回正",
          "检查对应控制节点及速度话题是否持续发布。",
          diagnostic_msgs::msg::DiagnosticStatus::WARN);
        RequestStopAndCenter(StopAndCenter::Goal::CMD_TIMEOUT);
      }
    }

    if (centering_active_.load() || !selected || stop_cycles_remaining_ > 0) {
      PublishZero();
      if (stop_cycles_remaining_ > 0) {
        --stop_cycles_remaining_;
      }
      return;
    }

    geometry_msgs::msg::Twist output;
    output.linear.x = Clamp(selected->command.linear.x, selected->max_linear_x);
    output.linear.y = Clamp(selected->command.linear.y, selected->max_linear_y);
    output.angular.z = Clamp(selected->command.angular.z, selected->max_angular_z);
    if (output.linear.x != selected->command.linear.x ||
      output.linear.y != selected->command.linear.y ||
      output.angular.z != selected->command.angular.z)
    {
      SetDiagnosticFault(
        "ANAV-ARB-007", "速度指令超过限制，已限幅",
        "检查速度参数是否与底盘能力一致。",
        diagnostic_msgs::msg::DiagnosticStatus::WARN, 1.0);
    }
    if (!IsZero(output)) {
      motion_since_center_ = true;
    }
    PublishCommand(selected->name, output);
  }

  void PublishZero()
  {
    geometry_msgs::msg::Twist zero;
    PublishCommand(active_input_, zero);
  }

  void PublishCommand(
    const std::string & source, const geometry_msgs::msg::Twist & command)
  {
    cmd_vel_arbiter::msg::ArbitratedCommand output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.source = source;
    output.command = command;
    output_pub_->publish(output);
    PublishDiagnostic(source);
  }

  void SetDiagnosticFault(
    const std::string & code, const std::string & message,
    const std::string & action, uint8_t level, double hold_seconds = 3.0)
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_code_ = code;
    diagnostic_message_ = message;
    diagnostic_action_ = action;
    diagnostic_level_ = level;
    diagnostic_until_ = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(hold_seconds));
  }

  void PublishDiagnostic(const std::string & source)
  {
    const TimePoint now_steady = SteadyClock::now();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/anav/cmd_vel_arbiter";
    status.hardware_id = "ranger";
    std::string code;
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      if (diagnostic_until_ != TimePoint{} && now_steady < diagnostic_until_) {
        status.level = diagnostic_level_;
        status.message = diagnostic_message_;
        code = diagnostic_code_;
        AddDiagnosticValue(status, "code", code);
        AddDiagnosticValue(status, "action", diagnostic_action_);
        AddDiagnosticValue(status, "active", "true");
        AddDiagnosticValue(status, "kind", "FAULT");
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "速度仲裁运行正常";
        code = "ANAV-ARB-000";
        AddDiagnosticValue(status, "code", code);
        AddDiagnosticValue(status, "active", "false");
        AddDiagnosticValue(status, "kind", "STATE");
      }
    }
    const std::string state = centering_active_.load() ? "CENTERING" : "RUNNING";
    const std::string source_name = source.empty() ? "none" : source;
    AddDiagnosticValue(status, "state", state);
    AddDiagnosticValue(status, "source", source_name);

    const std::string signature = std::to_string(status.level) + "|" + code +
      "|" + state + "|" + source_name;
    const bool state_changed = signature != last_diagnostic_signature_;
    const bool heartbeat_due = last_diagnostic_publish_ == TimePoint{} ||
      std::chrono::duration<double>(
      now_steady - last_diagnostic_publish_).count() >= 1.0 / diagnostic_rate_;
    if (!state_changed && !heartbeat_due) {
      return;
    }
    last_diagnostic_signature_ = signature;
    last_diagnostic_publish_ = now_steady;
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::vector<std::shared_ptr<Input>> inputs_;
  rclcpp::Publisher<cmd_vel_arbiter::msg::ArbitratedCommand>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<cmd_vel_arbiter::srv::FinishMotion>::SharedPtr finish_motion_server_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp_action::Client<StopAndCenter>::SharedPtr stop_center_client_;
  std::string output_topic_;
  std::string stop_and_center_action_;
  std::string active_input_;
  double publish_rate_ = 50.0;
  double diagnostic_rate_ = 1.0;
  double stop_and_center_wait_timeout_ = 7.0;
  double finish_source_suppression_ = 0.5;
  int switch_stop_cycles_ = 1;
  int stop_cycles_remaining_ = 0;
  bool center_on_nav_loss_ = true;
  bool center_on_tag_loss_ = true;
  bool motion_since_center_ = false;
  std::atomic<bool> centering_active_{false};
  std::atomic<bool> last_center_success_{false};
  std::mutex action_mutex_;
  std::condition_variable action_cv_;
  bool action_done_ = false;
  bool action_success_ = false;
  std::string action_message_;
  std::mutex diagnostic_mutex_;
  std::string diagnostic_code_;
  std::string diagnostic_message_;
  std::string diagnostic_action_;
  uint8_t diagnostic_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
  TimePoint diagnostic_until_{};
  std::string last_diagnostic_signature_;
  TimePoint last_diagnostic_publish_{};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelArbiter>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
