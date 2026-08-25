/********x2bot_joy.cpp***************
 * 功能：接收 Joy，使用 deadman 按钮将手柄量转换为 /cmd_vel/teleop。
 ****************************************/
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class X2botTeleop : public rclcpp::Node
{
public:
  X2botTeleop()
  : Node("x2bot_teleop"),
    linear_(7),
    angular_(6),
    deadman_axis_(0),
    l_scale_(0.3),
    a_scale_(0.9),
    deadman_pressed_(false),
    zero_twist_published_(false)
  {
    linear_ = declare_parameter<int>("axis_linear", linear_);
    angular_ = declare_parameter<int>("axis_angular", angular_);
    deadman_axis_ = declare_parameter<int>("axis_deadman", deadman_axis_);
    a_scale_ = declare_parameter<double>("scale_angular", a_scale_);
    l_scale_ = declare_parameter<double>("scale_linear", l_scale_);
    const auto cmd_vel_topic =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel/teleop");

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&X2botTeleop::joyCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&X2botTeleop::publish, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    if (linear_ < 0 || angular_ < 0 || deadman_axis_ < 0 ||
      static_cast<size_t>(linear_) >= joy->axes.size() ||
      static_cast<size_t>(angular_) >= joy->axes.size() ||
      static_cast<size_t>(deadman_axis_) >= joy->buttons.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message does not contain configured axes/buttons (%d, %d, %d).",
        linear_, angular_, deadman_axis_);
      return;
    }

    std::lock_guard<std::mutex> lock(publish_mutex_);
    last_published_.angular.z = a_scale_ * joy->axes[angular_];
    last_published_.linear.x = l_scale_ * joy->axes[linear_];
    deadman_pressed_ = joy->buttons[deadman_axis_];
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (deadman_pressed_) {
      vel_pub_->publish(last_published_);
      zero_twist_published_ = false;
    } else if (!zero_twist_published_) {
      vel_pub_->publish(geometry_msgs::msg::Twist());
      zero_twist_published_ = true;
    }
  }

  int linear_;
  int angular_;
  int deadman_axis_;
  double l_scale_;
  double a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  geometry_msgs::msg::Twist last_published_;
  std::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<X2botTeleop>());
  rclcpp::shutdown();
  return 0;
}
