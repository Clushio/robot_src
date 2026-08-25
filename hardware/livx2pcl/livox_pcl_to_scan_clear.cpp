#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LivoxPclToScanClear : public rclcpp::Node
{
public:
  LivoxPclToScanClear()
  : Node("livox_pcl_to_scan_clear")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/livox_pcl0");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/livox_scan_clear");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    min_angle_deg_ = declare_parameter<double>("min_angle_deg", -110.0);
    max_angle_deg_ = declare_parameter<double>("max_angle_deg", 110.0);
    angle_increment_deg_ = declare_parameter<double>(
      "angle_increment_deg", 1.0);
    range_min_ = declare_parameter<double>("range_min", 0.10);
    range_max_ = declare_parameter<double>("range_max", 3.5);
    min_height_ = declare_parameter<double>("min_height", 0.2);
    max_height_ = declare_parameter<double>("max_height", 1.2);

    min_angle_ = min_angle_deg_ * M_PI / 180.0;
    max_angle_ = max_angle_deg_ * M_PI / 180.0;
    angle_increment_ = angle_increment_deg_ * M_PI / 180.0;

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, 5);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 5,
      std::bind(
        &LivoxPclToScanClear::CloudCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "livox_pcl_to_scan_clear: %s -> %s, angle [%.1f, %.1f], range %.2f",
      input_topic_.c_str(), output_topic_.c_str(), min_angle_deg_,
      max_angle_deg_, range_max_);
  }

private:
  void CloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = cloud_msg->header.stamp;
    scan.header.frame_id = frame_id_;
    scan.angle_min = min_angle_;
    scan.angle_max = max_angle_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    const int beam_count = static_cast<int>(
      std::floor((max_angle_ - min_angle_) / angle_increment_)) + 1;
    scan.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < min_height_ || z > max_height_) {
        continue;
      }

      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = std::atan2(y, x);
      if (angle < min_angle_ || angle > max_angle_) {
        continue;
      }

      const int index = static_cast<int>(
        (angle - min_angle_) / angle_increment_);
      if (index >= 0 && index < beam_count) {
        scan.ranges[index] = std::min(
          scan.ranges[index], static_cast<float>(range));
      }
    }

    scan_pub_->publish(scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double min_angle_deg_ = -110.0;
  double max_angle_deg_ = 110.0;
  double angle_increment_deg_ = 1.0;
  double range_min_ = 0.10;
  double range_max_ = 3.5;
  double min_height_ = 0.2;
  double max_height_ = 1.2;
  double min_angle_ = 0.0;
  double max_angle_ = 0.0;
  double angle_increment_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPclToScanClear>());
  rclcpp::shutdown();
  return 0;
}
