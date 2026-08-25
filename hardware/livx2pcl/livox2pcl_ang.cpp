#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace
{

using PointType = pcl::PointXYZINormal;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;

class LivoxAngleRepublisher : public rclcpp::Node
{
public:
  LivoxAngleRepublisher()
  : Node("livox_repub_ang")
  {
    min_angle_deg_ = declare_parameter<double>("min_angle_deg", -120.0);
    max_angle_deg_ = declare_parameter<double>("max_angle_deg", 120.0);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox_pcl0", 100);
    subscription_ = create_subscription<CustomMsg>(
      "/livox/lidar", 100,
      std::bind(
        &LivoxAngleRepublisher::LivoxMsgCallback, this,
        std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Angle range set to [%.2f, %.2f] degrees",
      min_angle_deg_, max_angle_deg_);
  }

private:
  static double CalculateAngle(float x, float y)
  {
    double angle = std::atan2(y, x) * 180.0 / M_PI;
    if (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  void LivoxMsgCallback(CustomMsg::ConstSharedPtr livox_msg_in)
  {
    livox_data_.push_back(livox_msg_in);
    if (livox_data_.size() < messages_to_merge_) {
      return;
    }

    pcl::PointCloud<PointType> pcl_in;
    for (const auto & livox_msg : livox_data_) {
      const auto time_end = livox_msg->points.back().offset_time;
      for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
        const float x = livox_msg->points[i].x;
        const float y = livox_msg->points[i].y;
        const float z = livox_msg->points[i].z;
        const double angle = CalculateAngle(x, y);
        if (angle < min_angle_deg_ || angle > max_angle_deg_) {
          continue;
        }

        PointType point;
        point.x = x;
        point.y = y;
        point.z = z;
        const float offset_ratio =
          livox_msg->points[i].offset_time / static_cast<float>(time_end);
        point.intensity = livox_msg->points[i].line +
          livox_msg->points[i].reflectivity / 10000.0F;
        point.curvature = offset_ratio * 0.1F;
        pcl_in.push_back(point);
      }
    }

    const uint64_t timebase_ns = livox_data_.front()->timebase;
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(pcl_in, output);
    output.header.stamp = rclcpp::Time(static_cast<int64_t>(timebase_ns));
    output.header.frame_id = "livox_frame";
    publisher_->publish(output);
    livox_data_.clear();
  }

  const std::size_t messages_to_merge_ = 1;
  double min_angle_deg_ = -120.0;
  double max_angle_deg_ = 120.0;
  std::vector<CustomMsg::ConstSharedPtr> livox_data_;
  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxAngleRepublisher>());
  rclcpp::shutdown();
  return 0;
}
