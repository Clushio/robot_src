#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class LivoxPclToScanClear {
 public:
  LivoxPclToScanClear() : private_nh_("~") {
    private_nh_.param<std::string>("input_topic", input_topic_, "/livox_pcl0");
    private_nh_.param<std::string>("output_topic", output_topic_, "/livox_scan_clear");
    private_nh_.param<std::string>("frame_id", frame_id_, "livox_frame");
    private_nh_.param("min_angle_deg", min_angle_deg_, -110.0);
    private_nh_.param("max_angle_deg", max_angle_deg_, 110.0);
    private_nh_.param("angle_increment_deg", angle_increment_deg_, 1.0);
    private_nh_.param("range_min", range_min_, 0.10);
    private_nh_.param("range_max", range_max_, 3.5);
    private_nh_.param("min_height", min_height_, 0.2);
    private_nh_.param("max_height", max_height_, 1.2);

    min_angle_ = min_angle_deg_ * M_PI / 180.0;
    max_angle_ = max_angle_deg_ * M_PI / 180.0;
    angle_increment_ = angle_increment_deg_ * M_PI / 180.0;

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 5);
    cloud_sub_ = nh_.subscribe(input_topic_, 5, &LivoxPclToScanClear::cloudCallback, this);

    ROS_INFO("livox_pcl_to_scan_clear: %s -> %s, angle [%.1f, %.1f], range %.2f",
             input_topic_.c_str(), output_topic_.c_str(), min_angle_deg_, max_angle_deg_, range_max_);
  }

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    sensor_msgs::LaserScan scan;
    scan.header.stamp = cloud_msg->header.stamp;
    scan.header.frame_id = frame_id_;
    scan.angle_min = min_angle_;
    scan.angle_max = max_angle_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    const int beam_count = static_cast<int>(std::floor((max_angle_ - min_angle_) / angle_increment_)) + 1;
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

      const int index = static_cast<int>((angle - min_angle_) / angle_increment_);
      if (index >= 0 && index < beam_count) {
        scan.ranges[index] = std::min(scan.ranges[index], static_cast<float>(range));
      }
    }

    scan_pub_.publish(scan);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher scan_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double min_angle_deg_;
  double max_angle_deg_;
  double angle_increment_deg_;
  double range_min_;
  double range_max_;
  double min_height_;
  double max_height_;
  double min_angle_;
  double max_angle_;
  double angle_increment_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_pcl_to_scan_clear");
  LivoxPclToScanClear node;
  ros::spin();
  return 0;
}
