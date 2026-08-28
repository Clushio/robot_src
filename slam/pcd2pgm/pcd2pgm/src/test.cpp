#include <iostream>
#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

std::string file_directory;
std::string file_name;
std::string pcd_file;
std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::msg::OccupancyGrid map_topic_msg;

double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;

double grid_x = 0.1;
double grid_y = 0.1;
double grid_z = 0.1;

double map_resolution = 0.05;

double thre_radius = 0.1;
bool radius_filter_enable = false;
int radius_min_neighbors = 10;

bool savemap_f = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(
  new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(
  new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::msg::PointCloud2 outputpclmsg;

void PassThroughFilter(const double & thre_low, const double & thre_high, const bool & flag_in);

void RadiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const double & radius,
  const int & thre_count);

void SetMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  nav_msgs::msg::OccupancyGrid & msg,
  const rclcpp::Time & stamp);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("pcl_filters");

  file_directory = node->declare_parameter<std::string>(
    "file_directory", "/home/suv/suvrobot/src/slam/LIO-Lite-eskf-base/src/LIO-Lite/maps");
  RCLCPP_INFO(node->get_logger(), "*** file_directory = %s ***", file_directory.c_str());

  file_name = node->declare_parameter<std::string>("file_name", "GlobalMap");
  RCLCPP_INFO(node->get_logger(), "*** file_name = %s ***", file_name.c_str());

  pcd_file = file_directory + file_name + pcd_format;
  RCLCPP_INFO(node->get_logger(), "*** pcd_file = %s ***", pcd_file.c_str());

  thre_z_min = node->declare_parameter<double>("thre_z_min", -0.1);
  thre_z_max = node->declare_parameter<double>("thre_z_max", 0.3);
  flag_pass_through = node->declare_parameter<int>("flag_pass_through", 0);
  grid_x = node->declare_parameter<double>("grid_x", 0.1);
  grid_y = node->declare_parameter<double>("grid_y", 0.1);
  grid_z = node->declare_parameter<double>("grid_z", 0.1);
  thre_radius = node->declare_parameter<double>("thre_radius", 0.5);
  radius_filter_enable = node->declare_parameter<bool>("radius_filter_enable", false);
  radius_min_neighbors = node->declare_parameter<int>("radius_min_neighbors", 10);
  map_resolution = node->declare_parameter<double>("map_resolution", 0.05);
  map_topic_name = node->declare_parameter<std::string>("map_topic_name", "map");
  savemap_f = node->declare_parameter<bool>("savemap", true);

  const auto map_topic_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_name, rclcpp::QoS(1));
  const auto pcl_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "pcl_output", rclcpp::QoS(1));
  outputpclmsg.header.frame_id = "map";

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    RCLCPP_ERROR(node->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
    rclcpp::shutdown();
    return -1;
  }

  std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;
  std::cout << "threlow= " << thre_z_min << "threhig = " << thre_z_max << std::endl;

  PassThroughFilter(thre_z_min, thre_z_max, static_cast<bool>(flag_pass_through));

  if (radius_filter_enable) {
    RadiusOutlierFilter(cloud_after_PassThrough, thre_radius, radius_min_neighbors);
    SetMapTopicMsg(cloud_after_Radius, map_topic_msg, node->now());
  } else {
    SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg, node->now());
  }

  rclcpp::WallRate loop_rate(1.0);
  while (rclcpp::ok()) {
    map_topic_pub->publish(map_topic_msg);
    pcl_pub->publish(outputpclmsg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

void PassThroughFilter(const double & thre_low, const double & thre_high, const bool & flag_in)
{
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  std::cout << "threlow= " << thre_low << "threhig =" << thre_high << std::endl;
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_PassThrough);
  std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;

  pcl::toROSMsg(*cloud_after_PassThrough, outputpclmsg);
  outputpclmsg.header.frame_id = "map";
}

void RadiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const double & radius,
  const int & thre_count)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  radiusoutlier.setInputCloud(cloud);
  radiusoutlier.setRadiusSearch(radius);
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}

void SetMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  nav_msgs::msg::OccupancyGrid & msg,
  const rclcpp::Time & stamp)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";

  msg.info.map_load_time = stamp;
  msg.info.resolution = map_resolution;

  double x_min;
  double x_max;
  double y_min;
  double y_max;
  const double z_max_grey_rate = 0.05;
  const double z_min_grey_rate = 0.95;
  const double k_line =
    (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  const double b_line =
    (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
    (thre_z_max - thre_z_min);
  (void)k_line;
  (void)b_line;

  if (cloud->points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("pcd2pgm"), "pcd is empty!");
    return;
  }

  for (std::size_t i = 0; i + 1 < cloud->points.size(); ++i) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    const double x = cloud->points[i].x;
    const double y = cloud->points[i].y;

    if (x < x_min) {x_min = x;}
    if (x > x_max) {x_max = x;}
    if (y < y_min) {y_min = y;}
    if (y > y_max) {y_max = y;}
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = static_cast<uint32_t>((x_max - x_min) / map_resolution);
  msg.info.height = static_cast<uint32_t>((y_max - y_min) / map_resolution);

  msg.data.assign(msg.info.width * msg.info.height, 0);
  RCLCPP_INFO(rclcpp::get_logger("pcd2pgm"), "data size = %zu", msg.data.size());

  for (const auto & point : cloud->points) {
    const int i = static_cast<int>((point.x - x_min) / map_resolution);
    if (i < 0 || i >= static_cast<int>(msg.info.width)) {
      continue;
    }

    const int j = static_cast<int>((point.y - y_min) / map_resolution);
    if (j < 0 || j >= static_cast<int>(msg.info.height) - 1) {
      continue;
    }

    msg.data[static_cast<std::size_t>(i) +
      static_cast<std::size_t>(j) * msg.info.width] = 100;
  }
}
