#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// Parameters for angle range
float min_angle_deg, max_angle_deg;

// Helper function to calculate the angle of a point in degrees
float calculateAngle(float x, float y) {
    float angle = atan2(y, x) * 180.0 / M_PI; // Convert radians to degrees
    if (angle < -180.0) angle += 360.0; // Normalize to [-180, 180]
    return angle;
}

void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      float x = livox_msg->points[i].x;
      float y = livox_msg->points[i].y;
      float z = livox_msg->points[i].z;

      // Calculate the angle of the point
      float angle = calculateAngle(x, y);

      // Only process points within the desired angle range
      if (angle >= min_angle_deg && angle <= max_angle_deg) {
        PointType pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        float s = livox_msg->points[i].offset_time / (float)time_end;

        pt.intensity = livox_msg->points[i].line + livox_msg->points[i].reflectivity / 10000.0; // The integer part is line number and the decimal part is timestamp
        pt.curvature = s * 0.1;
        pcl_in.push_back(pt);
      }
    }
  }

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "livox_frame";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); // Private node handle for parameters

  ROS_INFO("start livox_repub");

  // Load parameters from the parameter server
  private_nh.param<float>("min_angle_deg", min_angle_deg, -120.0); // Default: -120
  private_nh.param<float>("max_angle_deg", max_angle_deg, 120.0);  // Default: 120

  ROS_INFO("Angle range set to [%.2f, %.2f] degrees", min_angle_deg, max_angle_deg);

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);

  ros::spin();
}