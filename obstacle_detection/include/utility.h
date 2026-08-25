#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

#include <set> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI PointType;

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

// HDL-64E
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 0.427;
// extern const float ang_bottom = 24.9;
// extern const int groundScanInd = 60;//lidar光束在地面下的数量

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

// 传感器采样频率
extern const float scanPeriod = 0.1;

// struct smoothness_t{ 
//     float value;
//     size_t ind;
// };

// struct by_value{ 
//     bool operator()(smoothness_t const &left, smoothness_t const &right) { 
//         return left.value < right.value;
//     }
// };

namespace velodyne_pcl
{
    struct PointXYZIRT
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float         intensity;            ///< laser intensity reading
        std::uint16_t ring;                 ///< laser ring number
        float         time;                 ///< laser time reading
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    }
    EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
