#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>


tf2_ros::Buffer tf_buffer_;
	 std::ofstream file_;
	 int id = 0;
	 int workstation_id = 0;

 int idx=0;
 int idt=0;

 std::ofstream file_poseRecordTest;

 // 文件路径
std::string filename_ = "/home/nav/maps/robot_positions.txt";
    // 标志变量，用于指示文件是否已经打开
bool file_opened = false;

std::stringstream time2filename()
{
     // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    // 创建一个 stringstream 来格式化时间
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss;

}

bool file_1_openend = false;

bool openPositionFile()
{
    if (!file_opened) {
        file_.open(filename_, std::ios_base::out);
        if (!file_.is_open()) {
            std::cerr << "Unable to open file: " << filename_ << std::endl;
            return false;
        }
        file_opened = true;
    }
    return true;
}

bool lookupCurrentPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
{
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    z = transform.transform.translation.z;

    geometry_msgs::Quaternion quat = transform.transform.rotation;
    tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    roll = 0;
    pitch = 0;
    return true;
}

void saveCurrentPose(const std::string& label)
{
    if (!openPositionFile()) {
        return;
    }

    try {
        double x, y, z, roll, pitch, yaw;
        lookupCurrentPose(x, y, z, roll, pitch, yaw);

        id++;
        std::cout << id << " save position at " << x << "," << y << "," << z << ","
                  << roll << "," << pitch << "," << yaw;
        file_ << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw;
        if (!label.empty()) {
            std::cout << " " << label;
            file_ << " " << label;
        }
        std::cout << std::endl;
        file_ << std::endl;
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_STREAM("Failed to lookup transform: " << ex.what());
    }
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
	{
	    if (joy_msg->buttons[2]) // 检查按钮 2 是否按下
	    {
            saveCurrentPose("");
	    }

         if (joy_msg->buttons[4]) // 工位点
        {
            workstation_id++;
            saveCurrentPose("W" + std::to_string(workstation_id));
        }

	     if (joy_msg->buttons[5]) // 检查按钮 5 是否按下
        {
             // 如果文件还没有打开，则以新建模式打开文件
            if (!file_1_openend) {
                std::stringstream ss = time2filename();
                std::string filename_1 = "/home/nav/maps/robot_positions_test"+ss.str() + ".txt";
                file_poseRecordTest.open(filename_1, std::ios_base::out);
                if (!file_poseRecordTest.is_open()) {
                    std::cerr << "Unable to open file: " << filename_1 << std::endl;
                    return ;
                }
                file_1_openend = true; // 设置标志变量，表示文件已打开
            }
            try
            {
                // 查询 map 到 baselink 的变换
                geometry_msgs::TransformStamped transform;
             //   tf_buffer_.waitForTransform("map", "baselink", ros::Time(0), ros::Duration(1.0));
                transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
                //tf_listene
                // 提取并保存位置数据
                double x = transform.transform.translation.x;
                double y = transform.transform.translation.y;
                double z = transform.transform.translation.z;

                // 提取并保存方向角四元数
                geometry_msgs::Quaternion quat = transform.transform.rotation;
                double roll, pitch, yaw;
                tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
                tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

                idx++;
                roll = 0;
                pitch = 0;
                if (idx==31)
                {
                    idx = 0;
                    idt++;
                }
                std::cout<<idt<<" robot is in  position at "<<x << "," << y << "," << z << "," << roll << "," << pitch << "," << yaw << std::endl;
                file_poseRecordTest<<"poseID="<<idt<<" " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN_STREAM("Failed to lookup transform: " << ex.what());
            }
        }
    }



int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_location_saver");
   // ros::NodeHandle nh;
    ros::NodeHandle nh_;

    tf2_ros::TransformListener tfListener(tf_buffer_);
    ros::Subscriber joy_sub_ = nh_.subscribe("/joy", 1, &joyCallback);

    //std::string filename_ = "/home/jgl20/map/robot_positions.txt";
    //file_.open(filename_, std::ios_base::out);
  //  JoyLocationSaver location_saver();
    ros::spin();
    file_.close();
    file_poseRecordTest.close();

    return 0;
}
