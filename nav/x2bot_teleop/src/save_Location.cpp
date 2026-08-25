#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class JoyLocationSaver : public rclcpp::Node
{
public:
  JoyLocationSaver()
  : Node("joy_location_saver"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    id_(0),
    workstation_id_(0),
    idx_(0),
    idt_(0),
    file_opened_(false),
    counters_initialized_(false),
    test_file_opened_(false)
  {
    filename_ = declare_parameter<std::string>("positions_file", defaultPositionFilename());
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&JoyLocationSaver::joyCallback, this, std::placeholders::_1));
  }

  ~JoyLocationSaver() override
  {
    file_.close();
    test_file_.close();
  }

private:
  std::string defaultPositionFilename()
  {
    const char * home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0') {
      std::string home_dir(home);
      while (home_dir.size() > 1 && home_dir.back() == '/') {
        home_dir.pop_back();
      }
      return home_dir + "/maps/robot_positions.txt";
    }
    RCLCPP_WARN(get_logger(), "HOME is not set; using robot_positions.txt in the working directory.");
    return "robot_positions.txt";
  }

  static std::string timeToFilename()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream stream;
    stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return stream.str();
  }

  void loadExistingPointCounters()
  {
    std::ifstream input(filename_);
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream stream(line);
      double x, y, z, roll, pitch, yaw;
      if (!(stream >> x >> y >> z >> roll >> pitch >> yaw)) {
        continue;
      }
      ++id_;
      std::string label;
      if (stream >> label && label.size() > 1 && label[0] == 'W') {
        try {
          workstation_id_ = std::max(workstation_id_, std::stoi(label.substr(1)));
        } catch (const std::exception &) {
          // Non-numeric labels do not participate in W1, W2... numbering.
        }
      }
    }
    RCLCPP_INFO(
      get_logger(), "Continue point recording at P%d; next workstation is W%d.",
      id_, workstation_id_ + 1);
  }

  bool openPositionFile()
  {
    if (!file_opened_) {
      if (!counters_initialized_) {
        loadExistingPointCounters();
        counters_initialized_ = true;
      }
      file_.open(filename_, std::ios_base::out | std::ios_base::app);
      if (!file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Unable to open file: %s", filename_.c_str());
        return false;
      }
      file_opened_ = true;
    }
    return true;
  }

  bool lookupCurrentPose(
    double & x, double & y, double & z,
    double & roll, double & pitch, double & yaw)
  {
    const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = tf2::getYaw(transform.transform.rotation);
    return true;
  }

  void saveCurrentPose(const std::string & label)
  {
    if (!openPositionFile()) {
      return;
    }
    try {
      double x, y, z, roll, pitch, yaw;
      lookupCurrentPose(x, y, z, roll, pitch, yaw);
      ++id_;
      std::cout << id_ << " save position at " << x << "," << y << "," << z << ","
                << roll << "," << pitch << "," << yaw;
      file_ << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw;
      if (!label.empty()) {
        std::cout << " " << label;
        file_ << " " << label;
      }
      std::cout << std::endl;
      file_ << std::endl;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }

  void saveTestPose()
  {
    if (!test_file_opened_) {
      const std::string test_filename =
        "/home/nav/maps/robot_positions_test" + timeToFilename() + ".txt";
      test_file_.open(test_filename, std::ios_base::out);
      if (!test_file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Unable to open file: %s", test_filename.c_str());
        return;
      }
      test_file_opened_ = true;
    }

    try {
      double x, y, z, roll, pitch, yaw;
      lookupCurrentPose(x, y, z, roll, pitch, yaw);
      // Preserve the original test recorder's 3D z value.
      const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      z = transform.transform.translation.z;
      ++idx_;
      if (idx_ == 31) {
        idx_ = 0;
        ++idt_;
      }
      std::cout << idt_ << " robot is in position at " << x << "," << y << "," << z
                << "," << roll << "," << pitch << "," << yaw << std::endl;
      test_file_ << "poseID=" << idt_ << " " << x << " " << y << " " << z << " "
                 << roll << " " << pitch << " " << yaw << std::endl;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    if (joy->buttons.size() <= 5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message has %zu buttons; location saver requires buttons 2, 4 and 5.",
        joy->buttons.size());
      return;
    }
    if (joy->buttons[2]) {
      saveCurrentPose("");
    }
    if (joy->buttons[4]) {
      if (!openPositionFile()) {
        return;
      }
      ++workstation_id_;
      saveCurrentPose("W" + std::to_string(workstation_id_));
    }
    if (joy->buttons[5]) {
      saveTestPose();
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::ofstream file_;
  std::ofstream test_file_;
  int id_;
  int workstation_id_;
  int idx_;
  int idt_;
  std::string filename_;
  bool file_opened_;
  bool counters_initialized_;
  bool test_file_opened_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyLocationSaver>());
  rclcpp::shutdown();
  return 0;
}
