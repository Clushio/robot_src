#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <x2bot_teleop/SetInt.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

struct TargetPose {
    double x, y, z, roll, pitch, yaw;
    std::string label;
};

struct TopoEdge {
    int to;
    double cost;
    bool trusted;
    bool bidirectional;
    bool configured_blocked;
    ros::WallTime blocked_until;
    int failure_count;
    std::string source;
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MVClient;

class mynav
{
public:
    mynav()
        : global_ac(nullptr),
          nh_(),
          tf_listener_(tf_buffer_),
          joy_spinner_(1, &joy_callback_queue_),
          reference_status_spinner_(1, &reference_status_queue_),
          current_pose_index(0),
          numofpnts(0),
          current_pnt(0),
          stop_and_quit(false),
          pause_robot(false),
          pause_reentry_requested_(false),
          current_status(0),
          blocked_timeout_(10.0),
          blocked_cooldown_initial_(20.0),
          blocked_cooldown_max_(120.0),
          blocked_backoff_factor_(2.0),
          blocked_wait_timeout_(180.0),
          progress_distance_(0.05),
          progress_yaw_(6.0 * M_PI / 180.0),
          waypoint_reached_distance_(0.20),
          validate_pass_through_action_success_distance_(false),
          goal_timeout_(120.0),
          block_bidirectional_(true),
          static_map_loaded_(false),
          static_map_width_(0),
          static_map_height_(0),
          static_map_resolution_(0.05),
          static_map_origin_x_(0.0),
          static_map_origin_y_(0.0),
          static_map_occupied_thresh_(0.65),
          static_map_negate_(false),
          static_map_inflation_radius_(0.40)
    {
        ros::NodeHandle private_nh("~");
        maps_dir_ = defaultMapsDir();
        private_nh.param("maps_dir", maps_dir_, maps_dir_);
        normalizeMapsDir();
        ROS_INFO_STREAM("Topology navigation maps_dir: " << maps_dir_);
        private_nh.param("blocked_timeout", blocked_timeout_, blocked_timeout_);
        private_nh.param("blocked_cooldown_initial", blocked_cooldown_initial_,
                         blocked_cooldown_initial_);
        private_nh.param("blocked_cooldown_max", blocked_cooldown_max_,
                         blocked_cooldown_max_);
        private_nh.param("blocked_backoff_factor", blocked_backoff_factor_,
                         blocked_backoff_factor_);
        private_nh.param("blocked_wait_timeout", blocked_wait_timeout_,
                         blocked_wait_timeout_);
        blocked_cooldown_initial_ = std::max(0.1, blocked_cooldown_initial_);
        blocked_cooldown_max_ = std::max(blocked_cooldown_initial_, blocked_cooldown_max_);
        blocked_backoff_factor_ = std::max(1.0, blocked_backoff_factor_);
        blocked_wait_timeout_ = std::max(0.0, blocked_wait_timeout_);
        private_nh.param("progress_distance", progress_distance_, progress_distance_);
        private_nh.param("progress_yaw", progress_yaw_, progress_yaw_);
        private_nh.param("waypoint_reached_distance", waypoint_reached_distance_,
                         waypoint_reached_distance_);
        private_nh.param("validate_pass_through_action_success_distance",
                         validate_pass_through_action_success_distance_,
                         validate_pass_through_action_success_distance_);
        private_nh.param("goal_timeout", goal_timeout_, goal_timeout_);
        private_nh.param("block_bidirectional", block_bidirectional_, block_bidirectional_);
        private_nh.param("static_map_inflation_radius", static_map_inflation_radius_,
                         static_map_inflation_radius_);

        initializeGlobalAC();
        std::string cmd_vel_topic;
        private_nh.param("cmd_vel_topic", cmd_vel_topic,
                         std::string("/cmd_vel/safety"));
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
        ros::SubscribeOptions joy_options =
            ros::SubscribeOptions::create<sensor_msgs::Joy>(
                "joy", 10, boost::bind(&mynav::joyCallback, this, _1),
                ros::VoidPtr(), &joy_callback_queue_);
        joy_sub_ = nh_.subscribe(joy_options);
        joy_spinner_.start();
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("topology_markers", 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("topology_plan", 1, true);
        ros::SubscribeOptions status_options =
            ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(
                "/bspline_status", 1,
                boost::bind(&mynav::referenceStatusCallback, this, _1),
                ros::VoidPtr(), &reference_status_queue_);
        reference_status_sub_ = nh_.subscribe(status_options);
        reference_status_spinner_.start();

        plan_path_service = nh_.advertiseService("plan_path_and_go", &mynav::planPathCallback, this);
        ROS_INFO("Topology navigation service /plan_path_and_go started.");
    }

    ~mynav()
    {
        joy_spinner_.stop();
        reference_status_spinner_.stop();
        if (runth_ && runth_->joinable())
        {
            runth_->join();
        }
        delete global_ac;
    }

    bool planPathCallback(x2bot_teleop::SetInt::Request &req,
                          x2bot_teleop::SetInt::Response &res)
    {
        const int target_index = resolvePoseIndex(req.data);
        const int requested_current = resolvePoseIndex(req.currentID);
        const bool exec_path = req.run > 0;

        if (!validIndex(target_index))
        {
            res.success = false;
            res.message = "目标点序号无效";
            ROS_ERROR("Invalid target index request: %d", req.data);
            return true;
        }

        int start_index = nearestPoseIndex();
        if (!validIndex(start_index))
        {
            start_index = requested_current;
        }
        if (!validIndex(start_index))
        {
            res.success = false;
            res.message = "当前点序号无效";
            ROS_ERROR("Invalid current index request: %d", req.currentID);
            return true;
        }

        current_pose_index = start_index;
        std::vector<int> path_indices = dijkstraShortestPath(start_index, target_index);
        if (path_indices.empty() && exec_path)
        {
            waitForTemporaryPath(start_index, target_index, path_indices);
        }
        if (path_indices.empty())
        {
            res.success = false;
            res.message = "无法找到从 P" + std::to_string(start_index) +
                          " 到 P" + std::to_string(target_index) + " 的拓扑路径";
            ROS_ERROR_STREAM(res.message);
            stopRobot();
            return true;
        }

        publishTopologyPath(path_indices);
        res.message = formatPathMessage("plan ok:", path_indices);
        ROS_INFO_STREAM(res.message);

        if (exec_path)
        {
            const bool ok = runTopologyMission(target_index, path_indices);
            res.success = ok;
            if (ok)
            {
                res.message = "arrived:" + formatPathMessage("", path_indices);
            }
            else
            {
                res.message = "目标暂时不可达，已停车";
            }
        }
        else
        {
            res.success = true;
        }

        return true;
    }

    void publishNavPointsMarkers()
    {
        visualization_msgs::Marker marker = makeMarker("nav_points", 0, visualization_msgs::Marker::SPHERE_LIST);
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        setMarkerScale(marker, 0.28);
        setMarkerColor(marker, 1.0, 0.82, 0.05, 0.85);

        for (const auto &pose : target_poses)
        {
            geometry_msgs::Point p;
            p.x = pose.x;
            p.y = pose.y;
            p.z = pose.z;
            marker.points.push_back(p);
        }

        marker_pub.publish(marker);
        publishTopologyMarkers();
    }

    bool loadNavPnts()
    {
        numofpnts = 0;
        const std::string positions_file = joinPath(maps_dir_, "robot_positions.txt");
        std::ifstream infile(positions_file);
        if (!infile.is_open())
        {
            ROS_ERROR_STREAM("Failed to open " << positions_file);
            return false;
        }

        target_poses.clear();
        workstation_indices.clear();
        std::string line;
        while (std::getline(infile, line))
        {
            line = trim(line);
            if (line.empty())
            {
                continue;
            }
            try
            {
                TargetPose pose = parseTargetPose(line);
                const int index = target_poses.size();
                if (!pose.label.empty())
                {
                    workstation_indices[pose.label] = index;
                    ROS_INFO_STREAM(pose.label << " maps to point " << index);
                }
                target_poses.push_back(pose);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("Error parsing target pose: " << e.what());
            }
        }

        numofpnts = target_poses.size();
        ROS_INFO("Loaded %d navigation points.", numofpnts);
        publishNavPointsMarkers();
        return numofpnts > 0;
    }

    bool loadTopology()
    {
        graph.clear();
        if (loadTopologyYaml(joinPath(maps_dir_, "topology.yaml")))
        {
            ROS_INFO_STREAM("Loaded topology.yaml with " << edgeCount() << " directed edges.");
            return true;
        }

        ROS_WARN("topology.yaml not found or invalid, try legacy topo.txt.");
        if (loadTopoFromTxt())
        {
            ROS_INFO_STREAM("Loaded topo.txt with " << edgeCount() << " directed edges.");
            return true;
        }

        ROS_WARN("No topology file available, build sequential trusted topology from robot_positions.txt.");
        buildSequentialTopology();
        return edgeCount() > 0;
    }

    bool loadStaticMap()
    {
        const std::string map_yaml = joinPath(maps_dir_, "map.yaml");
        std::ifstream yaml_file(map_yaml);
        if (!yaml_file.is_open())
        {
            ROS_WARN_STREAM("Static map yaml not found: " << map_yaml);
            return false;
        }

        std::string image_file = "map.pgm";
        std::string line;
        while (std::getline(yaml_file, line))
        {
            line = trim(line);
            if (line.empty() || startsWith(line, "#"))
            {
                continue;
            }
            if (startsWith(line, "image:"))
            {
                image_file = afterColon(line);
            }
            else if (startsWith(line, "resolution:"))
            {
                static_map_resolution_ = std::stod(afterColon(line));
            }
            else if (startsWith(line, "occupied_thresh:"))
            {
                static_map_occupied_thresh_ = std::stod(afterColon(line));
            }
            else if (startsWith(line, "negate:"))
            {
                static_map_negate_ = std::stoi(afterColon(line)) != 0;
            }
            else if (startsWith(line, "origin:"))
            {
                parseMapOrigin(afterColon(line));
            }
        }

        std::string map_dir = maps_dir_;
        if (!map_dir.empty() && map_dir[map_dir.size() - 1] != '/')
        {
            map_dir += '/';
        }
        std::string pgm_file = image_file;
        if (!image_file.empty() && image_file[0] != '/')
        {
            pgm_file = map_dir + image_file;
        }
        else if (!fileExists(pgm_file))
        {
            const std::size_t slash = image_file.find_last_of('/');
            const std::string basename = slash == std::string::npos ? image_file : image_file.substr(slash + 1);
            const std::string fallback = map_dir + basename;
            if (fileExists(fallback))
            {
                ROS_WARN_STREAM("Map image path " << pgm_file << " not found, use " << fallback);
                pgm_file = fallback;
            }
        }

        std::vector<unsigned char> pixels;
        int max_value = 255;
        if (!readPgm(pgm_file, static_map_width_, static_map_height_, max_value, pixels))
        {
            ROS_WARN_STREAM("Failed to read static map image: " << pgm_file);
            return false;
        }

        static_map_occupied_.assign(pixels.size(), 0);
        for (std::size_t i = 0; i < pixels.size(); ++i)
        {
            const double normalized = static_cast<double>(pixels[i]) / std::max(1, max_value);
            const double occupancy = static_map_negate_ ? normalized : (1.0 - normalized);
            static_map_occupied_[i] = occupancy >= static_map_occupied_thresh_ ? 1 : 0;
        }
        inflateStaticMap();

        static_map_loaded_ = true;
        ROS_INFO("Loaded static map for topology connect check: %dx%d, resolution %.3f, inflation %.2f m.",
                 static_map_width_, static_map_height_, static_map_resolution_,
                 static_map_inflation_radius_);
        return true;
    }

private:
    MVClient *global_ac;
    std::unique_ptr<std::thread> runth_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher vel_pub_;
    ros::CallbackQueue joy_callback_queue_;
    ros::AsyncSpinner joy_spinner_;
    ros::Subscriber joy_sub_;
    ros::Publisher marker_pub;
    ros::Publisher marker_array_pub_;
    ros::Publisher path_pub_;
    ros::CallbackQueue reference_status_queue_;
    ros::AsyncSpinner reference_status_spinner_;
    ros::Subscriber reference_status_sub_;
    std::mutex reference_status_mutex_;
    ros::Time reference_status_stamp_;
    int reference_status_path_index_ = -1;
    int reference_status_code_ = 0;
    ros::ServiceServer plan_path_service;

    int current_pose_index;
    std::vector<TargetPose> target_poses;
    std::map<std::string, int> workstation_indices;
    int numofpnts;
    int current_pnt;
    bool stop_and_quit;
    std::atomic<bool> pause_robot;
    std::atomic<bool> pause_reentry_requested_;
    std::map<int, std::vector<TopoEdge>> graph;
    int current_status; // 0 init, 1 load pnts ok, 2 running, 3 finished, 4 quit

    double blocked_timeout_;
    double blocked_cooldown_initial_;
    double blocked_cooldown_max_;
    double blocked_backoff_factor_;
    double blocked_wait_timeout_;
    double progress_distance_;
    double progress_yaw_;
    double waypoint_reached_distance_;
    bool validate_pass_through_action_success_distance_;
    double goal_timeout_;
    bool block_bidirectional_;
    std::vector<int> active_path_;
    int active_next_index_ = -1;
    std::string maps_dir_;
    bool static_map_loaded_;
    int static_map_width_;
    int static_map_height_;
    double static_map_resolution_;
    double static_map_origin_x_;
    double static_map_origin_y_;
    double static_map_occupied_thresh_;
    bool static_map_negate_;
    double static_map_inflation_radius_;
    std::vector<unsigned char> static_map_occupied_;

    static std::string trim(const std::string &value)
    {
        const std::string whitespace = " \t\r\n";
        const std::size_t start = value.find_first_not_of(whitespace);
        if (start == std::string::npos)
        {
            return "";
        }
        const std::size_t end = value.find_last_not_of(whitespace);
        return value.substr(start, end - start + 1);
    }

    static bool startsWith(const std::string &value, const std::string &prefix)
    {
        return value.compare(0, prefix.size(), prefix) == 0;
    }

    static std::string afterColon(const std::string &line)
    {
        const std::size_t pos = line.find(':');
        if (pos == std::string::npos)
        {
            return "";
        }
        return trim(line.substr(pos + 1));
    }

    static bool parseBool(const std::string &value, bool default_value)
    {
        if (value == "true" || value == "True" || value == "1")
        {
            return true;
        }
        if (value == "false" || value == "False" || value == "0")
        {
            return false;
        }
        return default_value;
    }

    static bool fileExists(const std::string &filename)
    {
        std::ifstream file(filename);
        return file.good();
    }

    static std::string joinPath(const std::string &dir, const std::string &name)
    {
        if (dir.empty())
        {
            return name;
        }
        if (dir[dir.size() - 1] == '/')
        {
            return dir + name;
        }
        return dir + "/" + name;
    }

    static std::string defaultMapsDir()
    {
        return "/home/nav/maps";
    }

    void normalizeMapsDir()
    {
        if (!maps_dir_.empty() && maps_dir_[maps_dir_.size() - 1] == '/')
        {
            maps_dir_.erase(maps_dir_.size() - 1);
        }
    }

    void parseMapOrigin(const std::string &value)
    {
        std::string cleaned = value;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
        std::istringstream iss(cleaned);
        iss >> static_map_origin_x_ >> static_map_origin_y_;
    }

    bool readPgmToken(std::istream &stream, std::string &token)
    {
        token.clear();
        while (stream >> token)
        {
            if (!token.empty() && token[0] == '#')
            {
                std::string ignored;
                std::getline(stream, ignored);
                continue;
            }
            return true;
        }
        return false;
    }

    bool readPgm(const std::string &filename, int &width, int &height, int &max_value,
                 std::vector<unsigned char> &pixels)
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open())
        {
            return false;
        }

        std::string magic;
        if (!readPgmToken(file, magic) || (magic != "P5" && magic != "P2"))
        {
            return false;
        }

        std::string token;
        if (!readPgmToken(file, token))
        {
            return false;
        }
        width = std::stoi(token);
        if (!readPgmToken(file, token))
        {
            return false;
        }
        height = std::stoi(token);
        if (!readPgmToken(file, token))
        {
            return false;
        }
        max_value = std::stoi(token);

        pixels.clear();
        pixels.resize(width * height);
        if (magic == "P5")
        {
            file.get();
            file.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
            return file.gcount() == static_cast<std::streamsize>(pixels.size());
        }

        for (int i = 0; i < width * height; ++i)
        {
            if (!readPgmToken(file, token))
            {
                return false;
            }
            pixels[i] = static_cast<unsigned char>(std::stoi(token));
        }
        return true;
    }

    void inflateStaticMap()
    {
        if (static_map_occupied_.empty() || static_map_inflation_radius_ <= 0.0)
        {
            return;
        }

        const int radius_cells = static_cast<int>(
            std::ceil(static_map_inflation_radius_ / std::max(0.001, static_map_resolution_)));
        std::vector<int> distance(static_map_occupied_.size(), -1);
        std::queue<int> q;
        for (int i = 0; i < static_cast<int>(static_map_occupied_.size()); ++i)
        {
            if (static_map_occupied_[i])
            {
                distance[i] = 0;
                q.push(i);
            }
        }

        const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        while (!q.empty())
        {
            const int index = q.front();
            q.pop();
            if (distance[index] >= radius_cells)
            {
                continue;
            }
            const int x = index % static_map_width_;
            const int y = index / static_map_width_;
            for (int i = 0; i < 8; ++i)
            {
                const int nx = x + dx[i];
                const int ny = y + dy[i];
                if (nx < 0 || ny < 0 || nx >= static_map_width_ || ny >= static_map_height_)
                {
                    continue;
                }
                const int next = ny * static_map_width_ + nx;
                if (distance[next] >= 0)
                {
                    continue;
                }
                distance[next] = distance[index] + 1;
                static_map_occupied_[next] = 1;
                q.push(next);
            }
        }
    }

    bool worldToStaticMap(double wx, double wy, int &mx, int &my) const
    {
        mx = static_cast<int>(std::floor((wx - static_map_origin_x_) / static_map_resolution_));
        my = static_cast<int>(std::floor((wy - static_map_origin_y_) / static_map_resolution_));
        return mx >= 0 && my >= 0 && mx < static_map_width_ && my < static_map_height_;
    }

    bool staticMapSegmentFree(double x0, double y0, double x1, double y1) const
    {
        if (!static_map_loaded_)
        {
            return true;
        }

        const double distance = std::hypot(x1 - x0, y1 - y0);
        const double step = std::max(0.02, static_map_resolution_ * 0.5);
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
        for (int i = 0; i <= steps; ++i)
        {
            const double t = static_cast<double>(i) / steps;
            const double wx = x0 + (x1 - x0) * t;
            const double wy = y0 + (y1 - y0) * t;
            int mx = 0;
            int my = 0;
            if (!worldToStaticMap(wx, wy, mx, my))
            {
                return false;
            }
            if (static_map_occupied_[my * static_map_width_ + mx])
            {
                return false;
            }
        }
        return true;
    }

    bool validIndex(int index) const
    {
        return index >= 0 && index < static_cast<int>(target_poses.size());
    }

    visualization_msgs::Marker makeMarker(const std::string &ns, int id, int type)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    }

    void setMarkerColor(visualization_msgs::Marker &marker, double r, double g, double b, double a)
    {
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
    }

    void setMarkerScale(visualization_msgs::Marker &marker, double size)
    {
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
    }

    geometry_msgs::Point pointForIndex(int index) const
    {
        geometry_msgs::Point point;
        if (validIndex(index))
        {
            point.x = target_poses[index].x;
            point.y = target_poses[index].y;
            point.z = target_poses[index].z + 0.06;
        }
        return point;
    }

    TargetPose parseTargetPose(const std::string &line)
    {
        std::istringstream iss(line);
        TargetPose pose;
        if (!(iss >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw))
        {
            throw std::runtime_error("Invalid target pose format");
        }
        iss >> pose.label;
        return pose;
    }

    int resolvePoseIndex(int requested_index)
    {
        if (requested_index >= 0)
        {
            return requested_index;
        }

        const std::string workstation_label = "W" + std::to_string(-requested_index);
        auto it = workstation_indices.find(workstation_label);
        if (it == workstation_indices.end())
        {
            ROS_ERROR_STREAM("Workstation " << workstation_label << " not found in robot_positions.txt");
            return -1;
        }
        return it->second;
    }

    geometry_msgs::PoseStamped toPoseStamped(const TargetPose &pose)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = pose.x;
        msg.pose.position.y = pose.y;
        msg.pose.position.z = pose.z;

        tf::Quaternion q;
        q.setRPY(pose.roll, pose.pitch, pose.yaw);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        return msg;
    }

    geometry_msgs::PoseStamped toPoseStampedWithYaw(const TargetPose &pose, double yaw)
    {
        TargetPose adjusted_pose = pose;
        adjusted_pose.roll = 0.0;
        adjusted_pose.pitch = 0.0;
        adjusted_pose.yaw = yaw;
        return toPoseStamped(adjusted_pose);
    }

    double yawBetween(int from_index, int to_index) const
    {
        if (!validIndex(from_index) || !validIndex(to_index))
        {
            return target_poses[to_index].yaw;
        }
        return std::atan2(target_poses[to_index].y - target_poses[from_index].y,
                          target_poses[to_index].x - target_poses[from_index].x);
    }

    double poseDistance(const TargetPose &a, const TargetPose &b) const
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double poseDistance(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) const
    {
        return std::hypot(a.pose.position.x - b.pose.position.x,
                          a.pose.position.y - b.pose.position.y);
    }

    double poseYawDelta(const geometry_msgs::PoseStamped &a,
                        const geometry_msgs::PoseStamped &b) const
    {
        double delta = tf::getYaw(a.pose.orientation) - tf::getYaw(b.pose.orientation);
        while (delta > M_PI)
        {
            delta -= 2.0 * M_PI;
        }
        while (delta < -M_PI)
        {
            delta += 2.0 * M_PI;
        }
        return std::fabs(delta);
    }

    double distanceToNode(const geometry_msgs::PoseStamped &pose, int index) const
    {
        if (!validIndex(index))
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::hypot(pose.pose.position.x - target_poses[index].x,
                          pose.pose.position.y - target_poses[index].y);
    }

    bool getCurrentRobotPose(geometry_msgs::PoseStamped &pose)
    {
        try
        {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.2));
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Failed to lookup map->base_link: %s", ex.what());
            return false;
        }
    }

    int nearestPoseIndex()
    {
        geometry_msgs::PoseStamped robot_pose;
        if (!getCurrentRobotPose(robot_pose) || target_poses.empty())
        {
            return -1;
        }

        std::vector<std::pair<double, int>> candidates;
        for (int i = 0; i < static_cast<int>(target_poses.size()); ++i)
        {
            const double distance = std::hypot(robot_pose.pose.position.x - target_poses[i].x,
                                               robot_pose.pose.position.y - target_poses[i].y);
            candidates.push_back(std::make_pair(distance, i));
        }
        std::sort(candidates.begin(), candidates.end());

        for (const auto &candidate : candidates)
        {
            const int index = candidate.second;
            if (staticMapSegmentFree(robot_pose.pose.position.x, robot_pose.pose.position.y,
                                     target_poses[index].x, target_poses[index].y))
            {
                ROS_INFO("Nearest reachable topology node is P%d at %.2f m.",
                         index, candidate.first);
                return index;
            }
        }

        ROS_WARN("No topology connect node is directly reachable in static map, fall back to nearest P%d at %.2f m.",
                 candidates.front().second, candidates.front().first);
        return candidates.front().second;
    }

    int forwardReentryPosition(const std::vector<int> &path_indices,
                               std::size_t search_start)
    {
        geometry_msgs::PoseStamped robot_pose;
        if (!getCurrentRobotPose(robot_pose) || path_indices.empty())
        {
            return -1;
        }

        search_start = std::min(search_start, path_indices.size() - 1);
        for (std::size_t position = search_start;
             position < path_indices.size(); ++position)
        {
            const int candidate = path_indices[position];
            if (!validIndex(candidate))
            {
                continue;
            }

            if (position > 0)
            {
                const int previous = path_indices[position - 1];
                if (!validIndex(previous))
                {
                    continue;
                }
                const double edge_x = target_poses[candidate].x - target_poses[previous].x;
                const double edge_y = target_poses[candidate].y - target_poses[previous].y;
                const double edge_length = std::hypot(edge_x, edge_y);
                const double to_candidate_x =
                    target_poses[candidate].x - robot_pose.pose.position.x;
                const double to_candidate_y =
                    target_poses[candidate].y - robot_pose.pose.position.y;
                const double forward_projection =
                    edge_x * to_candidate_x + edge_y * to_candidate_y;

                // If the robot is already beyond this node in the route direction,
                // reconnecting to it would make the vehicle reverse along the edge.
                if (edge_length > 1e-6 &&
                    forward_projection < -0.05 * edge_length)
                {
                    continue;
                }
            }

            if (staticMapSegmentFree(robot_pose.pose.position.x,
                                     robot_pose.pose.position.y,
                                     target_poses[candidate].x,
                                     target_poses[candidate].y))
            {
                ROS_INFO("Forward topology re-entry selects route position %zu, P%d (%.2f m).",
                         position, candidate,
                         distanceToNode(robot_pose, candidate));
                return static_cast<int>(position);
            }
        }

        ROS_WARN("No forward node on the remaining topology route is directly reachable.");
        return -1;
    }

    bool buildForwardReentryPath(const std::vector<int> &current_path,
                                 std::size_t current_position,
                                 bool current_goal_passed,
                                 std::vector<int> &reentry_path,
                                 std::size_t &execution_start)
    {
        if (current_path.empty() || current_position >= current_path.size())
        {
            return false;
        }

        std::size_t search_start = current_position;
        if (current_goal_passed && search_start + 1 < current_path.size())
        {
            ++search_start;
        }

        const int reentry_position =
            forwardReentryPosition(current_path, search_start);
        if (reentry_position < 0)
        {
            return false;
        }

        reentry_path.clear();
        execution_start = 0;
        if (reentry_position > 0)
        {
            // Keep the preceding node only as B-spline direction context. The
            // execution loop starts at reentry_node and never drives back to it.
            reentry_path.push_back(current_path[reentry_position - 1]);
            execution_start = 1;
        }
        reentry_path.insert(reentry_path.end(),
                            current_path.begin() + reentry_position,
                            current_path.end());
        return true;
    }

    void addDirectedEdge(int from, int to, double cost, bool trusted,
                         bool bidirectional, bool blocked, const std::string &source)
    {
        if (!validIndex(from) || !validIndex(to) || from == to)
        {
            return;
        }
        if (cost <= 0.0)
        {
            cost = poseDistance(target_poses[from], target_poses[to]);
        }
        TopoEdge edge;
        edge.to = to;
        edge.cost = cost;
        edge.trusted = trusted;
        edge.bidirectional = bidirectional;
        edge.configured_blocked = blocked;
        edge.blocked_until = ros::WallTime();
        edge.failure_count = 0;
        edge.source = source;
        graph[from].push_back(edge);
    }

    void addEdge(int from, int to, double cost, bool trusted,
                 bool bidirectional, bool blocked, const std::string &source)
    {
        addDirectedEdge(from, to, cost, trusted, bidirectional, blocked, source);
        if (bidirectional)
        {
            addDirectedEdge(to, from, cost, trusted, bidirectional, blocked, source);
        }
    }

    void buildSequentialTopology()
    {
        graph.clear();
        for (int i = 0; i + 1 < static_cast<int>(target_poses.size()); ++i)
        {
            addEdge(i, i + 1, poseDistance(target_poses[i], target_poses[i + 1]),
                    true, true, false, "sequential");
        }
    }

    bool loadTopologyYaml(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        bool in_edges = false;
        bool have_edge = false;
        int from = -1;
        int to = -1;
        double cost = 0.0;
        bool trusted = true;
        bool bidirectional = true;
        bool blocked = false;
        std::string source = "manual";
        int loaded = 0;

        auto flush_edge = [&]() {
            if (have_edge && validIndex(from) && validIndex(to))
            {
                addEdge(from, to, cost, trusted, bidirectional, blocked, source);
                loaded++;
            }
            have_edge = false;
            from = -1;
            to = -1;
            cost = 0.0;
            trusted = true;
            bidirectional = true;
            blocked = false;
            source = "manual";
        };

        std::string raw_line;
        while (std::getline(file, raw_line))
        {
            std::string line = trim(raw_line);
            if (line.empty() || startsWith(line, "#"))
            {
                continue;
            }
            if (line == "edges:")
            {
                in_edges = true;
                continue;
            }
            if (!in_edges)
            {
                continue;
            }

            if (startsWith(line, "- from:"))
            {
                flush_edge();
                have_edge = true;
                from = std::stoi(afterColon(line));
            }
            else if (startsWith(line, "from:"))
            {
                have_edge = true;
                from = std::stoi(afterColon(line));
            }
            else if (startsWith(line, "to:"))
            {
                to = std::stoi(afterColon(line));
            }
            else if (startsWith(line, "cost:"))
            {
                cost = std::stod(afterColon(line));
            }
            else if (startsWith(line, "trusted:"))
            {
                trusted = parseBool(afterColon(line), true);
            }
            else if (startsWith(line, "bidirectional:"))
            {
                bidirectional = parseBool(afterColon(line), true);
            }
            else if (startsWith(line, "blocked:"))
            {
                blocked = parseBool(afterColon(line), false);
            }
            else if (startsWith(line, "source:"))
            {
                source = afterColon(line);
            }
        }
        flush_edge();
        return loaded > 0;
    }

    bool loadTopoFromTxt()
    {
        std::ifstream file(joinPath(maps_dir_, "topo.txt"));
        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        int loaded = 0;
        while (std::getline(file, line))
        {
            line = trim(line);
            if (line.empty())
            {
                continue;
            }

            std::istringstream iss(line);
            std::string node_str, neighbors_str;
            if (std::getline(iss, node_str, ':') && std::getline(iss, neighbors_str))
            {
                const int node_id = std::stoi(node_str);
                std::stringstream nss(neighbors_str);
                std::string neighbor;
                while (std::getline(nss, neighbor, ','))
                {
                    neighbor = trim(neighbor);
                    if (!neighbor.empty())
                    {
                        const int to = std::stoi(neighbor);
                        addEdge(node_id, to, 0.0, true, true, false, "legacy_txt");
                        loaded++;
                    }
                }
            }
        }
        return loaded > 0;
    }

    std::size_t edgeCount() const
    {
        std::size_t count = 0;
        for (const auto &item : graph)
        {
            count += item.second.size();
        }
        return count;
    }

    bool edgeIsBlocked(const TopoEdge &edge) const
    {
        if (edge.configured_blocked)
        {
            return true;
        }
        return !edge.blocked_until.isZero() &&
               ros::WallTime::now() < edge.blocked_until;
    }

    double nextTemporaryUnblockDelay() const
    {
        const ros::WallTime now = ros::WallTime::now();
        double delay = std::numeric_limits<double>::infinity();
        for (const auto &item : graph)
        {
            for (const TopoEdge &edge : item.second)
            {
                if (!edge.configured_blocked && !edge.blocked_until.isZero() &&
                    now < edge.blocked_until)
                {
                    delay = std::min(delay, (edge.blocked_until - now).toSec());
                }
            }
        }
        return delay;
    }

    bool waitForTemporaryPath(int start, int target, std::vector<int> &path)
    {
        if (blocked_wait_timeout_ <= 0.0)
        {
            return false;
        }

        const ros::WallTime wait_start = ros::WallTime::now();
        bool announced = false;
        while (ros::ok() && !stop_and_quit)
        {
            path = dijkstraShortestPath(start, target);
            if (!path.empty())
            {
                ROS_INFO("A temporary topology block expired; retry path to P%d.", target);
                return true;
            }

            const double next_delay = nextTemporaryUnblockDelay();
            if (!std::isfinite(next_delay))
            {
                return false;
            }

            const double elapsed = (ros::WallTime::now() - wait_start).toSec();
            if (elapsed >= blocked_wait_timeout_)
            {
                ROS_ERROR("Timed out after %.1f seconds waiting for a temporary topology block to expire.",
                          blocked_wait_timeout_);
                return false;
            }

            if (!announced)
            {
                ROS_WARN("No alternate topology path. Stop and wait up to %.1f seconds for a temporary block to expire.",
                         blocked_wait_timeout_);
                announced = true;
            }
            stopRobot();
            ros::WallDuration(std::min(0.2, std::max(0.01, next_delay))).sleep();
        }
        return false;
    }

    std::vector<int> dijkstraShortestPath(int start, int goal)
    {
        if (!validIndex(start) || !validIndex(goal))
        {
            return {};
        }
        if (start == goal)
        {
            return {start};
        }

        const int n = target_poses.size();
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> parent(n, -1);
        typedef std::pair<double, int> QueueItem;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> q;

        dist[start] = 0.0;
        q.push(QueueItem(0.0, start));

        while (!q.empty())
        {
            const double cost = q.top().first;
            const int node = q.top().second;
            q.pop();
            if (cost > dist[node])
            {
                continue;
            }
            if (node == goal)
            {
                break;
            }

            auto it = graph.find(node);
            if (it == graph.end())
            {
                continue;
            }
            for (const TopoEdge &edge : it->second)
            {
                if (edgeIsBlocked(edge) || !validIndex(edge.to))
                {
                    continue;
                }
                const double next_cost = cost + edge.cost;
                if (next_cost < dist[edge.to])
                {
                    dist[edge.to] = next_cost;
                    parent[edge.to] = node;
                    q.push(QueueItem(next_cost, edge.to));
                }
            }
        }

        if (parent[goal] < 0)
        {
            return {};
        }

        std::vector<int> path;
        for (int at = goal; at >= 0; at = parent[at])
        {
            path.push_back(at);
            if (at == start)
            {
                break;
            }
        }
        if (path.back() != start)
        {
            return {};
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void blockEdge(int from, int to)
    {
        if (!validIndex(from) || !validIndex(to))
        {
            return;
        }

        int blocked_count = 0;
        double longest_cooldown = 0.0;
        auto block_one_direction = [&](int a, int b) {
            auto it = graph.find(a);
            if (it == graph.end())
            {
                return;
            }
            for (TopoEdge &edge : it->second)
            {
                if (edge.to == b)
                {
                    if (edge.configured_blocked)
                    {
                        continue;
                    }
                    edge.failure_count++;
                    const double cooldown = std::min(
                        blocked_cooldown_initial_ *
                            std::pow(blocked_backoff_factor_, edge.failure_count - 1),
                        blocked_cooldown_max_);
                    edge.blocked_until = ros::WallTime::now() + ros::WallDuration(cooldown);
                    longest_cooldown = std::max(longest_cooldown, cooldown);
                    blocked_count++;
                }
            }
        };

        block_one_direction(from, to);
        if (block_bidirectional_)
        {
            block_one_direction(to, from);
        }

        ROS_WARN("Temporarily blocked topology edge P%d -> P%d%s for %.1f seconds (%d directed edges).",
                 from, to, block_bidirectional_ ? " bidirectional" : "",
                 longest_cooldown, blocked_count);
    }

    void markEdgeSuccess(int from, int to)
    {
        auto clear_one_direction = [&](int a, int b) {
            auto it = graph.find(a);
            if (it == graph.end())
            {
                return;
            }
            for (TopoEdge &edge : it->second)
            {
                if (edge.to == b)
                {
                    edge.blocked_until = ros::WallTime();
                    edge.failure_count = 0;
                }
            }
        };

        clear_one_direction(from, to);
        if (block_bidirectional_)
        {
            clear_one_direction(to, from);
        }
    }

    std::string formatPathMessage(const std::string &prefix, const std::vector<int> &path)
    {
        std::string message = prefix;
        for (int index : path)
        {
            message += " -> P" + std::to_string(index);
            if (!target_poses[index].label.empty())
            {
                message += "(" + target_poses[index].label + ")";
            }
        }
        return message;
    }

    void publishTopologyPath(const std::vector<int> &path_indices)
    {
        active_path_ = path_indices;
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        for (int index : path_indices)
        {
            if (validIndex(index))
            {
                path.poses.push_back(toPoseStamped(target_poses[index]));
            }
        }
        path_pub_.publish(path);
        publishTopologyMarkers();
    }

    void publishTopologyMarkers()
    {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker default_edges =
            makeMarker("topology_default_edges", 0, visualization_msgs::Marker::LINE_LIST);
        setMarkerScale(default_edges, 0.035);
        setMarkerColor(default_edges, 1.0, 0.82, 0.05, 0.75);

        std::set<std::pair<int, int>> emitted_edges;
        for (const auto &item : graph)
        {
            const int from = item.first;
            for (const TopoEdge &edge : item.second)
            {
                if (!validIndex(from) || !validIndex(edge.to) || edgeIsBlocked(edge))
                {
                    continue;
                }
                const std::pair<int, int> key(std::min(from, edge.to), std::max(from, edge.to));
                if (emitted_edges.count(key))
                {
                    continue;
                }
                emitted_edges.insert(key);
                default_edges.points.push_back(pointForIndex(from));
                default_edges.points.push_back(pointForIndex(edge.to));
            }
        }

        visualization_msgs::Marker default_nodes =
            makeMarker("topology_default_nodes", 1, visualization_msgs::Marker::SPHERE_LIST);
        setMarkerScale(default_nodes, 0.24);
        setMarkerColor(default_nodes, 1.0, 0.82, 0.05, 0.90);
        for (int i = 0; i < static_cast<int>(target_poses.size()); ++i)
        {
            default_nodes.points.push_back(pointForIndex(i));
        }

        visualization_msgs::Marker selected_edges =
            makeMarker("topology_selected_edges", 2, visualization_msgs::Marker::LINE_LIST);
        setMarkerScale(selected_edges, 0.065);
        setMarkerColor(selected_edges, 0.0, 0.85, 0.20, 0.95);
        for (std::size_t i = 1; i < active_path_.size(); ++i)
        {
            if (!validIndex(active_path_[i - 1]) || !validIndex(active_path_[i]))
            {
                continue;
            }
            selected_edges.points.push_back(pointForIndex(active_path_[i - 1]));
            selected_edges.points.push_back(pointForIndex(active_path_[i]));
        }

        visualization_msgs::Marker selected_nodes =
            makeMarker("topology_selected_nodes", 3, visualization_msgs::Marker::SPHERE_LIST);
        setMarkerScale(selected_nodes, 0.32);
        setMarkerColor(selected_nodes, 0.0, 0.85, 0.20, 0.95);
        for (int index : active_path_)
        {
            if (validIndex(index))
            {
                selected_nodes.points.push_back(pointForIndex(index));
            }
        }

        visualization_msgs::Marker next_node =
            makeMarker("topology_next_goal", 4, visualization_msgs::Marker::SPHERE);
        setMarkerScale(next_node, 0.46);
        setMarkerColor(next_node, 1.0, 0.05, 0.02, 1.0);
        if (validIndex(active_next_index_))
        {
            next_node.pose.position = pointForIndex(active_next_index_);
        }
        else
        {
            next_node.action = visualization_msgs::Marker::DELETE;
        }

        markers.markers.push_back(default_edges);
        markers.markers.push_back(default_nodes);
        markers.markers.push_back(selected_edges);
        markers.markers.push_back(selected_nodes);
        markers.markers.push_back(next_node);
        marker_array_pub_.publish(markers);
    }

    void initializeGlobalAC()
    {
        global_ac = new MVClient("move_base", true);
        ROS_INFO("Waiting for move_base action server to start...");
        global_ac->waitForServer();
        ROS_INFO("Connected to move_base action server");
    }

    enum ReferenceStatusCode
    {
        REFERENCE_ACTIVE = 1,
        REFERENCE_PASSED = 2,
        REFERENCE_PATH_DEVIATED = 3
    };

    enum GoalMonitorResult
    {
        GOAL_REACHED,
        GOAL_FAILED,
        GOAL_PATH_DEVIATED,
        GOAL_PAUSED,
        GOAL_PAUSED_AFTER_PASS
    };

    void referenceStatusCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(reference_status_mutex_);
        reference_status_stamp_ = msg->header.stamp;
        reference_status_path_index_ = static_cast<int>(std::lround(msg->vector.y));
        reference_status_code_ = static_cast<int>(std::lround(msg->vector.z));
    }

    void resetReferenceStatus()
    {
        std::lock_guard<std::mutex> lock(reference_status_mutex_);
        reference_status_stamp_ = ros::Time(0);
        reference_status_path_index_ = -1;
        reference_status_code_ = 0;
    }

    bool referenceStatusMatches(int topology_path_index, int status_code,
                                const ros::Time &goal_start_time)
    {
        std::lock_guard<std::mutex> lock(reference_status_mutex_);
        return reference_status_stamp_ >= goal_start_time &&
               reference_status_path_index_ == topology_path_index &&
               reference_status_code_ == status_code;
    }

    bool referenceTrackingOrPassed(int topology_path_index,
                                   const ros::Time &goal_start_time)
    {
        std::lock_guard<std::mutex> lock(reference_status_mutex_);
        return reference_status_stamp_ >= goal_start_time &&
               reference_status_path_index_ == topology_path_index &&
               (reference_status_code_ == REFERENCE_ACTIVE ||
                reference_status_code_ == REFERENCE_PASSED);
    }

    GoalMonitorResult sendGoalAndMonitor(int target_index, int previous_index,
                                         bool final_goal, int topology_path_index)
    {
        const TargetPose &target_pose = target_poses[target_index];
        move_base_msgs::MoveBaseGoal mb_goal;
        if (final_goal)
        {
            mb_goal.target_pose = toPoseStamped(target_pose);
        }
        else if (validIndex(previous_index))
        {
            mb_goal.target_pose = toPoseStampedWithYaw(target_pose,
                                                       yawBetween(previous_index, target_index));
        }
        else
        {
            mb_goal.target_pose = toPoseStamped(target_pose);
        }

        active_next_index_ = target_index;
        resetReferenceStatus();
        publishTopologyMarkers();
        ROS_INFO_STREAM("Sending topology " << (final_goal ? "final" : "pass-through")
                                            << " goal P" << target_index << ": ("
                                            << target_pose.x << ", " << target_pose.y << ")");
        const ros::Time goal_start_time = ros::Time::now();
        global_ac->sendGoal(mb_goal);

        geometry_msgs::PoseStamped last_progress_pose;
        bool have_progress_pose = getCurrentRobotPose(last_progress_pose);
        ros::Time last_progress_time = ros::Time::now();
        const ros::Time start_time = last_progress_time;

        while (ros::ok())
        {
            if (stop_and_quit)
            {
                global_ac->cancelAllGoals();
                stopRobot();
                return GOAL_FAILED;
            }

            if (pause_reentry_requested_.load())
            {
                global_ac->cancelGoal();
                stopRobot();
                while (pause_robot.load() && ros::ok())
                {
                    stopRobot();
                    ros::Duration(0.2).sleep();
                }
                if (!ros::ok())
                {
                    return GOAL_FAILED;
                }

                const bool current_goal_passed =
                    referenceStatusMatches(topology_path_index,
                                           REFERENCE_PASSED,
                                           goal_start_time);
                pause_reentry_requested_.store(false);
                active_next_index_ = -1;
                publishTopologyMarkers();
                ROS_INFO("Resume via topology re-entry after pausing at path index %d%s.",
                         topology_path_index,
                         current_goal_passed ? " (current waypoint already passed)" : "");
                return current_goal_passed ? GOAL_PAUSED_AFTER_PASS : GOAL_PAUSED;
            }

            if (referenceStatusMatches(topology_path_index,
                                       REFERENCE_PATH_DEVIATED,
                                       goal_start_time))
            {
                ROS_WARN("B-spline path deviation at topology path index %d; "
                         "stop the current action and hand re-entry to topology.",
                         topology_path_index);
                global_ac->cancelGoal();
                stopRobot();
                active_next_index_ = -1;
                publishTopologyMarkers();
                return GOAL_PATH_DEVIATED;
            }

            if (global_ac->waitForResult(ros::Duration(0.2)))
            {
                // pause() cancels the action from the independent Joy callback
                // queue. Handle that cancellation as a resumable topology
                // re-entry instead of a failed/blocked edge.
                if (pause_reentry_requested_.load())
                {
                    continue;
                }
                if (referenceStatusMatches(topology_path_index,
                                           REFERENCE_PATH_DEVIATED,
                                           goal_start_time))
                {
                    global_ac->cancelGoal();
                    stopRobot();
                    active_next_index_ = -1;
                    publishTopologyMarkers();
                    return GOAL_PATH_DEVIATED;
                }
                const actionlib::SimpleClientGoalState state = global_ac->getState();
                if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    geometry_msgs::PoseStamped current_pose;
                    if (!final_goal &&
                        validate_pass_through_action_success_distance_ &&
                        getCurrentRobotPose(current_pose))
                    {
                        const double target_distance = distanceToNode(current_pose, target_index);
                        if (target_distance > waypoint_reached_distance_)
                        {
                            ROS_WARN("move_base reported P%d reached, but robot is %.2f m away. Retry same topology goal.",
                                     target_index, target_distance);
                            global_ac->sendGoal(mb_goal);
                            continue;
                        }
                    }
                    else if (!final_goal && getCurrentRobotPose(current_pose))
                    {
                        const double target_distance = distanceToNode(current_pose, target_index);
                        ROS_INFO("Pass-through P%d accepted by move_base action success at %.2f m "
                                 "from topo point; reference curve may not pass exactly through the waypoint.",
                                 target_index, target_distance);
                    }
                    ROS_INFO_STREAM("Reached topology goal P" << target_index);
                    current_pose_index = target_index;
                    active_next_index_ = -1;
                    publishTopologyMarkers();
                    return GOAL_REACHED;
                }
                ROS_WARN_STREAM("Goal P" << target_index << " failed with state: " << state.toString());
                while (ros::ok() &&
                       (ros::Time::now() - last_progress_time).toSec() < blocked_timeout_)
                {
                    if (stop_and_quit)
                    {
                        stopRobot();
                        return GOAL_FAILED;
                    }
                    stopRobot();
                    ros::Duration(0.2).sleep();
                }
                return GOAL_FAILED;
            }

            if (!final_goal &&
                referenceStatusMatches(topology_path_index, REFERENCE_PASSED,
                                       goal_start_time))
            {
                ROS_INFO("B-spline pass-through P%d reached at topology path index %d; "
                         "send the next goal without cancelling or stopping.",
                         target_index, topology_path_index);
                current_pose_index = target_index;
                active_next_index_ = -1;
                publishTopologyMarkers();
                return GOAL_REACHED;
            }

            geometry_msgs::PoseStamped current_pose;
            if (getCurrentRobotPose(current_pose))
            {
                const double target_distance = distanceToNode(current_pose, target_index);
                if (!final_goal && target_distance <= waypoint_reached_distance_)
                {
                    const bool bspline_tracking =
                        referenceTrackingOrPassed(topology_path_index, goal_start_time);
                    ROS_INFO("Pass-through waypoint P%d reached by XY distance %.2f m%s.",
                             target_index, target_distance,
                             bspline_tracking ?
                                 "; keep the B-spline action live for seamless preemption" : "");
                    if (!bspline_tracking)
                    {
                        global_ac->cancelGoal();
                    }
                    current_pose_index = target_index;
                    active_next_index_ = -1;
                    publishTopologyMarkers();
                    return GOAL_REACHED;
                }
                if (final_goal && target_distance <= waypoint_reached_distance_)
                {
                    last_progress_time = ros::Time::now();
                }

                if (!have_progress_pose ||
                    poseDistance(current_pose, last_progress_pose) >= progress_distance_ ||
                    poseYawDelta(current_pose, last_progress_pose) >= progress_yaw_)
                {
                    last_progress_pose = current_pose;
                    have_progress_pose = true;
                    last_progress_time = ros::Time::now();
                }
            }

            const ros::Time now = ros::Time::now();
            if ((now - last_progress_time).toSec() >= blocked_timeout_)
            {
                ROS_WARN("No effective motion for %.1f seconds, treat current edge as blocked.",
                         blocked_timeout_);
                global_ac->cancelGoal();
                stopRobot();
                active_next_index_ = -1;
                publishTopologyMarkers();
                return GOAL_FAILED;
            }
            if ((now - start_time).toSec() >= goal_timeout_)
            {
                ROS_WARN("Goal P%d timeout after %.1f seconds.", target_index, goal_timeout_);
                global_ac->cancelGoal();
                stopRobot();
                active_next_index_ = -1;
                publishTopologyMarkers();
                return GOAL_FAILED;
            }
        }

        stopRobot();
        active_next_index_ = -1;
        publishTopologyMarkers();
        return GOAL_FAILED;
    }

    bool runTopologyMission(int target_index, std::vector<int> path_indices)
    {
        int start_index = path_indices.empty() ? nearestPoseIndex() : path_indices.front();
        std::size_t execution_start = 0;
        int replan_count = 0;
        const int max_replans = std::max(3, static_cast<int>(target_poses.size()) * 2);

        while (ros::ok() && replan_count <= max_replans)
        {
            if (path_indices.empty())
            {
                path_indices = dijkstraShortestPath(start_index, target_index);
                if (path_indices.empty())
                {
                    ROS_ERROR("No available topology path to target P%d.", target_index);
                    stopRobot();
                    return false;
                }
                execution_start = 0;
                publishTopologyPath(path_indices);
                ROS_INFO_STREAM(formatPathMessage("replan ok:", path_indices));
            }

            for (std::size_t i = execution_start; i < path_indices.size(); ++i)
            {
                const int next_index = path_indices[i];
                if (!validIndex(next_index))
                {
                    stopRobot();
                    return false;
                }

                const int previous_index = (i == 0) ? -1 : path_indices[i - 1];
                const bool final_goal = next_index == target_index;
                const GoalMonitorResult goal_result =
                    sendGoalAndMonitor(next_index, previous_index, final_goal,
                                       static_cast<int>(i));
                if (goal_result == GOAL_REACHED)
                {
                    if (validIndex(previous_index))
                    {
                        markEdgeSuccess(previous_index, next_index);
                    }
                    if (next_index == target_index)
                    {
                        current_status = 3;
                        active_next_index_ = -1;
                        publishTopologyMarkers();
                        return true;
                    }
                    continue;
                }

                const bool path_deviated = goal_result == GOAL_PATH_DEVIATED;
                const bool paused = goal_result == GOAL_PAUSED ||
                                    goal_result == GOAL_PAUSED_AFTER_PASS;
                const bool needs_forward_reentry = path_deviated || paused;

                if (needs_forward_reentry)
                {
                    std::vector<int> reentry_path;
                    std::size_t reentry_execution_start = 0;
                    const bool current_goal_passed =
                        goal_result == GOAL_PAUSED_AFTER_PASS;
                    if (buildForwardReentryPath(path_indices, i,
                                                current_goal_passed,
                                                reentry_path,
                                                reentry_execution_start))
                    {
                        path_indices.swap(reentry_path);
                        execution_start = reentry_execution_start;
                        start_index = path_indices[execution_start];
                        ++replan_count;
                        publishTopologyPath(path_indices);
                        ROS_WARN_STREAM(formatPathMessage(
                            paused ? "topology forward re-entry after pause:"
                                   : "topology forward re-entry after B-spline deviation:",
                            path_indices));
                        break;
                    }
                    ROS_WARN("Forward topology re-entry failed; fall back to nearest reachable node.");
                }

                if (!needs_forward_reentry && validIndex(previous_index))
                {
                    blockEdge(previous_index, next_index);
                }

                start_index = nearestPoseIndex();
                if (!validIndex(start_index))
                {
                    start_index = current_pose_index;
                }
                path_indices = dijkstraShortestPath(start_index, target_index);
                execution_start = 0;
                ++replan_count;
                if (path_indices.empty())
                {
                    if (!waitForTemporaryPath(start_index, target_index, path_indices))
                    {
                        ROS_ERROR("Target P%d remains unreachable after waiting for temporary blocks.",
                                  target_index);
                        stopRobot();
                        return false;
                    }
                }
                publishTopologyPath(path_indices);
                ROS_WARN_STREAM(formatPathMessage(
                    needs_forward_reentry ? "nearest-node topology re-entry fallback:"
                                          : "replan after blocked edge:",
                    path_indices));
                break;
            }
        }

        ROS_ERROR("Exceeded topology replan limit for target P%d.", target_index);
        stopRobot();
        return false;
    }

    void run_planed_pathnode(const std::vector<int> &path_indices)
    {
        if (path_indices.empty())
        {
            return;
        }
        runTopologyMission(path_indices.back(), path_indices);
    }

    void startRun_()
    {
        if (target_poses.empty())
        {
            return;
        }

        std::vector<int> path_indices;
        for (int i = 0; i < static_cast<int>(target_poses.size()); ++i)
        {
            path_indices.push_back(i);
        }
        run_planed_pathnode(path_indices);
    }

    void start()
    {
        if (numofpnts <= 0)
        {
            return;
        }
        runth_ = std::make_unique<std::thread>(&mynav::startRun_, this);
    }

    void pause()
    {
        pause_robot.store(true);
        pause_reentry_requested_.store(true);
        global_ac->cancelAllGoals();
        stopRobot();
    }

    void resume()
    {
        if (pause_robot.exchange(false))
        {
            ROS_INFO("Resume topology navigation.");
        }
    }

    void exitgoals()
    {
        global_ac->cancelAllGoals();
        stop_and_quit = true;
        stopRobot();
    }

    void stopRobot()
    {
        vel_pub_.publish(geometry_msgs::Twist());
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        const bool Apressed = joy->buttons[0];
        const bool Xpressed = joy->buttons[2];
        const bool Ypressed = joy->buttons[3];
        if (Apressed)
        {
            ROS_INFO("A pressed, pause.");
            pause();
        }

        if (Xpressed)
        {
            ROS_INFO("X pressed, resume.");
            resume();
        }

        if (Ypressed)
        {
            ROS_INFO("Y pressed, start or resume.");
            pause_robot.store(false);
            if ((nullptr == runth_) || (current_status == 3))
            {
                start();
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_pose_loader");

    mynav current_nav;
    current_nav.loadNavPnts();
    current_nav.loadStaticMap();
    current_nav.loadTopology();

    while (ros::ok())
    {
        current_nav.publishNavPointsMarkers();
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
