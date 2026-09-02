#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <collision_monitor/collision_checker.h>
#include <cmd_vel_arbiter/FinishMotion.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <x2bot_teleop/NavConfig.h>
#include <x2bot_teleop/EdgeBlockState.h>
#include <x2bot_teleop/SetInt.h>
#include <x2bot_teleop/terminal_goal_policy.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
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
    double length;
    double min_clearance;
    bool trusted;
    bool bidirectional;
    bool configured_blocked;
    ros::WallTime blocked_until;
    ros::WallTime failure_decay_at;
    int failure_count;
    std::string source;
};

struct NavConfigValues {
    double blocked_timeout;
    double blocked_cooldown_initial;
    double blocked_cooldown_max;
    double blocked_backoff_factor;
    double blocked_wait_timeout;
    double goal_timeout;
    bool block_bidirectional;
    double waypoint_reached_distance;
    double fixed_route_final_xy_tolerance;
};

struct LocalSegmentCost {
    bool blocked = false;
    double penalty = 0.0;
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MVClient;

namespace
{
enum TopologySafetyPhase : uint8_t
{
    TOPOLOGY_SAFETY_NORMAL = 0,
    TOPOLOGY_SAFETY_START_SEGMENT = 1,
    TOPOLOGY_SAFETY_FINAL_SEGMENT = 2
};

const char *topologySafetyPhaseName(uint8_t phase)
{
    switch (phase)
    {
    case TOPOLOGY_SAFETY_NORMAL:
        return "NORMAL";
    case TOPOLOGY_SAFETY_START_SEGMENT:
        return "START_SEGMENT";
    case TOPOLOGY_SAFETY_FINAL_SEGMENT:
        return "FINAL_SEGMENT";
    default:
        return "INVALID";
    }
}

void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus &status,
                        const std::string &key, const std::string &value)
{
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
}
}

class mynav
{
public:
    mynav()
        : global_ac(nullptr),
          nh_(),
          tf_listener_(tf_buffer_),
          joy_spinner_(1, &joy_callback_queue_),
          reference_status_spinner_(1, &reference_status_queue_),
          safety_phase_spinner_(1, &safety_phase_callback_queue_),
          current_pose_index(0),
          numofpnts(0),
          current_pnt(0),
          stop_and_quit(false),
          pause_robot(false),
          pause_reentry_requested_(false),
          cancel_requested_(false),
          current_status(0),
          blocked_timeout_(10.0),
          blocked_cooldown_initial_(60.0),
          blocked_cooldown_max_(180.0),
          blocked_backoff_factor_(2.0),
          blocked_wait_timeout_(240.0),
          progress_distance_(0.05),
          progress_yaw_(6.0 * M_PI / 180.0),
          waypoint_reached_distance_(0.20),
          fixed_route_final_xy_tolerance_(0.03),
          legacy_waypoint_reached_distance_(0.05),
          validate_pass_through_action_success_distance_(false),
          goal_timeout_(120.0),
          terminal_yaw_wait_timeout_(60.0),
          block_bidirectional_(true),
          static_map_loaded_(false),
          static_map_width_(0),
          static_map_height_(0),
          static_map_resolution_(0.05),
          static_map_origin_x_(0.0),
          static_map_origin_y_(0.0),
          static_map_origin_yaw_(0.0),
          static_map_occupied_thresh_(0.65),
          static_map_free_thresh_(0.196),
          static_map_negate_(false),
          static_map_unknown_is_obstacle_(true),
          static_map_inflation_radius_(std::hypot(0.36, 0.25) + 0.15),
          local_replan_map_timeout_(0.30),
          local_replan_block_threshold_(100),
          local_replan_cost_weight_(2.0)
    {
        ros::NodeHandle private_nh("~");
        maps_dir_ = defaultMapsDir();
        private_nh.param("maps_dir", maps_dir_, maps_dir_);
        normalizeMapsDir();
        ROS_INFO_STREAM("Topology navigation maps_dir: " << maps_dir_);
        loadSavedNavConfig(joinPath(maps_dir_, "autonav_params.yaml"));
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
        private_nh.param("fixed_route_final_xy_tolerance",
                         fixed_route_final_xy_tolerance_,
                         fixed_route_final_xy_tolerance_);
        private_nh.param("legacy_waypoint_reached_distance",
                         legacy_waypoint_reached_distance_,
                         legacy_waypoint_reached_distance_);
        legacy_waypoint_reached_distance_ =
            std::max(0.01, legacy_waypoint_reached_distance_);
        private_nh.param("validate_pass_through_action_success_distance",
                         validate_pass_through_action_success_distance_,
                         validate_pass_through_action_success_distance_);
        private_nh.param("goal_timeout", goal_timeout_, goal_timeout_);
        private_nh.param("terminal_yaw_wait_timeout",
                         terminal_yaw_wait_timeout_,
                         terminal_yaw_wait_timeout_);
        terminal_yaw_wait_timeout_ =
            std::max(1.0, terminal_yaw_wait_timeout_);
        private_nh.param("block_bidirectional", block_bidirectional_, block_bidirectional_);
        private_nh.param("static_map_inflation_radius", static_map_inflation_radius_,
                         static_map_inflation_radius_);
        private_nh.param("local_replan_map_topic", local_replan_map_topic_,
                         std::string("/mxb_move_base/local_costmap/costmap"));
        private_nh.param("local_replan_map_timeout", local_replan_map_timeout_,
                         local_replan_map_timeout_);
        private_nh.param("local_replan_block_threshold",
                         local_replan_block_threshold_,
                         local_replan_block_threshold_);
        private_nh.param("local_replan_cost_weight", local_replan_cost_weight_,
                         local_replan_cost_weight_);
        local_replan_map_timeout_ = std::max(0.05, local_replan_map_timeout_);
        local_replan_block_threshold_ =
            std::max(1, std::min(100, local_replan_block_threshold_));
        local_replan_cost_weight_ = std::max(0.0, local_replan_cost_weight_);
        configureLocalReplanFootprint();

        initializeGlobalAC();
        std::string cmd_vel_topic;
        private_nh.param("cmd_vel_topic", cmd_vel_topic,
                         std::string("/cmd_vel/nav"));
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
        private_nh.param("topology_safety_phase_topic",
                         topology_safety_phase_topic_,
                         std::string("/anav/topology_safety_phase"));
        private_nh.param("topology_safety_phase_rate",
                         topology_safety_phase_rate_, 10.0);
        topology_safety_phase_rate_ =
            std::max(2.0, topology_safety_phase_rate_);
        topology_safety_phase_pub_ = nh_.advertise<std_msgs::UInt8>(
            topology_safety_phase_topic_, 1, false);
        safety_phase_nh_.setCallbackQueue(&safety_phase_callback_queue_);
        safety_phase_timer_ = safety_phase_nh_.createWallTimer(
            ros::WallDuration(1.0 / topology_safety_phase_rate_),
            &mynav::topologySafetyPhaseTimerCallback, this);
        safety_phase_spinner_.start();
        setTopologySafetyPhase(TOPOLOGY_SAFETY_NORMAL);
        fixed_route_mode_pub_ =
            nh_.advertise<std_msgs::Bool>("/anav/fixed_route_mode", 1, true);
        publishFixedRouteMode(false);
        finish_motion_client_ =
            nh_.serviceClient<cmd_vel_arbiter::FinishMotion>(
                "/cmd_vel_arbiter/finish_motion");
        ros::SubscribeOptions joy_options =
            ros::SubscribeOptions::create<sensor_msgs::Joy>(
                "joy", 10, boost::bind(&mynav::joyCallback, this, _1),
                ros::VoidPtr(), &joy_callback_queue_);
        joy_sub_ = nh_.subscribe(joy_options);
        control_nh_.setCallbackQueue(&joy_callback_queue_);
        cancel_navigation_service_ = control_nh_.advertiseService(
            "/anav/cancel_navigation", &mynav::cancelNavigationCallback, this);
        nav_config_service_ = control_nh_.advertiseService(
            "/anav/nav_config", &mynav::navConfigCallback, this);
        // Reload mutates the graph and therefore runs on the main callback
        // queue, serialized with planning and marker publication.
        reload_topology_service_ = nh_.advertiseService(
            "/anav/reload_topology", &mynav::reloadTopologyCallback, this);
        joy_spinner_.start();
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("topology_markers", 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("topology_plan", 1, true);
        task_status_pub_ = nh_.advertise<std_msgs::String>("/anav/task_status", 10, true);
        diagnostics_pub_ =
            nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
        ros::SubscribeOptions status_options =
            ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(
                "/bspline_status", 1,
                boost::bind(&mynav::referenceStatusCallback, this, _1),
                ros::VoidPtr(), &reference_status_queue_);
        reference_status_sub_ = nh_.subscribe(status_options);
        ros::SubscribeOptions terminal_options =
            ros::SubscribeOptions::create<std_msgs::UInt8>(
                "/anav/terminal_motion_state", 1,
                boost::bind(&mynav::terminalMotionStateCallback, this, _1),
                ros::VoidPtr(), &reference_status_queue_);
        terminal_motion_state_sub_ = nh_.subscribe(terminal_options);
        ros::SubscribeOptions control_mode_options =
            ros::SubscribeOptions::create<std_msgs::UInt8>(
                "/anav/path_control_mode", 1,
                boost::bind(&mynav::pathControlModeCallback, this, _1),
                ros::VoidPtr(), &reference_status_queue_);
        path_control_mode_sub_ = nh_.subscribe(control_mode_options);
        reference_status_spinner_.start();

        ros::SubscribeOptions local_map_options =
            ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>(
                local_replan_map_topic_, 1,
                boost::bind(&mynav::localMapCallback, this, _1),
                ros::VoidPtr(), &reference_status_queue_);
        local_map_sub_ = nh_.subscribe(local_map_options);

        plan_path_service = nh_.advertiseService("plan_path_and_go", &mynav::planPathCallback, this);
        edge_block_state_service_ = control_nh_.advertiseService(
            "/anav/edge_block_state", &mynav::edgeBlockStateCallback, this);
        ROS_INFO("Topology navigation service /plan_path_and_go started.");
    }

    ~mynav()
    {
        setTopologySafetyPhase(TOPOLOGY_SAFETY_NORMAL);
        safety_phase_timer_.stop();
        safety_phase_spinner_.stop();
        joy_spinner_.stop();
        reference_status_spinner_.stop();
        if (runth_ && runth_->joinable())
        {
            runth_->join();
        }
        delete global_ac;
    }

    void publishDiagnostic(uint8_t level, const std::string &code,
                           const std::string &message,
                           const std::string &detail,
                           const std::string &action,
                           bool active,
                           const std::string &name = "/anav/navigation")
    {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status;
        status.level = level;
        status.name = name;
        status.hardware_id = "ranger";
        status.message = message;
        addDiagnosticValue(status, "code", code);
        addDiagnosticValue(status, "active", active ? "true" : "false");
        addDiagnosticValue(status, "kind", active ? "FAULT" : "STATE");
        addDiagnosticValue(status, "detail", detail);
        addDiagnosticValue(status, "action", action);
        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    bool planPathCallback(x2bot_teleop::SetInt::Request &req,
                          x2bot_teleop::SetInt::Response &res)
    {
        const int target_index = resolvePoseIndex(req.data);
        const int requested_current = resolvePoseIndex(req.currentID);
        const bool exec_path = req.run > 0;
        const bool fixed_route = req.run >= 2;
        const std::string target_name = requestedTargetName(req.data);

        if (exec_path && cancel_requested_.exchange(false))
        {
            res.success = false;
            res.message = "导航任务在启动前已取消";
            stopRobot();
            publishTaskStatus("canceled", req.data, target_name, res.message);
            return true;
        }

        if (!validIndex(target_index))
        {
            res.success = false;
            res.message = "目标点序号无效";
            publishTaskStatus("failed", req.data, target_name, res.message);
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
            publishTaskStatus("failed", req.data, target_name, res.message);
            ROS_ERROR("Invalid current index request: %d", req.currentID);
            return true;
        }

        if (exec_path)
        {
            publishTaskStatus("running", req.data, target_name, "正在规划导航路径");
        }

        current_pose_index = start_index;
        std::vector<int> path_indices =
            dijkstraShortestPath(start_index, target_index, fixed_route);
        if (path_indices.empty() && exec_path && !fixed_route)
        {
            waitForTemporaryPath(start_index, target_index, path_indices);
        }
        if (path_indices.empty())
        {
            res.success = false;
            res.message = "无法找到从 P" + std::to_string(start_index) +
                          " 到 P" + std::to_string(target_index) + " 的拓扑路径";
            publishTaskStatus("failed", req.data, target_name, res.message);
            ROS_ERROR_STREAM(res.message);
            stopRobot();
            return true;
        }

        publishTopologyPath(path_indices);
        res.message = formatPathMessage("plan ok:", path_indices);
        ROS_INFO_STREAM(res.message);

        if (exec_path)
        {
            publishFixedRouteMode(fixed_route);
            setTopologySafetyPhase(TOPOLOGY_SAFETY_START_SEGMENT);
            {
                std::lock_guard<std::mutex> lock(nav_config_mutex_);
                navigation_active_.store(true);
            }
            const bool ok = runTopologyMission(
                target_index, path_indices, fixed_route);
            {
                std::lock_guard<std::mutex> lock(nav_config_mutex_);
                navigation_active_.store(false);
            }
            publishFixedRouteMode(false);
            const bool canceled = cancel_requested_.exchange(false);
            const bool centered = notifyMotionFinished(
                ok ? cmd_vel_arbiter::FinishMotionRequest::TASK_FINISHED
                   : (canceled
                          ? cmd_vel_arbiter::FinishMotionRequest::TASK_CANCELED
                          : cmd_vel_arbiter::FinishMotionRequest::TASK_FAILED));
            setTopologySafetyPhase(TOPOLOGY_SAFETY_NORMAL);
            res.success = ok;
            if (ok)
            {
                res.message = "arrived:" + formatPathMessage("", path_indices);
                if (!centered)
                {
                    res.message += "; steering centering failed";
                }
                publishTaskStatus("arrived", req.data, target_name, res.message);
            }
            else if (canceled)
            {
                res.message = "导航任务已取消";
                if (!centered)
                {
                    res.message += "，但轮组回正失败";
                }
                publishTaskStatus("canceled", req.data, target_name, res.message);
            }
            else
            {
                res.message = "目标暂时不可达，已停车";
                if (!centered)
                {
                    res.message += "，但轮组回正失败";
                }
                publishTaskStatus("failed", req.data, target_name, res.message);
            }
        }
        else
        {
            res.success = true;
            publishTaskStatus("planned", req.data, target_name, res.message);
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
        if (!fingerprintFiles({positions_file}, loaded_positions_fingerprint_))
        {
            ROS_ERROR_STREAM("Failed to fingerprint " << positions_file);
            return false;
        }
        ROS_INFO("Loaded %d navigation points.", numofpnts);
        publishNavPointsMarkers();
        return numofpnts > 0;
    }

    bool loadTopology()
    {
        const std::map<int, std::vector<TopoEdge>> previous_graph = graph;
        graph.clear();
        if (loadTopologyYaml(joinPath(maps_dir_, "topology.yaml")))
        {
            ROS_INFO_STREAM("Loaded topology.yaml with " << edgeCount() << " directed edges.");
            return true;
        }
        graph = previous_graph;
        ROS_ERROR("topology.yaml is missing, stale, or unsafe; refusing implicit sequential fallback.");
        return false;
    }

    bool loadStaticMap()
    {
        const std::string map_yaml = joinPath(maps_dir_, "map.yaml");
        static_map_yaml_path_ = map_yaml;
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
            else if (startsWith(line, "free_thresh:"))
            {
                static_map_free_thresh_ = std::stod(afterColon(line));
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
        static_map_image_path_ = pgm_file;

        static_map_occupied_.assign(pixels.size(), 0);
        int unknown_cells = 0;
        for (int my = 0; my < static_map_height_; ++my)
        {
            const int raster_y = static_map_height_ - 1 - my;
            for (int mx = 0; mx < static_map_width_; ++mx)
            {
                const std::size_t raster_index = raster_y * static_map_width_ + mx;
                const std::size_t map_index = my * static_map_width_ + mx;
                const double normalized =
                    static_cast<double>(pixels[raster_index]) / std::max(1, max_value);
                const double occupancy =
                    static_map_negate_ ? normalized : (1.0 - normalized);
                const bool occupied = occupancy >= static_map_occupied_thresh_;
                const bool unknown = !occupied && occupancy > static_map_free_thresh_;
                if (unknown)
                {
                    ++unknown_cells;
                }
                static_map_occupied_[map_index] =
                    occupied || (static_map_unknown_is_obstacle_ && unknown) ? 1 : 0;
            }
        }
        for (int mx = 0; mx < static_map_width_; ++mx)
        {
            static_map_occupied_[mx] = 1;
            static_map_occupied_[(static_map_height_ - 1) * static_map_width_ + mx] = 1;
        }
        for (int my = 0; my < static_map_height_; ++my)
        {
            static_map_occupied_[my * static_map_width_] = 1;
            static_map_occupied_[my * static_map_width_ + static_map_width_ - 1] = 1;
        }
        inflateStaticMap();

        if (!fingerprintFiles({static_map_yaml_path_, static_map_image_path_},
                              loaded_static_map_fingerprint_))
        {
            ROS_ERROR("Failed to fingerprint the loaded static map.");
            return false;
        }

        static_map_loaded_ = true;
        ROS_INFO("Loaded static map for topology connect check: %dx%d, resolution %.3f, inflation %.3f m, unknown=%d.",
                 static_map_width_, static_map_height_, static_map_resolution_,
                 static_map_inflation_radius_, unknown_cells);
        return true;
    }

private:
    MVClient *global_ac;
    std::unique_ptr<std::thread> runth_;

    ros::NodeHandle nh_;
    ros::NodeHandle control_nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher vel_pub_;
    ros::Publisher fixed_route_mode_pub_;
    ros::CallbackQueue joy_callback_queue_;
    ros::AsyncSpinner joy_spinner_;
    ros::Subscriber joy_sub_;
    ros::Publisher marker_pub;
    ros::Publisher marker_array_pub_;
    ros::Publisher path_pub_;
    ros::Publisher task_status_pub_;
    ros::Publisher diagnostics_pub_;
    ros::CallbackQueue reference_status_queue_;
    ros::AsyncSpinner reference_status_spinner_;
    ros::Subscriber reference_status_sub_;
    ros::Subscriber terminal_motion_state_sub_;
    ros::Subscriber path_control_mode_sub_;
    ros::Subscriber local_map_sub_;
    std::mutex local_map_mutex_;
    nav_msgs::OccupancyGrid local_replan_map_;
    ros::WallTime local_replan_map_received_at_;
    bool have_local_replan_map_ = false;
    ros::NodeHandle safety_phase_nh_;
    ros::CallbackQueue safety_phase_callback_queue_;
    ros::AsyncSpinner safety_phase_spinner_;
    ros::WallTimer safety_phase_timer_;
    ros::Publisher topology_safety_phase_pub_;
    std::mutex topology_safety_phase_mutex_;
    uint8_t topology_safety_phase_ = TOPOLOGY_SAFETY_NORMAL;
    std::string topology_safety_phase_topic_;
    double topology_safety_phase_rate_ = 10.0;
    std::mutex reference_status_mutex_;
    ros::Time reference_status_stamp_;
    int reference_status_path_index_ = -1;
    int reference_status_code_ = 0;
    std::mutex terminal_motion_state_mutex_;
    ros::Time terminal_motion_state_stamp_;
    uint8_t terminal_motion_state_ = x2bot_teleop::TERMINAL_TRACKING;
    std::mutex path_control_mode_mutex_;
    ros::Time path_control_mode_stamp_;
    uint8_t path_control_mode_ = x2bot_teleop::PATH_CONTROL_UNKNOWN;
    ros::ServiceServer plan_path_service;
    ros::ServiceServer cancel_navigation_service_;
    ros::ServiceServer nav_config_service_;
    ros::ServiceServer reload_topology_service_;
    ros::ServiceServer edge_block_state_service_;
    ros::ServiceClient finish_motion_client_;

    int current_pose_index;
    std::vector<TargetPose> target_poses;
    std::map<std::string, int> workstation_indices;
    int numofpnts;
    int current_pnt;
    bool stop_and_quit;
    std::atomic<bool> pause_robot;
    std::atomic<bool> pause_reentry_requested_;
    std::atomic<bool> cancel_requested_;
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
    double fixed_route_final_xy_tolerance_;
    double legacy_waypoint_reached_distance_;
    bool validate_pass_through_action_success_distance_;
    double goal_timeout_;
    double terminal_yaw_wait_timeout_;
    bool block_bidirectional_;
    std::atomic<bool> navigation_active_{false};
    std::mutex nav_config_mutex_;
    mutable std::mutex edge_block_mutex_;
    std::vector<int> active_path_;
    int active_next_index_ = -1;
    std::string maps_dir_;
    bool static_map_loaded_;
    int static_map_width_;
    int static_map_height_;
    double static_map_resolution_;
    double static_map_origin_x_;
    double static_map_origin_y_;
    double static_map_origin_yaw_;
    double static_map_occupied_thresh_;
    double static_map_free_thresh_;
    bool static_map_negate_;
    bool static_map_unknown_is_obstacle_;
    double static_map_inflation_radius_;
    std::vector<unsigned char> static_map_occupied_;
    std::string static_map_yaml_path_;
    std::string static_map_image_path_;
    std::string loaded_positions_fingerprint_;
    std::string loaded_static_map_fingerprint_;
    std::string local_replan_map_topic_;
    double local_replan_map_timeout_;
    int local_replan_block_threshold_;
    double local_replan_cost_weight_;
    collision_monitor::CollisionChecker local_replan_collision_checker_;
    collision_monitor::GridPolicy local_replan_grid_policy_;
    mutable std::mutex local_replan_footprint_mutex_;
    bool local_replan_footprint_ready_ = false;
    int local_replan_footprint_source_priority_ = 0;
    ros::WallTime local_replan_footprint_retry_at_;
    double local_replan_max_corner_step_ = 0.025;

    bool notifyMotionFinished(uint8_t reason)
    {
        cmd_vel_arbiter::FinishMotion service;
        service.request.source = "nav";
        service.request.reason = reason;
        if (!finish_motion_client_.call(service))
        {
            ROS_ERROR("Failed to call /cmd_vel_arbiter/finish_motion for navigation.");
            return false;
        }
        if (!service.response.centered)
        {
            ROS_ERROR_STREAM("Navigation stopped, but centering was not confirmed: "
                             << service.response.message);
        }
        return service.response.centered;
    }

    void publishFixedRouteMode(bool enabled)
    {
        ros::param::set("/anav/fixed_route_mode", enabled);
        std_msgs::Bool mode;
        mode.data = enabled;
        fixed_route_mode_pub_.publish(mode);
        ROS_INFO("Fixed-route navigation mode %s.", enabled ? "enabled" : "disabled");
    }

    void publishTopologySafetyPhase()
    {
        std::lock_guard<std::mutex> lock(topology_safety_phase_mutex_);
        std_msgs::UInt8 message;
        message.data = topology_safety_phase_;
        topology_safety_phase_pub_.publish(message);
    }

    void topologySafetyPhaseTimerCallback(const ros::WallTimerEvent &)
    {
        publishTopologySafetyPhase();
    }

    void setTopologySafetyPhase(uint8_t phase)
    {
        uint8_t previous;
        {
            std::lock_guard<std::mutex> lock(topology_safety_phase_mutex_);
            previous = topology_safety_phase_;
            topology_safety_phase_ = phase;
            std_msgs::UInt8 message;
            message.data = phase;
            topology_safety_phase_pub_.publish(message);
        }
        if (previous != phase)
        {
            ROS_INFO("Topology safety phase: %s -> %s.",
                     topologySafetyPhaseName(previous),
                     topologySafetyPhaseName(phase));
        }
    }

    void selectTopologySafetyPhase(bool start_segment_active,
                                   bool final_goal)
    {
        if (final_goal)
        {
            setTopologySafetyPhase(TOPOLOGY_SAFETY_FINAL_SEGMENT);
        }
        else if (start_segment_active)
        {
            setTopologySafetyPhase(TOPOLOGY_SAFETY_START_SEGMENT);
        }
        else
        {
            setTopologySafetyPhase(TOPOLOGY_SAFETY_NORMAL);
        }
    }

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

    static std::string unquote(const std::string &value)
    {
        if (value.size() >= 2 &&
            ((value.front() == '"' && value.back() == '"') ||
             (value.front() == '\'' && value.back() == '\'')))
        {
            return value.substr(1, value.size() - 2);
        }
        return value;
    }

    static bool fingerprintFiles(const std::vector<std::string> &filenames,
                                 std::string &fingerprint)
    {
        const uint64_t offset = UINT64_C(14695981039346656037);
        const uint64_t prime = UINT64_C(1099511628211);
        uint64_t value = offset;
        char buffer[64 * 1024];
        for (const std::string &filename : filenames)
        {
            std::ifstream file(filename, std::ios::binary);
            if (!file.is_open())
            {
                return false;
            }
            while (file.good())
            {
                file.read(buffer, sizeof(buffer));
                const std::streamsize count = file.gcount();
                for (std::streamsize i = 0; i < count; ++i)
                {
                    value ^= static_cast<unsigned char>(buffer[i]);
                    value *= prime;
                }
            }
        }
        std::ostringstream stream;
        stream << std::hex << std::setfill('0') << std::setw(16) << value;
        fingerprint = stream.str();
        return true;
    }

    static std::string defaultMapsDir()
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0')
        {
            std::string home_dir(home);
            while (home_dir.size() > 1 && home_dir[home_dir.size() - 1] == '/')
            {
                home_dir.erase(home_dir.size() - 1);
            }
            return home_dir + "/maps";
        }

        ROS_WARN("HOME is not set; falling back to /home/nav/maps.");
        return "/home/nav/maps";
    }

    void normalizeMapsDir()
    {
        if (!maps_dir_.empty() && maps_dir_[maps_dir_.size() - 1] == '/')
        {
            maps_dir_.erase(maps_dir_.size() - 1);
        }
    }

    NavConfigValues currentNavConfig() const
    {
        NavConfigValues config;
        config.blocked_timeout = blocked_timeout_;
        config.blocked_cooldown_initial = blocked_cooldown_initial_;
        config.blocked_cooldown_max = blocked_cooldown_max_;
        config.blocked_backoff_factor = blocked_backoff_factor_;
        config.blocked_wait_timeout = blocked_wait_timeout_;
        config.goal_timeout = goal_timeout_;
        config.block_bidirectional = block_bidirectional_;
        config.waypoint_reached_distance = waypoint_reached_distance_;
        config.fixed_route_final_xy_tolerance = fixed_route_final_xy_tolerance_;
        return config;
    }

    static bool validateNavConfig(const NavConfigValues &config,
                                  std::string &reason)
    {
        const double values[] = {
            config.blocked_timeout,
            config.blocked_cooldown_initial,
            config.blocked_cooldown_max,
            config.blocked_backoff_factor,
            config.blocked_wait_timeout,
            config.goal_timeout,
            config.waypoint_reached_distance,
            config.fixed_route_final_xy_tolerance};
        for (double value : values)
        {
            if (!std::isfinite(value))
            {
                reason = "参数必须是有限数值";
                return false;
            }
        }
        if (config.blocked_timeout < 0.1 || config.blocked_timeout > 3600.0)
        {
            reason = "无有效运动超时必须在 0.1 到 3600 秒之间";
            return false;
        }
        if (config.blocked_cooldown_initial < 0.1 ||
            config.blocked_cooldown_initial > 3600.0)
        {
            reason = "首次阻塞禁用时间必须在 0.1 到 3600 秒之间";
            return false;
        }
        if (config.blocked_cooldown_max < config.blocked_cooldown_initial ||
            config.blocked_cooldown_max > 86400.0)
        {
            reason = "最大阻塞禁用时间不得小于首次禁用时间，且不得超过 86400 秒";
            return false;
        }
        if (config.blocked_backoff_factor < 1.0 ||
            config.blocked_backoff_factor > 10.0)
        {
            reason = "阻塞退避倍数必须在 1.0 到 10.0 之间";
            return false;
        }
        if (config.blocked_wait_timeout < 0.0 ||
            config.blocked_wait_timeout > 86400.0)
        {
            reason = "无替代路线等待时间必须在 0 到 86400 秒之间";
            return false;
        }
        if (config.goal_timeout < 0.1 || config.goal_timeout > 86400.0)
        {
            reason = "单目标超时必须在 0.1 到 86400 秒之间";
            return false;
        }
        if (config.waypoint_reached_distance < 0.01 ||
            config.waypoint_reached_distance > 2.0)
        {
            reason = "中间点到达距离必须在 0.01 到 2.0 米之间";
            return false;
        }
        if (config.fixed_route_final_xy_tolerance < 0.005 ||
            config.fixed_route_final_xy_tolerance > 1.0)
        {
            reason = "固定路线终点容差必须在 0.005 到 1.0 米之间";
            return false;
        }
        reason.clear();
        return true;
    }

    void applyNavConfig(const NavConfigValues &config)
    {
        blocked_timeout_ = config.blocked_timeout;
        blocked_cooldown_initial_ = config.blocked_cooldown_initial;
        blocked_cooldown_max_ = config.blocked_cooldown_max;
        blocked_backoff_factor_ = config.blocked_backoff_factor;
        blocked_wait_timeout_ = config.blocked_wait_timeout;
        goal_timeout_ = config.goal_timeout;
        block_bidirectional_ = config.block_bidirectional;
        waypoint_reached_distance_ = config.waypoint_reached_distance;
        fixed_route_final_xy_tolerance_ =
            config.fixed_route_final_xy_tolerance;
    }

    void fillNavConfigResponse(x2bot_teleop::NavConfig::Response &response)
    {
        const NavConfigValues config = currentNavConfig();
        response.navigation_active = navigation_active_.load();
        response.blocked_timeout = config.blocked_timeout;
        response.blocked_cooldown_initial = config.blocked_cooldown_initial;
        response.blocked_cooldown_max = config.blocked_cooldown_max;
        response.blocked_backoff_factor = config.blocked_backoff_factor;
        response.blocked_wait_timeout = config.blocked_wait_timeout;
        response.goal_timeout = config.goal_timeout;
        response.block_bidirectional = config.block_bidirectional;
        response.waypoint_reached_distance = config.waypoint_reached_distance;
        response.fixed_route_final_xy_tolerance =
            config.fixed_route_final_xy_tolerance;
    }

    bool navConfigCallback(x2bot_teleop::NavConfig::Request &request,
                           x2bot_teleop::NavConfig::Response &response)
    {
        std::lock_guard<std::mutex> lock(nav_config_mutex_);
        if (!request.apply)
        {
            response.success = true;
            response.message = "已读取 AutoNAV 当前参数";
            fillNavConfigResponse(response);
            return true;
        }
        if (navigation_active_.load())
        {
            response.success = false;
            response.message = "导航任务正在执行，请停止任务后再应用参数";
            fillNavConfigResponse(response);
            return true;
        }

        NavConfigValues config;
        config.blocked_timeout = request.blocked_timeout;
        config.blocked_cooldown_initial = request.blocked_cooldown_initial;
        config.blocked_cooldown_max = request.blocked_cooldown_max;
        config.blocked_backoff_factor = request.blocked_backoff_factor;
        config.blocked_wait_timeout = request.blocked_wait_timeout;
        config.goal_timeout = request.goal_timeout;
        config.block_bidirectional = request.block_bidirectional;
        config.waypoint_reached_distance = request.waypoint_reached_distance;
        config.fixed_route_final_xy_tolerance =
            request.fixed_route_final_xy_tolerance;
        std::string reason;
        if (!validateNavConfig(config, reason))
        {
            response.success = false;
            response.message = reason;
            fillNavConfigResponse(response);
            return true;
        }

        applyNavConfig(config);
        response.success = true;
        response.message = "AutoNAV 参数已应用到当前节点";
        fillNavConfigResponse(response);
        ROS_INFO("Applied AutoNAV runtime configuration from GUI.");
        return true;
    }

    bool reloadTopologyCallback(std_srvs::Trigger::Request &,
                                std_srvs::Trigger::Response &response)
    {
        std::lock_guard<std::mutex> lock(nav_config_mutex_);
        if (navigation_active_.load())
        {
            response.success = false;
            response.message = "导航任务正在执行，拒绝热加载拓扑";
            return true;
        }
        {
            std::lock_guard<std::mutex> edge_lock(edge_block_mutex_);
            if (!loadTopology())
            {
                response.success = false;
                response.message = "拓扑校验失败，继续使用上一版本";
                return true;
            }
        }
        active_path_.clear();
        active_next_index_ = -1;
        publishTopologyMarkers();
        response.success = true;
        response.message = "拓扑已校验并热加载";
        return true;
    }

    int effectiveFailureCount(const TopoEdge &edge,
                              const ros::WallTime &now) const
    {
        if (edge.failure_count <= 0)
        {
            return 0;
        }
        if (edge.failure_decay_at.isZero() || now < edge.failure_decay_at)
        {
            return edge.failure_count;
        }
        const double decay_period = std::max(0.1, blocked_cooldown_initial_);
        const int decay_steps = 1 + static_cast<int>(
            std::floor((now - edge.failure_decay_at).toSec() / decay_period));
        return std::max(0, edge.failure_count - decay_steps);
    }

    bool edgeBlockStateCallback(
        x2bot_teleop::EdgeBlockState::Request &request,
        x2bot_teleop::EdgeBlockState::Response &response)
    {
        if (request.clear)
        {
            if (navigation_active_.load())
            {
                response.success = false;
                response.message = "导航任务正在执行，不能清除边封锁记录";
                return true;
            }
        }

        std::lock_guard<std::mutex> edge_lock(edge_block_mutex_);

        if (request.clear)
        {
            bool found = false;
            bool clear_reverse = false;
            auto clear_direction = [&](int from, int to) {
                auto graph_it = graph.find(from);
                if (graph_it == graph.end())
                {
                    return;
                }
                for (TopoEdge &edge : graph_it->second)
                {
                    if (edge.to != to)
                    {
                        continue;
                    }
                    found = true;
                    clear_reverse = clear_reverse || edge.bidirectional;
                    edge.blocked_until = ros::WallTime();
                    edge.failure_decay_at = ros::WallTime();
                    edge.failure_count = 0;
                }
            };
            clear_direction(request.from_id, request.to_id);
            if (clear_reverse)
            {
                clear_direction(request.to_id, request.from_id);
            }
            if (!found)
            {
                response.success = false;
                response.message = "指定的拓扑边不存在";
                return true;
            }
            ROS_INFO("Cleared runtime block history for topology edge P%d %s P%d.",
                     request.from_id, clear_reverse ? "<->" : "->",
                     request.to_id);
            response.message = "已清除指定边的运行时封锁次数";
        }

        const ros::WallTime now = ros::WallTime::now();
        for (const auto &item : graph)
        {
            const int from = item.first;
            for (const TopoEdge &edge : item.second)
            {
                if (edge.bidirectional && from > edge.to)
                {
                    continue;
                }

                int failure_count = effectiveFailureCount(edge, now);
                bool configured_blocked = edge.configured_blocked;
                double remaining_seconds = edgeIsBlocked(edge)
                    ? std::max(0.0, (edge.blocked_until - now).toSec()) : 0.0;
                std::string source = edge.source;
                if (edge.bidirectional)
                {
                    const auto reverse_it = graph.find(edge.to);
                    if (reverse_it != graph.end())
                    {
                        for (const TopoEdge &reverse : reverse_it->second)
                        {
                            if (reverse.to == from)
                            {
                                failure_count = std::max(
                                    failure_count,
                                    effectiveFailureCount(reverse, now));
                                configured_blocked = configured_blocked ||
                                                     reverse.configured_blocked;
                                if (edgeIsBlocked(reverse))
                                {
                                    remaining_seconds = std::max(
                                        remaining_seconds,
                                        std::max(0.0,
                                            (reverse.blocked_until - now).toSec()));
                                }
                                break;
                            }
                        }
                    }
                }
                response.from_ids.push_back(from);
                response.to_ids.push_back(edge.to);
                response.bidirectional.push_back(edge.bidirectional);
                response.configured_blocked.push_back(configured_blocked);
                response.failure_counts.push_back(failure_count);
                response.remaining_seconds.push_back(remaining_seconds);
                response.sources.push_back(source);
            }
        }
        response.success = true;
        if (!request.clear)
        {
            response.message = "已读取拓扑边封锁状态";
        }
        return true;
    }

    void loadSavedNavConfig(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            ROS_INFO_STREAM("AutoNAV parameter file not found; use defaults: "
                            << filename);
            return;
        }

        NavConfigValues config = currentNavConfig();
        std::string line;
        try
        {
            while (std::getline(file, line))
            {
                const std::size_t comment = line.find('#');
                if (comment != std::string::npos)
                {
                    line.erase(comment);
                }
                const std::size_t separator = line.find(':');
                if (separator == std::string::npos)
                {
                    continue;
                }
                const std::string key = trim(line.substr(0, separator));
                const std::string value = trim(line.substr(separator + 1));
                if (key == "blocked_timeout")
                    config.blocked_timeout = std::stod(value);
                else if (key == "blocked_cooldown_initial")
                    config.blocked_cooldown_initial = std::stod(value);
                else if (key == "blocked_cooldown_max")
                    config.blocked_cooldown_max = std::stod(value);
                else if (key == "blocked_backoff_factor")
                    config.blocked_backoff_factor = std::stod(value);
                else if (key == "blocked_wait_timeout")
                    config.blocked_wait_timeout = std::stod(value);
                else if (key == "goal_timeout")
                    config.goal_timeout = std::stod(value);
                else if (key == "block_bidirectional")
                    config.block_bidirectional =
                        parseBool(value, config.block_bidirectional);
                else if (key == "waypoint_reached_distance")
                    config.waypoint_reached_distance = std::stod(value);
                else if (key == "fixed_route_final_xy_tolerance")
                    config.fixed_route_final_xy_tolerance = std::stod(value);
            }
        }
        catch (const std::exception &error)
        {
            ROS_ERROR_STREAM("Invalid AutoNAV parameter file " << filename
                             << ": " << error.what() << "; use defaults.");
            return;
        }

        std::string reason;
        if (!validateNavConfig(config, reason))
        {
            ROS_ERROR_STREAM("Rejected AutoNAV parameter file " << filename
                             << ": " << reason << "; use defaults.");
            return;
        }
        applyNavConfig(config);
        ROS_INFO_STREAM("Loaded AutoNAV parameters from " << filename);
    }

    void parseMapOrigin(const std::string &value)
    {
        std::string cleaned = value;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
        std::istringstream iss(cleaned);
        iss >> static_map_origin_x_ >> static_map_origin_y_ >> static_map_origin_yaw_;
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

    static void squaredDistanceTransform1D(const std::vector<double> &input,
                                           std::vector<double> &output)
    {
        const int size = static_cast<int>(input.size());
        output.resize(size);
        if (size == 0)
        {
            return;
        }
        std::vector<int> locations(size);
        std::vector<double> boundaries(size + 1);
        int envelope = 0;
        locations[0] = 0;
        boundaries[0] = -std::numeric_limits<double>::infinity();
        boundaries[1] = std::numeric_limits<double>::infinity();
        for (int q = 1; q < size; ++q)
        {
            double intersection = 0.0;
            while (true)
            {
                const int location = locations[envelope];
                intersection =
                    ((input[q] + static_cast<double>(q) * q) -
                     (input[location] + static_cast<double>(location) * location)) /
                    (2.0 * (q - location));
                if (intersection > boundaries[envelope] || envelope == 0)
                {
                    break;
                }
                --envelope;
            }
            ++envelope;
            locations[envelope] = q;
            boundaries[envelope] = intersection;
            boundaries[envelope + 1] = std::numeric_limits<double>::infinity();
        }
        envelope = 0;
        for (int q = 0; q < size; ++q)
        {
            while (boundaries[envelope + 1] < q)
            {
                ++envelope;
            }
            const double delta = q - locations[envelope];
            output[q] = delta * delta + input[locations[envelope]];
        }
    }

    void inflateStaticMap()
    {
        if (static_map_occupied_.empty() || static_map_inflation_radius_ <= 0.0)
        {
            return;
        }

        // Exact Euclidean distance transform, matching build_topology.py.
        // A finite sentinel avoids inf-inf when a complete row/column is free.
        const double far_distance = 1e12;
        std::vector<double> intermediate(static_map_occupied_.size(), far_distance);
        std::vector<double> input(static_map_height_);
        std::vector<double> output;
        for (int x = 0; x < static_map_width_; ++x)
        {
            for (int y = 0; y < static_map_height_; ++y)
            {
                input[y] = static_map_occupied_[y * static_map_width_ + x]
                    ? 0.0 : far_distance;
            }
            squaredDistanceTransform1D(input, output);
            for (int y = 0; y < static_map_height_; ++y)
            {
                intermediate[y * static_map_width_ + x] = output[y];
            }
        }

        const double radius_cells =
            static_map_inflation_radius_ / std::max(0.001, static_map_resolution_);
        const double radius_squared = radius_cells * radius_cells;
        input.resize(static_map_width_);
        for (int y = 0; y < static_map_height_; ++y)
        {
            for (int x = 0; x < static_map_width_; ++x)
            {
                input[x] = intermediate[y * static_map_width_ + x];
            }
            squaredDistanceTransform1D(input, output);
            for (int x = 0; x < static_map_width_; ++x)
            {
                static_map_occupied_[y * static_map_width_ + x] =
                    output[x] + 1e-9 < radius_squared ? 1 : 0;
            }
        }
    }

    bool worldToStaticMap(double wx, double wy, int &mx, int &my) const
    {
        const double dx = wx - static_map_origin_x_;
        const double dy = wy - static_map_origin_y_;
        const double cos_yaw = std::cos(static_map_origin_yaw_);
        const double sin_yaw = std::sin(static_map_origin_yaw_);
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        mx = static_cast<int>(std::floor(local_x / static_map_resolution_));
        my = static_cast<int>(std::floor(local_y / static_map_resolution_));
        return mx >= 0 && my >= 0 && mx < static_map_width_ && my < static_map_height_;
    }

    bool staticMapSegmentFree(double x0, double y0, double x1, double y1) const
    {
        if (!static_map_loaded_)
        {
            return false;
        }

        const double distance = std::hypot(x1 - x0, y1 - y0);
        const double step = std::max(0.005, static_map_resolution_ * 0.5);
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

    std::string requestedTargetName(int requested_index) const
    {
        if (requested_index < 0)
        {
            return "工位 W" + std::to_string(-requested_index);
        }
        return "导航点 " + std::to_string(requested_index + 1);
    }

    std::string cleanStatusField(std::string value) const
    {
        std::replace(value.begin(), value.end(), '\t', ' ');
        std::replace(value.begin(), value.end(), '\n', ' ');
        std::replace(value.begin(), value.end(), '\r', ' ');
        return value;
    }

    void publishTaskStatus(const std::string &state, int requested_index,
                           const std::string &target_name, const std::string &detail)
    {
        std_msgs::String status;
        status.data = cleanStatusField(state) + "\t" + std::to_string(requested_index) +
                      "\t" + cleanStatusField(target_name) + "\t" + cleanStatusField(detail);
        task_status_pub_.publish(status);
        if (state == "failed")
        {
            publishDiagnostic(
                diagnostic_msgs::DiagnosticStatus::ERROR,
                "ANAV-NAV-012", "导航任务执行失败", detail,
                "查看目标点、拓扑路径、局部规划和障碍安全状态。", true);
        }
        else if (state == "running" || state == "arrived")
        {
            publishDiagnostic(
                diagnostic_msgs::DiagnosticStatus::OK,
                "ANAV-NAV-000", "导航任务状态正常", detail, "", false);
        }
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

    bool parseFootprint(const XmlRpc::XmlRpcValue &value,
                        std::vector<geometry_msgs::Point> &footprint,
                        std::string &reason) const
    {
        footprint.clear();
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            value.size() < 3)
        {
            reason = "footprint must contain at least three [x, y] points";
            return false;
        }
        for (int i = 0; i < value.size(); ++i)
        {
            if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
                value[i].size() < 2)
            {
                reason = "each footprint point must be an [x, y] array";
                return false;
            }
            auto read_number = [](const XmlRpc::XmlRpcValue &item,
                                  double &number) {
                if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    number = static_cast<double>(item);
                    return std::isfinite(number);
                }
                if (item.getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    number = static_cast<int>(item);
                    return true;
                }
                return false;
            };
            geometry_msgs::Point point;
            if (!read_number(value[i][0], point.x) ||
                !read_number(value[i][1], point.y))
            {
                reason = "footprint coordinates must be finite numbers";
                return false;
            }
            point.z = 0.0;
            footprint.push_back(point);
        }
        return true;
    }

    bool installLocalReplanFootprint(
        const std::vector<geometry_msgs::Point> &footprint,
        const std::string &footprint_source, int source_priority)
    {
        double footprint_padding = 0.10;
        ros::param::param("/collision_monitor/navigation_footprint_padding",
                          footprint_padding, footprint_padding);
        footprint_padding = std::max(0.0, footprint_padding);
        collision_monitor::GridPolicy grid_policy;
        ros::param::param("/collision_monitor/local_occupied_threshold",
                          grid_policy.occupied_threshold,
                          local_replan_block_threshold_);
        ros::param::param("/collision_monitor/local_unknown_is_obstacle",
                          grid_policy.unknown_is_obstacle, false);
        ros::param::param("/collision_monitor/local_outside_is_obstacle",
                          grid_policy.outside_is_obstacle, false);
        double max_corner_step = 0.025;
        ros::param::param("/collision_monitor/max_corner_step",
                          max_corner_step, max_corner_step);
        grid_policy.occupied_threshold = std::max(
            0, std::min(100, grid_policy.occupied_threshold));
        max_corner_step = std::max(0.005, max_corner_step);

        collision_monitor::CollisionChecker checker;
        std::string error;
        if (!checker.setFootprint(footprint, footprint_padding, &error))
        {
            ROS_ERROR("Cannot install local topology replanning footprint from %s: %s.",
                      footprint_source.c_str(), error.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(local_replan_footprint_mutex_);
            local_replan_collision_checker_ = checker;
            local_replan_grid_policy_ = grid_policy;
            local_replan_max_corner_step_ = max_corner_step;
            local_replan_footprint_ready_ = true;
            local_replan_footprint_source_priority_ = source_priority;
        }
        ROS_INFO("Local topology replanning uses %s with %.3f m navigation padding "
                 "and collision threshold %d.",
                 footprint_source.c_str(), footprint_padding,
                 grid_policy.occupied_threshold);
        return true;
    }

    bool tryLocalReplanFootprintParameter(
        const std::string &footprint_source, int source_priority,
        bool warn_invalid)
    {
        XmlRpc::XmlRpcValue footprint_value;
        if (!ros::param::get(footprint_source, footprint_value))
        {
            return false;
        }

        std::vector<geometry_msgs::Point> footprint;
        std::string error;
        if (!parseFootprint(footprint_value, footprint, error))
        {
            if (warn_invalid)
            {
                ROS_WARN("Invalid %s for local topology replanning: %s; trying a safe fallback.",
                         footprint_source.c_str(), error.c_str());
            }
            else
            {
                ROS_WARN_THROTTLE(
                    10.0,
                    "Invalid %s for local topology replanning: %s; keep the current footprint.",
                    footprint_source.c_str(), error.c_str());
            }
            return false;
        }
        return installLocalReplanFootprint(
            footprint, footprint_source, source_priority);
    }

    void configureLocalReplanFootprint()
    {
        if (tryLocalReplanFootprintParameter(
                "/collision_monitor/footprint", 2, true) ||
            tryLocalReplanFootprintParameter(
                "/mxb_move_base/local_costmap/footprint", 1, true))
        {
            return;
        }

        // AutoNAV is also used by itself in tests. Always keep local rerouting
        // available when the other nodes have not loaded their parameters yet.
        std::vector<geometry_msgs::Point> footprint;
        const double coordinates[4][2] = {
            {0.36, 0.25}, {0.36, -0.25},
            {-0.36, -0.25}, {-0.36, 0.25}};
        for (const auto &coordinate : coordinates)
        {
            geometry_msgs::Point point;
            point.x = coordinate[0];
            point.y = coordinate[1];
            footprint.push_back(point);
        }
        ROS_WARN("No valid collision monitor or local costmap footprint is available; "
                 "using the deployed 0.72 x 0.50 m footprint fallback.");
        installLocalReplanFootprint(
            footprint, "built-in robot_footprint.yaml fallback", 0);
    }

    void refreshLocalReplanFootprint()
    {
        const ros::WallTime now = ros::WallTime::now();
        int source_priority = 0;
        {
            std::lock_guard<std::mutex> lock(local_replan_footprint_mutex_);
            source_priority = local_replan_footprint_source_priority_;
            if (source_priority >= 2 ||
                (!local_replan_footprint_retry_at_.isZero() &&
                 now < local_replan_footprint_retry_at_))
            {
                return;
            }
            local_replan_footprint_retry_at_ = now + ros::WallDuration(1.0);
        }

        if (tryLocalReplanFootprintParameter(
                "/collision_monitor/footprint", 2, false))
        {
            return;
        }
        if (source_priority == 0)
        {
            tryLocalReplanFootprintParameter(
                "/mxb_move_base/local_costmap/footprint", 1, false);
        }
    }

    bool localReplanFootprintReady() const
    {
        std::lock_guard<std::mutex> lock(local_replan_footprint_mutex_);
        return local_replan_footprint_ready_;
    }

    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &message)
    {
        refreshLocalReplanFootprint();
        std::lock_guard<std::mutex> lock(local_map_mutex_);
        local_replan_map_ = *message;
        local_replan_map_received_at_ = ros::WallTime::now();
        have_local_replan_map_ = true;
    }

    bool localMapSnapshot(nav_msgs::OccupancyGrid &map,
                          std::string &reason)
    {
        std::lock_guard<std::mutex> lock(local_map_mutex_);
        if (!have_local_replan_map_)
        {
            reason = "local costmap has not been received";
            return false;
        }
        const double age =
            (ros::WallTime::now() - local_replan_map_received_at_).toSec();
        if (!std::isfinite(age) || age > local_replan_map_timeout_)
        {
            std::ostringstream stream;
            stream << "local costmap is stale (age=" << std::fixed
                   << std::setprecision(3) << age << "s)";
            reason = stream.str();
            return false;
        }
        if (local_replan_map_.header.frame_id != "map" &&
            local_replan_map_.header.frame_id != "/map")
        {
            reason = "local costmap frame is not map";
            return false;
        }
        const std::size_t expected =
            static_cast<std::size_t>(local_replan_map_.info.width) *
            local_replan_map_.info.height;
        if (local_replan_map_.info.width == 0 ||
            local_replan_map_.info.height == 0 ||
            !std::isfinite(local_replan_map_.info.resolution) ||
            local_replan_map_.info.resolution <= 0.0 ||
            local_replan_map_.data.size() != expected)
        {
            reason = "local costmap dimensions are invalid";
            return false;
        }
        map = local_replan_map_;
        return true;
    }

    bool worldToLocalMap(const nav_msgs::OccupancyGrid &map,
                         double wx, double wy, int &mx, int &my) const
    {
        const geometry_msgs::Quaternion &q = map.info.origin.orientation;
        const double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double dx = wx - map.info.origin.position.x;
        const double dy = wy - map.info.origin.position.y;
        const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        mx = static_cast<int>(std::floor(local_x / map.info.resolution));
        my = static_cast<int>(std::floor(local_y / map.info.resolution));
        return mx >= 0 && my >= 0 &&
               mx < static_cast<int>(map.info.width) &&
               my < static_cast<int>(map.info.height);
    }

    LocalSegmentCost localSegmentCost(const nav_msgs::OccupancyGrid &map,
                                      double x0, double y0,
                                      double x1, double y1) const
    {
        LocalSegmentCost result;
        std::lock_guard<std::mutex> footprint_lock(
            local_replan_footprint_mutex_);
        if (!local_replan_footprint_ready_)
        {
            result.blocked = true;
            return result;
        }
        const double length = std::hypot(x1 - x0, y1 - y0);
        const double sample_step = std::max(
            0.005, std::min(static_cast<double>(map.info.resolution) * 0.5,
                            local_replan_max_corner_step_));
        const int samples =
            std::max(1, static_cast<int>(std::ceil(length / sample_step)));
        const double integrated_step = length / samples;
        const double segment_yaw =
            length > 1e-9 ? std::atan2(y1 - y0, x1 - x0) : 0.0;
        for (int sample = 0; sample <= samples; ++sample)
        {
            const double ratio = static_cast<double>(sample) / samples;
            const double wx = x0 + (x1 - x0) * ratio;
            const double wy = y0 + (y1 - y0) * ratio;
            collision_monitor::Pose2D footprint_pose;
            footprint_pose.x = wx;
            footprint_pose.y = wy;
            footprint_pose.yaw = segment_yaw;
            if (local_replan_collision_checker_.poseCollides(
                    footprint_pose, map, local_replan_grid_policy_))
            {
                result.blocked = true;
                return result;
            }
            int mx = 0;
            int my = 0;
            if (!worldToLocalMap(map, wx, wy, mx, my))
            {
                // A topo point or most of an edge may be outside the rolling
                // map. Only the currently observed overlap affects replanning.
                continue;
            }
            const int value = map.data[my * map.info.width + mx];
            if (value < 0)
            {
                // Unknown is not a newly observed dynamic obstacle. Static-map
                // and topology clearance checks remain authoritative there.
                continue;
            }
            const double normalized = static_cast<double>(value) / 100.0;
            result.penalty += local_replan_cost_weight_ * normalized * normalized *
                              integrated_step;
        }
        return result;
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
                         bool bidirectional, bool blocked, const std::string &source,
                         double length = 0.0,
                         double min_clearance = std::numeric_limits<double>::infinity())
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
        edge.length = length > 0.0 ? length : poseDistance(target_poses[from], target_poses[to]);
        edge.min_clearance = min_clearance;
        edge.trusted = trusted;
        edge.bidirectional = bidirectional;
        edge.configured_blocked = blocked;
        edge.blocked_until = ros::WallTime();
        edge.failure_decay_at = ros::WallTime();
        edge.failure_count = 0;
        edge.source = source;
        graph[from].push_back(edge);
    }

    void addEdge(int from, int to, double cost, bool trusted,
                 bool bidirectional, bool blocked, const std::string &source,
                 double length = 0.0,
                 double min_clearance = std::numeric_limits<double>::infinity())
    {
        addDirectedEdge(from, to, cost, trusted, bidirectional, blocked, source,
                        length, min_clearance);
        if (bidirectional)
        {
            addDirectedEdge(to, from, cost, trusted, bidirectional, blocked, source,
                            length, min_clearance);
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

    bool topologyStronglyConnected() const
    {
        const int node_count = static_cast<int>(target_poses.size());
        if (node_count == 0)
        {
            return false;
        }
        auto reachable = [&](bool reverse) {
            std::vector<unsigned char> seen(node_count, 0);
            std::queue<int> queue;
            seen[0] = 1;
            queue.push(0);
            while (!queue.empty())
            {
                const int node = queue.front();
                queue.pop();
                if (!reverse)
                {
                    auto it = graph.find(node);
                    if (it == graph.end())
                    {
                        continue;
                    }
                    for (const TopoEdge &edge : it->second)
                    {
                        if (!edge.configured_blocked && validIndex(edge.to) && !seen[edge.to])
                        {
                            seen[edge.to] = 1;
                            queue.push(edge.to);
                        }
                    }
                }
                else
                {
                    for (const auto &item : graph)
                    {
                        for (const TopoEdge &edge : item.second)
                        {
                            if (!edge.configured_blocked && edge.to == node &&
                                validIndex(item.first) && !seen[item.first])
                            {
                                seen[item.first] = 1;
                                queue.push(item.first);
                            }
                        }
                    }
                }
            }
            return std::find(seen.begin(), seen.end(), 0) == seen.end();
        };
        return reachable(false) && reachable(true);
    }

    bool loadTopologyYaml(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        if (!static_map_loaded_)
        {
            ROS_ERROR("Refuse topology load because the static map is unavailable.");
            return false;
        }

        bool in_edges = false;
        bool have_edge = false;
        bool valid = true;
        int schema_version = 0;
        std::string positions_fingerprint;
        std::string map_fingerprint;
        double topology_required_clearance = -1.0;
        int from = -1;
        int to = -1;
        double cost = 0.0;
        double length = 0.0;
        double min_clearance = -1.0;
        bool trusted = true;
        bool bidirectional = true;
        bool blocked = false;
        std::string source = "manual";
        int loaded = 0;
        std::set<std::pair<int, int>> directed_edges;

        auto flush_edge = [&]() {
            if (!have_edge)
            {
                return;
            }
            const double geometric_length =
                validIndex(from) && validIndex(to)
                    ? poseDistance(target_poses[from], target_poses[to]) : 0.0;
            if (!validIndex(from) || !validIndex(to) || from == to ||
                !std::isfinite(cost) || cost <= 0.0 ||
                !std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(min_clearance) ||
                std::fabs(length - geometric_length) > 0.05 ||
                min_clearance + 1e-6 < static_map_inflation_radius_)
            {
                ROS_ERROR("Invalid topology edge P%d -> P%d.", from, to);
                valid = false;
            }
            else if (!staticMapSegmentFree(target_poses[from].x, target_poses[from].y,
                                           target_poses[to].x, target_poses[to].y))
            {
                ROS_ERROR("Topology edge P%d -> P%d fails current static-map clearance check.",
                          from, to);
                valid = false;
            }
            else
            {
                const std::pair<int, int> forward(from, to);
                const std::pair<int, int> reverse(to, from);
                if (directed_edges.count(forward) ||
                    (bidirectional && directed_edges.count(reverse)))
                {
                    ROS_ERROR("Duplicate topology edge P%d -> P%d.", from, to);
                    valid = false;
                }
                else
                {
                    directed_edges.insert(forward);
                    if (bidirectional)
                    {
                        directed_edges.insert(reverse);
                    }
                    addEdge(from, to, cost, trusted, bidirectional, blocked, source,
                            length, min_clearance);
                    loaded++;
                }
            }
            have_edge = false;
            from = -1;
            to = -1;
            cost = 0.0;
            length = 0.0;
            min_clearance = -1.0;
            trusted = true;
            bidirectional = true;
            blocked = false;
            source = "manual";
        };

        try
        {
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
                    if (startsWith(line, "schema_version:"))
                        schema_version = std::stoi(afterColon(line));
                    else if (startsWith(line, "positions_fingerprint:"))
                        positions_fingerprint = unquote(afterColon(line));
                    else if (startsWith(line, "map_fingerprint:"))
                        map_fingerprint = unquote(afterColon(line));
                    else if (startsWith(line, "required_clearance:"))
                        topology_required_clearance = std::stod(afterColon(line));
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
                    to = std::stoi(afterColon(line));
                else if (startsWith(line, "length:"))
                    length = std::stod(afterColon(line));
                else if (startsWith(line, "cost:"))
                    cost = std::stod(afterColon(line));
                else if (startsWith(line, "min_clearance:"))
                    min_clearance = std::stod(afterColon(line));
                else if (startsWith(line, "trusted:"))
                    trusted = parseBool(afterColon(line), true);
                else if (startsWith(line, "bidirectional:"))
                    bidirectional = parseBool(afterColon(line), true);
                else if (startsWith(line, "blocked:"))
                    blocked = parseBool(afterColon(line), false);
                else if (startsWith(line, "source:"))
                    source = unquote(afterColon(line));
            }
            flush_edge();
        }
        catch (const std::exception &error)
        {
            ROS_ERROR_STREAM("Invalid topology yaml " << filename << ": " << error.what());
            return false;
        }

        std::string current_positions_fingerprint;
        std::string current_map_fingerprint;
        const bool fingerprints_read =
            fingerprintFiles({joinPath(maps_dir_, "robot_positions.txt")},
                             current_positions_fingerprint) &&
            fingerprintFiles({static_map_yaml_path_, static_map_image_path_},
                             current_map_fingerprint);
        if (schema_version != 2 || !fingerprints_read ||
            current_positions_fingerprint != loaded_positions_fingerprint_ ||
            current_map_fingerprint != loaded_static_map_fingerprint_ ||
            positions_fingerprint != loaded_positions_fingerprint_ ||
            map_fingerprint != loaded_static_map_fingerprint_ ||
            !std::isfinite(topology_required_clearance) ||
            topology_required_clearance + 1e-6 < static_map_inflation_radius_)
        {
            ROS_ERROR("Topology metadata is stale or incompatible; rebuild topology.yaml.");
            valid = false;
        }
        if (!valid || loaded <= 0 || !topologyStronglyConnected())
        {
            if (valid && loaded > 0)
            {
                ROS_ERROR("Topology is not strongly connected across all configured nodes.");
            }
            return false;
        }
        return true;
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
        std::lock_guard<std::mutex> lock(edge_block_mutex_);
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

    std::vector<int> dijkstraShortestPath(
        int start, int goal, bool fixed_route = false,
        const nav_msgs::OccupancyGrid *local_map = nullptr,
        double *path_cost = nullptr, int *path_failure_score = nullptr,
        bool log_failed_edges = true)
    {
        std::lock_guard<std::mutex> lock(edge_block_mutex_);
        if (path_cost != nullptr)
        {
            *path_cost = std::numeric_limits<double>::infinity();
        }
        if (path_failure_score != nullptr)
        {
            *path_failure_score = std::numeric_limits<int>::max();
        }
        if (!validIndex(start) || !validIndex(goal))
        {
            return {};
        }
        if (start == goal)
        {
            if (path_cost != nullptr)
            {
                *path_cost = 0.0;
            }
            if (path_failure_score != nullptr)
            {
                *path_failure_score = 0;
            }
            return {start};
        }

        const int n = target_poses.size();
        // A temporary block is a hard exclusion until blocked_until. After it
        // expires, failure history becomes a finite route penalty and decays
        // one level per initial-cooldown period. This lets a recovered short
        // edge return to service without immediately retrying it.
        std::vector<int> failed_edge_score(n, std::numeric_limits<int>::max());
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> parent(n, -1);
        typedef std::pair<std::pair<double, int>, int> QueueItem;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> q;

        failed_edge_score[start] = 0;
        dist[start] = 0.0;
        q.push(QueueItem(std::make_pair(0.0, 0), start));

        while (!q.empty())
        {
            const double cost = q.top().first.first;
            const int failure_score = q.top().first.second;
            const int node = q.top().second;
            q.pop();
            if (cost > dist[node] ||
                (cost == dist[node] &&
                 failure_score > failed_edge_score[node]))
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
                const bool blocked = fixed_route
                    ? edge.configured_blocked
                    : edgeIsBlocked(edge);
                if (blocked || !validIndex(edge.to))
                {
                    continue;
                }
                LocalSegmentCost local_cost;
                if (local_map != nullptr)
                {
                    local_cost = localSegmentCost(
                        *local_map,
                        target_poses[node].x, target_poses[node].y,
                        target_poses[edge.to].x, target_poses[edge.to].y);
                    if (local_cost.blocked)
                    {
                        continue;
                    }
                }
                const int edge_failure_score = fixed_route
                    ? 0 : effectiveFailureCount(edge, ros::WallTime::now());
                const int next_failure_score =
                    failure_score + edge_failure_score;
                const double next_cost = cost + edge.cost +
                    edge.cost * edge_failure_score + local_cost.penalty;
                if (next_cost < dist[edge.to] ||
                    (next_cost == dist[edge.to] &&
                     next_failure_score < failed_edge_score[edge.to]))
                {
                    failed_edge_score[edge.to] = next_failure_score;
                    dist[edge.to] = next_cost;
                    parent[edge.to] = node;
                    q.push(QueueItem(
                        std::make_pair(next_cost, next_failure_score), edge.to));
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
        if (path_cost != nullptr)
        {
            *path_cost = dist[goal];
        }
        if (path_failure_score != nullptr)
        {
            *path_failure_score = failed_edge_score[goal];
        }
        if (log_failed_edges && !fixed_route && failed_edge_score[goal] > 0)
        {
            ROS_WARN("Topology path to P%d includes recovering edges "
                     "(failure score=%d).",
                     goal, failed_edge_score[goal]);
        }
        return path;
    }

    bool localAwareReroute(
        int goal, std::vector<int> &path, bool &map_ready,
        std::string &reason,
        const std::map<int, ros::WallTime> *blocked_reentry_nodes = nullptr)
    {
        path.clear();
        map_ready = false;
        if (!validIndex(goal))
        {
            reason = "invalid target topology node";
            return false;
        }
        if (!localReplanFootprintReady())
        {
            reason = "collision-monitor footprint is unavailable";
            return false;
        }

        nav_msgs::OccupancyGrid local_map;
        if (!localMapSnapshot(local_map, reason))
        {
            return false;
        }
        map_ready = true;

        geometry_msgs::PoseStamped robot_pose;
        if (!getCurrentRobotPose(robot_pose))
        {
            reason = "current robot pose is unavailable";
            return false;
        }

        const double robot_x = robot_pose.pose.position.x;
        const double robot_y = robot_pose.pose.position.y;
        const int node_count = static_cast<int>(target_poses.size());
        double connector_radius = 0.0;
        for (const auto &edges : graph)
        {
            for (const TopoEdge &edge : edges.second)
            {
                connector_radius = std::max(connector_radius, edge.length);
            }
        }
        // Use the scale of the topology produced by build_topology.py rather
        // than the rolling costmap size. This permits a candidate outside the
        // local window without allowing an arbitrary long topology jump.
        connector_radius = std::max(0.1, connector_radius);
        int selected_node = -1;
        int selected_failure_score = std::numeric_limits<int>::max();
        double selected_cost = std::numeric_limits<double>::infinity();
        int locally_blocked_connectors = 0;
        int cooldown_blocked_connectors = 0;
        const ros::WallTime now = ros::WallTime::now();
        for (int node = 0; node < node_count; ++node)
        {
            if (blocked_reentry_nodes != nullptr)
            {
                const auto blocked_it = blocked_reentry_nodes->find(node);
                if (blocked_it != blocked_reentry_nodes->end() &&
                    now < blocked_it->second)
                {
                    ++cooldown_blocked_connectors;
                    continue;
                }
            }
            const double connector_length =
                std::hypot(target_poses[node].x - robot_x,
                           target_poses[node].y - robot_y);
            if (connector_length > connector_radius + 1e-9 ||
                !staticMapSegmentFree(robot_x, robot_y,
                                      target_poses[node].x,
                                      target_poses[node].y))
            {
                continue;
            }

            const LocalSegmentCost local_cost = localSegmentCost(
                local_map, robot_x, robot_y,
                target_poses[node].x, target_poses[node].y);
            if (local_cost.blocked)
            {
                ++locally_blocked_connectors;
                continue;
            }

            double topology_cost = 0.0;
            int topology_failure_score = 0;
            const std::vector<int> candidate_path = dijkstraShortestPath(
                node, goal, false, &local_map, &topology_cost,
                &topology_failure_score, false);
            if (candidate_path.empty())
            {
                continue;
            }
            const double total_cost = connector_length + local_cost.penalty +
                                      topology_cost;
            if (topology_failure_score < selected_failure_score ||
                (topology_failure_score == selected_failure_score &&
                 total_cost + 1e-12 < selected_cost))
            {
                selected_node = node;
                selected_failure_score = topology_failure_score;
                selected_cost = total_cost;
                path = candidate_path;
            }
        }

        if (selected_node < 0 || path.empty())
        {
            std::ostringstream stream;
            stream << "no safe route through a topology connector within "
                   << std::fixed << std::setprecision(2)
                   << connector_radius << "m; local map rejected "
                   << locally_blocked_connectors << " connector(s), cooldown rejected "
                   << cooldown_blocked_connectors << " connector(s)";
            reason = stream.str();
            return false;
        }

        ROS_INFO("Local-aware topology replan selected P%d -> P%d: "
                 "score=%.2f, failure_score=%d, local rejected %d connector(s), "
                 "cooldown rejected %d connector(s).",
                 selected_node, goal, selected_cost, selected_failure_score,
                 locally_blocked_connectors, cooldown_blocked_connectors);
        return true;
    }

    void blockEdge(int from, int to)
    {
        std::lock_guard<std::mutex> lock(edge_block_mutex_);
        if (!validIndex(from) || !validIndex(to))
        {
            return;
        }

        int blocked_count = 0;
        double longest_cooldown = 0.0;
        const ros::WallTime now = ros::WallTime::now();
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
                    edge.failure_count = effectiveFailureCount(edge, now) + 1;
                    const double cooldown = std::min(
                        blocked_cooldown_initial_ *
                            std::pow(blocked_backoff_factor_, edge.failure_count - 1),
                        blocked_cooldown_max_);
                    edge.blocked_until = now + ros::WallDuration(cooldown);
                    edge.failure_decay_at = edge.blocked_until +
                        ros::WallDuration(blocked_cooldown_initial_);
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
        std::lock_guard<std::mutex> lock(edge_block_mutex_);
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
                    edge.failure_decay_at = ros::WallTime();
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
        std::lock_guard<std::mutex> lock(edge_block_mutex_);
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
        GOAL_PAUSED_AFTER_PASS,
        GOAL_CANCELED
    };

    void referenceStatusCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(reference_status_mutex_);
        reference_status_stamp_ = msg->header.stamp;
        reference_status_path_index_ = static_cast<int>(std::lround(msg->vector.y));
        reference_status_code_ = static_cast<int>(std::lround(msg->vector.z));
    }

    void terminalMotionStateCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        if (msg->data > x2bot_teleop::TERMINAL_COMPLETE)
        {
            ROS_WARN_THROTTLE(1.0, "Ignore invalid terminal motion state %u.",
                              static_cast<unsigned int>(msg->data));
            return;
        }
        std::lock_guard<std::mutex> lock(terminal_motion_state_mutex_);
        terminal_motion_state_stamp_ = ros::Time::now();
        terminal_motion_state_ = msg->data;
    }

    void resetTerminalMotionState()
    {
        std::lock_guard<std::mutex> lock(terminal_motion_state_mutex_);
        terminal_motion_state_stamp_ = ros::Time(0);
        terminal_motion_state_ = x2bot_teleop::TERMINAL_TRACKING;
    }

    bool currentTerminalMotionState(const ros::Time &goal_start_time,
                                    uint8_t &state)
    {
        std::lock_guard<std::mutex> lock(terminal_motion_state_mutex_);
        if (terminal_motion_state_stamp_ < goal_start_time)
        {
            return false;
        }
        state = terminal_motion_state_;
        return true;
    }

    void pathControlModeCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        if (msg->data > x2bot_teleop::LEGACY_FALLBACK)
        {
            ROS_WARN_THROTTLE(1.0, "Ignore invalid path control mode %u.",
                              static_cast<unsigned int>(msg->data));
            return;
        }
        std::lock_guard<std::mutex> lock(path_control_mode_mutex_);
        path_control_mode_stamp_ = ros::Time::now();
        path_control_mode_ = msg->data;
    }

    void resetPathControlMode()
    {
        std::lock_guard<std::mutex> lock(path_control_mode_mutex_);
        path_control_mode_stamp_ = ros::Time(0);
        path_control_mode_ = x2bot_teleop::PATH_CONTROL_UNKNOWN;
    }

    bool currentPathControlMode(const ros::Time &goal_start_time,
                                uint8_t &mode)
    {
        std::lock_guard<std::mutex> lock(path_control_mode_mutex_);
        if (path_control_mode_stamp_ < goal_start_time)
        {
            return false;
        }
        mode = path_control_mode_;
        return true;
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

    GoalMonitorResult sendGoalAndMonitor(int target_index, int previous_index,
                                         bool final_goal, int topology_path_index,
                                         bool fixed_route = false)
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
        resetTerminalMotionState();
        resetPathControlMode();
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
        x2bot_teleop::TerminalGoalPolicy terminal_policy(
            terminal_yaw_wait_timeout_, progress_yaw_);

        const auto clear_terminal_error = [&](const std::string &detail)
        {
            publishDiagnostic(
                diagnostic_msgs::DiagnosticStatus::OK,
                "ANAV-NAV-000", "终点旋转等待已恢复", detail, "", false,
                "/anav/navigation/terminal_yaw");
        };

        while (ros::ok())
        {
            if (cancel_requested_.load())
            {
                global_ac->cancelAllGoals();
                stopRobot();
                if (terminal_policy.errorActive())
                {
                    clear_terminal_error("终点旋转等待已由人工取消。");
                }
                return GOAL_CANCELED;
            }
            if (stop_and_quit)
            {
                global_ac->cancelAllGoals();
                stopRobot();
                if (terminal_policy.errorActive())
                {
                    clear_terminal_error("导航停止，终点旋转等待状态结束。");
                }
                return GOAL_FAILED;
            }

            geometry_msgs::PoseStamped current_pose;
            const bool have_current_pose = getCurrentRobotPose(current_pose);
            uint8_t terminal_motion_state = x2bot_teleop::TERMINAL_TRACKING;
            uint8_t path_control_mode = x2bot_teleop::PATH_CONTROL_UNKNOWN;
            const bool have_path_control_mode =
                currentPathControlMode(goal_start_time, path_control_mode);
            if (final_goal &&
                currentTerminalMotionState(goal_start_time,
                                           terminal_motion_state))
            {
                const x2bot_teleop::TerminalGoalUpdate terminal_update =
                    terminal_policy.update(
                        terminal_motion_state, have_current_pose,
                        have_current_pose
                            ? tf::getYaw(current_pose.pose.orientation)
                            : 0.0,
                        ros::Time::now().toSec());
                if (terminal_update.error_activated)
                {
                    std::ostringstream detail;
                    detail << "终点 P" << target_index
                           << " 已锁定，最终旋转连续 "
                           << std::fixed << std::setprecision(1)
                           << terminal_yaw_wait_timeout_
                           << " 秒没有有效角度进展；保持停车等待，不进行拓扑重规划。";
                    publishDiagnostic(
                        diagnostic_msgs::DiagnosticStatus::ERROR,
                        "ANAV-NAV-016", "终点旋转等待超时",
                        detail.str(),
                        "检查终点周围障碍物；障碍清除后车辆会继续原地旋转。",
                        true, "/anav/navigation/terminal_yaw");
                    ROS_ERROR_STREAM(detail.str());
                }
                else if (terminal_update.error_recovered)
                {
                    clear_terminal_error(
                        "检测到有效角度进展，继续完成当前终点的原地旋转。");
                    ROS_INFO("Terminal yaw at P%d resumed after a prolonged stop.",
                             target_index);
                }
            }

            if (pause_reentry_requested_.load())
            {
                global_ac->cancelGoal();
                stopRobot();
                while (pause_robot.load() && ros::ok())
                {
                    if (cancel_requested_.load())
                    {
                        global_ac->cancelAllGoals();
                        stopRobot();
                        if (terminal_policy.errorActive())
                        {
                            clear_terminal_error("终点旋转等待已由人工取消。");
                        }
                        return GOAL_CANCELED;
                    }
                    stopRobot();
                    ros::Duration(0.2).sleep();
                }
                if (!ros::ok())
                {
                    if (terminal_policy.errorActive())
                    {
                        clear_terminal_error("导航节点停止，终点旋转等待状态结束。");
                    }
                    return GOAL_FAILED;
                }

                const bool current_goal_passed =
                    referenceStatusMatches(topology_path_index,
                                           REFERENCE_PASSED,
                                           goal_start_time);
                pause_reentry_requested_.store(false);
                if (terminal_policy.locked())
                {
                    active_next_index_ = target_index;
                    publishTopologyMarkers();
                    ROS_INFO("Resume the locked terminal goal P%d after pause; topology re-entry is disabled.",
                             target_index);
                    global_ac->sendGoal(mb_goal);
                    continue;
                }
                active_next_index_ = -1;
                publishTopologyMarkers();
                ROS_INFO("Resume via topology re-entry after pausing at path index %d%s.",
                         topology_path_index,
                         current_goal_passed ? " (current waypoint already passed)" : "");
                return current_goal_passed ? GOAL_PAUSED_AFTER_PASS : GOAL_PAUSED;
            }

            if (!terminal_policy.locked() &&
                referenceStatusMatches(topology_path_index,
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
                if (cancel_requested_.load())
                {
                    global_ac->cancelAllGoals();
                    stopRobot();
                    if (terminal_policy.errorActive())
                    {
                        clear_terminal_error("终点旋转等待已由人工取消。");
                    }
                    return GOAL_CANCELED;
                }
                // pause() cancels the action from the independent Joy callback
                // queue. Handle that cancellation as a resumable topology
                // re-entry instead of a failed/blocked edge.
                if (pause_reentry_requested_.load())
                {
                    continue;
                }
                if (!terminal_policy.locked() &&
                    referenceStatusMatches(topology_path_index,
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
                    if (!final_goal && have_path_control_mode &&
                        path_control_mode == x2bot_teleop::LEGACY_FALLBACK &&
                        have_current_pose)
                    {
                        const double target_distance =
                            distanceToNode(current_pose, target_index);
                        if (target_distance > legacy_waypoint_reached_distance_)
                        {
                            ROS_WARN("Legacy controller reported P%d reached at %.3f m; require %.3f m and retry the same goal.",
                                     target_index, target_distance,
                                     legacy_waypoint_reached_distance_);
                            global_ac->sendGoal(mb_goal);
                            continue;
                        }
                    }
                    else if (!final_goal &&
                        validate_pass_through_action_success_distance_ &&
                        have_current_pose)
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
                    else if (!final_goal && have_current_pose)
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
                    if (terminal_policy.errorActive())
                    {
                        clear_terminal_error(
                            "最终朝向已经完成，终点旋转等待故障解除。");
                    }
                    return GOAL_REACHED;
                }
                if (terminal_policy.locked())
                {
                    ROS_WARN_STREAM(
                        "Locked terminal goal P" << target_index
                        << " returned state " << state.toString()
                        << "; stop and retry the same terminal goal without topology replanning.");
                    stopRobot();
                    ros::Duration(0.5).sleep();
                    global_ac->sendGoal(mb_goal);
                    continue;
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

            if (!final_goal && have_path_control_mode &&
                path_control_mode == x2bot_teleop::REFERENCE_TRACKING &&
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

            if (have_current_pose)
            {
                const double target_distance = distanceToNode(current_pose, target_index);
                if (!final_goal && have_path_control_mode &&
                    path_control_mode == x2bot_teleop::REFERENCE_TRACKING &&
                    target_distance <= waypoint_reached_distance_)
                {
                    // A live B-spline reference owns its waypoint-completion
                    // policy.  In particular, its terminal reference waypoint
                    // must run into reference_terminal_xy_tolerance instead of
                    // being preempted by this legacy 0.20 m fallback.
                    ROS_DEBUG_THROTTLE(
                        1.0,
                        "Ignore XY arrival for B-spline waypoint P%d at %.3f m; "
                        "wait for REFERENCE_PASSED.",
                        target_index, target_distance);
                }
                else if (have_path_control_mode &&
                         x2bot_teleop::legacyIntermediateWaypointReached(
                             path_control_mode, final_goal, target_distance,
                             legacy_waypoint_reached_distance_))
                {
                    ROS_INFO("Legacy waypoint P%d reached by XY distance %.3f m (limit %.3f m).",
                             target_index, target_distance,
                             legacy_waypoint_reached_distance_);
                    global_ac->cancelGoal();
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
                if (fixed_route || terminal_policy.locked())
                {
                    ROS_WARN_THROTTLE(
                        2.0,
                        "%s P%d: no motion for %.1f seconds; keep the same goal and wait without replanning.",
                        terminal_policy.locked() ? "Locked terminal" : "Fixed route",
                        target_index,
                        (now - last_progress_time).toSec());
                    continue;
                }
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
                if (fixed_route || terminal_policy.locked())
                {
                    ROS_WARN_THROTTLE(
                        2.0,
                        "%s P%d: goal remains active after %.1f seconds; wait indefinitely without replanning.",
                        terminal_policy.locked() ? "Locked terminal" : "Fixed route",
                        target_index,
                        (now - start_time).toSec());
                    continue;
                }
                ROS_WARN("Goal P%d timeout after %.1f seconds.", target_index, goal_timeout_);
                global_ac->cancelGoal();
                stopRobot();
                active_next_index_ = -1;
                publishTopologyMarkers();
                return GOAL_FAILED;
            }
        }

        stopRobot();
        if (terminal_policy.errorActive())
        {
            clear_terminal_error("导航节点停止，终点旋转等待状态结束。");
        }
        active_next_index_ = -1;
        publishTopologyMarkers();
        return GOAL_FAILED;
    }

    std::size_t initialTopologyExecutionStart(
        const std::vector<int> &path_indices)
    {
        // Never skip a one-node path: that node is the real final goal and
        // must still complete the normal XY + yaw terminal state machine.
        if (path_indices.size() <= 1 || !validIndex(path_indices.front()))
        {
            return 0;
        }

        geometry_msgs::PoseStamped current_pose;
        if (!getCurrentRobotPose(current_pose))
        {
            return 0;
        }
        const double anchor_distance =
            distanceToNode(current_pose, path_indices.front());
        if (!std::isfinite(anchor_distance) || anchor_distance < 0.0 ||
            anchor_distance > waypoint_reached_distance_)
        {
            return 0;
        }

        current_pose_index = path_indices.front();
        ROS_INFO("Starting topology anchor P%d confirmed at %.3f m; "
                 "begin motion with the next path node.",
                 current_pose_index, anchor_distance);
        return 1;
    }

    bool runFixedTopologyMission(int target_index,
                                 const std::vector<int> &path_indices)
    {
        if (path_indices.empty())
        {
            return false;
        }

        bool start_segment_active = true;

        const std::size_t execution_start =
            initialTopologyExecutionStart(path_indices);
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
            selectTopologySafetyPhase(start_segment_active, final_goal);
            while (ros::ok())
            {
                const GoalMonitorResult result = sendGoalAndMonitor(
                    next_index, previous_index, final_goal,
                    static_cast<int>(i), true);
                if (result == GOAL_REACHED ||
                    result == GOAL_PAUSED_AFTER_PASS)
                {
                    if (validIndex(previous_index))
                    {
                        markEdgeSuccess(previous_index, next_index);
                    }
                    break;
                }
                if (result == GOAL_PAUSED)
                {
                    ROS_INFO("Fixed route: resume the same topology goal P%d without replanning.",
                             next_index);
                    continue;
                }
                if (result == GOAL_CANCELED)
                {
                    stopRobot();
                    active_next_index_ = -1;
                    publishTopologyMarkers();
                    return false;
                }

                stopRobot();
                active_next_index_ = -1;
                publishTopologyMarkers();
                ROS_ERROR("Fixed route stopped at P%d; no alternate path will be planned.",
                          next_index);
                return false;
            }
            // Keep the terminal profile through the complete first departure
            // edge, then expand back to the normal padding.
            if (i >= 1)
            {
                start_segment_active = false;
            }
            if (!ros::ok())
            {
                stopRobot();
                return false;
            }
        }

        current_status = 3;
        active_next_index_ = -1;
        publishTopologyMarkers();
        return true;
    }

    bool runTopologyMission(int target_index, std::vector<int> path_indices,
                            bool fixed_route = false)
    {
        if (fixed_route)
        {
            return runFixedTopologyMission(target_index, path_indices);
        }
        int start_index = path_indices.empty() ? nearestPoseIndex() : path_indices.front();
        const int mission_start_index = start_index;
        bool start_segment_active = true;
        std::size_t execution_start =
            initialTopologyExecutionStart(path_indices);
        int replan_count = 0;
        int active_reentry_node = -1;
        std::map<int, ros::WallTime> blocked_reentry_until;
        std::map<int, int> reentry_failure_count;
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
                selectTopologySafetyPhase(start_segment_active, final_goal);
                const GoalMonitorResult goal_result =
                    sendGoalAndMonitor(next_index, previous_index, final_goal,
                                       static_cast<int>(i), false);
                if (goal_result == GOAL_REACHED)
                {
                    if (next_index == active_reentry_node)
                    {
                        active_reentry_node = -1;
                    }
                    if (start_segment_active &&
                        previous_index == mission_start_index)
                    {
                        start_segment_active = false;
                    }
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

                if (goal_result == GOAL_CANCELED)
                {
                    stopRobot();
                    active_next_index_ = -1;
                    publishTopologyMarkers();
                    return false;
                }

                const bool path_deviated = goal_result == GOAL_PATH_DEVIATED;
                const bool paused = goal_result == GOAL_PAUSED ||
                                    goal_result == GOAL_PAUSED_AFTER_PASS;
                if (start_segment_active &&
                    goal_result == GOAL_PAUSED_AFTER_PASS &&
                    previous_index == mission_start_index)
                {
                    start_segment_active = false;
                }
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
                        active_reentry_node = -1;
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

                if (!needs_forward_reentry)
                {
                    if (validIndex(previous_index))
                    {
                        blockEdge(previous_index, next_index);
                    }
                    else if (next_index == active_reentry_node)
                    {
                        const int failure_count =
                            ++reentry_failure_count[next_index];
                        const double cooldown = std::min(
                            blocked_cooldown_initial_ * std::pow(
                                blocked_backoff_factor_, failure_count - 1),
                            blocked_cooldown_max_);
                        blocked_reentry_until[next_index] =
                            ros::WallTime::now() + ros::WallDuration(cooldown);
                        ROS_WARN("Temporarily blocked failed local topology connector "
                                 "to P%d for %.1f seconds (failure count=%d).",
                                 next_index, cooldown, failure_count);
                        active_reentry_node = -1;
                    }

                    std::vector<int> local_path;
                    bool map_ready = false;
                    std::string local_reason;
                    bool local_plan_ok = localAwareReroute(
                        target_index, local_path, map_ready, local_reason,
                        &blocked_reentry_until);

                    // Preserve the existing wait-for-temporary-block behavior
                    // when the graph truly has no alternate route. After a
                    // block expires, still require the local-map-aware choice.
                    if (!local_plan_ok && map_ready &&
                        validIndex(previous_index))
                    {
                        std::vector<int> topology_path;
                        if (dijkstraShortestPath(previous_index,
                                                 target_index).empty() &&
                            waitForTemporaryPath(previous_index, target_index,
                                                 topology_path))
                        {
                            local_plan_ok = localAwareReroute(
                                target_index, local_path,
                                map_ready, local_reason,
                                &blocked_reentry_until);
                        }
                    }

                    if (!local_plan_ok)
                    {
                        ROS_ERROR("Local-aware topology replan to P%d refused: %s.",
                                  target_index, local_reason.c_str());
                        stopRobot();
                        active_next_index_ = -1;
                        publishTopologyMarkers();
                        return false;
                    }

                    path_indices.swap(local_path);
                    start_index = path_indices.front();
                    active_reentry_node = path_indices.front();
                    execution_start = 0;
                    ++replan_count;
                    publishTopologyPath(path_indices);
                    ROS_WARN_STREAM(formatPathMessage(
                        "local-map-aware replan after blocked edge:",
                        path_indices));
                    break;
                }

                // Keep the existing pause/B-spline-deviation fallback exactly
                // as before. Local-map scoring applies only after a failed
                // edge has completed its normal blocked timeout.
                if (validIndex(previous_index))
                {
                    start_index = previous_index;
                    ROS_INFO("Anchor blocked-edge replan at last confirmed node P%d "
                             "instead of choosing an arbitrary nearest node.",
                             start_index);
                }
                else
                {
                    start_index = nearestPoseIndex();
                    if (!validIndex(start_index))
                    {
                        start_index = current_pose_index;
                    }
                }
                path_indices = dijkstraShortestPath(start_index, target_index);
                active_reentry_node = -1;
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
        setTopologySafetyPhase(TOPOLOGY_SAFETY_START_SEGMENT);
        const bool ok = runTopologyMission(path_indices.back(), path_indices);
        notifyMotionFinished(
            ok ? cmd_vel_arbiter::FinishMotionRequest::TASK_FINISHED
               : cmd_vel_arbiter::FinishMotionRequest::TASK_FAILED);
        setTopologySafetyPhase(TOPOLOGY_SAFETY_NORMAL);
    }

    void startRun_()
    {
        if (target_poses.empty())
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(nav_config_mutex_);
            navigation_active_.store(true);
        }

        std::vector<int> path_indices;
        for (int i = 0; i < static_cast<int>(target_poses.size()); ++i)
        {
            path_indices.push_back(i);
        }
        run_planed_pathnode(path_indices);
        {
            std::lock_guard<std::mutex> lock(nav_config_mutex_);
            navigation_active_.store(false);
        }
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

    bool cancelNavigationCallback(std_srvs::Trigger::Request &,
                                  std_srvs::Trigger::Response &res)
    {
        cancel_requested_.store(true);
        pause_robot.store(false);
        pause_reentry_requested_.store(false);
        global_ac->cancelAllGoals();
        stopRobot();
        res.success = true;
        res.message = "navigation cancellation requested";
        ROS_INFO("Navigation cancellation requested by GUI.");
        return true;
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
    if (!current_nav.loadNavPnts())
    {
        current_nav.publishDiagnostic(
            diagnostic_msgs::DiagnosticStatus::ERROR,
            "ANAV-MAP-004", "导航点文件缺失或内容无效",
            "无法加载 robot_positions.txt。",
            "检查 ~/maps/robot_positions.txt 的路径、格式和点位数量。", true);
        ROS_FATAL("Navigation points are missing or invalid; topology navigation will not start.");
        ros::WallDuration(0.2).sleep();
        return 2;
    }
    if (!current_nav.loadStaticMap())
    {
        current_nav.publishDiagnostic(
            diagnostic_msgs::DiagnosticStatus::ERROR,
            "ANAV-MAP-002", "静态地图检查未通过",
            "map.yaml 或其图像文件缺失、格式错误或无法读取。",
            "检查 ~/maps/map.yaml、image 路径及文件权限。", true);
        ROS_FATAL("Static map is missing or invalid; topology navigation will not start.");
        ros::WallDuration(0.2).sleep();
        return 3;
    }
    if (!current_nav.loadTopology())
    {
        current_nav.publishDiagnostic(
            diagnostic_msgs::DiagnosticStatus::ERROR,
            "ANAV-MAP-009", "导航拓扑检查未通过",
            "topology.yaml 缺失、与点位/地图不匹配或校验失败。",
            "在 GUI 中重新构建导航拓扑并检查构建输出。", true);
        ROS_FATAL("Validated topology is unavailable; topology navigation will not start.");
        ros::WallDuration(0.2).sleep();
        return 4;
    }
    current_nav.publishDiagnostic(
        diagnostic_msgs::DiagnosticStatus::OK,
        "ANAV-NAV-000", "导航配置检查通过", "点位、静态地图和拓扑已加载。",
        "", false);

    while (ros::ok())
    {
        current_nav.publishNavPointsMarkers();
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
