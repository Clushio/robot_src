#include <fstream>
#include <sstream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <x2bot_teleop/SetInt.h>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <queue>
#include <map>
#include <set>


struct TargetPose {
    double x, y, z, roll, pitch, yaw;
    std::string label;
};

// 在合适的位置（如源文件顶部）声明全局变量

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MVClient;

class mynav
{
    public:
    MVClient *global_ac;
    std::unique_ptr<std::thread> runth_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Publisher marker_pub;

    int current_pose_index;               // 当前机器人所在的点索引
    ros::ServiceServer plan_path_service; // 路径规划服务

    mynav(): nh_()
    {
        initializeGlobalAC();
        current_status  = 0;
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &mynav::joyCallback, this);
        pause_robot = false;
        stop_and_quit = false;

        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        // 广播服务
        plan_path_service = nh_.advertiseService("plan_path_and_go", &mynav::planPathCallback, this);
        ROS_INFO("路径规划服务已启动...");
        current_pose_index = 0;              // 当前机器人所在的点索引
    }
public:
    bool planPathCallback_TOPO(x2bot_teleop::SetInt::Request &req, x2bot_teleop::SetInt::Response &res) 
    {
        int target_index = req.data;
        int current_indx = req.currentID;
        int exec_path = req.run;
        current_pose_index = current_indx;

        // 检查目标点是否有效
        if (target_index < 0 || target_index >= target_poses.size()) {
            res.success = false;
            res.message = "目标点序号无效";
            ROS_ERROR("目标点序号无效: %d", target_index);
            return true;
        }              

        // 计算路径
        std::vector<int> path_indices= bfs_shortest_path(current_indx, target_index);

        if (path_indices.empty()) {
            res.success = false;
            res.message = "无法找到从 P" + std::to_string(current_indx) + " 到 P" + std::to_string(target_index) + " 的路径";
            ROS_ERROR_STREAM(res.message);
            return true;
        }

        // 构造路径消息
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (const auto& index : path_indices) {
            geometry_msgs::PoseStamped pose_stamped = toPoseStamped(target_poses[index]);
            path.poses.push_back(pose_stamped);
        }

        // 更新当前点索引
        //current_pose_index = target_index;

        // 返回结果
        res.success = true;
        res.message = "plan ok:";

        // 打印路径信息
        ROS_INFO("规划路径: ");
        for (const auto& index : path_indices) {
            ROS_INFO(" -> 点 %d", index);
            res.message += " -> P "+std::to_string(index);
        }
        
        if(exec_path>0)
        {
            run_planed_pathnode(path_indices);
        }

        return true;
    }
    // Service 回调函数
    bool planPathCallback(x2bot_teleop::SetInt::Request &req, x2bot_teleop::SetInt::Response &res) 
    {
        int target_index = resolvePoseIndex(req.data);
        int current_indx = resolvePoseIndex(req.currentID);
        int exec_path = req.run;
        current_pose_index = current_indx;

        // 检查目标点是否有效
        if (target_index < 0 || target_index >= target_poses.size()) {
            res.success = false;
            res.message = "目标点序号无效";
            ROS_ERROR("目标点序号无效: %d", req.data);
            return true;
        }
        if (current_indx < 0 || current_indx >= target_poses.size()) {
            res.success = false;
            res.message = "当前点序号无效";
            ROS_ERROR("当前点序号无效: %d", req.currentID);
            return true;
        }              

        // 计算路径
        std::vector<int> path_indices;
        if (current_pose_index <= target_index) {
            // 向前移动
            for (int i = current_pose_index; i <= target_index; ++i) {
                path_indices.push_back(i);
            }
        } else {
            // 向后移动
            for (int i = current_pose_index; i >= target_index; --i) {
                path_indices.push_back(i);
            }
        }

        // 构造路径消息
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (const auto& index : path_indices) {
            geometry_msgs::PoseStamped pose_stamped = toPoseStamped(target_poses[index]);
            path.poses.push_back(pose_stamped);
        }

        // 更新当前点索引
        //current_pose_index = target_index;

        // 返回结果
        res.success = true;
        res.message = "plan ok:";

        // 打印路径信息
        ROS_INFO("规划路径: ");
        for (const auto& index : path_indices) {
            ROS_INFO(" -> 点 %d", index);
            res.message += " -> P "+std::to_string(index);
        }
        
        if(exec_path>0)
        {
            run_planed_pathnode(path_indices);
        }

        return true;
    }
    void publishNavPointsMarkers() 
    {
        //ros::NodeHandle n;
        

        visualization_msgs::Marker marker;
        // 设置marker的一些基本属性
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "nav_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3; // 直径
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 0.7; // 不透明度
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // 添加所有导航点到marker
        for (const auto& pose : target_poses) {
            geometry_msgs::Point p;
            p.x = pose.x;
            p.y = pose.y;
            p.z = pose.z;
            marker.points.push_back(p);
        }

        // 发布marker
        marker_pub.publish(marker);
    }


    ~mynav()
    {
        if(runth_ && runth_->joinable())
        {
            runth_->join();
        }

    }
    // 从字符串中解析TargetPose对象
    TargetPose parseTargetPose(const std::string& line) {
        std::istringstream iss(line);
        TargetPose pose;
        //std::cout<<iss;
        if (!(iss >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw)) {
            throw std::runtime_error("Invalid target pose format");
        }
        iss >> pose.label;
        return pose;
    }

    int resolvePoseIndex(int requested_index) {
        if (requested_index >= 0) {
            return requested_index;
        }

        std::string workstation_label = "W" + std::to_string(-requested_index);
        auto it = workstation_indices.find(workstation_label);
        if (it == workstation_indices.end()) {
            ROS_ERROR_STREAM("Workstation " << workstation_label << " not found in robot_positions.txt");
            return -1;
        }
        return it->second;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
         bool Apressed = joy->buttons[0];
         bool Xpressed = joy->buttons[2];
         bool Ypressed = joy->buttons[3];
         if(Apressed)
         {
            std::cout<<"A pressed"<<std::endl;
            pause_robot = true;
            pause();
         }

          if(Xpressed)
         {
            std::cout<<"X pressed"<<std::endl;
            pause_robot = false;
            resume();
         }

         if(Ypressed)
         {
            std::cout<<"Y pressed, start! or resume run!"<<std::endl;
            pause_robot = false;
            if((nullptr == runth_)||(current_status == 3))
            {
                start();
            }
         }

    }

    // 将TargetPose对象转换为geometry_msgs::PoseStamped消息
    geometry_msgs::PoseStamped toPoseStamped(const TargetPose& pose) {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "map"; // 根据实际情况设置参考坐标系
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

    ros::NodeHandle nh_;    
    std::vector<TargetPose> target_poses;
    std::map<std::string, int> workstation_indices;
    int numofpnts;
    int current_pnt;
    bool stop_and_quit;
    bool pause_robot;
    std::map<int, std::vector<int>> graph;

    int current_status;//0 init, 1 load pnts ok; 2 running; 3 finished; 4 quit

    bool loadTopoFromTxt() {

        std::ifstream file("/home/nav/maps/topo.txt");
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open file: " << "/home/nav/maps/topo.txt");
            return false;
        }
        
       // std::ifstream file(filename);
        std::string line;
    
        while (std::getline(file, line)) {
            // 跳过空行
            if (line.empty()) continue;
    
            std::istringstream iss(line);
            std::string node_str, neighbors_str;
            
            // 分割 "节点: 邻居列表"
            if (std::getline(iss, node_str, ':') && std::getline(iss, neighbors_str)) {
                int node_id = std::stoi(node_str);
                std::vector<int> neighbors;
    
                std::stringstream nss(neighbors_str);
                std::string neighbor;
                while (std::getline(nss, neighbor, ',')) {
                    // 去除空格
                    neighbor.erase(0, neighbor.find_first_not_of(" \t"));
                    neighbor.erase(neighbor.find_last_not_of(" \t") + 1);
                    if (!neighbor.empty()) {
                        neighbors.push_back(std::stoi(neighbor));
                    }
                }
                graph[node_id] = neighbors;
                std::cout << "Node "<<node_id<<" connects to: ";
                for(int n:neighbors)
                {
                    std::cout << n;
                }
                std::cout<<std::endl;
            }
        }
        return true;
    }

    std::vector<int> bfs_shortest_path(int start, int goal) {
        if (start == goal) return {start};
    
        std::queue<int> q;
        std::map<int, bool> visited;
        std::map<int, int> parent;  // 记录路径
    
        q.push(start);
        visited[start] = true;
    
        while (!q.empty()) {
            int u = q.front(); q.pop();
    
            for (int v : graph[u]) {
                if (!visited[v]) {
                    visited[v] = true;
                    parent[v] = u;
                    q.push(v);
    
                    if (v == goal) {
                        // 回溯路径
                        std::vector<int> path;
                        for (int at = goal; at != start; at = parent[at]) {
                            path.push_back(at);
                        }
                        path.push_back(start);
                        std::reverse(path.begin(), path.end());
                        return path;
                    }
                }
            }
        }
    
        // 找不到路径
        return {};
    }

    bool loadNavPnts()
    {
        numofpnts = 0;
        std::ifstream infile("/home/nav/maps/robot_positions.txt");
        if (!infile.is_open()) {
            ROS_ERROR_STREAM("Failed to open file: " << "/home/nav/maps/robot_positions.txt");
            return false;
        }
        target_poses.clear();
        workstation_indices.clear();
        std::string line;
        while (std::getline(infile, line)) {
            try {
                 TargetPose pose = parseTargetPose(line);
                 int index = target_poses.size();
                 if (!pose.label.empty()) {
                     workstation_indices[pose.label] = index;
                     std::cout << pose.label << " maps to point " << index << std::endl;
                 }
                 target_poses.push_back(pose);
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Error parsing target pose: " << e.what());
                continue;
            }
         }
       infile.close();
       numofpnts = target_poses.size();
     //  numofpnts = target_poses.size();
       std::cout<<"load ok total "<<numofpnts<<" nav points!"<<std::endl;


       publishNavPointsMarkers();

       return true;
    }

    void initializeGlobalAC() 
    {
        global_ac = new MVClient("move_base", true);
      //  MVClient global_ac("move_base", true);
       // global_ac = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
        ROS_INFO("Waiting for move_base action server to start...");
        global_ac->waitForServer(); // will wait for infinite time
        ROS_INFO("Connected to move_base action server");
    }


    // 运行函数：按照规划的路径导航
    void run_planed_pathnode(const std::vector<int>& path_indices) {
        int cycle = 1; // 假设只运行一个周期，可以扩展为多周期
        while (cycle > 0) {
            std::cout << "Remaining cycles: " << cycle << std::endl;
            cycle--;

            for (const auto& index : path_indices) {
                const TargetPose& target_pose = target_poses[index]; // 获取目标点
                geometry_msgs::PoseStamped goal = toPoseStamped(target_pose);

                move_base_msgs::MoveBaseGoal mb_goal;
                mb_goal.target_pose = goal;

                ROS_INFO_STREAM("Sending goal: (" << target_pose.x << ", " << target_pose.y << ")");
                global_ac->sendGoal(mb_goal);
                std::cout << "Moving to goal: Point " << index << std::endl;

                // 等待目标到达或超时（可根据实际情况调整超时时间）
                global_ac->waitForResult(ros::Duration(120.0)); // 120秒超时
                actionlib::SimpleClientGoalState state = global_ac->getState();

                if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO_STREAM("Goal reached successfully");
                    ros::Duration(1).sleep(); // 到达后暂停10秒
                } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                    ROS_WARN_STREAM("Goal aborted");
                    break;
                } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
                    ROS_WARN_STREAM("Goal preempted");
                } else {
                    ROS_WARN_STREAM("Goal failed with state: " << state.toString());
                }

                // 更新当前点索引
                current_pose_index = index;

                // 检查是否需要退出
                if (stop_and_quit) {
                    std::cout << "Quit at current point!" << std::endl;
                    vel_pub_.publish(geometry_msgs::Twist()); // 发布零速度
                    return; // 退出线程
                }

                // 检查是否需要暂停
                while (pause_robot) {
                    ros::Duration(1).sleep();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    std::cout << "Pause at current point! = " << current_pose_index << std::endl;
                }
            }
        }
       
    }

    
    void startRun_()
    {
      
        int cycle = 1;
        while (cycle>0)
        {
            current_pnt = 0;
            std::cout<<"remain cycles="<<cycle<<std::endl;
            cycle--;
            for (const auto& target_pose : target_poses) 
             {
                
                current_pnt++;
                geometry_msgs::PoseStamped goal = toPoseStamped(target_pose);
                move_base_msgs::MoveBaseGoal mb_goal;
                mb_goal.target_pose = goal;

                ROS_INFO_STREAM("Sending goal: (" << target_pose.x << ", " << target_pose.y << ")");
                global_ac->sendGoal(mb_goal);
                std::cout<<"moving to goal"<<current_pnt<<std::endl;

               // ros::Duration(0.5).sleep();

                // 等待目标到达或超时（可根据实际情况调整超时时间）
                global_ac->waitForResult(ros::Duration(120.0));  // 60 seconds timeout
                actionlib::SimpleClientGoalState state = global_ac->getState();

                if (state == actionlib::SimpleClientGoalState::SUCCEEDED) 
                {
                    
                    ROS_INFO_STREAM("Goal reached successfully");
                    ros::Duration(10).sleep();
                } 
                else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                        ROS_WARN_STREAM("Goal aborted");
                        break;
                    } 
                else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
                        ROS_WARN_STREAM("Goal preempted");
                    } 
                else 
                    {
                        ROS_WARN_STREAM("Goal failed with state: " << state.toString());
                    }

                if(stop_and_quit)
                {
                    std::cout<<"quit at current point!"<<std::endl;
                    vel_pub_.publish(*new geometry_msgs::Twist());//没有按下deadman开关则发送的速度均为0
                    return;//exit thread
                }
                    
                while (pause_robot)
                {
                //   thread.sleep(1);
                    ros::Duration(1).sleep();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    std::cout<<"pause at current point! = "<< current_pnt <<std::endl;
                    /* code */
                }
           
              }


        }
        
      
        std::cout<<"moving mission ok!"<<std::endl;

        current_status = 3;
       // runth_ = nullptr;
    }

    void start()
    {
        if(numofpnts <=0)
        {
            return;
        }
        //runth_ = new std::thread(startRun_);
         runth_ = std::make_unique<std::thread>(&mynav::startRun_,this); // 使用智能指针创建并启动线程
    //    runth_->start();
    }

    void pause()
    {
        global_ac->cancelAllGoals(); // 停
        pause_robot = true;
        vel_pub_.publish(*new geometry_msgs::Twist());//没有按下deadman开关则发送的速度均为0
    }

    void resume()
    {
        if(pause_robot)
        {
            std::cout<<"resume runing ="<< current_pnt<<std::endl;
            pause_robot = false;
        }
   //     global_ac->cancelAllGoals(); // 停
  //      pause_robot = true;
    }

    void exitgoals()
    {
        global_ac->cancelAllGoals(); // 停
        stop_and_quit = true;
        vel_pub_.publish(*new geometry_msgs::Twist());//没有按下deadman开关则发送的速度均为0
    }
    

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "target_pose_loader");

   // ros::NodeHandle nh;

    mynav current_nav;
    current_nav.loadNavPnts();
    current_nav.loadTopoFromTxt();



    while (ros::ok())
    {
        current_nav.publishNavPointsMarkers();
        /* code */
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
  
    return 0;
}
