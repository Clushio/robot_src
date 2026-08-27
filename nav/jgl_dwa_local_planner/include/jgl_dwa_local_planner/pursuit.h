#include<iostream>
#include <vector>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
struct Result{
    double x;
    int y;
};
class VehicleState{
    public:
    double x;     //位置
    double y;
    double yaw;   //偏航角
    double v;     //速度

    VehicleState(double a,double b,double c,double d);
    VehicleState();

};
class Pursuit{
public:
    double kk ; // 前视距离系数
    double Lfc ;  // 前视距离
    bool nearEnd;


    Pursuit(double a,double b);
    int calc_target_index(
        VehicleState state,
        const std::vector<geometry_msgs::msg::PoseStamped> &distPoint);
    struct Result pure_pursuit(
        VehicleState state,
        const std::vector<geometry_msgs::msg::PoseStamped> &distPoint,
        int pind);

    geometry_msgs::msg::PoseStamped refPosition;

    geometry_msgs::msg::PoseStamped normalizeVector(const geometry_msgs::msg::PoseStamped& vector);

    geometry_msgs::msg::PoseStamped getDirectionVector(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2);

    geometry_msgs::msg::PoseStamped calculateTargetPoint(const std::vector<geometry_msgs::msg::PoseStamped>& distPoint, double distance);
};

