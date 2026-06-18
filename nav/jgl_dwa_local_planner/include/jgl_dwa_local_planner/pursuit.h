#include<iostream>
#include <vector>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
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
    int  calc_target_index(VehicleState state,std::vector<geometry_msgs::PoseStamped>distPoint);
    struct Result pure_pursuit(VehicleState state,std::vector<geometry_msgs::PoseStamped>distPoint,int pind);

    geometry_msgs::PoseStamped refPosition;

    geometry_msgs::PoseStamped normalizeVector(const geometry_msgs::PoseStamped& vector);

    geometry_msgs::PoseStamped getDirectionVector(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

    geometry_msgs::PoseStamped calculateTargetPoint(const std::vector<geometry_msgs::PoseStamped>& distPoint, double distance); 
};


