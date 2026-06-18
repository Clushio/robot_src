/********x2bot_joy.cpp***************
*功能：x2bot遥控节点，接收joy/joy_node节点发来的手柄按键信息，转换为控制速度cmd_vel，最大线速度0.85m/s，最大角速度8.20rad/s，
*     由于采用deadman机制，必须按住2号按钮（2/B）才能用方向键控制方向
*作者：张京林
*时间：2016.06.27
***********************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class X2botTeleop
{
public:
  X2botTeleop();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_,nh_;

  int linear_, angular_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;
};

X2botTeleop::X2botTeleop():
 ph_("~"),
 linear_(7),//前后方向键,PXN stick
 angular_(6),//左右方向键,pxn stick
 deadman_axis_(0),
 l_scale_(0.3),
 a_scale_(0.9)
{
  //NodeHandle::param(parameter name,paramer value,default value)
  ph_.param("axis_linear", linear_, linear_);//线速度下标，axes中代表前后的控制量，对于莱仕达方向键下标是1，上1，下-1
  ph_.param("axis_angular", angular_, angular_);//角速度下标，axes中代表左右的控制量，对于莱仕达方向键下标是0，左1，右-1
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);//按住2号按钮（2/B）然后用方向键控制方向
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &X2botTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&X2botTeleop::publish, this));
}

void X2botTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  std::cout<<"deadman"<<deadman_pressed_<<std::endl;
}

void X2botTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
    std::cout<<"real"<<std::endl ;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());//没有按下deadman开关则发送的速度均为0
    zero_twist_published_=true;
    std::cout<<"stop"<<std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "x2bot_teleop");
  X2botTeleop X2botTeleop;

  ros::spin();
}
