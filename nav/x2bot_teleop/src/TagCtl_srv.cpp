#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <x2bot_teleop/SetTagY.h>
#include <tf/transform_datatypes.h> // 用于处理四元数
#include <algorithm>
#include <cmath>

namespace {
const char* const COLOR_YELLOW = "\033[33m";
const char* const COLOR_RESET = "\033[0m";

double degToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

double normalizeAngleRad(double angle) {
    while (angle > 3.14159265358979323846) {
        angle -= 2.0 * 3.14159265358979323846;
    }
    while (angle < -3.14159265358979323846) {
        angle += 2.0 * 3.14159265358979323846;
    }
    return angle;
}
}

class RobotController {
public:
    RobotController(ros::NodeHandle& nh)
        : nh_(nh),
          current_x_m_(0.0),
          current_y_m_(0.0),
          target_x_m_(0.0),
          target_y_m_(0.0),
          target_yaw_rad_(0.0),
          current_yaw_rad_(0.0),
          current_valid_(-99.0),
          filter_initialized_(false),
          yaw_pd_initialized_(false),
          previous_yaw_error_(0.0),
          previous_angular_speed_(0.0),
          yaw_stable_count_(0),
          yaw_stable_required_(5),
          tag_update_seq_(0),
          last_yaw_stable_seq_(0),
          last_update_time_(ros::Time(0)) {
        // 初始化参数
        ros::NodeHandle private_nh("~");
        private_nh.param("kp", kp_, 0.5);                  // y 方向比例增益
        private_nh.param("kp_x", kp_x_, 0.5);              // x 方向比例增益
        private_nh.param("target_x", target_x_m_, 0.0);    // x 方向目标偏差，单位 m，默认对中
        private_nh.param("min_speed", min_speed_, 0.01);   // 死区外最低启动速度
        private_nh.param("max_speed", max_speed_, 0.05);   // 最高速度绝对值
        private_nh.param("arrival_threshold_y", arrival_threshold_y_, 0.005); // y 到达目标阈值
        private_nh.param("arrival_threshold", arrival_threshold_y_, arrival_threshold_y_); // 兼容旧参数名
        private_nh.param("arrival_threshold_x", arrival_threshold_x_, 0.005); // x 到达目标阈值
        private_nh.param("data_timeout", data_timeout_, 3.0);              // 数据超时时间（秒）
        private_nh.param("rotation_threshold", rotation_threshold_, 0.015);  // 旋转角度阈值
        private_nh.param("yaw_kp", yaw_kp_, 0.5);             // 角度 PD 的 P
        private_nh.param("yaw_kd", yaw_kd_, 0.00);            // 角度 PD 的 D
        private_nh.param("max_angular_speed", max_angular_speed_, 0.15); // 最大角速度
        private_nh.param("max_angular_accel", max_angular_accel_, 0.15); // 最大角加速度
        private_nh.param("filter_alpha", filter_alpha_, 0.5); // 位置/角度低通滤波系数
        private_nh.param("yaw_stable_count", yaw_stable_required_, 5); // 连续多少帧 yaw 达标才切换/成功
        private_nh.param("final_yaw_kp", final_yaw_kp_, 0.2); // Y+Yaw 阶段角度 P，独立于初始旋转
        private_nh.param("final_yaw_cmd_deadband", final_yaw_cmd_deadband_, rotation_threshold_); // 最后 Y+Yaw 阶段的角速度死区
        private_nh.param("enable_final_y_yaw_after_x_check", enable_final_y_yaw_after_x_check_, false); // 最终 X 后是否再补一轮 Y+Yaw

        sanitizeParameters();

        // 订阅 /tag_position 话题
        tag_subscriber_ = nh_.subscribe("/tag_position", 10, &RobotController::tagCallback, this);

        // 发布 /cmd_vel 消息
        cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 创建服务
        service_ = nh_.advertiseService("set_target_y", &RobotController::handleSetTargetY, this);

        ROS_INFO(
            "[TAGCTL] Robot controller node initialized. kp=%.3f kp_x=%.3f min_speed=%.3f max_speed=%.3f "
            "arrival_y=%.3f arrival_x=%.3f yaw_kp=%.3f yaw_kd=%.3f max_ang=%.3f max_ang_acc=%.3f "
            "yaw_stable_count=%d final_yaw_kp=%.3f final_yaw_cmd_deadband=%.3f enable_final_y_yaw_after_x_check=%d",
            kp_, kp_x_, min_speed_, max_speed_, arrival_threshold_y_, arrival_threshold_x_,
            yaw_kp_, yaw_kd_, max_angular_speed_, max_angular_accel_,
            yaw_stable_required_, final_yaw_kp_, final_yaw_cmd_deadband_, enable_final_y_yaw_after_x_check_);
    }

    void run() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    ros::ServiceServer service_;

    double kp_;                    // 比例增益
    double kp_x_;                  // x 方向比例增益
    double min_speed_;             // 最低速度绝对值
    double max_speed_;             // 最高速度绝对值
    double arrival_threshold_y_;   // y 到达目标阈值
    double arrival_threshold_x_;   // x 到达目标阈值
    double rotation_threshold_;    // 旋转角度阈值
    double yaw_kp_;
    double yaw_kd_;
    double max_angular_speed_;
    double max_angular_accel_;
    double filter_alpha_;
    double final_yaw_kp_;
    double final_yaw_cmd_deadband_;
    bool enable_final_y_yaw_after_x_check_;

    double current_x_m_;           // 当前 x 偏差，单位 m
    double current_y_m_;           // 当前 y 偏差，单位 m
    double target_x_m_;            // 目标 x 偏差，单位 m
    double target_y_m_;            // 目标 y 偏差，单位 m
    double target_yaw_rad_;        // 目标 yaw，单位 rad
    double current_yaw_rad_;       // 当前 yaw，单位 rad
    ros::Time last_update_time_;   // 上次更新时间
    double data_timeout_;          // 数据超时时间（秒）
    double current_valid_;
    bool filter_initialized_;
    bool yaw_pd_initialized_;
    double previous_yaw_error_;
    double previous_angular_speed_;
    ros::Time previous_yaw_time_;
    int yaw_stable_count_;
    int yaw_stable_required_;
    unsigned int tag_update_seq_;
    unsigned int last_yaw_stable_seq_;

    enum State { ROTATING, CORRECTING_X, MOVING_Y_YAW, FINAL_CORRECTING_X };
    State state_ = ROTATING;       // 初始状态为旋转

    // 订阅回调函数
    void tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_valid_ = msg->pose.position.z;
        if(current_valid_>=-1)
        {
            const double measured_x_m = msg->pose.position.x / 100.0;
            const double measured_y_m = msg->pose.position.y / 100.0;

            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg->pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            if (!filter_initialized_) {
                current_x_m_ = measured_x_m;
                current_y_m_ = measured_y_m;
                current_yaw_rad_ = yaw;
                filter_initialized_ = true;
            } else {
                current_x_m_ = filter_alpha_ * measured_x_m + (1.0 - filter_alpha_) * current_x_m_;
                current_y_m_ = filter_alpha_ * measured_y_m + (1.0 - filter_alpha_) * current_y_m_;
                const double yaw_diff = normalizeAngleRad(yaw - current_yaw_rad_);
                current_yaw_rad_ = normalizeAngleRad(current_yaw_rad_ + filter_alpha_ * yaw_diff);
            }
            ++tag_update_seq_;
        }

        last_update_time_ = ros::Time::now();         // 更新时间戳
    }

    // 检查数据是否过期
    bool isDataStale() {
        if (last_update_time_.isZero()) {
            return true;
        }
        return (ros::Time::now() - last_update_time_).toSec() > data_timeout_;
    }

    // 限制速度范围
    double clamp(double value, double min_value, double max_value) {
        return std::max(std::min(value, max_value), min_value);
    }

    void sanitizeParameters() {
        if (kp_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid kp %.3f, reset to 0.5", kp_);
            kp_ = 0.5;
        }
        if (kp_x_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid kp_x %.3f, reset to 0.5", kp_x_);
            kp_x_ = 0.5;
        }
        if (min_speed_ < 0.0) {
            ROS_WARN("[TAGCTL] Invalid min_speed %.3f, reset to 0.01", min_speed_);
            min_speed_ = 0.01;
        }
        if (max_speed_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid max_speed %.3f, reset to 0.05", max_speed_);
            max_speed_ = 0.05;
        }
        if (min_speed_ > max_speed_) {
            ROS_WARN("[TAGCTL] min_speed %.3f is larger than max_speed %.3f, clamp min_speed to max_speed.",
                     min_speed_, max_speed_);
            min_speed_ = max_speed_;
        }
        if (arrival_threshold_y_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid arrival_threshold_y %.3f, reset to 0.01", arrival_threshold_y_);
            arrival_threshold_y_ = 0.01;
        }
        if (arrival_threshold_x_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid arrival_threshold_x %.3f, reset to 0.1", arrival_threshold_x_);
            arrival_threshold_x_ = 0.1;
        }
        if (max_angular_speed_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid max_angular_speed %.3f, reset to 0.15", max_angular_speed_);
            max_angular_speed_ = 0.15;
        }
        if (max_angular_accel_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid max_angular_accel %.3f, reset to 0.15", max_angular_accel_);
            max_angular_accel_ = 0.15;
        }
        if (yaw_stable_required_ <= 0) {
            ROS_WARN("[TAGCTL] Invalid yaw_stable_count %d, reset to 5", yaw_stable_required_);
            yaw_stable_required_ = 5;
        }
        if (final_yaw_kp_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid final_yaw_kp %.3f, reset to 0.1", final_yaw_kp_);
            final_yaw_kp_ = 0.1;
        }
        if (final_yaw_cmd_deadband_ <= 0.0) {
            ROS_WARN("[TAGCTL] Invalid final_yaw_cmd_deadband %.3f, reset to rotation_threshold %.3f",
                     final_yaw_cmd_deadband_, rotation_threshold_);
            final_yaw_cmd_deadband_ = rotation_threshold_;
        }
    }

    void resetYawStableCounter() {
        yaw_stable_count_ = 0;
        last_yaw_stable_seq_ = tag_update_seq_;
    }

    bool yawReachedForStableFrames() {
        if (tag_update_seq_ == last_yaw_stable_seq_) {
            return yaw_stable_count_ >= yaw_stable_required_;
        }

        last_yaw_stable_seq_ = tag_update_seq_;
        if (std::abs(yawError()) <= rotation_threshold_) {
            ++yaw_stable_count_;
        } else {
            yaw_stable_count_ = 0;
        }

        return yaw_stable_count_ >= yaw_stable_required_;
    }

    double speedFromError(double error, double kp, double arrival_threshold) {
        if (std::abs(error) <= arrival_threshold) {
            return 0.0;
        }

        double speed = kp * error;
        if (std::abs(speed) < min_speed_) {
            speed = (speed >= 0.0) ? min_speed_ : -min_speed_;
        }

        return clamp(speed, -max_speed_, max_speed_);
    }

    double lateralSpeedFromError(double error) {
        return speedFromError(error, kp_x_, arrival_threshold_x_);
    }

    double yawError() const {
        return normalizeAngleRad(current_yaw_rad_ - target_yaw_rad_);
    }

    double computeYawCommandWithKp(double yaw_kp) {
        const ros::Time now = ros::Time::now();
        const double yaw_error = yawError();
        double yaw_rate_error = 0.0;

        double dt = 0.0;
        if (yaw_pd_initialized_) {
            dt = (now - previous_yaw_time_).toSec();
            if (dt > 1e-3) {
                yaw_rate_error = (yaw_error - previous_yaw_error_) / dt;
            }
        }

        previous_yaw_error_ = yaw_error;
        previous_yaw_time_ = now;

        const double raw_angular_speed =
            clamp(yaw_kp * yaw_error + yaw_kd_ * yaw_rate_error,
                  -max_angular_speed_, max_angular_speed_);

        double angular_speed = raw_angular_speed;
        if (yaw_pd_initialized_ && dt > 1e-3) {
            const double max_delta = max_angular_accel_ * dt;
            angular_speed = clamp(
                raw_angular_speed,
                previous_angular_speed_ - max_delta,
                previous_angular_speed_ + max_delta);
        }

        previous_angular_speed_ = angular_speed;
        yaw_pd_initialized_ = true;
        return angular_speed;
    }

    double computeYawCommand() {
        return computeYawCommandWithKp(yaw_kp_);
    }

    void publishStop(ros::Rate& rate, int count = 3) {
        geometry_msgs::Twist cmd_vel_msg;
        for (int i = 0; i < count; ++i) {
            cmd_vel_publisher_.publish(cmd_vel_msg);
            rate.sleep();
        }
    }

    // 服务回调函数
    bool handleSetTargetY(x2bot_teleop::SetTagY::Request& req, x2bot_teleop::SetTagY::Response& res) {
        target_x_m_ = req.target_x;
        target_y_m_ = req.target_y;
        target_yaw_rad_ = degToRad(req.target_angle);
        ROS_INFO("[TAGCTL] Received target x: %.3f m, target y: %.3f m, target angle: %.2f deg",
                 target_x_m_, target_y_m_, req.target_angle);
        state_ = ROTATING;
        yaw_pd_initialized_ = false;
        previous_angular_speed_ = 0.0;
        resetYawStableCounter();

        ros::Rate rate(30);  // 30 Hz
        while (ros::ok()) {
            // 检查数据是否过期
            if (isDataStale() || (current_valid_ <0) ) {
                ROS_WARN_THROTTLE(1.0, "[TAGCTL] Tag position data is stale or invalid. Stopping the robot.");
                publishStop(rate, 3);
                for (int i = 0; i < 20 && ros::ok(); ++i) {
                    rate.sleep();
                }
                continue;
            }

            if (state_ == ROTATING) {
                // 初始旋转阶段：调整方向直到当前角度接近目标角度
                if (yawReachedForStableFrames()) {
                    publishStop(rate, 2);
                    ROS_INFO("[TAGCTL] Initial rotation completed after %d stable tag frames. Switching to x correction state.",
                             yaw_stable_count_);
                    state_ = CORRECTING_X;  // 切换到 x 修正状态
                    yaw_pd_initialized_ = false;
                    previous_angular_speed_ = 0.0;
                    resetYawStableCounter();
                } else {
                    // PD 角度控制，最大角速度由 max_angular_speed 限制
                    double angular_speed = computeYawCommand();
                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.angular.z = angular_speed;
                    cmd_vel_publisher_.publish(cmd_vel_msg);
                    ROS_INFO_THROTTLE(1.0, "%s[TAGCTL] Rotating: yaw=%.3f rad, target_yaw=%.3f rad, yaw_err=%.3f rad, angular.z=%.3f, stable=%d/%d%s",
                                      COLOR_YELLOW,
                                      current_yaw_rad_, target_yaw_rad_, yawError(), cmd_vel_msg.angular.z,
                                      yaw_stable_count_, yaw_stable_required_,
                                      COLOR_RESET);
                }
            } else if (state_ == CORRECTING_X) {
                const double error_x_m = current_x_m_ - target_x_m_;
                const bool x_reached = std::abs(error_x_m) <= arrival_threshold_x_;

                if (x_reached) {
                    publishStop(rate, 2);
                    ROS_INFO("[TAGCTL] X reached. Switching to Y+Yaw state. x=%.3f m, target_x=%.3f m, yaw_err=%.3f rad",
                             current_x_m_, target_x_m_, yawError());
                    state_ = MOVING_Y_YAW;
                    yaw_pd_initialized_ = false;
                    previous_angular_speed_ = 0.0;
                    resetYawStableCounter();
                } else {
                    const double linear_y_speed = lateralSpeedFromError(error_x_m);

                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = linear_y_speed;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel_msg);

                    ROS_INFO_THROTTLE(
                        1.0,
                        "%s[TAGCTL] Correcting X: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, yaw_err=%.3f rad%s",
                        COLOR_YELLOW,
                        cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z,
                        error_x_m, yawError(),
                        COLOR_RESET);
                }
            } else if (state_ == MOVING_Y_YAW) {
                const double error_y_m = target_y_m_ - current_y_m_;
                const double error_x_m = current_x_m_ - target_x_m_;
                const bool x_reached = std::abs(error_x_m) <= arrival_threshold_x_;
                const bool y_reached = std::abs(error_y_m) <= arrival_threshold_y_;
                const bool yaw_reached = yawReachedForStableFrames();

                if (y_reached) {
                    if (x_reached) {
                        publishStop(rate, 2);
                        ROS_INFO("[TAGCTL] X and Y reached. Stop Y+Yaw without standalone yaw rotation. x=%.3f m, y=%.3f m, yaw_err=%.3f rad",
                                 current_x_m_, current_y_m_, yawError());
                        res.success = true;
                        return true;
                    }

                    publishStop(rate, 2);
                    ROS_INFO("[TAGCTL] Y reached. Switching to final X state without standalone yaw rotation. x=%.3f m, target_x=%.3f m, err_x=%.3f m, yaw_err=%.3f rad",
                             current_x_m_, target_x_m_, error_x_m, yawError());
                    state_ = FINAL_CORRECTING_X;
                    yaw_pd_initialized_ = false;
                    previous_angular_speed_ = 0.0;
                    resetYawStableCounter();
                } else {
                    const double linear_x_speed = speedFromError(error_y_m, kp_, arrival_threshold_y_);
                    const bool yaw_cmd_enabled =
                        !yaw_reached &&
                        linear_x_speed != 0.0 &&
                        std::abs(yawError()) > final_yaw_cmd_deadband_;
                    const double angular_speed = yaw_cmd_enabled ? computeYawCommandWithKp(final_yaw_kp_) : 0.0;

                    // 创建并发布 /cmd_vel 消息
                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = linear_x_speed;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.angular.z = angular_speed;
                    cmd_vel_publisher_.publish(cmd_vel_msg);

                    ROS_INFO_THROTTLE(
                        1.0,
                        "%s[TAGCTL] Moving Y+Yaw: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, err_y=%.3f m, yaw_err=%.3f rad, yaw_stable=%d/%d%s",
                        COLOR_YELLOW,
                        cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z,
                        error_x_m, error_y_m, yawError(), yaw_stable_count_, yaw_stable_required_,
                        COLOR_RESET);
                }
            } else if (state_ == FINAL_CORRECTING_X) {
                const double error_y_m = target_y_m_ - current_y_m_;
                const double error_x_m = current_x_m_ - target_x_m_;
                const bool x_reached = std::abs(error_x_m) <= arrival_threshold_x_;
                const bool y_reached = std::abs(error_y_m) <= arrival_threshold_y_;
                const bool yaw_in_threshold = std::abs(yawError()) <= rotation_threshold_;
                const bool yaw_reached = yawReachedForStableFrames();

                if (x_reached) {
                    if (!enable_final_y_yaw_after_x_check_) {
                        publishStop(rate, 2);
                        ROS_INFO("[TAGCTL] Final X reached. Final Y+Yaw recheck disabled. Reached target position! x=%.3f m, y=%.3f m, yaw=%.3f rad, target_yaw=%.3f rad",
                                 current_x_m_, current_y_m_, current_yaw_rad_, target_yaw_rad_);
                        res.success = true;
                        return true;
                    }

                    if (y_reached && yaw_reached) {
                        publishStop(rate, 2);
                        ROS_INFO("[TAGCTL] Final X, Y and yaw reached. Reached target position! x=%.3f m, y=%.3f m, yaw=%.3f rad, target_yaw=%.3f rad",
                                 current_x_m_, current_y_m_, current_yaw_rad_, target_yaw_rad_);
                        res.success = true;
                        return true;
                    }

                    if (!y_reached || !yaw_in_threshold) {
                        publishStop(rate, 2);
                        ROS_INFO("[TAGCTL] Final X reached, but Y/Yaw drifted. Switching back to Y+Yaw state. err_y=%.3f m, yaw_err=%.3f rad",
                                 error_y_m, yawError());
                        state_ = MOVING_Y_YAW;
                        yaw_pd_initialized_ = false;
                        previous_angular_speed_ = 0.0;
                        resetYawStableCounter();
                    } else {
                        publishStop(rate, 1);
                        ROS_INFO_THROTTLE(
                            1.0,
                            "%s[TAGCTL] Final X reached. Waiting for yaw stable frames: yaw_err=%.3f rad, stable=%d/%d%s",
                            COLOR_YELLOW, yawError(), yaw_stable_count_, yaw_stable_required_, COLOR_RESET);
                    }
                } else {
                    const double linear_y_speed = lateralSpeedFromError(error_x_m);

                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = linear_y_speed;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel_msg);

                    ROS_INFO_THROTTLE(
                        1.0,
                        "%s[TAGCTL] Final correcting X: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, err_y=%.3f m, yaw_err=%.3f rad%s",
                        COLOR_YELLOW,
                        cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z,
                        error_x_m, error_y_m, yawError(),
                        COLOR_RESET);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }

        res.success = false;
        return false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    RobotController controller(nh);
    controller.run();

    return 0;
}
