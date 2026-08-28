#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>

#include <cmd_vel_arbiter/srv/finish_motion.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <x2bot_teleop/srv/set_tag_y.hpp>

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

void addDiagnosticValue(diagnostic_msgs::msg::DiagnosticStatus& status,
                        const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
}

class ActiveControlGuard {
public:
    explicit ActiveControlGuard(std::atomic<bool>& active) : active_(active) {}
    ~ActiveControlGuard() { active_.store(false); }

private:
    std::atomic<bool>& active_;
};
}

class RobotController : public rclcpp::Node {
public:
    RobotController()
        : Node("robot_controller"),
          current_x_m_(0.0),
          current_y_m_(0.0),
          target_x_m_(0.0),
          target_y_m_(0.0),
          target_yaw_rad_(0.0),
          current_yaw_rad_(0.0),
          last_update_time_(0, 0, get_clock()->get_clock_type()),
          current_valid_(-99.0),
          filter_initialized_(false),
          yaw_pd_initialized_(false),
          previous_yaw_error_(0.0),
          previous_angular_speed_(0.0),
          previous_yaw_time_(0, 0, get_clock()->get_clock_type()),
          yaw_stable_count_(0),
          yaw_stable_required_(5),
          tag_update_seq_(0),
          last_yaw_stable_seq_(0),
          control_active_(false) {
        // 初始化参数
        kp_ = declare_parameter<double>("kp", 0.5);
        kp_x_ = declare_parameter<double>("kp_x", 0.5);
        target_x_m_ = declare_parameter<double>("target_x", 0.0);
        min_speed_ = declare_parameter<double>("min_speed", 0.01);
        max_speed_ = declare_parameter<double>("max_speed", 0.05);
        arrival_threshold_y_ = declare_parameter<double>("arrival_threshold_y", 0.005);
        arrival_threshold_y_ = declare_parameter<double>("arrival_threshold", arrival_threshold_y_);
        arrival_threshold_x_ = declare_parameter<double>("arrival_threshold_x", 0.005);
        data_timeout_ = declare_parameter<double>("data_timeout", 3.0);
        tag_abort_timeout_ = declare_parameter<double>("tag_abort_timeout", 10.0);
        rotation_threshold_ = declare_parameter<double>("rotation_threshold", 0.015);
        yaw_kp_ = declare_parameter<double>("yaw_kp", 0.5);
        yaw_kd_ = declare_parameter<double>("yaw_kd", 0.0);
        max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.15);
        max_angular_accel_ = declare_parameter<double>("max_angular_accel", 0.15);
        filter_alpha_ = declare_parameter<double>("filter_alpha", 0.5);
        yaw_stable_required_ = declare_parameter<int>("yaw_stable_count", 5);
        final_yaw_kp_ = declare_parameter<double>("final_yaw_kp", 0.2);
        final_yaw_cmd_deadband_ =
            declare_parameter<double>("final_yaw_cmd_deadband", rotation_threshold_);
        enable_final_y_yaw_after_x_check_ =
            declare_parameter<bool>("enable_final_y_yaw_after_x_check", false);

        sanitizeParameters();

        // 订阅 /tag_position 话题
        tag_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = tag_callback_group_;
        tag_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tag_position", rclcpp::QoS(10),
            std::bind(&RobotController::tagCallback, this, std::placeholders::_1),
            subscription_options);

        // 发布到仲裁器输入；只有 cmd_vel_arbiter 可以发布 /cmd_vel。
        std::string cmd_vel_topic;
        cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel/tag");
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        diagnostics_publisher_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", rclcpp::QoS(10).transient_local());
        finish_motion_client_ =
            create_client<cmd_vel_arbiter::srv::FinishMotion>(
                "/cmd_vel_arbiter/finish_motion", rmw_qos_profile_services_default,
                client_callback_group_);

        // 创建服务
        service_ = create_service<x2bot_teleop::srv::SetTagY>(
            "set_target_y",
            std::bind(
                &RobotController::handleSetTargetY, this,
                std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_callback_group_);

        RCLCPP_INFO(get_logger(),
            "[TAGCTL] Robot controller node initialized. kp=%.3f kp_x=%.3f min_speed=%.3f max_speed=%.3f "
            "arrival_y=%.3f arrival_x=%.3f yaw_kp=%.3f yaw_kd=%.3f max_ang=%.3f max_ang_acc=%.3f "
            "yaw_stable_count=%d final_yaw_kp=%.3f final_yaw_cmd_deadband=%.3f "
            "enable_final_y_yaw_after_x_check=%d data_timeout=%.1f tag_abort_timeout=%.1f",
            kp_, kp_x_, min_speed_, max_speed_, arrival_threshold_y_, arrival_threshold_x_,
            yaw_kp_, yaw_kd_, max_angular_speed_, max_angular_accel_,
            yaw_stable_required_, final_yaw_kp_, final_yaw_cmd_deadband_,
            enable_final_y_yaw_after_x_check_, data_timeout_, tag_abort_timeout_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr tag_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::Service<x2bot_teleop::srv::SetTagY>::SharedPtr service_;
    rclcpp::Client<cmd_vel_arbiter::srv::FinishMotion>::SharedPtr finish_motion_client_;

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
    rclcpp::Time last_update_time_;   // 上次更新时间
    double data_timeout_;          // 数据超时时间（秒）
    double tag_abort_timeout_;     // 连续无效数据导致任务失败的超时
    double current_valid_;
    bool filter_initialized_;
    bool yaw_pd_initialized_;
    double previous_yaw_error_;
    double previous_angular_speed_;
    rclcpp::Time previous_yaw_time_;
    int yaw_stable_count_;
    int yaw_stable_required_;
    unsigned int tag_update_seq_;
    unsigned int last_yaw_stable_seq_;
    std::atomic<bool> control_active_;

    enum State { ROTATING, CORRECTING_X, MOVING_Y_YAW, FINAL_CORRECTING_X };
    State state_ = ROTATING;       // 初始状态为旋转

    // 订阅回调函数
    void tagCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        current_valid_ = msg->pose.position.z;
        if(current_valid_>=-1)
        {
            const double measured_x_m = msg->pose.position.x / 100.0;
            const double measured_y_m = msg->pose.position.y / 100.0;

            const double yaw = tf2::getYaw(msg->pose.orientation);

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

        last_update_time_ = now();         // 更新时间戳
    }

    // 检查数据是否过期
    bool isDataStale() {
        if (last_update_time_.nanoseconds() == 0) {
            return true;
        }
        return (now() - last_update_time_).seconds() > data_timeout_;
    }

    // 限制速度范围
    double clamp(double value, double min_value, double max_value) {
        return std::max(std::min(value, max_value), min_value);
    }

    void sanitizeParameters() {
        if (kp_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid kp %.3f, reset to 0.5", kp_);
            kp_ = 0.5;
        }
        if (kp_x_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid kp_x %.3f, reset to 0.5", kp_x_);
            kp_x_ = 0.5;
        }
        if (min_speed_ < 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid min_speed %.3f, reset to 0.01", min_speed_);
            min_speed_ = 0.01;
        }
        if (max_speed_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid max_speed %.3f, reset to 0.05", max_speed_);
            max_speed_ = 0.05;
        }
        if (min_speed_ > max_speed_) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] min_speed %.3f is larger than max_speed %.3f, clamp min_speed to max_speed.",
                     min_speed_, max_speed_);
            min_speed_ = max_speed_;
        }
        if (arrival_threshold_y_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid arrival_threshold_y %.3f, reset to 0.01", arrival_threshold_y_);
            arrival_threshold_y_ = 0.01;
        }
        if (arrival_threshold_x_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid arrival_threshold_x %.3f, reset to 0.1", arrival_threshold_x_);
            arrival_threshold_x_ = 0.1;
        }
        if (max_angular_speed_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid max_angular_speed %.3f, reset to 0.15", max_angular_speed_);
            max_angular_speed_ = 0.15;
        }
        if (max_angular_accel_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid max_angular_accel %.3f, reset to 0.15", max_angular_accel_);
            max_angular_accel_ = 0.15;
        }
        if (tag_abort_timeout_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid tag_abort_timeout %.3f, reset to 10.0", tag_abort_timeout_);
            tag_abort_timeout_ = 10.0;
        }
        if (yaw_stable_required_ <= 0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid yaw_stable_count %d, reset to 5", yaw_stable_required_);
            yaw_stable_required_ = 5;
        }
        if (final_yaw_kp_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid final_yaw_kp %.3f, reset to 0.1", final_yaw_kp_);
            final_yaw_kp_ = 0.1;
        }
        if (final_yaw_cmd_deadband_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Invalid final_yaw_cmd_deadband %.3f, reset to rotation_threshold %.3f",
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
        const auto current_time = now();
        const double yaw_error = yawError();
        double yaw_rate_error = 0.0;

        double dt = 0.0;
        if (yaw_pd_initialized_) {
            dt = (current_time - previous_yaw_time_).seconds();
            if (dt > 1e-3) {
                yaw_rate_error = (yaw_error - previous_yaw_error_) / dt;
            }
        }

        previous_yaw_error_ = yaw_error;
        previous_yaw_time_ = current_time;

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

    void publishStop(rclcpp::Rate& rate, int count = 3) {
        geometry_msgs::msg::Twist cmd_vel_msg;
        for (int i = 0; i < count; ++i) {
            cmd_vel_publisher_->publish(cmd_vel_msg);
            rate.sleep();
        }
    }

    bool notifyMotionFinished(uint8_t reason) {
        if (!finish_motion_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "[TAGCTL] Failed to call /cmd_vel_arbiter/finish_motion.");
            publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "ANAV-TAG-010", "标签任务停车回正服务调用失败",
                              "无法连接 /cmd_vel_arbiter/finish_motion。",
                              "检查速度仲裁与底盘节点。", true);
            return false;
        }
        auto request = std::make_shared<cmd_vel_arbiter::srv::FinishMotion::Request>();
        request->source = "tag";
        request->reason = reason;
        auto future = finish_motion_client_->async_send_request(request);
        future.wait();
        const auto response = future.get();
        if (!response->centered) {
            RCLCPP_ERROR(
                get_logger(),
                "[TAGCTL] Tag motion stopped, but centering was not confirmed: %s",
                response->message.c_str());
            publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "ANAV-TAG-010", "标签任务结束但轮组未确认回正",
                              response->message,
                              "检查转向执行机构和底盘停车回正日志。", true);
        } else if (reason == cmd_vel_arbiter::srv::FinishMotion::Request::TASK_FINISHED) {
            publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                              "ANAV-TAG-000", "标签靠站任务完成", "", "", false);
        }
        return response->centered;
    }

    void publishDiagnostic(uint8_t level, const std::string& code,
                           const std::string& message,
                           const std::string& detail,
                           const std::string& action, bool active) {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.level = level;
        status.name = "/anav/tag_control";
        status.hardware_id = "ranger";
        status.message = message;
        addDiagnosticValue(status, "code", code);
        addDiagnosticValue(status, "active", active ? "true" : "false");
        addDiagnosticValue(status, "kind", active ? "FAULT" : "STATE");
        addDiagnosticValue(status, "detail", detail);
        addDiagnosticValue(status, "action", action);
        array.status.push_back(status);
        diagnostics_publisher_->publish(array);
    }

    // 服务回调函数
    void handleSetTargetY(
        const std::shared_ptr<x2bot_teleop::srv::SetTagY::Request> req,
        std::shared_ptr<x2bot_teleop::srv::SetTagY::Response> res) {
        bool expected = false;
        if (!control_active_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(get_logger(), "[TAGCTL] Reject a new target because another Tag control request is still active.");
            res->success = false;
            publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                              "ANAV-TAG-008", "标签控制器正忙",
                              "已有一个靠站请求正在执行。",
                              "等待当前任务结束后再发起新请求。", true);
            return;
        }
        ActiveControlGuard control_guard(control_active_);

        target_x_m_ = req->target_x;
        target_y_m_ = req->target_y;
        target_yaw_rad_ = degToRad(req->target_angle);
        RCLCPP_INFO(get_logger(), "[TAGCTL] Received target x: %.3f m, target y: %.3f m, target angle: %.2f deg",
                 target_x_m_, target_y_m_, req->target_angle);
        state_ = ROTATING;
        yaw_pd_initialized_ = false;
        previous_angular_speed_ = 0.0;
        resetYawStableCounter();

        rclcpp::Rate rate(30.0);  // 30 Hz
        std::chrono::steady_clock::time_point invalid_since;
        bool invalid_active = false;
        while (rclcpp::ok()) {

            // 检查数据是否过期
            if (isDataStale() || (current_valid_ <0) ) {
                if (!invalid_active) {
                    invalid_since = std::chrono::steady_clock::now();
                    invalid_active = true;
                    publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                                      "ANAV-TAG-006", "标签数据无效或超时",
                                      "活动靠站任务未收到有效 /tag_position。",
                                      "检查标签是否在视野内、串口读取和识别节点。", true);
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[TAGCTL] Tag position data is stale or invalid. Stopping the robot.");
                publishStop(rate, 3);
                for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
                    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
                    if (!isDataStale() && current_valid_ >= 0) {
                        RCLCPP_INFO(get_logger(), "[TAGCTL] Tag data became valid; resume the active control request.");
                        invalid_active = false;
                        publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                                          "ANAV-TAG-000", "标签数据已恢复",
                                          "", "", false);
                        break;
                    }
                    if (std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - invalid_since).count() >= tag_abort_timeout_) {
                        RCLCPP_ERROR(get_logger(), "[TAGCTL] Tag remained invalid for %.1f seconds; abort the task.",
                                  tag_abort_timeout_);
                        publishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                                          "ANAV-TAG-007", "标签数据持续超时，任务已终止",
                                          "超过 tag_abort_timeout 仍无有效标签数据。",
                                          "检查标签、相机/串口链路后重新发起任务。", true);
                        publishStop(rate, 3);
                        notifyMotionFinished(
                            cmd_vel_arbiter::srv::FinishMotion::Request::TASK_FAILED);
                        res->success = false;
                        return;
                    }
                    rate.sleep();
                }
                continue;
            }
            invalid_active = false;

            if (state_ == ROTATING) {
                // 初始旋转阶段：调整方向直到当前角度接近目标角度
                if (yawReachedForStableFrames()) {
                    publishStop(rate, 2);
                    RCLCPP_INFO(get_logger(), "[TAGCTL] Initial rotation completed after %d stable tag frames. Switching to x correction state.",
                             yaw_stable_count_);
                    state_ = CORRECTING_X;  // 切换到 x 修正状态
                    yaw_pd_initialized_ = false;
                    previous_angular_speed_ = 0.0;
                    resetYawStableCounter();
                } else {
                    // PD 角度控制，最大角速度由 max_angular_speed 限制
                    double angular_speed = computeYawCommand();
                    geometry_msgs::msg::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.angular.z = angular_speed;
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s[TAGCTL] Rotating: yaw=%.3f rad, target_yaw=%.3f rad, yaw_err=%.3f rad, angular.z=%.3f, stable=%d/%d%s",
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
                    RCLCPP_INFO(get_logger(), "[TAGCTL] X reached. Switching to Y+Yaw state. x=%.3f m, target_x=%.3f m, yaw_err=%.3f rad",
                             current_x_m_, target_x_m_, yawError());
                    state_ = MOVING_Y_YAW;
                    yaw_pd_initialized_ = false;
                    previous_angular_speed_ = 0.0;
                    resetYawStableCounter();
                } else {
                    const double linear_y_speed = lateralSpeedFromError(error_x_m);

                    geometry_msgs::msg::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = linear_y_speed;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(cmd_vel_msg);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s[TAGCTL] Correcting X: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, yaw_err=%.3f rad%s",
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
                        RCLCPP_INFO(get_logger(), "[TAGCTL] X and Y reached. Stop Y+Yaw without standalone yaw rotation. x=%.3f m, y=%.3f m, yaw_err=%.3f rad",
                                 current_x_m_, current_y_m_, yawError());
                        notifyMotionFinished(
                            cmd_vel_arbiter::srv::FinishMotion::Request::TASK_FINISHED);
                        res->success = true;
                        return;
                    }

                    publishStop(rate, 2);
                    RCLCPP_INFO(get_logger(), "[TAGCTL] Y reached. Switching to final X state without standalone yaw rotation. x=%.3f m, target_x=%.3f m, err_x=%.3f m, yaw_err=%.3f rad",
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
                    geometry_msgs::msg::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = linear_x_speed;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.angular.z = angular_speed;
                    cmd_vel_publisher_->publish(cmd_vel_msg);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s[TAGCTL] Moving Y+Yaw: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, err_y=%.3f m, yaw_err=%.3f rad, yaw_stable=%d/%d%s",
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
                        RCLCPP_INFO(get_logger(), "[TAGCTL] Final X reached. Final Y+Yaw recheck disabled. Reached target position! x=%.3f m, y=%.3f m, yaw=%.3f rad, target_yaw=%.3f rad",
                                 current_x_m_, current_y_m_, current_yaw_rad_, target_yaw_rad_);
                        notifyMotionFinished(
                            cmd_vel_arbiter::srv::FinishMotion::Request::TASK_FINISHED);
                        res->success = true;
                        return;
                    }

                    if (y_reached && yaw_reached) {
                        publishStop(rate, 2);
                        RCLCPP_INFO(get_logger(), "[TAGCTL] Final X, Y and yaw reached. Reached target position! x=%.3f m, y=%.3f m, yaw=%.3f rad, target_yaw=%.3f rad",
                                 current_x_m_, current_y_m_, current_yaw_rad_, target_yaw_rad_);
                        notifyMotionFinished(
                            cmd_vel_arbiter::srv::FinishMotion::Request::TASK_FINISHED);
                        res->success = true;
                        return;
                    }

                    if (!y_reached || !yaw_in_threshold) {
                        publishStop(rate, 2);
                        RCLCPP_INFO(get_logger(), "[TAGCTL] Final X reached, but Y/Yaw drifted. Switching back to Y+Yaw state. err_y=%.3f m, yaw_err=%.3f rad",
                                 error_y_m, yawError());
                        state_ = MOVING_Y_YAW;
                        yaw_pd_initialized_ = false;
                        previous_angular_speed_ = 0.0;
                        resetYawStableCounter();
                    } else {
                        publishStop(rate, 1);
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s[TAGCTL] Final X reached. Waiting for yaw stable frames: yaw_err=%.3f rad, stable=%d/%d%s",
                            COLOR_YELLOW, yawError(), yaw_stable_count_, yaw_stable_required_, COLOR_RESET);
                    }
                } else {
                    const double linear_y_speed = lateralSpeedFromError(error_x_m);

                    geometry_msgs::msg::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = linear_y_speed;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(cmd_vel_msg);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s[TAGCTL] Final correcting X: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | err_x=%.3f m, err_y=%.3f m, yaw_err=%.3f rad%s",
                        COLOR_YELLOW,
                        cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z,
                        error_x_m, error_y_m, yawError(),
                        COLOR_RESET);
                }
            }

            rate.sleep();
        }

        res->success = false;
        return;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<RobotController>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
