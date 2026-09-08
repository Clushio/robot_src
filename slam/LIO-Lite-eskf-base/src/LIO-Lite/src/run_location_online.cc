#include <unistd.h>
#include <cfenv>
#include <csignal>

#if defined(__SSE__)
#include <xmmintrin.h>
#endif

#include "custom_logs.hpp"
#include "laser_mapping.h"


DEFINE_bool(need_logs, true, "save logs for check");


void SigHandle(int sig) {
    lio_lite::options::FLAG_EXIT = true;
    LOG(WARNING) << "catch sig " << sig;
}


namespace {

void ConfigureNonStopFloatingPoint() {
    // 恢复默认浮点环境
    std::fesetenv(FE_DFL_ENV);

    // 清除当前已经产生的浮点异常标志
    std::feclearexcept(FE_ALL_EXCEPT);

#if defined(__SSE__)
    constexpr unsigned int kExceptionFlags = 0x003fu;
    constexpr unsigned int kExceptionMasks = 0x1f80u;

    const unsigned int old_mxcsr = _mm_getcsr();

    // 清除异常 flag，同时屏蔽所有 SSE 浮点异常 trap
    const unsigned int new_mxcsr =
        (old_mxcsr & ~kExceptionFlags) | kExceptionMasks;

    _mm_setcsr(new_mxcsr);

    LOG(INFO)
        << "configured non-stop floating point: MXCSR 0x"
        << std::hex << old_mxcsr
        << " -> 0x" << new_mxcsr
        << std::dec;
#endif
}

}  // namespace


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    init_log();

    if (FLAGS_need_logs) {
        save_log("local");
    }

    // ROS2 初始化
    rclcpp::init(argc, argv);

    // ROS2 初始化完成以后，再恢复浮点环境
    ConfigureNonStopFloatingPoint();

    auto node = std::make_shared<rclcpp::Node>("lio_lite");

    LOG(INFO) << "\033[1;32m run_location_online \033[0m";

    auto laser_mapping = std::make_shared<lio_lite::LaserMapping>();

    if (!laser_mapping->InitROS(node)) {
        LOG(ERROR) << "failed to initialize LIO";

        rclcpp::shutdown();
        return 1;
    }

    signal(SIGINT, SigHandle);

    rclcpp::WallRate rate(5000.0);

    // 加载定位地图
    laser_mapping->Load_map();

    while (rclcpp::ok()) {
        if (lio_lite::options::FLAG_EXIT) {
            break;
        }

        rclcpp::spin_some(node);

        laser_mapping->Run_location();

        rate.sleep();
    }

    LOG(INFO) << "finishing!";

    laser_mapping->Finish();

    lio_lite::Timer::PrintAll();

    rclcpp::shutdown();

    return 0;
}