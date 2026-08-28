
#include <unistd.h>
#include <csignal>

#include "custom_logs.hpp"
#include "laser_mapping.h"

DEFINE_bool(need_logs, true, "save logs for check");
void SigHandle(int sig) {
    lio_lite::options::FLAG_EXIT = true;
    LOG(WARNING) << "catch sig " << sig;
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    init_log();
    if(FLAGS_need_logs){
        save_log("local");
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lio_lite");
    
    LOG(INFO) << "\033[1;32m run_location_online \033[0m";

    auto laser_mapping = std::make_shared<lio_lite::LaserMapping>();
    if (!laser_mapping->InitROS(node)) {
        LOG(ERROR) << "failed to initialize LIO";
        rclcpp::shutdown();
        return 1;
    }
    //time.sleep(5)

    signal(SIGINT, SigHandle);
    rclcpp::WallRate rate(5000.0);
    
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
