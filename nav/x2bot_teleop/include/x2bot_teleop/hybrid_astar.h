#ifndef X2BOT_TELEOP_HYBRID_ASTAR_H_
#define X2BOT_TELEOP_HYBRID_ASTAR_H_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace x2bot_teleop
{

struct HybridPose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct HybridAStarOptions
{
    double xy_resolution = 0.10;
    int heading_bins = 72;
    double primitive_length = 0.10;
    double collision_sample_step = 0.025;
    double min_turn_radius = 0.55;
    double goal_xy_tolerance = 0.12;
    double goal_yaw_tolerance = 12.0 * 3.14159265358979323846 / 180.0;
    double steering_cost = 0.08;
    double steering_change_cost = 0.12;
    double heuristic_weight = 1.05;
    double search_margin = 1.5;
    double max_planning_time = 0.35;
    std::size_t max_iterations = 60000;
};

struct HybridAStarResult
{
    bool success = false;
    std::string reason;
    double cost = 0.0;
    std::size_t expanded = 0;
    std::vector<HybridPose> poses;
};

class HybridAStar
{
public:
    using PoseFreeFn = std::function<bool(double, double, double)>;

    explicit HybridAStar(const HybridAStarOptions &options = HybridAStarOptions());

    HybridAStarResult plan(const HybridPose &start,
                           const HybridPose &goal,
                           const PoseFreeFn &pose_free) const;

    static std::vector<HybridPose> guidePoints(
        const std::vector<HybridPose> &path,
        double max_straight_spacing = 0.40,
        double preserve_turn_angle = 3.0 * 3.14159265358979323846 / 180.0);

private:
    HybridAStarOptions options_;
};

}  // namespace x2bot_teleop

#endif  // X2BOT_TELEOP_HYBRID_ASTAR_H_
