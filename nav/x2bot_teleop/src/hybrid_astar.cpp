#include <x2bot_teleop/hybrid_astar.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace x2bot_teleop
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle <= -kPi) angle += 2.0 * kPi;
    return angle;
}

struct Key
{
    int x;
    int y;
    int yaw;

    bool operator==(const Key &other) const
    {
        return x == other.x && y == other.y && yaw == other.yaw;
    }
};

struct KeyHash
{
    std::size_t operator()(const Key &key) const
    {
        std::size_t value = static_cast<std::size_t>(key.x) * 73856093U;
        value ^= static_cast<std::size_t>(key.y) * 19349663U;
        value ^= static_cast<std::size_t>(key.yaw) * 83492791U;
        return value;
    }
};

struct Node
{
    HybridPose pose;
    double g = 0.0;
    double curvature = 0.0;
    int parent = -1;
};

struct QueueItem
{
    double f;
    int node;

    bool operator>(const QueueItem &other) const { return f > other.f; }
};

HybridPose propagate(const HybridPose &pose, double curvature, double distance)
{
    HybridPose result = pose;
    if (std::fabs(curvature) < 1e-9)
    {
        result.x += distance * std::cos(pose.yaw);
        result.y += distance * std::sin(pose.yaw);
        return result;
    }
    result.yaw = normalizeAngle(pose.yaw + curvature * distance);
    result.x += (std::sin(result.yaw) - std::sin(pose.yaw)) / curvature;
    result.y += (-std::cos(result.yaw) + std::cos(pose.yaw)) / curvature;
    return result;
}

double distance(const HybridPose &a, const HybridPose &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

HybridAStar::HybridAStar(const HybridAStarOptions &options) : options_(options)
{
    options_.xy_resolution = std::max(0.02, options_.xy_resolution);
    options_.heading_bins = std::max(16, options_.heading_bins);
    options_.primitive_length = std::max(0.04, options_.primitive_length);
    options_.collision_sample_step = std::max(0.005, options_.collision_sample_step);
    options_.min_turn_radius = std::max(0.10, options_.min_turn_radius);
    options_.goal_xy_tolerance = std::max(options_.xy_resolution,
                                           options_.goal_xy_tolerance);
    options_.goal_yaw_tolerance = std::max(1.0 * kPi / 180.0,
                                            options_.goal_yaw_tolerance);
    options_.max_planning_time = std::max(0.01, options_.max_planning_time);
    options_.max_iterations = std::max<std::size_t>(100, options_.max_iterations);
}

HybridAStarResult HybridAStar::plan(const HybridPose &start,
                                    const HybridPose &goal,
                                    const PoseFreeFn &pose_free) const
{
    HybridAStarResult result;
    if (!pose_free || !std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(start.yaw) || !std::isfinite(goal.x) ||
        !std::isfinite(goal.y) || !std::isfinite(goal.yaw))
    {
        result.reason = "invalid input";
        return result;
    }
    if (!pose_free(start.x, start.y, start.yaw) ||
        !pose_free(goal.x, goal.y, goal.yaw))
    {
        result.reason = "start or goal footprint is blocked";
        return result;
    }

    const double direct_distance = distance(start, goal);
    const double margin = std::max(options_.search_margin, direct_distance * 0.75);
    const double min_x = std::min(start.x, goal.x) - margin;
    const double max_x = std::max(start.x, goal.x) + margin;
    const double min_y = std::min(start.y, goal.y) - margin;
    const double max_y = std::max(start.y, goal.y) + margin;
    const double max_curvature = 1.0 / options_.min_turn_radius;
    const double curvatures[] = {
        -max_curvature, -0.5 * max_curvature, 0.0,
        0.5 * max_curvature, max_curvature};

    const auto make_key = [&](const HybridPose &pose) {
        Key key;
        key.x = static_cast<int>(std::floor(pose.x / options_.xy_resolution));
        key.y = static_cast<int>(std::floor(pose.y / options_.xy_resolution));
        const double normalized = normalizeAngle(pose.yaw) + kPi;
        key.yaw = std::min(options_.heading_bins - 1,
            static_cast<int>(std::floor(normalized / (2.0 * kPi) *
                                         options_.heading_bins)));
        return key;
    };
    const auto heuristic = [&](const HybridPose &pose) {
        const double heading = std::fabs(normalizeAngle(goal.yaw - pose.yaw));
        return distance(pose, goal) + 0.15 * options_.min_turn_radius * heading;
    };
    const auto transition_free = [&](const HybridPose &pose, double curvature) {
        const int samples = std::max(1, static_cast<int>(std::ceil(
            options_.primitive_length / options_.collision_sample_step)));
        for (int sample = 1; sample <= samples; ++sample)
        {
            const HybridPose check = propagate(
                pose, curvature,
                options_.primitive_length * static_cast<double>(sample) / samples);
            if (!pose_free(check.x, check.y, check.yaw)) return false;
        }
        return true;
    };

    std::vector<Node> nodes;
    nodes.reserve(std::min<std::size_t>(options_.max_iterations, 10000));
    Node first;
    first.pose = start;
    nodes.push_back(first);
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    open.push(QueueItem{heuristic(start), 0});
    std::unordered_map<Key, double, KeyHash> best;
    best[make_key(start)] = 0.0;
    int reached = -1;
    const auto planning_started = std::chrono::steady_clock::now();

    while (!open.empty() && result.expanded < options_.max_iterations)
    {
        if ((result.expanded & 0xffU) == 0U)
        {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - planning_started).count();
            if (elapsed >= options_.max_planning_time)
            {
                result.reason = "planning time limit reached";
                return result;
            }
        }
        const QueueItem item = open.top();
        open.pop();
        const Node current = nodes[item.node];
        const auto known = best.find(make_key(current.pose));
        if (known != best.end() && current.g > known->second + 1e-9) continue;
        ++result.expanded;

        if (distance(current.pose, goal) <= options_.goal_xy_tolerance &&
            std::fabs(normalizeAngle(current.pose.yaw - goal.yaw)) <=
                options_.goal_yaw_tolerance)
        {
            reached = item.node;
            break;
        }

        for (double curvature : curvatures)
        {
            if (!transition_free(current.pose, curvature)) continue;
            const HybridPose next_pose = propagate(
                current.pose, curvature, options_.primitive_length);
            if (next_pose.x < min_x || next_pose.x > max_x ||
                next_pose.y < min_y || next_pose.y > max_y) continue;
            const double normalized_steer = std::fabs(curvature) / max_curvature;
            const double normalized_change =
                std::fabs(curvature - current.curvature) / max_curvature;
            const double next_g = current.g + options_.primitive_length *
                (1.0 + options_.steering_cost * normalized_steer +
                 options_.steering_change_cost * normalized_change);
            const Key key = make_key(next_pose);
            const auto previous = best.find(key);
            if (previous != best.end() && previous->second <= next_g) continue;
            best[key] = next_g;
            Node next;
            next.pose = next_pose;
            next.g = next_g;
            next.curvature = curvature;
            next.parent = item.node;
            nodes.push_back(next);
            const int next_index = static_cast<int>(nodes.size() - 1);
            open.push(QueueItem{next_g + options_.heuristic_weight *
                               heuristic(next_pose), next_index});
        }
    }

    if (reached < 0)
    {
        result.reason = result.expanded >= options_.max_iterations
                            ? "iteration limit reached" : "no path";
        return result;
    }

    result.cost = nodes[reached].g;
    for (int index = reached; index >= 0; index = nodes[index].parent)
    {
        result.poses.push_back(nodes[index].pose);
    }
    std::reverse(result.poses.begin(), result.poses.end());
    if (distance(result.poses.back(), goal) > 1e-6)
    {
        result.poses.push_back(goal);
    }
    result.success = true;
    result.reason = "ok";
    return result;
}

std::vector<HybridPose> HybridAStar::guidePoints(
    const std::vector<HybridPose> &path,
    double max_straight_spacing,
    double preserve_turn_angle)
{
    if (path.size() <= 2) return path;
    max_straight_spacing = std::max(0.10, max_straight_spacing);
    preserve_turn_angle = std::max(0.5 * kPi / 180.0, preserve_turn_angle);
    std::vector<HybridPose> guides;
    guides.push_back(path.front());
    std::size_t last_kept = 0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        const double before = std::atan2(path[i].y - path[i - 1].y,
                                         path[i].x - path[i - 1].x);
        const double after = std::atan2(path[i + 1].y - path[i].y,
                                        path[i + 1].x - path[i].x);
        const bool turning = std::fabs(normalizeAngle(after - before)) >=
                             preserve_turn_angle;
        const bool spacing = distance(path[last_kept], path[i]) >=
                             max_straight_spacing;
        if (turning || spacing)
        {
            guides.push_back(path[i]);
            last_kept = i;
        }
    }
    guides.push_back(path.back());
    return guides;
}

}  // namespace x2bot_teleop
