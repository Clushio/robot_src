#include "alignment_fitness.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

namespace lio_lite {

PointCloudType::Ptr MakeFinitePointCloud(const PointCloudType &input) {
    auto output = std::make_shared<PointCloudType>();
    output->header = input.header;
    output->sensor_origin_ = input.sensor_origin_;
    output->sensor_orientation_ = input.sensor_orientation_;
    output->reserve(input.size());

    for (const auto &point : input.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            output->push_back(point);
        }
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

AlignmentFitnessResult ComputeAlignmentFitness(
    const PointCloudType &aligned_source,
    const PointCloudType::ConstPtr &finite_target,
    double max_squared_distance) {
    AlignmentFitnessResult result;
    if (!finite_target || finite_target->empty() || max_squared_distance < 0.0) {
        return result;
    }

    pcl::KdTreeFLANN<PointType> target_tree;
    target_tree.setInputCloud(finite_target);

    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    double squared_distance_sum = 0.0;

    for (const auto &point : aligned_source.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (target_tree.nearestKSearch(point, 1, nearest_index, nearest_squared_distance) != 1) {
            continue;
        }
        const double distance = nearest_squared_distance.front();
        if (!std::isfinite(distance) || distance > max_squared_distance) {
            continue;
        }
        squared_distance_sum += distance;
        ++result.matched_points;
    }

    if (result.matched_points > 0) {
        result.mean_squared_distance = squared_distance_sum / result.matched_points;
    }
    return result;
}

}  // namespace lio_lite
