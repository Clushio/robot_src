#include "alignment_fitness.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

PointType Point(float x, float y, float z) {
    PointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

bool Near(double actual, double expected, double tolerance = 1e-6) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
    PointCloudType dirty;
    dirty.push_back(Point(0.0F, 0.0F, 0.0F));
    dirty.push_back(Point(std::numeric_limits<float>::quiet_NaN(), 1.0F, 0.0F));
    dirty.push_back(Point(2.0F, 0.0F, 0.0F));
    dirty.is_dense = true;

    const auto finite = lio_lite::MakeFinitePointCloud(dirty);
    if (finite->size() != 2 || !finite->is_dense || finite->height != 1 || finite->width != 2) {
        std::cerr << "finite point filtering failed" << std::endl;
        return 1;
    }

    PointCloudType aligned;
    aligned.push_back(Point(1.0F, 0.0F, 0.0F));
    aligned.push_back(Point(3.0F, 0.0F, 0.0F));
    aligned.push_back(Point(std::numeric_limits<float>::infinity(), 0.0F, 0.0F));

    const auto score = lio_lite::ComputeAlignmentFitness(aligned, finite);
    if (score.matched_points != 2 || !Near(score.mean_squared_distance, 1.0)) {
        std::cerr << "alignment score failed: matches=" << score.matched_points
                  << " score=" << score.mean_squared_distance << std::endl;
        return 1;
    }

    const auto rejected = lio_lite::ComputeAlignmentFitness(aligned, finite, 0.5);
    if (rejected.matched_points != 0 ||
        rejected.mean_squared_distance != std::numeric_limits<double>::max()) {
        std::cerr << "maximum-distance rejection failed" << std::endl;
        return 1;
    }

    std::cout << "[PASS] Alignment fitness synthetic tests" << std::endl;
    return 0;
}
