#ifndef LIO_LITE_ALIGNMENT_FITNESS_H
#define LIO_LITE_ALIGNMENT_FITNESS_H

#include <cstddef>
#include <limits>

#include "common_lib.h"

namespace lio_lite {

struct AlignmentFitnessResult {
    double mean_squared_distance = std::numeric_limits<double>::max();
    std::size_t matched_points = 0;
};

/// Copy only points with finite XYZ coordinates and correct the PCL density metadata.
PointCloudType::Ptr MakeFinitePointCloud(const PointCloudType &input);

/// Score an already transformed cloud without invoking pcl::transformPointCloud().
AlignmentFitnessResult ComputeAlignmentFitness(
    const PointCloudType &aligned_source,
    const PointCloudType::ConstPtr &finite_target,
    double max_squared_distance = std::numeric_limits<double>::max());

}  // namespace lio_lite

#endif  // LIO_LITE_ALIGNMENT_FITNESS_H
