#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "static_map_filter.h"

namespace {

using lio_lite::StaticMapFilter;

PointType MakePoint(float x, float y, float z = 0.8F) {
    PointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.0F;
    point.normal_x = 0.0F;
    point.normal_y = 0.0F;
    point.normal_z = 0.0F;
    point.curvature = 0.0F;
    return point;
}

PointCloudType MakeWall(float x, float y_offset = 0.0F) {
    PointCloudType cloud;
    for (int i = -4; i <= 4; ++i) {
        cloud.push_back(MakePoint(x, y_offset + static_cast<float>(i) * 0.25F));
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    return cloud;
}

PointCloudType MakeCluster(float x) {
    PointCloudType cloud;
    for (int i = 0; i < 12; ++i) {
        const float y = static_cast<float>(i % 4) * 0.03F;
        const float z = 0.6F + static_cast<float>(i / 4) * 0.08F;
        cloud.push_back(MakePoint(x + static_cast<float>(i % 2) * 0.02F, y, z));
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    return cloud;
}

StaticMapFilter::Options TestOptions() {
    StaticMapFilter::Options options;
    options.enabled = true;
    options.keyframe_interval = 0.5;
    options.voxel_size = 0.30;
    options.match_radius = 0.30;
    options.min_hits = 3;
    options.min_observation_span = 1.0;
    options.free_keyframes_to_remove = 3;
    options.ray_end_margin = 0.40;
    options.candidate_timeout = 5.0;
    options.max_global_samples_per_voxel = 80;
    options.max_feature_samples_per_voxel = 40;
    options.min_confirmed_voxels = 1;
    options.publish_debug = false;
    return options;
}

bool HasPointNearX(const PointCloudType &cloud, float x, float tolerance) {
    for (const PointType &point : cloud.points) {
        if (std::fabs(point.x - x) <= tolerance) {
            return true;
        }
    }
    return false;
}

bool Expect(bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << std::endl;
        return false;
    }
    return true;
}

bool TestStaticWallAndFeatures() {
    StaticMapFilter filter(TestOptions());
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    PointCloudType debug_static;
    PointCloudType debug_dynamic;
    for (int frame = 0; frame < 3; ++frame) {
        PointCloudType wall = MakeWall(5.0F, static_cast<float>(frame) * 0.005F);
        filter.ProcessFrame(wall, wall, origin, static_cast<double>(frame) * 0.5,
                            &debug_static, &debug_dynamic);
    }

    const auto stats = filter.GetStats();
    CloudPtr static_map = filter.BuildStaticMap();
    CloudPtr feature_map = filter.BuildStaticFeatureMap();
    return Expect(stats.processed_keyframes == 3, "static wall should use three keyframes") &&
           Expect(stats.confirmed_voxels > 0, "static wall should confirm voxels") &&
           Expect(!static_map->empty(), "static wall should enter GlobalMap") &&
           Expect(!feature_map->empty(), "static wall features should enter FeatureMap");
}

bool TestMovingClusterRejected() {
    StaticMapFilter filter(TestOptions());
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (int frame = 0; frame < 4; ++frame) {
        const PointCloudType cluster = MakeCluster(2.0F + static_cast<float>(frame) * 0.65F);
        filter.ProcessFrame(cluster, PointCloudType(), origin,
                            static_cast<double>(frame) * 0.5, nullptr, nullptr);
    }

    return Expect(filter.GetStats().confirmed_voxels == 0,
                  "translating cluster must not become static") &&
           Expect(filter.BuildStaticMap()->empty(),
                  "translating cluster must not enter the static map");
}

bool TestDuplicatePointsCountOnce() {
    StaticMapFilter filter(TestOptions());
    PointCloudType duplicates;
    for (int i = 0; i < 100; ++i) {
        duplicates.push_back(MakePoint(3.0F, 0.0F));
    }
    filter.ProcessFrame(duplicates, PointCloudType(), Eigen::Vector3d::Zero(), 0.0,
                        nullptr, nullptr);

    const auto stats = filter.GetStats();
    return Expect(stats.processed_keyframes == 1, "duplicate frame should be one keyframe") &&
           Expect(stats.candidate_voxels == 1, "duplicate points should share one voxel") &&
           Expect(stats.confirmed_voxels == 0,
                  "duplicates in one frame must not satisfy min_hits");
}

bool TestConfirmedObjectClearedByFreeSpace() {
    StaticMapFilter filter(TestOptions());
    // Keep the synthetic background on the same viewing rays as the object.
    const Eigen::Vector3d origin(0.0, 0.0, 0.8);
    const PointCloudType object = MakeCluster(3.0F);

    for (int frame = 0; frame < 3; ++frame) {
        filter.ProcessFrame(object, object, origin, static_cast<double>(frame) * 0.5,
                            nullptr, nullptr);
    }
    if (!Expect(HasPointNearX(*filter.BuildStaticMap(), 3.0F, 0.15F),
                "stationary object should initially confirm")) {
        return false;
    }

    const PointCloudType background = MakeCluster(5.0F);
    for (int frame = 3; frame < 6; ++frame) {
        filter.ProcessFrame(background, background, origin,
                            static_cast<double>(frame) * 0.5, nullptr, nullptr);
    }

    CloudPtr static_map = filter.BuildStaticMap();
    CloudPtr feature_map = filter.BuildStaticFeatureMap();
    return Expect(!HasPointNearX(*static_map, 3.0F, 0.15F),
                  "three free-space keyframes should remove the old object") &&
           Expect(HasPointNearX(*static_map, 5.0F, 0.15F),
                  "new persistent background should be retained") &&
           Expect(!HasPointNearX(*feature_map, 3.0F, 0.15F),
                  "removed object features must leave FeatureMap") &&
           Expect(HasPointNearX(*feature_map, 5.0F, 0.15F),
                  "confirmed background features must remain in FeatureMap") &&
           Expect(filter.GetStats().removed_voxels > 0,
                  "free-space removal should update statistics");
}

bool TestDisabledFilterDoesNothing() {
    StaticMapFilter::Options options = TestOptions();
    options.enabled = false;
    StaticMapFilter filter(options);
    const PointCloudType wall = MakeWall(5.0F);
    const bool keyframe = filter.ProcessFrame(wall, wall, Eigen::Vector3d::Zero(), 0.0,
                                              nullptr, nullptr);
    return Expect(!keyframe, "disabled filter should not process keyframes") &&
           Expect(filter.GetStats().processed_frames == 0,
                  "disabled filter should not retain state") &&
           Expect(filter.BuildStaticMap()->empty(),
                  "disabled filter should leave map saving to the legacy path");
}

}  // namespace

int main() {
    bool ok = true;
    ok = TestStaticWallAndFeatures() && ok;
    ok = TestMovingClusterRejected() && ok;
    ok = TestDuplicatePointsCountOnce() && ok;
    ok = TestConfirmedObjectClearedByFreeSpace() && ok;
    ok = TestDisabledFilterDoesNothing() && ok;
    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "[PASS] StaticMapFilter synthetic tests" << std::endl;
    return EXIT_SUCCESS;
}
