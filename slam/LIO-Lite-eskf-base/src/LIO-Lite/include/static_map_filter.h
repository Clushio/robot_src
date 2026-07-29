#ifndef LIO_LITE_STATIC_MAP_FILTER_H
#define LIO_LITE_STATIC_MAP_FILTER_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "common_lib.h"

namespace lio_lite {

class StaticMapFilter {
   public:
    struct Options {
        bool enabled = false;
        double keyframe_interval = 0.5;
        double voxel_size = 0.30;
        double match_radius = 0.30;
        int min_hits = 3;
        double min_observation_span = 1.0;
        int free_keyframes_to_remove = 3;
        double ray_end_margin = 0.40;
        double candidate_timeout = 5.0;
        std::size_t max_global_samples_per_voxel = 80;
        std::size_t max_feature_samples_per_voxel = 40;
        std::size_t min_confirmed_voxels = 100;
        bool publish_debug = true;
        bool save_raw = true;
    };

    struct Stats {
        std::size_t processed_frames = 0;
        std::size_t processed_keyframes = 0;
        std::size_t confirmed_voxels = 0;
        std::size_t candidate_voxels = 0;
        std::size_t removed_voxels = 0;
        std::size_t stored_global_points = 0;
        std::size_t stored_feature_points = 0;
    };

    explicit StaticMapFilter(const Options &options);

    bool enabled() const { return options_.enabled; }

    /// Returns true when this frame was used as a confidence keyframe.
    bool ProcessFrame(const PointCloudType &scan_world, const PointCloudType &feature_world,
                      const Eigen::Vector3d &sensor_origin, double timestamp,
                      PointCloudType *static_debug, PointCloudType *dynamic_debug);

    CloudPtr BuildStaticMap() const;
    CloudPtr BuildStaticFeatureMap() const;
    Stats GetStats() const;
    void Reset();

   private:
    struct VoxelKey {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const VoxelKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey &key) const;
    };

    struct VoxelState {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        std::size_t point_count = 0;
        int hit_keyframes = 0;
        std::size_t last_hit_keyframe = std::numeric_limits<std::size_t>::max();
        std::size_t last_free_keyframe = std::numeric_limits<std::size_t>::max();
        int consecutive_free_keyframes = 0;
        double first_hit_time = 0.0;
        double last_hit_time = 0.0;
        bool confirmed = false;
        PointVector global_samples;
        PointVector feature_samples;
    };

    using StateMap = std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash>;
    using KeySet = std::unordered_set<VoxelKey, VoxelKeyHash>;

    VoxelKey KeyForPoint(double x, double y, double z) const;
    VoxelKey FindOrCreateState(const PointType &point);
    bool FindMatchingState(const PointType &point, VoxelKey *key) const;
    void AddGlobalSample(VoxelState *state, const PointType &point);
    void AddFeatureSample(VoxelState *state, const PointType &point);
    void CollectFreeSpace(const PointCloudType &scan_world, const Eigen::Vector3d &sensor_origin,
                          const KeySet &hit_keys, KeySet *free_keys) const;
    void UpdateStats();
    static void FinalizeCloud(PointCloudType *cloud);

    Options options_;
    StateMap states_;
    double last_keyframe_time_ = -std::numeric_limits<double>::infinity();
    std::size_t keyframe_id_ = 0;
    std::size_t processed_frames_ = 0;
    std::size_t removed_voxels_ = 0;
    Stats stats_;
};

}  // namespace lio_lite

#endif  // LIO_LITE_STATIC_MAP_FILTER_H
