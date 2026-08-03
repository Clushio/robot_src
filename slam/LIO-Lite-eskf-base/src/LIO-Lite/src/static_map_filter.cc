#include "static_map_filter.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <utility>

#include <pcl/common/point_tests.h>

namespace lio_lite {

namespace {

constexpr std::size_t kInvalidKeyframe = std::numeric_limits<std::size_t>::max();

double SquaredDistance(const PointType &point, double cx, double cy, double cz) {
    const double dx = static_cast<double>(point.x) - cx;
    const double dy = static_cast<double>(point.y) - cy;
    const double dz = static_cast<double>(point.z) - cz;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

std::size_t StaticMapFilter::VoxelKeyHash::operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
}

StaticMapFilter::StaticMapFilter(const Options &options) : options_(options) {
    options_.keyframe_interval = std::max(0.0, options_.keyframe_interval);
    options_.voxel_size = std::max(0.01, options_.voxel_size);
    options_.match_radius = std::max(0.0, options_.match_radius);
    options_.min_hits = std::max(1, options_.min_hits);
    options_.min_observation_span = std::max(0.0, options_.min_observation_span);
    options_.free_keyframes_to_remove = std::max(1, options_.free_keyframes_to_remove);
    options_.ray_end_margin = std::max(0.0, options_.ray_end_margin);
    options_.candidate_timeout = std::max(0.0, options_.candidate_timeout);
    options_.max_global_samples_per_voxel =
        std::max<std::size_t>(1, options_.max_global_samples_per_voxel);
    options_.max_feature_samples_per_voxel =
        std::max<std::size_t>(1, options_.max_feature_samples_per_voxel);
    UpdateStats();
}

StaticMapFilter::VoxelKey StaticMapFilter::KeyForPoint(double x, double y, double z) const {
    return {static_cast<int>(std::floor(x / options_.voxel_size)),
            static_cast<int>(std::floor(y / options_.voxel_size)),
            static_cast<int>(std::floor(z / options_.voxel_size))};
}

bool StaticMapFilter::FindMatchingState(const PointType &point, VoxelKey *key,
                                        bool confirmed_only) const {
    const VoxelKey center = KeyForPoint(point.x, point.y, point.z);
    const int radius_cells =
        std::max(0, static_cast<int>(std::ceil(options_.match_radius / options_.voxel_size)));
    const double max_distance_sq = options_.match_radius * options_.match_radius;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                const VoxelKey candidate{center.x + dx, center.y + dy, center.z + dz};
                const auto iter = states_.find(candidate);
                if (iter == states_.end() || iter->second.point_count == 0 ||
                    (confirmed_only && !iter->second.confirmed)) {
                    continue;
                }
                const VoxelState &state = iter->second;
                const double count = static_cast<double>(state.point_count);
                const double distance_sq =
                    SquaredDistance(point, state.sum_x / count, state.sum_y / count,
                                    state.sum_z / count);
                if (distance_sq <= max_distance_sq && distance_sq < best_distance_sq) {
                    best_distance_sq = distance_sq;
                    *key = candidate;
                    found = true;
                }
            }
        }
    }

    if (!found) {
        const auto exact = states_.find(center);
        if (exact != states_.end() && (!confirmed_only || exact->second.confirmed)) {
            *key = center;
            return true;
        }
    }
    return found;
}

StaticMapFilter::VoxelKey StaticMapFilter::FindOrCreateState(const PointType &point) {
    VoxelKey key;
    if (FindMatchingState(point, &key)) {
        return key;
    }

    key = KeyForPoint(point.x, point.y, point.z);
    states_.emplace(key, VoxelState{});
    return key;
}

void StaticMapFilter::AddGlobalSample(VoxelState *state, const PointType &point) {
    state->sum_x += point.x;
    state->sum_y += point.y;
    state->sum_z += point.z;
    state->point_count++;
    if (state->global_samples.size() < options_.max_global_samples_per_voxel) {
        state->global_samples.push_back(point);
    }
}

void StaticMapFilter::AddFeatureSample(VoxelState *state, const PointType &point) {
    if (state->feature_samples.size() < options_.max_feature_samples_per_voxel) {
        state->feature_samples.push_back(point);
    }
}

void StaticMapFilter::CollectFreeSpace(const PointCloudType &scan_world,
                                       const Eigen::Vector3d &sensor_origin,
                                       const KeySet &hit_keys, KeySet *free_keys) const {
    for (const PointType &point : scan_world.points) {
        const Eigen::Vector3d endpoint(point.x, point.y, point.z);
        const Eigen::Vector3d ray = endpoint - sensor_origin;
        const double range = ray.norm();
        const double stop_range = range - options_.ray_end_margin;
        if (!std::isfinite(range) || stop_range <= 0.0) {
            continue;
        }

        const Eigen::Vector3d direction = ray / range;
        VoxelKey key = KeyForPoint(sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
        const auto initialize_axis = [this](double origin, double axis_direction,
                                            int cell, int *cell_step,
                                            double *next_crossing,
                                            double *crossing_step) {
            if (axis_direction > 0.0) {
                *cell_step = 1;
                const double boundary = static_cast<double>(cell + 1) * options_.voxel_size;
                *next_crossing = (boundary - origin) / axis_direction;
                *crossing_step = options_.voxel_size / axis_direction;
            } else if (axis_direction < 0.0) {
                *cell_step = -1;
                const double boundary = static_cast<double>(cell) * options_.voxel_size;
                *next_crossing = (boundary - origin) / axis_direction;
                *crossing_step = -options_.voxel_size / axis_direction;
            } else {
                *cell_step = 0;
                *next_crossing = std::numeric_limits<double>::infinity();
                *crossing_step = std::numeric_limits<double>::infinity();
            }
        };

        int step_x = 0;
        int step_y = 0;
        int step_z = 0;
        double next_x = 0.0;
        double next_y = 0.0;
        double next_z = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_z = 0.0;
        initialize_axis(sensor_origin.x(), direction.x(), key.x, &step_x, &next_x, &delta_x);
        initialize_axis(sensor_origin.y(), direction.y(), key.y, &step_y, &next_y, &delta_y);
        initialize_axis(sensor_origin.z(), direction.z(), key.z, &step_z, &next_z, &delta_z);

        while (true) {
            const double next = std::min(next_x, std::min(next_y, next_z));
            if (!std::isfinite(next) || next >= stop_range) {
                break;
            }
            constexpr double kTieTolerance = 1e-10;
            if (next_x <= next + kTieTolerance) {
                key.x += step_x;
                next_x += delta_x;
            }
            if (next_y <= next + kTieTolerance) {
                key.y += step_y;
                next_y += delta_y;
            }
            if (next_z <= next + kTieTolerance) {
                key.z += step_z;
                next_z += delta_z;
            }
            if (hit_keys.find(key) == hit_keys.end() && states_.find(key) != states_.end()) {
                free_keys->insert(key);
            }
        }
    }
}

bool StaticMapFilter::ProcessFrame(const PointCloudType &scan_world,
                                   const PointCloudType &feature_world,
                                   const Eigen::Vector3d &sensor_origin, double timestamp,
                                   PointCloudType *static_debug,
                                   PointCloudType *dynamic_debug) {
    if (static_debug != nullptr) {
        static_debug->clear();
    }
    if (dynamic_debug != nullptr) {
        dynamic_debug->clear();
    }
    if (!options_.enabled || scan_world.empty() || !std::isfinite(timestamp)) {
        return false;
    }

    processed_frames_++;
    const bool is_keyframe =
        !std::isfinite(last_keyframe_time_) ||
        timestamp - last_keyframe_time_ + 1e-9 >= options_.keyframe_interval;
    if (!is_keyframe) {
        UpdateStats();
        return false;
    }
    const std::size_t current_keyframe = keyframe_id_;

    std::vector<VoxelKey> scan_keys;
    scan_keys.reserve(scan_world.size());
    KeySet hit_keys;

    for (const PointType &point : scan_world.points) {
        if (!pcl::isFinite(point)) {
            scan_keys.push_back(VoxelKey{});
            continue;
        }
        const VoxelKey key = FindOrCreateState(point);
        scan_keys.push_back(key);
        VoxelState &state = states_.at(key);
        AddGlobalSample(&state, point);

        if (is_keyframe && state.last_hit_keyframe != current_keyframe) {
            if (state.hit_keyframes == 0) {
                state.first_hit_time = timestamp;
            }
            state.hit_keyframes++;
            state.last_hit_keyframe = current_keyframe;
            state.last_hit_time = timestamp;
            state.consecutive_free_keyframes = 0;
            hit_keys.insert(key);
        }
    }

    for (const PointType &point : feature_world.points) {
        if (!pcl::isFinite(point)) {
            continue;
        }
        VoxelKey key;
        if (FindMatchingState(point, &key)) {
            AddFeatureSample(&states_.at(key), point);
        }
    }

    for (const VoxelKey &key : hit_keys) {
        VoxelState &state = states_.at(key);
        if (!state.confirmed && state.hit_keyframes >= options_.min_hits &&
            timestamp - state.first_hit_time + 1e-9 >= options_.min_observation_span) {
            state.confirmed = true;
        }
    }

    KeySet free_keys;
    CollectFreeSpace(scan_world, sensor_origin, hit_keys, &free_keys);

    std::vector<VoxelKey> erase_keys;
    erase_keys.reserve(free_keys.size());
    for (const VoxelKey &key : free_keys) {
        auto iter = states_.find(key);
        if (iter == states_.end()) {
            continue;
        }
        VoxelState &state = iter->second;
        if (dynamic_debug != nullptr) {
            dynamic_debug->points.insert(dynamic_debug->points.end(),
                                         state.global_samples.begin(),
                                         state.global_samples.end());
        }
        if (!state.confirmed) {
            erase_keys.push_back(key);
            continue;
        }

        if (state.last_free_keyframe != kInvalidKeyframe &&
            state.last_free_keyframe + 1 == current_keyframe) {
            state.consecutive_free_keyframes++;
        } else {
            state.consecutive_free_keyframes = 1;
        }
        state.last_free_keyframe = current_keyframe;
        if (state.consecutive_free_keyframes >= options_.free_keyframes_to_remove) {
            erase_keys.push_back(key);
        }
    }

    for (const auto &entry : states_) {
        const VoxelState &state = entry.second;
        if (!state.confirmed && state.hit_keyframes > 0 &&
            timestamp - state.last_hit_time > options_.candidate_timeout) {
            erase_keys.push_back(entry.first);
        }
    }

    std::sort(erase_keys.begin(), erase_keys.end(), [](const VoxelKey &lhs, const VoxelKey &rhs) {
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        return lhs.z < rhs.z;
    });
    erase_keys.erase(std::unique(erase_keys.begin(), erase_keys.end()), erase_keys.end());
    for (const VoxelKey &key : erase_keys) {
        removed_voxels_ += states_.erase(key);
    }

    if (static_debug != nullptr || dynamic_debug != nullptr) {
        for (std::size_t i = 0; i < scan_world.size(); ++i) {
            const PointType &point = scan_world.points[i];
            if (!pcl::isFinite(point)) {
                continue;
            }
            const auto iter = states_.find(scan_keys[i]);
            if (iter != states_.end() && iter->second.confirmed) {
                if (static_debug != nullptr) {
                    static_debug->push_back(point);
                }
            } else if (dynamic_debug != nullptr) {
                dynamic_debug->push_back(point);
            }
        }
        if (static_debug != nullptr) {
            FinalizeCloud(static_debug);
        }
        if (dynamic_debug != nullptr) {
            FinalizeCloud(dynamic_debug);
        }
    }

    last_keyframe_time_ = timestamp;
    keyframe_id_++;
    UpdateStats();
    return true;
}

CloudPtr StaticMapFilter::BuildStaticMap() const {
    CloudPtr cloud(new PointCloudType());
    for (const auto &entry : states_) {
        if (entry.second.confirmed) {
            cloud->points.insert(cloud->points.end(), entry.second.global_samples.begin(),
                                 entry.second.global_samples.end());
        }
    }
    FinalizeCloud(cloud.get());
    return cloud;
}

CloudPtr StaticMapFilter::BuildStaticFeatureMap() const {
    CloudPtr cloud(new PointCloudType());
    for (const auto &entry : states_) {
        if (entry.second.confirmed) {
            cloud->points.insert(cloud->points.end(), entry.second.feature_samples.begin(),
                                 entry.second.feature_samples.end());
        }
    }
    FinalizeCloud(cloud.get());
    return cloud;
}

CloudPtr StaticMapFilter::BuildStaticCloudFromRaw(const PointCloudType &raw_cloud,
                                                  std::size_t max_samples_per_state,
                                                  std::uint32_t random_seed) const {
    struct Reservoir {
        std::size_t seen = 0;
        PointVector samples;
    };

    std::unordered_map<VoxelKey, Reservoir, VoxelKeyHash> reservoirs;
    reservoirs.reserve(states_.size());
    std::mt19937 random_generator(random_seed);

    for (const PointType &point : raw_cloud.points) {
        if (!pcl::isFinite(point)) {
            continue;
        }

        VoxelKey key;
        if (!FindMatchingState(point, &key, true)) {
            continue;
        }

        Reservoir &reservoir = reservoirs[key];
        reservoir.seen++;
        if (reservoir.samples.size() < max_samples_per_state) {
            reservoir.samples.push_back(point);
            continue;
        }

        std::uniform_int_distribution<std::size_t> replacement_distribution(
            0, reservoir.seen - 1);
        const std::size_t replacement_index = replacement_distribution(random_generator);
        if (replacement_index < max_samples_per_state) {
            reservoir.samples[replacement_index] = point;
        }
    }

    std::vector<VoxelKey> ordered_keys;
    ordered_keys.reserve(reservoirs.size());
    for (const auto &entry : reservoirs) {
        ordered_keys.push_back(entry.first);
    }
    std::sort(ordered_keys.begin(), ordered_keys.end(),
              [](const VoxelKey &lhs, const VoxelKey &rhs) {
                  if (lhs.x != rhs.x) return lhs.x < rhs.x;
                  if (lhs.y != rhs.y) return lhs.y < rhs.y;
                  return lhs.z < rhs.z;
              });

    CloudPtr cloud(new PointCloudType());
    for (const VoxelKey &key : ordered_keys) {
        const Reservoir &reservoir = reservoirs.at(key);
        cloud->points.insert(cloud->points.end(), reservoir.samples.begin(),
                             reservoir.samples.end());
    }
    FinalizeCloud(cloud.get());
    return cloud;
}

CloudPtr StaticMapFilter::BuildStaticMapFromRaw(const PointCloudType &raw_cloud) const {
    constexpr std::uint32_t kGlobalSamplingSeed = 0x474c4f42U;
    return BuildStaticCloudFromRaw(raw_cloud, options_.max_global_samples_per_voxel,
                                   kGlobalSamplingSeed);
}

CloudPtr StaticMapFilter::BuildStaticFeatureMapFromRaw(const PointCloudType &raw_cloud) const {
    constexpr std::uint32_t kFeatureSamplingSeed = 0x46454154U;
    return BuildStaticCloudFromRaw(raw_cloud, options_.max_feature_samples_per_voxel,
                                   kFeatureSamplingSeed);
}

StaticMapFilter::Stats StaticMapFilter::GetStats() const {
    return stats_;
}

void StaticMapFilter::Reset() {
    states_.clear();
    last_keyframe_time_ = -std::numeric_limits<double>::infinity();
    keyframe_id_ = 0;
    processed_frames_ = 0;
    removed_voxels_ = 0;
    UpdateStats();
}

void StaticMapFilter::UpdateStats() {
    stats_ = Stats{};
    stats_.processed_frames = processed_frames_;
    stats_.processed_keyframes = keyframe_id_;
    stats_.removed_voxels = removed_voxels_;
    for (const auto &entry : states_) {
        const VoxelState &state = entry.second;
        if (state.confirmed) {
            stats_.confirmed_voxels++;
        } else {
            stats_.candidate_voxels++;
        }
        stats_.stored_global_points += state.global_samples.size();
        stats_.stored_feature_points += state.feature_samples.size();
    }
}

void StaticMapFilter::FinalizeCloud(PointCloudType *cloud) {
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
}

}  // namespace lio_lite
