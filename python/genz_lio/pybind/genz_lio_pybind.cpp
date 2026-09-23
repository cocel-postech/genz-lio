// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Python bindings for the core pipeline. Scans arrive as arrays rather than one
// point at a time, so the boundary is crossed once per scan.
#include "core/GenZLIO.hpp"
#include "core/ScanBuffer.hpp"

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace py = pybind11;
using namespace genz_lio;

namespace {

using ArrayXd = py::array_t<double, py::array::c_style | py::array::forcecast>;
using ArrayXf = py::array_t<float, py::array::c_style | py::array::forcecast>;

// Select original display samples, never centroids or estimator-map mutations.
// Called only for changed render tiles. Input/output arrays have independent
// ownership, so the estimator worker can prepare its next frame concurrently.
py::array_t<double> downsampleDisplayMap(const ArrayXd &points, double spacing) {
    if (points.ndim() != 2 || points.shape(1) != 3)
        throw std::invalid_argument("display map points must have shape (N, 3)");
    if (!std::isfinite(spacing) || spacing < 0.)
        throw std::invalid_argument("display map spacing must be finite and nonnegative");
    const auto xyz = points.unchecked<2>();
    std::vector<py::ssize_t> selected;
    selected.reserve(xyz.shape(0));
    {
        py::gil_scoped_release release;
        tsl::robin_map<VoxelKey, bool, VoxelHash> occupied;
        if (spacing > 0.) occupied.reserve(xyz.shape(0));
        for (py::ssize_t i = 0; i < xyz.shape(0); ++i) {
            if (spacing == 0.) { selected.push_back(i); continue; }
            const Vec3 cell = Vec3(xyz(i, 0), xyz(i, 1), xyz(i, 2)) / spacing;
            // Keep the integer conversion defined even for malformed input or
            // impractically small display cells.
            if (!cell.allFinite() || (cell.array().abs() >= 9.0e18).any())
                throw std::invalid_argument("display map cell coordinates are out of range");
            if (occupied.emplace(VoxelKey(cell), true).second) selected.push_back(i);
        }
    }
    py::array_t<double> result({static_cast<py::ssize_t>(selected.size()), py::ssize_t(3)});
    auto out = result.mutable_unchecked<2>();
    {
        py::gil_scoped_release release;
        for (std::size_t i = 0; i < selected.size(); ++i)
            for (int j = 0; j < 3; ++j) out(i, j) = xyz(selected[i], j);
    }
    return result;
}

// A display-only cache: root samples stay compact in CPU memory, and only
// changed spatial tiles cross the Python/GPU boundary. Optional display sampling
// is cached per changed root, never repeated over whole render tiles. The
// authoritative estimator map remains untouched.
class VisualizationMap {
    using Key = std::array<std::int64_t, 3>;
    struct Samples { std::vector<Vec3> points; std::size_t original_count = 0; };
    using Roots = std::map<Key, Samples>;
    GenZLIO &engine_;
    std::map<Key, Roots> tiles_;
    std::int64_t roots_per_tile_;
    std::set<Key> split_parents_;
    double spacing_ = 0.;
    std::size_t retained_points_ = 0;
    tsl::robin_map<VoxelKey, bool, VoxelHash> occupied_;

    Key tileKey(const Key &root) const {
        Key tile;
        for (int axis = 0; axis < 3; ++axis) {
            tile[axis] = root[axis] / roots_per_tile_;
            if (root[axis] % roots_per_tile_ < 0) --tile[axis];
        }
        return tile;
    }

    static void collect(const OctoTree *cell, std::vector<Vec3> &points) {
        bool children = false;
        for (const auto *child : cell->leaves_) {
            if (child) { children = true; collect(child, points); }
        }
        if (children) return;
        const auto &samples = cell->grid_points_.empty() ? cell->temp_points_ : cell->grid_points_;
        for (const auto &sample : samples) points.push_back(sample.point);
        if (samples.empty() && cell->plane_ptr_->is_plane) points.push_back(cell->plane_ptr_->center);
    }

    template <typename Matches> static py::array_t<double> matchArray(const Matches &matches) {
        py::array_t<double> result({static_cast<py::ssize_t>(matches.size()), py::ssize_t(3)});
        auto values = result.mutable_unchecked<2>();
        for (std::size_t i = 0; i < matches.size(); ++i)
            for (int j = 0; j < 3; ++j) values(i, j) = matches[i].point_world[j];
        return result;
    }

public:
    explicit VisualizationMap(GenZLIO &engine) : engine_(engine) {
        // Smaller display tiles avoid retransmitting millions of unchanged
        // samples when only a small part of the retained map changes.
        roots_per_tile_ = std::max<std::int64_t>(1, std::llround(16.0 / engine.config().mapping.voxel_size));
        engine_.startMapTracking();
    }
    ~VisualizationMap() { engine_.stopMapTracking(); }

    py::dict update(double spacing = 0.) {
        if (!std::isfinite(spacing) || spacing < 0.)
            throw std::invalid_argument("display map spacing must be finite and nonnegative");
        // The pipeline worker exclusively owns the engine and this cache.
        // Only Python object construction needs the GIL; map traversal and
        // owned-buffer fills must not block the GUI's Python thread.
        bool reset = false;
        std::set<Key> dirty;
        struct Chunk { Key key; std::size_t count; std::vector<const Roots *> roots; };
        std::vector<Chunk> changed;
        std::vector<Key> erased;
        {
            py::gil_scoped_release release;
            auto changes = engine_.takeMapChanges();
            if (spacing != spacing_) {
                spacing_ = spacing;
                changes.reset = true;
                changes.updated.clear();
                changes.removed.clear();
                for (const auto &entry : engine_.map()) changes.updated.push_back(entry.first);
            }
            if (changes.reset) { tiles_.clear(); split_parents_.clear(); retained_points_ = 0; }
            for (const auto &voxel : changes.removed) {
                const Key root{voxel.x, voxel.y, voxel.z}, tile = tileKey(root);
                auto found = tiles_.find(tile);
                if (found != tiles_.end()) {
                    const auto previous = found->second.find(root);
                    if (previous != found->second.end()) {
                        retained_points_ -= previous->second.original_count;
                        found->second.erase(previous);
                        dirty.insert(tile);
                    }
                }
            }
            std::vector<Vec3> samples;
            for (const auto &voxel : changes.updated) {
                const auto found = engine_.map().find(voxel);
                if (found == engine_.map().end()) continue;
                const Key root{voxel.x, voxel.y, voxel.z}, tile = tileKey(root);
                samples.clear();
                collect(found->second, samples);
                const auto original_count = samples.size();
                if (spacing_ > 0.) {
                    occupied_.clear();
                    std::size_t kept = 0;
                    for (std::size_t i = 0; i < samples.size(); ++i) {
                        const Vec3 cell = samples[i] / spacing_;
                        if (!cell.allFinite() || (cell.array().abs() >= 9.0e18).any())
                            throw std::invalid_argument("display map cell coordinates are out of range");
                        if (occupied_.emplace(VoxelKey(cell), true).second) samples[kept++] = samples[i];
                    }
                    samples.resize(kept);
                }
                auto &cached = tiles_[tile][root];
                retained_points_ -= cached.original_count;
                retained_points_ += original_count;
                cached.original_count = original_count;
                auto &previous = cached.points;
                const bool same = samples.size() == previous.size() &&
                    std::equal(samples.begin(), samples.end(), previous.begin(),
                               [](const Vec3 &a, const Vec3 &b) { return (a.array() == b.array()).all(); });
                if (!same) {
                    previous.swap(samples);
                    dirty.insert(tile);
                }
            }
            reset = changes.reset;
            // Keep sparse areas in 32 m batches to limit GPU draw calls. Split
            // only dense batches into their 16 m children, so local edits do
            // not retransmit a large unchanged cloud. Splits are monotonic
            // until a parent becomes empty or the map resets (no oscillation).
            std::set<Key> parents;
            for (const auto &key : dirty) {
                Key parent;
                for (int axis = 0; axis < 3; ++axis) {
                    parent[axis] = key[axis] / 2;
                    if (key[axis] % 2 < 0) --parent[axis];
                }
                parents.insert(parent);
                auto found = tiles_.find(key);
                bool empty = true;
                for (const auto &root : found->second)
                    if (!root.second.points.empty()) { empty = false; break; }
                if (empty) tiles_.erase(found);
            }
            for (const auto &parent : parents) {
                const Key anchor{parent[0]*2, parent[1]*2, parent[2]*2};
                std::vector<Chunk> children;
                std::size_t total = 0;
                for (int x = 0; x < 2; ++x)
                    for (int y = 0; y < 2; ++y)
                        for (int z = 0; z < 2; ++z) {
                            Key key{anchor[0]+x, anchor[1]+y, anchor[2]+z};
                            const auto found = tiles_.find(key);
                            if (found == tiles_.end()) continue;
                            std::size_t count = 0;
                            for (const auto &root : found->second) count += root.second.points.size();
                            total += count;
                            children.push_back({key, count, {&found->second}});
                        }
                const bool was_split = split_parents_.count(parent) != 0;
                const bool split = was_split || total > 32768;
                if (split) {
                    if (!was_split) erased.push_back(anchor);
                    for (int x = 0; x < 2; ++x)
                        for (int y = 0; y < 2; ++y)
                            for (int z = 0; z < 2; ++z) {
                                Key key{anchor[0]+x, anchor[1]+y, anchor[2]+z};
                                if (was_split && dirty.count(key) && !tiles_.count(key))
                                    erased.push_back(key);
                            }
                    for (auto &child : children)
                        if (!was_split || dirty.count(child.key)) changed.push_back(std::move(child));
                    if (total) split_parents_.insert(parent);
                    else split_parents_.erase(parent);
                } else if (total) {
                    Chunk chunk{anchor, total, {}};
                    for (const auto &child : children) chunk.roots.push_back(child.roots.front());
                    changed.push_back(std::move(chunk));
                } else erased.push_back(anchor);
            }
        }

        py::dict updates;
        py::list removed;
        for (const auto &key : erased)
            removed.append(py::make_tuple(key[0], key[1], key[2]));
        // Allocate Python-owned buffers together, then fill the whole batch
        // without the GIL. Reacquiring it for each small tile causes repeated
        // contention with the GUI when many tiles change in the same scan.
        std::vector<py::array_t<double>> buffers;
        std::vector<double *> destinations;
        buffers.reserve(changed.size());
        destinations.reserve(changed.size());
        for (const auto &entry : changed) {
            buffers.emplace_back(std::vector<py::ssize_t>{
                static_cast<py::ssize_t>(entry.count), py::ssize_t(3)});
            destinations.push_back(buffers.back().mutable_data());
        }
        {
            py::gil_scoped_release release;
            for (std::size_t index = 0; index < changed.size(); ++index) {
                double *out = destinations[index];
                for (const auto *roots : changed[index].roots) for (const auto &root : *roots) {
                    for (const auto &point : root.second.points) {
                        for (int j = 0; j < 3; ++j) *out++ = point[j];
                    }
                }
            }
        }
        for (std::size_t index = 0; index < changed.size(); ++index) {
            const auto &key = changed[index].key;
            updates[py::make_tuple(key[0], key[1], key[2])] = std::move(buffers[index]);
        }
        py::dict result;
        result["reset"] = reset;
        result["sampled_in_core"] = true;
        result["map_spacing"] = spacing_;
        result["retained_map_points"] = retained_points_;
        result["tiles"] = std::move(updates);
        result["removed"] = std::move(removed);
        result["planar"] = matchArray(engine_.planeMatches());
        result["non_planar"] = matchArray(engine_.pointMatches());
        return result;
    }
};

/// Builds a RawScan from an (N, 3) array of positions and optional per-point
/// timing, intensity and ring arrays.
RawScan makeRawScan(const ArrayXd &positions, const py::object &timestamps,
                    const py::object &intensities, const py::object &rings) {
    if (positions.ndim() != 2) throw std::invalid_argument("points must have shape (N, 3)");
    const auto xyz = positions.unchecked<2>();
    if (xyz.shape(1) != 3) {
        throw std::invalid_argument("points must have shape (N, 3)");
    }
    const auto count = static_cast<std::size_t>(xyz.shape(0));

    RawScan scan;
    scan.points.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        scan.points[i].position = Vec3(xyz(i, 0), xyz(i, 1), xyz(i, 2));
    }

    if (!timestamps.is_none()) {
        const auto times = timestamps.cast<ArrayXd>();
        if (times.ndim() != 1) throw std::invalid_argument("timestamps must have one entry per point");
        const auto t = times.unchecked<1>();
        if (static_cast<std::size_t>(t.shape(0)) != count) {
            throw std::invalid_argument("timestamps must have one entry per point");
        }
        double min_time = std::numeric_limits<double>::infinity(), max_time = -min_time;
        for (std::size_t i = 0; i < count; ++i) {
            scan.points[i].time_offset = static_cast<float>(t(i));
            if (std::isfinite(t(i))) {
                min_time = std::min(min_time, t(i));
                max_time = std::max(max_time, t(i));
            }
        }
        scan.time_min = std::isfinite(min_time) ? min_time : 0.0;
        scan.time_max = std::isfinite(max_time) ? max_time : 0.0;
    } else {
        // Without timing the scan cannot be deskewed, and the preprocessor will
        // reconstruct offsets from azimuth if ring indices came with it.
        scan.has_time_offsets = false;
    }

    if (!intensities.is_none()) {
        // Keep the owning array alive: forcecast may allocate a temporary.
        const auto array = intensities.cast<ArrayXf>();
        if (array.ndim() != 1 || static_cast<std::size_t>(array.shape(0)) != count)
            throw std::invalid_argument("intensities must have one entry per point");
        const auto values = array.unchecked<1>();
        for (std::size_t i = 0; i < count; ++i) scan.points[i].intensity = values(i);
    }
    if (!rings.is_none()) {
        const auto array = rings.cast<ArrayXd>();
        if (array.ndim() != 1 || static_cast<std::size_t>(array.shape(0)) != count)
            throw std::invalid_argument("rings must have one entry per point");
        const auto values = array.unchecked<1>();
        scan.rings.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            const double ring = values(i);
            if (!std::isfinite(ring) || ring < 0 || ring >= 65535 || ring != std::floor(ring))
                throw std::invalid_argument("rings must contain finite integer indices in [0, 65535)");
            scan.rings[i] = static_cast<std::uint16_t>(ring);
        }
    }
    // Preserve the caller's time origin while dropping invalid returns together
    // with their ring indices. The synchronizing buffer recomputes its own bounds.
    const bool has_rings = !scan.rings.empty();
    std::size_t kept = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (!scan.points[i].position.allFinite() || !std::isfinite(scan.points[i].time_offset)) continue;
        scan.points[kept] = scan.points[i];
        if (has_rings) scan.rings[kept] = scan.rings[i];
        ++kept;
    }
    scan.points.resize(kept);
    if (has_rings) scan.rings.resize(kept);
    return scan;
}

/// IMU samples as an (N, 7) array: timestamp, acceleration, angular velocity.
std::vector<ImuSample> makeImu(const ArrayXd &array) {
    if (array.ndim() != 2) throw std::invalid_argument("imu must have shape (N, 7)");
    const auto values = array.unchecked<2>();
    if (values.shape(1) != 7) {
        throw std::invalid_argument(
            "imu must have shape (N, 7): timestamp, ax, ay, az, gx, gy, gz");
    }
    std::vector<ImuSample> samples(static_cast<std::size_t>(values.shape(0)));
    for (std::size_t i = 0; i < samples.size(); ++i) {
        for (int j = 0; j < 7; ++j)
            if (!std::isfinite(values(i, j))) throw std::invalid_argument("imu must contain finite values");
        if (i > 0 && values(i, 0) < values(i - 1, 0))
            throw std::invalid_argument("imu timestamps must be nondecreasing");
        samples[i].timestamp = values(i, 0);
        samples[i].linear_acceleration = Vec3(values(i, 1), values(i, 2), values(i, 3));
        samples[i].angular_velocity = Vec3(values(i, 4), values(i, 5), values(i, 6));
    }
    return samples;
}

py::array_t<double> toArray(const PointCloud &cloud) {
    py::array_t<double> out({static_cast<py::ssize_t>(cloud.size()), py::ssize_t(3)});
    auto view = out.mutable_unchecked<2>();
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        view(i, 0) = cloud[i].position.x();
        view(i, 1) = cloud[i].position.y();
        view(i, 2) = cloud[i].position.z();
    }
    return out;
}

/// The pose as a 4x4 homogeneous transform, which is what downstream tooling
/// almost always wants.
Eigen::Matrix4d toMatrix(const Pose &pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = pose.rotation;
    matrix.block<3, 1>(0, 3) = pose.translation;
    return matrix;
}

}  // namespace

PYBIND11_MODULE(genz_lio_pybind, m) {
    m.doc() = "GenZ-LIO: generalizable LiDAR-inertial odometry";

    py::enum_<LidarType>(m, "LidarType")
        .value("LIVOX", LidarType::Livox)
        .value("VELODYNE", LidarType::Velodyne)
        .value("OUSTER", LidarType::Ouster)
        .value("LIVOX_PCL", LidarType::LivoxPcl)
        .value("HESAI", LidarType::Hesai)
        .value("ROBOSENSE", LidarType::Robosense);


    py::class_<PreprocessConfig>(m, "PreprocessConfig")
        .def(py::init<>())
        .def_readwrite("lidar_type", &PreprocessConfig::lidar_type)
        .def_readwrite("scan_line", &PreprocessConfig::scan_line)
        .def_readwrite("scan_rate", &PreprocessConfig::scan_rate)
        .def_readwrite("blind_min", &PreprocessConfig::blind_min)
        .def_readwrite("blind_max", &PreprocessConfig::blind_max)
        .def_readwrite("point_filter_num", &PreprocessConfig::point_filter_num);

    py::class_<MappingConfig>(m, "MappingConfig")
        .def(py::init<>())
        .def_readwrite("max_iteration", &MappingConfig::max_iteration)
        .def_readwrite("down_sample_size", &MappingConfig::down_sample_size)
        .def_readwrite("voxel_size", &MappingConfig::voxel_size)
        .def_readwrite("max_layer", &MappingConfig::max_layer)
        .def_readwrite("layer_point_size", &MappingConfig::layer_point_size)
        .def_readwrite("planar_threshold", &MappingConfig::planar_threshold)
        .def_readwrite("sigma_num", &MappingConfig::sigma_num)
        .def_readwrite("max_points_size", &MappingConfig::max_points_size)
        .def_readwrite("max_mature_points_size", &MappingConfig::max_mature_points_size)
        .def_readwrite("map_range", &MappingConfig::map_range)
        .def_readwrite("extrinsic_est_en", &MappingConfig::extrinsic_est_en)
        .def_readwrite("extrinsic_t", &MappingConfig::extrinsic_t)
        .def_readwrite("extrinsic_r", &MappingConfig::extrinsic_r);

    py::class_<AdaptiveVoxelizationConfig>(m, "AdaptiveVoxelizationConfig")
        .def(py::init<>())
        .def_readwrite("enable", &AdaptiveVoxelizationConfig::enable)
        .def_readwrite("window_size", &AdaptiveVoxelizationConfig::window_size)
        .def_readwrite("scale_threshold", &AdaptiveVoxelizationConfig::scale_threshold)
        .def_readwrite("setpoint_exponent", &AdaptiveVoxelizationConfig::setpoint_exponent)
        .def_readwrite("min_points", &AdaptiveVoxelizationConfig::min_points)
        .def_readwrite("max_points", &AdaptiveVoxelizationConfig::max_points)
        .def_readwrite("error_sensitivity", &AdaptiveVoxelizationConfig::error_sensitivity)
        .def_readwrite("error_rate_sensitivity",
                       &AdaptiveVoxelizationConfig::error_rate_sensitivity)
        .def_readwrite("p_gain_min", &AdaptiveVoxelizationConfig::p_gain_min)
        .def_readwrite("p_gain_max", &AdaptiveVoxelizationConfig::p_gain_max)
        .def_readwrite("d_gain_min", &AdaptiveVoxelizationConfig::d_gain_min)
        .def_readwrite("d_gain_max", &AdaptiveVoxelizationConfig::d_gain_max);

    py::class_<AdaptiveThresholdConfig>(m, "AdaptiveThresholdConfig")
        .def(py::init<>())
        .def_readwrite("initial_threshold", &AdaptiveThresholdConfig::initial_threshold)
        .def_readwrite("max_range_motion", &AdaptiveThresholdConfig::max_range_motion);

    py::class_<HybridMetricConfig>(m, "HybridMetricConfig")
        .def(py::init<>())
        .def_readwrite("enable", &HybridMetricConfig::enable)
        .def_readwrite("lambda_po", &HybridMetricConfig::lambda_po)
        .def_readwrite("sigma_num", &HybridMetricConfig::sigma_num)
        .def_readwrite("adaptive_threshold", &HybridMetricConfig::adaptive_threshold)
        .def_readwrite("max_points_per_voxel", &HybridMetricConfig::max_points_per_voxel)
        .def_readwrite("reduction_ratio", &HybridMetricConfig::reduction_ratio);

    py::class_<NoiseModelConfig>(m, "NoiseModelConfig")
        .def(py::init<>())
        .def_readwrite("ranging_cov", &NoiseModelConfig::ranging_cov)
        .def_readwrite("angle_cov", &NoiseModelConfig::angle_cov)
        .def_readwrite("acc_cov", &NoiseModelConfig::acc_cov)
        .def_readwrite("gyr_cov", &NoiseModelConfig::gyr_cov)
        .def_readwrite("b_acc_cov", &NoiseModelConfig::b_acc_cov)
        .def_readwrite("b_gyr_cov", &NoiseModelConfig::b_gyr_cov);

    py::class_<Config>(m, "Config")
        .def(py::init<>())
        .def_readwrite("max_threads", &Config::max_threads)
        .def_readwrite("preprocess", &Config::preprocess)
        .def_readwrite("mapping", &Config::mapping)
        .def_readwrite("adaptive_voxelization", &Config::adaptive_voxelization)
        .def_readwrite("hybrid_metric", &Config::hybrid_metric)
        .def_readwrite("noise_model", &Config::noise_model);

    py::class_<GenZLIO::Result::Timing>(m, "Timing")
        .def_readonly("preprocess", &GenZLIO::Result::Timing::preprocess)
        .def_readonly("deskew", &GenZLIO::Result::Timing::deskew)
        .def_readonly("voxelize", &GenZLIO::Result::Timing::voxelize)
        .def_readonly("covariance", &GenZLIO::Result::Timing::covariance)
        .def_readonly("update", &GenZLIO::Result::Timing::update)
        .def_readonly("map", &GenZLIO::Result::Timing::map);

    py::class_<GenZLIO::Result>(m, "Result")
        .def_readonly("valid", &GenZLIO::Result::valid)
        .def_readonly("update_rejected", &GenZLIO::Result::update_rejected)
        .def_readonly("median_range", &GenZLIO::Result::median_range)
        .def_readonly("timestamp", &GenZLIO::Result::timestamp)
        .def_property_readonly("pose", [](const GenZLIO::Result &r) { return toMatrix(r.pose); })
        .def_readonly("velocity", &GenZLIO::Result::velocity)
        .def_property_readonly("deskewed",
                               [](const GenZLIO::Result &r) { return toArray(r.deskewed); })
        .def_readonly("leaf_size", &GenZLIO::Result::leaf_size)
        .def_readonly("scale_indicator", &GenZLIO::Result::scale_indicator)
        .def_readonly("setpoint", &GenZLIO::Result::setpoint)
        .def_readonly("scan_points", &GenZLIO::Result::scan_points)
        .def_readonly("voxelized_points", &GenZLIO::Result::voxelized_points)
        .def_readonly("mapping_points", &GenZLIO::Result::mapping_points)
        .def_readonly("plane_matches", &GenZLIO::Result::plane_matches)
        .def_readonly("point_matches", &GenZLIO::Result::point_matches)
        .def_readonly("processing_time_ms", &GenZLIO::Result::processing_time_ms)
        .def_readonly("timing", &GenZLIO::Result::timing);

    py::class_<GenZLIO>(m, "GenZLIO")
        .def(py::init<const Config &>(), py::arg("config"))
        .def(
            "register_scan",
            [](GenZLIO &self, const ArrayXd &points, const double scan_begin_time,
               const double scan_end_time, const ArrayXd &imu, const py::object &timestamps,
               const py::object &intensities, const py::object &rings, bool timing_prepared) {
                if (!std::isfinite(scan_begin_time) || !std::isfinite(scan_end_time) || scan_end_time < scan_begin_time)
                    throw std::invalid_argument("scan times must be finite and ordered");
                auto scan = makeRawScan(points, timestamps, intensities, rings);
                scan.timing_prepared = timing_prepared;
                const auto samples = makeImu(imu);
                // The GIL is not needed while the pipeline runs, and holding it
                // would serialize callers that drive several pipelines at once.
                py::gil_scoped_release release;
                return self.registerScan(scan, scan_begin_time, scan_end_time, samples);
            },
            py::arg("points"), py::arg("scan_begin_time"), py::arg("scan_end_time"),
            py::arg("imu"), py::arg("timestamps") = py::none(),
            py::arg("intensities") = py::none(), py::arg("rings") = py::none(),
            py::arg("timing_prepared") = false,
            "Consume one scan with the IMU samples spanning it.")
        .def("reset", &GenZLIO::reset)
        .def_property_readonly("covariance", &GenZLIO::covariance)
        .def_property_readonly("initialized", &GenZLIO::initialized)
        .def_property_readonly("map_size", [](const GenZLIO &s) { return s.map().size(); })
        .def_property_readonly("lidar_pose", [](const GenZLIO &s) {
            // Deskewed points remain in the LiDAR frame. Use the current filter
            // extrinsic, including online calibration, when displaying them.
            const auto &state = s.state();
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 3>(0, 0) = (state.rot * state.offset_R_L_I).toRotationMatrix();
            pose.block<3, 1>(0, 3) = state.rot * state.offset_T_L_I + state.pos;
            return pose;
        }, "Current LiDAR-to-odometry transform, including estimated extrinsics.")
        .def_property_readonly("visualization_snapshot", [](const GenZLIO &s) {
            auto matches = [](const auto &points) {
                py::array_t<double> out({static_cast<py::ssize_t>(points.size()), py::ssize_t(3)});
                auto values = out.mutable_unchecked<2>();
                for (std::size_t i = 0; i < points.size(); ++i)
                    for (int j = 0; j < 3; ++j) values(i, j) = points[i].point_world[j];
                return out;
            };
            // Collect small leaf descriptors, then write directly into the final
            // owned NumPy buffer. A growing vector of every XYZ used to allocate
            // and copy the full map several times on each displayed frame.
            struct Samples {
                const std::vector<PointWithCov> *points;
                const Vec3 *center;
            };
            std::vector<Samples> leaves;
            leaves.reserve(s.map().size());
            std::size_t count = 0;
            // Render retained leaf candidates, or a fitted plane centre when
            // mature planar cells have released their raw samples. No writes.
            auto collect = [&](auto &&self, const OctoTree *cell) -> void {
                bool children = false;
                for (const auto *child : cell->leaves_) {
                    if (child) { children = true; self(self, child); }
                }
                if (children) return;
                const auto &points = cell->grid_points_.empty() ? cell->temp_points_ : cell->grid_points_;
                if (!points.empty()) {
                    leaves.push_back({&points, nullptr});
                    count += points.size();
                } else if (cell->plane_ptr_->is_plane) {
                    leaves.push_back({nullptr, &cell->plane_ptr_->center});
                    ++count;
                }
            };
            for (const auto &entry : s.map()) collect(collect, entry.second);
            py::array_t<double> map({static_cast<py::ssize_t>(count), py::ssize_t(3)});
            auto values = map.mutable_unchecked<2>();
            std::size_t index = 0;
            for (const auto &leaf : leaves) {
                if (leaf.points) {
                    for (const auto &point : *leaf.points) {
                        for (int j = 0; j < 3; ++j) values(index, j) = point.point[j];
                        ++index;
                    }
                } else {
                    for (int j = 0; j < 3; ++j) values(index, j) = (*leaf.center)[j];
                    ++index;
                }
            }
            py::dict snapshot;
            snapshot["planar"] = matches(s.planeMatches());
            snapshot["non_planar"] = matches(s.pointMatches());
            snapshot["map"] = std::move(map);
            return snapshot;
        }, "Owned copies of match points and retained map samples in the odometry frame.")
        .def_property_readonly("gravity_alignment", &GenZLIO::gravityAlignment);

    py::class_<VisualizationMap>(m, "_VisualizationMap")
        .def(py::init<GenZLIO &>(), py::keep_alive<1, 2>())
        .def("update", &VisualizationMap::update,
             py::arg("spacing") = 0.,
             "Owned changed map tiles and current matches; one consumer per engine.");

    m.def("_downsample_display_map", &downsampleDisplayMap,
          py::arg("points"), py::arg("spacing"),
          "One original sample per display cell; zero preserves all points.");

    // Internal reader API: shares timing preparation, IMU selection and resets with ROS.
    py::class_<ScanBuffer>(m, "_ScanBuffer")
        .def(py::init<const PreprocessConfig &>())
        .def("push_scan", [](ScanBuffer &self, const ArrayXd &points, double stamp,
                             const py::object &timestamps, const py::object &intensities,
                             const py::object &rings) {
            return static_cast<int>(self.push(makeRawScan(points, timestamps, intensities, rings), stamp, 0.0));
        }, py::arg("points"), py::arg("stamp"), py::arg("timestamps") = py::none(),
           py::arg("intensities") = py::none(), py::arg("rings") = py::none())
        .def("push_imu", [](ScanBuffer &self, const ArrayXd &values) {
            int event = 0;
            for (const auto &sample : makeImu(values)) {
                const int next = static_cast<int>(self.push(sample));
                if (next) event = next;
            }
            return event;
        })
        .def("pop", [](ScanBuffer &self) -> py::object {
            ScanBuffer::BufferedScan bundle;
            std::vector<ImuSample> imu;
            if (!self.pop(bundle, imu)) return py::none();
            const auto n = static_cast<py::ssize_t>(bundle.scan.points.size());
            py::array_t<double> times(n);
            py::array_t<float> intensities(n);
            py::array_t<int> rings(static_cast<py::ssize_t>(bundle.scan.rings.size()));
            for (py::ssize_t i = 0; i < n; ++i) {
                times.mutable_at(i) = bundle.scan.points[i].time_offset;
                intensities.mutable_at(i) = bundle.scan.points[i].intensity;
            }
            for (std::size_t i = 0; i < bundle.scan.rings.size(); ++i)
                rings.mutable_at(i) = bundle.scan.rings[i];
            py::array_t<double> samples({static_cast<py::ssize_t>(imu.size()), py::ssize_t(7)});
            auto view = samples.mutable_unchecked<2>();
            for (std::size_t i = 0; i < imu.size(); ++i) {
                view(i, 0) = imu[i].timestamp;
                for (int j = 0; j < 3; ++j) {
                    view(i, j + 1) = imu[i].linear_acceleration[j];
                    view(i, j + 4) = imu[i].angular_velocity[j];
                }
            }
            return py::make_tuple(toArray(bundle.scan.points), bundle.begin_time, bundle.end_time,
                                  samples, times, intensities,
                                  bundle.scan.rings.empty() ? py::object(py::none()) : py::object(rings), bundle.epoch);
        })
        .def_property_readonly("pending_scans", &ScanBuffer::pendingScans);

    m.attr("__version__") = "0.1.0";
    m.attr("__core_source_hash__") = GENZ_LIO_SOURCE_HASH;
}
