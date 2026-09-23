// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "GenZLIO.hpp"

#include "Downsample.hpp"
#include "FilterUpdate.hpp"
#include "StateTransforms.hpp"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <set>
#include <string>
#include <stdexcept>

namespace genz_lio {
namespace {

/// A scan too small to constrain the state is skipped rather than fed to the filter.
constexpr std::size_t kMinScanPoints = 5;

/// Count physical cores from Linux thread-sibling topology. This does not set
/// CPU affinity. Fall back to the processor count reported by OpenMP if no
/// topology entries are available.
int physicalCoreCount() {
    std::set<std::string> cores;
    for (int cpu = 0; cpu < 1024; ++cpu) {
        const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) +
                                 "/topology/thread_siblings_list";
        std::ifstream file(path);
        if (!file) break;
        std::string siblings;
        if (std::getline(file, siblings)) cores.insert(siblings);
    }
    if (cores.empty()) return std::max(1, omp_get_num_procs());
    return static_cast<int>(cores.size());
}

/// Points whose measurement is most certain are folded into the map first, so
/// that plane fits are seeded by the returns that deserve the most weight.
bool moreCertainFirst(const PointWithCov &a, const PointWithCov &b) {
    return a.cov.diagonal().squaredNorm() < b.cov.diagonal().squaredNorm();
}

}  // namespace

GenZLIO::GenZLIO(const Config &config)
    : config_(config),
      preprocessor_(config.preprocess),
      voxelizer_(config.adaptive_voxelization, config.mapping.down_sample_size),
      update_(config) {
    convergence_limit_.fill(0.001);
    max_threads_ = config_.max_threads > 0 ? config_.max_threads : physicalCoreCount();
    omp_set_num_threads(max_threads_);

    imu_processor_.setExtrinsic(config_.mapping.extrinsic_t, config_.mapping.extrinsic_r);
    const auto &noise = config_.noise_model;
    imu_processor_.setGyrCov(Vec3::Constant(noise.gyr_cov));
    imu_processor_.setAccCov(Vec3::Constant(noise.acc_cov));
    imu_processor_.setGyrBiasCov(Vec3::Constant(noise.b_gyr_cov));
    imu_processor_.setAccBiasCov(Vec3::Constant(noise.b_acc_cov));

    installObservationModel();
}

GenZLIO::~GenZLIO() { clearMap(); }

void GenZLIO::installObservationModel() {
    kf_.init_dyn_share(getF, dfDx, dfDw,
                       [this](state_ikfom &state, esekfom::dyn_share_datastruct<double> &data) {
                           update_(state, data);
                       },
                       config_.mapping.max_iteration, convergence_limit_.data());
}

void GenZLIO::clearMap() {
    for (auto &entry : voxel_map_) delete entry.second;
    voxel_map_.clear();
    map_built_ = false;
    display_map_changes_.clear();
    display_map_reset_ = track_map_;
}

void GenZLIO::startMapTracking() {
    if (track_map_) throw std::logic_error("a display map consumer is already attached");
    track_map_ = true;
    display_map_reset_ = true;
}

void GenZLIO::stopMapTracking() {
    track_map_ = false;
    display_map_reset_ = false;
    display_map_changes_.clear();
}

GenZLIO::MapChanges GenZLIO::takeMapChanges() {
    if (!track_map_) throw std::logic_error("display map tracking is not enabled");
    MapChanges changes;
    changes.reset = display_map_reset_;
    if (changes.reset) {
        changes.updated.reserve(voxel_map_.size());
        for (const auto &entry : voxel_map_) changes.updated.push_back(entry.first);
    } else {
        for (const auto &entry : display_map_changes_)
            (entry.second ? changes.updated : changes.removed).push_back(entry.first);
    }
    display_map_changes_.clear();
    display_map_reset_ = false;
    return changes;
}

void GenZLIO::reset() {
    clearMap();
    imu_processor_.reset();
    voxelizer_ = ScaleAwareVoxelizer(config_.adaptive_voxelization, config_.mapping.down_sample_size);
    update_ = HybridMetricUpdate(config_);
    kf_ = EsekfState();
    installObservationModel();
}

std::vector<Mat3> GenZLIO::bodyCovariances(const PointCloud &cloud) const {
    std::vector<Mat3> covariances;
    covariances.reserve(cloud.size());
    for (const auto &point : cloud) {
        Vec3 p = point.position;
        covariances.push_back(computeBodyCovariance(p, config_.noise_model.ranging_cov,
                                                    config_.noise_model.angle_cov));
    }
    return covariances;
}

void GenZLIO::seedMap(const PointCloud &cloud) {
    const state_ikfom state = kf_.get_x();
    const auto covariances = bodyCovariances(cloud);

    std::vector<PointWithCov> points;
    points.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        PointWithCov pv;
        pv.point = transformPointToWorld(state, cloud[i].position);
        pv.point_world = pv.point;
        pv.cov_lidar = covariances[i];
        pv.cov = transformCovarianceToWorld(cloud[i].position, kf_, pv.cov_lidar);
        points.push_back(pv);
    }

    const auto &mapping = config_.mapping;
    buildVoxelMap(points, mapping.voxel_size, mapping.max_layer, mapping.layer_point_size,
                  mapping.max_points_size, mapping.max_mature_points_size,
                  mapping.planar_threshold, config_.hybrid_metric.max_points_per_voxel,
                  config_.hybrid_metric.reduction_ratio, config_.hybrid_metric.enable, voxel_map_);
    map_built_ = true;
    if (track_map_) display_map_reset_ = true;
}

void GenZLIO::foldScanIntoMap(const PointCloud &cloud, const std::vector<Mat3> &body_covariances) {
    const state_ikfom state = kf_.get_x();

    std::vector<PointWithCov> points;
    points.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        PointWithCov pv;
        // The map stores world-frame points; the covariance is propagated from
        // the body frame before the point itself is overwritten.
        pv.cov_lidar = body_covariances[i];
        pv.cov = transformCovarianceToWorld(cloud[i].position, kf_, pv.cov_lidar);
        pv.point = transformPointToWorld(state, cloud[i].position);
        pv.point_world = pv.point;
        points.push_back(pv);
    }
    std::sort(points.begin(), points.end(), moreCertainFirst);

    const auto &mapping = config_.mapping;
    updateVoxelMap(points, mapping.voxel_size, mapping.max_layer, mapping.layer_point_size,
                   mapping.max_points_size, mapping.max_mature_points_size,
                   mapping.planar_threshold, config_.hybrid_metric.max_points_per_voxel,
                   config_.hybrid_metric.reduction_ratio, config_.hybrid_metric.enable,
                   voxel_map_);

    if (track_map_ && !display_map_reset_) {
        // Use exactly the root-key arithmetic used inside updateVoxelMap (its
        // voxel-size argument is float). Mapping and estimation clouds differ.
        const float voxel_size = mapping.voxel_size;
        for (const auto &point : points)
            display_map_changes_[VoxelKey(point.point / voxel_size)] = true;
    }

    if (mapping.map_range > 0.0) {
        std::vector<VoxelKey> removed;
        removePointsFarFromLocation(state.pos, voxel_map_, mapping.map_range,
                                    track_map_ && !display_map_reset_ ? &removed : nullptr);
        for (const auto &key : removed) display_map_changes_[key] = false;
    }
}

GenZLIO::Result GenZLIO::registerScan(const RawScan &raw_scan, const double scan_begin_time,
                                      const double scan_end_time,
                                      const std::vector<ImuSample> &imu) {
    // OpenMP settings belong to the calling thread. ROS 2 executor workers (or
    // Python worker threads) need the instance's limit here, not just at construction.
    omp_set_num_threads(max_threads_);
    using Clock = std::chrono::high_resolution_clock;
    const auto started = Clock::now();
    auto elapsed_ms = [](const Clock::time_point &from) {
        return std::chrono::duration<double, std::milli>(Clock::now() - from).count();
    };

    Result result;
    result.timestamp = scan_end_time;

    auto stage = Clock::now();
    MeasurementBundle bundle;
    bundle.scan = preprocessor_.process(raw_scan);
    result.timing.preprocess = elapsed_ms(stage);
    bundle.scan_begin_time = scan_begin_time;
    bundle.scan_end_time = scan_end_time;
    bundle.imu = imu;
    result.scan_points = static_cast<int>(bundle.scan.size());

    stage = Clock::now();
    PointCloud deskewed;
    imu_processor_.process(bundle, kf_, deskewed);
    result.timing.deskew = elapsed_ms(stage);
    if (!imu_processor_.initialized() || deskewed.empty()) return result;

    // The first usable scan defines the map rather than being aligned to it.
    if (!map_built_) {
        seedMap(deskewed);
        result.valid = true;
        result.pose.rotation = kf_.get_x().rot.toRotationMatrix();
        result.pose.translation = kf_.get_x().pos;
        result.deskewed = std::move(deskewed);
        return result;
    }

    stage = Clock::now();
    auto voxelized = voxelizer_.process(deskewed, scan_end_time);
    result.timing.voxelize = elapsed_ms(stage);
    result.leaf_size = voxelized.leaf_size;
    result.median_range = voxelized.median_range;
    result.scale_indicator = voxelized.scale_indicator;
    result.setpoint = voxelized.setpoint;
    result.voxelized_points = static_cast<int>(voxelized.estimation.size());
    result.mapping_points = static_cast<int>(voxelized.mapping.size());

    if (voxelized.estimation.size() < kMinScanPoints) {
        result.deskewed = std::move(deskewed);
        return result;
    }

    stage = Clock::now();
    const auto estimation_cov = bodyCovariances(voxelized.estimation);
    result.timing.covariance = elapsed_ms(stage);
    update_.setFrame(&voxelized.estimation, &estimation_cov, &voxel_map_, &kf_);

    stage = Clock::now();
    const state_ikfom predicted = kf_.get_x();
    result.update_rejected = !updateWithRollback(kf_);
    state_ikfom corrected = kf_.get_x();
    result.timing.update = elapsed_ms(stage);

    // How far the update had to move the prediction; this drives the residual
    // gate on the next scan.
    const Eigen::Quaterniond deviation_rotation =
        Eigen::Quaterniond(predicted.rot.conjugate() * corrected.rot);
    const Vec3 deviation_translation =
        predicted.rot.conjugate() * (corrected.pos - predicted.pos);
    if (!result.update_rejected && update_.stats().plane_matches + update_.stats().point_matches > 0)
        update_.setPoseDeviation(deviation_rotation, deviation_translation);

    stage = Clock::now();
    if (!result.update_rejected) {
        const auto mapping_cov = bodyCovariances(voxelized.mapping);
        foldScanIntoMap(voxelized.mapping, mapping_cov);
    }
    result.timing.map = elapsed_ms(stage);

    result.valid = true;
    result.pose.rotation = corrected.rot.toRotationMatrix();
    result.pose.translation = corrected.pos;
    result.velocity = corrected.vel;
    result.plane_matches = update_.stats().plane_matches;
    result.point_matches = update_.stats().point_matches;
    result.deskewed = std::move(deskewed);
    // Transfer the already-computed cloud only after all filter/map work.
    result.voxelized = std::move(voxelized.estimation);

    result.processing_time_ms = elapsed_ms(started);
    return result;
}

}  // namespace genz_lio
