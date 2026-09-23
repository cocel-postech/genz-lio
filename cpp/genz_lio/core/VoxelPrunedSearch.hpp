// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Voxel-pruned correspondence search and residual construction (paper Sec. V-B).
//
// Finding the closest map point to a query would ordinarily mean visiting all 27
// voxels around it. Instead a compact candidate set is chosen from where the
// query falls inside its root voxel, and candidates are then skipped outright
// once their distance to the query exceeds the closest match found so far.
#pragma once

#include "VoxelMap.hpp"

#include <vector>

namespace genz_lio {

/// Nearest map point to the query, among those within the residual gate.
void associateEuclidean(const PointWithCov &query, const std::vector<PointWithCov> &neighbors,
                        PointWithCov &closest_neighbor, double &squared_closest_distance,
                        bool &updated, const double squared_point_sigma_threshold);


void buildHybridResidual(const PointWithCov &pv, const OctoTree *current_octo,
                           const double sigma_num, bool &plane_match_found,
                           double &prob, PointToPlaneMatch &plane_match,
                           const double squared_point_sigma_threshold,
                           bool &point_match_found,
                           double &squared_closest_distance, PointToPointMatch &point_match,
                           int &point_search_count);


void buildPlaneResidual(const PointWithCov &pv, const OctoTree *current_octo,
                                const int current_layer, const int max_layer,
                                const double sigma_num, bool &plane_match_found,
                                double &prob, PointToPlaneMatch &plane_match);

void buildPointResidual(const PointWithCov &pv, const OctoTree *current_octo,
                                const double squared_point_sigma_threshold,
                                bool &point_match_found,
                                double &squared_closest_distance, PointToPointMatch &point_match,
                                int &point_search_count);


std::vector<Vec3i> getCandidateVoxels(const int mag_shift, const Vec3i max_voxel_shift);

double getDistanceToNeighborVoxel(const Vec3i voxel_shift, const VoxelKey  near_position, const Vec3 loc_xyz, const double voxel_size);


void buildHybridResidualListPerPointRange(const tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &voxel_map,
                                          const double voxel_size, 
                                          const double sigma_num,
                                          const double point_sigma_num,
                                          const double initial_threshold,
                                          const double max_range_motion,
                                          const Eigen::Quaterniond &rot, const Vec3 &pos, const int num_samples,
                                          const int max_layer,
                                          const std::vector<PointWithCov> &pv_list,
                                          std::vector<PointToPlaneMatch> &plane_matches,
                                          std::vector<PointToPointMatch> &point_matches);

void buildPlaneResidualList(const tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<PointWithCov> &pv_list,
                          std::vector<PointToPlaneMatch> &plane_matches,
                          std::vector<Eigen::Vector3d> &non_match);


}  // namespace genz_lio
