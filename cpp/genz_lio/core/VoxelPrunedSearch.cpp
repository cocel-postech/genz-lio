// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "VoxelPrunedSearch.hpp"

#include <algorithm>
#include <cmath>

namespace genz_lio {

void associateEuclidean(const PointWithCov &query, const std::vector<PointWithCov> &neighbors,
                        PointWithCov &closest_neighbor, double &squared_closest_distance,
                        bool &updated, const double squared_point_sigma_threshold) {
  for (const auto &neighbor : neighbors) {
    const double squared_distance = (neighbor.point - query.point_world).squaredNorm();
    if (squared_distance >= squared_point_sigma_threshold) continue;
    if (squared_closest_distance <= squared_distance) continue;

    closest_neighbor = neighbor;
    squared_closest_distance = squared_distance;
    updated = true;
  }
}

void buildHybridResidual(const PointWithCov &pv, const OctoTree *current_octo,
                           const double sigma_num, bool &plane_match_found,
                           double &prob, PointToPlaneMatch &plane_match,
                           const double squared_point_sigma_threshold,
                           bool &point_match_found,
                           double &squared_closest_distance, PointToPointMatch &point_match,
                           int &point_search_count) {
  const int current_layer = current_octo->layer_;
  const int max_layer = current_octo->max_layer_;

  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_world;
  if (current_octo->plane_ptr_->is_plane) {
    Plane &plane = *current_octo->plane_ptr_;
    float dis_to_plane =
        fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
             plane.normal(2) * p_w(2) + plane.d);
    float dis_to_center =
        (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
        (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
        (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane.radius) {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center;
      J_nq.block<1, 3>(0, 3) = -plane.normal;
      double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
      sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
      if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
        plane_match_found = true;
        double this_prob = 1.0 / (sqrt(sigma_l)) *
                           exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob) {
          prob = this_prob;
          plane_match.point = pv.point;
          plane_match.point_world = pv.point_world;
          plane_match.plane_cov = plane.plane_cov;
          plane_match.normal = plane.normal;
          plane_match.center = plane.center;
          plane_match.d = plane.d;
          plane_match.layer = current_layer;
          plane_match.cov_lidar = pv.cov_lidar;
        }
        return;
      } else {
        if (plane_match_found)  return; // A point already matched to a plane needs no point-to-point search.

        bool updated = false;

        // The cell is planar but the residual was rejected: fall back to its candidates.
        PointWithCov closest_neighbor = current_octo->pv_center;
        std::vector<PointWithCov> octo_center;
        octo_center.push_back(std::move(closest_neighbor));

        associateEuclidean(pv, octo_center, closest_neighbor,
                                            squared_closest_distance, updated,
                                            squared_point_sigma_threshold);
        point_search_count += octo_center.size();
        const auto &neighbors = current_octo->grid_points_;
        associateEuclidean(pv, neighbors, closest_neighbor,
                                            squared_closest_distance, updated,
                                            squared_point_sigma_threshold);
        point_search_count += neighbors.size();

        if (updated) {
          point_match_found = true;
          point_match.point = pv.point;
          point_match.point_world = pv.point_world;
          point_match.cov_lidar = pv.cov_lidar;
          point_match.point_center = closest_neighbor.point;
          point_match.point_cov = closest_neighbor.cov;
        }
        return;
      }
    } else {
      if (plane_match_found)  return;

      // Planar cell whose residual was rejected.
      bool updated = false;
      PointWithCov closest_neighbor = current_octo->pv_center;
      std::vector<PointWithCov> octo_center;
      octo_center.push_back(std::move(closest_neighbor));

      associateEuclidean(pv, octo_center, closest_neighbor,
                                          squared_closest_distance, updated,
                                          squared_point_sigma_threshold);
      point_search_count += octo_center.size();
      const auto &neighbors = current_octo->grid_points_;
      associateEuclidean(pv, neighbors, closest_neighbor,
                                          squared_closest_distance, updated,
                                          squared_point_sigma_threshold);
      point_search_count += neighbors.size();

      if (updated) {
        point_match_found = true;
        point_match.point = pv.point;
        point_match.point_world = pv.point_world;
        point_match.cov_lidar = pv.cov_lidar;
        point_match.point_center = closest_neighbor.point;
        point_match.point_cov = closest_neighbor.cov;
      }
      return;
    }
  } else {
    // Only search for a point-to-point match when no plane was found.
    if (!plane_match_found) {
      // Too few points to fit a plane, so search the temporary points instead.
      // The deepest layer cannot subdivide further, so it searches its candidates.
      bool updated = false;
      PointWithCov closest_neighbor = current_octo->pv_center;
      if (!(current_octo->init_octo_)){ 
        const auto &neighbors = current_octo->temp_points_;
        associateEuclidean(pv, neighbors, closest_neighbor,
                                            squared_closest_distance, updated,
                                            squared_point_sigma_threshold);
        point_search_count += neighbors.size();

      }
      else if(current_layer == max_layer - 1){
        std::vector<PointWithCov> octo_center;
        octo_center.push_back(std::move(closest_neighbor));
        associateEuclidean(pv, octo_center, closest_neighbor,
                                            squared_closest_distance, updated,
                                            squared_point_sigma_threshold);
        point_search_count += octo_center.size();

        const auto &neighbors = current_octo->grid_points_;
        associateEuclidean(pv, neighbors, closest_neighbor,
                                            squared_closest_distance, updated,
                                            squared_point_sigma_threshold);
        point_search_count += neighbors.size();

      }
      if (updated) {  // Either uninitialized or at the deepest layer, so there is nothing below.
        point_match_found = true;
        point_match.point = pv.point;
        point_match.point_world = pv.point_world;
        point_match.cov_lidar = pv.cov_lidar;
        point_match.point_center = closest_neighbor.point;
        point_match.point_cov = closest_neighbor.cov;
        return;
      }
    }

    // Without enough points there are no children to descend into.
    if (!(current_octo->init_octo_))  return;

    if (current_layer < max_layer-1) {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (current_octo->leaves_[leafnum] != nullptr) {

          OctoTree *leaf_octo = current_octo->leaves_[leafnum];
          buildHybridResidual(pv, leaf_octo,
                                        sigma_num, plane_match_found, prob, plane_match, 
                                        squared_point_sigma_threshold, point_match_found,
                                        squared_closest_distance, point_match, point_search_count);
        }
      }
      return;
    } else {
      return;
    }
  }
}

void buildPlaneResidual(const PointWithCov &pv, const OctoTree *current_octo,
                                const int current_layer, const int max_layer,
                                const double sigma_num, bool &plane_match_found,
                                double &prob, PointToPlaneMatch &plane_match) {
  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_world;
  if (current_octo->plane_ptr_->is_plane) {
    Plane &plane = *current_octo->plane_ptr_;
    float dis_to_plane =
        fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
             plane.normal(2) * p_w(2) + plane.d);
    float dis_to_center =
        (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
        (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
        (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane.radius) {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center;
      J_nq.block<1, 3>(0, 3) = -plane.normal;
      double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
      sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
      if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
        plane_match_found = true;
        double this_prob = 1.0 / (sqrt(sigma_l)) *
                           exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob) {
          prob = this_prob;
          plane_match.point = pv.point;
          plane_match.point_world = pv.point_world;
          plane_match.plane_cov = plane.plane_cov;
          plane_match.normal = plane.normal;
          plane_match.center = plane.center;
          plane_match.d = plane.d;
          plane_match.layer = current_layer;
          plane_match.cov_lidar = pv.cov_lidar;
        }
        return;
      } else {
        return;
      }
    } else {
      return;
    }
  } else {
    if (!(current_octo->init_octo_))  return;
    if (current_layer < max_layer - 1) {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (current_octo->leaves_[leafnum] != nullptr) {

          OctoTree *leaf_octo = current_octo->leaves_[leafnum];
          buildPlaneResidual(pv, leaf_octo, current_layer + 1, max_layer,
                                    sigma_num, plane_match_found, prob, plane_match);
        }
      }
      return;
    } else {
      return;
    }
  }
}

void buildPointResidual(const PointWithCov &pv, const OctoTree *current_octo,
                                const double squared_point_sigma_threshold,
                                bool &point_match_found,
                                double &squared_closest_distance, PointToPointMatch &point_match,
                                int &point_search_count) {
  const int current_layer = current_octo->layer_;
  const int max_layer = current_octo->max_layer_;

  Eigen::Vector3d p_w = pv.point_world;
  if (current_octo->plane_ptr_->is_plane) {
    bool updated = false;

    PointWithCov closest_neighbor = current_octo->pv_center;
    std::vector<PointWithCov> octo_center;
    octo_center.push_back(std::move(closest_neighbor));

    associateEuclidean(pv, octo_center, closest_neighbor,
                                        squared_closest_distance, updated,
                                        squared_point_sigma_threshold);
    point_search_count += octo_center.size();
    const auto &neighbors = current_octo->grid_points_;
    associateEuclidean(pv, neighbors, closest_neighbor,
                                        squared_closest_distance, updated,
                                        squared_point_sigma_threshold);
    point_search_count += neighbors.size();

    if (updated) {
      point_match_found = true;
      point_match.point = pv.point;
      point_match.point_world = pv.point_world;
      point_match.cov_lidar = pv.cov_lidar;
      point_match.point_center = closest_neighbor.point;
      point_match.point_cov = closest_neighbor.cov;
    }
    return;
  } else {
    bool updated = false;
    PointWithCov closest_neighbor = current_octo->pv_center;
    if (!(current_octo->init_octo_)){ 
      const auto &neighbors = current_octo->temp_points_;
      associateEuclidean(pv, neighbors, closest_neighbor,
                                          squared_closest_distance, updated,
                                          squared_point_sigma_threshold);
      point_search_count += neighbors.size();

    }
    else if(current_layer == max_layer - 1){
      std::vector<PointWithCov> octo_center;
      octo_center.push_back(std::move(closest_neighbor));
      associateEuclidean(pv, octo_center, closest_neighbor,
                                          squared_closest_distance, updated,
                                          squared_point_sigma_threshold);
      point_search_count += octo_center.size();

      const auto &neighbors = current_octo->grid_points_;
      associateEuclidean(pv, neighbors, closest_neighbor,
                                          squared_closest_distance, updated,
                                          squared_point_sigma_threshold);
      point_search_count += neighbors.size();

    }
    if (updated) {  // Either uninitialized or at the deepest layer, so there is nothing below.
      point_match_found = true;
      point_match.point = pv.point;
      point_match.point_world = pv.point_world;
      point_match.cov_lidar = pv.cov_lidar;
      point_match.point_center = closest_neighbor.point;
      point_match.point_cov = closest_neighbor.cov;
      return;
    }

    if (!(current_octo->init_octo_))  return;

    if (current_layer < max_layer-1) {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (current_octo->leaves_[leafnum] != nullptr) {

          OctoTree *leaf_octo = current_octo->leaves_[leafnum];
          buildPointResidual(pv, leaf_octo,
                                    squared_point_sigma_threshold,point_match_found,
                                    squared_closest_distance, point_match, point_search_count);
        }
      }
      return;
    } else {
      return;
    }
  }
}

std::vector<Vec3i> getCandidateVoxels(const int mag_shift, const Vec3i max_voxel_shift) {
  const int num_shift = (1 << mag_shift) - 1;
  std::vector<Vec3i> voxel_shifts(num_shift);
  const int dx = max_voxel_shift.x();
  const int dy = max_voxel_shift.y();
  const int dz = max_voxel_shift.z();
  if (mag_shift == 3) {
    voxel_shifts[0] = Vec3i(dx, 0, 0);
    voxel_shifts[1] = Vec3i(0, dy, 0);
    voxel_shifts[2] = Vec3i(0, 0, dz);
    voxel_shifts[3] = Vec3i(dx, dy, 0);
    voxel_shifts[4] = Vec3i(dx, 0, dz);
    voxel_shifts[5] = Vec3i(0, dy, dz);
    voxel_shifts[6] = Vec3i(dx, dy, dz);
  }
  else if (mag_shift == 2) {
    if (dx == 0) {
      voxel_shifts[0] = Vec3i(0, dy, 0);
      voxel_shifts[1] = Vec3i(0, 0, dz);
      voxel_shifts[2] = Vec3i(0, dy, dz);

    }
    else if (dy == 0) {
      voxel_shifts[0] = Vec3i(dx, 0, 0);
      voxel_shifts[1] = Vec3i(0, 0, dz);
      voxel_shifts[2] = Vec3i(dx, 0, dz);

    }
    else {
      voxel_shifts[0] = Vec3i(dx, 0, 0);
      voxel_shifts[1] = Vec3i(0, dy, 0);
      voxel_shifts[2] = Vec3i(dx, dy, 0);
    }
  }
  else {
    if (dx != 0) voxel_shifts[0] = Vec3i(dx, 0, 0);
    else if (dy != 0) voxel_shifts[0] = Vec3i(0, dy, 0);
    else  voxel_shifts[0] = Vec3i(0, 0, dz);
  }
  // const std::vector<Vec3i> voxel_shifts{
  //     {Vec3i{1, 0, 0},   Vec3i{-1, 0, 0},  Vec3i{0, 1, 0},   Vec3i{0, -1, 0},
  //     Vec3i{0, 0, 1},   Vec3i{0, 0, -1},  Vec3i{1, 1, 0},   Vec3i{1, -1, 0},  Vec3i{-1, 1, 0},
  //     Vec3i{-1, -1, 0}, Vec3i{1, 0, 1},   Vec3i{1, 0, -1},  Vec3i{-1, 0, 1},  Vec3i{-1, 0, -1},
  //     Vec3i{0, 1, 1},   Vec3i{0, 1, -1},  Vec3i{0, -1, 1},  Vec3i{0, -1, -1}, Vec3i{1, 1, 1},
  //     Vec3i{1, 1, -1},  Vec3i{1, -1, 1},  Vec3i{1, -1, -1}, Vec3i{-1, 1, 1},  Vec3i{-1, 1, -1},
  //     Vec3i{-1, -1, 1}, Vec3i{-1, -1, -1}}};

  return voxel_shifts;
}

double getDistanceToNeighborVoxel(const Vec3i voxel_shift, const VoxelKey  near_position, const Vec3 loc_xyz, const double voxel_size) {
  double squared_dist_to_near;
  double mag_voxel_shift = std::abs(voxel_shift.x()) + std::abs(voxel_shift.y()) + std::abs(voxel_shift.z());
  Vec3 near_loc_xyz(near_position.x * voxel_size, near_position.y * voxel_size, near_position.z * voxel_size);
  // A neighbour on the negative side is measured from the current cell, so one
// voxel size is added back.
  // For a point at -0.8 inside cell -1 looking towards cell -2, the closest
// approach to that neighbour is the boundary at -1.
  if (voxel_shift.x() == -1) near_loc_xyz.x() += voxel_size;
  if (voxel_shift.y() == -1) near_loc_xyz.y() += voxel_size;
  if (voxel_shift.z() == -1) near_loc_xyz.z() += voxel_size;

  const double dx = near_loc_xyz.x() - loc_xyz.x();
  const double dy = near_loc_xyz.y() - loc_xyz.y();
  const double dz = near_loc_xyz.z() - loc_xyz.z();

  if (mag_voxel_shift == 3) {
    squared_dist_to_near = dx*dx + dy*dy + dz*dz;
  }
  else if(mag_voxel_shift == 2) {
    if (voxel_shift.x() == 0)       squared_dist_to_near = dy*dy + dz*dz;
    else if (voxel_shift.y()== 0)   squared_dist_to_near = dz*dz + dx*dx;
    else                            squared_dist_to_near = dx*dx + dy*dy;

  }
  else {
    if (voxel_shift.x() != 0)       squared_dist_to_near = dx*dx;
    else if (voxel_shift.y() != 0)  squared_dist_to_near = dy*dy;
    else                            squared_dist_to_near = dz*dz;
  }

  return squared_dist_to_near;
}

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
                                          std::vector<PointToPointMatch> &point_matches) {

  std::mutex mylock;
  plane_matches.clear();
  point_matches.clear();
  std::vector<PointToPlaneMatch> all_plane_matches(pv_list.size());
  std::vector<PointToPointMatch> all_point_matches(pv_list.size());
  std::vector<int> correspondence_type(pv_list.size(), 0);
  std::vector<size_t> index(pv_list.size());

  size_t plane_match_count = 0;
  size_t point_match_count = 0;

  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

#pragma omp parallel for
  for (int i = 0; i < index.size(); i++) {
    PointWithCov pv = pv_list[i];
    const Vec3& loc_xyz = pv.point_world;
    VoxelKey position(loc_xyz / voxel_size);

    auto iter = voxel_map.find(position);

    if (iter != voxel_map.end()) {
      OctoTree *current_octo = iter->second;
      PointToPlaneMatch plane_match;
      PointToPointMatch point_match;
      bool plane_match_found = false, point_match_found = false;
      double prob = 0;
      int point_search_count = 0;

      // Retain the startup uncertainty as a floor. A stationary or nearly
      // converged correction must not collapse the correspondence radius to zero.
      const double initial_variance = initial_threshold * initial_threshold;
      const double squared_point_sigma = num_samples > 1
          ? std::max(initial_variance, computeMotionThreshold(rot, pos, max_range_motion, pv.point))
          : initial_variance;
      const double squared_point_sigma_threshold = point_sigma_num * point_sigma_num * squared_point_sigma;
      double squared_closest_distance = point_sigma_num * point_sigma_num * squared_point_sigma;
      buildHybridResidual(pv, current_octo,
                                    sigma_num, plane_match_found, prob, plane_match, 
                                    squared_point_sigma_threshold,
                                    point_match_found, squared_closest_distance, 
                                    point_match, point_search_count);
      point_match.num_candidate_voxels = 1;

      if (!plane_match_found) {

        VoxelKey nearest_position = position;
        Vec3i max_voxel_shift = Vec3i::Zero();
        int mag_shift = 3;

        const double searching_threshold = current_octo->quater_length_*2.0/3.0;
        for (int j=0; j<3 ; j++) {
          if (loc_xyz(j) > (current_octo->voxel_center_[j] + searching_threshold)) max_voxel_shift(j) = 1;
          else if (loc_xyz(j) < (current_octo->voxel_center_[j] - searching_threshold)) max_voxel_shift(j) = -1;
          else  mag_shift--;
        }

        double closest_distance_to_surf = voxel_size;
        int near_surf = -1;
        const Vec3i current_position(position.x, position.y, position.z);
        for (int j =0; j<3; j++) {
          if (max_voxel_shift(j) == 0) continue;
          const double dist_to_surf = (max_voxel_shift(j) > 0) ?
            std::abs((current_position(j) + max_voxel_shift(j)) * voxel_size - loc_xyz(j)) :
            std::abs((current_position(j)) * voxel_size - loc_xyz(j));

          if (dist_to_surf < closest_distance_to_surf) {
            closest_distance_to_surf = dist_to_surf;
            near_surf = j;
          }
        }

        if (near_surf == 0)       nearest_position.x += max_voxel_shift.x();
        else if (near_surf == 1)  nearest_position.y += max_voxel_shift.y();
        else if (near_surf == 2)  nearest_position.z += max_voxel_shift.z();

        if (near_surf != -1) {
          auto iter_nearest = voxel_map.find(nearest_position);
          if (iter_nearest != voxel_map.end()) {

            const double squared_closest_distance_to_surf = closest_distance_to_surf * closest_distance_to_surf;
            if (squared_closest_distance_to_surf < squared_closest_distance && squared_closest_distance_to_surf < squared_point_sigma_threshold)
              buildHybridResidual(pv, iter_nearest->second,
                                            sigma_num, plane_match_found, prob, plane_match,
                                            squared_point_sigma_threshold, 
                                            point_match_found, squared_closest_distance, 
                                            point_match, point_search_count);
              buildPlaneResidual(pv, iter_nearest->second, 0, max_layer, sigma_num,
                                        plane_match_found, prob, plane_match);

            point_match.num_candidate_voxels++;
          }

          if (!plane_match_found) {
            const auto &voxel_shifts = getCandidateVoxels(mag_shift, max_voxel_shift);

            for (const Vec3i & voxel_shift:voxel_shifts)  {
              VoxelKey near_position = position;
              near_position.x += voxel_shift.x();
              near_position.y += voxel_shift.y();
              near_position.z += voxel_shift.z();

              if (near_position == nearest_position)  continue;
              double squared_dist_to_near = getDistanceToNeighborVoxel(voxel_shift, near_position, loc_xyz, voxel_size);
              if (squared_dist_to_near >= squared_closest_distance || squared_dist_to_near >= squared_point_sigma_threshold) continue;

              auto iter_near = voxel_map.find(near_position);
              if (iter_near != voxel_map.end()) {
                buildPointResidual(pv, iter_near->second,
                                          squared_point_sigma_threshold,point_match_found,
                                          squared_closest_distance, point_match, point_search_count);
                point_match.num_candidate_voxels++;
              }
            }
          }
        }
      }
      point_match.num_neighbors = point_search_count;

      if (plane_match_found) {
        mylock.lock();
        plane_match_count++;
        correspondence_type[i] = 1; 
        all_plane_matches[i] = plane_match;
        mylock.unlock();
      } else if (point_match_found) {
        mylock.lock();
        point_match_count++;
        correspondence_type[i] = 2;
        all_point_matches[i] = point_match;
        mylock.unlock();
      } else {
        mylock.lock();
        correspondence_type[i] = 0;
        mylock.unlock();
      }

    }
  }
  plane_matches.reserve(plane_match_count);
  point_matches.reserve(point_match_count);
  for (size_t i = 0; i < correspondence_type.size(); i++) {
    if (correspondence_type[i] == 1) {
      plane_matches.emplace_back(all_plane_matches[i]);
    } else if (correspondence_type[i] == 2) {
      point_matches.emplace_back(all_point_matches[i]);
    } 
  }
}

void buildPlaneResidualList(const tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<PointWithCov> &pv_list,
                          std::vector<PointToPlaneMatch> &plane_matches,
                          std::vector<Eigen::Vector3d> &non_match) {
  std::mutex mylock;
  plane_matches.clear();
  non_match.clear();
  std::vector<PointToPlaneMatch> all_plane_matches(pv_list.size());
  std::vector<Vec3> non_match_list(pv_list.size());
  std::vector<bool> plane_match_flags(pv_list.size());
  std::vector<size_t> index(pv_list.size());

  size_t plane_match_count = 0;
  size_t non_match_num = 0;
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
    plane_match_flags[i] = false;
  }
#pragma omp parallel for
  for (int i = 0; i < index.size(); i++) {
    PointWithCov pv = pv_list[i];
    const Vec3& loc_xyz = pv.point_world;
    VoxelKey position(loc_xyz / voxel_size);
    // The root voxel the query point falls in.
    auto iter = voxel_map.find(position);

    if (iter != voxel_map.end()) {
      OctoTree *current_octo = iter->second;
      PointToPlaneMatch plane_match;
      bool plane_match_found = false;
      double prob = 0;

      buildPlaneResidual(pv, current_octo, 0, max_layer, sigma_num,
                                plane_match_found, prob, plane_match);
      if (!plane_match_found) {
        VoxelKey near_position = position;

        Vec3i max_voxel_shift = Vec3i::Zero();

        const double searching_threshold = current_octo->quater_length_;
        for (int j=0; j<3 ; j++) {
          if (loc_xyz(j) > (current_octo->voxel_center_[j] + searching_threshold)) max_voxel_shift(j) = 1;
          else if (loc_xyz(j) < (current_octo->voxel_center_[j] - searching_threshold)) max_voxel_shift(j) = -1;
        }

        double closest_distance_to_surf = voxel_size;
        int near_surf = -1;
        const Vec3i current_position(position.x, position.y, position.z);
        for (int j =0; j<3; j++) {
          if (max_voxel_shift(j) == 0) continue;
          const double dist_to_surf = (max_voxel_shift(j) > 0) ?
            std::abs((current_position(j) + max_voxel_shift(j)) * voxel_size - loc_xyz(j)) :
            std::abs((current_position(j)) * voxel_size - loc_xyz(j));

          if (dist_to_surf < closest_distance_to_surf) {
            closest_distance_to_surf = dist_to_surf;
            near_surf = j;
          }
        }

        if (near_surf == 0)       near_position.x += max_voxel_shift.x();
        else if (near_surf == 1)  near_position.y += max_voxel_shift.y();
        else if (near_surf == 2)  near_position.z += max_voxel_shift.z();

        if (near_surf != -1) {
          auto iter_near = voxel_map.find(near_position);
          if (iter_near != voxel_map.end()) {
            buildPlaneResidual(pv, iter_near->second, 0, max_layer, sigma_num,
                                      plane_match_found, prob, plane_match);
          }
        }

      }
      if (plane_match_found) {
        mylock.lock();
        plane_match_count++;
        plane_match_flags[i] = true;
        all_plane_matches[i] = plane_match;
        mylock.unlock();
      } else {
        mylock.lock();
        non_match_num++;
        plane_match_flags[i] = false;
        non_match_list[i] = pv.point;
        mylock.unlock();
      }
    }
    else {
      mylock.lock();
      non_match_num++;
      plane_match_flags[i] = false;
      non_match_list[i] = pv.point;
      mylock.unlock();
    }
  }
  plane_matches.reserve(plane_match_count);
  non_match.reserve(non_match_num);
  for (size_t i = 0; i < plane_match_flags.size(); i++) {
    if (plane_match_flags[i]) {
      plane_matches.emplace_back(all_plane_matches[i]);
    }
    else{
      non_match.emplace_back(non_match_list[i]);
    }
  }
}

}  // namespace genz_lio
