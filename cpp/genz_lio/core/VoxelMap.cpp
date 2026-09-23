// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "VoxelMap.hpp"

#include <algorithm>
#include <cmath>

namespace genz_lio {

std::atomic<int> g_plane_id{0};

double computeMotionThreshold(const Eigen::Quaterniond &rot, const Vec3 &pos, const double max_range_motion, const Vec3 query)
{
  // Derives the point-to-point threshold from each point's range, clamped so that
// distant returns do not open up an unbounded search radius.
  // Unlike KISS-ICP, which uses the maximum range, the rotation term is scaled by
// the range of the query point itself.
  const double range_query = std::clamp(query.norm(), 0.1, max_range_motion);
  const double theta = Eigen::AngleAxisd(rot).angle();
  const double delta_rot = 2.0 * range_query * std::sin(theta / 2.0);
  const double delta_trans = pos.norm();

  return (delta_trans + delta_rot)*(delta_trans + delta_rot);
}

void buildVoxelMap(const std::vector<PointWithCov> &input_points, // pv_list
                   const float voxel_size, // max_voxel_size
                   const int max_layer, // max_layer (2~4)
                   const std::vector<int> &layer_point_size, // layer_size ([5, 5, 5, 5, 5])
                   const int max_points_size, // max_points_size (1000)
                   const int max_mature_points_size, // max_mature_points_size (1000)                
                   const float planer_threshold, // min_eigen_value (0.01)
                   const int max_points_per_voxel,
                   const int reduction_ratio,
                   const bool hybrid_metric_enabled,
                   tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map) {

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
        const PointWithCov p_v = input_points[i];
    // Allign a float array
    const Vec3& loc_xyz = p_v.point;

    // Align voxel loc id
    VoxelKey position(loc_xyz / voxel_size);


    // Find the pointer of query voxel
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_num_++;
    } else {
      OctoTree *octo_tree =
          new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                       max_mature_points_size,
                       planer_threshold,
                       voxel_size, max_points_per_voxel, reduction_ratio, hybrid_metric_enabled);
      feat_map[position] = octo_tree;
      feat_map[position]->quater_length_ = voxel_size / 4;
      feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;

      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_num_++;
      feat_map[position]->layer_point_size_ = layer_point_size;
    }
    Eigen::Vector3d center(feat_map[position]->voxel_center_[0], 
                           feat_map[position]->voxel_center_[1], 
                           feat_map[position]->voxel_center_[2]);
    if ((p_v.point - center).squaredNorm() < (feat_map[position]->pv_center.point - center).squaredNorm()) {
      feat_map[position]->pv_center = p_v;
    }
  }

  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    iter->second->initialize();
  }
}

void updateVoxelMap(const std::vector<PointWithCov> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size,
                    const int max_mature_points_size,
                    const float planer_threshold,
                    const int max_points_per_voxel,
                    const int reduction_ratio,
                    const double hybrid_metric_enabled,
                    tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const PointWithCov p_v = input_points[i];
        const Vec3& loc_xyz = p_v.point;
        VoxelKey position(loc_xyz / voxel_size);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->update(p_v);
        } else {
            OctoTree *octo_tree =
                    new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                 max_mature_points_size,
                                 planer_threshold,
                                 voxel_size, max_points_per_voxel, reduction_ratio, hybrid_metric_enabled);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->update(p_v);
        }
    }
}

void removePointsFarFromLocation(const Vec3 &pos,
                                 tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map,
                                 const double max_distance,
                                 std::vector<VoxelKey> *removed) {
    const auto squared_max_distance = max_distance * max_distance;
    for (auto it = feat_map.begin(); it != feat_map.end();) {
        const auto &[voxel, current_octo] = *it;
        const auto pt = Vec3(current_octo->voxel_center_);
        if ((pt - pos).squaredNorm() >= (squared_max_distance)) {
            if (removed) removed->push_back(voxel);
            delete  current_octo;
            it = feat_map.erase(it);
        } else {
            ++it;
        }
    }
}

Mat3 computeBodyCovariance(Eigen::Vector3d &pb, const float range_inc, const float degree_inc)
{
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(degToRad(degree_inc)), 2), 0, 0,
      pow(sin(degToRad(degree_inc)), 2);
  if (!(range > 0.0f)) return Mat3::Zero();
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  // Any orthonormal basis of the plane normal to `direction` gives the same
  // covariance below, because direction_var is isotropic and N * N^T is then the
  // projector onto that plane. The reference built base_vector1 as
  // (1, 1, -(dx + dy) / dz), which divides by zero for a point lying exactly in
  // the sensor's z = 0 plane; Livox scan patterns produce those, and the
  // resulting NaN covariance propagates into the ESIKF. Cross with whichever
  // axis `direction` leans on least instead, which never degenerates.
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d abs_direction = direction.cwiseAbs();
  if (abs_direction.y() <= abs_direction.x() && abs_direction.y() <= abs_direction.z()) {
    axis = Eigen::Vector3d::UnitY();
  } else if (abs_direction.z() <= abs_direction.x() && abs_direction.z() <= abs_direction.y()) {
    axis = Eigen::Vector3d::UnitZ();
  }
  Eigen::Vector3d base_vector1 = direction.cross(axis);
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  return direction * range_var * direction.transpose() +
        A * direction_var * A.transpose();
}

}  // namespace genz_lio
