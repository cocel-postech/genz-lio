// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Probabilistic voxel map: an octree per root voxel whose cells fit local
// planes and carry the uncertainty of those planes, inherited from VoxelMap and
// PV-LIO. Cells that fail the planarity test are subdivided, and the points
// they hold are what the point-to-point metric matches against.
#pragma once

#include "Config.hpp"
#include "SO3Math.hpp"
#include "Types.hpp"

#include <tsl/robin_map.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <limits>
#include <vector>

namespace genz_lio {

using VoxelMapType = tsl::robin_map<VoxelKey, class OctoTree *, VoxelHash>;

/// Identifiers handed to newly fitted planes. Atomic because cells are fitted
/// from several threads while the map is updated.
extern std::atomic<int> g_plane_id;

/// Minimum number of neighbours a cell needs before a plane is worth fitting.
constexpr int kNumMatchPoints = 5;
constexpr double degToRad(const double degrees) { return degrees * M_PI / 180.0; }

class OctoTree {
public:
  std::vector<PointWithCov> temp_points_; // all points in an octo tree
  std::vector<PointWithCov> new_points_;  // new points in an octo tree
  std::vector<PointWithCov> grid_points_; // Point-to-point candidates, kept only once a plane has been fitted. A cell that
// fails the planarity test is subdivided instead, so its points live deeper down.
  Plane *plane_ptr_;
  int max_layer_;
  bool indoor_mode_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  PointWithCov pv_center;
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int max_plane_update_threshold_;
  int update_size_threshold_;
  int all_points_num_;
  int new_points_num_;
  int max_points_size_;

  int max_mature_points_size_;
  // By SJKim
  double octo_size_;
  int max_points_per_voxel_;  // Point budget of a root voxel.
  double squared_map_resolution_;     // voxel_size^2, the discretization scale implied by the root voxel's budget.
  double voxel_size_;       // Root edge length in metres, including sub-metre sizes.
  int reduction_ratio_;       // A subdivided cell keeps max_points_per_voxel / reduction_ratio^layer points.
  int max_points_per_octo_;   // Point budget of this cell.

  bool init_octo_;
  bool init_grids_;           // Set once a cell first fits a plane and seeds its point-to-point candidates.
  bool update_enable_;
  bool hybrid_metric_enabled_;              // Skips storing point-to-point candidates when the hybrid metric is disabled.
  OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
           int max_point_size,
           int max_mature_points_size,
           float planer_threshold,
           double voxel_size, int max_points_per_voxel, int reduction_ratio, bool hybrid_metric_enabled)
      : max_layer_(max_layer), layer_(layer),
        layer_point_size_(layer_point_size), max_points_size_(max_point_size),
        max_mature_points_size_(max_mature_points_size),
        planer_threshold_(planer_threshold),
        voxel_size_(voxel_size), max_points_per_voxel_(max_points_per_voxel),
        reduction_ratio_(reduction_ratio), hybrid_metric_enabled_(hybrid_metric_enabled){
    temp_points_.clear();
    octo_state_ = 0;
    new_points_num_ = 0;
    all_points_num_ = 0;
    // when new points num > 5, do a update
    update_size_threshold_ = 5;
    init_octo_ = false;
    init_grids_ = false;
    update_enable_ = true;
    max_plane_update_threshold_ = layer_point_size_[layer_];


    // Storage budgets. At the root, layer_ is 0, so the two coincide.
    // Map resolution follows from the cell size and its point budget, as in KISS-ICP.
    max_points_per_octo_ = std::max((int)(max_points_per_voxel_/std::pow(reduction_ratio_, layer_)), 1);
    octo_size_ = voxel_size_/std::pow(2.0, layer_);
    squared_map_resolution_ = octo_size_*octo_size_/max_points_per_octo_;

    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }

  // Used when pruning voxels that have fallen far behind the sensor.
  ~OctoTree() {
    std::vector<PointWithCov>().swap(temp_points_);
    std::vector<PointWithCov>().swap(new_points_);
    std::vector<PointWithCov>().swap(grid_points_);
    if (plane_ptr_) {
        delete plane_ptr_;
        plane_ptr_ = nullptr;
    }
    for (int i = 0; i < 8; i++) {
        if (leaves_[i]) {
            delete leaves_[i];
            leaves_[i] = nullptr;
        }
    }
  }

  /**
   * Fits a plane to `points`: the centre, the covariance and its eigenvalues and
   * eigenvectors, from which the normal and the remaining plane parameters
   * (d, radius, and the uncertainty of the plane itself) follow.
   *
   * @param points          points with their per-point measurement covariance
   * @param plane           plane to fit
   * @param new_points_num  how many of the points at the end are newly arrived
   */
  void initPlane(const std::vector<PointWithCov> &points, Plane *plane, const int new_points_num) {
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();

    // Folds only the newly arrived points into the existing centre and covariance,
// rather than refitting over every point the cell has ever held.
    if (plane->is_init) {
      plane->covariance += plane->center * plane->center.transpose();
      plane->covariance *= plane->points_size;

      plane->center *= plane->points_size;
    }

    plane->normal = Vec3::Zero();
    plane->points_size = points.size();
    plane->radius = 0;

    // Accumulate only the new points.
    for (int i = points.size() - new_points_num; i < points.size(); ++i) {
      const auto &point = points[i].point;
      plane->center += point;
      plane->covariance += point * point.transpose();
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size -
                        plane->center * plane->center.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
    if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
      plane->is_plane = false;
      plane->is_init = true;
      return;
    }
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();

    // SelfAdjointEigenSolver sorts eigenvalues. Searching for min/max indices
    // independently can select the same index when all eigenvalues coincide.
    constexpr int evalsMin = 0, evalsMid = 1, evalsMax = 2;

    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);


    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 
           0, 1.0 / plane->points_size, 0, 
           0, 0, 1.0 / plane->points_size;

    // A line or repeated point has no identifiable plane normal. Require a
    // numerically resolved eigengap before dividing by it in the normal Jacobian.
    const double eigengap_tolerance = 64 * std::numeric_limits<double>::epsilon() *
                                      std::max(1.0, evalsReal.cwiseAbs().maxCoeff());
    bool estimate_plane = (evalsReal(evalsMin) < planer_threshold_) &&
                          (evalsReal(evalsMid) - evalsReal(evalsMin) > eigengap_tolerance);
    if (estimate_plane) {
      std::vector<int> index(points.size());
      std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(points.size());

      for (int i = 0; i < points.size(); i++) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
          if (m != (int)evalsMin) {
            Eigen::Matrix<double, 1, 3> F_m =
                (points[i].point - plane->center).transpose() /
                ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
            F.row(m) = F_m;
          } else {
            Eigen::Matrix<double, 1, 3> F_m;
            F_m << 0, 0, 0;
            F.row(m) = F_m;
          }
        }
        J.block<3, 3>(0, 0) = evecs.real() * F;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_cov += J * points[i].cov * J.transpose();
      }

      plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
          evecs.real()(2, evalsMin);
      plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
          evecs.real()(2, evalsMid);
      plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
          evecs.real()(2, evalsMax);
      plane->min_eigen_value = evalsReal(evalsMin);
      plane->mid_eigen_value = evalsReal(evalsMid);
      plane->max_eigen_value = evalsReal(evalsMax);
      plane->radius = sqrt(std::max(0.0, evalsReal(evalsMax)));
      plane->d = -(plane->normal(0) * plane->center(0) +
                   plane->normal(1) * plane->center(1) +
                   plane->normal(2) * plane->center(2));
      plane->is_plane = plane->plane_cov.allFinite();

      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }
      if (!plane->is_init) {
        plane->id = g_plane_id;
        g_plane_id++;
        plane->is_init = true;
      }

    } else {
      if (!plane->is_init) {
        plane->id = g_plane_id;
        g_plane_id++;
        plane->is_init = true;
      }
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }
      plane->is_plane = false;
      plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
          evecs.real()(2, evalsMin);
      plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
          evecs.real()(2, evalsMid);
      plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
          evecs.real()(2, evalsMax);
      plane->min_eigen_value = evalsReal(evalsMin);
      plane->mid_eigen_value = evalsReal(evalsMid);
      plane->max_eigen_value = evalsReal(evalsMax);
      plane->radius = sqrt(std::max(0.0, evalsReal(evalsMax)));
      plane->d = -(plane->normal(0) * plane->center(0) +
                   plane->normal(1) * plane->center(1) +
                   plane->normal(2) * plane->center(2));
    }
  }

  void initCandidatePoints(){
    // On the first successful plane fit, seed the point-to-point candidates from the
// points gathered so far.
    grid_points_.clear();
    grid_points_.reserve(max_points_per_octo_);

    for (const auto& point : temp_points_) {
      // Check if the point is already in the grid points
      if (grid_points_.size() == max_points_per_octo_)
        break;
      if (std::any_of(grid_points_.cbegin(), grid_points_.cend(),
                      [&](const auto &grid_point) {
                          return (grid_point.point - point.point).squaredNorm() < squared_map_resolution_;
                      })){
        continue;
      }
      grid_points_.emplace_back(point);
    }
    init_grids_ = true;
    return;
  }

  // Folds a single point into the cell.
  void updateCandidatePoint(const PointWithCov &point) {
    if (grid_points_.size() == max_points_per_octo_ || std::any_of(grid_points_.cbegin(), grid_points_.cend(),
                    [&](const auto &grid_point) {
                        return (grid_point.point - point.point).squaredNorm() < squared_map_resolution_;
                    })){
      return;
    }
    grid_points_.emplace_back(point);
    return;
  }

  void clearCandidatePoints(){
    std::vector<PointWithCov>().swap(grid_points_);
    init_grids_ = false;
  }

  void initialize() {
    if (temp_points_.size() > max_plane_update_threshold_) {
      initPlane(temp_points_, plane_ptr_, new_points_num_);
      if (plane_ptr_->is_plane == true) {
        octo_state_ = 0;
                if (temp_points_.size() > max_points_size_) {
          update_enable_ = false;
        }
        // Before initialization the cell searches its temporary points; once a plane is
// fitted the candidates are seeded, and otherwise the cell is subdivided.
        if (hybrid_metric_enabled_ && !init_grids_)  initCandidatePoints();

      } else {
        octo_state_ = 1;
        subdivide();
        if (hybrid_metric_enabled_ && layer_ == max_layer_ - 1 && !init_grids_)
          initCandidatePoints();
        // Only release the temporary points if the cell can still be subdivided.
        if (hybrid_metric_enabled_ && init_grids_ && layer_ < max_layer_ - 1) clearCandidatePoints();
      }
      init_octo_ = true;
      new_points_num_ = 0;
    }
  }

  void subdivide() {
    if (layer_ >= max_layer_ - 1) {
      octo_state_ = 0;
      return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
            int xyz[3] = {
        temp_points_[i].point[0] > voxel_center_[0] ? 1 : 0,
        temp_points_[i].point[1] > voxel_center_[1] ? 1 : 0,
        temp_points_[i].point[2] > voxel_center_[2] ? 1 : 0
      };
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves_[leafnum] == nullptr) {
        leaves_[leafnum] = new OctoTree(
            max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
            max_mature_points_size_,
            planer_threshold_,
            voxel_size_, max_points_per_voxel_, reduction_ratio_, hybrid_metric_enabled_);

        leaves_[leafnum]->voxel_center_[0] =
            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[1] =
            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[2] =
            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;

        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        leaves_[leafnum]->pv_center = temp_points_[i];
      }

      leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
      leaves_[leafnum]->new_points_num_++;
    }
    for (uint i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        if (leaves_[i]->temp_points_.size() >
            leaves_[i]->max_plane_update_threshold_) {

          initPlane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_, leaves_[i]->new_points_num_);
          if (leaves_[i]->plane_ptr_->is_plane) {
            leaves_[i]->octo_state_ = 0;
            if (hybrid_metric_enabled_ && !leaves_[i]->init_grids_) leaves_[i]->initCandidatePoints(); 
          } else {
            leaves_[i]->octo_state_ = 1;
            leaves_[i]->subdivide();
            if (hybrid_metric_enabled_ && leaves_[i]->layer_ == max_layer_ - 1 &&
                !leaves_[i]->init_grids_) leaves_[i]->initCandidatePoints();
            if (hybrid_metric_enabled_ && leaves_[i]->init_grids_ && leaves_[i]->layer_ < max_layer_ - 1) leaves_[i]->clearCandidatePoints();
          }
          leaves_[i]->init_octo_ = true;
          leaves_[i]->new_points_num_ = 0;
        }
      }
    }
  }

  // Reuses the two conditions under which a point-to-plane residual is rejected.
  bool isPointOnPlane(const Plane *plane, const PointWithCov &pv) {

    bool is_on_plane = false;
    const auto &point = pv.point;
    const double radius_k = 3.0;
    const double sigma_num = 3.0;


    float dis_to_plane =
        fabs(plane->normal(0) * point(0) + plane->normal(1) * point(1) +
             plane->normal(2) * point(2) + plane->d);

    float dis_to_center =
        (plane->center(0) - point(0)) * (plane->center(0) - point(0)) +
        (plane->center(1) - point(1)) * (plane->center(1) - point(1)) +
        (plane->center(2) - point(2)) * (plane->center(2) - point(2));

    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane->radius) {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point - plane->center;
      J_nq.block<1, 3>(0, 3) = -plane->normal;
      double sigma_l = J_nq * plane->plane_cov * J_nq.transpose();
      sigma_l += plane->normal.transpose() * pv.cov * plane->normal;
      if (dis_to_plane < sigma_num * sqrt(sigma_l)) is_on_plane = true;
    }

    return is_on_plane;
  }

  // Once enough points have fallen outside the plane, refit the cell using them.
  void refitPlaneWithOutliers(const PointWithCov &pv) {
    if (!update_enable_) {
      if (all_points_num_ < max_mature_points_size_) {
        if (!isPointOnPlane(plane_ptr_, pv)) {

          plane_ptr_->num_out_of_plane++;
          temp_points_.push_back(pv);
          all_points_num_++;
          ++new_points_num_;

          if (plane_ptr_->num_out_of_plane > max_plane_update_threshold_) {
            initPlane(temp_points_, plane_ptr_, new_points_num_);
            new_points_num_ = 0;
            plane_ptr_->num_out_of_plane = 0;
          }
        }
        }
        else {
          if (temp_points_.size() != 0) std::vector<PointWithCov>().swap(temp_points_);
        }
    }
    return;
  }

  void update(const PointWithCov &pv) {
    // save the most closest point to the center of the voxel
    Eigen::Vector3d center(voxel_center_[0], voxel_center_[1], voxel_center_[2]);
    if ((pv.point - center).squaredNorm() < (pv_center.point - center).squaredNorm()) {
      pv_center = pv;
    }

    if (!init_octo_) {
      new_points_num_++;
      all_points_num_++;
      temp_points_.push_back(pv);
      if (temp_points_.size() > max_plane_update_threshold_) {
        initialize();
      }
    } else {
      if (plane_ptr_->is_plane) {
        // With a plane in place, every incoming point is also offered to the
// point-to-point candidate set.
        if (hybrid_metric_enabled_ && init_grids_) updateCandidatePoint(pv);
        if (update_enable_) {
          new_points_num_++;
          all_points_num_++;
          if (update_enable_) temp_points_.push_back(pv);

          if (new_points_num_ > update_size_threshold_) {
            if (update_enable_) initPlane(temp_points_, plane_ptr_, new_points_num_);
            new_points_num_ = 0;
          }
          if (all_points_num_ >= max_points_size_) {
            update_enable_ = false;
          }
        } else {
          // Refit path for a cell whose point budget is exhausted and whose updates are off.
          if (max_mature_points_size_ != max_points_size_)  refitPlaneWithOutliers(pv);
          return;
        }
      } else {
        if (layer_ < max_layer_-1) {
          if (temp_points_.size() != 0) {
            std::vector<PointWithCov>().swap(temp_points_);
          }
          int xyz[3] = {
            pv.point[0] > voxel_center_[0] ? 1 : 0,
            pv.point[1] > voxel_center_[1] ? 1 : 0,
            pv.point[2] > voxel_center_[2] ? 1 : 0
          };
          int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
          if (leaves_[leafnum] != nullptr) {
            leaves_[leafnum]->update(pv);
          } else {
            leaves_[leafnum] = new OctoTree(
                max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                max_mature_points_size_,
                planer_threshold_,
                voxel_size_, max_points_per_voxel_, reduction_ratio_, hybrid_metric_enabled_);
            leaves_[leafnum]->layer_point_size_ = layer_point_size_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            leaves_[leafnum]->update(pv);
          }
          if (hybrid_metric_enabled_ && init_grids_) clearCandidatePoints();
        } else {
          if (update_enable_) {
            new_points_num_++;
            all_points_num_++;
            if (update_enable_) temp_points_.push_back(pv);

            if (new_points_num_ > update_size_threshold_) {
              if (update_enable_) initPlane(temp_points_, plane_ptr_, new_points_num_);
              new_points_num_ = 0;
            }
            if (all_points_num_ >= max_points_size_) update_enable_ = false;
          }
          // At the deepest layer the candidates may never have been seeded, since that
// only happens on a successful plane fit; check before updating.
          if (hybrid_metric_enabled_){
            if (init_grids_) updateCandidatePoint(pv);
            else initCandidatePoints();
          } 
        }
      }
    }
  }
};

double computeMotionThreshold(const Eigen::Quaterniond &rot, const Vec3 &pos, const double max_range_motion, const Vec3 query);


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
                   tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map);

void updateVoxelMap(const std::vector<PointWithCov> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size,
                    const int max_mature_points_size,
                    const float planer_threshold,
                    const int max_points_per_voxel,
                    const int reduction_ratio,
                    const double hybrid_metric_enabled,
                    tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map);

void removePointsFarFromLocation(const Vec3 &pos,
                                 tsl::robin_map<VoxelKey, OctoTree *, VoxelHash> &feat_map,
                                 const double max_distance,
                                 std::vector<VoxelKey> *removed = nullptr);

// Nearest-neighbour search.

Mat3 computeBodyCovariance(Eigen::Vector3d &pb, const float range_inc, const float degree_inc);

}  // namespace genz_lio
