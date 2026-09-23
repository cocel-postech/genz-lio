// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "StateTransforms.hpp"

#include "SO3Math.hpp"

namespace genz_lio {

Mat3 transformCovarianceToWorld(const Vec3 &point_lidar, const EsekfState &kf,
                                const Mat3 &cov_lidar) {
    const state_ikfom state = kf.get_x();
    const auto P = kf.get_P();

    // LiDAR frame -> IMU body frame, absorbing the extrinsic's own uncertainty.
    const Mat3 lidar_cross = skewSymmetric<double>(point_lidar);
    const Mat3 extrinsic_rot_var = P.block<3, 3>(6, 6);
    const Mat3 extrinsic_t_var = P.block<3, 3>(9, 9);

    const Mat3 cov_body =
        state.offset_R_L_I * cov_lidar * state.offset_R_L_I.conjugate() +
        state.offset_R_L_I * (-lidar_cross) * extrinsic_rot_var * (-lidar_cross).transpose() *
            state.offset_R_L_I.conjugate() +
        extrinsic_t_var;

    // IMU body frame -> world frame, absorbing the pose uncertainty.
    const Vec3 point_body = state.offset_R_L_I * point_lidar + state.offset_T_L_I;
    const Mat3 body_cross = skewSymmetric<double>(point_body);
    const Mat3 rot_var = P.block<3, 3>(3, 3);
    const Mat3 t_var = P.block<3, 3>(0, 0);

    return state.rot * cov_body * state.rot.conjugate() +
           state.rot * (-body_cross) * rot_var * (-body_cross).transpose() * state.rot.conjugate() +
           t_var;
}

}  // namespace genz_lio
