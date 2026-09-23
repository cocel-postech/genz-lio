// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// State manifold and process model for the error-state iterated Kalman filter
// (paper Sec. V-A). The 23-dimensional error state carries the IMU pose,
// velocity, biases, gravity direction and the LiDAR-IMU extrinsic.
#pragma once

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

#include <Eigen/Core>

namespace genz_lio {

using Vect3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;
using S2 = MTK::S2<double, 98090, 10000, 1>;

MTK_BUILD_MANIFOLD(state_ikfom,
((Vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((Vect3, offset_T_L_I))
((Vect3, vel))
((Vect3, bg))
((Vect3, ba))
((S2, grav))
);

MTK_BUILD_MANIFOLD(input_ikfom,
((Vect3, acc))
((Vect3, gyro))
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((Vect3, ng))
((Vect3, na))
((Vect3, nbg))
((Vect3, nba))
);

inline MTK::get_cov<process_noise_ikfom>::type processNoiseCov() {
    // Declared with the concrete type: `auto` would bind the lazy Eigen
    // expression rather than a matrix, which setDiagonal cannot take.
    MTK::get_cov<process_noise_ikfom>::type cov =
        MTK::get_cov<process_noise_ikfom>::type::Zero();
    MTK::setDiagonal<process_noise_ikfom, Vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);
    MTK::setDiagonal<process_noise_ikfom, Vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);
    MTK::setDiagonal<process_noise_ikfom, Vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001);
    MTK::setDiagonal<process_noise_ikfom, Vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);
    return cov;
}

/// Continuous-time state derivative.
inline Eigen::Matrix<double, 24, 1> getF(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
    Vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    const Vect3 a_inertial = s.rot * (in.acc - s.ba);
    for (int i = 0; i < 3; ++i) {
        res(i) = s.vel[i];
        res(i + 3) = omega[i];
        res(i + 12) = a_inertial[i] + s.grav[i];
    }
    return res;
}

/// Jacobian of the process model with respect to the error state.
inline Eigen::Matrix<double, 24, 23> dfDx(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

    Vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    Vect3 omega;
    in.gyro.boxminus(omega, s.bg);

    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
    cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();

    Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 2>(12, 21) = grav_matrix;

    cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
    return cov;
}

/// Jacobian of the process model with respect to the process noise.
inline Eigen::Matrix<double, 24, 12> dfDw(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
    cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
    return cov;
}

using EsekfState = esekfom::esekf<state_ikfom, 12, input_ikfom>;

}  // namespace genz_lio
