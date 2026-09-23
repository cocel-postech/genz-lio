// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include <Eigen/Core>

#include <cmath>

namespace genz_lio {

template <typename T>
Eigen::Matrix<T, 3, 3> skewSymmetric(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> m;
    m << T(0), -v[2], v[1],
         v[2], T(0), -v[0],
        -v[1], v[0], T(0);
    return m;
}

/// Exponential map of a rotation vector.
template <typename T>
Eigen::Matrix<T, 3, 3> expSO3(const Eigen::Matrix<T, 3, 1> &ang) {
    const T ang_norm = ang.norm();
    const Eigen::Matrix<T, 3, 3> eye = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm <= T(1e-7)) return eye;

    const Eigen::Matrix<T, 3, 3> K = skewSymmetric<T>(ang / ang_norm);
    return eye + std::sin(ang_norm) * K + (T(1) - std::cos(ang_norm)) * K * K;
}

/// Exponential map of an angular velocity integrated over `dt`.
template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> expSO3(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    const T ang_vel_norm = ang_vel.norm();
    const Eigen::Matrix<T, 3, 3> eye = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_vel_norm <= T(1e-7)) return eye;

    const Eigen::Matrix<T, 3, 3> K = skewSymmetric<T>(ang_vel / ang_vel_norm);
    const T r_ang = ang_vel_norm * dt;
    return eye + std::sin(r_ang) * K + (T(1) - std::cos(r_ang)) * K * K;
}

/// Logarithm of a rotation matrix.
template <typename T>
Eigen::Matrix<T, 3, 1> logSO3(const Eigen::Matrix<T, 3, 3> &R) {
    const T theta = (R.trace() > T(3) - T(1e-6)) ? T(0) : std::acos(T(0.5) * (R.trace() - T(1)));
    const Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < T(1e-3)) ? (T(0.5) * K) : (T(0.5) * theta / std::sin(theta) * K);
}

}  // namespace genz_lio
