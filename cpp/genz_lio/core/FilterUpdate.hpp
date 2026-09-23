// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once
#include "StateIkfom.hpp"

namespace genz_lio {

inline bool finiteState(const state_ikfom &s) {
    return s.pos.allFinite() && s.vel.allFinite() && s.bg.allFinite() && s.ba.allFinite() &&
           s.rot.coeffs().allFinite() && s.offset_R_L_I.coeffs().allFinite() &&
           s.offset_T_L_I.allFinite() && s.grav.get_vect().allFinite();
}

/// An update is atomic with respect to the prediction: an invalid state or
/// covariance restores both. The caller must skip map insertion on failure.
inline bool updateWithRollback(EsekfState &kf) {
    auto predicted = kf.get_x();
    auto predicted_covariance = kf.get_P();
    kf.update_iterated_dyn_share_diagonal();
    if (finiteState(kf.get_x()) && kf.get_P().allFinite()) return true;
    kf.change_x(predicted);
    kf.change_P(predicted_covariance);
    return false;
}

}  // namespace genz_lio
