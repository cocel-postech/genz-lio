#include "core/StateIkfom.hpp"
#include "core/FilterUpdate.hpp"
#include "Check.hpp"
#include <algorithm>
#include <cstdio>
#include <limits>

using namespace genz_lio;
void invalidUpdatesRestoreStateAndCovariance() {
    for (int fault=0;fault<9;++fault) {
        EsekfState kf;
        double limit[23]; std::fill(limit,limit+23,1e-10);
        kf.init_dyn_share(getF,dfDx,dfDw,
            [&](state_ikfom& s, esekfom::dyn_share_datastruct<double>& d) {
                const double nan=std::numeric_limits<double>::quiet_NaN();
                // Fault injection isolates every field checked by rollback.
                d.valid=false;
                auto P=kf.get_P(); P(0,0)=0.25; kf.change_P(P);
                switch(fault) {
                    case 0: s.pos.x()=nan; break;
                    case 1: s.vel.x()=nan; break;
                    case 2: s.bg.x()=nan; break;
                    case 3: s.ba.x()=nan; break;
                    case 4: s.rot.coeffs()[0]=nan; break;
                    case 5: s.offset_R_L_I.coeffs()[0]=nan; break;
                    case 6: s.offset_T_L_I.x()=nan; break;
                    case 7: s.grav=S2(Eigen::Vector3d(nan,0,1)); break;
                    case 8: P(0,0)=nan; kf.change_P(P); break;
                }
            },1,limit);
        auto before=kf.get_x(); const auto before_P=kf.get_P();
        GENZ_CHECK(!updateWithRollback(kf));
        GENZ_CHECK(finiteState(kf.get_x()));
        Eigen::Matrix<double,23,1> delta; auto after=kf.get_x(); after.boxminus(delta,before);
        GENZ_CHECK(delta.norm()<1e-12);
        GENZ_CHECK((kf.get_P()-before_P).norm()<1e-12);
    }
}
int main() {
    invalidUpdatesRestoreStateAndCovariance();
    // One informative x observation and any number of zero-information rows
    // must give the same posterior on both sides of the 23-row solver branch.
    for (double variance : {0.01, 1.0, 100.0}) {
        for (int rows : {1, 22, 23, 24}) {
            EsekfState kf;
            double limit[23]; std::fill(limit, limit+23, 1e-10);
            kf.init_dyn_share(getF,dfDx,dfDw,
                [=](state_ikfom& s, esekfom::dyn_share_datastruct<double>& d) {
                    d.valid=true;
                    d.h_x=Eigen::MatrixXd::Zero(rows,12);
                    d.h=Eigen::VectorXd::Zero(rows);
                    d.R=Eigen::VectorXd::Constant(rows,1.0/variance);
                    d.h_x(0,0)=1; d.h(0)=1-s.pos.x();
                },2,limit);
            GENZ_CHECK(updateWithRollback(kf));
            const double gain=1/(1+variance);
            GENZ_CHECK(std::abs(kf.get_x().pos.x()-gain)<1e-10);
            GENZ_CHECK(std::abs(kf.get_P()(0,0)-(1-gain))<1e-10);
            GENZ_CHECK(kf.get_P().allFinite());
            GENZ_CHECK((kf.get_P()-kf.get_P().transpose()).norm()<1e-10);
        }
    }
    std::puts("filter row-count and variance checks ok");
}
