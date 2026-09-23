#include "core/ImuProcessor.hpp"
#include "Check.hpp"
#include <algorithm>
#include <cstdio>

using namespace genz_lio;

namespace {
EsekfState filter() {
    EsekfState kf;
    double limit[23]; std::fill(limit, limit + 23, 0.001);
    kf.init_dyn_share(getF, dfDx, dfDw,
        [](state_ikfom &, esekfom::dyn_share_datastruct<double> &d) { d.valid = false; },
        1, limit);
    return kf;
}
MeasurementBundle bundle(double begin, const Vec3 &acc, const Vec3 &gyr) {
    MeasurementBundle b;
    b.scan_begin_time = begin; b.scan_end_time = begin + 0.1;
    for (int i = 0; i <= 20; ++i) b.imu.push_back({begin + 0.005 * i, acc, gyr});
    Point p; p.position = Vec3(5, 1, 0.5); p.time_offset = 0.08f;
    b.scan.push_back(p);
    return b;
}
void configuredBiasNoiseSurvivesInitializationAndReset() {
    for (double q : {1e-6, 1e-2}) {
        ImuProcessor imu;
        const Vec3 qg(q, 2*q, 3*q), qa(4*q, 5*q, 6*q);
        imu.setGyrBiasCov(qg); imu.setAccBiasCov(qa);
        for (int repetition = 0; repetition < 2; ++repetition) {
            if (repetition) imu.reset();
            auto kf = filter(); PointCloud out;
            imu.process(bundle(10, Vec3(0, 0, 9.81), Vec3::Zero()), kf, out);
            const auto before = kf.get_P();
            imu.process(bundle(10.1, Vec3(0, 0, 9.81), Vec3::Zero()), kf, out);
            // IKFoM discretizes this random walk as sum(dt^2 * Q).
            const double time_factor = 20 * 0.005 * 0.005;
            for (int axis = 0; axis < 3; ++axis) {
                GENZ_CHECK(std::abs(kf.get_P()(15+axis,15+axis) - before(15+axis,15+axis)
                                    - time_factor*qg[axis]) < 1e-12);
                GENZ_CHECK(std::abs(kf.get_P()(18+axis,18+axis) - before(18+axis,18+axis)
                                    - time_factor*qa[axis]) < 1e-12);
            }
        }
    }
}
void resetBehavesLikeANewProcessor() {
    ImuProcessor reused, fresh;
    for (auto *imu : {&reused, &fresh}) {
        imu->setAccCov(Vec3::Constant(0.2)); imu->setGyrCov(Vec3::Constant(0.05));
        imu->setAccBiasCov(Vec3::Constant(0.0043)); imu->setGyrBiasCov(Vec3::Constant(0.000266));
        imu->setExtrinsic(Vec3(0.1,-0.2,0.3), Eigen::AngleAxisd(0.2,Vec3::UnitY()).toRotationMatrix());
    }
    auto old = filter(); PointCloud unused;
    reused.process(bundle(100, Vec3(9.81,0,0), Vec3::Zero()), old, unused);
    reused.process(bundle(100.1, Vec3(9.81,0,0), Vec3(0,0,0.5)), old, unused);
    reused.reset();
    GENZ_CHECK(!reused.initialized());
    GENZ_CHECK((reused.initialGravityAlignment()-Mat3::Identity()).norm() < 1e-12);
    auto a=filter(), b=filter(); PointCloud ca,cb;
    for (int i=0;i<5;++i) {
        const auto input=bundle(10+0.1*i, Vec3(0,0,9.81), i?Vec3(0,0,0.5):Vec3::Zero());
        reused.process(input,a,ca); fresh.process(input,b,cb);
        GENZ_CHECK(reused.firstScanTime()==fresh.firstScanTime());
        GENZ_CHECK((a.get_P()-b.get_P()).norm()<1e-12);
        auto sa=a.get_x(), sb=b.get_x(); Eigen::Matrix<double,23,1> delta;
        sa.boxminus(delta,sb); GENZ_CHECK(delta.norm()<1e-12);
        GENZ_CHECK(ca.size()==cb.size());
        for (size_t j=0;j<ca.size();++j) GENZ_CHECK((ca[j].position-cb[j].position).norm()<1e-12);
    }
}
}
int main() {
    configuredBiasNoiseSurvivesInitializationAndReset();
    resetBehavesLikeANewProcessor();
    std::puts("IMU configuration and reset ok");
}
