#include "core/ImuProcessor.hpp"
#include "Check.hpp"
#include <algorithm>
#include <cstdio>

using namespace genz_lio;
namespace {
EsekfState filter() {
    EsekfState kf;
    double limit[23]; std::fill(limit,limit+23,0.001);
    kf.init_dyn_share(getF,dfDx,dfDw,
        [](state_ikfom&,esekfom::dyn_share_datastruct<double>& d){d.valid=false;},1,limit);
    return kf;
}
MeasurementBundle bundle(double begin) {
    MeasurementBundle b; b.scan_begin_time=begin; b.scan_end_time=begin+0.1;
    for (int i=0;i<=20;++i) b.imu.push_back({begin+0.005*i,Vec3(0,0,9.81),Vec3::Zero()});
    return b;
}
void checkMotion(const Vec3& velocity,double rate,bool extrinsics,
                 const std::vector<float>& times) {
    ImuProcessor imu; auto kf=filter(); PointCloud out;
    const Mat3 Rli=extrinsics?Eigen::AngleAxisd(0.3,Vec3::UnitY()).toRotationMatrix():Mat3::Identity();
    const Vec3 tli=extrinsics?Vec3(0.4,-0.1,0.2):Vec3::Zero();
    imu.setExtrinsic(tli,Rli);
    imu.process(bundle(10),kf,out);
    auto state=kf.get_x(); state.vel=velocity; state.bg=Vec3(0,0,-rate); kf.change_x(state);
    auto scan=bundle(10.1);
    for(float t:times) {Point p; p.position=Vec3(5,1,0.5);p.time_offset=t;scan.scan.push_back(p);}
    imu.process(scan,kf,out); GENZ_CHECK(out.size()==times.size());
    const Mat3 Rend=Eigen::AngleAxisd(rate*0.1,Vec3::UnitZ()).toRotationMatrix();
    auto sorted=times; std::sort(sorted.begin(),sorted.end());
    for(size_t i=0;i<out.size();++i) {
        const double t=sorted[i];
        const Mat3 Ri=Eigen::AngleAxisd(rate*t,Vec3::UnitZ()).toRotationMatrix();
        const Vec3 expected=Rli.transpose()*(Rend.transpose()*(Ri*(Rli*Vec3(5,1,0.5)+tli)
                              +velocity*(t-0.1))-tli);
        // The state manifold fixes gravity at 9.809 while this IMU uses 9.81;
        // its vertical drift over 0.1 s is below this geometric tolerance.
        GENZ_CHECK((out[i].position-expected).norm()<1e-5);
    }
}
}
int main() {
    for(bool extrinsics:{false,true}) {
        for(const auto& times: {std::vector<float>{0,0.005f,0.03f,0.099f},
                               std::vector<float>{0.075f},
                               std::vector<float>{0.09f,0,0,0.05f}}) {
            checkMotion(Vec3::Zero(),1,extrinsics,times);
            checkMotion(Vec3(0.3,-0.2,0),0,extrinsics,times);
            checkMotion(Vec3(0.3,-0.2,0),1,extrinsics,times);
        }
    }
    std::puts("deskew analytic motion checks ok");
}
