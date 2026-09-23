#include "core/HybridMetricUpdate.hpp"
#include "Check.hpp"
#include <cstdio>
#include <limits>
using namespace genz_lio;

int matches(const Vec3& query,double distance,bool correction,
            const Eigen::Quaterniond& rotation,const Vec3& translation,double range_cap=100) {
    Config config; config.mapping.voxel_size=100; config.mapping.max_layer=1;
    config.hybrid_metric.sigma_num=1;
    config.hybrid_metric.adaptive_threshold.initial_threshold=0.1;
    config.hybrid_metric.adaptive_threshold.max_range_motion=range_cap;
    PointWithCov p; p.point=p.point_world=query+Vec3(0,distance,0);
    p.cov=p.cov_lidar=Mat3::Identity()*1e-4;
    VoxelMapType map; buildVoxelMap({p},100,1,{5},100,100,0.01f,64,4,true,map);
    PointCloud scan(1);scan[0].position=query;
    std::vector<Mat3> covariance(1,Mat3::Identity()*1e-4);EsekfState kf;
    HybridMetricUpdate update(config);update.setFrame(&scan,&covariance,&map,&kf);
    if(correction) update.setPoseDeviation(rotation,translation);
    auto state=kf.get_x();esekfom::dyn_share_datastruct<double> d;d.valid=true;
    update(state,d); const int n=update.stats().point_matches;
    for(auto& e:map)delete e.second;
    return n;
}
int main() {
    const auto I=Eigen::Quaterniond::Identity(); const Vec3 zero=Vec3::Zero();
    GENZ_CHECK(matches(Vec3(5,5,5),0.05,false,I,zero)==1);
    GENZ_CHECK(matches(Vec3(5,5,5),0.3,false,I,zero)==0);
    GENZ_CHECK(matches(Vec3(5,5,5),0.3,true,I,Vec3(1,0,0))==1);
    // Zero correction keeps the startup noise floor; it must not erase all matches.
    GENZ_CHECK(matches(Vec3(5,5,5),0.05,true,I,zero)==1);
    const Eigen::Quaterniond turn(Eigen::AngleAxisd(0.04,Vec3::UnitZ()));
    GENZ_CHECK(matches(Vec3(2,2,2),0.3,true,turn,zero)==0);
    GENZ_CHECK(matches(Vec3(20,2,2),0.3,true,turn,zero)==1);
    GENZ_CHECK(matches(Vec3(20,2,2),0.3,true,turn,zero,5)==0);
    GENZ_CHECK(matches(Vec3(5,5,5),0.05,true,I,Vec3(std::numeric_limits<double>::quiet_NaN(),0,0))==1);
    std::puts("adaptive gate startup, stationary, motion and range checks ok");
}
