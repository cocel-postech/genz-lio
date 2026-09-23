#include "core/VoxelMap.hpp"
#include "Check.hpp"
#include <cstdio>
using namespace genz_lio;
PointWithCov point(const Vec3& xyz) {
    PointWithCov p; p.point=xyz; p.cov=Mat3::Identity()*1e-4; return p;
}
void checkBatchStatistics(const OctoTree& cell) {
    Vec3 mean=Vec3::Zero(); Mat3 second=Mat3::Zero();
    for (const auto& p:cell.temp_points_) {mean+=p.point;second+=p.point*p.point.transpose();}
    mean/=cell.temp_points_.size(); second/=cell.temp_points_.size();
    const Mat3 covariance=second-mean*mean.transpose();
    GENZ_CHECK((cell.plane_ptr_->center-mean).norm()<1e-12);
    GENZ_CHECK((cell.plane_ptr_->covariance-covariance).norm()<1e-12);
}
int main() {
    OctoTree cell(1,0,{5},12,40,0.01f,1.0,64,4,true);
    cell.voxel_center_[0]=cell.voxel_center_[1]=cell.voxel_center_[2]=0.5;
    cell.quater_length_=0.25;
    // Reach the ordinary update budget through the public update path.
    for(int i=0;i<12;++i) cell.update(point(Vec3(0.1+0.1*(i%3),0.1+0.1*((i/3)%2),0)));
    GENZ_CHECK(!cell.update_enable_);
    GENZ_CHECK(cell.new_points_num_==0);
    checkBatchStatistics(cell);
    for(int batch=0;batch<2;++batch) {
        for(int i=0;i<6;++i) cell.refitPlaneWithOutliers(point(Vec3(1+batch+0.1*i,2+0.03*i,3+0.2*i)));
        GENZ_CHECK(cell.new_points_num_==0);
        checkBatchStatistics(cell);
    }
    std::puts("mature plane incremental statistics match batch fit");
}
