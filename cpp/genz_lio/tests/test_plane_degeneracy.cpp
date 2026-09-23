#include "core/VoxelMap.hpp"
#include "core/VoxelPrunedSearch.hpp"
#include "Check.hpp"
#include <cstdio>
using namespace genz_lio;
PointWithCov point(const Vec3& xyz) {
    PointWithCov p; p.point=p.point_world=xyz;p.cov=p.cov_lidar=Mat3::Identity()*1e-4;return p;
}
void rejectsUnobservableNormal(const std::vector<PointWithCov>& points) {
    OctoTree cell(1,0,{5},100,100,1.0f,4.0,64,4,true);
    cell.initPlane(points,cell.plane_ptr_,points.size());
    GENZ_CHECK(!cell.plane_ptr_->is_plane);
    GENZ_CHECK(cell.plane_ptr_->plane_cov.allFinite());
}
int main() {
    std::vector<PointWithCov> line;
    for(int i=0;i<8;++i) line.push_back(point(Vec3(i,0,0)));
    rejectsUnobservableNormal(line);
    rejectsUnobservableNormal(std::vector<PointWithCov>(8,point(Vec3(1,2,3))));
    std::vector<PointWithCov> isotropic;
    for(int axis=0;axis<3;++axis) for(double sign:{-1.,1.})
        isotropic.push_back(point(sign*Vec3::Unit(axis)));
    rejectsUnobservableNormal(isotropic);

    OctoTree planar(1,0,{5},100,100,0.01f,4.0,64,4,true);
    std::vector<PointWithCov> plane;
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) plane.push_back(point(Vec3(i*0.1,j*0.1,1)));
    planar.initPlane(plane,planar.plane_ptr_,plane.size());
    GENZ_CHECK(planar.plane_ptr_->is_plane);
    GENZ_CHECK(std::abs(planar.plane_ptr_->normal.z())>1-1e-12);
    GENZ_CHECK(planar.plane_ptr_->plane_cov.allFinite());
    Eigen::SelfAdjointEigenSolver<Mat6> es(planar.plane_ptr_->plane_cov);
    GENZ_CHECK(es.eigenvalues().minCoeff()>-1e-12);

    // Rejecting a line as a plane must retain its measured points for the
    // hybrid metric at a leaf that cannot subdivide further.
    VoxelMapType map; line.clear();
    for(int i=0;i<8;++i) line.push_back(point(Vec3(1+0.1*i,0.5,0.5)));
    buildVoxelMap(line,4,1,{5},100,100,0.01f,64,4,true,map);
    auto* cell=map.begin()->second;
    GENZ_CHECK(!cell->plane_ptr_->is_plane && cell->init_grids_);
    GENZ_CHECK(!cell->grid_points_.empty());
    auto query=point(Vec3(1.01,0.5,0.5));
    bool found=false;double nearest=0.1;int count=0;PointToPointMatch match;
    buildPointResidual(query,cell,0.1,found,nearest,match,count);
    GENZ_CHECK(found);
    for(auto& e:map) delete e.second;
    std::puts("degenerate geometry falls back to point candidates");
}
