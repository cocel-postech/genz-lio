#include "3rdparty/IKFoM_toolkit/esekfom/reference_product.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <limits>
void check(bool x){if(!x)throw std::runtime_error("reference product check failed");}
template<int Rows> void checkRows() {
 for (int depth : {0,1,22,23,24,47,48,100,447,448,671,672,759,760,761,1000,2000,2682,4096,10000}) {
  Eigen::Matrix<double,Rows,Eigen::Dynamic> a(Rows,depth);
  Eigen::Matrix<double,Eigen::Dynamic,12> b(depth,12);
  for(int k=0;k<depth;++k){
   for(int r=0;r<Rows;++r)a(r,k)=std::sin(.37*(r+1)*(k+1));
   for(int c=0;c<12;++c)b(k,c)=std::cos(.29*(c+1)*(k+1));
  }
  Eigen::setCpuCacheSizes(49152,2097152,31457280);
  const Eigen::Matrix<double,Rows,12> expected=a*b;
  const auto p=esekfom::detail::referenceObservationProduct(a,b);
  Eigen::setCpuCacheSizes(32768,4194304,31457280);
  const auto e=esekfom::detail::referenceObservationProduct(a,b);
  if(!(p.array()==expected.array()).all()){std::cerr<<Rows<<","<<depth<<" original mismatch "<<(p-expected).cwiseAbs().maxCoeff()<<"\n";check(false);}
  check((p.array()==e.array()).all());
  check(Eigen::l1CacheSize()==32768 && Eigen::l2CacheSize()==4194304);
  for(int r=0;r<Rows;++r)for(int c=0;c<12;++c){
   long double ref=0,scale=0;
   for(int k=0;k<depth;++k){long double term=(long double)a(r,k)*b(k,c);ref+=term;scale+=std::abs(term);}
   check(std::abs((long double)p(r,c)-ref)<=64*std::numeric_limits<double>::epsilon()*std::max(1.L,scale));
  }
 }
}
int main(){checkRows<12>();checkRows<23>();std::cout<<"P/E cache independence, reference compatibility, global cache preservation and long-double checks passed\n";}
