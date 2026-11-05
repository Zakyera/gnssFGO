// Read-only equation audit. No nonlinear optimizer or source replay.
#include "../Checkpoint.h"
#include <iostream>
using namespace gtsam;
int main(int argc,char**argv){try{
  cpv36::require(argc==3,"checkpoint and new directory required");
  auto s=cpv36::load(argv[1]);std::filesystem::path out=argv[2];
  cpv36::require(std::filesystem::create_directories(out),"inspection output exists");
  auto vv=s.theta.localCoordinates(s.estimate);Vector delta=vv.vector();
  cpv36::require(delta.size()+1==s.cached.rows(),"equation/state dimensions");
  std::ofstream f(out/"equations.txt");f<<std::setprecision(17);
  for(const auto& item:std::vector<std::pair<std::string,Matrix>>{{"cached",s.cached},{"tree",s.tree}}){
    int n=delta.size();auto H=item.second.topLeftCorner(n,n);Vector eta=item.second.topRightCorner(n,1),r=H*delta-eta;
    double scaled=0;for(int i=0;i<n;++i)scaled=std::max(scaled,std::abs(r[i])/std::sqrt(std::max(1e-30,std::abs(H(i,i)))));
    f<<item.first<<"_linear_residual_norm,"<<r.norm()<<'\n'
     <<item.first<<"_scaled_linear_residual_max,"<<scaled<<'\n'
     <<item.first<<"_backward_relative,"<<r.norm()/(1+eta.norm()+H.norm()*delta.norm())<<'\n';
  }
  int n=delta.size();
  f<<"H_cache_tree_relative,"<<(s.cached.topLeftCorner(n,n)-s.tree.topLeftCorner(n,n)).norm()/(1+s.cached.topLeftCorner(n,n).norm())<<'\n';
  f<<"eta_cache_tree_relative,"<<(s.cached.topRightCorner(n,1)-s.tree.topRightCorner(n,1)).norm()/(1+s.cached.topRightCorner(n,1).norm())<<'\n';
  f<<"constant_difference,"<<s.cached(n,n)-s.tree(n,n)<<'\n';
  std::ofstream types(out/"native_factor_map.csv");types<<"packed_index,slot,type,keys\n";
  size_t idx=0;for(size_t i=0;i<s.graph.size();++i)if(auto factor=s.graph[i]){
    types<<idx++<<','<<i<<','<<typeid(*factor).name()<<','<<factor->keys().size()<<'\n';
  }
  std::ofstream done(out/"complete.txt");done<<"read-only linear equation inspection; not nonlinear stationarity\n";
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}

