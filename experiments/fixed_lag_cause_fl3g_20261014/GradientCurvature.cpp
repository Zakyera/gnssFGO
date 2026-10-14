#define main FL3G_UNUSED_AUDIT_MAIN
#include "../fixed_lag_failure_fl3a_20261013/SnapshotAudit.cpp"
#undef main

Vector ResidualG(const NonlinearFactor::shared_ptr& f,const Values& x) {
  if(auto* m=dynamic_cast<fl2a::AnchoredMarginal*>(f.get()))
    return m->A()*m->delta(x).vector(f->keys())-m->b();
  if(auto* n=dynamic_cast<NoiseModelFactor*>(f.get()))return n->whitenedError(x);
  throw std::runtime_error("unrecognized residual");
}
int main(int argc,char** argv) {
 try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  if(argc!=5)throw std::runtime_error("usage: gradient_curvature SNAPSHOT VALUES MODE OUTPUT");
  if(fs::exists(argv[4]))throw std::runtime_error("refuse overwrite");
  const auto graph=LoadGraph(argv[1]);const auto v=LoadValues(argv[2]);
  if(std::abs(fl2a::CheckedGradient(graph,v)-7.977808819599585e-6)>1e-9)throw std::runtime_error("reference mismatch");
  const Vector mode=LoadMatrix(argv[3]);VectorValues d;size_t off=0;
  for(auto k:v.keys()){auto n=v.at(k).dim();d.insert(k,mode.segment(off,n));off+=n;}
  std::ofstream out(argv[4]);out<<std::setprecision(17)<<"h,slot,gn_curvature,true_curvature,residual_curvature\n";
  for(double h:{.001,.0003,.0001}) {
    auto dp=d,dm=d;for(auto& [k,x]:dp)x*=h;for(auto& [k,x]:dm)x*=-h;
    auto vp=v.retract(dp),vm=v.retract(dm);
    for(size_t i=0;i<graph.size();++i)if(graph[i]) {
      const auto& f=graph[i];const Vector df=d.vector(f->keys());
      const Vector jd=f->linearize(v)->jacobian().first*df;
      const double gp=ResidualG(f,vp).dot(f->linearize(vp)->jacobian().first*df);
      const double gm=ResidualG(f,vm).dot(f->linearize(vm)->jacobian().first*df);
      const double gn=jd.squaredNorm(),real=(gp-gm)/(2*h);
      out<<h<<','<<i<<','<<gn<<','<<real<<','<<real-gn<<'\n';
    }
  }
  std::cout<<"FL3G_GRADIENT_CURVATURE_COMPLETE\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
