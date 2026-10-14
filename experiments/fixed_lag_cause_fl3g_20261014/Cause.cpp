#define main FL3G_UNUSED_AUDIT_MAIN
#include "../fixed_lag_failure_fl3a_20261013/SnapshotAudit.cpp"
#undef main

Vector Residual(const NonlinearFactor::shared_ptr& f,const Values& x) {
  if(auto* m=dynamic_cast<fl2a::AnchoredMarginal*>(f.get()))
    return m->A()*m->delta(x).vector(f->keys())-m->b();
  if(auto* n=dynamic_cast<NoiseModelFactor*>(f.get()))return n->whitenedError(x);
  throw std::runtime_error("unrecognized residual");
}
VectorValues Scaled(VectorValues d,double scale){for(auto& [k,x]:d)x*=scale;return d;}
std::vector<std::string> Groups(const fs::path& root,size_t n) {
  std::vector<std::string> result(n);std::ifstream in(root/"factors.txt");std::string line;
  while(std::getline(in,line)){
    std::istringstream row(line);size_t i;std::string kind;row>>i>>kind;
    if(kind=="source")row>>kind;
    result.at(i)=kind;
  }return result;
}
int main(int argc,char** argv) {
 try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  if(argc!=4)throw std::runtime_error("usage: cause SNAPSHOT FL3D_DIAGNOSTICS NEW_OUTPUT");
  fs::path root(argv[1]),reference(argv[2]),out(argv[3]);
  if(!fs::create_directory(out))throw std::runtime_error("refuse output reuse");
  const auto graph=LoadGraph(root);const auto groups=Groups(root,graph.size());
  const auto initial=LoadValues(root/"initial.values"),v=LoadValues(reference/"last_accepted.values");
  const Ordering ordering(v.keys());
  auto stream=[&](const char* name){std::ofstream f(out/name);f<<std::setprecision(17);return f;};
  auto checks=stream("checks.txt");
  checks<<"initial_cost "<<graph.error(initial)<<"\ninitial_gradient "<<fl2a::CheckedGradient(graph,initial)
    <<"\nreference_cost "<<graph.error(v)<<"\nreference_gradient "<<fl2a::CheckedGradient(graph,v)<<'\n';
  if(std::abs(graph.error(initial)-2794137.3456364814)>1e-7 ||
     std::abs(fl2a::CheckedGradient(graph,initial)-1.5863889609968425)>1e-10 ||
     std::abs(fl2a::CheckedGradient(graph,v)-7.977808819599585e-6)>1e-9)
    throw std::runtime_error("snapshot reproduction fails");
  std::map<int,Vector> coord;
  for(int i=111;i<=116;++i)coord[i]=v.localCoordinates(LoadValues(reference/("trial_"+std::to_string(i)+".values"))).vector(ordering);
  auto steps=stream("late_steps.csv");steps<<"step,norm,ratio_to_previous,cosine_to_previous\n";
  for(int i=113;i<=116;++i){Vector a=coord.at(i-1)-coord.at(i-2),b=coord.at(i)-coord.at(i-1);
    steps<<i<<','<<b.norm()<<','<<b.norm()/a.norm()<<','<<a.dot(b)/(a.norm()*b.norm())<<'\n';}
  Vector direction=coord.at(115)-coord.at(114);direction.normalize();
  VectorValues d;size_t offset=0;
  for(auto k:ordering){const auto n=v.at(k).dim();d.insert(k,direction.segment(offset,n));offset+=n;}
  MatrixOut(out/"mode",direction);
  auto keys=stream("ordering.txt");for(auto k:ordering)keys<<DefaultKeyFormatter(k)<<' '<<v.at(k).dim()<<'\n';
  auto offsets=stream("marginal_offsets.csv");offsets<<"key,rotation_or_vector_norm,translation_norm\n";
  for(const auto& f:graph)if(auto* m=dynamic_cast<fl2a::AnchoredMarginal*>(f.get())){
    const auto dd=m->delta(v);
    for(auto k:f->keys()){const auto& a=dd.at(k);offsets<<DefaultKeyFormatter(k)<<','<<a.head(3).norm()<<','<<(a.size()==6?a.tail(3).norm():0.)<<'\n';}
  }
  auto table=stream("curvature.csv");
  table<<"h,slot,group,residual_cost,gn_curvature,residual_curvature,true_curvature,fd_jd_error,analytic_jd_norm\n";
  for(double h:{.001,.0003,.0001}) {
    const auto plus=v.retract(Scaled(d,h)),minus=v.retract(Scaled(d,-h));
    for(size_t i=0;i<graph.size();++i)if(graph[i]) {
      const auto& f=graph[i];Vector r=Residual(f,v),rp=Residual(f,plus),rm=Residual(f,minus);
      const auto ab=f->linearize(v)->jacobian();const Vector jd=ab.first*d.vector(f->keys());
      const Vector fd=(rp-rm)/(2*h),second=(rp+rm-2*r)/(h*h);
      const double gn=jd.squaredNorm(),extra=r.dot(second);
      if(!r.allFinite()||!jd.allFinite()||!second.allFinite())throw std::runtime_error("nonfinite curvature");
      table<<h<<','<<i<<','<<groups.at(i)<<','<<.5*r.squaredNorm()<<','<<gn<<','<<extra<<','<<gn+extra<<','<<(jd-fd).norm()<<','<<jd.norm()<<'\n';
    }
  }
  std::cout<<"FL3G_CAUSE_DIAGNOSTIC_COMPLETE\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
