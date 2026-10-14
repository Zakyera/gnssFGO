#define main fl3c_unused_snapshot_main
#include "../fixed_lag_failure_fl3a_20261013/SnapshotAudit.cpp"
#undef main
#include "BoundedGN.h"

struct Changes {
  double p2=0,pm=0,q2=0,qm=0,v2=0,vm=0,w2=0,wm=0,pn=0,qn=0,vn=0,wn=0;
  size_t n=0;
};
Changes Compare(const Values& a,const Values& b) {
  Changes d;
  for(Key k:a.keys())if(Symbol(k).chr()=='x') {
    auto i=Symbol(k).index();const auto x=a.at<Pose3>(k),y=b.at<Pose3>(k);
    const double p=(x.translation()-y.translation()).norm();
    const double q=Rot3::Logmap(x.rotation().between(y.rotation())).norm();
    const double v=(a.at<Vector3>(Symbol('v',i))-b.at<Vector3>(Symbol('v',i))).norm();
    const double w=(a.at<Vector3>(Symbol('w',i))-b.at<Vector3>(Symbol('w',i))).norm();
    ++d.n;d.p2+=p*p;d.q2+=q*q;d.v2+=v*v;d.w2+=w*w;
    d.pm=std::max(d.pm,p);d.qm=std::max(d.qm,q);d.vm=std::max(d.vm,v);d.wm=std::max(d.wm,w);
    if(i==1224){d.pn=p;d.qn=q;d.vn=v;d.wn=w;}
  }return d;
}
void ChangesHeader(std::ostream& f){f<<"p_rms_m,p_max_m,q_rms_rad,q_max_rad,v_rms_mps,v_max_mps,w_rms_radps,w_max_radps,newest_p_m,newest_q_rad,newest_v_mps,newest_w_radps";}
void ChangesOut(std::ostream& f,const Changes& d){f<<std::sqrt(d.p2/d.n)<<','<<d.pm<<','<<std::sqrt(d.q2/d.n)<<','<<d.qm<<','<<std::sqrt(d.v2/d.n)<<','<<d.vm<<','<<std::sqrt(d.w2/d.n)<<','<<d.wm<<','<<d.pn<<','<<d.qn<<','<<d.vn<<','<<d.wn;}
void Save(const fs::path& p,const Values& value) {
  std::ofstream f(p);f<<std::setprecision(17);
  for(Key key:value.keys()){const Symbol s(key);f<<s.chr()<<' '<<s.index();
    if(s.chr()=='x'){auto m=value.at<Pose3>(key).matrix();for(int r=0;r<3;++r)for(int c=0;c<4;++c)f<<' '<<m(r,c);}
    else {auto v=value.at<Vector3>(key);for(int i=0;i<3;++i)f<<' '<<v[i];}f<<'\n';}
}
std::pair<Key,size_t> Worst(const NonlinearFactorGraph& graph,const Values& v) {
  auto linear=graph.linearize(v);auto g=linear->gradientAtZero(),h=linear->hessianDiagonal();double max=-1;std::pair<Key,size_t> result;
  for(const auto& [k,x]:g)for(int j=0;j<x.size();++j){double n=std::abs(x[j])/std::sqrt(std::max(1e-30,h.at(k)[j]));if(n>max){max=n;result={k,j};}}
  return result;
}
int main(int argc,char** argv){try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  if(argc!=3)throw std::runtime_error("usage: fixture SNAPSHOT NEW_OUTPUT");const fs::path root(argv[1]),out(argv[2]);
  if(!fs::create_directory(out))throw std::runtime_error("refuse overwrite");
  const auto graph=LoadGraph(root),frozen=graph;const auto seed=LoadValues(root/"initial.values");
  const auto initialLinear=graph.linearize(seed)->jacobian(Ordering(seed.keys()));
  const double cost=graph.error(seed),gradient=fl2a::CheckedGradient(graph,seed);
  if(cost!=2794137.3456364814||gradient!=1.5863889609968425)throw std::runtime_error("initial identity mismatch");
  std::ofstream trace(out/"trials.csv"),equivalence(out/"first20_equivalence.csv");trace<<std::setprecision(17);equivalence<<std::setprecision(17);
  trace<<"attempt,cost,gradient,worst_key,worst_coordinate,solve_seconds,check_seconds,";ChangesHeader(trace);trace<<'\n';
  equivalence<<"attempt,max_local_coordinate_difference,cost_difference,gradient_difference\n";
  Values step20,previous;size_t equivalent=0;double maxLocal=0,maxCost=0,maxGradient=0;
  const auto begin=fl3c::Clock::now();
  auto result=fl3c::Solve(graph,seed,200,[&](const fl3c::Step& row,const Values& old,const Values& next){
    previous=old;
    if(row.attempt<=20) {
      const auto saved=LoadValues(root/("trial_"+std::to_string(row.attempt)+".values"));double local=0;
      for(const auto& [key,d]:next.localCoordinates(saved))local=std::max(local,d.cwiseAbs().maxCoeff());
      const double ec=std::abs(graph.error(saved)-row.cost),eg=std::abs(fl2a::CheckedGradient(graph,saved)-row.gradient);
      equivalence<<row.attempt<<','<<local<<','<<ec<<','<<eg<<'\n';
      maxLocal=std::max(maxLocal,local);maxCost=std::max(maxCost,ec);maxGradient=std::max(maxGradient,eg);
      if(local>1e-8||ec>1e-7||eg>1e-7)throw std::runtime_error("first20 GN equivalence failed");++equivalent;
      if(row.attempt==20){step20=next;Save(out/"step20.values",next);}
    }
    const auto worst=Worst(graph,next);trace<<row.attempt<<','<<row.cost<<','<<row.gradient<<','<<DefaultKeyFormatter(worst.first)<<','<<worst.second<<','<<row.solveSeconds<<','<<row.checkSeconds<<',';
    ChangesOut(trace,Compare(old,next));trace<<'\n';
  });
  const double seconds=fl3c::Seconds(begin);double solveSeconds=0,checkSeconds=0;
  for(const auto& row:result.steps){solveSeconds+=row.solveSeconds;checkSeconds+=row.checkSeconds;}
  const auto finalLinear=graph.linearize(seed)->jacobian(Ordering(seed.keys()));
  bool unchanged=graph.size()==frozen.size()&&seed.equals(LoadValues(root/"initial.values"),0);
  for(size_t i=0;i<graph.size();++i)unchanged=unchanged&&graph[i]==frozen[i];
  unchanged=unchanged&&(initialLinear.first-finalLinear.first).norm()==0&&(initialLinear.second-finalLinear.second).norm()==0
    &&graph.error(seed)==cost&&fl2a::CheckedGradient(graph,seed)==gradient;
  if(!unchanged)throw std::runtime_error("objective mutated");
  Save(out/"initial.values",seed);Save(out/"final_diagnostic_only.values",result.diagnosticValues);
  std::ofstream changes(out/"state_changes.csv");changes<<std::setprecision(17)<<"comparison,";ChangesHeader(changes);changes<<'\n';
  auto compare=[&](const char* label,const Values& v){changes<<label<<',';ChangesOut(changes,Compare(v,result.diagnosticValues));changes<<'\n';};
  compare("initial_to_final",seed);if(!step20.empty())compare("step20_to_final",step20);if(!previous.empty())compare("last_step",previous);
  std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\n"
    <<"\"passed\":"<<(result.passed?"true":"false")<<",\n\"stop\":\""<<result.stop<<"\",\n"
    <<"\"attempts\":"<<result.attempts<<",\n\"accepted\":"<<result.accepted<<",\n"
    <<"\"initial_cost\":"<<cost<<",\n\"final_cost\":"<<graph.error(result.diagnosticValues)<<",\n"
    <<"\"initial_gradient\":"<<gradient<<",\n\"final_gradient\":"<<result.finalGradient<<",\n"
    <<"\"equivalent_first_steps\":"<<equivalent<<",\n\"max_first20_local_difference\":"<<maxLocal<<",\n"
    <<"\"max_first20_cost_difference\":"<<maxCost<<",\n\"max_first20_gradient_difference\":"<<maxGradient<<",\n"
    <<"\"original_objective_unchanged\":true,\n\"solve_seconds\":"<<solveSeconds<<",\n\"check_seconds\":"<<checkSeconds<<",\n"
    <<"\"total_seconds_with_diagnostics\":"<<seconds<<"\n}\n";
  std::cout<<std::setprecision(17)<<(result.passed?"PASS":"NOT_CONVERGED")<<" attempts="<<result.attempts<<" accepted="<<result.accepted<<" gradient="<<result.finalGradient<<" stop="<<result.stop<<'\n';
  return result.stop.starts_with("exception:")?1:0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
