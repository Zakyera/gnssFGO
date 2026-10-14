// Reuse the exact saved-objective loader already validated by FL3a.
#define main fl3b_unused_snapshot_main
#include "../fixed_lag_failure_fl3a_20261013/SnapshotAudit.cpp"
#undef main
#include "BoundedAdaptive.h"
#include <chrono>
int main(int argc,char** argv){try{
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  if(argc!=3)throw std::runtime_error("usage: fixture SNAPSHOT NEW_OUTPUT");
  const fs::path root(argv[1]),out(argv[2]);
  if(!fs::create_directory(out))throw std::runtime_error("refuse overwrite");
  const auto graph=LoadGraph(root);const auto seed=LoadValues(root/"initial.values");
  const auto frozen=graph;const auto before=graph.linearize(seed)->jacobian(Ordering(seed.keys()));
  const double cost=graph.error(seed),gradient=fl2a::CheckedGradient(graph,seed);
  if(cost!=2794137.3456364814||gradient!=1.5863889609968425)
    throw std::runtime_error("fixture does not reproduce exact initial cost/gradient");
  const auto start=std::chrono::steady_clock::now();const auto result=fl3b::Solve(graph,seed);
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  const auto after=graph.linearize(seed)->jacobian(Ordering(seed.keys()));
  bool unchanged=graph.size()==frozen.size()&&seed.equals(LoadValues(root/"initial.values"),0);
  for(size_t i=0;i<graph.size();++i)unchanged=unchanged&&graph[i]==frozen[i];
  unchanged=unchanged&&(before.first-after.first).norm()==0&&(before.second-after.second).norm()==0
    &&graph.error(seed)==cost&&fl2a::CheckedGradient(graph,seed)==gradient;
  if(!unchanged)throw std::runtime_error("original objective/seed mutated");
  std::ofstream trace(out/"trials.csv");trace<<std::setprecision(17)
    <<"attempt,accepted,library_done,accepted_iterations,library_inner_iterations,lambda_before,lambda_after,cost_before,cost_after,gradient\n";
  for(const auto& t:result.trace)trace<<t.attempt<<','<<t.accepted<<','<<t.libraryDone<<','<<t.acceptedIterations<<','<<t.libraryInnerIterations<<','<<t.lambdaBefore<<','<<t.lambdaAfter<<','<<t.costBefore<<','<<t.costAfter<<','<<t.gradient<<'\n';
  std::ofstream v(out/"diagnostic_only.values");v<<std::setprecision(17);
  for(auto key:result.diagnosticValues.keys()) {
    const Symbol s(key);v<<s.chr()<<' '<<s.index();
    if(s.chr()=='x'){const auto m=result.diagnosticValues.at<Pose3>(key).matrix();
      for(int r=0;r<3;++r)for(int c=0;c<4;++c)v<<' '<<m(r,c);}
    else {const auto x=result.diagnosticValues.at<Vector3>(key);for(int j=0;j<3;++j)v<<' '<<x[j];}v<<'\n';
  }
  std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\n"
    <<"\"passed\": "<<(result.passed?"true":"false")<<",\n\"stop\": \""<<result.stop<<"\",\n"
    <<"\"attempts\": "<<result.attempts<<",\n\"accepted\": "<<result.accepted<<",\n\"rejected\": "<<result.rejected<<",\n"
    <<"\"initial_cost\": "<<cost<<",\n\"final_cost\": "<<graph.error(result.diagnosticValues)<<",\n"
    <<"\"initial_gradient\": "<<gradient<<",\n\"final_gradient\": "<<result.after<<",\n"
    <<"\"original_objective_unchanged\": true,\n\"seconds_with_diagnostics\": "<<seconds<<"\n}\n";
  std::cout<<std::setprecision(17)<<(result.passed?"PASS":"NOT_CONVERGED")<<" attempts="<<result.attempts<<" accepted="<<result.accepted<<" rejected="<<result.rejected<<" gradient="<<result.after<<" stop="<<result.stop<<'\n';
  // A recorded numerical failure is a completed diagnostic, not a crash.
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
