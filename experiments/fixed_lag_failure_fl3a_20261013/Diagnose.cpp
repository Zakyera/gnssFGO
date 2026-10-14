// One observed failure. No diagnostic estimate is ever returned to the receiver.
#define main FL3A_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "Observe.h"
// Frozen header is already included. Only the private receiver's base-class
// name is substituted; its bookkeeping/factors/publication remain unmodified.
#define BoundarySmoother ObservedBoundarySmoother
#include "../fixed_lag_lifecycle_fl2b_20261013/Receiver.h"
#undef BoundarySmoother
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/HessianFactor.h>
#include <fstream>

namespace fl3a {
using namespace gtsam;
namespace fs=std::filesystem;
std::ofstream NewFile(const fs::path& p) {
  if(fs::exists(p))throw std::runtime_error("refuse diagnostic overwrite: "+p.string());
  std::ofstream f(p);f.exceptions(std::ios::failbit|std::ios::badbit);f<<std::setprecision(17);return f;
}
void ValuesFile(const fs::path& p,const Values& v) {
  auto out=NewFile(p);
  for(auto k:v.keys()) {
    out<<Symbol(k).chr()<<' '<<Symbol(k).index();
    if(Symbol(k).chr()=='x') {
      auto m=v.at<Pose3>(k).matrix();
      for(int r=0;r<3;++r)for(int c=0;c<4;++c)out<<' '<<m(r,c);
    }else for(double n:v.at<Vector3>(k))out<<' '<<n;
    out<<'\n';
  }
}
void MatrixFile(const fs::path& p,const Matrix& m) {
  auto out=NewFile(p);out<<m.rows()<<' '<<m.cols()<<'\n';
  for(int r=0;r<m.rows();++r){for(int col=0;col<m.cols();++col)out<<m(r,col)<<' ';out<<'\n';}
}
struct GradientInfo {double scaled=0,g=0,h=0;Key key=0;size_t coordinate=0;};
GradientInfo Worst(const NonlinearFactorGraph& graph,const Values& v) {
  const auto linear=graph.linearize(v);auto g=linear->gradientAtZero(),h=linear->hessianDiagonal();GradientInfo w;
  for(const auto& [key,block]:g)for(int j=0;j<block.size();++j){
    double z=std::abs(block[j])/std::sqrt(std::max(1e-30,h.at(key)[j]));
    if(z>w.scaled)w={z,block[j],h.at(key)[j],key,size_t(j)};
  }return w;
}
void Row(std::ostream& out,const std::string& label,const NonlinearFactorGraph& graph,
         const Values& v,const Values& reference) {
  const auto w=Worst(graph,v);double p=0,r=0,vel=0,omega=0;
  for(Key k:v.keys()) {
    if(Symbol(k).chr()=='x') {
      auto a=reference.at<Pose3>(k),b=v.at<Pose3>(k);
      p=std::max(p,(a.translation()-b.translation()).norm());r=std::max(r,Rot3::Logmap(a.rotation().between(b.rotation())).norm());
    }else if(Symbol(k).chr()=='v')vel=std::max(vel,(v.at<Vector3>(k)-reference.at<Vector3>(k)).norm());
    else omega=std::max(omega,(v.at<Vector3>(k)-reference.at<Vector3>(k)).norm());
  }
  out<<label<<','<<graph.error(v)<<','<<w.scaled<<','<<DefaultKeyFormatter(w.key)<<','<<w.coordinate<<','<<w.g<<','<<w.h
    <<','<<p<<','<<r<<','<<vel<<','<<omega<<'\n';
}
const char* Columns="label,cost,gradient,worst_key,coordinate,raw_gradient,hessian_diagonal,max_p_m,max_r_rad,max_v,max_w\n";

// Store exact source payloads alongside original joint marginal anchor/Gaussian.
// Motion recipe uses the same keys/dt/Qc/flags as the frozen implementation.
void Snapshot(const fs::path& dir,const fl2b::State& prior) {
  auto& c=*capture;fs::create_directory(dir);
  ValuesFile(dir/"initial.values",c.initial);ValuesFile(dir/"last_accepted.values",c.last);
  std::map<const NonlinearFactor*,fl2b::Belief> known;
  for(const auto& [id,e]:prior.active)known.emplace(e.owner.get(),e.latest);
  auto factors=NewFile(dir/"factors.txt");
  for(size_t slot=0;slot<c.slots.size();++slot) {
    const auto& f=c.slots[slot];if(!f)continue;
    factors<<slot<<' ';
    if(auto* m=dynamic_cast<const fl2a::AnchoredMarginal*>(f.get())) {
      factors<<"marginal ";
      for(auto k:f->keys())factors<<DefaultKeyFormatter(k)<<' ';
      factors<<"gaussian_type="<<typeid(m->gaussian()).name()<<'\n';
      auto stem=std::to_string(slot);
      ValuesFile(dir/(stem+".anchor"),m->anchor());
      MatrixFile(dir/(stem+".A"),m->A());MatrixFile(dir/(stem+".b"),m->b());
      MatrixFile(dir/(stem+".augmented_information"),m->gaussian().augmentedInformation());
    }else if(dynamic_cast<const fgo::factor::GPWNOAPrior*>(f.get())) {
      factors<<"wnoa ";for(auto k:f->keys())factors<<DefaultKeyFormatter(k)<<' ';factors<<"dt=0.1 useAutoDiff=1 calcJacobian=1\n";
    }else if(dynamic_cast<const fgo::offline::WnoaRelativeFactor*>(f.get())) {
      fl2b::Belief b;
      if(known.count(f.get()))b=known.at(f.get());
      else {
        if(prior.pending.size()!=1)throw std::runtime_error("ambiguous new source snapshot");
        b=prior.pending.front();
        auto endpoint=[&](int64_t t) {
          auto offset=t-prior.origin;size_t i=offset/fl2b::Period;auto part=offset%fl2b::Period;
          return part?fgo::offline::WnoaEndpoint::Between(i,i+1,.1,part*1e-9,fl2b::Qc(),t).withCache(nullptr):fgo::offline::WnoaEndpoint::Exact(i);
        };
        auto rebuilt=boost::make_shared<fgo::offline::WnoaRelativeFactor>(endpoint(b.start),endpoint(b.end),b.mean,noiseModel::Gaussian::Covariance(b.covariance));
        if(rebuilt->keys()!=f->keys()||!rebuilt->linearize(c.initial)->equals(*f->linearize(c.initial),0))
          throw std::runtime_error("new source snapshot does not exactly match");
      }
      factors<<"source "<<b.source<<' '<<b.session<<' '<<b.from<<' '<<b.to<<' '<<b.revision<<' '<<b.ordinal
        <<' '<<b.start<<' '<<b.end<<' '<<b.arrival;
      auto m=b.mean.matrix();for(int r=0;r<3;++r)for(int col=0;col<4;++col)factors<<' '<<m(r,col);
      for(int r=0;r<6;++r)for(int col=0;col<6;++col)factors<<' '<<b.covariance(r,col);
      factors<<'\n';
    }else throw std::runtime_error("unhandled snapshot factor type");
  }
  auto info=NewFile(dir/"README.txt");info<<"Original failed retained objective, NOT a trajectory output.\n"
    <<"origin_ns "<<prior.origin<<"\nfirst_support "<<prior.oldest<<"\nlast_support 1224\n"
    <<"Qc .00375 .00375 .00375 .015 .015 .015; Gaussian scale1; frozen exact factory.\n"
    <<"Values: x index then 3x4 [R,t] row-major; v/w index then 3 components.\n"
    <<"Source row: slot type source session from to revision ordinal start end arrival,3x4 mean,6x6 covariance row-major.\n"
    <<"Marginal files preserve original anchor, A,b and augmented information INCLUDING additive constant.\n";
}
void ReferenceSolve(const fs::path& dir,const NonlinearFactorGraph& graph,const Values& seed,
                    const Values& initial,const std::string& label,std::ostream& summary) {
  LevenbergMarquardtParams p;p.maxIterations=100;p.relativeErrorTol=1e-12;p.absoluteErrorTol=1e-12;
  p.setLinearSolverType("MULTIFRONTAL_QR");
  auto trace=NewFile(dir/(label+".csv"));trace<<Columns;size_t n=0;
  p.iterationHook=[&](size_t i,double before,double after){trace<<"# iteration "<<i<<" cost "<<before<<" -> "<<after<<'\n';n=i;};
  LevenbergMarquardtOptimizer optimizer(graph,seed,p);const Values best=optimizer.optimize();
  Row(trace,"final",graph,best,initial);Row(summary,label,graph,best,initial);
  ValuesFile(dir/(label+".values"),best);
  trace<<"# iterations "<<optimizer.iterations()<<" lambda "<<optimizer.lambda()<<" hook_last "<<n<<'\n';
}
void Analyze(const fs::path& dir,const fl2b::State& prior) {
  if(!capture)throw std::runtime_error("failure occurred before target");
  auto& c=*capture;Snapshot(dir/"snapshot",prior);
  auto result=NewFile(dir/"result.txt");
  result<<"stop "<<c.stop<<"\nbudget "<<c.budget<<"\nattempts "<<c.result.steps<<"\npassed "<<c.result.passed
    <<"\ninitial_gradient "<<c.result.before<<"\nfinal_accepted_gradient "<<c.result.after
    <<"\nfixed_keys "<<c.fixedKeys<<"\nvariables "<<c.initial.size()<<'\n';
  auto trace=NewFile(dir/"trials.csv");trace<<Columns;
  Row(trace,"initial",c.graph,c.initial,c.initial);
  auto decision=NewFile(dir/"decisions.csv");decision<<"step,accepted,old_cost,trial_cost,tolerance,relinearized,reeliminated\n";
  for(size_t i=0;i<c.steps.size();++i) {
    const auto& s=c.steps[i];std::string name="trial_"+std::to_string(i+1);
    decision<<i+1<<','<<s.accepted<<','<<s.oldCost<<','<<s.newCost<<','<<1e-12*(1+std::abs(s.oldCost))<<','<<s.relinearized<<','<<s.reeliminated<<'\n';
    Row(trace,name,c.graph,s.value,c.initial);ValuesFile(dir/"snapshot"/(name+".values"),s.value);
  }
  trace.flush();decision.flush();result.flush();
  auto controls=NewFile(dir/"controls.csv");controls<<Columns;
  Row(controls,"last_accepted",c.graph,c.last,c.initial);
  // Reference optimizer sees identical original factors, fixed Gaussian/anchor.
  ReferenceSolve(dir,c.graph,c.initial,c.initial,"lm_from_initial",controls);controls.flush();
  ReferenceSolve(dir,c.graph,c.last,c.initial,"lm_from_last",controls);controls.flush();
  // Fresh QR direction eliminates iSAM2 incremental/cached linearization as a variable.
  const auto linear=c.graph.linearize(c.last);const auto delta=linear->optimize(EliminateQR);
  auto lines=NewFile(dir/"fresh_qr_direction.csv");lines<<Columns;
  for(double a:{0.,.125,.25,.5,1.}) {
    VectorValues d=delta;for(auto& [k,v]:d)v*=a;
    auto value=c.last.retract(d);Row(lines,std::to_string(a),c.graph,value,c.last);
    if(a==1.)ValuesFile(dir/"fresh_qr_full.values",value);
  }
  // Original marginal vs independently finite-differentiated current-chart prior.
  auto charts=NewFile(dir/"marginal_chart_check.csv");charts<<"slot,A_max_difference,b_max_difference,cost_difference\n";
  for(size_t slot=0;slot<c.slots.size();++slot)if(auto* m=dynamic_cast<const fl2a::AnchoredMarginal*>(c.slots[slot].get())) {
    fl2::ChartMarginal numeric(m->gaussian().clone(),m->anchor());
    auto a=m->linearize(c.last)->jacobian(),b=numeric.linearize(c.last)->jacobian();
    charts<<slot<<','<<(a.first-b.first).cwiseAbs().maxCoeff()<<','<<(a.second-b.second).cwiseAbs().maxCoeff()
      <<','<<m->error(c.last)-numeric.error(c.last)<<'\n';
  }
  const auto worst=Worst(c.graph,c.last);
  auto contributions=NewFile(dir/"worst_key_contributions.csv");
  contributions<<"slot,type,cost,gradient,hessian_diagonal\n";
  for(size_t slot=0;slot<c.slots.size();++slot) {
    const auto& f=c.slots[slot];if(!f)continue;
    auto l=f->linearize(c.last);const auto g=l->gradientAtZero(),h=l->hessianDiagonal();
    if(!g.exists(worst.key))continue;
    const std::string type=dynamic_cast<const fl2a::AnchoredMarginal*>(f.get())?"marginal":dynamic_cast<const fgo::factor::GPWNOAPrior*>(f.get())?"wnoa":"source";
    contributions<<slot<<','<<type<<','<<f->error(c.last)<<','<<g.at(worst.key)[worst.coordinate]<<','<<h.at(worst.key)[worst.coordinate]<<'\n';
  }
  auto fd=NewFile(dir/"worst_coordinate_cost_derivative.csv");fd<<"h,finite_difference,linearized_gradient,normalized_fd,normalized_gradient\n";
  for(double h:{1e-4,1e-5,1e-6,1e-7}) {
    double g=(c.graph.error(fl2::Perturb(c.last,worst.key,worst.coordinate,h))-c.graph.error(fl2::Perturb(c.last,worst.key,worst.coordinate,-h)))/(2*h);
    fd<<h<<','<<g<<','<<worst.g<<','<<g/std::sqrt(worst.h)<<','<<worst.g/std::sqrt(worst.h)<<'\n';
  }
  std::cout<<"FL3A_ANALYSIS_COMPLETE stop="<<c.stop<<" attempts="<<c.result.steps<<" gradient="<<c.result.after<<'\n';
}
}

int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  std::unique_ptr<fl2b::Receiver> receiver;size_t ordinal=0;
  try {
    if(argc!=4)throw std::runtime_error("usage: diagnose JOURNAL NEW_ARCHIVE NEW_DIAGNOSTICS");
    if(!std::filesystem::create_directory(argv[3]))throw std::runtime_error("refuse diagnostic directory reuse");
    EventStream input(argv[1]);receiver=std::make_unique<fl2b::Receiver>(argv[2]);
    while(ordinal<84958) {
      const auto e=input.Next();++ordinal;
      if(e.kind=='E')throw std::runtime_error("unexpected EOF before failure");
      if(e.kind=='I')receiver->imuClock(e.imu.timestamp_ns,e.arrival_ns);
      else {const auto& r=e.belief;fl2b::Belief b;
        b.source=r.source_agent=="g"?"glim":r.source_agent=="k"?"kimera":r.source_agent;
        b.session=r.session_id;b.from=r.from_index;b.to=r.to_index;b.ordinal=r.ordinal;b.revision=r.revision;
        b.start=r.from_timestamp_ns;b.end=r.to_timestamp_ns;b.arrival=r.record_timestamp_ns;b.mean=r.measurement;b.covariance=r.covariance;receiver->belief(b);
      }
    }
    throw std::runtime_error("expected failure did not recur");
  }catch(const std::exception& e) {
    std::cerr<<"FL3A_STOP event="<<ordinal<<" reason="<<e.what()<<'\n';
    if(ordinal!=84958||std::string(e.what())!="pre-retirement stationarity")return 2;
    if(!receiver||receiver->state().event!=84957||receiver->state().newest!=1223)return 3;
    try{fl3a::Analyze(argv[3],receiver->state());return 0;}
    catch(const std::exception& error){std::cerr<<"DIAGNOSTIC_FAILURE "<<error.what()<<'\n';return 4;}
  }
}
