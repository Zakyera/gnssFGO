// Two isolated final-graph diagnostics; production sources are read-only.
#define main previous_audit_main
#include "/old_audit/audit.cpp"
#undef main
#include "/previous/build/wnoa/SnapshotAudit.h"
namespace sa=snapshot_audit;

void Trajectory(const std::string& path,const Data&d,const gtsam::Values&v){
  std::ofstream out(path);
  for(size_t i=0;i<d.times.size();++i){const auto p=v.at<gtsam::Pose3>(X(i));const auto q=p.rotation().toQuaternion();
    out<<std::fixed<<std::setprecision(9)<<d.times[i]*1e-9<<std::setprecision(16)<<' '<<p.x()<<' '<<p.y()<<' '<<p.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';}
}
// Same solver and stopping checks as the previous authorized counterfactual.
void Solve(const std::string& label,const Graph&g,const Data&d,const std::map<size_t,std::string>&sources){
  const std::string dir="/audit/"+label;Check(!std::filesystem::exists(dir),"refuse duplicate solve");std::filesystem::create_directory(dir);
  const auto probe=sa::Fingerprint(g,d.values);
  sa::FactorErrors(dir+"/initial_factors.csv",g,d.values,sources);
  sa::LinearHealth(dir+"/initial_linear_health.csv",g,d.values);
  auto params=gtsam::LevenbergMarquardtParams::CeresDefaults();params.setLinearSolverType("MULTIFRONTAL_QR");
  params.maxIterations=500;params.relativeErrorTol=1e-11;params.absoluteErrorTol=1e-11;
  std::ofstream log(dir+"/iterations.csv");log<<std::setprecision(17)<<"phase,iteration,before,after\n";int phase=1;
  params.iterationHook=[&](size_t n,double before,double after){log<<phase<<','<<n<<','<<before<<','<<after<<'\n';log.flush();};
  auto start=std::chrono::steady_clock::now();gtsam::LevenbergMarquardtOptimizer optimizer(g,d.values,params);
  const auto batch=optimizer.optimize();phase=2;params.maxIterations=100;
  gtsam::LevenbergMarquardtOptimizer polish(g,batch,params);const auto result=polish.optimize();
  double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  const double before=g.error(d.values),after=g.error(result),gradient=fgo::offline::NormalizedGradient(g,result);
  Check(std::isfinite(after)&&after<=before+1e-7,"invalid objective outcome");
  sa::States(dir+"/states.csv",result,d.times,d.times.size()-1);Trajectory(dir+"/trajectory.tum",d,result);
  sa::FactorErrors(dir+"/final_factors.csv",g,result,sources);sa::LinearHealth(dir+"/final_linear_health.csv",g,result);
  double pos=0,rot=0,vel=0,omega=0;
  for(size_t i=0;i<d.times.size();++i){auto a=ReadSourceWnoaState(d.values,i),b=ReadSourceWnoaState(result,i);
    pos=std::max(pos,(b.pose.translation()-a.pose.translation()).norm());rot=std::max(rot,gtsam::Rot3::Logmap(a.pose.rotation().between(b.pose.rotation())).norm());
    vel=std::max(vel,(b.velocity-a.velocity).norm());omega=std::max(omega,(b.omega-a.omega).norm());}
  Check(probe==sa::Fingerprint(g,d.values),"optimizer mutated objective");
  std::ofstream out(dir+"/optimizer.json");out<<std::setprecision(17)<<"{\"initial_cost\":"<<before<<",\"final_cost\":"<<after
    <<",\"normalized_gradient\":"<<gradient<<",\"batch_iterations\":"<<optimizer.iterations()<<",\"polish_iterations\":"<<polish.iterations()
    <<",\"seconds\":"<<elapsed<<",\"max_position_change_m\":"<<pos<<",\"max_rotation_change_rad\":"<<rot<<",\"max_velocity_change_mps\":"<<vel<<",\"max_omega_change_radps\":"<<omega<<"}\n";out.close();
  Check(std::isfinite(gradient)&&gradient<=1e-5,"stationarity gate failed; no tuning/retry");
  Check(optimizer.iterations()<500&&polish.iterations()<100,"iteration budget exhausted");
  std::cout<<label<<" objective "<<before<<" -> "<<after<<" gradient "<<gradient<<" seconds "<<elapsed<<'\n'<<std::flush;
}

void FixedNoiseResiduals(const std::string& label,const Graph&original,const Data&d,const std::map<size_t,std::string>&sources){
  const std::string dir="/audit/fixed_on_noise";
  sa::FactorErrors(dir+"/"+label+"_factors.csv",original,d.values,sources);
  std::ofstream out(dir+"/"+label+"_source_raw.csv");out<<std::setprecision(17)<<"slot,source,r0,r1,r2,r3,r4,r5\n";
  for(const auto&[slot,source]:sources){auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(original[slot]);auto r=f->unwhitenedError(d.values);
    Check(r.size()==6&&r.allFinite(),"source residual invalid");out<<slot<<','<<source;for(double x:r)out<<','<<x;out<<'\n';}
}

int main(){try{
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,1);
  const std::string dir="/base/on_wnoa/snapshot_final";
  const auto d=Load(dir+"/incremental_states.csv");double reconstruction;
  const auto original=Build(d,dir,"/base/payloads/on_final.csv",reconstruction);auto[ce,re]=Probe(original,d,dir+"/factor_payload_probe.txt");
  Check(ce>=0&&re>=0&&d.times.size()==1193&&original.nrFactors()==2965,"unexpected original graph/probe");
  Check(fgo::offline::NormalizedGradient(original,d.values)<=1e-5,"original not stationary");
  std::map<size_t,std::string> sources;std::map<size_t,M> donors;
  for(const auto&r:Read("/prior/donors.csv")){Check(r.size()==40,"donor schema");size_t slot=std::stoull(r[0]);M c(6,6);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)c(i,j)=std::stod(r[4+6*i+j]);
    Check(c.allFinite()&&(c-c.transpose()).norm()<1e-12&&Eigen::SelfAdjointEigenSolver<M>(c).eigenvalues().minCoeff()>0,"donor not SPD");
    Check(donors.emplace(slot,c).second,"duplicate donor slot");sources[slot]=r[1];}
  Check(donors.size()==1770,"unexpected donor count");
  const auto fingerprint=sa::Fingerprint(original,d.values);
  auto delta=d.values.zeroVectors();for(auto&[key,v]:delta)for(int k=0;k<v.size();++k)v[k]=1e-4*std::sin(double(key%997)+k);
  const auto perturbed=d.values.retract(delta);
  std::filesystem::create_directory("/audit/fixed_on_noise");
  FixedNoiseResiduals("original_on",original,d,sources);
  const auto both=Load("/prior/on_means_off_cov/states.csv");Check(both.times==d.times,"both reference times differ");
  FixedNoiseResiduals("both_off_cov",original,both,sources);
  for(const auto& [label,selected]:std::vector<std::pair<std::string,std::string>>{{"glim_off_cov","glim_hesai"},{"kimera_off_cov","kimera_front"}}){
    Graph changed=original;size_t changed_count=0,unchanged_source=0,non_source=0;double rawerr=0,jacerr=0,coverr=0;
    std::ofstream mapping("/audit/"+label+"_factor_checks.csv");mapping<<std::setprecision(17)<<"slot,source,raw_error,jacobian_error,covariance_error\n";
    for(size_t slot=0;slot<original.size();++slot)if(original[slot]){
      if(!donors.count(slot)||sources.at(slot)!=selected){Check(changed[slot].get()==original[slot].get(),"unchanged factor pointer mismatch");
        if(donors.count(slot))++unchanged_source;else ++non_source;continue;}
      auto old=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(original[slot]);Check(old&&old->dim()==6,"source dimension/type");
      auto fresh=old->cloneWithNewNoiseModel(gtsam::noiseModel::Gaussian::Covariance(donors.at(slot),false));
      Check(fresh.get()!=old.get()&&fresh->keys()==old->keys()&&typeid(*fresh)==typeid(*old),"clone changed structure");
      double e=0,j=0;
      for(const auto* values:{&d.values,&perturbed}){
        std::vector<M>a(old->keys().size()),b(old->keys().size());auto ra=old->unwhitenedError(*values,a),rb=fresh->unwhitenedError(*values,b);
        e=std::max(e,(ra-rb).cwiseAbs().maxCoeff());for(size_t k=0;k<a.size();++k)j=std::max(j,(a[k]-b[k]).cwiseAbs().maxCoeff());}
      auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(fresh->noiseModel());double c=(noise->covariance()-donors.at(slot)).cwiseAbs().maxCoeff();
      Check(e<1e-12&&j<1e-12&&c<1e-12,"not covariance-only");
      rawerr=std::max(rawerr,e);jacerr=std::max(jacerr,j);coverr=std::max(coverr,c);changed[slot]=fresh;++changed_count;
      mapping<<slot<<','<<selected<<','<<e<<','<<j<<','<<c<<'\n';}
    Check(non_source==1195&&changed_count==(selected=="glim_hesai"?1189:581)&&changed_count+unchanged_source==1770&&changed.nrFactors()==2965,"factor accounting");
    std::ofstream valid("/audit/"+label+"_validation.json");valid<<std::setprecision(17)
      <<"{\"passed\":true,\"changed_factors\":"<<changed_count<<",\"unchanged_source_factors\":"<<unchanged_source<<",\"unchanged_non_source_factors\":"<<non_source
      <<",\"original_cost_relative_error\":"<<reconstruction<<",\"original_probe_cov_error\":"<<ce<<",\"original_probe_residual_error\":"<<re
      <<",\"changed_covariance_error\":"<<coverr<<",\"raw_error\":"<<rawerr<<",\"jacobian_error\":"<<jacerr<<"}\n";valid.close();
    Solve(label,changed,d,sources);
    const auto solved=Load("/audit/"+label+"/states.csv");Check(solved.times==d.times,"new result times differ");
    FixedNoiseResiduals(label,original,solved,sources);
    Check(fingerprint==sa::Fingerprint(original,d.values),"mutated original graph");
  }
  std::ofstream done("/audit/complete.json");done<<"{\"passed\":true,\"new_solves\":2,\"original_graph_unchanged\":true}\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
