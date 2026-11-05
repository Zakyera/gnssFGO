// OFFLINE ONLY. Never linked into the estimator or fed back to it.
#include "Checkpoint.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/JacobianFactor.h>
#include <chrono>
#include <iostream>
using namespace gtsam;
using cpv36::require;

class JointStereo final:public NonlinearFactor {
  cpv36::Smart smart_;Key landmark_;
 public:
  JointStereo(const cpv36::Smart& s,Key l):NonlinearFactor(s.keys()),smart_(s),landmark_(l){keys_.push_back(l);}
  size_t dim()const override{return 3*smart_.keys().size();}
  void equations(const Values& x,cpv36::Smart::FBlocks& F,Matrix& E,Vector& b)const {
    auto cams=smart_.cameras(x);const auto& p=x.at<Point3>(landmark_);
    for(const auto& c:cams)require(c.pose().transformTo(p).z()>0,"joint cheirality");
    smart_.computeJacobians(F,E,b,cams,p);smart_.whitenJacobians(F,E,b);
    require(b.allFinite()&&E.allFinite(),"joint nonfinite");
  }
  double error(const Values& x)const override{cpv36::Smart::FBlocks F;Matrix E;Vector b;equations(x,F,E,b);return .5*b.squaredNorm();}
  GaussianFactor::shared_ptr linearize(const Values& x)const override {
    cpv36::Smart::FBlocks F;Matrix E;Vector b;equations(x,F,E,b);
    std::vector<std::pair<Key,Matrix>> terms;for(size_t i=0;i<F.size();++i){Matrix J=Matrix::Zero(b.size(),F[i].cols());J.block(3*i,0,3,F[i].cols())=F[i];terms.emplace_back(smart_.keys()[i],J);}terms.emplace_back(landmark_,E);
    return std::make_shared<JacobianFactor>(terms,b,noiseModel::Unit::Create(b.size()));
  }
  NonlinearFactor::shared_ptr clone()const override{return std::make_shared<JointStereo>(*this);}
};
struct Joint{NonlinearFactorGraph graph;Values values;std::map<size_t,bool> activity;size_t valid=0,invalid=0;};
Joint joint(const cpv36::Snapshot& s,const Values& start) {
  Joint j;j.values=start;
  for(size_t slot=0;slot<s.graph.size();++slot){auto f=s.graph[slot];if(!f)continue;
    if(auto smart=dynamic_cast<const cpv36::Smart*>(f.get())){
      auto q=*smart;double e=q.error(start);auto point=q.point();j.activity[slot]=bool(point);
      if(!point){require(std::abs(e)<1e-12,"invalid smart nonzero cost");++j.invalid;continue;}
      Key l=Symbol('l',slot);j.values.insert(l,*point);auto jf=std::make_shared<JointStereo>(q,l);
      require(std::abs(jf->error(j.values)-e)<1e-8*(1+std::abs(e)),"joint initial visual equality");j.graph.push_back(jf);++j.valid;
    }else if(auto prior=dynamic_cast<const LinearContainerFactor*>(f.get()))j.graph.push_back(std::make_shared<fl4l::OriginalChartPrior>(*prior,s.theta));
    else j.graph.push_back(f->clone());
  }return j;
}
bool derivatives(const NonlinearFactorGraph& graph,const Values& x,const std::filesystem::path& file) {
  std::ofstream out(file);out<<std::setprecision(17)<<"factor,direction,analytic,finite_difference,absolute_difference,pass\n";bool passed=true;
  for(size_t i=0;i<graph.size();++i){if(!graph[i])continue;
    for(int trial=0;trial<3;++trial){VectorValues d;for(Key k:graph[i]->keys()){Vector v(x.at(k).dim());for(int z=0;z<v.size();++z)v[z]=.1*std::sin((Symbol(k).index()+1)*.31+(z+1)*.73+trial*1.23);d.insert(k,v);}
      auto gf=graph[i]->clone()->linearize(x);double analytic=0;
      if(gf)for(const auto& kv:gf->gradientAtZero())analytic+=kv.second.dot(d.at(kv.first));
      const double eps=1e-5;VectorValues plus,minus;for(Key k:x.keys()){Vector v=Vector::Zero(x.at(k).dim());if(d.exists(k))v=d.at(k);plus.insert(k,eps*v);minus.insert(k,-eps*v);}
      double ep=graph[i]->clone()->error(x.retract(plus)),em=graph[i]->clone()->error(x.retract(minus));double fd=(ep-em)/(2*eps);
      bool ok=std::abs(fd-analytic)<=1e-5+1e-5*std::max(std::abs(fd),std::abs(analytic));passed&=ok;
      out<<i<<','<<trial<<','<<analytic<<','<<fd<<','<<std::abs(fd-analytic)<<','<<ok<<'\n';
    }
  }return passed;
}
struct Stats{double imuCost=0,pos=0,rot=0,vel=0;};
Stats imuStats(const cpv36::Snapshot& s,const Values& x){Stats v;for(auto f:s.graph)if(auto p=dynamic_cast<const ImuFactor*>(f.get())){
  auto r=p->unwhitenedError(x);v.imuCost+=p->error(x);v.rot=std::max(v.rot,r.head(3).norm()*180/M_PI);v.pos=std::max(v.pos,r.segment(3,3).norm());v.vel=std::max(v.vel,r.tail(3).norm());}return v;}
double scaledGradient(const NonlinearFactorGraph& g,const Values& x){auto m=g.linearize(x)->augmentedHessian();double v=0;for(int i=0;i<m.rows()-1;++i)v=std::max(v,std::abs(m(i,m.cols()-1))/std::sqrt(std::max(1e-30,std::abs(m(i,i)))));return v;}
void dumpValues(const Values& x,const std::filesystem::path& p){std::ofstream out(p);out<<std::setprecision(17);for(const auto& kv:x){char c=Symbol(kv.key).chr();out<<DefaultKeyFormatter(kv.key);
  if(c=='x'){auto pose=x.at<Pose3>(kv.key);auto q=pose.rotation().toQuaternion();out<<','<<pose.x()<<','<<pose.y()<<','<<pose.z()<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z();}
  else if(c=='b'){auto v=x.at<imuBias::ConstantBias>(kv.key).vector();for(int i=0;i<v.size();++i)out<<','<<v[i];}
  else {auto v=x.at<Vector3>(kv.key);for(int i=0;i<3;++i)out<<','<<v[i];}out<<'\n';}}
void solve(const cpv36::Snapshot& s,const Values& start,const std::filesystem::path& root){
  require(std::filesystem::create_directories(root),"duplicate offline output");auto j=joint(s,start);
  auto native=fl4l::anchored(s.graph,s.theta);double nativeCost=native.error(start),jointCost=j.graph.error(j.values);
  require(std::abs(nativeCost-jointCost)<1e-5+1e-10*std::abs(nativeCost),"full initial cost equality");
  bool nativeDerivative=derivatives(native,start,root/"native_derivatives.csv");
  bool jointDerivative=derivatives(j.graph,j.values,root/"joint_derivatives.csv");
  std::ofstream meta(root/"model.txt");meta<<std::setprecision(17)<<"native_initial_cost,"<<nativeCost<<"\njoint_initial_cost,"<<jointCost<<"\nactive_visual,"<<j.valid<<"\ninactive_visual,"<<j.invalid<<"\nnative_derivative_pass,"<<nativeDerivative<<"\njoint_derivative_pass,"<<jointDerivative<<'\n';meta.flush();
  require(jointDerivative,"joint derivative gate failed; no optimization");
  LevenbergMarquardtParams params;params.maxIterations=30;params.relativeErrorTol=0;params.absoluteErrorTol=0;params.errorTol=0;params.lambdaInitial=1e-3;params.lambdaUpperBound=1e12;params.setDiagonalDamping(true);
  LevenbergMarquardtOptimizer opt(j.graph,j.values,params);std::ofstream trace(root/"iterations.csv");trace<<std::setprecision(17)<<"iteration,cost,imu_cost,max_position_residual,max_rotation_residual_deg,max_velocity_residual,scaled_gradient,lambda\n";
  auto begin=std::chrono::steady_clock::now();std::string stop="iteration_bound";
  for(int i=0;i<=30;++i){const auto& x=opt.values();auto st=imuStats(s,x);double grad=scaledGradient(j.graph,x);
    trace<<i<<','<<opt.error()<<','<<st.imuCost<<','<<st.pos<<','<<st.rot<<','<<st.vel<<','<<grad<<','<<opt.lambda()<<'\n';trace.flush();
    if(grad<1e-5){stop="scaled_gradient_gate";break;}if(i==30)break;
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>180){stop="wall_bound";break;}
    auto prior=opt.values();double cost=opt.error();try{opt.iterate();}catch(const std::exception& e){stop=std::string("solver_exception:")+e.what();break;}
    require(opt.error()<=cost+1e-8*(1+std::abs(cost)),"joint cost increased");
    if(opt.values().equals(prior,0)){stop="no_accepted_step";break;}
  }
  dumpValues(opt.values(),root/"final_values.csv");size_t activityChanges=0,far=0;
  for(const auto& item:j.activity){auto smart=std::dynamic_pointer_cast<cpv36::Smart>(s.graph[item.first]->clone());smart->error(opt.values());if(bool(smart->point())!=item.second)++activityChanges;
    if(item.second){Key l=Symbol('l',item.first);const auto& point=opt.values().at<Point3>(l);auto cams=smart->cameras(opt.values());double lim=smart->cpv36Params().triangulation.landmarkDistanceThreshold;
      if(lim>0)for(const auto& c:cams)if((c.pose().translation()-point).norm()>lim){++far;break;}}
  }
  meta<<"stop,"<<stop<<"\nfinal_native_cost,"<<fl4l::anchored(s.graph,s.theta).error(opt.values())<<"\nvisual_activity_changes,"<<activityChanges<<"\njoint_landmarks_over_native_distance_limit,"<<far<<'\n';
  meta<<"interpretation,isolated joint-landmark conditional diagnostic; not native output or production fix\n";
}
int main(int argc,char**argv){try{
  require(argc==3,"checkpoint and fresh output directory required");std::filesystem::path root=argv[2];require(std::filesystem::create_directories(root),"output exists");auto s=cpv36::load(argv[1]);
  auto fresh=cpv36::clone(s.graph).linearize(s.theta)->augmentedHessian();std::ofstream eq(root/"equations.txt");eq<<std::setprecision(17)<<"cached_fresh_relative,"<<(s.cached-fresh).norm()/(1+fresh.norm())<<"\ncached_tree_relative,"<<(s.cached-s.tree).norm()/(1+s.cached.norm())<<'\n';
  bool passed=true;
  for(const auto& item:std::vector<std::pair<std::string,const Values*>>{{"initialized",&s.initial},{"incremental_result",&s.estimate}}){
    try{solve(s,*item.second,root/item.first);}
    catch(const std::exception& e){passed=false;std::ofstream failure(root/(item.first+"_failed.txt"));failure<<e.what()<<'\n';}
  }
  std::ofstream done(root/(passed?"complete.txt":"failed.txt"));done<<"offline diagnostics finished; inspect derivative/domain/convergence qualifications\n";return passed?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
