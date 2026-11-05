#define main cpk1_snapshot_main
#include "smart_snapshot.cpp"
#undef main
#include "ConditionalStereo.h"
#include <Eigen/QR>
#include <chrono>

void require(bool ok,const std::string& why){if(!ok)throw std::runtime_error(why);}
std::vector<Saved> readSaved(const char* path){
  std::ifstream in(path);require(bool(in),"snapshot absent");
  std::vector<Saved> result;Saved s;bool have=false;std::string line;
  while(std::getline(in,line)){
    if(!have&&line.rfind("smart,",0)!=0)continue;
    auto f=fields(line);const auto&t=f[0];
    if(t=="smart"){if(have)result.push_back(s);s=Saved();s.slot=std::stoull(f[1]);have=true;}
    else if(t=="params"){
      s.params.linearizationMode=LinearizationMode(std::stoi(f[1]));s.params.degeneracyMode=DegeneracyMode(std::stoi(f[2]));
      s.params.retriangulationThreshold=std::stod(f[3]);s.params.triangulation.rankTolerance=std::stod(f[4]);
      s.params.triangulation.enableEPI=std::stoi(f[5]);s.params.triangulation.landmarkDistanceThreshold=std::stod(f[6]);
      s.params.triangulation.dynamicOutlierRejectionThreshold=std::stod(f[7]);s.params.triangulation.useLOST=std::stoi(f[8]);
      s.params.throwCheirality=std::stoi(f[9]);s.params.verboseCheirality=std::stoi(f[10]);s.sigma=std::stod(f[11]);
    }else if(t=="body")s.body=pose(f,1);
    else if(t=="cache_status"){
      const int status=std::stoi(f[1]);s.result=status==0?TriangulationResult(Point3(std::stod(f[2]),std::stod(f[3]),std::stod(f[4]))):TriangulationResult::Degenerate();
      s.result.status=TriangulationResult::Status(status);
    }else if(t=="measurement"){
      s.keys.push_back(std::stoull(f[1]));s.measurements.emplace_back(std::stod(f[2]),std::stod(f[3]),std::stod(f[4]));
      s.calibrations.push_back(std::make_shared<Cal3_S2Stereo>(std::stod(f[5]),std::stod(f[6]),std::stod(f[7]),std::stod(f[8]),std::stod(f[9]),std::stod(f[10])));
    }else if(t=="cached_pose")s.cached.push_back(pose(f,2));
    else if(t.rfind("pose_",0)==0)s.values.insert(std::stoull(t.substr(5)),pose(f,3));
    else if(t.rfind("dir_",0)==0){Vector6 d;for(int i=0;i<6;++i)d[i]=std::stod(f[3+i]);s.direction.insert(std::stoull(t.substr(4)),d);}
  }
  if(have)result.push_back(s);return result;
}

double independentDense(const cpk2::Smart& factor,const cpk2::Fit& fit){
  Matrix C(fit.b.size(),6*fit.F.size()+1);C.leftCols(C.cols()-1)=cpk2::denseF(fit);C.rightCols(1)=fit.b;
  Eigen::HouseholderQR<Matrix> qr(fit.E);
  const Matrix Q=qr.householderQ()*Matrix::Identity(fit.E.rows(),3);
  const Matrix projected=C-Q*(Q.transpose()*C);
  const Matrix ref=projected.transpose()*projected;
  const Matrix actual=cpk2::schur(factor,fit)->augmentedInformation();
  return (actual-ref).norm()/std::max(1.,ref.norm());
}

double directional(const cpk2::Smart& factor,const cpk2::Fit& fit,const Saved&s){
  double a=0;for(const auto&kv:cpk2::schur(factor,fit)->gradientAtZero())a+=kv.second.dot(s.direction.at(kv.first));return a;
}
double fd(const cpk2::Smart& factor,const cpk2::Fit& fit,const Saved&s,double alpha){
  auto p=s.direction,m=s.direction;for(auto&kv:p)kv.second*=alpha;for(auto&kv:m)kv.second*=-alpha;
  // Deterministic independent solves from the same stationary center, not a
  // shared mutable triangulation cache; the activity set is fixed in this gate.
  const auto plus=cpk2::minimize(factor,s.values.retract(p),fit.point);
  const auto minus=cpk2::minimize(factor,s.values.retract(m),fit.point);
  return (plus.cost-minus.cost)/(2*alpha);
}

void fixtures(){
  Saved s;s.params.degeneracyMode=ZERO_ON_DEGENERACY;s.sigma=.7;
  s.body=Pose3(Rot3::RzRyRx(.04,-.02,.01),Point3(.1,.03,-.02));
  auto K=std::make_shared<Cal3_S2Stereo>(500,510,0,320,240,.2);Point3 landmark(.2,.3,5);
  for(size_t i=0;i<3;++i){
    const Pose3 p(Rot3::RzRyRx(0,.01*i,0),Point3(.4*i,0,0));
    s.keys.push_back(i+1);s.calibrations.push_back(K);s.values.insert(i+1,p);
    const auto z=StereoCamera(p.compose(s.body),K).project(landmark);
    s.measurements.emplace_back(z.uL()+.2*i,i==1?std::numeric_limits<double>::quiet_NaN():z.uR()-.1*i,z.v()+.3*(i+1));
    s.cached.push_back(p.compose(s.body));s.direction.insert(i+1,(Vector6()<<.1,-.05,0,.2,0,-.1).finished());
  }
  s.result=TriangulationResult(landmark+Point3(.01,-.02,.04));
  RestoredSmart factor(s);const auto initial=*factor.point();
  const auto fit=cpk2::minimize(factor,s.values,initial);
  require(fit.cost<=cpk2::equations(factor,factor.cameras(s.values),initial).cost,"fixture cost");
  require((initial-*factor.point()).norm()==0,"fixture mutated source cache");
  require(independentDense(factor,fit)<=1e-9,"fixture dense Schur");
  const double analytic=directional(factor,fit,s);
  require(std::abs(analytic-fd(factor,fit,s,1e-5))<=1e-5,"fixture derivative/extrinsic/missing-right");
  require(fit.b[4]==0 && fit.E.row(4).norm()==0 && fit.F[1].row(1).norm()==0,"missing right not omitted");
  auto repeat=cpk2::minimize(factor,s.values,initial);
  require((repeat.point-fit.point).norm()==0 && repeat.cost==fit.cost,"fixture nondeterminism");
  bool rejected=false;try{cpk2::minimize(factor,s.values,Point3(0,0,-1));}catch(const std::runtime_error&){rejected=true;}
  require(rejected && (initial-*factor.point()).norm()==0,"failure mutated cache");
  Saved singular;singular.keys={1};singular.calibrations={K};singular.values.insert(1,Pose3());
  singular.measurements={StereoPoint2(320,std::numeric_limits<double>::quiet_NaN(),240)};
  singular.result=TriangulationResult(Point3(0,0,5));RestoredSmart sf(singular);rejected=false;
  try{cpk2::minimize(sf,singular.values,*sf.point());}catch(const std::runtime_error&){rejected=true;}
  require(rejected,"rank-deficient point accepted");
  std::cout<<"PASS fixtures: exact stereo residual, nonidentity body extrinsic, missing-right, dense Schur, deterministic, failure nonmutation\n";
}

int main(int argc,char**argv){try{
  fixtures();if(argc==1)return 0;
  require(argc==3,"snapshot and NEW output required");require(!std::ifstream(argv[2]).good(),"refuse overwrite");
  auto all=readSaved(argv[1]);std::ofstream out(argv[2]);out<<std::setprecision(17);
  out<<"slot,status,keys,original_cost,conditional_cost,iterations,scaled_gradient,schur_slope,fd_001,fd_01,fd_1,schur_dense_error,held_schur_gap,seconds\n";
  size_t valid=0,invalid=0,failed=0;double totalA=0,totalFD[3]={0,0,0},maxFD=0,maxDense=0,oldCost=0,newCost=0;
  for(const auto&s:all){
    RestoredSmart factor(s);const auto before=factor.point();
    if(!before){require(factor.error(s.values)==0,"invalid snapshot activated");++invalid;continue;}
    ++valid;const auto started=std::chrono::steady_clock::now();
    try{
      const auto old=cpk2::equations(factor,factor.cameras(s.values),*before);
      const auto fit=cpk2::minimize(factor,s.values,*before);
      require(fit.cost<=old.cost+1e-12*(1+old.cost),"cost increased");
      const double a=directional(factor,fit,s);double nums[3];const double alphas[3]={.001,.01,.1};
      for(int i=0;i<3;++i)nums[i]=fd(factor,fit,s,alphas[i]);
      const auto F=cpk2::denseF(fit);Vector d(6*s.keys.size());for(size_t i=0;i<s.keys.size();++i)d.segment(6*i,6)=s.direction.at(s.keys[i]);
      const double held=-(F*d).dot(fit.b),dense=independentDense(factor,fit);
      const auto repeat=cpk2::minimize(factor,s.values,*before);
      require(repeat.cost==fit.cost && (repeat.point-fit.point).norm()==0,"nondeterministic point solve");
      require(factor.point().status==before.status && (*before-*factor.point()).norm()==0,"source cache mutation");
      for(int i=0;i<3;++i){maxFD=std::max(maxFD,std::abs(nums[i]-a));totalFD[i]+=nums[i];}
      oldCost+=old.cost;newCost+=fit.cost;totalA+=a;maxDense=std::max(maxDense,dense);
      const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      out<<s.slot<<','<<before.status<<','<<s.keys.size()<<','<<old.cost<<','<<fit.cost<<','<<fit.iterations<<','<<fit.scaledGradient<<','<<a<<','<<nums[0]<<','<<nums[1]<<','<<nums[2]<<','<<dense<<','<<std::abs(held-a)<<','<<seconds<<'\n';out.flush();
    }catch(const std::exception&e){++failed;std::cerr<<"FAILED slot "<<s.slot<<": "<<e.what()<<'\n';}
  }
  std::cout<<std::setprecision(17)<<"valid="<<valid<<" invalid="<<invalid<<" failed="<<failed<<" successful_subset_original_cost="<<oldCost<<" successful_subset_conditional_cost="<<newCost
    <<" analytic="<<totalA<<" FD001="<<totalFD[0]<<" FD01="<<totalFD[1]<<" FD1="<<totalFD[2]<<" max_factor_gap="<<maxFD<<" max_dense_relative_error="<<maxDense<<'\n';
  require(valid==173 && invalid==514,"captured activity changed");require(maxFD<=1e-8,"factor derivative gate");
  for(double n:totalFD)require(std::abs(n-totalA)<=1e-7,"aggregate derivative gate");
  require(maxDense<=1e-9,"dense Schur gate");require(failed==0,"retained-factor gate FAILED; no integration/replay permitted");
  std::cout<<"PASS retained-factor gate; NOT an integrated estimator repair\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
