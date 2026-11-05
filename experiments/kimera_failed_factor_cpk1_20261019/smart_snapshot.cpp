// Retained-factor analysis only. Never linked into the estimator.
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace gtsam;
struct Saved {
  size_t slot=0; SmartStereoProjectionParams params; double sigma=1;
  Pose3 body; TriangulationResult result=TriangulationResult::Degenerate();
  std::vector<StereoPoint2> measurements;KeyVector keys;
  std::vector<std::shared_ptr<Cal3_S2Stereo>> calibrations;
  std::vector<Pose3> cached;Values values;VectorValues direction;
};
class RestoredSmart:public SmartStereoProjectionPoseFactor {
 public:
  RestoredSmart(const Saved&s):SmartStereoProjectionPoseFactor(noiseModel::Isotropic::Sigma(3,s.sigma),s.params,s.body){
    add(s.measurements,s.keys,s.calibrations);result_=s.result;
    cameraPosesTriangulation_.assign(s.cached.begin(),s.cached.end());
  }
  void refresh(){cameraPosesTriangulation_.clear();}
};
std::vector<std::string> fields(const std::string&line){
  std::vector<std::string> f;std::string c;std::istringstream ss(line);
  while(std::getline(ss,c,','))f.push_back(c);return f;
}
Pose3 pose(const std::vector<std::string>& f,size_t start){
  Matrix4 m;for(int i=0;i<4;++i)for(int j=0;j<4;++j)m(i,j)=std::stod(f.at(start+4*i+j));return Pose3(m);
}
struct Slopes{double cost=0,analytic=0,raw=0,schur_correction=0,point_gradient=0;int status;};
Slopes slopes(RestoredSmart s,const Saved& saved){
  Slopes o;o.cost=s.error(saved.values);o.status=s.point().status;
  auto gf=s.linearize(saved.values);
  if(gf)for(const auto&kv:gf->gradientAtZero())o.analytic+=kv.second.dot(saved.direction.at(kv.first));
  if(s.point()){
    SmartStereoProjectionPoseFactor::FBlocks F;Matrix E;Vector b;
    s.computeJacobiansWithTriangulatedPoint(F,E,b,s.cameras(saved.values));s.whitenJacobians(F,E,b);
    Vector fd=Vector::Zero(b.size());for(size_t i=0;i<F.size();++i)fd.segment(3*i,3)=F[i]*saved.direction.at(saved.keys.at(i));
    Vector point_gradient=E.transpose()*b;
    o.raw=-fd.dot(b);o.point_gradient=point_gradient.norm();
    o.schur_correction=fd.dot(E*(E.transpose()*E).inverse()*point_gradient);
  }
  return o;
}
double numerical(RestoredSmart s,const Saved&saved,double alpha,bool fresh){
  auto p=saved.direction,m=saved.direction;for(auto&kv:p)kv.second*=alpha;for(auto&kv:m)kv.second*=-alpha;
  auto a=s,b=s;if(fresh){a.refresh();b.refresh();}
  return (a.error(saved.values.retract(p))-b.error(saved.values.retract(m)))/(2*alpha);
}
void emit(const Saved&s,std::ostream&out){
  RestoredSmart original(s);auto c=slopes(original,s);auto fresh=original;fresh.refresh();auto f=slopes(fresh,s);
  out<<s.slot<<','<<s.keys.size()<<','<<c.status<<','<<c.cost<<','<<c.analytic<<','<<numerical(original,s,.01,false)
     <<','<<c.raw<<','<<c.schur_correction<<','<<c.point_gradient<<','<<f.status<<','<<f.cost<<','<<f.analytic
     <<','<<numerical(original,s,.01,true)<<','<<f.raw<<','<<f.schur_correction<<','<<f.point_gradient<<'\n';
}
int main(int argc,char**argv){try{
  if(argc!=3)throw std::runtime_error("input snapshot and NEW output required");
  if(std::ifstream(argv[2]).good())throw std::runtime_error("refuse overwrite");
  std::ifstream in(argv[1]);if(!in)throw std::runtime_error("input missing");
  std::ofstream out(argv[2]);out<<std::setprecision(17)<<"slot,keys,status,cost,schur_slope,cached_fd,held_point_slope,schur_correction,point_gradient_norm,fresh_status,fresh_cost,fresh_schur_slope,fresh_fd,fresh_held_point_slope,fresh_schur_correction,fresh_point_gradient_norm\n";
  Saved s;bool have=false;std::string line;size_t count=0;
  while(std::getline(in,line)){
    if(!have&&line.rfind("smart,",0)!=0)continue;
    auto f=fields(line);const auto&t=f[0];
    if(t=="smart") {if(have){emit(s,out);++count;}s=Saved();s.slot=std::stoull(f[1]);have=true;}
    else if(t=="params"){
      s.params.linearizationMode=LinearizationMode(std::stoi(f[1]));s.params.degeneracyMode=DegeneracyMode(std::stoi(f[2]));
      s.params.retriangulationThreshold=std::stod(f[3]);s.params.triangulation.rankTolerance=std::stod(f[4]);
      s.params.triangulation.enableEPI=std::stoi(f[5]);s.params.triangulation.landmarkDistanceThreshold=std::stod(f[6]);
      s.params.triangulation.dynamicOutlierRejectionThreshold=std::stod(f[7]);s.params.triangulation.useLOST=std::stoi(f[8]);
      s.params.throwCheirality=std::stoi(f[9]);s.params.verboseCheirality=std::stoi(f[10]);s.sigma=std::stod(f[11]);
    }else if(t=="body")s.body=pose(f,1);
    else if(t=="cache_status"){
      int status=std::stoi(f[1]);s.result= status==0?TriangulationResult(Point3(std::stod(f[2]),std::stod(f[3]),std::stod(f[4]))):TriangulationResult::Degenerate();
      s.result.status=TriangulationResult::Status(status);
    }else if(t=="measurement"){
      s.keys.push_back(std::stoull(f[1]));s.measurements.emplace_back(std::stod(f[2]),std::stod(f[3]),std::stod(f[4]));
      s.calibrations.push_back(std::make_shared<Cal3_S2Stereo>(std::stod(f[5]),std::stod(f[6]),std::stod(f[7]),std::stod(f[8]),std::stod(f[9]),std::stod(f[10])));
    }else if(t=="cached_pose")s.cached.push_back(pose(f,2));
    else if(t.rfind("pose_",0)==0)s.values.insert(std::stoull(t.substr(5)),pose(f,3));
    else if(t.rfind("dir_",0)==0){Vector6 d;for(int i=0;i<6;++i)d[i]=std::stod(f[3+i]);s.direction.insert(std::stoull(t.substr(4)),d);}
  }
  if(have){emit(s,out);++count;}if(!count||!out)throw std::runtime_error("empty snapshot/output failure");
  std::cout<<"Restored "<<count<<" smart factors; no graph optimization\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
