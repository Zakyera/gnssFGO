// Read-only linearization/QR diagnostics of ONE stored objective; no replay.
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include "../fixed_lag_lifecycle_fl2b_20261013/Receiver.h"
#include <tbb/global_control.h>
#include <fstream>
#include <iomanip>
using namespace gtsam;
namespace fs=std::filesystem;
Values LoadValues(const fs::path& p) {
  std::ifstream f(p);if(!f)throw std::runtime_error("missing values");Values out;char kind;size_t i;
  while(f>>kind>>i) {
    if(kind=='x'){Matrix3 R;Vector3 t;for(int r=0;r<3;++r){for(int c=0;c<3;++c)f>>R(r,c);f>>t[r];}out.insert(Symbol(kind,i),Pose3(Rot3(R),t));}
    else {Vector3 v;for(int k=0;k<3;++k)f>>v[k];out.insert(Symbol(kind,i),v);}
  }return out;
}
Matrix LoadMatrix(const fs::path& p) {
  std::ifstream f(p);int rows,cols;if(!(f>>rows>>cols))throw std::runtime_error("missing matrix");Matrix m(rows,cols);
  for(int r=0;r<rows;++r)for(int c=0;c<cols;++c)if(!(f>>m(r,c)))throw std::runtime_error("short matrix");return m;
}
void MatrixOut(const fs::path& p,const Matrix& m) {
  std::ofstream f(p);f<<std::setprecision(17)<<m.rows()<<' '<<m.cols()<<'\n';
  for(int r=0;r<m.rows();++r){for(int c=0;c<m.cols();++c)f<<m(r,c)<<' ';f<<'\n';}
}
Key KeyOf(const std::string& s){return Symbol(s[0],std::stoull(s.substr(1)));}
NonlinearFactorGraph LoadGraph(const fs::path& dir) {
  NonlinearFactorGraph graph;std::ifstream f(dir/"factors.txt");std::string line;
  constexpr int64_t origin=1727775008583475000;
  while(std::getline(f,line)) {
    std::istringstream in(line);size_t slot;std::string type;in>>slot>>type;NonlinearFactor::shared_ptr factor;
    if(type=="marginal") {
      const auto stem=std::to_string(slot);auto anchor=LoadValues(dir/(stem+".anchor"));
      KeyVector keys;std::string s;while(in>>s){if(s.starts_with("gaussian_type=")){if(s!="gaussian_type=N5gtsam14JacobianFactorE")throw std::runtime_error("not recorded JacobianFactor");break;}keys.push_back(KeyOf(s));}
      auto A=LoadMatrix(dir/(stem+".A"));Vector b=LoadMatrix(dir/(stem+".b"));size_t offset=0;
      std::vector<std::pair<Key,Matrix>> terms;for(auto k:keys){auto n=anchor.at(k).dim();terms.emplace_back(k,A.middleCols(offset,n));offset+=n;}
      auto gaussian=boost::make_shared<JacobianFactor>(terms,b);
      const auto saved=LoadMatrix(dir/(stem+".augmented_information"));
      const double err=(gaussian->augmentedInformation()-saved).cwiseAbs().maxCoeff();
      if(err>1e-12*std::max(1.,saved.cwiseAbs().maxCoeff()))throw std::runtime_error("marginal Gaussian changed");
      factor=boost::make_shared<fl2a::AnchoredMarginal>(gaussian,anchor);
    }else if(type=="wnoa") {
      KeyVector k;std::string s;for(int j=0;j<6;++j){in>>s;k.push_back(KeyOf(s));}
      factor=boost::make_shared<fgo::factor::GPWNOAPrior>(k[0],k[1],k[2],k[3],k[4],k[5],.1,fl2b::Qc(),true,true);
    }else if(type=="source") {
      fl2b::Belief b;in>>b.source>>b.session>>b.from>>b.to>>b.revision>>b.ordinal>>b.start>>b.end>>b.arrival;
      Matrix3 R;Vector3 t;for(int r=0;r<3;++r){for(int c=0;c<3;++c)in>>R(r,c);in>>t[r];}b.mean=Pose3(Rot3(R),t);
      for(int r=0;r<6;++r)for(int c=0;c<6;++c)in>>b.covariance(r,c);
      auto endpoint=[&](int64_t time){auto off=time-origin;size_t i=off/fl2b::Period;auto part=off%fl2b::Period;
        return part?fgo::offline::WnoaEndpoint::Between(i,i+1,.1,part*1e-9,fl2b::Qc(),time).withCache(nullptr):fgo::offline::WnoaEndpoint::Exact(i);};
      factor=boost::make_shared<fgo::offline::WnoaRelativeFactor>(endpoint(b.start),endpoint(b.end),b.mean,noiseModel::Gaussian::Covariance(b.covariance));
    }else throw std::runtime_error("unknown snapshot factor");
    if(graph.size()<=slot)graph.resize(slot+1);graph[slot]=factor;
  }return graph;
}
int main(int argc,char** argv) {
 try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  if(argc!=3)throw std::runtime_error("usage: snapshot_audit SNAPSHOT NEW_OUTPUT");fs::path root(argv[1]),out(argv[2]);
  if(!fs::create_directory(out))throw std::runtime_error("refuse overwrite");
  const auto graph=LoadGraph(root);std::ofstream report(out/"linear_checks.csv");report<<std::setprecision(17);
  report<<"seed,cost,gradient,fresh_cost,fresh_gradient,max_local_difference_to_recorded_next\n";
  for(int seed:{0,1,18,19,20}) {
    auto v=LoadValues(root/(seed?"trial_"+std::to_string(seed)+".values":"initial.values"));
    auto linear=graph.linearize(v);auto d=linear->optimize(EliminateQR);auto next=v.retract(d);
    double diff=0;
    if(seed<20){auto recorded=LoadValues(root/("trial_"+std::to_string(seed+1)+".values"));
      for(const auto& [k,x]:next.localCoordinates(recorded))diff=std::max(diff,x.cwiseAbs().maxCoeff());}
    report<<seed<<','<<graph.error(v)<<','<<fl2a::CheckedGradient(graph,v)<<','<<graph.error(next)<<','<<fl2a::CheckedGradient(graph,next)<<','<<diff<<'\n';
    if(seed==20) {
      const auto order=Ordering(v.keys());const auto ab=linear->jacobian(order);
      MatrixOut(out/"A",ab.first);MatrixOut(out/"b",ab.second);
      std::ofstream keys(out/"ordering.txt");for(auto k:order)keys<<DefaultKeyFormatter(k)<<' '<<v.at(k).dim()<<'\n';
      const Vector dd=d.vector(order);MatrixOut(out/"delta",dd);
      const double predicted=(ab.first*dd).squaredNorm(),cost=graph.error(v);
      std::ofstream curve(out/"directional_curvature.csv");curve<<std::setprecision(17)<<"a,center_cost,plus_cost,minus_cost,true_second_derivative,gauss_newton_curvature,ratio\n";
      for(double a:{.25,.125,.0625,.03125}) {
        auto pos=d,neg=d;for(auto& [k,x]:pos)x*=a;for(auto& [k,x]:neg)x*=-a;
        double plus=graph.error(v.retract(pos)),minus=graph.error(v.retract(neg));
        const double curvature=(plus+minus-2*cost)/(a*a);
        curve<<a<<','<<cost<<','<<plus<<','<<minus<<','<<curvature<<','<<predicted<<','<<curvature/predicted<<'\n';
      }
    }
  }
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
