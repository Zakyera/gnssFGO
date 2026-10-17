#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <iostream>
using namespace gtsam;
class Quadratic:public NoiseModelFactor1<double>{
 public:
  Quadratic():NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1),Symbol('x',0)){}
  Vector evaluateError(const double& x,OptionalMatrixType H=nullptr)const override{
    if(H)*H=Matrix::Identity(1,1);return Vector::Constant(1,x-1);
  }
  NonlinearFactor::shared_ptr clone()const override{return std::make_shared<Quadratic>(*this);}
};
int main(int argc,char** argv){try{
  if(argc!=2)throw std::runtime_error("probe output required");
  NonlinearFactorGraph g;g.emplace_shared<Quadratic>();
  Values start,bad;start.insert(Symbol('x',0),1.);bad.insert(Symbol('x',0),2.);
  setenv("FL4U_PROBE",argv[1],1);bool rejected=false;
  try{fl4q::select(g,start,start,bad);}catch(const std::exception& e){
    rejected=std::string(e.what())=="FL4Q no decreasing current-chart LM step";
  }
  if(!rejected || start.at<double>(Symbol('x',0))!=1. || g.error(start)!=0.)
    throw std::runtime_error("probe changed original decision/values");
  std::ifstream f(argv[1]);std::string s((std::istreambuf_iterator<char>(f)),{});
  if(s.find("model,1,0,0,0,")==std::string::npos||s.find("done,1")==std::string::npos)
    throw std::runtime_error("stationary probe incorrect");
  std::cout<<"PASS logging-only stationary failure and nonmutation\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
