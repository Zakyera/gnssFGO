#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <iostream>
#include <sstream>
using namespace gtsam;
class TestQuadratic: public NoiseModelFactor1<double> {
  double jacobian_;
 public:
  explicit TestQuadratic(double j):NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1),Symbol('x',0)),jacobian_(j){}
  Vector evaluateError(const double& x,OptionalMatrixType H=nullptr) const override {
    if(H)*H=Matrix::Constant(1,1,jacobian_);return Vector::Constant(1,x-1);
  }
  NonlinearFactor::shared_ptr clone() const override {return std::make_shared<TestQuadratic>(*this);}
};
int main(int argc,char**argv){try{
  if(argc!=2)throw std::runtime_error("output prefix required");
  for(int j:{1,2}){
    NonlinearFactorGraph graph;graph.emplace_shared<TestQuadratic>(j);
    Values v;v.insert(Symbol('x',0),0.);
    const auto original=graph[0];const double initial=graph.error(v);
    std::string path=std::string(argv[1])+"_j"+std::to_string(j)+".txt";
    setenv("CPK1_PROBE",path.c_str(),1);fl4q::factorProbe(0,graph,v,v);
    if(v.at<double>(Symbol('x',0))!=0 || graph[0]!=original || graph.error(v)!=initial)
      throw std::runtime_error("probe mutated input");
    std::ifstream input(path);std::string line;bool checked=false;
    while(std::getline(input,line))if(line.rfind("factor,",0)==0){
      std::istringstream row(line);std::vector<std::string> cells;std::string cell;
      while(std::getline(row,cell,','))cells.push_back(cell);
      if(std::abs(std::stod(cells.at(5))+1.)>1e-12)throw std::runtime_error("analytic slope mismatch");
      for(int column:{7,10,13})if(std::abs(std::stod(cells.at(column))+1./j)>1e-9)
        throw std::runtime_error("numerical slope mismatch");
      checked=true;
    }
    if(!checked)throw std::runtime_error("missing factor row");
  }
  std::cout<<"PASS correct slope, deliberately wrong Jacobian localized, nonmutation\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
