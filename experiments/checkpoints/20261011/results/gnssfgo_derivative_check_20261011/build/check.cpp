#include "SourceWnoaModel.h"
#include <gtsam/linear/JacobianFactor.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <chrono>

using namespace fgo::offline;
using NF=gtsam::NoiseModelFactor;
std::vector<std::string> split(const std::string& s){std::stringstream ss(s);std::string x;std::vector<std::string> v;while(std::getline(ss,x,','))v.push_back(x);return v;}
gtsam::Key key(const std::string& s){return gtsam::Symbol(s[0],std::stoull(s.substr(1)));}
struct Pair{size_t slot;std::string kind;boost::shared_ptr<NF> native,analytic;};
struct Acc{long double g=0,d=0,absolute=0;void add(const gtsam::Vector& j,const gtsam::Vector& r){const double t=j.dot(r);g+=t;d+=j.squaredNorm();absolute+=std::abs(t);}};

int main(int argc,char**argv){
  if(argc!=3)return 2;const std::string root=argv[1],old=argv[2];
  gtsam::Values values;std::ifstream vf(old+"/failure_probe/simultaneous_values.csv");std::string line;std::getline(vf,line);
  while(std::getline(vf,line)){auto f=split(line);auto k=key(f[0]);
    if(f[0][0]=='x')values.insert(k,gtsam::Pose3(gtsam::Rot3::Quaternion(std::stod(f[7]),std::stod(f[4]),std::stod(f[5]),std::stod(f[6])),gtsam::Point3(std::stod(f[1]),std::stod(f[2]),std::stod(f[3]))));
    else values.insert(k,gtsam::Vector3(std::stod(f[1]),std::stod(f[2]),std::stod(f[3])));
  }
  const auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  auto endpoint=[&](int64_t t,bool numeric){
    const int64_t dt=t-1727775008583475000LL;const size_t left=dt/100000000LL;const int64_t rem=dt%100000000LL;
    if(dt<0)throw std::runtime_error("outside support");
    if(rem==0)return WnoaEndpoint::Exact(left);
    auto ep=WnoaEndpoint::Between(left,left+1,.1,rem*1e-9,qc);
    if(!numeric)ep.interpolator=std::make_shared<fgo::models::GPWNOAInterpolator>(qc,.1,rem*1e-9,false,true);
    return ep;
  };
  std::vector<Pair> factors;std::ifstream input(root+"/selected_factors.csv");
  while(std::getline(input,line)){auto f=split(line);Pair p;p.slot=std::stoull(f[0]);p.kind=f[1];
    if(p.kind=="source"){
      gtsam::Vector6 mu;gtsam::Matrix6 cov;for(int c=0;c<6;++c)mu[c]=std::stod(f[4+c]);
      for(int i=0;i<6;++i)for(int j=0;j<6;++j)cov(i,j)=std::stod(f[10+6*i+j]);
      const auto noise=gtsam::noiseModel::Gaussian::Covariance(cov,false);const auto z=gtsam::Pose3::Expmap(mu);
      p.native=boost::make_shared<WnoaRelativeFactor>(endpoint(std::stoll(f[2]),true),endpoint(std::stoll(f[3]),true),z,noise);
      p.analytic=boost::make_shared<WnoaRelativeFactor>(endpoint(std::stoll(f[2]),false),endpoint(std::stoll(f[3]),false),z,noise);
    }else{
      const size_t i=std::stoull(f[2]),j=std::stoull(f[3]);
      p.native=boost::make_shared<fgo::factor::GPWNOAPrior>(X(i),V(i),W(i),X(j),V(j),W(j),(j-i)*.1,qc,true,true);
      p.analytic=boost::make_shared<fgo::factor::GPWNOAPrior>(X(i),V(i),W(i),X(j),V(j),W(j),(j-i)*.1,qc,false,true);
    }factors.push_back(p);
  }
  std::map<size_t,std::pair<gtsam::Vector,gtsam::Matrix>> expected;
  std::ifstream ef(root+"/expected.csv");while(std::getline(ef,line)){auto f=split(line);int n=std::stoi(f[1]);gtsam::Vector r(n);gtsam::Matrix c(n,n);
    for(int i=0;i<n;++i)r[i]=std::stod(f[2+i]);for(int i=0;i<n;++i)for(int j=0;j<n;++j)c(i,j)=std::stod(f[2+n+n*i+j]);expected[std::stoull(f[0])]={r,c};}
  std::map<std::pair<gtsam::Key,int>,int> targets;std::ifstream tf(root+"/targets.csv");while(std::getline(tf,line)){auto f=split(line);targets[{key(f[1]),std::stoi(f[2])}]=std::stoi(f[0]);}
  std::map<std::tuple<int,std::string,double>,Acc> sum;
  std::ofstream valid(root+"/reconstruction.csv"),blocks(root+"/jacobians.csv"),grad(root+"/gradient_contributions.csv"),agg(root+"/gradients.csv");
  for(auto*o:{&valid,&blocks,&grad,&agg})*o<<std::setprecision(17);
  valid<<"slot,kind,residual_diff,covariance_diff,analytic_residual_diff,linearized_residual_diff\n";
  blocks<<"slot,kind,key,step,native_vs_numeric,analytic_vs_numeric,native_vs_analytic,reference_norm\n";
  grad<<"rank,slot,kind,method,step,gradient,diagonal\n";
  double max_res=0,max_cov=0,max_model=0;const auto start=std::chrono::steady_clock::now();
  for(const auto&p:factors){
    gtsam::Values local;for(auto k:p.native->keys())local.insert(k,values.at(k));
    const auto r=p.native->whitenedError(local);const auto un=p.native->unwhitenedError(local);
    const auto ra=p.analytic->unwhitenedError(local);
    const auto gn=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(p.native->noiseModel());
    auto jn=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(p.native->linearize(local));
    auto ja=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(p.analytic->linearize(local));
    const double rd=(un-expected.at(p.slot).first).cwiseAbs().maxCoeff(),cd=(gn->covariance()-expected.at(p.slot).second).cwiseAbs().maxCoeff(),ad=(un-ra).cwiseAbs().maxCoeff();
    max_res=std::max(max_res,rd);max_cov=std::max(max_cov,cd);max_model=std::max(max_model,ad);
    valid<<p.slot<<','<<p.kind<<','<<rd<<','<<cd<<','<<ad<<','<<(jn->getb()-ja->getb()).cwiseAbs().maxCoeff()<<'\n';
    if(rd>1e-9||cd>1e-12||ad>1e-12)throw std::runtime_error("snapshot reconstruction/model mismatch");
    for(auto k:p.native->keys()){
      const gtsam::Matrix native=jn->getA(std::find(jn->begin(),jn->end(),k));
      const gtsam::Matrix analytic=ja->getA(std::find(ja->begin(),ja->end(),k));
      for(double h:{1e-3,1e-4,1e-5}){
        gtsam::Matrix numeric(r.size(),local.at(k).dim());
        for(int c=0;c<numeric.cols();++c){auto d=local.zeroVectors();
          d.at(k)[c]=2*h;const auto p2=p.native->whitenedError(local.retract(d));
          d.at(k)[c]=h;const auto p1=p.native->whitenedError(local.retract(d));
          d.at(k)[c]=-h;const auto m1=p.native->whitenedError(local.retract(d));
          d.at(k)[c]=-2*h;const auto m2=p.native->whitenedError(local.retract(d));
          numeric.col(c)=(8*(p1-m1)-(p2-m2))/(12*h);
          const auto it=targets.find({k,c});if(it!=targets.end()){
            auto add=[&](const std::string&method,const gtsam::Vector&j,double step){auto&a=sum[{it->second,method,step}];a.add(j,r);grad<<it->second<<','<<p.slot<<','<<p.kind<<','<<method<<','<<step<<','<<j.dot(r)<<','<<j.squaredNorm()<<'\n';};
            add("direct5",numeric.col(c),h);if(h==1e-3){add("native",native.col(c),0);add("analytic",analytic.col(c),0);}
          }
        }
        blocks<<p.slot<<','<<p.kind<<','<<gtsam::DefaultKeyFormatter(k)<<','<<h<<','<<(native-numeric).cwiseAbs().maxCoeff()<<','<<(analytic-numeric).cwiseAbs().maxCoeff()<<','<<(native-analytic).cwiseAbs().maxCoeff()<<','<<numeric.cwiseAbs().maxCoeff()<<'\n';
      }
    }
  }
  agg<<"rank,key,column,method,step,gradient,diagonal,normalized,absolute_sum\n";
  for(const auto&[identity,a]:sum){auto[rank,method,h]=identity;auto t=std::find_if(targets.begin(),targets.end(),[&](const auto&x){return x.second==rank;});
    agg<<rank<<','<<gtsam::DefaultKeyFormatter(t->first.first)<<','<<t->first.second<<','<<method<<','<<h<<','<<a.g<<','<<a.d<<','<<std::abs(a.g)/std::sqrt(a.d)<<','<<a.absolute<<'\n';}
  std::ofstream summary(root+"/check_summary.txt");summary<<std::setprecision(17)<<"factors="<<factors.size()<<" state_values="<<values.size()<<" max_saved_residual_diff="<<max_res<<" max_covariance_diff="<<max_cov<<" max_analytic_residual_diff="<<max_model<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
}
