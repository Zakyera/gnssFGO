#include "NativeGaussianHistory.h"
#include "Timing.h"
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

using namespace shared_native;
using namespace gtsam;
using Clock=diagnostic_timing::Clock;
void check(bool x,const char* why){if(!x)throw std::runtime_error(why);}
double elapsed(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
struct Reader {
  std::ifstream stream;
  std::map<Id,GaussianFactor::shared_ptr> blocks;
  explicit Reader(const char* path):stream(path,std::ios::binary){check(bool(stream),"open journal");check(u()==0x314a4753,"magic");}
  template<class T>T value(){T x;stream.read(reinterpret_cast<char*>(&x),sizeof(x));check(bool(stream),"truncated journal");return x;}
  uint64_t u(){return value<uint64_t>();}
  double scalar(){return value<double>();}
  uint64_t count(uint64_t max=1000000){auto n=u();check(n<=max,"oversized journal record");return n;}
  KeySet ids(){KeySet s;auto n=count();for(uint64_t i=0;i<n;++i)check(s.insert(u()).second,"duplicate id");return s;}
  Matrix mat(){auto r=count(10000),c=count(10000);check(r*c<10000000,"oversized matrix");Matrix a(r,c);for(uint64_t i=0;i<r;++i)for(uint64_t j=0;j<c;++j)a(i,j)=scalar();check(a.allFinite(),"nonfinite matrix");return a;}
  GaussianFactor::shared_ptr factor(){
    auto n=count();KeyVector ks;std::vector<size_t> ds;size_t total=0;KeySet seen;
    for(uint64_t i=0;i<n;++i){auto k=u(),d=count(1000);check(d>0&&seen.insert(k).second,"factor key/dim");ks.push_back(k);ds.push_back(d);total+=d;}
    auto a=mat();check(a.rows()==total+1&&a.cols()==total+1,"factor shape");
    SymmetricBlockMatrix b(ds,a,true);
    auto f=std::make_shared<HessianFactor>(ks,b);
    check((f->augmentedInformation()-a).norm()==0,"factor roundtrip");return f;
  }
  Snapshot snapshot(Id epoch){
    Snapshot s;s.epoch=epoch;s.generation=u();s.retiring=ids();s.fixed_after=ids();s.removed=ids();
    auto n=count();for(uint64_t i=0;i<n;++i){auto k=u(),d=count(1000),typ=u();auto a=mat();
      if(typ=='x'){check(d==6&&a.rows()==4&&a.cols()==4,"pose shape");s.theta.insert(k,Pose3(a));}
      else if(typ=='v'){check(d==3&&a.rows()==3&&a.cols()==1,"velocity shape");s.theta.insert(k,Vector3(a));}
      else if(typ=='b'){check(d==6&&a.rows()==6&&a.cols()==1,"bias shape");s.theta.insert(k,imuBias::ConstantBias(a.topRows<3>(),a.bottomRows<3>()));}
      else throw std::runtime_error("unknown Value type");
    }
    n=count();std::map<Id,GaussianFactor::shared_ptr> keep;std::set<size_t> slots;
    for(uint64_t i=0;i<n;++i){Id id=u(),version=u();auto slot=count(),changed=u();check(changed<=1,"changed flag");
      auto f=changed?factor():blocks.at(id);check(keep.emplace(id,f).second&&slots.insert(slot).second,"duplicate ref");
      if(s.cache.size()<=slot)s.cache.resize(slot+1);s.cache[slot]=f;s.native.push_back({id,version,slot});
    }
    blocks=std::move(keep);return s;
  }
};
struct Output {
  FILE* f;
  explicit Output(const char* p){int fd=open(p,O_CREAT|O_EXCL|O_WRONLY,0600);check(fd>=0,"output exists/unwritable");f=fdopen(fd,"wb");if(!f){close(fd);throw std::runtime_error("fdopen");}}
  ~Output(){if(f)fclose(f);}
  void matrix(const Matrix& a){for(int i=0;i<a.rows();++i)for(int j=0;j<a.cols();++j){double x=a(i,j);check(fwrite(&x,8,1,f)==1,"output write");}}
  void flush(){check(fflush(f)==0,"output flush");}
};
void row(const char* kind,Id epoch,const std::map<std::string,double>& times,const std::map<std::string,double>& extra={}){
  std::cout<<"{\"kind\":\""<<kind<<"\",\"epoch\":"<<epoch;
  for(auto& x:times)std::cout<<",\""<<x.first<<"\":"<<x.second;
  for(auto& x:extra)std::cout<<",\""<<x.first<<"\":"<<x.second;
  std::cout<<"}\n";
}
double error(const Matrix& a,const Matrix& b){check(a.rows()==b.rows()&&a.cols()==b.cols(),"matrix mismatch");return (a-b).norm()/std::max(1e-300,b.norm());}
int main(int argc,char** argv){
 try{
  check(argc==3,"usage: replay journal output_matrices.bin");std::cout<<std::setprecision(17);
  Reader r(argv[1]);Output out(argv[2]);NativeGaussianHistory h(Mode::Shadow);Snapshot pending;
  bool staged=false,query=false;Key from=0,to=0;Vector6 df,dt;Id qe=0;size_t nq=0,nretire=0;
  double max_info=0,max_cov=0;std::map<std::string,double> totals;
  const auto start=Clock::now();
  while(true){const auto parse=Clock::now();int t=r.stream.get();if(t==EOF)break;Id epoch=r.u();
    if(t=='S'){
      check(!staged&&!query,"stage ordering");pending=r.snapshot(epoch);totals["parse_s"]+=elapsed(parse);
      std::map<std::string,double> ts;diagnostic_timing::output=&ts;auto begin=Clock::now();auto result=h.stage(pending);ts["stage_total_s"]=elapsed(begin);diagnostic_timing::output=nullptr;check(result.ok,result.message.c_str());
      if(!pending.retiring.empty())++nretire;
      size_t dim=0;for(const auto& kv:pending.theta)dim+=kv.value.dim();
      row("stage",epoch,ts,{{"native",double(pending.native.size())},{"dimension",double(dim)},{"retiring",double(pending.retiring.size())}});staged=true;
    }else if(t=='C'){
      check(staged&&!query&&pending.epoch==epoch,"commit ordering");auto admitted=r.u(),active=r.u(),absorbed=r.u(),removed=r.u();r.scalar();totals["parse_s"]+=elapsed(parse);
      auto begin=Clock::now();auto result=h.commit(pending.generation,epoch);double duration=elapsed(begin);check(result.ok,result.message.c_str());
      check(admitted==h.admittedCount()&&active==h.activeCount()&&absorbed==h.absorbedCount()&&removed==h.removedCount(),"conservation mismatch");
      row("commit",epoch,{{"commit_total_s",duration}});staged=false;
    }else if(t=='P'){
      check(!staged&&!query&&h.epoch()==epoch,"query ordering");qe=epoch;from=r.u();to=r.u();auto pf=r.mat(),pt=r.mat();df=r.mat();dt=r.mat();
      auto n=r.count();GaussianFactorGraph expected;for(uint64_t i=0;i<n;++i)expected.push_back(r.factor());totals["parse_s"]+=elapsed(parse);
      const auto validate=Clock::now();
      auto current=h.graph(epoch);auto a=HessianFactor(current).augmentedInformation(),b=HessianFactor(expected).augmentedInformation();
      check(current.keys()==expected.keys(),"graph keys differ");double e=error(a,b);check(e<1e-9,"information mismatch");max_info=std::max(max_info,e);
      auto theta=h.theta(epoch);check(theta.at<Pose3>(from).retract(df).equals(Pose3(pf),1e-12),"from pose mismatch");check(theta.at<Pose3>(to).retract(dt).equals(Pose3(pt),1e-12),"to pose mismatch");
      totals["validate_s"]+=elapsed(validate);query=true;
    }else if(t=='Q'){
      check(query&&qe==epoch,"query output ordering");auto joint=r.mat(),transport=r.mat(),current=r.mat(),relative=r.mat();double recorded_s=r.scalar();totals["parse_s"]+=elapsed(parse);
      std::map<std::string,double> ts;diagnostic_timing::output=&ts;const auto begin=Clock::now();auto c=h.covariance(epoch,from,to,df,dt);ts["query_total_s"]=elapsed(begin);diagnostic_timing::output=nullptr;
      const auto validate=Clock::now();double e=std::max({error(c.joint_theta,joint),error(c.transport,transport),error(c.joint_current,current),error(c.relative,relative)});check(e<1e-8,"covariance mismatch");max_cov=std::max(max_cov,e);totals["validate_s"]+=elapsed(validate);
      auto write=Clock::now();for(const Matrix* a:{&c.joint_theta,&c.transport,&c.joint_current})out.matrix(*a);out.matrix(c.relative);totals["output_s"]+=elapsed(write);
      row("query",epoch,ts,{{"recorded_s",recorded_s},{"max_relative_error",e}});query=false;++nq;
    }else throw std::runtime_error("unknown/failure journal tag");
  }
  check(!staged&&!query&&h.epoch()>0&&nq>0,"incomplete history");out.flush();totals["wall_s"]=elapsed(start);
  row("summary",h.epoch(),totals,{{"queries",double(nq)},{"retirements",double(nretire)},{"admitted",double(h.admittedCount())},{"active",double(h.activeCount())},{"absorbed",double(h.absorbedCount())},{"removed",double(h.removedCount())},{"max_information_relative_error",max_info},{"max_covariance_relative_error",max_cov}});
  return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
