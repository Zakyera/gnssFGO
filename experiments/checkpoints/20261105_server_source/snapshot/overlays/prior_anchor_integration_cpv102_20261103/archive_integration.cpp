// Synthetic integration through the actual out-of-line private backend symbol.
#include "off_tests.inc"
#include "Checkpoint.h"
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace fs=std::filesystem;
void persistentMode102() {
  mode(true);setenv("CPV40_REFRESH","1",1);setenv("CPV38_OFF_REFRESH","0",1);
  setenv("CPV76_PERSISTENT_COMPAT","1",1);setenv("CPV69_RETAIN_COMPAT","0",1);
  unsetenv("CPV66_AUDIT_DIR");unsetenv("CPV66_TEST_ALL");
}
auto persistentParams102() {
  auto p=params();p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::Persistent;return p;
}
struct Probe : cbs::BPSAM {
  Probe():BPSAM(persistentParams102()){}
  int updates() const{return update_count_;}
};
void initialize(Probe& b) {
  Fixture f;NonlinearFactorGraph graph;Values initial;
  for(int k=0;k<3;++k){graph.push_back(f.factors(k));initial.insert(f.initial(k));}
  cbs::BPSAM::UpdateParams u;u.constrainedKeys=groups(0,2);
  b.update(graph,initial,u);b.marginalizeLeaves(retired(0));
  require(!b.getFixedVariables().empty(),"real native marginal prior missing");
}
size_t corrupt(cbs::BPSAM& b) {
  auto& cache=const_cast<GaussianFactorGraph&>(b.fl4nLinearCache());
  const auto& graph=b.getFactorsUnsafe();
  for(size_t slot=0;slot<graph.size();++slot) {
    if(auto* prior=dynamic_cast<LinearContainerFactor*>(graph[slot].get())) {
      std::vector<size_t> dims;
      for(auto k=prior->factor()->begin();k!=prior->factor()->end();++k)dims.push_back(prior->factor()->getDim(k));
      cache[slot]=cpv36::hessian(prior->keys(),dims,cache[slot]->augmentedInformation()*1.01);
      return slot;
    }
  }
  throw std::runtime_error("no raw prior to corrupt in fixture");
}
std::vector<fs::path> archives(const fs::path& root) {
  std::vector<fs::path> paths;
  for(const auto& e:fs::directory_iterator(root))if(e.is_directory())paths.push_back(e.path());
  std::sort(paths.begin(),paths.end());return paths;
}
std::string bytes(const fs::path& path) {
  std::ifstream in(path,std::ios::binary);require(bool(in),"read fixture file");
  return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
void readback(const fs::path& directory,const cpk9::PriorPreservationError& error,int expected_update) {
  require(fs::exists(directory/"complete.marker"),"missing completed archive marker");
  std::ifstream stream(directory/"failure.bin",std::ios::binary);cpv36::BinaryIn a(stream);
  std::string magic;int update,agent;long wall;size_t slot;KeyVector keys;
  double original_error,converted_error;Matrix original,cached,converted;
  a>>magic>>update>>agent>>wall>>slot>>keys>>original_error>>converted_error>>original>>cached>>converted;
  const auto anchor=cpv36::loadValues(a);
  require(stream.peek()==std::char_traits<char>::eof(),"unexpected archive trailing bytes");
  require(magic=="CPV102_PRIOR_FAILURE_1_NATIVE_X86_64"&&update==expected_update&&agent==int('k')&&wall>0,"archive context");
  require(slot==error.slot&&keys==error.keys,"archive slot/keys");
  require(original_error==error.original_cache_error&&converted_error==error.converted_cache_error,"archive errors");
  require(rel(original,error.original)==0&&rel(cached,error.cached)==0&&rel(converted,error.converted)==0,"archive exact matrices");
  require(anchor.equals(error.anchor,0),"archive exact typed anchor");
  require(bytes(directory/"metadata.txt").find(error.what())!=std::string::npos,"archive message");
}
template<class F> std::unique_ptr<cpk9::PriorPreservationError> failure(F action) {
  try{action();}catch(const cpk9::PriorPreservationError& e){return std::make_unique<cpk9::PriorPreservationError>(e);}
  throw std::runtime_error("guard did not preserve its typed exception");
}
int main(int argc,char**argv) try {
  require(argc==2,"fresh test output required");fs::path root(argv[1]);
  require(fs::create_directory(root),"refuse existing fixture output");
  persistentMode102();spdlog::set_level(spdlog::level::err);
  Dl_info dso{};
  require(dladdr(reinterpret_cast<void*>(&cpv102::promoteWithFailureArchive),&dso)&&dso.dli_fname,"archive integration symbol resolution");
  const std::string actual=dso.dli_fname;
  require(actual.find("prior_anchor_integration_cpv102_20261103/private_build/libkimera_vio.so")!=std::string::npos,"wrong backend library resolved");
  std::cout<<"private_backend="<<actual<<'\n';
  std::ofstream(root/"process_maps.txt")<<std::ifstream("/proc/self/maps").rdbuf();
  const auto output=root/"archives";require(fs::create_directory(output),"archive output");
  setenv("CPV102_PRIOR_FAILURE_DIR",output.c_str(),1);
  Probe success;initialize(success);const auto before=success.calculateEstimate();
  require(success.cpk9PromotePriorCharts().priors>0,"successful promotion missing");
  require(success.calculateEstimate().equals(before,0)&&archives(output).empty(),"success changed states or wrote archive");
  Probe bad;initialize(bad);const size_t slot=corrupt(bad);
  const auto pointers=bad.getFactorsUnsafe();const auto locks=bad.getFixedVariables();
  const auto theta=bad.getLinearizationPoint();const auto cache=bad.fl4nLinearCache();
  auto first=failure([&]{bad.cpk9PromotePriorCharts();});
  require(first->slot==slot&&first->original_cache_error>1e-8,"wrong guard branch");
  auto saved=archives(output);require(saved.size()==1,"first archive count");readback(saved[0],*first,bad.updates());
  std::map<std::string,std::string> old_files;
  for(const auto& e:fs::directory_iterator(saved[0]))old_files[e.path().filename().string()]=bytes(e.path());
  failure([&]{bad.cpk9PromotePriorCharts();});
  auto repeated=archives(output);require(repeated.size()==2,"failure archives overwrite");
  for(const auto& p:repeated)readback(p,*first,bad.updates());
  for(const auto& kv:old_files)require(bytes(saved[0]/kv.first)==kv.second,"old archive overwritten");
  require(bad.getFixedVariables()==locks&&bad.getLinearizationPoint().equals(theta,0),"failure mutated locks/theta");
  for(size_t i=0;i<pointers.size();++i)require(bad.getFactorsUnsafe()[i]==pointers[i]&&bad.fl4nLinearCache()[i]==cache[i],"failure mutated graph/cache pointer");
  unsetenv("CPV102_PRIOR_FAILURE_DIR");
  auto disabled=failure([&]{bad.cpk9PromotePriorCharts();});require(std::string(disabled->what())==first->what(),"disabled archive changed failure");
  const auto file=root/"not_a_directory";std::ofstream(file)<<"fixture\n";
  setenv("CPV102_PRIOR_FAILURE_DIR",file.c_str(),1);
  auto unwritable=failure([&]{bad.cpk9PromotePriorCharts();});require(std::string(unwritable->what())==first->what(),"bad output changed failure");
  setenv("CPV102_PRIOR_FAILURE_DIR","relative-not-permitted",1);
  auto relative=failure([&]{bad.cpk9PromotePriorCharts();});require(std::string(relative->what())==first->what(),"relative output changed failure");
  require(archives(output).size()==2,"disabled/bad path wrote an archive");
  // Real smoother transaction: corruption occurs AFTER its backup. Typed guard
  // must propagate and the existing rollback must restore every saved property.
  setenv("CPV102_PRIOR_FAILURE_DIR",output.c_str(),1);
  Fixture f;Smoother smoother(3.5,persistentParams102());
  for(int k=0;k<5;++k) {
    if(k>=2)smoother.addOdometryBeliefsDetailed({f.belief(k-2,k-1)});
    smoother.update(f.factors(k),f.initial(k),{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}});
  }
  Snapshot snapshot(smoother,X(4));const auto old_cache=smoother.bpsam().fl4nLinearCache();
  const auto receipt=smoother.bpsam().cpv40TrackingReceipt();NonlinearFactorGraph none;
  auto rolled=failure([&]{fl4q::transaction(smoother,none,[&]{
    corrupt(smoother.bpsam());smoother.bpsam().cpk9PromotePriorCharts();return 0;
  });});
  snapshot.check(smoother,X(4));require(receipt==smoother.bpsam().cpv40TrackingReceipt(),"rollback CBS ownership");
  for(size_t i=0;i<old_cache.size();++i)require(old_cache[i]==smoother.bpsam().fl4nLinearCache()[i],"rollback cache pointers");
  const auto final_archives=archives(output);
  require(final_archives.size()==3,"transaction failure archive missing");
  for(const auto& path:final_archives)
    if(std::find(repeated.begin(),repeated.end(),path)==repeated.end())
      readback(path,*rolled,std::stoi(receipt));
  unsetenv("CPV102_PRIOR_FAILURE_DIR");
  std::cout<<"PASS CP102: actual backend DSO; success no-write; three unique archives; exact matrix/anchor readback; disabled/bad/relative path preserve exception; graph/locks/cache unchanged; persistent smoother rollback preserved.\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL CP102 "<<e.what()<<'\n';return 1;}
