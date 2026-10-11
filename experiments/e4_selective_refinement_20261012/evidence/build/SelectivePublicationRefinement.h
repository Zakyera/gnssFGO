#pragma once
#include "FullPublicationRefinement.h"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace e4 {
using Clock = std::chrono::steady_clock;
inline double Elapsed(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now()-t).count();
}
struct Diagnostic {
  size_t variables{}, factors{}, seeds{}, selected{}, relinearized{};
  double verification_s{}, selection_s{}, copy_update_s{}, fallback_s{};
  std::string branch{"disabled"};
};
inline thread_local Diagnostic diagnostic;
struct Certificate { double maximum{}; gtsam::KeySet violating; };
// Same current-Values normalization and zero-information rules as the frozen
// certificate. Selection additionally retains which keys violate that test.
inline Certificate Certify(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::Values& values, double tolerance) {
  const auto begin=Clock::now();
  const auto linear=graph.linearize(values);
  const auto gradient=linear->gradientAtZero(), diagonal=linear->hessianDiagonal();
  Certificate c;
  for(const auto& [key,g]:gradient) {
    const auto& d=diagonal.at(key);
    if(!g.allFinite() || !d.allFinite() || (d.array()<0).any())
      throw std::runtime_error("nonfinite/negative linear system entry");
    for(int i=0;i<g.size();++i) {
      if(d[i]==0 && g[i]!=0)throw std::runtime_error("nonzero gradient in a zero-information coordinate");
      if(d[i]>0) {
        const double n=std::abs(g[i])/std::sqrt(d[i]);
        c.maximum=std::max(c.maximum,n);
        if(n>tolerance)c.violating.insert(key);
      }
    }
  }
  diagnostic.verification_s+=Elapsed(begin);
  return c;
}
inline gtsam::KeySet Neighbours(const gtsam::NonlinearFactorGraph& graph,
                                const gtsam::KeySet& seeds) {
  auto selected=seeds;
  for(const auto& f:graph)if(f) {
    bool touches=false;
    for(auto k:f->keys())if(seeds.count(k)){touches=true;break;}
    if(touches)selected.insert(f->keys().begin(),f->keys().end());
  }
  return selected;
}
inline bool Enabled() {
  const char* s=std::getenv("E4_SELECTIVE_REFINEMENT");
  return s && std::string(s)=="1";
}
inline numerical_audit::Result Refine(std::unique_ptr<gtsam::ISAM2>& solver,
                                      size_t limit=20,double tolerance=1e-5,
                                      bool enabled=Enabled()) {
  diagnostic={};
  if(!solver || !std::isfinite(tolerance) || tolerance<=0)
    throw std::invalid_argument("invalid selective publication policy");
  auto fallback=[&](size_t remaining) {
    const auto t=Clock::now();
    auto result=numerical_audit::Refine(solver,remaining,tolerance);
    diagnostic.fallback_s+=Elapsed(t);
    return result;
  };
  if(!enabled)return fallback(limit);
  diagnostic.branch="initial";
  const auto initial=solver->calculateEstimate();
  const auto& graph=solver->getFactorsUnsafe();
  diagnostic.variables=initial.size(); diagnostic.factors=graph.nrFactors();
  numerical_audit::Result result;
  result.error_before=result.error_after=graph.error(initial);
  if(!std::isfinite(result.error_before))throw std::runtime_error("nonfinite input objective");
  const auto certificate=Certify(graph,initial,tolerance);
  result.gradient_before=result.gradient_after=certificate.maximum;
  if(certificate.maximum<=tolerance) {
    diagnostic.branch="stationary";result.status="stationary";return result;
  }
  const auto select_begin=Clock::now();
  const auto selected=Neighbours(graph,certificate.violating);
  diagnostic.seeds=certificate.violating.size();diagnostic.selected=selected.size();
  diagnostic.selection_s=Elapsed(select_begin);
  if(limit==0 || selected.empty() || selected.size()==initial.size()) {
    diagnostic.branch=limit==0?"no_budget":"all_keys";return fallback(limit);
  }
  // One selective attempt consumes ONE of the unchanged total attempt budget.
  // A nonstationary trial is never committed, even when its cost decreases.
  numerical_audit::Step step{};
  step.iteration=1; step.before=result.error_before;
  step.after=step.gradient=std::numeric_limits<double>::quiet_NaN();
  try {
    const auto t=Clock::now();
    auto trial=std::make_unique<gtsam::ISAM2>(*solver);
    gtsam::ISAM2UpdateParams params;
    params.force_relinearize=true;params.forceFullSolve=true;
    params.noRelinKeys=gtsam::FastList<gtsam::Key>();
    for(auto key:initial.keys())if(!selected.count(key))params.noRelinKeys->push_back(key);
    const auto update=trial->update(gtsam::NonlinearFactorGraph(),gtsam::Values(),params);
    const auto estimate=trial->calculateBestEstimate();
    diagnostic.copy_update_s=Elapsed(t);
    diagnostic.relinearized=step.relinearized=update.variablesRelinearized;
    fgo::offline::RequireSameFactorSlots(graph,trial->getFactorsUnsafe());
    step.after=graph.error(estimate);
    step.state_step_norm=initial.localCoordinates(estimate).norm();
    step.gradient=Certify(graph,estimate,tolerance).maximum;
    const bool ordinary=std::isfinite(step.after)&&step.after>=0&&step.after<=result.error_before;
    bool stable=false;
    if(!ordinary && std::isfinite(step.after)&&step.after>=0) {
      step.cost=fgo::offline::GaussianCostChange(graph,initial,estimate);
      stable=fgo::offline::StableStationaryDescent(step.cost,result.gradient_before,step.gradient,tolerance);
    }
    if(std::isfinite(step.state_step_norm) && step.gradient<=tolerance && (ordinary||stable)) {
      step.accepted=true;step.roundoff=!ordinary;
      result.steps.push_back(step);result.attempted=result.accepted=1;
      result.error_after=step.after;result.gradient_after=step.gradient;result.status="stationary";
      solver.swap(trial);diagnostic.branch="selective_accepted";return result;
    }
    diagnostic.branch="selective_rejected";
  } catch(const std::exception&) { diagnostic.branch="selective_exception"; }
  auto full=fallback(limit-1);
  for(auto& s:full.steps)++s.iteration;
  full.steps.insert(full.steps.begin(),step);++full.attempted;
  return full;
}
inline void Flush(const std::string& out,size_t index,int64_t physical,const std::string& phase) {
  const auto path=std::filesystem::path(out)/"e4_refinement.csv";
  const bool exists=std::filesystem::exists(path);
  std::ofstream f(path,std::ios::app);f<<std::setprecision(17);
  if(!exists)f<<"phase,index,physical_ns,branch,variables,factors,seeds,selected,relinearized,verification_s,selection_s,copy_update_s,fallback_s\n";
  const auto& d=diagnostic;
  f<<phase<<','<<index<<','<<physical<<','<<d.branch<<','<<d.variables<<','<<d.factors<<','<<d.seeds<<','<<d.selected<<','<<d.relinearized<<','<<d.verification_s<<','<<d.selection_s<<','<<d.copy_update_s<<','<<d.fallback_s<<'\n';
}
} // namespace e4
