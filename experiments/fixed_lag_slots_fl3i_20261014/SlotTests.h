#pragma once
#include "../fixed_lag_native_fl3h_20261014/Receiver.h"
// Independent library ISAM2, no copy of candidate elimination as oracle.
class InspectSlotISAM : public fl3i_slots::SlotISAM2 {
 public:
  using fl3i_slots::SlotISAM2::SlotISAM2;
  bool coherent() const {
    gtsam::VariableIndex fresh(nonlinearFactors_);
    if(fresh.size()!=variableIndex_.size()||fresh.nEntries()!=variableIndex_.nEntries())return false;
    for(const auto& [key,indices]:fresh){auto a=indices,b=variableIndex_[key];std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());if(a!=b)return false;}
    if(params_.cacheLinearizedFactors) {
      if(linearFactors_.size()!=nonlinearFactors_.size())return false;
      for(size_t i=0;i<nonlinearFactors_.size();++i) {
        if(bool(nonlinearFactors_[i])!=bool(linearFactors_[i]))return false;
        if(linearFactors_[i]&&linearFactors_[i]->keys()!=nonlinearFactors_[i]->keys())return false;
      }
    }
    return variableIndex_.nFactors()<=nonlinearFactors_.size();
  }
};
void SlotTurnover(bool cache,bool reuse) {
  auto p=fl3i_solver::Parameters();p.cacheLinearizedFactors=cache;p.findUnusedFactorSlots=reuse;
  InspectSlotISAM candidate(p);gtsam::ISAM2 stock(p);
  double stateMax=0,covMax=0,hMax=0;size_t warmSlots=0,peak=0,retirements=0,reused=0;
  for(size_t i=0;i<1040;++i) {
    NonlinearFactorGraph g;Values v;const Key key=X(i);
    v.insert(key,Vector3(.1*i,.02*i,.001*i));
    if(!i)g.emplace_shared<PriorFactor<Vector3>>(key,Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.1));
    else g.emplace_shared<BetweenFactor<Vector3>>(X(i-1),key,Vector3(.1,.02,.001),noiseModel::Isotropic::Sigma(3,.03));
    auto ci=candidate.update(g,v),si=stock.update(g,v);
    // Remove/replace using EACH solver's own slot, not a shared numeric index.
    if(i) {
      auto revised=boost::make_shared<BetweenFactor<Vector3>>(X(i-1),key,Vector3(.10001,.02001,.00101),noiseModel::Isotropic::Sigma(3,.0301));
      candidate.update(NonlinearFactorGraph{revised},{},ci.newFactorsIndices);
      stock.update(NonlinearFactorGraph{revised},{},si.newFactorsIndices);
    }
    if(i>10) {
      FastMap<Key,int> order;for(auto k:candidate.getLinearizationPoint().keys())order[k]=k==X(i-11)?0:1;
      candidate.update({}, {}, {},order);stock.update({}, {}, {},order);
      const auto before=candidate.getFactorsUnsafe();
      FastList<Key> leaves;leaves.push_back(X(i-11));
      FactorIndices added,removed;candidate.marginalizeLeaves(leaves,added,removed);
      stock.marginalizeLeaves(leaves);++retirements;
      for(auto j:removed)Check(j<before.size()&&bool(before[j]),"removed slot identifies old factor");
      for(auto j:added){Check(candidate.getFactorsUnsafe().exists(j),"returned marginal slot exists");Check(dynamic_cast<const LinearContainerFactor*>(candidate.getFactorsUnsafe()[j].get())!=nullptr,"returned marginal slot is raw marginal");reused+=j<before.size();}
      for(size_t j=0;j<before.size();++j)if(before[j]&&std::find(removed.begin(),removed.end(),j)==removed.end())
        Check(candidate.getFactorsUnsafe()[j]==before[j],"surviving source slot and pointer unchanged");
      Check(candidate.coherent(),"variable index and linear cache coherent after retirement");
    }
    const auto a=candidate.calculateEstimate(),b=stock.calculateEstimate();
    Check(a.keys()==b.keys()&&candidate.getFixedVariables()==stock.getFixedVariables(),"retained/fixed keys match stock");
    for(auto k:a.keys())stateMax=std::max(stateMax,(a.at<Vector3>(k)-b.at<Vector3>(k)).norm());
    // Probe covariance/objective periodically, including final iteration.
    if(i%47==0||i==1039) {
      for(auto k:a.keys())covMax=std::max(covMax,(candidate.marginalCovariance(k)-stock.marginalCovariance(k)).cwiseAbs().maxCoeff());
      const auto ca=candidate.getFactorsUnsafe().linearize(candidate.getLinearizationPoint())->augmentedHessian(Ordering(a.keys()));
      const auto cb=stock.getFactorsUnsafe().linearize(stock.getLinearizationPoint())->augmentedHessian(Ordering(b.keys()));
      hMax=std::max(hMax,(ca-cb).norm()/std::max(1.,cb.norm()));
    }
    peak=std::max(peak,candidate.getFactorsUnsafe().size());if(i==100)warmSlots=peak;
  }
  const std::string tag=std::string(cache?"cache":"nocache")+(reuse?"_reuse":"_append");
  Metric(tag+"_state_max",stateMax);Metric(tag+"_covariance_max",covMax);Metric(tag+"_h_relative_max",hMax);
  Metric(tag+"_peak_slots",peak);Metric(tag+"_stock_slots",stock.getFactorsUnsafe().size());Metric(tag+"_retirements",retirements);
  Check(stateMax<=1e-9&&covMax<=1e-10&&hMax<=1e-9,"long-turnover stock state/covariance/objective agreement");
  if(reuse)Check(peak==warmSlots&&peak<=15&&reused==retirements,"slot storage plateaus across more than1000 retirements");
  else Check(reused==0&&peak==stock.getFactorsUnsafe().size(),"no-reuse parameter preserves append behavior");
}

void NonlinearStockReference() {
  f::Receiver candidate(testRoot/"wnoa_candidate",.8);
  fl3h::Receiver stock(testRoot/"wnoa_stock",.8);
  for(int i=0;i<59;++i){auto t=Origin-(58-i)*2500000;candidate.imuClock(t,t);stock.imuClock(t,t);}
  double stateMax=0,covRelative=0;size_t slotPeak=0;
  auto send=[&](const f::Belief& a) {
    fl3h::Belief b;b.source=a.source;b.session=a.session;b.from=a.from;b.to=a.to;
    b.revision=a.revision;b.ordinal=a.ordinal;b.start=a.start;b.end=a.end;
    b.arrival=a.arrival;b.mean=a.mean;b.covariance=a.covariance;
    candidate.belief(a);stock.belief(b);
  };
  for(int i=1;i<=110;++i) {
    auto t=i*f::Period;candidate.imuClock(Origin+t,Origin+t);stock.imuClock(Origin+t,Origin+t);
    if(i>=2){send(Bel("lidar",t-2*f::Period,t,t));send(Bel("camera",t-2*f::Period+23000000,t-f::Period+71000000,t));}
    if(i>=5)send(Bel("lidar",t-5*f::Period,t-3*f::Period,t,1));
    stateMax=std::max(stateMax,Difference(candidate.state().solver.estimate(),stock.state().solver.estimate()));
    const auto& a=candidate.state().solver;const auto& b=stock.state().solver;
    Check(a.isam().getFixedVariables()==b.isam().getFixedVariables(),"nonlinear WNOA fixed-key equivalence");
    if(i%27==0||i==110) {
      const Matrix ca=Cov(a.getFactors(),a.getLinearizationPoint());
      const Matrix cb=Cov(b.getFactors(),b.getLinearizationPoint());
      covRelative=std::max(covRelative,(ca-cb).norm()/std::max(1.,cb.norm()));
      Check((ca-cb).cwiseAbs().maxCoeff()<=1e-10+1e-8*cb.cwiseAbs().maxCoeff(),"nonlinear WNOA complete covariance equivalence");
    }
    slotPeak=std::max(slotPeak,a.getFactors().size());
  }
  Metric("wnoa_state_max",stateMax);Metric("wnoa_covariance_relative",covRelative);Metric("wnoa_peak_slots",slotPeak);
  Check(stateMax<=1e-9&&covRelative<=1e-8,"nonlinear receiver stock equivalence through retirement/revisions");
  Check(slotPeak<40,"nonlinear WNOA slot storage bounded");
  candidate.end(Origin+111*f::Period);stock.end(Origin+111*f::Period);
}
