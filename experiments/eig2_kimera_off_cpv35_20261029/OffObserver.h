#pragma once
#include "LinearParityAudit.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <filesystem>
#include <fstream>
#include <iomanip>

// Passive private observer. Never updates, replaces, or relinearizes LIVE factors.
namespace cpv35 {
using namespace gtsam;
inline bool target(double u) {
  return u<=30 || int(u)%200==0 || (u>=1329 && u<=1341) ||
      (u>=1408 && u<=1412) || (u>=1465 && u<=1469);
}
inline void values(std::ostream& out, const Values& x) {
  out<<std::setprecision(17);
  for(const auto& kv:x) {
    const Symbol s(kv.key);out<<DefaultKeyFormatter(kv.key);
    if(s.chr()=='x') {
      const auto p=x.at<Pose3>(kv.key);const auto q=p.rotation().toQuaternion();
      out<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z();
    } else if(s.chr()=='v') {
      const auto v=x.at<Vector3>(kv.key);for(int i=0;i<3;++i)out<<','<<v[i];
    } else if(s.chr()=='b') {
      const auto b=x.at<imuBias::ConstantBias>(kv.key).vector();for(int i=0;i<6;++i)out<<','<<b[i];
    } else throw std::runtime_error("CPV35 unexpected state type");
    out<<'\n';
  }
}
inline void capture(double u, const char* phase, const cbs::BPSAM& solver) {
  const char* root=std::getenv("CPV35_AUDIT_DIR");if(!root||!target(u))return;
  const auto dir=std::filesystem::path(root)/("update_"+std::to_string(int(u)))/phase;
  if(!std::filesystem::create_directories(dir))throw std::runtime_error("CPV35 duplicate snapshot");
  const auto estimate=solver.calculateEstimate();const auto theta=solver.getLinearizationPoint();
  std::ofstream state(dir/"state.csv"),anchor(dir/"theta.csv"),fixed(dir/"fixed_keys.txt"),cost(dir/"factor_costs.csv"),imu(dir/"imu_residuals.csv");
  values(state,estimate);values(anchor,theta);
  for(Key k:solver.getFixedVariables())fixed<<DefaultKeyFormatter(k)<<'\n';
  cost<<std::setprecision(17)<<"slot,family,keys,error_at_state,error_at_theta\n";
  imu<<std::setprecision(17)<<"slot,dt,rotation_x,rotation_y,rotation_z,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z\n";
  const auto live=solver.getFactorsUnsafe();
  const Eigen::IOFormat fmt(17,Eigen::DontAlignCols,",","\n");
  for(size_t i=0;i<live.size();++i) {
    if(!live[i])continue;
    auto a=live[i]->clone(),b=live[i]->clone();
    if(a.get()==live[i].get()||b.get()==live[i].get())throw std::runtime_error("CPV35 clone alias");
    cost<<i<<','<<fl4n::family(*a)<<','<<a->keys().size()<<','
        <<cbs::fixedLagFactorErrorInChart(*a,estimate,theta)<<','
        <<cbs::fixedLagFactorErrorInChart(*b,theta,theta)<<'\n';
    if(auto f=dynamic_cast<const ImuFactor*>(a.get())) {
      const auto r=f->unwhitenedError(estimate);imu<<i<<','<<f->preintegratedMeasurements().deltaTij();
      for(int j=0;j<r.size();++j)imu<<','<<r[j];imu<<'\n';
    }
    if(auto p=dynamic_cast<const LinearContainerFactor*>(a.get())) {
      std::ofstream h(dir/("prior_"+std::to_string(i)+"_augmented.csv")),keys(dir/("prior_"+std::to_string(i)+"_keys.csv"));
      h<<p->factor()->augmentedInformation().format(fmt)<<'\n';
      for(auto k=p->factor()->begin();k!=p->factor()->end();++k)keys<<DefaultKeyFormatter(*k)<<','<<p->factor()->getDim(k)<<'\n';
      if(p->hasLinearizationPoint())throw std::runtime_error("CPV35 expected native unanchored OFF prior");
      if(!h||!keys)throw std::runtime_error("CPV35 prior write failed");
    }
  }
  if(!state||!anchor||!fixed||!cost||!imu)throw std::runtime_error("CPV35 snapshot write failed");
  std::ofstream done(dir/"complete.txt");done<<"copied_factor_observer\n";
}
}
