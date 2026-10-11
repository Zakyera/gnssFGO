#pragma once
#include "offline_process/BoundedISAM2Refinement.h"
// Diagnostic-only fork: no solution produced here is fed back into the receiver.
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Eigenvalues>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <map>
#include <set>
#include <sstream>
#include "SourceWnoaModel.h"

namespace snapshot_audit {
using namespace gtsam::symbol_shorthand;
inline bool FullSelected(size_t index) {
  const char* requested = std::getenv("WNOJ_AUDIT_INDICES");
  if (!requested) return false;
  std::istringstream ss(requested); std::string item;
  while (std::getline(ss, item, ',')) if (std::stoul(item) == index) return true;
  return false;
}
inline bool Selected(size_t index) {
  return std::getenv("AUDIT_RECORD_ALL") || FullSelected(index);
}
inline void States(const std::filesystem::path& path, const gtsam::Values& values,
                   const std::vector<int64_t>& times, size_t last) {
  std::ofstream out(path); out << std::setprecision(17);
  out << "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
  for (size_t i=0; i<=last; ++i) {
    const auto s=fgo::offline::ReadSourceWnoaState(values,i);
    const auto q=s.pose.rotation().toQuaternion();
    out << i << ',' << times[i] << ',' << s.pose.x() << ',' << s.pose.y() << ',' << s.pose.z()
        << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
    for(double x:s.velocity) out << ',' << x;
    for(double x:s.omega) out << ',' << x;
    out << '\n';
  }
}
inline std::string Kind(size_t slot, const gtsam::NonlinearFactor::shared_ptr& f,
                        const std::map<size_t,std::string>& sources) {
  if (sources.count(slot)) return sources.at(slot);
  if (dynamic_cast<const fgo::factor::GPWNOAPrior*>(f.get())) return "wnoa";
  return "prior";
}
inline void FactorErrors(const std::filesystem::path& path,
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::map<size_t,std::string>& sources) {
  std::ofstream out(path); out << std::setprecision(17);
  out << "slot,kind,dimension,half_squared_whitened_error,keys\n";
  for(size_t slot=0;slot<graph.size();++slot) if(graph[slot]) {
    out << slot << ',' << Kind(slot,graph[slot],sources) << ',' << graph[slot]->dim()
        << ',' << graph[slot]->error(values) << ',';
    for(auto k:graph[slot]->keys()) out << gtsam::DefaultKeyFormatter(k) << ';';
    out << '\n';
  }
}
inline void DensePoses(const std::filesystem::path& path,const gtsam::Values& values,
                       const std::vector<int64_t>& times,size_t last) {
  // Read-only sampling of the SAME stock conditional-mean interpolation used
  // by the source factors. Not a new motion model or optimization constraint.
  auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  std::ofstream out(path); out << std::setprecision(17)
    << "timestamp_ns,px,py,pz,qx,qy,qz,qw\n";
  auto write=[&](int64_t t,const gtsam::Pose3& p) {
    const auto q=p.rotation().toQuaternion();
    out << t << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  };
  for(size_t i=0;i<last;++i) {
    const auto dt=times[i+1]-times[i];
    for(int64_t offset=0;offset<dt;offset+=10000000) {
      auto endpoint=fgo::offline::WnoaEndpoint::Between(i,i+1,dt*1e-9,offset*1e-9,qc);
      write(times[i]+offset,endpoint.pose(values));
    }
  }
  write(times[last],values.at<gtsam::Pose3>(X(last)));
}
inline std::string Fingerprint(const gtsam::NonlinearFactorGraph& graph,
                               const gtsam::Values& fixed_values) {
  // Fixed-point payload probe includes all factor keys, dense covariance and residuals.
  // The serialized string is hashed outside C++ and checked unchanged after batch.
  std::ostringstream out; out << std::setprecision(17);
  for(size_t i=0;i<graph.size();++i) if(graph[i]) {
    out << i << ':' << typeid(*graph[i]).name() << ':';
    for(auto k:graph[i]->keys()) out << k << ',';
    auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[i]);
    if(!nf) throw std::runtime_error("expected noise factor");
    auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(nf->noiseModel());
    if(!noise) throw std::runtime_error("expected Gaussian noise");
    out << '\n' << noise->covariance() << '\n' << nf->unwhitenedError(fixed_values) << '\n';
  }
  return out.str();
}
inline void LinearHealth(const std::filesystem::path& path,
    const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& values) {
  // Same gradient metric, sparse blocks; omit the expensive dense spectrum.
  const auto linear=graph.linearize(values);
  const auto gradient=linear->gradientAtZero();
  const auto diagonal=linear->hessianDiagonal();
  double raw=0.,scaled=0.;size_t cols=0,rows=0;
  for(const auto& f:graph) if(f) rows+=f->dim();
  for(const auto& [key,g]:gradient) {
    const auto& d=diagonal.at(key);cols+=g.size();
    if(!g.allFinite()||!d.allFinite()||(d.array()<0).any())throw std::runtime_error("bad diagnostic linear system");
    raw=std::max(raw,g.cwiseAbs().maxCoeff());
    for(int i=0;i<g.size();++i) {
      if(d[i]==0&&g[i]!=0)throw std::runtime_error("zero information with nonzero gradient");
      if(d[i]>0)scaled=std::max(scaled,std::abs(g[i])/std::sqrt(d[i]));
    }
  }
  std::ofstream out(path);out<<std::setprecision(17)
    <<"rows,cols,gradient_inf,column_normalized_gradient_inf\n"
    <<rows<<','<<cols<<','<<raw<<','<<scaled<<'\n';
}
inline void DeepChecks(const std::filesystem::path& dir,const gtsam::ISAM2& receiver,
    const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& before,
    const gtsam::Values& batch,const std::vector<int64_t>& times,size_t index,
    const std::map<size_t,std::string>& sources) {
  // An independent optimizer copy only: no new factors, no production mutation.
  gtsam::ISAM2 extra(receiver);
  std::ofstream out(dir/"extra_incremental_iterations.csv"); out << std::setprecision(17)
    << "iteration,half_squared_whitened_error,newest_speed,best_estimate_error\n";
  for(size_t n=0;n<=20;++n) {
    if(n) extra.update(gtsam::NonlinearFactorGraph(),gtsam::Values(),gtsam::FactorIndices(),
                      boost::none,boost::none,boost::none,true);
    const auto current=extra.calculateEstimate();
    // Full back-substitution diagnostic runs on a separate copy, not this chain.
    gtsam::ISAM2 full(extra);
    const auto best=full.calculateBestEstimate();
    out << n << ',' << graph.error(current) << ',' << current.at<gtsam::Vector3>(V(index)).norm()
        << ',' << graph.error(best) << '\n';
    if(n==20) States(dir/"extra_incremental_states.csv",current,times,index);
  }
  gtsam::Marginals marginals(graph,batch,gtsam::Marginals::QR);
  std::ofstream cov(dir/"marginal_covariance.csv");cov << std::setprecision(17)
    << "key,row,column,value\n";
  for(size_t i=index>4?index-4:0;i<=index;++i) {
    for(auto key:{X(i),V(i),W(i)}) {
      const auto p=marginals.marginalCovariance(key);
      for(int r=0;r<p.rows();++r) for(int c=0;c<p.cols();++c)
        cov << gtsam::DefaultKeyFormatter(key) << ',' << r << ',' << c << ',' << p(r,c) << '\n';
    }
  }
  // Check every factor in these two small actual-state snapshots against
  // an independent finite-difference step. Uses the same manifold retract.
  std::ofstream jac(dir/"jacobian_check.csv");jac << std::setprecision(17)
    << "state,slot,kind,max_absolute_difference,max_scaled_difference\n";
  for(const auto& [label,values]:std::vector<std::pair<std::string,gtsam::Values>>{{"before",before},{"batch",batch}}) {
    for(size_t slot=0;slot<graph.size();++slot) if(graph[slot]) {
      auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[slot]);
      std::vector<gtsam::Matrix> h(f->keys().size());f->unwhitenedError(values,h);
      double absolute=0,scaled=0;
      for(size_t k=0;k<f->keys().size();++k) {
        const auto key=f->keys()[k];
        for(int c=0;c<values.at(key).dim();++c) {
          auto delta=values.zeroVectors();const double eps=2e-6;
          delta.at(key)[c]=eps;const auto plus=f->unwhitenedError(values.retract(delta));
          delta.at(key)[c]=-eps;const auto minus=f->unwhitenedError(values.retract(delta));
          const gtsam::Vector numeric=(plus-minus)/(2*eps);
          absolute=std::max(absolute,(numeric-h[k].col(c)).cwiseAbs().maxCoeff());
          scaled=std::max(scaled,((numeric-h[k].col(c)).array().abs()/(1+numeric.array().abs())).maxCoeff());
        }
      }
      jac << label << ',' << slot << ',' << Kind(slot,graph[slot],sources) << ',' << absolute << ',' << scaled << '\n';
    }
  }
}
inline void RunProfile(const std::filesystem::path& dir,
    const gtsam::NonlinearFactorGraph& original, const gtsam::Values& initial,
    const std::vector<int64_t>& times, size_t index, int64_t available,
    const std::map<size_t,std::string>& sources,const gtsam::ISAM2& receiver) {
  std::filesystem::create_directories(dir);
  // Preserve actual factor objects and slots. No graph reconstruction or new prior.
  gtsam::NonlinearFactorGraph graph;
  for(const auto& f:original) graph.push_back(f ? f->clone() : nullptr);
  const auto fingerprint=Fingerprint(graph,initial);
  if(fingerprint!=Fingerprint(original,initial)) throw std::runtime_error("clone changed factors");
  {std::ofstream out(dir/"factor_payload_probe.txt"); out << fingerprint;}
  States(dir/"incremental_states.csv",initial,times,index);
  DensePoses(dir/"incremental_dense_poses.csv",initial,times,index);
  FactorErrors(dir/"incremental_errors.csv",graph,initial,sources);
  LinearHealth(dir/"incremental_linear_health.csv",graph,initial);
  gtsam::ISAM2 full_copy(receiver);
  const auto full=full_copy.calculateBestEstimate();
  States(dir/"full_backsub_states.csv",full,times,index);
  FactorErrors(dir/"full_backsub_errors.csv",graph,full,sources);
  LinearHealth(dir/"full_backsub_linear_health.csv",graph,full);
  auto params=gtsam::LevenbergMarquardtParams::CeresDefaults();
  params.setLinearSolverType("MULTIFRONTAL_QR");
  params.maxIterations=500; params.relativeErrorTol=1e-11; params.absoluteErrorTol=1e-11;
  std::ofstream log(dir/"iterations.csv"); log << std::setprecision(17) << "phase,iteration,error_before,error_after\n";
  int phase=1;
  params.iterationHook=[&](size_t n,double before,double after) {
    log << phase << ',' << n << ',' << before << ',' << after << '\n'; log.flush();
  };
  const auto start=std::chrono::steady_clock::now();
  gtsam::LevenbergMarquardtOptimizer optimizer(graph,initial,params);
  const gtsam::Values batch=optimizer.optimize();
  States(dir/"batch_states.csv",batch,times,index);
  FactorErrors(dir/"batch_errors.csv",graph,batch,sources);
  phase=2; params.maxIterations=100;
  gtsam::LevenbergMarquardtOptimizer polish(graph,batch,params);
  const gtsam::Values result=polish.optimize();
  States(dir/"polished_states.csv",result,times,index);
  DensePoses(dir/"polished_dense_poses.csv",result,times,index);
  {
    std::ofstream residuals(dir/"source_residuals.csv"); residuals << std::setprecision(17)
      << "slot,kind,r0,r1,r2,r3,r4,r5,squared_whitened_residual\n";
    for(const auto& [slot,kind]:sources) {
      const auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.at(slot));
      const auto r=f->unwhitenedError(result);
      if(r.size()!=6 || !r.allFinite()) throw std::runtime_error("invalid source residual");
      residuals << slot << ',' << kind;
      for(double x:r) residuals << ',' << x;
      residuals << ',' << 2*f->error(result) << '\n';
    }
  }
  FactorErrors(dir/"polished_errors.csv",graph,result,sources);
  LinearHealth(dir/"polished_linear_health.csv",graph,result);
  { std::ofstream cert(dir/"batch_stationarity.csv");cert<<std::setprecision(17)
      <<"batch_gradient,polished_gradient,batch_stationary,polished_stationary\n";
    const double a=fgo::offline::NormalizedGradient(graph,batch),b=fgo::offline::NormalizedGradient(graph,result);
    cert<<a<<','<<b<<','<<(a<=1e-5)<<','<<(b<=1e-5)<<'\n'; }

  if(std::getenv("WNOJ_AUDIT_DEEP")) DeepChecks(dir,receiver,graph,initial,result,times,index,sources);
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  if(fingerprint!=Fingerprint(graph,initial) || fingerprint!=Fingerprint(original,initial))
    throw std::runtime_error("diagnostic mutated factors");
  std::ofstream summary(dir/"summary.csv"); summary << std::setprecision(17)
    << "index,physical_ns,available_ns,variables,non_null_factors,initial_error,batch_error,polished_error,batch_iterations,polish_iterations,wall_seconds\n"
    << index << ',' << times[index] << ',' << available << ',' << initial.size() << ','
    << graph.nrFactors() << ',' << graph.error(initial) << ',' << graph.error(batch) << ','
    << graph.error(result) << ',' << optimizer.iterations() << ',' << polish.iterations() << ',' << seconds << '\n';
  std::cout << "AUDIT snapshot=" << index << " objective=" << graph.error(initial)
            << " -> " << graph.error(result) << " newest_speed="
            << initial.at<gtsam::Vector3>(V(index)).norm() << " -> "
            << result.at<gtsam::Vector3>(V(index)).norm() << std::endl;
}

inline void Run(const std::filesystem::path& dir,
    const gtsam::NonlinearFactorGraph& original,const gtsam::Values& initial,
    const std::vector<int64_t>& times,size_t index,int64_t available,
    const std::map<size_t,std::string>& sources,const gtsam::ISAM2& receiver) {
  if(!FullSelected(index) && dir.filename()!="snapshot_final" && !std::getenv("AUDIT_FAILED_PUBLICATION") && !std::getenv("AUDIT_ROUNDOFF_PUBLICATION")) {
    States(dir/"incremental_states.csv",initial,times,index);
    FactorErrors(dir/"incremental_errors.csv",original,initial,sources);
    std::ofstream summary(dir/"summary.csv");summary<<std::setprecision(17)
      <<"index,physical_ns,available_ns,variables,non_null_factors,initial_error\n"
      <<index<<','<<times[index]<<','<<available<<','<<initial.size()<<','<<original.nrFactors()<<','<<original.error(initial)<<'\n';
    return;
  }
  // Audit the graph actually built by the receiver; NEVER replace its Qc here.
  const auto original_probe=Fingerprint(original,initial);
  const std::string profile=std::getenv("WNOJ_INCREMENTAL_PROFILE") ?
    std::getenv("WNOJ_INCREMENTAL_PROFILE") : "baseline";
  gtsam::Vector6 v=(gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished();
  if(profile=="nominal") v << .00375,.00375,.00375,.015,.015,.015;
  const gtsam::Matrix6 qc=v.asDiagonal();
  double max_cov_error=0.;size_t count=0;
  for(const auto& f:original) {
    const auto motion=boost::dynamic_pointer_cast<fgo::factor::GPWNOAPrior>(f);
    if(!motion) continue;
    const auto& keys=f->keys();
    const size_t i=gtsam::Symbol(keys[0]).index(),j=gtsam::Symbol(keys[3]).index();
    const auto expected=fgo::utils::calcQ<6>(qc,(times.at(j)-times.at(i))*1e-9);
    const auto actual=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(motion->noiseModel())->covariance();
    if(expected.llt().info()!=Eigen::Success) throw std::runtime_error("non-SPD covariance");
    max_cov_error=std::max(max_cov_error,(actual-expected).norm()/expected.norm());++count;
  }
  if(count!=index || max_cov_error>1e-10) throw std::runtime_error("receiver process covariance mismatch");
  {std::ofstream f(dir/"profile_validation.csv");f<<std::setprecision(17)
    <<"profile,q_rot,q_trans,motion_factors,max_covariance_relative_error\n"
    <<profile<<','<<v[0]<<','<<v[3]<<','<<count<<','<<max_cov_error<<'\n';}
  RunProfile(dir,original,initial,times,index,available,sources,receiver);
  if(original_probe!=Fingerprint(original,initial)) throw std::runtime_error("audit mutated receiver");
}
} // namespace snapshot_audit
