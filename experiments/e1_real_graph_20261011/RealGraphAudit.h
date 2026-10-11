#pragma once
// Included ONLY by the private diagnostic copy of cbs_bridge.cpp.
#include <glim_ros/cbs_local_graph_linearizer.hpp>
#include <cbs/utils/relative_pose_covariance.h>
#include <gtsam/nonlinear/Marginals.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>

namespace e1_real {
struct Pair {
  gtsam::Key a,b;
  gtsam::Pose3 pa,pb;
};
inline size_t selectedCall() {
  static size_t count=0;
  ++count;
  for(size_t selected : {10u,30u,60u,100u,150u,180u}) if(count==selected) return count;
  return 0;
}
inline void save(const std::filesystem::path& p,const gtsam::Matrix& m) {
  std::ofstream f(p); f.exceptions(std::ios::badbit|std::ios::failbit);
  f << std::setprecision(17) << m << '\n';
}
struct Difference {
  double max_abs=0,max_fraction=0;
  bool pass=true;
  void add(const gtsam::Matrix& a,const gtsam::Matrix& b,double atol) {
    if(a.rows()!=b.rows() || a.cols()!=b.cols() || !a.allFinite() || !b.allFinite())
      throw std::runtime_error("E1 dimension/nonfinite failure");
    const double delta=(a-b).cwiseAbs().maxCoeff();
    const double bound=atol+1e-6*a.cwiseAbs().maxCoeff();
    max_abs=std::max(max_abs,delta); max_fraction=std::max(max_fraction,delta/bound);
    pass=pass && delta<=bound;
  }
};
inline gtsam::GaussianFactorGraph::shared_ptr compare(
    size_t sample,double stamp,const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,const std::vector<Pair>& pairs) {
  const char* root_env=std::getenv("GLIM_E1_REAL_AUDIT_ROOT");
  if(!root_env || !*root_env) throw std::runtime_error("E1 diagnostic root unset");
  const std::filesystem::path root(root_env), dir=root/("sample_"+std::to_string(sample));
  try {
    if(!std::filesystem::create_directory(dir)) throw std::runtime_error("E1 sample exists");
    const gtsam::Values before(values);
    const auto keys=values.keys();
    if(pairs.empty()) throw std::runtime_error("E1 no outgoing pairs");
    std::ofstream structure(dir/"structure.txt");
    structure << std::setprecision(17) << "stamp " << stamp << "\nkeys ";
    for(auto k:keys) structure << gtsam::DefaultKeyFormatter(k) << ' ';
    structure << "\n";
    size_t gpu_count=0,pending=0;
    for(size_t i=0;i<graph.size();++i) {
      structure << i << ' ';
      if(!graph[i]) {structure << "null\n"; continue;}
      structure << typeid(*graph[i]).name();
      for(auto k:graph[i]->keys()) structure << ' ' << gtsam::DefaultKeyFormatter(k);
      structure << '\n';
      if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(graph[i].get())) {
        ++gpu_count; pending+=gpu->has_cached_linearization();
      }
    }
    structure.close();
    if(!gpu_count || pending) throw std::runtime_error("E1 requires real uncached GPU factors");
    using Clock=std::chrono::steady_clock;
    auto elapsed=[](auto start) {return std::chrono::duration<double,std::milli>(Clock::now()-start).count();};
    auto start=Clock::now(); auto a=graph.linearize(values); const double a_ms=elapsed(start);
    start=Clock::now(); auto b=glim::linearizeCbsLocalGraphBatched(graph,values); const double b_ms=elapsed(start);
    start=Clock::now(); auto a2=graph.linearize(values); const double a2_ms=elapsed(start);
    if(a->size()!=graph.size() || b->size()!=graph.size() || a2->size()!=graph.size())
      throw std::runtime_error("E1 slots changed");
    for(size_t i=0;i<graph.size();++i) for(auto* linear : {a.get(),b.get(),a2.get()})
      if(bool((*linear)[i])!=bool(graph[i]) ||
         (graph[i] && (*linear)[i]->keys()!=graph[i]->keys())) throw std::runtime_error("E1 keys changed");
    Difference h,p,r,repeated;
    const auto ha=a->augmentedHessian(),hb=b->augmentedHessian(),ha2=a2->augmentedHessian();
    h.add(ha,hb,1e-8); repeated.add(ha,ha2,1e-8);
    save(dir/"hessian_A.txt",ha); save(dir/"hessian_B.txt",hb); save(dir/"hessian_A2.txt",ha2);
    start=Clock::now(); gtsam::Marginals ma(*a,values);
    const auto ca=ma.jointMarginalCovariance(keys).fullMatrix(); const double ca_ms=elapsed(start);
    start=Clock::now(); gtsam::Marginals mb(*b,values);
    const auto cb=mb.jointMarginalCovariance(keys).fullMatrix(); const double cb_ms=elapsed(start);
    gtsam::Marginals ma2(*a2,values); const auto ca2=ma2.jointMarginalCovariance(keys).fullMatrix();
    p.add(ca,cb,1e-10); repeated.add(ca,ca2,1e-10);
    save(dir/"covariance_A.txt",ca); save(dir/"covariance_B.txt",cb); save(dir/"covariance_A2.txt",ca2);
    size_t n=0;
    for(const auto& pair:pairs) {
      auto covariance=[&](gtsam::Marginals& m) {
        return cbs::relative_pose_covariance::compute(m.jointMarginalInformation({pair.a,pair.b}),
          pair.a,pair.b,pair.pa,pair.pb,pair.pa.between(pair.pb),
          cbs::relative_pose_covariance::Mode::SchurRelativeBetween).covariance;
      };
      const auto ra=covariance(ma),rb=covariance(mb),ra2=covariance(ma2);
      r.add(ra,rb,1e-10); repeated.add(ra,ra2,1e-10);
      save(dir/("relative_A_"+std::to_string(n)+".txt"),ra);
      save(dir/("relative_B_"+std::to_string(n)+".txt"),rb);
      save(dir/("main_from_"+std::to_string(n)+".txt"),pair.pa.matrix());
      save(dir/("main_to_"+std::to_string(n)+".txt"),pair.pb.matrix());
      ++n;
    }
    const bool pass=h.pass && p.pass && r.pass && repeated.pass && values.equals(before,0);
    std::ofstream out(dir/"summary.json"); out << std::setprecision(17)
      << "{\"sample\":" << sample << ",\"stamp\":" << stamp << ",\"passed\":" << (pass?"true":"false")
      << ",\"slots\":" << graph.size() << ",\"keys\":" << keys.size() << ",\"gpu_factors\":" << gpu_count
      << ",\"joint_dimension\":" << ca.rows() << ",\"pairs\":" << pairs.size()
      << ",\"hessian_max_abs\":" << h.max_abs << ",\"hessian_bound_fraction\":" << h.max_fraction
      << ",\"joint_cov_max_abs\":" << p.max_abs << ",\"joint_cov_bound_fraction\":" << p.max_fraction
      << ",\"relative_cov_max_abs\":" << r.max_abs << ",\"relative_cov_bound_fraction\":" << r.max_fraction
      << ",\"ordinary_repeat_bound_fraction\":" << repeated.max_fraction
      << ",\"linearize_A_ms\":" << a_ms << ",\"linearize_B_ms\":" << b_ms << ",\"linearize_A2_ms\":" << a2_ms
      << ",\"joint_query_A_ms\":" << ca_ms << ",\"joint_query_B_ms\":" << cb_ms
      << ",\"values_unchanged\":" << (values.equals(before,0)?"true":"false") << "}\n";
    out.close();
    if(!pass) throw std::runtime_error("E1 strict matrix gate failed");
    return a; // NEVER publish the candidate matrix in this diagnostic.
  } catch(const std::exception& e) {
    std::ofstream(root/"FAILED.txt") << "sample=" << sample << " " << e.what() << '\n';
    throw;
  }
}
} // namespace e1_real
