// Audit/reference utilities ONLY. No replacement optimizer or production factor.
#pragma once
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <memory>
#include <cmath>

namespace fl2 {
using namespace gtsam;
inline Values Subset(const Values& v, const KeyVector& keys) {
  Values out; for(Key k:keys) out.insert(k,v.at(k)); return out;
}
inline Values Perturb(const Values& v, Key k, size_t j, double h) {
  Values out(v); Vector d=Vector::Zero(v.at(k).dim()); d[j]=h;
  std::unique_ptr<Value> value(v.at(k).retract_(d)); out.update(k,*value); return out;
}

// Re-express the stored joint Gaussian in the current Values chart for checking.
// This factor is NEVER inserted into the candidate's iSAM2.
class ChartMarginal final : public NonlinearFactor {
 public:
  GaussianFactor::shared_ptr gaussian;
  Values anchor;
  Matrix A; Vector b;
  ChartMarginal(const GaussianFactor::shared_ptr& f,const Values& a)
      :NonlinearFactor(f->keys()),gaussian(f),anchor(Subset(a,f->keys())) {
    const auto ab=f->jacobian(); A=ab.first; b=ab.second;
  }
  Vector delta(const Values& x) const {
    Vector out(A.cols()); size_t c=0;
    for(Key k:keys()) { const auto d=anchor.at(k).localCoordinates_(x.at(k));
      out.segment(c,d.size())=d; c+=d.size(); }
    return out;
  }
  size_t dim() const override { return A.rows(); }
  double error(const Values& x) const override {return .5*(A*delta(x)-b).squaredNorm();}
  GaussianFactor::shared_ptr linearize(const Values& x) const override {
    std::vector<std::pair<Key,Matrix>> terms; size_t c=0;
    for(Key k:keys()) {
      const size_t n=x.at(k).dim(); Matrix D(n,n);
      for(size_t j=0;j<n;++j) {
        const auto plus=Perturb(x,k,j,1e-6),minus=Perturb(x,k,j,-1e-6);
        D.col(j)=(anchor.at(k).localCoordinates_(plus.at(k))-
                  anchor.at(k).localCoordinates_(minus.at(k)))/2e-6;
      }
      terms.emplace_back(k,A.middleCols(c,n)*D); c+=n;
    }
    return boost::make_shared<JacobianFactor>(terms,b-A*delta(x));
  }
  NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<ChartMarginal>(*this);
  }
};
inline double Gradient(const NonlinearFactorGraph& g,const Values& v) {
  auto linear=g.linearize(v); const auto grad=linear->gradientAtZero();
  const auto diag=linear->hessianDiagonal(); double result=0.;
  for(const auto& kv:grad) for(Eigen::Index i=0;i<kv.second.size();++i)
    result=std::max(result,std::abs(kv.second[i])/std::sqrt(std::max(1e-30,diag.at(kv.first)[i])));
  return result;
}
} // namespace fl2
