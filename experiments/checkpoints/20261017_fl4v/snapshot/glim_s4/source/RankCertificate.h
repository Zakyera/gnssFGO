#pragma once
// S4c private numerical certificate. Never used to compute a covariance.
#include <gtsam/base/Matrix.h>
#include <Eigen/SparseCholesky>
#include <cfenv>
#include <cmath>
#include <limits>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace rank_certificate {
using Sparse = Eigen::SparseMatrix<double>;
enum class Reason { Certified=0, Input=1, Environment=2, Scale=3,
                    Factorization=4, Bound=5, Boundary=6 };
struct Result {
  bool certified=false;
  Reason reason=Reason::Input;
  double upper=0, target=0, shift=0, error=0, lower=0, guard=0;
  int factor_nnz=0;
};
inline double up(double x) {
  return std::nextafter(x,std::numeric_limits<double>::infinity());
}
inline double down(double x) {
  return std::nextafter(x,-std::numeric_limits<double>::infinity());
}
inline double add(double a,double b) { return up(a+b); }
inline double mul(double a,double b) { return up(a*b); }
inline double div(double a,double b) { return up(a/b); }
inline double gamma(int k) {
  const double e=k*std::numeric_limits<double>::epsilon();
  return div(e,down(1.-e));
}
inline bool environment() {
#ifdef __FAST_MATH__
  return false;
#else
  if(!std::numeric_limits<double>::is_iec559 || std::fegetround()!=FE_TONEAREST) return false;
#if defined(__SSE__)
  if(_mm_getcsr() & ((1u<<15)|(1u<<6))) return false; // FTZ or DAZ
#endif
  return true;
#endif
}
// Upper bounds on row and column absolute sums. Each nonnegative reduction has
// at most n terms. Inflate for its rounding; account for gradual underflow.
inline std::pair<double,double> norms(const Sparse& a) {
  gtsam::Vector rows=gtsam::Vector::Zero(a.rows()), cols=gtsam::Vector::Zero(a.cols());
  for(int j=0;j<a.outerSize();++j) for(Sparse::InnerIterator it(a,j);it;++it) {
    const double v=std::abs(it.value());
    if(!std::isfinite(v)) return {INFINITY,INFINITY};
    rows[it.row()]+=v; cols[it.col()]+=v;
  }
  const int n=std::max(a.rows(),a.cols());
  const double den=down(1.-gamma(n));
  const double tiny=mul(double(n),std::numeric_limits<double>::min());
  return {div(add(rows.maxCoeff(),tiny),den),div(add(cols.maxCoeff(),tiny),den)};
}

// P A P^T is supplied, not the rounded shifted factorization input. Thus the
// residual includes shift-subtraction rounding and the factorization error.
inline Result verify(const Sparse& pap, const Sparse& l, Result r) {
  const int n=pap.rows();
  if(n==0 || pap.cols()!=n || l.rows()!=n || l.cols()!=n) return r;
  r.reason=Reason::Bound;
  const auto ln=norms(l);
  const double product_bound=mul(ln.first,ln.second);
  if(!std::isfinite(product_bound)) return r;
  const Sparse lt=l.transpose();
  const Sparse product=l*lt; // no pruning/tolerance; each dot has <=n terms
  Sparse residual=pap-product;
  for(int i=0;i<n;++i) residual.coeffRef(i,i)-=r.shift;
  residual.makeCompressed();
  const auto rn=norms(residual);
  const double computed=std::max(rn.first,rn.second);
  // Dot products and two subtractions, with epsilon (2*unit roundoff).
  const double rounding=mul(gamma(2*n+8),add(add(r.upper,r.shift),product_bound));
  const double underflow=mul(4.*n*n+16.*n,std::numeric_limits<double>::min());
  r.error=add(add(computed,rounding),underflow);
  r.lower=down(r.shift-r.error);
  if(!std::isfinite(r.error)||!std::isfinite(r.lower)) return r;
  r.certified=r.lower>add(r.target,r.guard);
  r.reason=r.certified?Reason::Certified:Reason::Boundary;
  r.factor_nnz=l.nonZeros();
  return r;
}

inline Result inspect(const gtsam::Matrix& h) {
  Result r;
  const int n=h.rows();
  if(n<=0 || n>4096 || h.cols()!=n || !h.allFinite()) return r;
  if(!environment()) {r.reason=Reason::Environment;return r;}
  // The authoritative eigensolver reads the lower triangle as self-adjoint.
  gtsam::Matrix a=h.selfadjointView<Eigen::Lower>();
  const double largest=a.cwiseAbs().maxCoeff();
  r.reason=Reason::Scale;
  if(!(largest>0)) return r;
  int exponent=0;std::frexp(largest,&exponent);
  if(exponent < -1022 || exponent > 1023) return r;
  const double inverse_scale=std::ldexp(1.,-exponent);
  if(!std::isfinite(inverse_scale) || inverse_scale==0) return r;
  for(int j=0;j<n;++j) for(int i=0;i<n;++i) {
    const double original=a(i,j);
    const double scaled=std::ldexp(original,-exponent);
    if(!std::isfinite(scaled) || std::ldexp(scaled,exponent)!=original) return r;
    a(i,j)=scaled;
  }
  const Sparse sparse=a.sparseView(0.,0.); // retain every nonzero coefficient
  const auto an=norms(sparse);
  r.upper=std::max(an.first,an.second);
  r.target=mul(1e-12,std::max(inverse_scale,r.upper));
  const double g=mul(gamma(2*n+8),r.upper);
  r.shift=add(r.target,mul(32.,g));
  r.guard=mul(8.,g);
  if(!std::isfinite(r.shift)||!(r.shift>0)||!std::isfinite(r.guard)) return r;
  Sparse shifted=sparse;
  for(int i=0;i<n;++i) shifted.coeffRef(i,i)-=r.shift;
  shifted.makeCompressed();
  Eigen::SimplicialLLT<Sparse,Eigen::Lower,Eigen::AMDOrdering<int>> factor;
  factor.compute(shifted);
  r.reason=Reason::Factorization;
  if(factor.info()!=Eigen::Success) return r;
  const Sparse l=factor.matrixL();
  const Sparse pap=factor.permutationP()*sparse*factor.permutationPinv();
  return verify(pap,l,r);
}
}
