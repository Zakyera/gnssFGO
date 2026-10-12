#include "RankCertificate.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <gtest/gtest.h>
#include <random>
#include <iostream>
#include <iomanip>

namespace shared_native {void probeRank(const gtsam::Matrix&);}
namespace original_rank {void positiveDefinite(const gtsam::Matrix&);}
namespace {
using gtsam::Matrix; using gtsam::Vector; using rank_certificate::Sparse;
bool accepts(void (*f)(const Matrix&),const Matrix& h) {
  try {f(h);return true;} catch(const std::exception&) {return false;}
}
void compare(const Matrix& h) {
  const bool original=accepts(original_rank::positiveDefinite,h);
  EXPECT_EQ(original,accepts(shared_native::probeRank,h));
  const auto r=rank_certificate::inspect(h);
  if(r.certified) {
    EXPECT_TRUE(original);
    int exponent=0;std::frexp(h.cwiseAbs().maxCoeff(),&exponent);
    Matrix scaled=h*std::ldexp(1.,-exponent);
    Eigen::SelfAdjointEigenSolver<Matrix> e(scaled);
    ASSERT_EQ(e.info(),Eigen::Success);
    EXPECT_LE(r.lower,e.eigenvalues().minCoeff());
  }
}
Matrix banded(int n) {
  Matrix h=4*Matrix::Identity(n,n);
  for(int i=1;i<n;++i) h(i,i-1)=h(i-1,i)=-.7;
  for(int i=5;i<n;++i) h(i,i-5)=h(i-5,i)=.12;
  return h;
}
TEST(RankCertificate, DiagonalAndBandedCertified) {
  for(int n:{12,96,765}) for(double scale:{.25,1.,1e6}) {
    const Matrix h=scale*banded(n);
    const auto r=rank_certificate::inspect(h);
    EXPECT_TRUE(r.certified)<<int(r.reason)<<" lower="<<r.lower<<" target="<<r.target;
    compare(h);
  }
}
TEST(RankCertificate, NearThresholdDelegatesToOriginal) {
  for(int n:{12,96}) for(double ratio:{.5,.99,1.,1.01,2.}) {
    Matrix h=Matrix::Identity(n,n);h(0,0)=ratio*1e-12;
    if(ratio<=1.01) EXPECT_FALSE(rank_certificate::inspect(h).certified);
    compare(h);
  }
}
TEST(RankCertificate, PermutedRotatedSparseSPD) {
  for(int n:{12,96}) {
    Matrix q=Matrix::Identity(n,n);
    for(int i=0;i+1<n;i+=2) {q(i,i)=q(i+1,i+1)=.8;q(i,i+1)=.6;q(i+1,i)=-.6;}
    Matrix h=q*banded(n)*q.transpose();h=(.5*(h+h.transpose())).eval();
    const auto r=rank_certificate::inspect(h);EXPECT_TRUE(r.certified);compare(h);
  }
}
TEST(RankCertificate, SingularIndefiniteAndWrongTriangle) {
  for(int n:{12,96}) for(double minimum:{-1.,0.,1e-14}) {
    Matrix h=Matrix::Identity(n,n);h(0,0)=minimum;
    EXPECT_FALSE(rank_certificate::inspect(h).certified);compare(h);
  }
  Matrix asym=banded(12);asym(0,1)+=.1;
  EXPECT_FALSE(accepts(shared_native::probeRank,asym));
}
TEST(RankCertificate, ScaleAndEnvironmentFallback) {
  Matrix h=Matrix::Identity(12,12);h(0,0)=1e300;h(1,1)=1e-300;
  EXPECT_FALSE(rank_certificate::inspect(h).certified);compare(h);
  const int saved=std::fegetround();ASSERT_EQ(std::fesetround(FE_UPWARD),0);
  const auto r=rank_certificate::inspect(banded(12));
  EXPECT_EQ(std::fesetround(saved),0);
  EXPECT_FALSE(r.certified);EXPECT_EQ(r.reason,rank_certificate::Reason::Environment);
}
TEST(RankCertificate, ForgedFactorCannotCertify) {
  Matrix h=banded(12);Sparse a=h.sparseView();Sparse wrong=Matrix::Identity(12,12).sparseView();
  rank_certificate::Result r;r.upper=6;r.target=1e-12;r.shift=1e-10;r.guard=1e-12;
  auto bad=rank_certificate::verify(a,wrong,r);
  EXPECT_FALSE(bad.certified);EXPECT_GT(bad.error,1.);
}
TEST(RankCertificate, RepeatedResultsAndInputUnchanged) {
  Matrix h=banded(96),copy=h;
  const auto first=rank_certificate::inspect(h);ASSERT_TRUE(first.certified);
  for(int k=0;k<5;++k) {
    const auto next=rank_certificate::inspect(h);
    EXPECT_EQ(first.certified,next.certified);EXPECT_EQ(first.lower,next.lower);
    EXPECT_EQ(first.error,next.error);EXPECT_EQ(first.factor_nnz,next.factor_nnz);
  }
  EXPECT_EQ((h-copy).norm(),0.);
}
TEST(RankCertificate, RandomSparseFamiliesMatchOriginal) {
  std::mt19937 rng(12345);std::uniform_real_distribution<double> u(-.1,.1);
  size_t fast=0;
  for(int n:{12,32,96}) for(int sample=0;sample<12;++sample) {
    Matrix l=Matrix::Identity(n,n);
    for(int i=0;i<n;++i) for(int j=std::max(0,i-8);j<i;++j) l(i,j)=u(rng);
    Matrix h=l*l.transpose();
    if(sample%3==0)h(0,0)-=2.;
    compare(h);fast+=rank_certificate::inspect(h).certified;
  }
  EXPECT_EQ(fast,24u);
}
TEST(RankCertificate, NormBoundsAndAnalyticResidual) {
  Matrix a=Matrix::Zero(3,3);a.diagonal()<<2.,3.,4.;
  const auto n=rank_certificate::norms(Sparse(a.sparseView()));
  EXPECT_GE(n.first,4.);EXPECT_GE(n.second,4.);
  Matrix l=Matrix::Identity(3,3);l(0,0)=2.;
  rank_certificate::Result r;r.upper=6;r.target=1e-12;r.shift=.1;r.guard=1e-12;
  const auto check=rank_certificate::verify(Sparse(a.sparseView()),Sparse(l.sparseView()),r);
  // Exact residual is diag(-2.1,1.9,2.9).
  EXPECT_GE(check.error,2.9);EXPECT_FALSE(check.certified);
}
}
