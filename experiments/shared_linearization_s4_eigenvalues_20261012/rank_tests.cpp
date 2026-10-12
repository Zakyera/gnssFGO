#include <gtsam/base/Matrix.h>
#include <Eigen/QR>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <random>
#include <string>

namespace shared_native { void probeRank(const gtsam::Matrix&); }
namespace original_rank { void positiveDefinite(const gtsam::Matrix&); }
namespace {
using gtsam::Matrix;
using gtsam::Vector;
std::string result(void (*f)(const Matrix&), const Matrix& h) {
  try { f(h); return "accepted"; }
  catch (const std::exception& e) { return e.what(); }
}
std::string compare(const Matrix& h) {
  const auto original = result(original_rank::positiveDefinite, h);
  EXPECT_EQ(original, result(shared_native::probeRank, h));
  return original;
}
Matrix rotation(int n, unsigned seed) {
  std::mt19937 rng(seed); std::normal_distribution<double> normal;
  Matrix m(n,n);
  for (int r=0;r<n;++r) for(int c=0;c<n;++c) m(r,c)=normal(rng);
  Eigen::HouseholderQR<Matrix> qr(m);
  return qr.householderQ()*Matrix::Identity(n,n);
}
Matrix denseSpectrum(const Matrix& q, double minimum, double maximum) {
  Vector eig = Vector::LinSpaced(q.rows(), minimum, maximum);
  Matrix h = q * eig.asDiagonal() * q.transpose();
  return (0.5*(h+h.transpose())).eval();
}
TEST(EigenvaluesOnlyRank, DiagonalStrictThresholdAndAdjacentRepresentableValues) {
  for (int n : {1, 12, 96}) for (double scale : {0.25, 1., 1.e6}) {
    // n=1 has a different max eigenvalue after assigning the sole entry.
    if(n==1 && scale>1.) continue;
    const double threshold=1.e-12*std::max(1.,scale);
    for (int sign : {-1,0,1}) {
      Matrix h=scale*Matrix::Identity(n,n);
      h(0,0)=sign==0?threshold:std::nextafter(threshold, sign<0?0.:std::numeric_limits<double>::infinity());
      const auto outcome=compare(h);
      EXPECT_EQ(outcome=="accepted",sign>0) << n << " " << scale << " " << sign;
    }
  }
}
TEST(EigenvaluesOnlyRank, InvalidInputsPreserveFailureReason) {
  EXPECT_EQ(compare(Matrix(0,0)),"empty native information");
  EXPECT_EQ(compare(Matrix::Zero(3,2)),"nonfinite Gaussian equations");
  Matrix h=Matrix::Identity(12,12);h(0,1)=0.1;
  EXPECT_EQ(compare(h),"asymmetric Gaussian equations");
  for(double value : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    h=Matrix::Identity(12,12);h(0,0)=value;
    EXPECT_EQ(compare(h),"nonfinite Gaussian equations");
  }
}
TEST(EigenvaluesOnlyRank, SingularAndIndefinitePreserveRejection) {
  for (int n : {12,96}) {
    const Matrix q=rotation(n,100+n);
    for(double minimum : {-1.,0.,1.e-14})
      EXPECT_NE(compare(denseSpectrum(q,minimum,1.)),"accepted");
  }
}
TEST(EigenvaluesOnlyRank, RotatedThresholdNeighborhoodPreservesDecision) {
  for(int n : {12,96}) for(unsigned seed : {17u,31u,67u}) {
    const Matrix q=rotation(n,seed);
    for(double scale : {0.25,1.,1.e6}) for(double ratio : {0.5,0.99,1.,1.01,2.}) {
      const double minimum=1.e-12*std::max(1.,scale)*ratio;
      compare(denseSpectrum(q,minimum,scale));
    }
  }
}
TEST(EigenvaluesOnlyRank, RepresentativeLargeDenseSystems) {
  const Matrix q=rotation(765,812);
  for(double minimum : {1.e-14,1.e-9,0.25}) {
    const auto outcome=compare(denseSpectrum(q,minimum,1.));
    EXPECT_EQ(outcome=="accepted",minimum>1.e-12);
  }
}
TEST(EigenvaluesOnlyRank, SymmetryBoundaryUnchanged) {
  for(double error : {1.e-14,1.e-9}) {
    Matrix h=Matrix::Identity(12,12);h(0,1)=error;
    EXPECT_EQ(compare(h)=="accepted",error<1.e-12);
  }
}
}
