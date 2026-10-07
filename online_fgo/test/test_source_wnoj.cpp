#include <gtest/gtest.h>
#include <set>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include "offline_process/SourceWnojModel.h"

namespace {
using namespace fgo::offline;
gtsam::SharedNoiseModel Qc() {
  return gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000));
}
gtsam::Matrix6 Covariance() {
  gtsam::Matrix6 l = gtsam::Matrix6::Identity() * 0.04;
  l(3, 0) = 0.015; l(5, 1) = -0.012; l(4, 3) = 0.02;
  return l * l.transpose();
}
gtsam::Values Perturb(const gtsam::Values& values, gtsam::Key key, int c, double d) {
  auto result = values;
  gtsam::Vector delta = gtsam::Vector::Zero(values.at(key).dim()); delta[c] = d;
  if (gtsam::Symbol(key).chr() == 'x') {
    result.update(key, values.at<gtsam::Pose3>(key).retract(delta));
  } else if (delta.size() == 3) {
    result.update(key, gtsam::Vector3(values.at<gtsam::Vector3>(key) + delta));
  } else {
    result.update(key, gtsam::Vector6(values.at<gtsam::Vector6>(key) + delta));
  }
  return result;
}
double JacobianDifference(const gtsam::NoiseModelFactor& factor, const gtsam::Values& values) {
  std::vector<gtsam::Matrix> h(factor.keys().size()); factor.unwhitenedError(values, h);
  double difference = 0;
  for (size_t k = 0; k < h.size(); ++k) {
    for (int c = 0; c < h[k].cols(); ++c) {
      auto plus = Perturb(values, factor.keys()[k], c, 1e-6);
      auto minus = Perturb(values, factor.keys()[k], c, -1e-6);
      const gtsam::Vector numerical = (factor.unwhitenedError(plus) -
        factor.unwhitenedError(minus)) / 2e-6;
      difference = std::max(difference, (numerical - h[k].col(c)).cwiseAbs().maxCoeff());
    }
  }
  return difference;
}
SourceWnojState GeneralState() {
  SourceWnojState s;
  s.pose = gtsam::Pose3::Expmap((gtsam::Vector6() << .2,-.1,.15,1,2,-.5).finished());
  s.velocity << .6,-.4,.1; s.omega << .1,.2,-.15;
  s.acceleration << .02,.03,-.01,.4,-.1,.2;
  return s;
}
TEST(SourceWnoj, StockPriorNumericalPathAndZeroNoisePrediction) {
  auto i = GeneralState(), j = PredictSourceWnoj(i, .1);
  gtsam::Values values; InsertSourceWnojState(values, 0, i); InsertSourceWnojState(values, 1, j);
  fgo::factor::GPWNOJPriorFull factor(X(0),V(0),W(0),A(0),X(1),V(1),W(1),A(1),.1,Qc(),true,true);
  EXPECT_LT(factor.unwhitenedError(values).norm(), 1e-12);
  values = Perturb(values, X(1), 0, .07);
  values = Perturb(values, A(1), 4, .3);
  const auto difference = JacobianDifference(factor, values);
  std::cout << "full_prior_jacobian_max=" << difference << '\n';
  EXPECT_LT(difference, 2e-6);
  // Audit (not use or alter) the optional stock analytic branch separately.
  fgo::factor::GPWNOJPriorFull analytic(X(0),V(0),W(0),A(0),X(1),V(1),W(1),A(1),.1,Qc(),false,true);
  std::cout << "unused_analytic_prior_jacobian_max=" << JacobianDifference(analytic, values) << '\n';
}
TEST(SourceWnoj, EndpointsSharedKeysDenseCovarianceAndJacobians) {
  gtsam::Values values; auto s = GeneralState();
  for (size_t i = 0; i < 4; ++i) { InsertSourceWnojState(values, i, s); s = PredictSourceWnoj(s,.1); }
  auto exact = [](size_t i) {return FullWnojEndpoint::Exact(i);};
  auto mid = [](size_t i, double tau) {return FullWnojEndpoint::Between(i,i+1,.1,tau,Qc());};
  std::vector<std::pair<FullWnojEndpoint,FullWnojEndpoint>> cases{
    {exact(0),exact(1)}, {exact(0),mid(0,.071)}, {mid(0,.023),exact(1)},
    {mid(0,.023),mid(0,.071)}, {mid(0,.071),mid(1,.023)}, {mid(0,.023),mid(2,.071)}};
  const std::vector<size_t> counts{2,8,8,8,12,16};
  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& pair = cases[i];
    auto z = pair.first.pose(values).between(pair.second.pose(values));
    z = z.retract((gtsam::Vector6() << .03,-.02,.01,.04,.02,-.01).finished());
    FullWnojRelativeFactor factor(pair.first,pair.second,z,
      gtsam::noiseModel::Gaussian::Covariance(Covariance(),false));
    EXPECT_EQ(factor.keys().size(),counts[i]);
    EXPECT_EQ(std::set<gtsam::Key>(factor.keys().begin(),factor.keys().end()).size(),counts[i]);
    auto expected = z.localCoordinates(pair.first.pose(values).between(pair.second.pose(values)));
    EXPECT_LT((factor.unwhitenedError(values)-expected).norm(),1e-12);
    EXPECT_LT(JacobianDifference(factor,values),2e-6) << "endpoint case " << i;
    const auto noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel());
    EXPECT_LT((noise->covariance()-Covariance()).norm(),1e-12);
    EXPECT_NEAR(factor.error(values),.5*expected.dot(Covariance().ldlt().solve(expected)),1e-10);
    EXPECT_NEAR(factor.clone()->error(values),factor.error(values),1e-12);
  }
  EXPECT_LT(gtsam::Pose3::Logmap(mid(0,0).pose(values).between(values.at<gtsam::Pose3>(X(0)))).norm(),1e-12);
  EXPECT_LT(gtsam::Pose3::Logmap(mid(0,.1).pose(values).between(values.at<gtsam::Pose3>(X(1)))).norm(),1e-12);
  // Sensitivity to each latent A: these are real optimizer variables.
  auto endpoint = mid(0,.05);
  std::vector<gtsam::Matrix> h;
  const auto p = endpoint.pose(values, &h);
  double interp_difference = 0;
  for (size_t k=0;k<h.size();++k) {
    for (int c=0;c<h[k].cols();++c) {
      const auto plus=endpoint.pose(Perturb(values,endpoint.keys()[k],c,1e-6));
      const auto minus=endpoint.pose(Perturb(values,endpoint.keys()[k],c,-1e-6));
      const gtsam::Vector6 numerical=(p.localCoordinates(plus)-p.localCoordinates(minus))/2e-6;
      interp_difference=std::max(interp_difference,(numerical-h[k].col(c)).cwiseAbs().maxCoeff());
    }
  }
  std::cout << "full_interpolator_jacobian_max=" << interp_difference << '\n';
  EXPECT_LT(interp_difference,2e-6);
  EXPECT_GT(endpoint.pose(values).localCoordinates(endpoint.pose(Perturb(values,A(0),3,1))).norm(),1e-5);
  EXPECT_THROW(mid(0,.2),std::invalid_argument);
}
TEST(SourceWnoj, StationaryVelocityAndAccelerationOptimization) {
  for (int scenario=0; scenario<3; ++scenario) {
    SourceWnojState s;
    if (scenario>0) s.velocity << .5,.2,-.1;
    if (scenario>1) s.acceleration << 0,0,0,.3,-.1,.2;
    const auto initial=s;
    gtsam::Values truth;
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0),s.pose,gtsam::noiseModel::Isotropic::Sigma(6,.01));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0),s.velocity,gtsam::noiseModel::Isotropic::Sigma(3,1));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(0),s.omega,gtsam::noiseModel::Isotropic::Sigma(3,1));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector6>>(A(0),s.acceleration,gtsam::noiseModel::Isotropic::Sigma(6,1));
    for(size_t i=0;i<4;++i) {
      InsertSourceWnojState(truth,i,s);
      EXPECT_NEAR(s.pose.x(),initial.velocity.x()*.1*i + .5*initial.acceleration[3]*.01*i*i,1e-12);
      if(i) {
        graph.emplace_shared<fgo::factor::GPWNOJPriorFull>(X(i-1),V(i-1),W(i-1),A(i-1),X(i),V(i),W(i),A(i),.1,Qc(),true,true);
        graph.emplace_shared<FullWnojRelativeFactor>(FullWnojEndpoint::Exact(i-1),FullWnojEndpoint::Exact(i),
          truth.at<gtsam::Pose3>(X(i-1)).between(s.pose),gtsam::noiseModel::Gaussian::Covariance(Covariance(),false));
      }
      s=PredictSourceWnoj(s,.1);
    }
    EXPECT_LT(graph.error(truth),1e-15);
    auto guess=truth;
    for(size_t i=1;i<4;++i) guess=Perturb(guess,X(i),3,.1);
    gtsam::LevenbergMarquardtParams params; params.setMaxIterations(100);
    params.setAbsoluteErrorTol(1e-14); params.setRelativeErrorTol(1e-14);
    auto result=gtsam::LevenbergMarquardtOptimizer(graph,guess,params).optimize();
    EXPECT_LT(graph.error(result),1e-10);
    gtsam::Marginals marginals(graph,result,gtsam::Marginals::QR);
    EXPECT_TRUE(marginals.marginalCovariance(A(3)).allFinite());
    const auto covariance=marginals.jointMarginalCovariance(result.keys()).fullMatrix();
    EXPECT_TRUE(covariance.allFinite());
    EXPECT_EQ(covariance.llt().info(),Eigen::Success);
    auto repeat=gtsam::LevenbergMarquardtOptimizer(graph,guess,params).optimize();
    gtsam::Marginals repeated_marginals(graph,repeat,gtsam::Marginals::QR);
    EXPECT_EQ((covariance-repeated_marginals.jointMarginalCovariance(repeat.keys()).fullMatrix()).norm(),0);
  }
}
}  // namespace
