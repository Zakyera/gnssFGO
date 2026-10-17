#include "NativeGaussianHistory.h"
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <iomanip>
#include <random>

using namespace gtsam;
using namespace shared_native;
namespace {
Key X(int n) { return Symbol('x',n); }
Key V(int n) { return Symbol('v',n); }
Key B(int n) { return Symbol('b',n); }
std::vector<Key> stateKeys(int n) { return {X(n),V(n),B(n)}; }
int dim(Key k) { return Symbol(k).chr() == 'v' ? 3 : 6; }
Matrix randomMatrix(int rows, int cols, unsigned seed) {
  std::mt19937 rng(seed); std::normal_distribution<double> normal(0,1);
  Matrix m(rows,cols);
  for (int r=0;r<rows;++r) for (int c=0;c<cols;++c) m(r,c)=normal(rng);
  return m;
}
struct Raw {
  std::vector<Key> keys;
  Matrix J;
  Vector b;
  GaussianFactor::shared_ptr factor() const {
    std::vector<std::pair<Key,Matrix>> terms;
    int col=0;
    for (auto k:keys) {terms.emplace_back(k,J.middleCols(col,dim(k))); col+=dim(k);}
    return std::make_shared<JacobianFactor>(terms,b);
  }
};
Raw prior(int n) {
  Matrix j = 2*Matrix::Identity(15,15)+.1*randomMatrix(15,15,17+n);
  return {stateKeys(n),j,.2*randomMatrix(15,1,30+n)};
}
Raw motion(int a,int b) {
  auto keys=stateKeys(a); const auto other=stateKeys(b);
  keys.insert(keys.end(),other.begin(),other.end());
  Matrix j(15,30);
  j.leftCols(15)=-Matrix::Identity(15,15)+.03*randomMatrix(15,15,80+a);
  j.rightCols(15)=Matrix::Identity(15,15)+.03*randomMatrix(15,15,120+b);
  return {keys,j,.3*randomMatrix(15,1,260+b)};
}
Raw lidar(int a,int b) {
  Matrix j(6,12);
  j.leftCols(6)=-2*Matrix::Identity(6,6)+.04*randomMatrix(6,6,43+a);
  j.rightCols(6)=2*Matrix::Identity(6,6)+.04*randomMatrix(6,6,87+b);
  return {{X(a),X(b)},j,.1*randomMatrix(6,1,92+b)};
}
void values(Values& t,int n) {
  Vector6 xi; xi<<.12*n,-.08*n,.15*n,1.+n,.7*n,-.3*n;
  t.insert(X(n),Pose3::Expmap(xi));
  t.insert(V(n),Vector3(.1*n,-.2,.03));
  t.insert(B(n),imuBias::ConstantBias(Vector3(.01,.02,.03),Vector3(.04,.01,-.02)));
}
std::map<Key,int> offsets(const KeyVector& keys) {
  std::map<Key,int> off; int col=0;
  for(auto key:keys){off[key]=col;col+=dim(key);}return off;
}
int dimensions(const KeyVector& keys) { int n=0;for(auto k:keys)n+=dim(k);return n; }
Matrix assemble(const std::map<Id,Raw>& raw, const KeyVector& keys) {
  auto off=offsets(keys);int n=dimensions(keys);Matrix full=Matrix::Zero(n+1,n+1);
  for(const auto& f:raw){
    Matrix j=Matrix::Zero(f.second.J.rows(),n);int col=0;
    for(auto k:f.second.keys){j.middleCols(off.at(k),dim(k))=f.second.J.middleCols(col,dim(k));col+=dim(k);}
    full.topLeftCorner(n,n)+=j.transpose()*j;
    full.topRightCorner(n,1)+=j.transpose()*f.second.b;
    full.bottomRightCorner(1,1)(0,0)+=f.second.b.squaredNorm();
  }
  full.bottomLeftCorner(1,n)=full.topRightCorner(n,1).transpose();return full;
}
Matrix select(const Matrix& m,const std::vector<int>& rows,const std::vector<int>& cols){
  Matrix r(rows.size(),cols.size());for(size_t i=0;i<rows.size();++i)for(size_t j=0;j<cols.size();++j)r(i,j)=m(rows[i],cols[j]);return r;
}
Matrix denseSchur(const Matrix& full,const KeyVector& all,const KeySet& retired) {
  std::vector<int> gone,kept;auto off=offsets(all);
  for(auto k:all)for(int i=0;i<dim(k);++i)(retired.count(k)?gone:kept).push_back(off.at(k)+i);
  kept.push_back(full.rows()-1);
  if(gone.empty())return full;
  Matrix hmm=select(full,gone,gone), hmr=select(full,gone,kept);
  return select(full,kept,kept)-hmr.transpose()*hmm.ldlt().solve(hmr);
}
void compare(const std::string& name,const Matrix& actual,const Matrix& expected,double tol=1e-9){
  ASSERT_EQ(actual.rows(),expected.rows());ASSERT_EQ(actual.cols(),expected.cols());
  const double absolute=(actual-expected).norm();const double relative=absolute/std::max(1e-30,expected.norm());
  std::cout<<std::setprecision(17)<<"NUMERIC "<<name<<" abs="<<absolute<<" relative="<<relative<<"\n";
  EXPECT_LE(relative,tol)<<name;
}
struct Fixture {
  Snapshot s;
  std::map<Id,Raw> raw;
  KeyVector all;
  KeySet retired;
  Fixture(){
    for(int n=0;n<3;++n)values(s.theta,n);
    all=s.theta.keys();
    add(1,prior(0)); add(2,motion(0,1)); add(3,motion(1,2));
    add(4,lidar(0,2)); add(5,lidar(1,2));
  }
  void add(Id id,const Raw& f){size_t slot=s.cache.size();s.cache.push_back(f.factor());s.native.push_back({id,1,slot});raw[id]=f;}
  Matrix expected()const{return denseSchur(assemble(raw,all),all,retired);}
  void accept(NativeGaussianHistory& h){ASSERT_TRUE(h.stage(s).ok);ASSERT_TRUE(h.commit(s.generation,s.epoch).ok);}
};
// Avoid constructing a set from iterators into two temporary key vectors.
void fixedAll(Snapshot& s){const auto keys=s.theta.keys();s.fixed_after=KeySet(keys.begin(),keys.end());}
void retire(Fixture& f,int n){for(auto k:stateKeys(n)){f.s.retiring.insert(k);f.retired.insert(k);}fixedAll(f.s);}
void compareGraph(NativeGaussianHistory& h,Fixture& f){
  auto actual=h.graph(f.s.epoch).augmentedHessian();
  compare("augmented_native_including_eta_constant",actual,f.expected());
  auto info=actual.topLeftCorner(actual.rows()-1,actual.cols()-1).eval();
  Eigen::SelfAdjointEigenSolver<Matrix> eigen(info);
  std::cout<<"CONDITION "<<eigen.eigenvalues().maxCoeff()/eigen.eigenvalues().minCoeff()<<"\n";
  EXPECT_EQ(h.admittedCount(),h.activeCount()+h.absorbedCount()+h.removedCount());
}
}

TEST(NativeGaussian, DisabledByDefault){NativeGaussianHistory h;Fixture f;EXPECT_FALSE(h.stage(f.s).ok);EXPECT_THROW(h.graph(1),std::exception);}
TEST(NativeGaussian, CopyThroughAndCovariance){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);compareGraph(h,f);
  auto actual=h.graph(1);for(size_t i=0;i<f.s.native.size();++i)EXPECT_TRUE(actual[i]->augmentedInformation().isApprox(f.s.cache[i]->augmentedInformation(),0));
  auto cov=h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero());
  const Matrix augmented=f.expected();const int n=augmented.rows()-1;
  Matrix full=augmented.topLeftCorner(n,n).ldlt().solve(Matrix::Identity(n,n));
  auto off=offsets(f.all);std::vector<int> indices;
  for(auto k:{X(1),X(2)})for(int i=0;i<6;++i)indices.push_back(off[k]+i);
  compare("endpoint_joint_covariance",cov.joint_theta,select(full,indices,indices),1e-8);
  EXPECT_GT(cov.joint_theta.topRightCorner(6,6).norm(),1e-3);
  EXPECT_GT(full.block(off[X(1)],off[V(1)],6,3).norm(),1e-4);
  EXPECT_GT(full.block(off[X(1)],off[B(1)],6,6).norm(),1e-4);
}
TEST(NativeGaussian, NativeOnlyHistoryExcludesExternalEvenAfterRetirement){
  Fixture f,g;f.s.cache.push_back(lidar(0,1).factor());Raw external=lidar(0,1);external.J*=1000;
  g.s.cache.push_back(external.factor()); // Same class as native; not positively admitted.
  NativeGaussianHistory a(Mode::Shadow),b(Mode::Shadow);retire(f,0);retire(g,0);f.accept(a);g.accept(b);
  compareGraph(a,f);EXPECT_TRUE(a.graph(1).augmentedHessian().isApprox(b.graph(1).augmentedHessian(),0));
  // Copy-on-ingestion: later mutation/replacement of MAIN cache cannot change history.
  f.s.cache[2]=external.factor();EXPECT_TRUE(a.graph(1).augmentedHessian().isApprox(b.graph(1).augmentedHessian(),0));
}
TEST(NativeGaussian, MultipleRetirementsVersionsAndNonincidentNoDoubleCount){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);retire(f,0);f.accept(h);compareGraph(h,f);
  EXPECT_EQ(h.absorbedCount(),3u);EXPECT_EQ(h.activeCount(),2u);
  f.s.epoch=2;f.s.retiring.clear();f.s.theta=h.theta(1);values(f.s.theta,3);
  auto newkeys=stateKeys(3);f.all.insert(f.all.end(),newkeys.begin(),newkeys.end());std::sort(f.all.begin(),f.all.end());
  f.s.native.erase(std::remove_if(f.s.native.begin(),f.s.native.end(),[](const NativeRef& r){return r.id==1||r.id==2||r.id==4;}),f.s.native.end());
  f.raw[5].J*=1.2;f.raw[5].b*=.8;f.s.cache[4]=f.raw[5].factor();f.s.native.back().version=2;
  f.add(6,motion(2,3));f.add(7,lidar(1,3));retire(f,1);f.accept(h);compareGraph(h,f);
  EXPECT_EQ(h.absorbedCount(),6u);EXPECT_EQ(h.activeCount(),1u);
  f.s.epoch=3;f.s.theta=h.theta(2);f.s.retiring.clear();f.s.native={{6,1,5}};
  values(f.s.theta,4);newkeys=stateKeys(4);f.all.insert(f.all.end(),newkeys.begin(),newkeys.end());std::sort(f.all.begin(),f.all.end());
  f.add(8,motion(3,4));retire(f,2);f.accept(h);compareGraph(h,f);
  auto cov=h.covariance(3,X(3),X(4),Vector6::Zero(),Vector6::Zero());
  auto remaining=h.theta(3).keys();auto off=offsets(remaining);Matrix aug=f.expected();int n=aug.rows()-1;
  Matrix full=aug.topLeftCorner(n,n).ldlt().solve(Matrix::Identity(n,n));std::vector<int> ix;
  for(auto k:{X(3),X(4)})for(int i=0;i<6;++i)ix.push_back(off[k]+i);
  compare("retired_joint_covariance",cov.joint_theta,select(full,ix,ix),1e-8);
}
TEST(NativeGaussian, RemovalSlotReuseAndNoResurrection){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);
  f.s.epoch=2;f.s.removed={5};f.s.native.pop_back();f.raw.erase(5);
  Raw replacement=lidar(0,1);f.s.cache[4]=replacement.factor();f.s.native.push_back({6,1,4});f.raw[6]=replacement;
  f.accept(h);compareGraph(h,f);EXPECT_EQ(h.removedCount(),1u);
  f.s.epoch=3;f.s.removed.clear();f.s.native.back()={5,2,4};EXPECT_FALSE(h.stage(f.s).ok);EXPECT_EQ(h.epoch(),2u);
}
TEST(NativeGaussian, StaleVersionsAndUndeclaredMutationRejected){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.s.native[2].version=3;f.accept(h);
  f.s.epoch=2;f.s.native[2].version=2;EXPECT_FALSE(h.stage(f.s).ok);
  f.s.native[2].version=3;f.raw[3].J*=2;f.s.cache[2]=f.raw[3].factor();EXPECT_FALSE(h.stage(f.s).ok);
  f.s.native[2].version=4;EXPECT_TRUE(h.stage(f.s).ok);h.abort();EXPECT_EQ(h.epoch(),1u);
}
TEST(NativeGaussian, MissingDuplicateAndInvalidDimensionsRejected){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);auto original=f.s;
  f.s.cache[0].reset();EXPECT_FALSE(h.stage(f.s).ok);f.s=original;
  f.s.native.push_back(f.s.native[0]);EXPECT_FALSE(h.stage(f.s).ok);f.s=original;
  f.s.native[1].slot=0;EXPECT_FALSE(h.stage(f.s).ok);f.s=original;
  f.s.theta.erase(V(0));f.s.theta.insert(V(0),Vector2::Zero().eval());EXPECT_FALSE(h.stage(f.s).ok);
}
TEST(NativeGaussian, NonfiniteRejected){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.raw[1].J(0,0)=std::numeric_limits<double>::quiet_NaN();
  f.s.cache[0]=f.raw[1].factor();EXPECT_FALSE(h.stage(f.s).ok);EXPECT_EQ(h.epoch(),0u);
}
TEST(NativeGaussian, PriorChartAndFixedKeyGuards){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);retire(f,0);f.accept(h);
  f.s.epoch=2;f.s.theta=h.theta(1);f.s.retiring.clear();f.s.native={{3,1,2},{5,1,4}};fixedAll(f.s);
  auto original=f.s.theta.at<Pose3>(X(1));Vector6 d=Vector6::Zero();d(0)=1e-3;
  f.s.theta.update(X(1),original.retract(d));EXPECT_FALSE(h.stage(f.s).ok);
  f.s.theta.update(X(1),original);f.s.fixed_after.clear();EXPECT_FALSE(h.stage(f.s).ok);
  fixedAll(f.s);EXPECT_TRUE(h.stage(f.s).ok);
}
TEST(NativeGaussian, NewPriorMustBeFixed){Fixture f;NativeGaussianHistory h(Mode::Shadow);retire(f,0);f.s.fixed_after.clear();EXPECT_FALSE(h.stage(f.s).ok);}
TEST(NativeGaussian, RefreshAllowedOnlyOutsidePrior){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);f.s.epoch=2;
  Vector6 d=Vector6::Constant(.03);f.s.theta.update(X(1),f.s.theta.at<Pose3>(X(1)).retract(d));
  EXPECT_FALSE(h.stage(f.s).ok);for(auto& r:f.s.native)++r.version;EXPECT_TRUE(h.stage(f.s).ok);
}
TEST(NativeGaussian, TransactionRollbackAndReset){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);Matrix before=h.graph(1).augmentedHessian();
  f.s.epoch=2;retire(f,0);EXPECT_TRUE(h.stage(f.s).ok);EXPECT_FALSE(h.stage(f.s).ok);
  EXPECT_FALSE(h.commit(1,3).ok);EXPECT_TRUE(h.graph(1).augmentedHessian().isApprox(before,0));
  h.abort();EXPECT_EQ(h.epoch(),1u);EXPECT_THROW(h.graph(2),std::exception);
  EXPECT_TRUE(h.reset(2).ok);EXPECT_EQ(h.epoch(),0u);EXPECT_THROW(h.graph(1),std::exception);
  EXPECT_FALSE(h.stage(f.s).ok);Fixture rebuilt;rebuilt.s.generation=2;rebuilt.accept(h);compareGraph(h,rebuilt);
  EXPECT_FALSE(h.reset(2).ok);
}
TEST(NativeGaussian, UnknownRemovalAndSkippedEpoch){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);f.s.epoch=3;EXPECT_FALSE(h.stage(f.s).ok);
  f.s.epoch=2;f.s.native.pop_back();EXPECT_FALSE(h.stage(f.s).ok);f.s.removed={88};EXPECT_FALSE(h.stage(f.s).ok);
}
TEST(NativeGaussian, UnderconstrainedCovarianceFailsWithoutAnchor){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.s.native.erase(f.s.native.begin());
  f.accept(h);EXPECT_THROW(h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()),std::exception);
}
TEST(NativeGaussian, ConstantTermSurvivesCompleteComponentElimination){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);
  // Keep a separate anchored component after eliminating all initial keys.
  values(f.s.theta,3);auto k=stateKeys(3);f.all.insert(f.all.end(),k.begin(),k.end());std::sort(f.all.begin(),f.all.end());
  f.add(6,prior(3));for(int i=0;i<3;++i)retire(f,i);f.accept(h);compareGraph(h,f);
  EXPECT_GT(f.expected().bottomRightCorner(1,1)(0,0),0.001);
}
TEST(NativeGaussian, NonzeroDeltaTransportAndRelativeFiniteDifference){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);
  Vector6 df,dt;df<<.31,-.22,.19,.7,-.3,.4;dt<<-.17,.12,.27,-.2,.8,.3;
  auto result=h.covariance(1,X(1),X(2),df,dt);Matrix num=Matrix::Zero(12,12);const double step=1e-6;
  Pose3 tf=f.s.theta.at<Pose3>(X(1)),tt=f.s.theta.at<Pose3>(X(2));Pose3 pf=tf.retract(df),pt=tt.retract(dt);
  for(int i=0;i<6;++i){Vector6 e=Vector6::Zero();e(i)=step;
    num.block<6,1>(0,i)=(pf.localCoordinates(tf.retract(df+e))-pf.localCoordinates(tf.retract(df-e)))/(2*step);
    num.block<6,1>(6,6+i)=(pt.localCoordinates(tt.retract(dt+e))-pt.localCoordinates(tt.retract(dt-e)))/(2*step);
  }
  EXPECT_LT((num-result.transport).norm(),1e-7);compare("current_chart_covariance",result.joint_current,num*result.joint_theta*num.transpose(),1e-8);
  Matrix j(6,12);Pose3 z=pf.between(pt);
  for(int i=0;i<6;++i){Vector6 e=Vector6::Zero();e(i)=step;
    j.col(i)=(z.localCoordinates(pf.retract(e).between(pt))-z.localCoordinates(pf.retract(-e).between(pt)))/(2*step);
    j.col(6+i)=(z.localCoordinates(pf.between(pt.retract(e)))-z.localCoordinates(pf.between(pt.retract(-e))))/(2*step);
  }
  compare("relative_schur_vs_numeric_propagation",result.relative,j*result.joint_current*j.transpose(),1e-8);
  EXPECT_GT((result.joint_current-result.joint_theta).norm(),.01);
  auto swapped=h.covariance(1,X(2),X(1),dt,df);compare("reversed_endpoint_order",swapped.joint_theta.topLeftCorner(6,6),result.joint_theta.bottomRightCorner(6,6),1e-8);
}
TEST(NativeGaussian, RepeatedRunIsExact){
  Fixture f;NativeGaussianHistory a(Mode::Shadow),b(Mode::Shadow);retire(f,0);f.accept(a);f.accept(b);
  EXPECT_TRUE(a.graph(1).augmentedHessian().isApprox(b.graph(1).augmentedHessian(),0));
  EXPECT_TRUE(a.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()).relative.isApprox(
      b.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()).relative,0));
}
TEST(NativeGaussian, MixedHessianAndWeightedJacobianNotWhitenedTwice){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);
  f.s.cache[0]=std::make_shared<HessianFactor>(*std::dynamic_pointer_cast<JacobianFactor>(f.s.cache[0]));
  auto unweighted=std::dynamic_pointer_cast<JacobianFactor>(f.s.cache[1]);
  Vector sigma=Vector::LinSpaced(15,.5,1.8);unweighted->get_model()=noiseModel::Diagonal::Sigmas(sigma);
  f.raw[2].J=sigma.cwiseInverse().asDiagonal()*f.raw[2].J;
  f.raw[2].b=sigma.cwiseInverse().asDiagonal()*f.raw[2].b;
  retire(f,0);f.accept(h);compareGraph(h,f);
}
TEST(NativeGaussian, DeepCopyProtectsAgainstInPlaceCacheMutation){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);auto before=h.graph(1).augmentedHessian();
  auto ptr=std::dynamic_pointer_cast<JacobianFactor>(f.s.cache[2]);
  Raw altered=f.raw[3];altered.J*=17;
  *ptr=*std::dynamic_pointer_cast<JacobianFactor>(altered.factor());
  EXPECT_TRUE(h.graph(1).augmentedHessian().isApprox(before,0));
}
TEST(NativeGaussian, FailureContainedAndOutputPreserved){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);
  EndpointCovariance result;ASSERT_TRUE(h.tryCovariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero(),result).ok);
  auto original=result.relative;
  EXPECT_FALSE(h.tryCovariance(2,X(1),X(2),Vector6::Zero(),Vector6::Zero(),result).ok);
  EXPECT_FALSE(h.tryCovariance(1,X(1),X(1),Vector6::Zero(),Vector6::Zero(),result).ok);
  EXPECT_FALSE(h.tryCovariance(1,X(1),X(99),Vector6::Zero(),Vector6::Zero(),result).ok);
  EXPECT_TRUE(original.isApprox(result.relative,0));
  auto before=h.graph(1).augmentedHessian();f.s.epoch=2;f.s.retiring={X(999)};
  EXPECT_FALSE(h.stage(f.s).ok);EXPECT_TRUE(h.graph(1).augmentedHessian().isApprox(before,0));
}
TEST(NativeGaussian, ConstrainedFactorsRejected){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);
  std::dynamic_pointer_cast<JacobianFactor>(f.s.cache[0])->get_model()=noiseModel::Constrained::All(15);
  EXPECT_FALSE(h.stage(f.s).ok);
}
TEST(NativeGaussian, AllKeysRetiredQueryFailsSafely){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);for(int i=0;i<3;++i)retire(f,i);
  f.accept(h);EXPECT_EQ(h.theta(1).size(),0u);EXPECT_EQ(h.activeCount(),0u);
  EndpointCovariance output;
  EXPECT_FALSE(h.tryCovariance(1,X(0),X(1),Vector6::Zero(),Vector6::Zero(),output).ok);
  compare("all_eliminated_constant",h.graph(1).augmentedHessian(),f.expected());
}
TEST(NativeGaussian, GaussianEliminationFailureRollsBack){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);f.accept(h);
  const Matrix before=h.graph(1).augmentedHessian();f.s.epoch=2;
  f.raw[1].J.setZero();f.s.cache[0]=f.raw[1].factor();f.s.native[0].version=2;
  f.s.native.erase(std::remove_if(f.s.native.begin(),f.s.native.end(),
      [](const NativeRef& r){return r.id==2||r.id==4;}),f.s.native.end());
  f.s.removed={2,4};retire(f,0);
  const auto status=h.stage(f.s);EXPECT_FALSE(status.ok);
  EXPECT_TRUE(h.graph(1).augmentedHessian().isApprox(before,0));EXPECT_EQ(h.absorbedCount(),0u);
  EXPECT_EQ(h.removedCount(),0u);EXPECT_EQ(h.epoch(),1u);
}
TEST(NativeGaussian, ChangedNativeEquationsCanChangeCovariance){
  Fixture f,g;NativeGaussianHistory a(Mode::Shadow),b(Mode::Shadow);f.accept(a);
  // An external factor can indirectly change MAIN's theta/correspondences/J.
  // This fixture changes the native equations to represent that boundary; it
  // does not pretend to model a nonlinear external-factor optimization.
  g.raw[4].J*=1.3;g.s.cache[3]=g.raw[4].factor();g.accept(b);
  EXPECT_GT((a.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()).relative-
      b.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()).relative).norm(),1e-4);
}
TEST(NativeGaussian, TwelveRetirementsKeepBoundedHistory){
  Fixture f;NativeGaussianHistory h(Mode::Shadow);Id nextid=6;
  for(int step=0;step<12;++step){
    if(step){
      f.s.theta=h.theta(step);f.s.epoch=step+1;f.s.retiring.clear();
      f.s.native.erase(std::remove_if(f.s.native.begin(),f.s.native.end(),[&](const NativeRef& r){
        for(auto key:f.raw.at(r.id).keys)if(f.retired.count(key))return true;return false;
      }),f.s.native.end());
      int n=step+2;values(f.s.theta,n);auto keys=stateKeys(n);
      f.all.insert(f.all.end(),keys.begin(),keys.end());std::sort(f.all.begin(),f.all.end());
      f.add(nextid++,motion(n-1,n));f.add(nextid++,lidar(n-2,n));
    }
    retire(f,step);f.accept(h);compareGraph(h,f);
    EXPECT_LE(h.activeCount(),2u);EXPECT_EQ(h.theta(step+1).size(),6u);
    EXPECT_LE(h.graph(step+1).size(),3u);
  }
  std::cout<<"BOUND retained_values="<<h.theta(12).size()<<" active_factors="<<h.activeCount()
           <<" absorbed="<<h.absorbedCount()<<" admitted="<<h.admittedCount()<<"\n";
}
