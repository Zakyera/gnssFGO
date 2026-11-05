#include "Receiver.h"
#include <random>
#include <iostream>
int main(){
 using namespace gtsam;using namespace fgo::offline;
 try {
 Vector6 q;q<<.00375,.00375,.00375,.015,.015,.015;
 const SharedNoiseModel base=noiseModel::Diagonal::Variances(q),candidate=fl3i::Qc();
 const auto qb=fgo::utils::getQc(base),qc=fgo::utils::getQc(candidate);
 fl3i::Require((qc-.5*qb).cwiseAbs().maxCoeff()<1e-15,"Qc scale");
 const auto Qb=fgo::utils::calcQ<6>(qb,.1),Qc=fgo::utils::calcQ<6>(qc,.1);
 fl3i::Require((Qc-.5*Qb).cwiseAbs().maxCoeff()<1e-15,"discrete Q scale");
 std::mt19937 rng(140026);std::normal_distribution<double> normal;
 auto random=[&](int n){Vector v(n);for(int j=0;j<n;++j)v[j]=normal(rng);return v;};
 double maxpose=0,maxres=0,maxjac=0,maxcost=0;
 for(int trial=0;trial<100;++trial){
   Values values;
   SourceWnoaState st;st.pose=Pose3::Expmap(.2*random(6));st.velocity=.5*random(3);st.omega=.2*random(3);
   for(size_t i=0;i<3;++i){
     InsertSourceWnoaState(values,i,st);st=PredictSourceWnoa(st,.1);
     st.pose=st.pose.retract(.005*random(6));st.velocity+=.03*random(3);st.omega+=.02*random(3);
   }
   const double tau=.001+.00097*trial;
   const auto b0=WnoaEndpoint::Between(0,1,.1,tau,base).withCache(nullptr);
   const auto c0=WnoaEndpoint::Between(0,1,.1,tau,candidate).withCache(nullptr);
   const auto b1=WnoaEndpoint::Between(1,2,.1,.099-tau*.5,base).withCache(nullptr);
   const auto c1=WnoaEndpoint::Between(1,2,.1,.099-tau*.5,candidate).withCache(nullptr);
   const Pose3 measurement=Pose3::Expmap(.02*random(6));
   Matrix6 M=Matrix6::Identity();for(int i=0;i<6;++i)for(int j=0;j<6;++j)M(i,j)+=.1*normal(rng);
   const Matrix6 C=1e-4*(M*M.transpose());auto noise=noiseModel::Gaussian::Covariance(C);
   WnoaRelativeFactor fb(b0,b1,measurement,noise),fc(c0,c1,measurement,noise);
   std::vector<Matrix> Jb,Jc;auto eb=fb.unwhitenedError(values,Jb),ec=fc.unwhitenedError(values,Jc);
   maxpose=std::max(maxpose,b0.pose(values).localCoordinates(c0.pose(values)).norm());
   maxres=std::max(maxres,(eb-ec).cwiseAbs().maxCoeff());
   fl3i::Require(fb.keys()==fc.keys()&&Jb.size()==Jc.size(),"source keys");
   for(size_t j=0;j<Jb.size();++j)maxjac=std::max(maxjac,(Jb[j]-Jc[j]).cwiseAbs().maxCoeff());
   fl3i::Require(fb.noiseModel()==fc.noiseModel(),"source noise changed");
   fgo::factor::GPWNOAPrior gb(X(0),V(0),W(0),X(1),V(1),W(1),.1,base,true,true);
   fgo::factor::GPWNOAPrior gc(X(0),V(0),W(0),X(1),V(1),W(1),.1,candidate,true,true);
   maxcost=std::max(maxcost,std::abs(gc.error(values)-2*gb.error(values))/std::max(1.,2*gb.error(values)));
 }
 fl3i::Require(maxpose<1e-12&&maxres<1e-12&&maxjac<1e-8&&maxcost<1e-12,"scalar Qc isolation audit failed");
 std::cout<<std::setprecision(17)<<"{\"pass\":true,\"cases\":100,\"qc_multiplier\":0.5,\"pose_max\":"<<maxpose
   <<",\"source_residual_max\":"<<maxres<<",\"source_jacobian_max\":"<<maxjac<<",\"motion_doubled_cost_relative_error\":"<<maxcost<<"}\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
