// Read-only queries of frozen WNOA Values/graphs. No optimizer or receiver.
#include "/previous/build/wnoa/SourceWnoaModel.h"
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Eigenvalues>
#include <tbb/global_control.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <map>
using namespace fgo::offline;
using M=gtsam::Matrix; using Vec=gtsam::Vector;
using Graph=gtsam::NonlinearFactorGraph; using Factor=gtsam::NonlinearFactor::shared_ptr;
using Rows=std::vector<std::vector<std::string>>;
void Check(bool b,const char* s){if(!b)throw std::runtime_error(s);}
Rows Read(const std::string& path){
 std::ifstream in(path);Check(bool(in),("missing "+path).c_str());Rows rr;std::string line;std::getline(in,line);
 while(std::getline(in,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty())continue;
  std::stringstream s(line);std::vector<std::string> r;std::string f;while(std::getline(s,f,','))r.push_back(f);rr.push_back(r);}return rr;
}
struct Data{gtsam::Values values;std::vector<int64_t> times;};
Data Load(const std::string& path){Data d;for(auto&r:Read(path)){
 Check(r.size()==15&&std::stoull(r[0])==d.times.size(),"state schema");
 auto v=[&](int k){double a=std::stod(r[k]);Check(std::isfinite(a),"nonfinite state");return a;};
 SourceWnoaState s;s.pose=gtsam::Pose3(gtsam::Rot3::Quaternion(v(8),v(5),v(6),v(7)),gtsam::Point3(v(2),v(3),v(4)));
 s.velocity<<v(9),v(10),v(11);s.omega<<v(12),v(13),v(14);int64_t t=std::stoll(r[1]);
 Check(d.times.empty()||t>d.times.back(),"clock order");InsertSourceWnoaState(d.values,d.times.size(),s);d.times.push_back(t);}return d;
}
gtsam::SharedNoiseModel Qc(){return gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());}
WnoaEndpoint Endpoint(const Data& d,int64_t t){auto it=std::lower_bound(d.times.begin(),d.times.end(),t);Check(it!=d.times.end()&&t>=d.times.front(),"endpoint outside history");size_t j=it-d.times.begin();return *it==t?WnoaEndpoint::Exact(j):WnoaEndpoint::Between(j-1,j,(*it-d.times[j-1])*1e-9,(t-d.times[j-1])*1e-9,Qc());}
gtsam::KeyVector Keys(size_t i){return {X(i),V(i),W(i)};}
M Joint(const gtsam::Marginals& m,const Data& d,const gtsam::KeyVector& keys){auto j=m.jointMarginalCovariance(keys);size_t dim=0;for(auto k:keys)dim+=d.values.at(k).dim();M p(dim,dim);size_t row=0;
 for(auto a:keys){size_t col=0;for(auto b:keys){p.block(row,col,d.values.at(a).dim(),d.values.at(b).dim())=j.at(a,b);col+=d.values.at(b).dim();}row+=d.values.at(a).dim();}
 Check(p.allFinite(),"nonfinite marginal");Check((p-p.transpose()).norm()<1e-8*(1+p.norm()),"marginal symmetry");return .5*(p+p.transpose()).eval();}
Graph Build(const Data& d,const std::string& dir,const std::string& payload,double& costerr){
 std::map<size_t,Factor> sources;for(auto&r:Read(payload)){Check(r.size()==46,"source schema");gtsam::Vector6 mu;gtsam::Matrix6 cov;
 for(int i=0;i<6;++i){mu[i]=std::stod(r[4+i]);for(int j=0;j<6;++j)cov(i,j)=std::stod(r[10+6*i+j]);}
 Check(sources.emplace(std::stoull(r[0]),boost::make_shared<WnoaRelativeFactor>(Endpoint(d,std::stoll(r[2])),Endpoint(d,std::stoll(r[3])),gtsam::Pose3::Expmap(mu),gtsam::noiseModel::Gaussian::Covariance(cov,false))).second,"duplicate slot");}
 auto errors=Read(dir+"/incremental_errors.csv");Graph g;g.resize(std::stoull(errors.back()[0])+1);costerr=0;
 for(auto&r:errors){size_t slot=std::stoull(r[0]);Factor f;
 if(sources.count(slot))f=sources.at(slot);
 else if(r[1]=="wnoa"){size_t i=std::stoull(r[4].substr(1,r[4].find(';')-1)),j=i+1;f=boost::make_shared<fgo::factor::GPWNOAPrior>(X(i),V(i),W(i),X(j),V(j),W(j),(d.times[j]-d.times[i])*1e-9,Qc(),true,true);}
 else if(slot==0)f=boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0),gtsam::Pose3(),gtsam::noiseModel::Isotropic::Sigma(6,1e-5));
 else if(slot==1)f=boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0),gtsam::Vector3::Zero().eval(),gtsam::noiseModel::Isotropic::Sigma(3,10));
 else if(slot==2)f=boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(0),gtsam::Vector3::Zero().eval(),gtsam::noiseModel::Isotropic::Sigma(3,3));
 else throw std::runtime_error("unknown factor");
 std::string keys;for(auto k:f->keys())keys+=gtsam::DefaultKeyFormatter(k)+";";Check(keys==r[4]&&f->dim()==std::stoull(r[2]),"factor structure mismatch");
 double e=std::abs(f->error(d.values)-std::stod(r[3]))/(1+std::abs(std::stod(r[3])));costerr=std::max(e,costerr);Check(e<1e-7,"factor cost mismatch");g[slot]=f;}return g;
}
std::pair<double,double> Probe(const Graph& g,const Data& d,const std::string& path){
 std::ifstream in(path);if(!in)return {-1,-1};std::string line;double ce=0,re=0;size_t count=0;
 while(std::getline(in,line)){if(line.empty())continue;size_t slot=std::stoull(line.substr(0,line.find(':')));auto f=g.at(slot);Check(bool(f),"empty probe slot");std::ostringstream h;h<<slot<<':'<<typeid(*f).name()<<':';for(auto k:f->keys())h<<k<<',';Check(h.str()==line,"factor probe keys/type");
 auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(nf->noiseModel());M actual=noise->covariance(),expected(actual.rows(),actual.cols());for(int i=0;i<expected.rows();++i)for(int j=0;j<expected.cols();++j)in>>expected(i,j);Vec e(f->dim());for(int i=0;i<e.size();++i)in>>e[i];Check(bool(in),"truncated probe");
 ce=std::max(ce,(actual-expected).cwiseAbs().maxCoeff());re=std::max(re,(nf->unwhitenedError(d.values)-e).cwiseAbs().maxCoeff());std::getline(in,line);++count;}
 size_t n=0;for(auto f:g)if(f)++n;Check(count==n&&ce<1e-10&&re<1e-9,"payload mismatch");return {ce,re};
}
std::pair<M,M> Transition(const Data& d,size_t i){fgo::factor::GPWNOAPrior f(X(i),V(i),W(i),X(i+1),V(i+1),W(i+1),(d.times[i+1]-d.times[i])*1e-9,Qc(),true,true);
 std::vector<M> h(6);f.unwhitenedError(d.values,h);M hi(12,12),hj(12,12);hi<<h[0],h[1],h[2];hj<<h[3],h[4],h[5];auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(f.noiseModel());M inv=hj.fullPivLu().inverse();return {(-inv*hi).eval(),(inv*noise->covariance()*inv.transpose()).eval()};}
void MatrixOut(std::ofstream&o,const std::string&pre,const M&p){for(int i=0;i<p.rows();++i)for(int j=0;j<p.cols();++j)o<<pre<<','<<i<<','<<j<<','<<p(i,j)<<'\n';}
void Stats(std::ofstream&o,const std::string&pre,const Data&d,const Data*next,size_t i,const M&p){
 Check(Eigen::SelfAdjointEigenSolver<M>(p).eigenvalues().minCoeff()>0,"nonpositive marginal");auto a=ReadSourceWnoaState(d.values,i);M world=M::Identity(12,12);world.block<3,3>(3,3)=a.pose.rotation().matrix();M pw=world*p*world.transpose();Vec delta=Vec::Zero(12);
 if(next){auto b=ReadSourceWnoaState(next->values,i);delta.head<3>()=gtsam::Rot3::Logmap(a.pose.rotation().between(b.pose.rotation()));delta.segment<3>(3)=b.pose.translation()-a.pose.translation();delta.segment<3>(6)=b.velocity-a.velocity;delta.tail<3>()=b.omega-a.omega;}
 std::vector<std::string> names={"rotation","position","velocity","omega"};
 for(int j=0;j<4;++j){M b=pw.block<3,3>(3*j,3*j);auto eig=Eigen::SelfAdjointEigenSolver<M>(b).eigenvalues().eval();Vec v=delta.segment<3>(3*j);double n=v.norm();o<<pre<<','<<i<<','<<d.times[i]<<','<<names[j]<<','<<sqrt(b.trace())<<','<<sqrt(eig.minCoeff())<<','<<sqrt(eig.maxCoeff())<<',';
 if(next)o<<n<<','<<sqrt(std::max(0.,v.dot(b.ldlt().solve(v))))<<','<<(n>0?sqrt(std::max(0.,v.dot(b*v)))/n:0.);else o<<",,";o<<'\n';}
}
void Synthetic(){for(bool moving:{false,true}){Data d;SourceWnoaState s;if(moving){s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.2,-.1,.3),gtsam::Point3(1,2,3));s.velocity<<.4,-.2,.1;s.omega<<.2,.1,-.3;}
 d.times={1000000000,1100000000};InsertSourceWnoaState(d.values,0,s);InsertSourceWnoaState(d.values,1,PredictSourceWnoa(s,.1));auto[f,q]=Transition(d,0);Check((q-q.transpose()).norm()<1e-10&&Eigen::SelfAdjointEigenSolver<M>(q).eigenvalues().minCoeff()>0,"process covariance");
 if(!moving)Check(std::abs(q.block<3,3>(3,3).trace()-.045*.001/3)<1e-10,"stationary dt cubed scaling");
 Graph g;g.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0),s.pose,gtsam::noiseModel::Isotropic::Sigma(6,1));g.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0),s.velocity,gtsam::noiseModel::Isotropic::Sigma(3,1));g.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(0),s.omega,gtsam::noiseModel::Isotropic::Sigma(3,1));g.emplace_shared<fgo::factor::GPWNOAPrior>(X(0),V(0),W(0),X(1),V(1),W(1),.1,Qc(),true,true);
 gtsam::Marginals m(g,d.values,gtsam::Marginals::QR);Check((Joint(m,d,Keys(1))-(f*f.transpose()+q)).norm()<1e-7,"dense covariance propagation");
 for(double tau:{0.,.1}){auto ep=WnoaEndpoint::Between(0,1,.1,tau,Qc());Check(gtsam::Pose3::Logmap(ep.pose(d.values).between(d.values.at<gtsam::Pose3>(X(tau==0?0:1)))).norm()<1e-12,"endpoint limit");}
 if(!moving){s.velocity<<.4,-.2,.1;Data x;x.times=d.times;InsertSourceWnoaState(x.values,0,s);InsertSourceWnoaState(x.values,1,PredictSourceWnoa(s,.1));for(double t:{.023,.071})Check((WnoaEndpoint::Between(0,1,.1,t,Qc()).pose(x.values).translation()-s.velocity*t).norm()<1e-12,"fractional interpolation");}}
 std::cout<<"PASS stationary/moving full covariance, cross terms, endpoints, fractional interpolation\n";}
void Dense(const Data&d,const std::string&prefix,std::ofstream&out,std::ofstream&summary,bool all){
 size_t n=d.times.size()-1,start=all?0:(n>5?n-5:0);
 for(size_t i=start;i<n;++i){auto a=d.values.at<gtsam::Pose3>(X(i)),b=d.values.at<gtsam::Pose3>(X(i+1));auto chord=(b.translation()-a.translation()).eval();double len=chord.norm(),arc=0,maxperp=0,minproj=0,maxproj=len,maxspeed=0,maxomega=0;gtsam::Pose3 previous=a;
 for(int j=0;j<=10;++j){int64_t t=d.times[i]+j*(d.times[i+1]-d.times[i])/10;auto p=Endpoint(d,t).pose(d.values);auto q=p.rotation().toQuaternion();out<<prefix<<','<<i<<','<<t<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
 if(j){double dt=(d.times[i+1]-d.times[i])*1e-10;double distance=(p.translation()-previous.translation()).norm();arc+=distance;maxspeed=std::max(maxspeed,distance/dt);maxomega=std::max(maxomega,gtsam::Rot3::Logmap(previous.rotation().between(p.rotation())).norm()/dt);}
 if(len>1e-9){auto offset=(p.translation()-a.translation()).eval();double proj=offset.dot(chord)/len;minproj=std::min(minproj,proj);maxproj=std::max(maxproj,proj);maxperp=std::max(maxperp,(offset-proj*chord/len).norm());}previous=p;}
 summary<<prefix<<','<<i<<','<<d.times[i]<<','<<len<<','<<arc<<','<<arc-len<<','<<maxperp<<','<<std::max(-minproj,maxproj-len)<<','<<maxspeed<<','<<maxomega<<'\n';}
}
int main(int argc,char**argv){try{tbb::global_control threads(tbb::global_control::max_allowed_parallelism,1);Synthetic();if(argc==2&&std::string(argv[1])=="--test")return 0;Check(argc==4,"usage: audit covariance|means output request.csv");
 std::filesystem::path out=argv[2];Check(!std::filesystem::exists(out),"refuse overwrite");std::filesystem::create_directories(out);std::string action=argv[1];
 if(action=="means"){
 std::ofstream dense(out/"dense_poses.csv"),intervals(out/"intervals.csv"),front(out/"frontier_changes.csv");for(auto*p:{&dense,&intervals,&front})*p<<std::setprecision(17);
 dense<<"mode,snapshot,interval,timestamp_ns,px,py,pz,qx,qy,qz,qw\n";intervals<<"mode,snapshot,interval,timestamp_ns,chord_m,arc_m,excess_m,max_perpendicular_m,overshoot_m,max_fd_speed_mps,max_fd_omega_radps\n";
 front<<"mode,snapshot,frontier_ns,next_position_change_m,next_orientation_change_rad\n";
 auto rr=Read(argv[3]);for(size_t r=0;r<rr.size();++r){auto row=rr[r];auto mode=row[0],label=row[1],prefix=mode+","+label;std::string dir="/previous/"+mode+"_wnoa/snapshot_"+label;auto d=Load(dir+"/incremental_states.csv");Dense(d,prefix,dense,intervals,label=="final");int64_t F=std::stoll(row[3]);
 if(F&&label!="final"){Check(r+1<rr.size()&&rr[r+1][0]==mode,"next row");auto next=Load("/previous/"+mode+"_wnoa/snapshot_"+rr[r+1][1]+"/incremental_states.csv");auto a=Endpoint(d,F).pose(d.values),b=Endpoint(next,F).pose(next.values);front<<prefix<<','<<F<<','<<(b.translation()-a.translation()).norm()<<','<<gtsam::Rot3::Logmap(a.rotation().between(b.rotation())).norm()<<'\n';}
 if(r%100==0)std::cout<<prefix<<" dense queries\n"<<std::flush;}return 0;}
 Check(action=="covariance","unknown action");
 std::ofstream stats(out/"marginal_summary.csv"),cov(out/"marginal_blocks.csv"),checks(out/"graph_checks.csv"),decomp(out/"prediction_decomposition.csv"),endpoint(out/"endpoint_uncertainty.csv");for(auto*p:{&stats,&cov,&checks,&decomp,&endpoint})*p<<std::setprecision(17);
 stats<<"mode,snapshot,zone,state_index,timestamp_ns,component,sqrt_trace,std_min_axis,std_max_axis,next_change_norm,next_change_mahalanobis,next_change_direction_sigma\n";
 cov<<"mode,snapshot,zone,row,col,value\n";checks<<"mode,snapshot,factors,variables,cost_relative_error,probe_cov_error,probe_residual_error,boundary,source_free_duration_s,propagation_relative_error,min_eigenvalue\n";
 decomp<<"mode,snapshot,component,current_trace,inherited_trace,process_trace\n";endpoint<<"mode,snapshot,timestamp_ns,mean_position_sqrt_trace,mean_rotation_sqrt_trace\n";
 for(auto&r:Read(argv[3])){auto mode=r[0],label=r[1],prefix=mode+","+label;std::string dir="/previous/"+mode+"_wnoa/snapshot_"+label;auto d=Load(dir+"/incremental_states.csv");double costerr;auto g=Build(d,dir,"/audit/"+mode+"_"+label+"_factors.csv",costerr);auto[ce,re]=Probe(g,d,dir+"/factor_payload_probe.txt");gtsam::Marginals marg(g,d.values,gtsam::Marginals::QR);
 size_t c=std::stoul(r[3]),b=std::stoul(r[4]),n=std::stoul(r[5]);Check(n==d.times.size()-1,"current index");for(auto&s:Read("/audit/"+mode+"_"+label+"_factors.csv"))for(auto key:g.at(std::stoull(s[0]))->keys())Check(gtsam::Symbol(key).index()<=b,"source touches prediction tail");
 Data next;Data*np=nullptr;if(label!="final"){auto nl=std::stoul(label)==1192?"final":std::to_string(std::stoul(label)+1);next=Load("/previous/"+mode+"_wnoa/snapshot_"+nl+"/incremental_states.csv");np=&next;}
 double mineig=1e100;for(auto z:std::vector<std::pair<std::string,size_t>>{{"covered",c},{"boundary",b},{"current",n}}){auto p=Joint(marg,d,Keys(z.second));mineig=std::min(mineig,Eigen::SelfAdjointEigenSolver<M>(p).eigenvalues().minCoeff());MatrixOut(cov,prefix+","+z.first,p);Stats(stats,prefix+","+z.first,d,np,z.second,p);}
 auto ep=Endpoint(d,std::stoll(r[2]));auto pjoint=Joint(marg,d,ep.keys());std::vector<M> hs;ep.pose(d.values,&hs);M h(6,pjoint.rows());int col=0;for(auto&a:hs){h.middleCols(col,a.cols())=a;col+=a.cols();}M pe=h*pjoint*h.transpose();MatrixOut(cov,prefix+",endpoint_support_joint",pjoint);MatrixOut(cov,prefix+",endpoint_mean",pe);endpoint<<prefix<<','<<r[2]<<','<<sqrt(pe.bottomRightCorner<3,3>().trace())<<','<<sqrt(pe.topLeftCorner<3,3>().trace())<<'\n';
 M pb=Joint(marg,d,Keys(b)),pn=Joint(marg,d,Keys(n)),f=M::Identity(12,12),q=M::Zero(12,12);for(size_t i=b;i<n;++i){auto[fi,qi]=Transition(d,i);q=(fi*q*fi.transpose()+qi).eval();f=(fi*f).eval();}M inherited=f*pb*f.transpose(),pred=inherited+q;double error=(pred-pn).norm()/pn.norm();Check(error<1e-5,"marginal propagation mismatch");MatrixOut(cov,prefix+",inherited",inherited);MatrixOut(cov,prefix+",process",q);
 size_t count=0;for(auto a:g)if(a)++count;checks<<prefix<<','<<count<<','<<d.values.size()<<','<<costerr<<','<<ce<<','<<re<<','<<b<<','<<(d.times[n]-d.times[b])*1e-9<<','<<error<<','<<mineig<<'\n';
 std::vector<std::string> names={"rotation","position","velocity","omega"};for(int j=0;j<4;++j)decomp<<prefix<<','<<names[j]<<','<<pn.block<3,3>(3*j,3*j).trace()<<','<<inherited.block<3,3>(3*j,3*j).trace()<<','<<q.block<3,3>(3*j,3*j).trace()<<'\n';std::cout<<prefix<<" covariance checked "<<error<<'\n'<<std::flush;}
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
