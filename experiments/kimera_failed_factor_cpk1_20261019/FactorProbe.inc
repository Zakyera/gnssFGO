// Read-only localization on independent factor clones, only after failure.
inline std::string cpk1Family(const NonlinearFactor& f) {
  if(dynamic_cast<const SmartStereoProjectionPoseFactor*>(&f))return "smart_stereo";
  if(dynamic_cast<const fl4l::OriginalChartPrior*>(&f))return "marginal_prior";
  if(dynamic_cast<const ImuFactorT<>*>(&f))return "imu";
  if(dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(&f))return "bias_between";
  if(dynamic_cast<const BetweenFactor<Pose3>*>(&f))return "pose_between";
  return typeid(f).name();
}
inline void cpk1Matrix(std::ostream& out,const std::string& label,const Matrix& m) {
  out<<label<<','<<m.rows()<<','<<m.cols();
  for(Eigen::Index i=0;i<m.rows();++i)for(Eigen::Index j=0;j<m.cols();++j)out<<','<<m(i,j);
  out<<'\n';
}
inline std::pair<double,int> cpk1Cost(const NonlinearFactor& f,const Values& v,bool refresh=false) {
  auto q=f.clone();auto smart=std::dynamic_pointer_cast<SmartStereoProjectionPoseFactor>(q);
  if(refresh&&smart)smart->cpk1ClearCache();
  double e=q->error(v);if(!std::isfinite(e))throw std::runtime_error("CPK1 nonfinite factor error");
  return {e,smart?int(smart->point().status):-1};
}
inline void factorProbe(double stamp,const NonlinearFactorGraph& graph,const Values& theta,
                        const Values& before) {
  const char* path=std::getenv("CPK1_PROBE");if(!path)return;
  if(std::ifstream(path).good())throw std::runtime_error("CPK1 refuse overwrite");
  std::ofstream out(path), snapshot(std::string(path)+".snapshot");
  out<<std::setprecision(17)<<std::unitbuf;snapshot<<std::setprecision(17);
  auto objective=anchored(graph,theta,true);
  const Ordering order(before.keys());auto hg=objective.clone().linearize(before)->hessian(order);
  Matrix H=.5*(hg.first+hg.first.transpose());Vector g=-hg.second;
  if(!H.allFinite()||!g.allFinite()||(H.diagonal().array()<=0).any())
    throw std::runtime_error("CPK1 invalid model");
  Vector scale=H.diagonal().array().sqrt().inverse(),scaled=scale.asDiagonal()*g;
  Vector direction=Vector::Zero(g.size());
  if(scaled.norm()>0)direction=-(scale.asDiagonal()*scaled)/scaled.norm();
  VectorValues d=before.zeroVectors();size_t offset=0;
  for(Key k:order){d.at(k)=direction.segment(offset,d.at(k).size());offset+=d.at(k).size();}
  cpk1Matrix(snapshot,"H",H);cpk1Matrix(snapshot,"g",g);cpk1Matrix(snapshot,"direction",direction);
  out<<"decision,"<<stamp<<','<<cost(graph,before,theta)<<','<<g.dot(direction)<<'\n';
  // fixed, independently declared numerical derivative scales; no state update.
  const std::array<double,3> alphas{.001,.01,.1};
  std::vector<Values> plus,minus;
  for(double a:alphas){auto p=d,m=d;for(auto& kv:p)kv.second*=a;for(auto& kv:m)kv.second*=-a;
    plus.push_back(before.retract(p));minus.push_back(before.retract(m));}
  double sum_cost=0,sum_analytic=0;std::array<double,3> sum_fd{0,0,0};
  for(size_t slot=0;slot<objective.size();++slot) {
    const auto& f=objective[slot];if(!f)continue;
    auto linear=f->clone()->linearize(before);double analytic=0;
    if(linear)for(const auto& kv:linear->gradientAtZero())analytic+=kv.second.dot(d.at(kv.first));
    auto initial=cpk1Cost(*f,before);sum_cost+=initial.first;sum_analytic+=analytic;
    out<<"factor,"<<slot<<','<<cpk1Family(*f)<<','<<f->keys().size()<<','<<initial.first<<','<<analytic<<','<<initial.second;
    for(size_t j=0;j<alphas.size();++j){auto p=cpk1Cost(*f,plus[j]),m=cpk1Cost(*f,minus[j]);
      double fd=(p.first-m.first)/(2*alphas[j]);sum_fd[j]+=fd;
      out<<','<<fd<<','<<p.second<<','<<m.second;}
    auto smart=std::dynamic_pointer_cast<SmartStereoProjectionPoseFactor>(f);
    if(smart){
      auto p=cpk1Cost(*f,plus[1],true),m=cpk1Cost(*f,minus[1],true);
      auto fresh=cpk1Cost(*f,before,true);
      out<<','<<fresh.first<<','<<(p.first-m.first)/(.02)<<','<<fresh.second<<','<<p.second<<','<<m.second;
      snapshot<<"smart,"<<slot<<'\n';smart->cpk1Dump(snapshot);
      for(Key k:f->keys()){
        cpk1Matrix(snapshot,"pose_"+std::to_string(k),before.at<Pose3>(k).matrix());
        cpk1Matrix(snapshot,"dir_"+std::to_string(k),d.at(k));}
    }
    out<<'\n';
  }
  out<<"sums,"<<sum_cost<<','<<sum_analytic;
  for(double fd:sum_fd)out<<','<<fd;out<<'\n';
  out<<"repeat,"<<cost(graph,before,theta)<<"\ndone,1\n";
  if(!out||!snapshot)throw std::runtime_error("CPK1 write failed");
}
