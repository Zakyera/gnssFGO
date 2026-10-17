// Called only AFTER the unchanged decision has failed. All factor evaluations
// and linearizations below use clones; no candidate is committed.
inline void stoppingProbe(double stamp,const NonlinearFactorGraph& graph,
    const Values& theta,const Values& before,const Values& full,
    const Values& last,const Decision& decision,
    const std::vector<std::array<double,4>>& trials) {
  const char* path=std::getenv("FL4U_PROBE");if(!path)return;
  if(std::ifstream(path).good())throw std::runtime_error("FL4U refuse probe overwrite");
  std::ofstream out(path);out<<std::setprecision(17)<<std::unitbuf;
  if(!out)throw std::runtime_error("FL4U probe open failed");
  const double f0=cost(graph,before,theta);
  out<<"decision,"<<stamp<<','<<decision.before<<','<<decision.full<<','
     <<decision.after<<','<<decision.iterations<<','<<1e-10*(1+std::abs(f0))<<'\n';
  for(const auto& t:trials)out<<"trial,"<<t[0]<<','<<t[1]<<','<<t[2]<<','<<t[3]<<'\n';
  out<<"last,"<<cost(graph,last,theta)<<','<<before.localCoordinates(last).norm()<<'\n';
  auto objective=anchored(graph,theta,true);
  const Ordering ordering(before.keys());
  auto linear=objective.clone().linearize(before);
  const auto hg=linear->hessian(ordering);
  const Matrix H=.5*(hg.first+hg.first.transpose());
  const Vector g=-hg.second;
  Vector diag=H.diagonal();
  out<<"finite,"<<H.allFinite()<<','<<g.allFinite()<<'\n';
  if(!H.allFinite()||!g.allFinite()||(diag.array()<=0).any())
    throw std::runtime_error("FL4U invalid/zero diagonal for scaling");
  Vector scale=diag.array().sqrt().inverse();
  const Vector scaled=scale.asDiagonal()*g;
  const Matrix C=scale.asDiagonal()*H*scale.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Matrix> eig(C);
  out<<"model,"<<g.size()<<','<<g.norm()<<','<<scaled.norm()<<','
     <<scaled.lpNorm<Eigen::Infinity>()<<','<<eig.eigenvalues().minCoeff()<<','
     <<eig.eigenvalues().maxCoeff()<<'\n';
  if(scaled.norm()>0) {
    const Vector direction=-(scale.asDiagonal()*scaled)/scaled.norm();
    VectorValues d=before.zeroVectors();size_t offset=0;
    for(Key key:ordering){d.at(key)=direction.segment(offset,d.at(key).size());offset+=d.at(key).size();}
    out<<"direction,"<<g.dot(direction)<<','<<direction.norm()<<'\n';
    for(double alpha:{1e-4,1e-3,1e-2,.1,1.}) {
      auto plus=d,minus=d;
      for(auto& kv:plus)kv.second*=alpha;
      for(auto& kv:minus)kv.second*=-alpha;
      const double fp=cost(graph,before.retract(plus),theta);
      const double fm=cost(graph,before.retract(minus),theta);
      out<<"probe,"<<alpha<<','<<fp<<','<<fm<<','<<(fp-fm)/(2*alpha)<<','<<fp-f0<<'\n';
    }
  }
  out<<"repeat_cost,"<<cost(graph,before,theta)<<','<<cost(graph,full,theta)<<'\n';
  out<<"done,1\n";
}
