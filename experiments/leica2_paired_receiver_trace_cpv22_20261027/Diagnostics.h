// CP-V22 observation only. Included inside fl3i; no writes to live solver values.
struct DiagnosticFiles {
  std::ofstream jac,endpoints,graphs,failures;
  explicit DiagnosticFiles(const fs::path& root) {
    auto open=[&](std::ofstream& f,const char* name,const char* header) {
      Require(!fs::exists(root/name),"refuse numerical audit overwrite");
      f.open(root/name);f.exceptions(std::ios::failbit|std::ios::badbit);
      f<<std::setprecision(17)<<header<<'\n';
    };
    open(jac,"jacobians.csv","snapshot,event,category,key,epsilon,max_absolute,max_scaled,residual_with_jacobian_difference,pass");
    open(endpoints,"endpoint_checks.csv","snapshot,event,source,start_left,start_right,start_tau,end_left,end_right,end_tau,reconstructed_residual_difference,boundary_pose_difference");
    open(graphs,"linearizations.csv","snapshot,event,newest,rows,cols,cost,rows_file,columns_file");
    open(failures,"numerical_failures.csv","snapshot,event,category,key,epsilon,row,col,implemented,numerical");
  }
};
inline DiagnosticFiles& AuditFiles(const fs::path& root) {static DiagnosticFiles files(root);return files;}
inline Values PerturbAudit(const Values& input,Key key,size_t column,double delta) {
  Values copy(input);const char kind=Symbol(key).chr();
  if(kind=='x') {Vector6 d=Vector6::Zero();d(column)=delta;copy.update(key,input.at<Pose3>(key).retract(d));}
  else {Require(kind=='v'||kind=='w',"unexpected audit variable");Vector3 d=input.at<Vector3>(key);d(column)+=delta;copy.update(key,d);}
  return copy;
}
inline void AuditJacobian(const State& s,size_t serial,const std::string& category,
                          const boost::shared_ptr<NoiseModelFactor>& factor,
                          const Values& values,const fs::path& root) {
  auto& files=AuditFiles(root);std::vector<Matrix> J;
  const Vector withJ=factor->unwhitenedError(values,J);
  const Vector plain=factor->unwhitenedError(values);
  const double residualDiff=(withJ-plain).cwiseAbs().maxCoeff();
  Require(J.size()==factor->keys().size(),"audit Jacobian key count");
  for(double epsilon:{1e-5,1e-6}) {
    for(size_t k=0;k<J.size();++k) {
      const Key key=factor->keys()[k];Matrix numerical(plain.size(),values.at(key).dim());
      for(size_t c=0;c<values.at(key).dim();++c) {
        const Vector plus=factor->unwhitenedError(PerturbAudit(values,key,c,epsilon));
        const Vector minus=factor->unwhitenedError(PerturbAudit(values,key,c,-epsilon));
        numerical.col(c)=(plus-minus)/(2*epsilon);
      }
      const Matrix abs=(J[k]-numerical).cwiseAbs();
      const double scaled=(abs.array()/(1.+numerical.array().abs())).maxCoeff();
      const bool pass=J[k].allFinite()&&numerical.allFinite()&&scaled<=2e-5&&residualDiff<=1e-12;
      files.jac<<serial<<','<<s.event<<','<<category<<','<<Symbol(key).chr()<<Symbol(key).index()
        <<','<<epsilon<<','<<abs.maxCoeff()<<','<<scaled<<','<<residualDiff<<','<<pass<<'\n';
      if(!pass)for(int r=0;r<J[k].rows();++r)for(int c=0;c<J[k].cols();++c)
        files.failures<<serial<<','<<s.event<<','<<category<<','<<Symbol(key).chr()<<Symbol(key).index()
          <<','<<epsilon<<','<<r<<','<<c<<','<<J[k](r,c)<<','<<numerical(r,c)<<'\n';
    }
  }
  files.jac.flush();files.failures.flush();
}
inline void AuditSource(const State& s,size_t serial,const Belief& b,
                        const NonlinearFactor::shared_ptr& incoming,const Values& values,const fs::path& root) {
  auto factor=boost::dynamic_pointer_cast<NoiseModelFactor>(incoming->clone());Require(bool(factor),"source audit noise type");
  AuditJacobian(s,serial,"source_"+b.source,factor,values,root);
  auto endpoint=[&](int64_t ns) {
    Require(ns>=s.origin,"audit source precedes origin");
    const int64_t delta=ns-s.origin;const size_t left=delta/Period;
    const double tau=(delta%Period)*1e-9;
    if(delta%Period==0)return std::make_pair(WnoaEndpoint::Exact(left),tau);
    return std::make_pair(WnoaEndpoint::Between(left,left+1,.1,tau,Qc(),ns).withCache(nullptr),tau);
  };
  const auto a=endpoint(b.start),z=endpoint(b.end);
  Require(a.first.right<=s.newest&&z.first.right<=s.newest,"audit endpoint beyond supports");
  const auto pi=a.first.pose(values),pj=z.first.pose(values);
  const Vector independently=Pose3::Logmap(b.mean.inverse().compose(pi.inverse().compose(pj)));
  const double residualDiff=(independently-factor->unwhitenedError(values)).cwiseAbs().maxCoeff();
  double boundary=0.;
  for(const auto& ep:{a.first,z.first})if(!ep.exact) {
    for(double tau:{0.,.1}) {
      const auto at=WnoaEndpoint::Between(ep.left,ep.right,.1,tau,Qc()).withCache(nullptr).pose(values);
      const auto exact=values.at<Pose3>(X(tau==0.?ep.left:ep.right));
      boundary=std::max(boundary,(at.matrix()-exact.matrix()).cwiseAbs().maxCoeff());
    }
  }
  auto& f=AuditFiles(root).endpoints;
  f<<serial<<','<<s.event<<','<<b.source<<','<<a.first.left<<','<<a.first.right<<','<<a.second
    <<','<<z.first.left<<','<<z.first.right<<','<<z.second<<','<<residualDiff<<','<<boundary<<'\n';f.flush();
}
inline void AuditGraph(const State& s,size_t serial,const Values& values,const fs::path& root) {
  NonlinearFactorGraph diagnostic;
  const auto& graph=s.solver.getFactors();
  for(size_t i=0;i<graph.size();++i) {
    if(!graph[i])continue;
    if(const auto* f=dynamic_cast<const LinearContainerFactor*>(graph[i].get()))
      diagnostic.push_back(boost::make_shared<fl2::ChartMarginal>(f->factor(),s.solver.marginals.at(i).anchor));
    else diagnostic.push_back(graph[i]->clone());
  }
  // Check the actual newest motion factor at every selected publication.
  for(const auto& f:diagnostic)if(dynamic_cast<const fgo::factor::GPWNOAPrior*>(f.get())&&
      std::find(f->keys().begin(),f->keys().end(),X(s.newest))!=f->keys().end())
    AuditJacobian(s,serial,"WNOA",boost::dynamic_pointer_cast<NoiseModelFactor>(f),values,root);
  Ordering order;for(const auto& entry:values)order.push_back(entry.key);
  const auto gaussian=diagnostic.linearize(values);
  const auto ab=gaussian->jacobian(order);
  const Matrix augmented=(Matrix(ab.first.rows(),ab.first.cols()+1)<<ab.first,ab.second).finished();
  Require(augmented.allFinite(),"nonfinite diagnostic linearization");
  const std::string name="linearization_"+std::to_string(s.newest)+".bin";
  const std::string columns="linearization_"+std::to_string(s.newest)+"_columns.csv";
  Require(!fs::exists(root/name)&&!fs::exists(root/columns),"refuse graph snapshot overwrite");
  std::ofstream out(root/name,std::ios::binary);out.exceptions(std::ios::failbit|std::ios::badbit);
  // Explicit row-major wire order, independent of Eigen storage convention.
  for(int r=0;r<augmented.rows();++r)for(int c=0;c<augmented.cols();++c){double x=augmented(r,c);out.write(reinterpret_cast<const char*>(&x),sizeof(x));}
  out.close();std::ofstream keys(root/columns);keys<<"key,dimension\n";
  for(Key key:order)keys<<Symbol(key).chr()<<Symbol(key).index()<<','<<values.at(key).dim()<<'\n';keys.close();
  auto& file=AuditFiles(root).graphs;
  file<<serial<<','<<s.event<<','<<s.newest<<','<<ab.first.rows()<<','<<ab.first.cols()<<','<<diagnostic.error(values)
      <<','<<name<<','<<columns<<'\n';file.flush();
}
